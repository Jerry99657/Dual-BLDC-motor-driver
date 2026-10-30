#include "compact_text.h"
#include "MT6701.h"
#include "spi_port.h"
#include "kk_oled_driver.h"
#include "motor_params.h"
#include <stdint.h>

uint8_t MT6701_Data[3] = {0};
volatile int Angle_Raw = 0;
volatile float Angle = 0.0f;

static volatile uint8_t mt6701_continuous = 0U;
static volatile uint8_t mt6701_transfer_active = 0U;
enum { MT_IDLE, MT_DMA, MT_PUBLISH, MT_RECOVER };
static volatile uint8_t mt6701_transfer_age;
static volatile uint32_t mt6701_transfer_at;
static volatile uint8_t mt6701_sample_valid = 0U;
static volatile uint8_t mt6701_status = 0U;
static volatile uint16_t mt6701_sample_age_cycles = UINT16_MAX;
static volatile uint32_t mt6701_sample_counter = 0U;
static volatile uint32_t mt6701_sample_at = 0U;
static MT6701Diagnostics first_diag;
typedef struct { float angle; uint32_t cycles; } TimedAngle;
static TimedAngle timed_angle[2][2];
static volatile uint8_t timed_index[2],timed_valid[2];
static uint32_t timed_start[2];
/* Publish a complete pair before changing the active index. The ADC ISR can
 * preempt a DMA publisher; it must see the old complete pair in that case. */
static void publish_timed(unsigned axis,float angle)
{
    unsigned index=timed_index[axis]^1U;
    timed_angle[axis][index]=(TimedAngle){angle,timed_start[axis]};
    __DMB();timed_index[axis]=(uint8_t)index;timed_valid[axis]=1;
}
bool MT6701_GetTimedAngle(unsigned axis,float *angle,uint32_t *cycles)
{
    if (axis>1 || !timed_valid[axis]) return false;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    TimedAngle sample=timed_angle[axis][timed_index[axis]];
    __set_PRIMASK(mask);*angle=sample.angle;*cycles=sample.cycles;return true;
}

/* 10 kHz 调度下允许连续 3 个控制周期没有新样本；超过后有感 FOC 撤掉输出。 */
#define MT6701_MAX_SAMPLE_AGE_CYCLES 3U

/* MT6701 SSI 帧：14 位角度 + 4 位状态 + 6 位 CRC。
 * CRC 覆盖前 18 位，生成多项式 x^6 + x + 1（去掉最高项后为 0x03），MSB 先入。 */
static uint8_t MT6701_CalculateCrc6(uint32_t data18)
{
    /* x^6 = x+1 modulo x^6+x+1. Fold the full 18-bit word three
     * times instead of storing a lookup table or iterating over 18 bits. */
    uint32_t crc=data18^(data18<<1);
    for(unsigned n=0;n<3;++n) {
        uint32_t high=crc>>6;
        crc=(crc&63U)^high^(high<<1);
    }
    return (uint8_t)crc;
}
#ifdef MOTOR_ENCODER_HOST_HOOKS
uint8_t EncoderHost_Crc(uint32_t data18) { return MT6701_CalculateCrc6(data18); }
#endif

static MT6701Error MT6701_Decode(const uint8_t data[3], uint16_t *angle, uint8_t *status)
{
    uint32_t frame = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *angle = (uint16_t)(frame >> 10);
    *status = (uint8_t)((frame >> 6) & 0x0FU);
    /* Zero angle / normal status has CRC zero too. It is valid SSI data. */
    if (frame == 0xFFFFFFU) return MT6701_ERROR_ONES;
    if ((frame & 0x3FU) != MT6701_CalculateCrc6(frame >> 6)) return MT6701_ERROR_CRC;
    if (*status & 0x08U) return MT6701_ERROR_TRACK;
    if (*status & 0x03U) return MT6701_ERROR_FIELD;
    return MT6701_ERROR_NONE;
}

static bool zero_jump(unsigned axis,uint16_t previous)
{
    if (!timed_valid[axis]) return false;
    uint32_t elapsed=timed_start[axis]-timed_angle[axis][timed_index[axis]].cycles;
    /* After a long gap admit a new origin, never extrapolate indefinitely.
     * Fixed 170 MHz is checked at every build. The physical speed bound is
     * independent of the command (the Follow master can be moved by hand). */
    if (elapsed>850000U) return false; /* 5 ms, including DWT wrap. */
    unsigned distance=previous>8192U ? 16384U-previous : previous;
    float bound=8.0f+(float)elapsed*(16384.0f/(170000000.0f*2.0f*PI))*
        MotorParams_GetAxis((MotorAxis)axis)->max_speed_mech;
    return (float)distance>bound;
}

static void MT6701_ObserveZero(MT6701Diagnostics *diag, MT6701Error error,
                               uint16_t raw, uint16_t previous, uint16_t age, uint32_t sampled_at)
{
    if (error==MT6701_ERROR_FRAME) {
        if (!diag->zero_count || diag->zero_after!=UINT16_MAX) {
            diag->zero_before=previous;
            diag->zero_after=UINT16_MAX;
            diag->zero_started_at=HAL_GetTick();
            /* Optional polling or a stopped scheduler can leave cycle age
             * frozen despite an expired wall-clock sample. Classify as GAP. */
            diag->zero_age=age<=MT6701_MAX_SAMPLE_AGE_CYCLES && HAL_GetTick()-sampled_at>5U ?
                MT6701_MAX_SAMPLE_AGE_CYCLES+1U : age;
            diag->zero_count=0U;
        }
        if (diag->zero_count<UINT16_MAX) ++diag->zero_count;
    } else if (error==MT6701_ERROR_NONE && diag->zero_count && diag->zero_after==UINT16_MAX) {
        diag->zero_ended_at=HAL_GetTick();
        diag->zero_after=raw;
    }
}

__attribute__((noinline,noclone)) static void record_error(MT6701Diagnostics *diag,MT6701Error error,volatile uint32_t *total)
{
    ++*total;
    diag->last_error = error;
    switch (error) {
    case MT6701_ERROR_CRC: ++diag->crc_errors; break;
    case MT6701_ERROR_FIELD: ++diag->field_errors; break;
    case MT6701_ERROR_TRACK: ++diag->track_errors; break;
    case MT6701_ERROR_SPI:
    case MT6701_ERROR_DMA:
    case MT6701_ERROR_TIMEOUT: ++diag->spi_errors; break;
    default: ++diag->frame_errors; break;
    }
}

static void MT6701_Fault(MT6701Error error)
{
    if (!mt6701_continuous || mt6701_transfer_active==MT_RECOVER) return;
    /* Hold ownership through main-loop cleanup. DMA callbacks and the ADC
     * scheduler must not publish/reuse this buffer during abort/reset. */
    mt6701_transfer_active=MT_RECOVER;
    mt6701_sample_valid=0U;
    record_error(&first_diag,error,&first_diag.errors);
}

static uint8_t MT6701_RequestSample(void)
{
    if (!mt6701_continuous || mt6701_transfer_active) {
        return 0U;
    }

    mt6701_transfer_active = MT_DMA;
    mt6701_transfer_age=0U;
    mt6701_transfer_at=HAL_GetTick();
    CS_Enable;
    timed_start[0]=DWT->CYCCNT;
    HAL_StatusTypeDef status = SpiPort_ExchangeDMA(
        &hspi1, MT6701_Data, MT6701_Data, 3);
    if (status != HAL_OK) {
        MT6701_Fault(MT6701_ERROR_SPI);
        return 0U;
    }
    return 1U;
}

static void second_complete(bool error);

/* SPI DMA 收发完成回调 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI2) { second_complete(false); return; }
    if (hspi->Instance == SPI1) {
        if (!mt6701_continuous || mt6701_transfer_active!=MT_DMA) return;
        mt6701_transfer_active=MT_PUBLISH;
        CS_Disable;
        /* Keep the buffer owned until decoding/publishing has finished:
         * the higher-priority ADC ISR may request the next DMA sample. */

        uint16_t raw_angle;
        uint8_t status;
        MT6701Error error = MT6701_Decode(MT6701_Data, &raw_angle, &status);
        bool zero=!(MT6701_Data[0]|MT6701_Data[1]|MT6701_Data[2]);
        if (zero && zero_jump(0,(uint16_t)Angle_Raw)) error=MT6701_ERROR_FRAME;
        uint32_t now=HAL_GetTick();
        float angle=2.0f*PI-2.0f*PI*(float)raw_angle/16384.0f;
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        mt6701_status = status;
        MT6701_ObserveZero(&first_diag,zero ? MT6701_ERROR_FRAME : error,raw_angle,(uint16_t)Angle_Raw,
                           mt6701_sample_age_cycles,mt6701_sample_at);

        /* Mg[1:0] 非零表示磁场过强/过弱，Mg[3] 表示失跟踪；
         * Mg[2] 是按压状态，不影响转子角度。合法零码也必须发布。 */
        if (error != MT6701_ERROR_NONE) {
            record_error(&first_diag,error,&first_diag.errors);
            mt6701_transfer_active = 0U;
            __set_PRIMASK(mask); return;
        }

        Angle_Raw = (int)raw_angle;
        Angle = angle;
        publish_timed(0,Angle);

        mt6701_sample_age_cycles = 0U;
        mt6701_sample_at = now;
        mt6701_sample_valid = 1U;
        mt6701_sample_counter++;
        mt6701_transfer_active = 0U;
        __set_PRIMASK(mask);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        if (mt6701_transfer_active==MT_DMA) MT6701_Fault(
            SpiPort_GetError(hspi)&HAL_SPI_ERROR_DMA ? MT6701_ERROR_DMA : MT6701_ERROR_SPI);
    } else if (hspi->Instance == SPI2) {
        second_complete(true);
    } else if (hspi->Instance == SPI3) {
        OLED_DriverHandleSpiError();
    }
}

void MT6701_StartContinuous(void)
{
    if (mt6701_continuous) return;
    mt6701_continuous = 1U;
    mt6701_transfer_active = 0U;
    mt6701_sample_valid = 0U;
    mt6701_sample_age_cycles = UINT16_MAX;
    MT6701_RequestSample();
}

void MT6701_StopContinuous(void)
{
    mt6701_continuous = 0U;
    SpiPort_DMAStop(&hspi1);
    CS_Disable;
    mt6701_transfer_active = 0U;
    mt6701_sample_valid = 0U;
    mt6701_sample_age_cycles = UINT16_MAX;
}

void MT6701_MainService(void)
{
    SpiPort_Service(&hspi1);
    /* Wall-clock fallback also covers a stopped ADC scheduler. The short
     * critical section prevents a completion/new frame racing this decision. */
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (mt6701_transfer_active==MT_DMA && HAL_GetTick()-mt6701_transfer_at>1U)
        MT6701_Fault(MT6701_ERROR_TIMEOUT);
    __set_PRIMASK(mask);
    if (mt6701_transfer_active!=MT_RECOVER) return;
    /* All potentially blocking work belongs to main; interrupts stay enabled.
     * Abort can fail with BSY stuck. Quiesce both DMA channels, then reset only
     * SPI1 and reapply its existing Init settings before releasing ownership. */
    (void)SpiPort_Abort(&hspi1);
    (void)SpiPort_DMAStop(&hspi1);
    CS_Disable;
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();
    if (HAL_SPI_Init(&hspi1)==HAL_OK) mt6701_transfer_active=MT_IDLE;
}

uint8_t MT6701_IsContinuous(void)
{
    return mt6701_continuous;
}

void MT6701_Service(void)
{
    SpiPort_Service(&hspi1);
    if (!mt6701_continuous) return;

    if (mt6701_sample_age_cycles < UINT16_MAX) {
        mt6701_sample_age_cycles++;
        if (mt6701_sample_age_cycles > first_diag.max_age_cycles)
            first_diag.max_age_cycles = mt6701_sample_age_cycles;
    }
    if (mt6701_transfer_active==MT_DMA && ++mt6701_transfer_age>3U)
        MT6701_Fault(MT6701_ERROR_TIMEOUT);
    MT6701_RequestSample();
}

uint8_t MT6701_IsHealthy(void)
{
    /* Fast admission needs a boolean, not the cold diagnostic bitfield and
     * saturated wall-time/failure packing. Keep R115's short-circuit gates,
     * taking one coherent publication with the caller's mask preserved. */
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool healthy=mt6701_continuous && mt6701_sample_valid && mt6701_transfer_active!=MT_RECOVER &&
        mt6701_sample_age_cycles<=MT6701_MAX_SAMPLE_AGE_CYCLES && HAL_GetTick()-mt6701_sample_at<=5U;
    __set_PRIMASK(mask); return healthy;
}
uint8_t MT6701_HasNearZeroGap(void)
{
    return 0U; /* No detector-only stale-angle bridge is needed anymore. */
}

uint16_t MT6701_GetRawAngle(void)
{
    return (uint16_t)Angle_Raw;
}

uint8_t MT6701_GetStatus(void)
{
    return mt6701_status;
}

uint32_t MT6701_GetSampleCounter(void)
{
    return mt6701_sample_counter;
}

uint32_t MT6701_GetErrorCounter(void)
{
    return first_diag.errors;
}

uint16_t MT6701_GetSampleAgeCycles(void)
{
    return mt6701_sample_age_cycles;
}

void MT6701_GetDiagnostics(MT6701Diagnostics *out)
{
    if (out == NULL) return;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *out = first_diag;
    out->age_cycles=mt6701_sample_age_cycles;
    out->status=mt6701_status;
    uint32_t health=MT6701_GetHealth(0);
    out->health=(uint8_t)health; out->wall_ms=(uint8_t)(health>>8); out->failures=0;
    __set_PRIMASK(mask);
}

const char *MT6701_ErrorText(MT6701Error error)
{
    static const char names[]=
        "ALL 0\0"
        "ALL 1\0"
        "CRC\0"
        "FIELD\0"
        "TRACK\0"
        "SPI\0"
        "DMA TMO\0"
        "DMA TE\0"
        "--";
    unsigned index;
    switch (error) {
    case MT6701_ERROR_FRAME: index=0; break;
    case MT6701_ERROR_ONES: index=1; break;
    case MT6701_ERROR_CRC: index=2; break;
    case MT6701_ERROR_FIELD: index=3; break;
    case MT6701_ERROR_TRACK: index=4; break;
    case MT6701_ERROR_SPI: index=5; break;
    case MT6701_ERROR_TIMEOUT: index=6; break;
    case MT6701_ERROR_DMA: index=7; break;
    default: index=8; break;
    }
    return CompactText_At(names,index);
}

static volatile MT6701_2_Snapshot second;
static MT6701Diagnostics second_diag;
static volatile uint32_t second_poll_at;
static uint32_t second_sample_at;
/* Optional ENC2 stops after five actual failures. A required ENC2 may probe
 * slowly after output releases its reservation; only valid data clears the
 * failure streak. Recovery never arms output or reserves SPI2 for motion. */
#define MT6701_2_MAX_CONSECUTIVE_FAILURES 5U
#define MT6701_2_RETRY_MS 250U
static volatile uint8_t second_failures;
static volatile bool second_realtime, second_active, second_reserved, second_flash;
static volatile bool second_recover,second_valid;
static volatile bool second_dma;
static uint8_t second_flash_depth;
static uint8_t second_rx[3], second_tx[3] = {255,255,255};
static volatile uint32_t second_transfer_at;
static MT6701_2_BusDiagnostics second_bus_diag;
static uint16_t second_transfer_age;
static volatile uint16_t second_age = UINT16_MAX;

uint32_t MT6701_GetHealth(unsigned axis)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool valid=axis ? second_valid && second.samples : mt6701_continuous && mt6701_sample_valid;
    bool recover=axis ? second_recover : mt6701_transfer_active==MT_RECOVER;
    unsigned failures=axis ? second_failures : 0U;
    unsigned age=axis ? second_age : mt6701_sample_age_cycles;
    uint32_t wall=HAL_GetTick()-(axis ? second_sample_at : mt6701_sample_at);
    unsigned flags=(!valid ? MT6701_HEALTH_MISSING : 0U) |
        (recover ? MT6701_HEALTH_RECOVER : 0U) |
        (failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES ? MT6701_HEALTH_STOPPED : 0U) |
        ((!axis || second_realtime) && age>MT6701_MAX_SAMPLE_AGE_CYCLES ? MT6701_HEALTH_AGE : 0U) |
        (wall>5U ? MT6701_HEALTH_WALL : 0U);
    __set_PRIMASK(mask);
    return flags|((wall>255U ? 255U : wall)<<8)|(failures<<16);
}

/* No saved register image: a failed/aborted transfer must never become the
 * next device's baseline. All owners use 8-bit full duplex, software CS.
 * Keep HAL's Init fields in agreement with the active hardware mode too. */
static void second_configure(bool encoder)
{
    hspi2.Init.CLKPhase=encoder ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    hspi2.Init.BaudRatePrescaler=encoder ? SPI_BAUDRATEPRESCALER_256 : SPI_BAUDRATEPRESCALER_8;
    hspi2.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;
    CLEAR_BIT(SPI2->CR1,SPI_CR1_SPE);
    SPI2->CR2=SPI_DATASIZE_8BIT|SPI_RXFIFO_THRESHOLD_QF;
    SPI2->CR1=SPI_MODE_MASTER|SPI_NSS_SOFT|hspi2.Init.CLKPhase|hspi2.Init.BaudRatePrescaler;
}
static bool second_bus_dirty(void)
{
    return hspi2.State!=HAL_SPI_STATE_READY ||
        (SPI2->SR & (SPI_SR_BSY|SPI_SR_FRLVL|SPI_SR_FTLVL|SPI_SR_OVR|SPI_SR_MODF|SPI_SR_FRE));
}
void MT6701_2_BusFault(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!second_recover) {
        second_bus_diag.cr1=(uint16_t)SPI2->CR1;
        second_bus_diag.cr2=(uint16_t)SPI2->CR2;
        second_bus_diag.sr=(uint16_t)SPI2->SR;
        second_recover=true;
        second_valid=false;
    }
    __set_PRIMASK(mask);
}
void MT6701_2_GetBusDiagnostics(MT6701_2_BusDiagnostics *out)
{
    if (!out) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=second_bus_diag;
    __set_PRIMASK(mask);
}
bool MT6701_2_StorageBlocked(void) { return second_reserved; }
bool MT6701_2_FlashBegin(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=!second_recover && (second_flash || (!second_reserved && !second_active &&
            !second_bus_dirty()));
    if (ok && !second_flash) second_configure(false);
    if (ok) { second_flash=true; ++second_flash_depth; }
    __set_PRIMASK(mask);
    return ok;
}
void MT6701_2_FlashEnd(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (second_flash_depth && --second_flash_depth==0U) second_flash=false;
    __set_PRIMASK(mask);
}
bool MT6701_2_Reserve(bool active)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=!active || (!second_flash && !second_recover &&
                         (!second_active || second_dma));
    if (ok) second_reserved=active;
    __set_PRIMASK(mask);
    return ok;
}
void MT6701_2_SetRealtime(bool enable) { second_realtime=enable; }
void MT6701_2_Retry(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    /* A stopped axis may still have one DMA frame in flight. Leave its CS
     * ownership untouched; retry must not be lost merely due to its timing. */
    if (!second_reserved) {
        second_failures=0U;
        second_age=UINT16_MAX;
    }
    __set_PRIMASK(mask);
}
static void second_restore(void)
{
    HAL_GPIO_WritePin(MT6701_CSN2_GPIO_Port, MT6701_CSN2_Pin, GPIO_PIN_SET);
    second_configure(false);
}
static void second_prepare(void)
{
    second_configure(true);
    HAL_GPIO_WritePin(MT6701_CSN2_GPIO_Port,MT6701_CSN2_Pin,GPIO_PIN_RESET);
    timed_start[1]=DWT->CYCCNT;
}


static void MT6701_2_RecordFailure(MT6701Error error)
{
    record_error(&second_diag,error,&second.errors);
    if (second_failures < MT6701_2_MAX_CONSECUTIVE_FAILURES) ++second_failures;
    if (second_failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES) second_poll_at=HAL_GetTick();
}

static void second_publish(const uint8_t rx[3])
{
    uint16_t raw;
    uint8_t status;
    MT6701Error error=MT6701_Decode(rx,&raw,&status);
    bool zero=!(rx[0]|rx[1]|rx[2]);
    if (zero && zero_jump(1,second.raw_angle)) error=MT6701_ERROR_FRAME;
    uint32_t now=HAL_GetTick();
    float angle=2.0f*PI-2.0f*PI*(float)raw/16384.0f;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    second.status=status;
    MT6701_ObserveZero(&second_diag,zero ? MT6701_ERROR_FRAME : error,raw,second.raw_angle,second_age,second_sample_at);
    if (error!=MT6701_ERROR_NONE) { MT6701_2_RecordFailure(error); __set_PRIMASK(mask); return; }
    second_failures=0U;
    second_valid=true;
    second.raw_angle=raw;
    second.angle=angle;
    publish_timed(1,second.angle);
    second_sample_at=now;
    second_age=0U;
    ++second.samples;
    __set_PRIMASK(mask);
}
static void second_complete(bool error)
{
    if (!second_active || !second_dma || second_recover) return;
    if (error) {
        MT6701_2_RecordFailure(SpiPort_GetError(&hspi2)&HAL_SPI_ERROR_DMA ? MT6701_ERROR_DMA : MT6701_ERROR_SPI);
        MT6701_2_BusFault(); /* HAL/DMA cleanup belongs to main, never ADC. */
        return;
    }
    second_dma=false; /* ADC may preempt decoding, but cannot time out publication. */
    second_publish(second_rx);
    second_restore();
    second_active=false; /* Publish last: ADC may preempt the DMA callback. */
}
void MT6701_2_FastService(void)
{
    SpiPort_Service(&hspi2);
    if (!second_realtime) return;
    if (second_age<UINT16_MAX) {
        ++second_age;
        if (second_age>second_diag.max_age_cycles) second_diag.max_age_cycles=second_age;
    }
    if (second_active) {
        if (second_dma && !second_recover && ++second_transfer_age>3U) {
            MT6701_2_RecordFailure(MT6701_ERROR_TIMEOUT);
            MT6701_2_BusFault();
        }
        return;
    }
    if (second_failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES || second_flash || second_recover ||
        second_bus_dirty() ||
        HAL_GPIO_ReadPin(W25_NSS_GPIO_Port,W25_NSS_Pin)!=GPIO_PIN_SET) return;
    second_active=true; second_dma=true;
    second_transfer_age=0U;
    second_transfer_at=HAL_GetTick();
    second_prepare();
    if (SpiPort_ExchangeDMA(&hspi2,second_tx,second_rx,3U)!=HAL_OK) {
        MT6701_2_RecordFailure(MT6701_ERROR_SPI);
        MT6701_2_BusFault();
    }
}
__attribute__((noinline,noclone)) void MT6701_2_Service(uint32_t now_ms)
{
    SpiPort_Service(&hspi2);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    /* Still runs if the ADC scheduler stops or M2 is disabled mid-frame. */
    if (second_active && second_dma && !second_recover && HAL_GetTick()-second_transfer_at>1U) {
        MT6701_2_RecordFailure(MT6701_ERROR_TIMEOUT);
        MT6701_2_BusFault();
    }
    if (!second_active && !second_flash &&
        HAL_GPIO_ReadPin(W25_NSS_GPIO_Port,W25_NSS_Pin)==GPIO_PIN_SET && second_bus_dirty())
        MT6701_2_BusFault();
    __set_PRIMASK(mask);
    if (second_flash) return;
    if (second_recover) {
        /* Hold ownership through abort/reset/init, including stale callbacks.
         * Abort alone can return READY with BSY/FIFO still dirty. Never restore
         * a pre-fault snapshot after reset. All blocking work stays in main. */
        (void)SpiPort_Abort(&hspi2);
        (void)SpiPort_DMAStop(&hspi2);
        HAL_GPIO_WritePin(W25_NSS_GPIO_Port,W25_NSS_Pin,GPIO_PIN_SET);
        second_restore();
        __HAL_RCC_SPI2_FORCE_RESET();
        __HAL_RCC_SPI2_RELEASE_RESET();
        if (HAL_SPI_Init(&hspi2)!=HAL_OK) return;
        ++second_bus_diag.resets;
        mask=__get_PRIMASK(); __disable_irq();
        second_recover=false; second_active=false;
        __set_PRIMASK(mask);
        return; /* Resume sampling on the next scheduler/service invocation. */
    }
    if (second_active || second_flash || second_reserved) return;
    bool stopped=second_failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES;
    if (second_realtime ? !stopped : stopped) return;
    if (stopped) now_ms=HAL_GetTick(); /* A fault IRQ may postdate main's tick. */
    /* One probe per interval, not a new five-frame burst. Keep DMA halted
     * until the probe passes all frame/CRC/magnetic checks. */
    if (now_ms-second_poll_at<(second_realtime ? MT6701_2_RETRY_MS : 1U)) return;
    mask=__get_PRIMASK(); __disable_irq();
    if (second_bus_dirty() ||
        HAL_GPIO_ReadPin(W25_NSS_GPIO_Port,W25_NSS_Pin)!=GPIO_PIN_SET) {
        __set_PRIMASK(mask); return;
    }
    second_poll_at=now_ms;
    second_active=true; second_dma=false;
    second_prepare();
    __set_PRIMASK(mask);
    uint8_t rx[3];
    HAL_StatusTypeDef result=SpiPort_Exchange(&hspi2,second_tx,rx,3U,1U);
    if (result==HAL_OK) second_publish(rx);
    else {
        MT6701_2_RecordFailure(MT6701_ERROR_SPI);
        MT6701_2_BusFault();
        /* Main owns this polling transfer; release CS, recover next service. */
    }
    second_restore();
    second_active=false;
}
void MT6701_2_GetSnapshot(MT6701_2_Snapshot *out)
{
    if (out==NULL) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=second;
    out->age_cycles=second_age;
    out->stopped=second_failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES;
    out->healthy=second_valid && !second_recover && !out->stopped && out->samples &&
        (!second_realtime || out->age_cycles<=MT6701_MAX_SAMPLE_AGE_CYCLES) && HAL_GetTick()-second_sample_at<=5U;
    __set_PRIMASK(mask);
}

void MT6701_2_GetDiagnostics(MT6701Diagnostics *out)
{
    if (!out) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=second_diag; out->errors=second.errors;
    out->age_cycles=second_age; out->status=second.status;
    uint32_t health=MT6701_GetHealth(1);
    out->health=(uint8_t)health;out->wall_ms=(uint8_t)(health>>8);out->failures=(uint8_t)(health>>16);
    __set_PRIMASK(mask);
}
