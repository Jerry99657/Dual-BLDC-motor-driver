#include "MT6701.h"
#include "kk_oled_driver.h"
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

/* 10 kHz 调度下允许连续 3 个控制周期没有新样本；超过后有感 FOC 撤掉输出。 */
#define MT6701_MAX_SAMPLE_AGE_CYCLES 3U

/* MT6701 SSI 帧：14 位角度 + 4 位状态 + 6 位 CRC。
 * CRC 覆盖前 18 位，生成多项式 x^6 + x + 1（去掉最高项后为 0x03），MSB 先入。 */
static uint8_t MT6701_CalculateCrc6(uint32_t data18)
{
    uint8_t crc = 0U;
    for (int8_t bit_index = 17; bit_index >= 0; bit_index--) {
        uint8_t feedback = (uint8_t)(((data18 >> bit_index) & 1U) ^
                                     ((crc >> 5) & 1U));
        crc = (uint8_t)((crc << 1) & 0x3FU);
        if (feedback) {
            crc ^= 0x03U;
        }
    }
    return crc;
}

static MT6701Error MT6701_Decode(const uint8_t data[3], uint16_t *angle, uint8_t *status)
{
    uint32_t frame = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *angle = (uint16_t)(frame >> 10);
    *status = (uint8_t)((frame >> 6) & 0x0FU);
    /* Keep the existing rejection policy, but distinguish stuck-level data.
     * Zero is CRC-valid for angle/status zero; this diagnostic alone cannot
     * distinguish that real position from a disconnected/held-low MISO. */
    if (frame == 0U) return MT6701_ERROR_FRAME;
    if (frame == 0xFFFFFFU) return MT6701_ERROR_ONES;
    if ((frame & 0x3FU) != MT6701_CalculateCrc6(frame >> 6)) return MT6701_ERROR_CRC;
    if (*status & 0x08U) return MT6701_ERROR_TRACK;
    if (*status & 0x03U) return MT6701_ERROR_FIELD;
    return MT6701_ERROR_NONE;
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

static void MT6701_RecordError(MT6701Error error)
{
    ++first_diag.errors;
    first_diag.last_error = error;
    switch (error) {
    case MT6701_ERROR_CRC: ++first_diag.crc_errors; break;
    case MT6701_ERROR_FIELD: ++first_diag.field_errors; break;
    case MT6701_ERROR_TRACK: ++first_diag.track_errors; break;
    case MT6701_ERROR_SPI:
    case MT6701_ERROR_TIMEOUT: ++first_diag.spi_errors; break;
    default: ++first_diag.frame_errors; break;
    }
}

static void MT6701_Fault(MT6701Error error)
{
    if (!mt6701_continuous || mt6701_transfer_active==MT_RECOVER) return;
    /* Hold ownership through main-loop cleanup. DMA callbacks and the ADC
     * scheduler must not publish/reuse this buffer during abort/reset. */
    mt6701_transfer_active=MT_RECOVER;
    mt6701_sample_valid=0U;
    MT6701_RecordError(error);
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
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
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
        mt6701_status = status;
        MT6701_ObserveZero(&first_diag,error,raw_angle,(uint16_t)Angle_Raw,
                           mt6701_sample_age_cycles,mt6701_sample_at);

        /* Mg[1:0] 非零表示磁场过强/过弱，Mg[3] 表示失跟踪；
         * Mg[2] 是按压状态，不影响转子角度。全 0/全 1 帧用于识别 MISO 卡死。 */
        if (error != MT6701_ERROR_NONE) {
            MT6701_RecordError(error);
            mt6701_transfer_active = 0U;
            return;
        }

        Angle_Raw = (int)raw_angle;
        Angle = 2.0f * PI - (2.0f * PI * (float)Angle_Raw / 16384.0f);

        mt6701_sample_age_cycles = 0U;
        mt6701_sample_at = HAL_GetTick();
        mt6701_sample_valid = 1U;
        mt6701_sample_counter++;
        mt6701_transfer_active = 0U;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        if (mt6701_transfer_active==MT_DMA) MT6701_Fault(MT6701_ERROR_SPI);
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
    HAL_SPI_DMAStop(&hspi1);
    CS_Disable;
    mt6701_transfer_active = 0U;
    mt6701_sample_valid = 0U;
    mt6701_sample_age_cycles = UINT16_MAX;
}

void MT6701_MainService(void)
{
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
    (void)HAL_SPI_Abort(&hspi1);
    (void)HAL_SPI_DMAStop(&hspi1);
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
    uint32_t sampled_at = mt6701_sample_at;
    return (mt6701_continuous && mt6701_sample_valid && mt6701_transfer_active!=MT_RECOVER &&
            HAL_GetTick() - sampled_at <= 5U &&
            mt6701_sample_age_cycles <= MT6701_MAX_SAMPLE_AGE_CYCLES) ? 1U : 0U;
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
    __set_PRIMASK(mask);
}

const char *MT6701_ErrorText(MT6701Error error)
{
    switch (error) {
    case MT6701_ERROR_FRAME: return "ALL 0";
    case MT6701_ERROR_ONES: return "ALL 1";
    case MT6701_ERROR_CRC: return "CRC";
    case MT6701_ERROR_FIELD: return "FIELD";
    case MT6701_ERROR_TRACK: return "TRACK";
    case MT6701_ERROR_SPI: return "SPI";
    case MT6701_ERROR_TIMEOUT: return "DMA TMO";
    default: return "--";
    }
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
}


static void MT6701_2_RecordFailure(MT6701Error error)
{
    ++second.errors;
    second_diag.last_error=error;
    if (error==MT6701_ERROR_CRC) ++second_diag.crc_errors;
    else if (error==MT6701_ERROR_FIELD) ++second_diag.field_errors;
    else if (error==MT6701_ERROR_TRACK) ++second_diag.track_errors;
    else if (error==MT6701_ERROR_SPI) ++second_diag.spi_errors;
    else ++second_diag.frame_errors;
    if (second_failures < MT6701_2_MAX_CONSECUTIVE_FAILURES) ++second_failures;
    if (second_failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES) second_poll_at=HAL_GetTick();
}

static void second_publish(const uint8_t rx[3])
{
    uint16_t raw;
    uint8_t status;
    MT6701Error error=MT6701_Decode(rx,&raw,&status);
    second.status=status;
    MT6701_ObserveZero(&second_diag,error,raw,second.raw_angle,second_age,second_sample_at);
    if (error!=MT6701_ERROR_NONE) { MT6701_2_RecordFailure(error); return; }
    second_failures=0U;
    second_valid=true;
    second.raw_angle=raw;
    second.angle=2.0f*PI-2.0f*PI*(float)raw/16384.0f;
    second_sample_at=HAL_GetTick();
    second_age=0U;
    ++second.samples;
}
static void second_complete(bool error)
{
    if (!second_active || !second_dma || second_recover) return;
    if (error) {
        MT6701_2_RecordFailure(MT6701_ERROR_SPI);
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
    if (!second_realtime) return;
    if (second_age<UINT16_MAX) {
        ++second_age;
        if (second_age>second_diag.max_age_cycles) second_diag.max_age_cycles=second_age;
    }
    if (second_active) {
        if (second_dma && !second_recover && ++second_transfer_age>3U) {
            MT6701_2_RecordFailure(MT6701_ERROR_SPI);
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
    if (HAL_SPI_TransmitReceive_DMA(&hspi2,second_tx,second_rx,3U)!=HAL_OK) {
        MT6701_2_RecordFailure(MT6701_ERROR_SPI);
        MT6701_2_BusFault();
    }
}
void MT6701_2_Service(uint32_t now_ms)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    /* Still runs if the ADC scheduler stops or M2 is disabled mid-frame. */
    if (second_active && second_dma && !second_recover && HAL_GetTick()-second_transfer_at>1U) {
        MT6701_2_RecordFailure(MT6701_ERROR_SPI);
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
        (void)HAL_SPI_Abort(&hspi2);
        (void)HAL_SPI_DMAStop(&hspi2);
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
    HAL_StatusTypeDef result=HAL_SPI_TransmitReceive(&hspi2,second_tx,rx,3U,1U);
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
    uint32_t at=second_sample_at;
    out->age_cycles=second_age;
    out->stopped=second_failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES;
    out->healthy=second_valid && !second_recover && !out->stopped && out->samples!=0U &&
                 (!second_realtime || second_age<=MT6701_MAX_SAMPLE_AGE_CYCLES);
    __set_PRIMASK(mask);
    out->healthy=out->healthy && HAL_GetTick()-at<=5U;
}

void MT6701_2_GetDiagnostics(MT6701Diagnostics *out)
{
    if (!out) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=second_diag; out->errors=second.errors;
    out->age_cycles=second_age; out->status=second.status;
    __set_PRIMASK(mask);
}
