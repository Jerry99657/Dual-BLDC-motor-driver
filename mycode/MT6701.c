#include "MT6701.h"
#include "kk_oled_driver.h"
#include <stdint.h>

uint8_t MT6701_Data[3] = {0};
volatile int Angle_Raw = 0;
volatile float Angle = 0.0f;

static volatile uint8_t mt6701_continuous = 0U;
static volatile uint8_t mt6701_transfer_active = 0U;
static volatile uint8_t mt6701_sample_valid = 0U;
static volatile uint8_t mt6701_status = 0U;
static volatile uint16_t mt6701_sample_age_cycles = UINT16_MAX;
static volatile uint32_t mt6701_sample_counter = 0U;
static volatile uint32_t mt6701_error_counter = 0U;
static volatile uint32_t mt6701_sample_at = 0U;
static volatile uint32_t crc_errors, field_errors, track_errors, spi_errors, frame_errors;
static volatile uint16_t max_sample_age_cycles;
static volatile MT6701Error last_error;

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
    if (frame == 0U || frame == 0xFFFFFFU) return MT6701_ERROR_FRAME;
    if ((frame & 0x3FU) != MT6701_CalculateCrc6(frame >> 6)) return MT6701_ERROR_CRC;
    if (*status & 0x08U) return MT6701_ERROR_TRACK;
    if (*status & 0x03U) return MT6701_ERROR_FIELD;
    return MT6701_ERROR_NONE;
}

static void MT6701_RecordError(MT6701Error error)
{
    ++mt6701_error_counter;
    last_error = error;
    switch (error) {
    case MT6701_ERROR_CRC: ++crc_errors; break;
    case MT6701_ERROR_FIELD: ++field_errors; break;
    case MT6701_ERROR_TRACK: ++track_errors; break;
    case MT6701_ERROR_SPI: ++spi_errors; break;
    default: ++frame_errors; break;
    }
}

static uint8_t MT6701_RequestSample(void)
{
    if (!mt6701_continuous || mt6701_transfer_active) {
        return 0U;
    }

    mt6701_transfer_active = 1U;
    CS_Enable;
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(
        &hspi1, MT6701_Data, MT6701_Data, 3);
    if (status != HAL_OK) {
        CS_Disable;
        mt6701_transfer_active = 0U;
        MT6701_RecordError(MT6701_ERROR_SPI);
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
        CS_Disable;
        /* Keep the buffer owned until decoding/publishing has finished:
         * the higher-priority ADC ISR may request the next DMA sample. */

        uint16_t raw_angle;
        uint8_t status;
        MT6701Error error = MT6701_Decode(MT6701_Data, &raw_angle, &status);
        mt6701_status = status;

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
        CS_Disable;
        mt6701_transfer_active = 0U;
        MT6701_RecordError(MT6701_ERROR_SPI);
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

uint8_t MT6701_IsContinuous(void)
{
    return mt6701_continuous;
}

void MT6701_Service(void)
{
    if (!mt6701_continuous) return;

    if (mt6701_sample_valid && mt6701_sample_age_cycles < UINT16_MAX) {
        mt6701_sample_age_cycles++;
        if (mt6701_sample_age_cycles > max_sample_age_cycles)
            max_sample_age_cycles = mt6701_sample_age_cycles;
    }
    MT6701_RequestSample();
}

uint8_t MT6701_IsHealthy(void)
{
    uint32_t sampled_at = mt6701_sample_at;
    return (mt6701_continuous && mt6701_sample_valid &&
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
    return mt6701_error_counter;
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
    *out = (MT6701Diagnostics){
        .errors=mt6701_error_counter, .crc_errors=crc_errors,
        .field_errors=field_errors, .track_errors=track_errors,
        .spi_errors=spi_errors, .frame_errors=frame_errors,
        .age_cycles=mt6701_sample_age_cycles, .max_age_cycles=max_sample_age_cycles,
        .status=mt6701_status, .last_error=last_error
    };
    __set_PRIMASK(mask);
}

const char *MT6701_ErrorText(MT6701Error error)
{
    switch (error) {
    case MT6701_ERROR_FRAME: return "NO FRAME";
    case MT6701_ERROR_CRC: return "CRC";
    case MT6701_ERROR_FIELD: return "FIELD";
    case MT6701_ERROR_TRACK: return "TRACK";
    case MT6701_ERROR_SPI: return "SPI";
    default: return "--";
    }
}

static volatile MT6701_2_Snapshot second;
static MT6701Diagnostics second_diag;
static uint32_t second_poll_at, second_sample_at;
/* Optional M2 encoder: a missing/broken device must not monopolize SPI2.
 * Count actual failed transactions, not bus contention; reset only on a valid
 * sample. Once stopped, leave SPI2 untouched until the next MCU reset. */
#define MT6701_2_MAX_CONSECUTIVE_FAILURES 5U
static volatile uint8_t second_failures;
static volatile bool second_realtime, second_active, second_reserved, second_flash;
static volatile bool second_recover;
static bool second_dma;
static uint8_t second_flash_depth;
static uint8_t second_rx[3], second_tx[3] = {255,255,255};
static uint32_t second_cr1, second_cr2;
static uint16_t second_transfer_age;
static volatile uint16_t second_age = UINT16_MAX;

bool MT6701_2_StorageBlocked(void) { return second_reserved; }
bool MT6701_2_FlashBegin(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=second_flash || (!second_reserved && !second_active &&
            hspi2.State==HAL_SPI_STATE_READY && !(SPI2->SR & SPI_SR_BSY));
    if (ok) { second_flash=true; ++second_flash_depth; }
    __set_PRIMASK(mask);
    return ok;
}
void MT6701_2_FlashEnd(void)
{
    if (second_flash_depth && --second_flash_depth==0U) second_flash=false;
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
    CLEAR_BIT(SPI2->CR1,SPI_CR1_SPE);
    SPI2->CR2=second_cr2;
    SPI2->CR1=second_cr1;
}
static void second_prepare(void)
{
    second_cr1=SPI2->CR1; second_cr2=SPI2->CR2;
    CLEAR_BIT(SPI2->CR1,SPI_CR1_SPE);
    MODIFY_REG(SPI2->CR1,SPI_CR1_CPOL|SPI_CR1_CPHA|SPI_CR1_BR,
               SPI_PHASE_2EDGE|SPI_BAUDRATEPRESCALER_256);
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
}

static void second_publish(const uint8_t rx[3])
{
    uint16_t raw;
    uint8_t status;
    MT6701Error error=MT6701_Decode(rx,&raw,&status);
    second.status=status;
    if (error!=MT6701_ERROR_NONE) { MT6701_2_RecordFailure(error); return; }
    second_failures=0U;
    second.raw_angle=raw;
    second.angle=2.0f*PI-2.0f*PI*(float)raw/16384.0f;
    second_sample_at=HAL_GetTick();
    second_age=0U;
    ++second.samples;
}
static void second_complete(bool error)
{
    if (!second_active || second_recover) return;
    if (error) {
        MT6701_2_RecordFailure(MT6701_ERROR_SPI);
        second_recover=true; /* HAL/DMA cleanup belongs to main, never ADC. */
        return;
    }
    second_publish(second_rx);
    second_restore();
    second_active=false; /* Publish last: ADC may preempt the DMA callback. */
}
void MT6701_2_FastService(void)
{
    if (!second_realtime) return;
    if (second.samples && second_age<UINT16_MAX) ++second_age;
    if (second.samples && second_age>second_diag.max_age_cycles) second_diag.max_age_cycles=second_age;
    if (second_active) {
        if (!second_recover && ++second_transfer_age>3U) {
            MT6701_2_RecordFailure(MT6701_ERROR_SPI);
            second_recover=true;
        }
        return;
    }
    if (second_failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES || second_flash ||
        hspi2.State!=HAL_SPI_STATE_READY || (SPI2->SR & SPI_SR_BSY) ||
        HAL_GPIO_ReadPin(W25_NSS_GPIO_Port,W25_NSS_Pin)!=GPIO_PIN_SET) return;
    second_active=true; second_dma=true;
    second_transfer_age=0U;
    second_prepare();
    if (HAL_SPI_TransmitReceive_DMA(&hspi2,second_tx,second_rx,3U)!=HAL_OK) {
        MT6701_2_RecordFailure(MT6701_ERROR_SPI);
        second_recover=true;
    }
}
void MT6701_2_Service(uint32_t now_ms)
{
    if (second_recover) {
        /* Keep active set until both DMA channels and SPI are quiescent. */
        (void)HAL_SPI_Abort(&hspi2);
        (void)HAL_SPI_DMAStop(&hspi2);
        second_restore();
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        second_recover=false; second_active=false;
        __set_PRIMASK(mask);
    }
    if (second_realtime || second_active || second_flash ||
        second_failures>=MT6701_2_MAX_CONSECUTIVE_FAILURES) return;
    if (now_ms-second_poll_at<1U) return;
    second_poll_at=now_ms;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (hspi2.State!=HAL_SPI_STATE_READY || (SPI2->SR & SPI_SR_BSY) ||
        HAL_GPIO_ReadPin(W25_NSS_GPIO_Port,W25_NSS_Pin)!=GPIO_PIN_SET) {
        __set_PRIMASK(mask); return;
    }
    second_active=true; second_dma=false;
    second_prepare();
    __set_PRIMASK(mask);
    uint8_t rx[3];
    HAL_StatusTypeDef result=HAL_SPI_TransmitReceive(&hspi2,second_tx,rx,3U,1U);
    if (result!=HAL_OK) (void)HAL_SPI_Abort(&hspi2);
    if (result==HAL_OK) second_publish(rx);
    else MT6701_2_RecordFailure(MT6701_ERROR_SPI);
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
    out->healthy=!out->stopped && out->samples!=0U &&
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
