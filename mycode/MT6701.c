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

static uint8_t MT6701_Decode(const uint8_t data[3], uint16_t *angle, uint8_t *status)
{
    uint32_t frame = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *angle = (uint16_t)(frame >> 10);
    *status = (uint8_t)((frame >> 6) & 0x0FU);
    return frame != 0U && frame != 0xFFFFFFU &&
           (frame & 0x3FU) == MT6701_CalculateCrc6(frame >> 6) &&
           (*status & 0x0BU) == 0U;
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
        mt6701_error_counter++;
        return 0U;
    }
    return 1U;
}

/* SPI DMA 收发完成回调 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        CS_Disable;
        /* Keep the buffer owned until decoding/publishing has finished:
         * the higher-priority ADC ISR may request the next DMA sample. */

        uint16_t raw_angle;
        uint8_t status;
        uint8_t valid = MT6701_Decode(MT6701_Data, &raw_angle, &status);
        mt6701_status = status;

        /* Mg[1:0] 非零表示磁场过强/过弱，Mg[3] 表示失跟踪；
         * Mg[2] 是按压状态，不影响转子角度。全 0/全 1 帧用于识别 MISO 卡死。 */
        if (!valid) {
            mt6701_error_counter++;
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
        mt6701_error_counter++;
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

static MT6701_2_Snapshot second;
static uint32_t second_poll_at, second_sample_at;

void MT6701_2_Service(uint32_t now_ms)
{
    if (now_ms - second_poll_at < 1U) return;
    second_poll_at = now_ms;
    if (hspi2.State != HAL_SPI_STATE_READY || (SPI2->SR & SPI_SR_BSY) ||
        HAL_GPIO_ReadPin(W25_NSS_GPIO_Port, W25_NSS_Pin) != GPIO_PIN_SET) return;

    /* Both clients run in the main loop. No SPI2 transaction may be launched
     * from an ISR: preserving the Flash command/CS interval is essential. */
    uint32_t saved_cr1 = SPI2->CR1;
    uint32_t saved_cr2 = SPI2->CR2;
    CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
    MODIFY_REG(SPI2->CR1, SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_BR,
               SPI_PHASE_2EDGE | SPI_BAUDRATEPRESCALER_256);
    HAL_GPIO_WritePin(MT6701_CSN2_GPIO_Port, MT6701_CSN2_Pin, GPIO_PIN_RESET);
    const uint8_t tx[3] = {0xFFU, 0xFFU, 0xFFU};
    uint8_t rx[3];
    HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3U, 1U);
    HAL_GPIO_WritePin(MT6701_CSN2_GPIO_Port, MT6701_CSN2_Pin, GPIO_PIN_SET);
    if (result != HAL_OK) (void)HAL_SPI_Abort(&hspi2);
    CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
    SPI2->CR2 = saved_cr2;
    SPI2->CR1 = saved_cr1;

    uint16_t raw;
    uint8_t status;
    if (result != HAL_OK) {
        ++second.errors;
        return;
    }
    uint8_t valid = MT6701_Decode(rx, &raw, &status);
    second.status = status;
    if (!valid) {
        ++second.errors;
        return;
    }
    second.raw_angle = raw;
    second.angle = 2.0f * PI - 2.0f * PI * (float)raw / 16384.0f;
    ++second.samples;
    second_sample_at = HAL_GetTick();
}

void MT6701_2_GetSnapshot(MT6701_2_Snapshot *out)
{
    if (out == NULL) return;
    *out = second;
    out->healthy = second.samples != 0U && HAL_GetTick() - second_sample_at <= 5U;
}
