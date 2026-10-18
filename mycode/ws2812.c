#include "ws2812.h"
#include "tim.h"
#include <string.h>

/* TIM3 = 170 MHz, ARR = 212: 1 bit = 213 / 170 MHz = 1.253 us */
#define WS2812_ZERO_DUTY           68U
#define WS2812_ONE_DUTY           136U
#define WS2812_LEADING_LOW_SLOTS   2U
#define WS2812_RESET_LOW_SLOTS     64U
#define WS2812_BITS_PER_LED        24U
#define WS2812_DMA_LENGTH (WS2812_LEADING_LOW_SLOTS + \
                           WS2812_LED_COUNT * WS2812_BITS_PER_LED + \
                           WS2812_RESET_LOW_SLOTS)

static uint32_t ws2812_colors[WS2812_LED_COUNT] = {0U};
/* HAL_TIM_PWM_Start_DMA 以 32 位访问比较值，缓冲区必须是 uint32_t */
static uint32_t ws2812_dma_buffer[WS2812_DMA_LENGTH];
static volatile uint8_t ws2812_busy = 0U;
static volatile uint8_t ws2812_pending = 0U;

static void WS2812_BuildDmaBuffer(void)
{
    memset(ws2812_dma_buffer, 0, sizeof(ws2812_dma_buffer));
    uint16_t write_index = WS2812_LEADING_LOW_SLOTS;

    for (uint8_t led = 0U; led < WS2812_LED_COUNT; led++) {
        uint32_t rgb = ws2812_colors[led];
        uint8_t red = (uint8_t)((rgb >> 16) & 0xFFU);
        uint8_t green = (uint8_t)((rgb >> 8) & 0xFFU);
        uint8_t blue = (uint8_t)(rgb & 0xFFU);
        uint32_t grb = ((uint32_t)green << 16) |
                       ((uint32_t)red << 8) |
                       (uint32_t)blue;

        for (int8_t bit = 23; bit >= 0; bit--) {
            ws2812_dma_buffer[write_index++] =
                (grb & (1UL << bit)) ? WS2812_ONE_DUTY
                                     : WS2812_ZERO_DUTY;
        }
    }
}

void WS2812_Init(void)
{
    ws2812_busy = 0U;
    ws2812_pending = 0U;
    memset(ws2812_colors, 0, sizeof(ws2812_colors));
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    WS2812_Show();
}

void WS2812_SetRgb(uint8_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if (index >= WS2812_LED_COUNT) {
        return;
    }
    ws2812_colors[index] = ((uint32_t)red << 16) |
                           ((uint32_t)green << 8) |
                           (uint32_t)blue;
}

void WS2812_Show(void)
{
    if (ws2812_busy) {
        ws2812_pending = 1U;
        return;
    }

    WS2812_BuildDmaBuffer();
    ws2812_pending = 0U;
    ws2812_busy = 1U;
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);

    if (HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1,
                              ws2812_dma_buffer,
                              (uint16_t)WS2812_DMA_LENGTH) != HAL_OK) {
        ws2812_busy = 0U;
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0U);
    }
}

void WS2812_Service(void)
{
    if (ws2812_pending && !ws2812_busy) {
        WS2812_Show();
    }
}

uint8_t WS2812_IsBusy(void)
{
    return ws2812_busy;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
        ws2812_busy = 0U;
    }
}

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0U);
        ws2812_busy = 0U;
    }
}
