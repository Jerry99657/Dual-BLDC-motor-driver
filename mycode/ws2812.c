#include "ws2812.h"
#include "tim.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_tim.h"
#include <stdbool.h>
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
/* Keep 32-bit base alignment, but pack items according to CubeMX's linked
 * DMA memory width. No DMA controller/channel/request/priority is fixed here. */
static union {
    uint32_t word[WS2812_DMA_LENGTH];
    uint16_t half[WS2812_DMA_LENGTH];
} ws2812_dma_buffer;
static volatile uint8_t ws2812_busy = 0U;
static volatile uint8_t ws2812_pending = 0U;
static uint32_t ws2812_fault_at;

static void stop(DMA_HandleTypeDef *dma)
{
    TIM_TypeDef *tim=htim3.Instance;
    LL_TIM_DisableDMAReq_CC1(tim);
    LL_TIM_DisableCounter(tim);
    LL_DMA_DisableChannel(dma->DmaBaseAddress,dma->ChannelIndex>>2U);
    CLEAR_BIT(dma->Instance->CCR,DMA_CCR_TCIE|DMA_CCR_HTIE|DMA_CCR_TEIE);
    LL_DMA_WriteReg(dma->DmaBaseAddress,IFCR,15UL<<dma->ChannelIndex);
    /* Latch zero even after an interrupted data pulse. TIM3 is dedicated to
     * this output; UPDATE runs only with its DMA request disabled. */
    LL_TIM_OC_SetCompareCH1(tim,0U);
    LL_TIM_GenerateEvent_UPDATE(tim);
    LL_TIM_CC_DisableChannel(tim,LL_TIM_CHANNEL_CH1);
}

static void WS2812_BuildDmaBuffer(void)
{
    memset(&ws2812_dma_buffer, 0, sizeof(ws2812_dma_buffer));
    uint16_t write_index = WS2812_LEADING_LOW_SLOTS;
    bool half=htim3.hdma[TIM_DMA_ID_CC1]->Init.MemDataAlignment==DMA_MDATAALIGN_HALFWORD;

    for (uint8_t led = 0U; led < WS2812_LED_COUNT; led++) {
        uint32_t rgb = ws2812_colors[led];
        uint8_t red = (uint8_t)((rgb >> 16) & 0xFFU);
        uint8_t green = (uint8_t)((rgb >> 8) & 0xFFU);
        uint8_t blue = (uint8_t)(rgb & 0xFFU);
        uint32_t grb = ((uint32_t)green << 16) |
                       ((uint32_t)red << 8) |
                       (uint32_t)blue;

        for (int8_t bit = 23; bit >= 0; bit--) {
            uint16_t duty=(grb & (1UL << bit)) ? WS2812_ONE_DUTY : WS2812_ZERO_DUTY;
            if (half) ws2812_dma_buffer.half[write_index]=duty;
            else ws2812_dma_buffer.word[write_index]=duty;
            ++write_index;
        }
    }
}

void WS2812_Init(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    stop(htim3.hdma[TIM_DMA_ID_CC1]);
    ws2812_busy = 0U;
    ws2812_pending = 0U;
    __set_PRIMASK(mask);
    memset(ws2812_colors, 0, sizeof(ws2812_colors));
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
    DMA_HandleTypeDef *dma=htim3.hdma[TIM_DMA_ID_CC1];
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    stop(dma);
    LL_TIM_SetCounter(htim3.Instance,0U);
    LL_DMA_SetPeriphAddress(dma->DmaBaseAddress,dma->ChannelIndex>>2U,
                          (uint32_t)(uintptr_t)&htim3.Instance->CCR1);
    LL_DMA_SetMemoryAddress(dma->DmaBaseAddress,dma->ChannelIndex>>2U,
                          (uint32_t)(uintptr_t)ws2812_dma_buffer.word);
    LL_DMA_SetDataLength(dma->DmaBaseAddress,dma->ChannelIndex>>2U,WS2812_DMA_LENGTH);
    SET_BIT(dma->Instance->CCR,DMA_CCR_TCIE|DMA_CCR_TEIE);
    LL_DMA_EnableChannel(dma->DmaBaseAddress,dma->ChannelIndex>>2U);
    LL_TIM_EnableDMAReq_CC1(htim3.Instance);
    LL_TIM_CC_EnableChannel(htim3.Instance,LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(htim3.Instance);
    __set_PRIMASK(mask);
}

void WS2812_Service(void)
{
    /* Also service a completed/error transfer if its IRQ was delayed. */
    if (ws2812_busy==1U) WS2812_DMAIRQ(htim3.hdma[TIM_DMA_ID_CC1]);
    /* After a truncated frame, hold low for >=1 ms (two tick boundaries)
     * before retrying the latest colors. Normal frames have 80 us low tail. */
    if (ws2812_busy==2U && (uint32_t)(HAL_GetTick()-ws2812_fault_at)>=2U)
        ws2812_busy=0U;
    if (ws2812_pending && !ws2812_busy) {
        WS2812_Show();
    }
}

uint8_t WS2812_IsBusy(void)
{
    return ws2812_busy;
}

void WS2812_DMAIRQ(DMA_HandleTypeDef *dma)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    uint32_t flags=LL_DMA_ReadReg(dma->DmaBaseAddress,ISR)>>dma->ChannelIndex;
    uint32_t enabled=dma->Instance->CCR;
    if (ws2812_busy==1U && (((flags&8U) && (enabled&DMA_CCR_TEIE)) ||
                       ((flags&2U) && (enabled&DMA_CCR_TCIE)))) {
        bool error=(flags&8U)!=0U || dma->Instance->CNDTR!=0U;
        stop(dma);
        ws2812_busy=error ? 2U : 0U;
        if (error) { ws2812_fault_at=HAL_GetTick(); ws2812_pending=1U; }
    }
    __set_PRIMASK(mask);
}
