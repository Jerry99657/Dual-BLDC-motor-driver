#ifndef __WS2812_H
#define __WS2812_H

#include "main.h"

/* 新板只有一颗 WS2812，只用于显示状态 */
#define WS2812_LED_COUNT 1U

void WS2812_Init(void);
void WS2812_SetRgb(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
void WS2812_Show(void);
void WS2812_Service(void);
uint8_t WS2812_IsBusy(void);
/* Generated vector routing owns the handle check; only the linked TIM3 CH1
 * DMA may enter this dispatcher. Initialization remains CubeMX/HAL-owned. */
void WS2812_DMAIRQ(DMA_HandleTypeDef *dma);

#endif /* __WS2812_H */
