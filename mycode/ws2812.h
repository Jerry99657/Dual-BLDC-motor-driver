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

#endif /* __WS2812_H */
