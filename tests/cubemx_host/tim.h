#pragma once
#include "spi.h"
#define TIM_DMA_ID_CC1 0U
typedef struct { DMA_HandleTypeDef *hdma[1]; } TIM_HandleTypeDef;
extern TIM_HandleTypeDef htim3;
