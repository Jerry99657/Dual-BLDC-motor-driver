#ifndef MOTOR_HOST_DAC_H
#define MOTOR_HOST_DAC_H
#include "main.h"
typedef struct { unsigned started,code; } DAC_HandleTypeDef;
extern DAC_HandleTypeDef hdac3;
#define DAC_CHANNEL_2 2U
#define DAC_ALIGN_12B_R 0U
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *,uint32_t);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *,uint32_t,uint32_t,uint32_t);
#endif
