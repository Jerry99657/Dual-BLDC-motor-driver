#ifndef MOTOR_HOST_COMP_H
#define MOTOR_HOST_COMP_H
#include "main.h"
typedef struct { unsigned started; } COMP_HandleTypeDef;
extern COMP_HandleTypeDef hcomp4;
#define COMP_OUTPUT_LEVEL_LOW 0U
#define COMP_OUTPUT_LEVEL_HIGH 1U
HAL_StatusTypeDef HAL_COMP_Start(COMP_HandleTypeDef *);
uint32_t HAL_COMP_GetOutputLevel(const COMP_HandleTypeDef *);
#endif
