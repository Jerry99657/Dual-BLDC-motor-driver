#ifndef MOTOR_HOST_LL_GPIO_H
#define MOTOR_HOST_LL_GPIO_H
#include "main.h"
static inline uint32_t LL_GPIO_IsInputPinSet(GPIO_TypeDef *port,uint32_t pin)
{ return HAL_GPIO_ReadPin(port,(uint16_t)pin)==GPIO_PIN_SET; }
#endif
