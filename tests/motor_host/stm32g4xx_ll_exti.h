#ifndef MOTOR_HOST_LL_EXTI_H
#define MOTOR_HOST_LL_EXTI_H
#include "main.h"
static inline uint32_t LL_EXTI_IsActiveFlag_0_31(uint32_t line)
{ return (host_exti_pending & line)==line; }
static inline void LL_EXTI_ClearFlag_0_31(uint32_t line)
{ host_exti_pending &= ~line; } /* Model the peripheral's W1C behavior. */
#endif
