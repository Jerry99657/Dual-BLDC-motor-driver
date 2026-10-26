#ifndef IRQ_HOST_TIM_H
#define IRQ_HOST_TIM_H
#include "main.h"
#include "irq_constants.h"
typedef struct { uint32_t SR,DIER; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
extern TIM_TypeDef irq_tim1,irq_tim2;
#define TIM1 (&irq_tim1)
#define TIM2 (&irq_tim2)
#endif
