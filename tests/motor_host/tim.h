#ifndef MOTOR_HOST_TIM_H
#define MOTOR_HOST_TIM_H
#include "main.h"
typedef struct { uint32_t CR1, CNT, CCR1, CCR2, CCR3, ARR, CCER, BDTR, EGR; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
extern TIM_TypeDef tim1, tim8, tim6;
extern TIM_HandleTypeDef htim1, htim8, htim6;
#define TIM2 ((TIM_TypeDef *)2)
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 4U
#define TIM_CHANNEL_3 8U
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_SET_COMPARE(h,ch,value) do { \
    if ((ch)==TIM_CHANNEL_1) (h)->Instance->CCR1=(value); \
    else if ((ch)==TIM_CHANNEL_2) (h)->Instance->CCR2=(value); \
    else (h)->Instance->CCR3=(value); \
} while(0)
#define TIM_CR1_CEN 1U
#define TIM_CR1_DIR (1U << 4)
#define TIM_CR1_CMS (3U << 5)
#define TIM_EVENTSOURCE_UPDATE 1U
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC1NE (1U << 2)
#define TIM_CCER_CC2E (1U << 4)
#define TIM_CCER_CC2NE (1U << 6)
#define TIM_CCER_CC3E (1U << 8)
#define TIM_CCER_CC3NE (1U << 10)
#define TIM_CCER_CC4E (1U << 12)
#define TIM_BDTR_MOE (1U << 15)
/* Same unconditional shutdown behavior as the real STM32 HAL macro. */
#define __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(h) ((h)->Instance->BDTR &= ~TIM_BDTR_MOE)
#define __HAL_TIM_MOE_ENABLE(h) ((h)->Instance->BDTR |= TIM_BDTR_MOE)
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *);
#endif
