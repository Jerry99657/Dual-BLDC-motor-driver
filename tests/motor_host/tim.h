#ifndef MOTOR_HOST_TIM_H
#define MOTOR_HOST_TIM_H
#define TIM_CCMR1_OC1PE (1U<<3)
#define TIM_CCMR1_OC2PE (1U<<11)
#define TIM_CCMR2_OC3PE (1U<<3)
#include "main.h"
typedef struct { uint32_t CR1, CNT, CCR1, CCR2, CCR3, ARR, CCER, BDTR, EGR, SR, DIER,PSC,CCMR1,CCMR2,CCR4; } TIM_TypeDef;
typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
extern TIM_TypeDef tim1, tim8, tim6;
#define TIM1 (&tim1)
extern TIM_HandleTypeDef htim1, htim8, htim6;
extern TIM_TypeDef tim7;
#define TIM7 (&tim7)
#define TIM7_IRQn 55
#define __HAL_RCC_TIM7_CLK_ENABLE() ((void)0)
static inline void HAL_NVIC_SetPriority(int n,unsigned p,unsigned s) { (void)n;(void)p;(void)s; }
static inline void HAL_NVIC_ClearPendingIRQ(int n) { (void)n; }
static inline void HAL_NVIC_EnableIRQ(int n) { (void)n; }
#define TIM_CR1_OPM (1U<<3)
#define TIM_CR1_URS (1U<<2)
#define TIM_EGR_UG 1U
#ifndef TIM_DIER_UIE
#define TIM_DIER_UIE 1U
#endif
#ifndef TIM_SR_UIF
#define TIM_SR_UIF 1U
#endif
#define TIM_OCMODE_PWM1 0x60U
#define TIM_OCMODE_FORCED_INACTIVE 0x40U
#define TIM_CCMR1_OC1M 0x10070U
#define TIM_CCMR1_OC2M 0x1007000U
#define TIM_CCMR2_OC3M TIM_CCMR1_OC1M
#ifdef MOTOR_IRQ_INTEGRATION
extern TIM_TypeDef tim2;
#define TIM2 (&tim2)
#else
#define TIM2 ((TIM_TypeDef *)2)
#endif
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
#define TIM_FLAG_BREAK (1U << 7)
#define TIM_IT_BREAK (1U << 7)
#ifdef MOTOR_SAMPLING_HOST_HOOKS
uint32_t HostTimFlagRead(TIM_TypeDef *,uint32_t);
void HostMoeWrite(TIM_TypeDef *);
#define __HAL_TIM_GET_FLAG(h,f) HostTimFlagRead((h)->Instance,(f))
#define __HAL_TIM_MOE_ENABLE(h) do { HostMoeWrite((h)->Instance); (h)->Instance->BDTR |= TIM_BDTR_MOE; } while(0)
#else
#define __HAL_TIM_GET_FLAG(h,f) ((h)->Instance->SR & (f))
#define __HAL_TIM_MOE_ENABLE(h) ((h)->Instance->BDTR |= TIM_BDTR_MOE)
#endif
#define __HAL_TIM_CLEAR_FLAG(h,f) ((h)->Instance->SR &= ~(f))
#define __HAL_TIM_ENABLE_IT(h,f) ((h)->Instance->DIER |= (f))
#define __HAL_TIM_DISABLE_IT(h,f) ((h)->Instance->DIER &= ~(f))
/* Same unconditional shutdown behavior as the real STM32 HAL macro. */
#define __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(h) ((h)->Instance->BDTR &= ~TIM_BDTR_MOE)
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *);
#endif
