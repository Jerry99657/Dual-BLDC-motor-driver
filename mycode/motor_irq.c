#include "motor_irq.h"
#include "main.h"
#include "motor_sampling.h"
#include "JerryFOC.h"
#include "tim.h"
#include "adc.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_adc.h"

void MotorIRQ_Key1(void)
{
    if (LL_EXTI_IsActiveFlag_0_31(KEY1_Pin)) {
        /* W1C only this line; retain events belonging to other shared lines. */
        LL_EXTI_ClearFlag_0_31(KEY1_Pin);
        MotorSampling_GlobalStopIRQ();
    }
}

void MotorIRQ_Tim2(void)
{
    if (LL_TIM_IsEnabledIT_UPDATE(TIM2) && LL_TIM_IsActiveFlag_UPDATE(TIM2)) {
        LL_TIM_ClearFlag_UPDATE(TIM2); /* rc_w0: preserve unrelated flags. */
        JerryFOC_OuterIRQ();
    }
}

void MotorIRQ_Break(void)
{
    /* SBIF is a system break too. B2 is disabled by this board, but any
     * unexpected B2 event must latch the same safe output state. */
    if (LL_TIM_IsEnabledIT_BRK(TIM1) && (LL_TIM_IsActiveFlag_BRK(TIM1) ||
        LL_TIM_IsActiveFlag_SYSBRK(TIM1) || LL_TIM_IsActiveFlag_BRK2(TIM1))) {
        LL_TIM_ClearFlag_BRK(TIM1);
        LL_TIM_ClearFlag_SYSBRK(TIM1);
        LL_TIM_ClearFlag_BRK2(TIM1);
        MotorSampling_BreakIRQ();
    }
}

/* HAL still owns calibration, starts/stops and regular DMA. Preserve its
 * error/state bits, but do not run the generic software-trigger/queue state
 * machine for this fixed external-trigger injected pair. */
static bool adc_errors(ADC_HandleTypeDef *adc,uint32_t pending,uint32_t allowed)
{
    if (pending & ADC_ISR_OVR) {
        adc->State |= HAL_ADC_STATE_REG_OVR;
        adc->ErrorCode |= HAL_ADC_ERROR_OVR;
        LL_ADC_ClearFlag_OVR(adc->Instance);
    }
    if (pending & ADC_ISR_JQOVF) {
        adc->State |= HAL_ADC_STATE_INJ_JQOVF;
        adc->ErrorCode |= HAL_ADC_ERROR_JQOVF;
        LL_ADC_ClearFlag_JQOVF(adc->Instance);
    }
    uint32_t unexpected=pending & ~(allowed|ADC_ISR_OVR|ADC_ISR_JQOVF);
    if (unexpected) {
        /* No regular EOC/JEOC/AWD interrupts are configured. An unexpected
         * enabled source is a configuration fault, never a partial frame or
         * an unacknowledged interrupt storm. Disabled flags are untouched. */
        CLEAR_BIT(adc->Instance->IER,unexpected);
        LL_ADC_WriteReg(adc->Instance,ISR,unexpected);
        adc->State |= HAL_ADC_STATE_ERROR_INTERNAL;
        adc->ErrorCode |= HAL_ADC_ERROR_INTERNAL;
    }
    return (pending & ~allowed)!=0U;
}

void MotorIRQ_Adc12(void)
{
    uint32_t master=LL_ADC_ReadReg(ADC1,ISR) & LL_ADC_ReadReg(ADC1,IER);
    uint32_t slave=LL_ADC_ReadReg(ADC2,ISR) & LL_ADC_ReadReg(ADC2,IER);
    bool error=adc_errors(&hadc1,master,ADC_ISR_JEOS);
    error |= adc_errors(&hadc2,slave,0U); /* Always inspect both before control. */
    if (error) MotorSampling_AdcErrorIRQ();
    if (master & ADC_ISR_JEOS) {
        if (!(hadc1.State & HAL_ADC_STATE_ERROR_INTERNAL)) hadc1.State |= HAL_ADC_STATE_INJ_EOC;
        /* Acknowledge master before control so a later frame stays pending.
         * Capture must inspect/read/clear ADC2 itself to qualify the pair. */
        LL_ADC_WriteReg(ADC1,ISR,ADC_ISR_JEOS|ADC_ISR_JEOC);
        if (!error) JerryFOC_FastIRQ();
        else LL_ADC_WriteReg(ADC2,ISR,ADC_ISR_JEOS|ADC_ISR_JEOC);
    }
}
