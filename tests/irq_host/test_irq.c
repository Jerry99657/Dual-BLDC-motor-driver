#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "motor_irq.h"
#include "motor_sampling.h"

EXTI_TypeDef irq_exti;
TIM_TypeDef irq_tim1,irq_tim2;
TIM_HandleTypeDef htim1={&irq_tim1};
HostDwt host_dwt;
ADC_TypeDef irq_adc1,irq_adc2;
ADC_HandleTypeDef hadc1={.Instance=ADC1},hadc2={.Instance=ADC2};
static unsigned key_calls,outer_calls,break_calls,fast_calls,error_calls,pair_errors;
static bool reenter_timer,reenter_adc,fuzzing;
static bool m1_power,m2_power;

void HostRegisterWrite(uint32_t *reg,uint32_t value)
{
    /* Model peripheral semantics, not ordinary RAM assignment. LL functions
     * must write 1 to clear ADC/EXTI, and 0 to clear TIM SR flags. */
    if (reg==&irq_exti.PR1 || reg==&irq_adc1.ISR || reg==&irq_adc2.ISR) *reg &= ~value;
    else if (reg==&irq_tim1.SR || reg==&irq_tim2.SR) *reg &= value;
    else assert(0);
}
void MotorSampling_GlobalStopIRQ(void)
{
    assert(!(irq_exti.PR1 & KEY1_Pin));
    ++key_calls; m1_power=m2_power=false;
}
void MotorSampling_BreakIRQ(void)
{
    assert(!(irq_tim1.SR & (TIM_SR_BIF|TIM_SR_SBIF|TIM_SR_B2IF)));
    ++break_calls; irq_tim1.DIER &= ~TIM_DIER_BIE; m1_power=false;
}
void MotorSampling_AdcErrorIRQ(void)
{
    assert(hadc1.ErrorCode || hadc2.ErrorCode);
    ++error_calls; m1_power=m2_power=false;
}
void JerryFOC_OuterIRQ(void)
{
    assert(!(irq_tim2.SR & TIM_SR_UIF));
    ++outer_calls;
    if (reenter_timer) { reenter_timer=false; irq_tim2.SR |= TIM_SR_UIF; }
}
void JerryFOC_FastIRQ(void)
{
    assert(!(irq_adc1.ISR & (ADC_ISR_JEOS|ADC_ISR_JEOC)));
    ++fast_calls;
    if (fuzzing) return;
    if (!(irq_adc2.ISR & ADC_ISR_JEOS)) { ++pair_errors; m1_power=m2_power=false; return; }
    HostRegisterWrite(&irq_adc2.ISR,ADC_ISR_JEOS|ADC_ISR_JEOC);
    if (reenter_adc) {
        reenter_adc=false;
        irq_adc1.ISR |= ADC_ISR_JEOS|ADC_ISR_JEOC;
        irq_adc2.ISR |= ADC_ISR_JEOS|ADC_ISR_JEOC;
    }
}
static void reset(void)
{
    irq_exti=(EXTI_TypeDef){0}; irq_tim1=irq_tim2=(TIM_TypeDef){0};
    irq_adc1=irq_adc2=(ADC_TypeDef){0};
    hadc1.State=hadc2.State=HAL_ADC_STATE_INJ_BUSY;
    hadc1.ErrorCode=hadc2.ErrorCode=0;
    key_calls=outer_calls=break_calls=fast_calls=error_calls=pair_errors=0;
    reenter_timer=reenter_adc=fuzzing=false; m1_power=m2_power=true;
}
static void check_timing_boundaries(void)
{
    extern unsigned diag_host_outer,diag_host_timings;
    extern uint32_t diag_host_at,diag_host_age;
    reset();unsigned previous=diag_host_timings;
    host_dwt.CYCCNT=123;MotorIRQ_Tim2();MotorIRQ_Adc12();assert(diag_host_timings==previous);
    irq_tim2.DIER=TIM_DIER_UIE;irq_tim2.SR=TIM_SR_UIF;MotorIRQ_Tim2();
    assert(diag_host_timings==previous+1 && diag_host_outer==1 && diag_host_at==123);
    host_dwt.CYCCNT=220;irq_tim1.CNT=345;irq_tim1.ARR=8500;
    irq_adc1.IER=ADC_IER_JEOSIE;irq_adc1.ISR=irq_adc2.ISR=ADC_ISR_JEOS;
    MotorIRQ_Adc12();assert(diag_host_outer==0 && diag_host_at==220 && diag_host_age==345);
    irq_tim1.CR1=TIM_CR1_DIR;irq_tim1.CNT=8100;irq_adc1.ISR=irq_adc2.ISR=ADC_ISR_JEOS;
    MotorIRQ_Adc12();assert(diag_host_age==400);
    puts("PASS: real IRQ timing hooks only on enabled events, entry timestamps before control and both PWM count directions");
}
static void check_exti_timer(void)
{
    reset(); MotorIRQ_Key1(); assert(key_calls==0);
    irq_exti.PR1=KEY1_Pin|(1U<<12); MotorIRQ_Key1();
    assert(key_calls==1 && irq_exti.PR1==(1U<<12) && !m1_power && !m2_power);
    MotorIRQ_Key1(); assert(key_calls==1);
    irq_tim2.SR=TIM_SR_UIF|TIM_SR_CC1IF; MotorIRQ_Tim2(); assert(outer_calls==0);
    irq_tim2.DIER=TIM_DIER_UIE; reenter_timer=true; MotorIRQ_Tim2();
    assert(outer_calls==1 && irq_tim2.SR==(TIM_SR_UIF|TIM_SR_CC1IF));
    MotorIRQ_Tim2(); MotorIRQ_Tim2(); assert(outer_calls==2 && irq_tim2.SR==TIM_SR_CC1IF);
    const uint32_t flags[]={TIM_SR_BIF,TIM_SR_SBIF,TIM_SR_B2IF};
    for (unsigned i=0;i<3;++i) {
        reset(); irq_tim1.SR=flags[i]|TIM_SR_CC1IF; MotorIRQ_Break();
        assert(break_calls==0 && m1_power);
        irq_tim1.DIER=TIM_DIER_BIE; MotorIRQ_Break();
        assert(break_calls==1 && irq_tim1.SR==TIM_SR_CC1IF && !m1_power && m2_power);
        irq_tim1.SR |= flags[i]; MotorIRQ_Break(); assert(break_calls==1);
    }
    puts("PASS: real ST LL EXTI W1C/TIM rc_w0, shared flags, disabled IRQs, delayed/new update, BIF/SBIF/B2IF latching and sustained-fault interrupt suppression");
}
static void check_adc(void)
{
    reset(); irq_adc1.ISR=ADC_ISR_JEOS|ADC_ISR_EOS; MotorIRQ_Adc12();
    assert(fast_calls==0 && irq_adc1.ISR==(ADC_ISR_JEOS|ADC_ISR_EOS));
    irq_adc1.IER=ADC_IER_JEOSIE; MotorIRQ_Adc12();
    assert(fast_calls==1 && pair_errors==1 && irq_adc1.ISR==ADC_ISR_EOS);
    reset(); irq_adc1.IER=ADC_IER_JEOSIE;
    irq_adc1.ISR=irq_adc2.ISR=ADC_ISR_JEOS|ADC_ISR_JEOC|ADC_ISR_EOS;
    reenter_adc=true; MotorIRQ_Adc12();
    assert(fast_calls==1 && irq_adc1.ISR==(ADC_ISR_JEOS|ADC_ISR_JEOC|ADC_ISR_EOS));
    MotorIRQ_Adc12(); assert(fast_calls==2 && !pair_errors && irq_adc1.ISR==ADC_ISR_EOS && irq_adc2.ISR==ADC_ISR_EOS);
    assert(hadc1.State==(HAL_ADC_STATE_INJ_BUSY|HAL_ADC_STATE_INJ_EOC));
    assert(hadc2.State==HAL_ADC_STATE_INJ_BUSY && !error_calls);
    reset(); irq_adc1.IER=ADC_IER_JEOSIE|ADC_IER_OVRIE;
    irq_adc1.ISR=ADC_ISR_JEOS|ADC_ISR_OVR; irq_adc2.ISR=ADC_ISR_JEOS|ADC_ISR_JQOVF;
    irq_adc2.IER=ADC_IER_JQOVFIE;
    MotorIRQ_Adc12();
    assert(!fast_calls && error_calls==1 && !m1_power && !m2_power);
    assert(hadc1.ErrorCode==HAL_ADC_ERROR_OVR && hadc2.ErrorCode==HAL_ADC_ERROR_JQOVF);
    assert(hadc1.State & HAL_ADC_STATE_REG_OVR); assert(hadc2.State & HAL_ADC_STATE_INJ_JQOVF);
    assert(!irq_adc1.ISR && !irq_adc2.ISR);
    MotorIRQ_Adc12(); assert(error_calls==1);
    reset(); irq_adc2.IER=ADC_IER_JEOSIE; irq_adc2.ISR=ADC_ISR_JEOS;
    MotorIRQ_Adc12(); assert(!fast_calls && error_calls==1 && !irq_adc2.IER && !irq_adc2.ISR);
    assert(hadc2.ErrorCode==HAL_ADC_ERROR_INTERNAL);
    puts("PASS: real ST LL ADC master acknowledgement/pair visibility, missing/late pair, next frame pending, HAL busy/error compatibility, errors before control, both-ADC diagnostics and unexpected-source shutdown");
}
static void exhaust_adc_sources(void)
{
    /* Every ISR/IER combination for the 11 implemented ADC status sources.
     * Masked flags cannot trigger callbacks; enabled unsupported sources must
     * leave no IRQ storm, and errors must never execute the fast control. */
    const uint32_t all=(ADC_ISR_JQOVF<<1)-1U;
    for (unsigned axis=0;axis<2;++axis) for (uint32_t flags=0;flags<=all;++flags)
        for (uint32_t enables=0;enables<=all;++enables) {
            reset(); fuzzing=true;
            ADC_TypeDef *adc=axis ? ADC2 : ADC1;
            adc->ISR=flags; adc->IER=enables;
            uint32_t pending=flags & enables;
            bool error=(pending & ~(axis ? 0U : ADC_ISR_JEOS))!=0U;
            MotorIRQ_Adc12();
            assert(error_calls==(unsigned)error);
            assert(fast_calls==(!axis && !error && (pending & ADC_ISR_JEOS) ? 1U : 0U));
            assert(!(adc->ISR & adc->IER));
            uint32_t consumed=pending;
            if (!axis && (pending & ADC_ISR_JEOS)) consumed |= ADC_ISR_JEOC;
            assert(adc->ISR==(flags & ~consumed));
        }
    puts("PASS: 8388608 ADC ISR/IER/axis combinations, no masked-source dispatch, no enabled pending source left unhandled, no fast control on ADC error");
}
int main(void)
{
    check_timing_boundaries();check_exti_timer(); check_adc(); exhaust_adc_sources(); return 0;
}
