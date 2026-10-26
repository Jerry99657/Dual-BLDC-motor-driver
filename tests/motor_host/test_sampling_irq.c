/* Real sampling/protection + new dispatcher + actual ST LL functions. */
#define main existing_sampling_main
#include "test_sampling.c"
#undef main
#include "motor_irq.h"
TIM_TypeDef tim2;
static unsigned fast_calls;
static bool capture_ok;
void JerryFOC_OuterIRQ(void) {}
void JerryFOC_FastIRQ(void) { ++fast_calls; capture_ok=MotorSampling_Capture(); }
void HostRegisterWrite(uint32_t *reg,uint32_t value)
{
    if (reg==&tim1.SR || reg==&tim2.SR) *reg &= value;
    else if (reg==&adc1.ISR || reg==&adc2.ISR || reg==&host_exti.PR1) *reg &= ~value;
    else assert(0);
}
static void both_off(void)
{
    assert(!MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_1));
    assert(!MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
    assert(!(tim1.CCER & 0x555U));
    assert(!(tim8.BDTR & TIM_BDTR_MOE) || !(tim8.CCER & 0x555U));
}
int main(int argc,char **argv)
{
    assert(argc==2);
    reset(); assert(MotorSampling_Start()); sample_ms(300,1354);
    assert(MotorSampling_IsReady());
    assert(MotorSampling_SetOutputEnabled(true));
    assert(MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true));
    sample_ms(2,1354);
    assert(MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_1));
    assert(MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
    hadc1.State=HAL_ADC_STATE_REG_BUSY|HAL_ADC_STATE_INJ_BUSY;
    hadc2.State=HAL_ADC_STATE_INJ_BUSY;
    adc1.IER=ADC_IER_JEOSIE|ADC_IER_OVRIE;
    frame(0); adc1.ISR=ADC_ISR_JEOS|ADC_ISR_JEOC;
    uint32_t samples=diagnostics().frame_samples;
    if (!strcmp(argv[1],"paired")) {
        MotorIRQ_Adc12();
        assert(fast_calls==1 && capture_ok && diagnostics().frame_samples==samples+1);
        assert(!adc1.ISR && !adc2.ISR && MotorSampling_IsReady());
        assert(hadc1.State==(HAL_ADC_STATE_REG_BUSY|HAL_ADC_STATE_INJ_BUSY|HAL_ADC_STATE_INJ_EOC));
        MotorSampling_Stop(); assert(!master_armed && !slave_armed && !dma);
        assert(MotorSampling_Start()); /* The HAL start/stop lifecycle remains usable. */
    } else if (!strcmp(argv[1],"missing")) {
        adc2.ISR=0; MotorIRQ_Adc12();
        assert(fast_calls==1 && !capture_ok && diagnostics().fault==MOTOR_SAMPLE_FAULT_PAIR);
        both_off();
    } else if (!strcmp(argv[1],"adc-error")) {
        adc2.IER=ADC_IER_OVRIE; adc2.ISR |= ADC_ISR_OVR; MotorIRQ_Adc12();
        assert(!fast_calls && diagnostics().fault==MOTOR_SAMPLE_FAULT_ADC);
        assert(diagnostics().adc2_error==HAL_ADC_ERROR_OVR); both_off();
        frame(0); adc1.ISR=ADC_ISR_JEOS; MotorIRQ_Adc12();
        assert(fast_calls==1 && capture_ok && diagnostics().fault==MOTOR_SAMPLE_FAULT_ADC); both_off();
        assert(diagnostics().frame_samples==samples+1); /* Diagnostic sampling continues. */
    } else if (!strcmp(argv[1],"key")) {
        host_exti_pending=KEY1_Pin|(1U<<12); MotorIRQ_Key1(); both_off();
        assert(host_exti_pending==(1U<<12) && MotorSampling_TakeGlobalStop());
        MotorIRQ_Adc12(); assert(fast_calls==1 && capture_ok);
        both_off(); assert(diagnostics().frame_samples==samples+1);
    } else {
        uint32_t flag=!strcmp(argv[1],"break") ? TIM_SR_BIF :
                      !strcmp(argv[1],"system-break") ? TIM_SR_SBIF : TIM_SR_B2IF;
        tim1.SR=flag|TIM_SR_CC1IF; tim1.BDTR &= ~TIM_BDTR_MOE; /* Hardware trip. */
        MotorIRQ_Break();
        assert(MotorSampling_IsIbusTripped() && diagnostics().ibus_trips==1);
        assert(!(tim1.DIER & TIM_DIER_BIE) && tim1.SR==TIM_SR_CC1IF);
        assert(!MotorSampling_IsOutputRequested() && !(tim1.CCER & 0x555U));
        assert(MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
        MotorIRQ_Adc12(); assert(fast_calls==1); /* M2/measurements remain alive. */
        assert(diagnostics().frame_samples==samples+1);
        tim1.SR |= flag; MotorIRQ_Break(); assert(diagnostics().ibus_trips==1);
        assert(!MotorSampling_IsOutputRequested());
    }
    printf("PASS: real sampling + LL dispatcher integration: %s, output gates/paired data/diagnostics retained\n",argv[1]);
    return 0;
}
