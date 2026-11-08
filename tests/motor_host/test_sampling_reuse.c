/* Compare published frames, faults and gates through the real ADC pipeline.
 * Clock-read instrumentation lives only in ignored copies of the fixture. */
#define main legacy_sampling_main
#include "test_sampling.c"
#undef main
extern unsigned test_tick_reads;

int main(void)
{
    for (unsigned wrap=0;wrap<2;++wrap) for (unsigned age=0;age<=6;++age) {
        reset();
        if (wrap) tick=0xfffffff0U;
        assert(MotorSampling_Start());sample_ms(301,1354);
        tick+=age;frame(25);test_tick_reads=0;
        bool captured=MotorSampling_Capture();
        unsigned reads=test_tick_reads;
        MotorSamplingDiagnostics d=diagnostics();
        const MotorSampleFrame *f=MotorSampling_GetFrame();
        printf("VALUE %u %u %d %u %u %u %u %u %u %a %a %a\n",wrap,age,captured,
            d.fault,f->sequence,MotorSampling_IsAxisReady(0),MotorSampling_IsAxisReady(1),
            tim1.CCER,tim1.BDTR,f->m1_current[0],f->m2_current[0],f->m1_voltage[0]);
        printf("EVAL %u %u %u\n",wrap,age,reads);
        if(age<=5) {
            assert(captured && d.fault==MOTOR_SAMPLE_FAULT_NONE);
            /* An earlier successful readiness result must not survive a
             * later stale supply or a newly asserted hardware comparator. */
            assert(MotorSampling_IsAxisReady(0));tick+=6;
            assert(!MotorSampling_IsAxisReady(0));
        } else assert(d.fault==MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT);
    }
    reset();assert(MotorSampling_Start());sample_ms(301,1354);
    irq_on_tick=true;assert(MotorSampling_IsAxisReady(0));
    assert(diagnostics().fault==MOTOR_SAMPLE_FAULT_NONE);
    comparator_high=true;tim1.SR|=TIM_FLAG_BREAK;assert(!MotorSampling_IsAxisReady(0));
    assert(!(tim1.CCER & 0x555U));
    puts("PASS: capture age 0..6ms including timeout equality/wrap, later readiness recheck, ISR publication during clock read, and a new IBUS trip");
    return 0;
}
