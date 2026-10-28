/* Execute the actual main USER CODE startup prefix against the real stop
 * implementation. Only SysTick, RC level and interrupt arrival are modeled. */
#define HAL_Delay SamplingModel_Delay
#define main original_sampling_main
#include "test_sampling.c"
#undef main
#undef HAL_Delay
#include "board_diag.h"

static BoardDiagCrash boot_crash;
const BoardDiagCrash *BoardDiag_LastCrash(void) { return &boot_crash; }
static bool boot_wait;
static unsigned elapsed,rc_release,press_at;

void HAL_Delay(uint32_t ms)
{
    if (!boot_wait) { SamplingModel_Delay(ms); return; }
    assert(!host_primask); /* EXTI and SysTick must remain enabled. */
    for (unsigned n=0;n<ms;++n) {
        assert(!(tim1.BDTR & TIM_BDTR_MOE) && !(tim8.BDTR & TIM_BDTR_MOE));
        assert(!(tim1.CCER & 0x555U) && !(tim8.CCER & 0x555U));
        assert(!(tim1.CR1 & TIM_CR1_CEN) && !(tim8.CR1 & TIM_CR1_CEN));
        SamplingModel_Delay(1U); ++elapsed;
        if (elapsed==rc_release) gpio_key.pins|=KEY1_Pin; /* Rising: no falling-edge IRQ. */
        if (elapsed==press_at) { gpio_key.pins&=~KEY1_Pin; key_irq(); }
        if (press_at && elapsed==press_at+1U) gpio_key.pins|=KEY1_Pin;
    }
}

static void startup_check(void)
{
#include "key_boot_prefix.inc"
}

int main(int argc,char **argv)
{
    assert(argc==2);
    bool rc=!strncmp(argv[1],"rc",2);
    bool held=!strncmp(argv[1],"held",4);
    bool wrap=strstr(argv[1],"wrap")!=NULL;
    reset(); tim1.CR1=tim8.CR1=0U; g_boot_stage=3U;
    if (wrap) tick=UINT32_MAX-5U;
    uint32_t started=tick;
    if (rc || held) gpio_key.pins&=~KEY1_Pin;
    if (rc) rc_release=7U; /* Worst nominal 55kohm/100nF -> 0.7VDD ~6.62ms. */
    if (!strcmp(argv[1],"press")) press_at=10U;
    if (!strcmp(argv[1],"cpu")) boot_crash.kind=3U;
    if (!strcmp(argv[1],"pre-edge")) key_irq();
    if (!strcmp(argv[1],"pending")) host_exti_pending|=KEY1_Pin;
    boot_wait=true; startup_check(); boot_wait=false;
    MotorGlobalStopRecord stop; MotorSampling_GetGlobalStop(&stop);
#if KEY_EXPECT_SETTLE
    assert(elapsed>=30U && tick-started>=30U);
    if (rc || !strcmp(argv[1],"released")) {
        assert(!MotorSampling_IsGlobalStopped() && !stop.source && stop.now==1U);
        /* Boot settling does not add run-time stop debounce. */
        assert(MotorSampling_Start()); sample_ms(301,1354);
        assert(MotorSampling_SetOutputEnabled(true));
        assert(MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true));
        sample_ms(2,1354);
        assert(MotorSampling_IsOutputEnabled() && MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_2));
        g_boot_stage=7U;uint32_t before=tick;
        gpio_key.pins&=~KEY1_Pin;key_irq();
        assert(tick==before && !(tim1.CCER & 0x555U) && !(tim8.CCER & 0x555U));
        assert(!(tim1.BDTR & TIM_BDTR_MOE) && !(tim8.BDTR & TIM_BDTR_MOE));
        assert(!MotorSampling_IsOutputRequested() && !MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2));
        MotorSampling_GetGlobalStop(&stop);assert(stop.source==MOTOR_STOP_KEY_EDGE && stop.stage==7U);
        return 0;
    }
    assert(MotorSampling_IsGlobalStopped());
    if (held) assert(stop.source==MOTOR_STOP_KEY_LOW && stop.at-started>=30U && stop.stage==4U && stop.key==0U);
    else if (press_at) assert(stop.source==MOTOR_STOP_KEY_EDGE && stop.at-started==10U && stop.stage==4U && stop.now==1U);
    else if (boot_crash.kind) assert(stop.source==MOTOR_STOP_CPU_BOOT && stop.at==started && stop.stage==4U);
    else if (!strcmp(argv[1],"pre-edge")) assert(stop.source==MOTOR_STOP_KEY_EDGE && stop.at==started && stop.stage==3U);
    else if (!strcmp(argv[1],"pending")) assert(stop.source==MOTOR_STOP_KEY_PENDING && stop.key==3U && stop.now==3U);
    else assert(0);
    /* Nothing after the wait clears a real stop or grants output. */
    assert(!MotorSampling_IsOutputRequested() && !MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2));
    assert(!MotorSampling_ResumeGlobalStop());
#else
    assert(rc && !elapsed && MotorSampling_IsGlobalStopped());
    assert(stop.source==MOTOR_STOP_KEY_LOW && stop.at==started && stop.stage==4U && stop.key==0U);
    gpio_key.pins|=KEY1_Pin;MotorSampling_GetGlobalStop(&stop);
    assert(stop.key==0U && stop.now==1U); /* Exact R102 report: 0/0 Now 1/0. */
#endif
    return 0;
}
