#include "motor_sampling.h"
#include "board_diag.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"

#include <string.h>
#include "adc.h"
#include "tim.h"
#include "comp.h"
#include "dac.h"
#include "app_storage.h"
#include "motor_params.h"
#include "motor_sixstep.h"
#include <math.h>

#define OFFSET_SAMPLES 2000U
#define ADC_FULL_SCALE 4095.0f
#define REFERENCE_START_VOLTS 3.26f /* Provisional scale; power waits for a valid block. */
#define REFERENCE_MIN_VOLTS 2.7f   /* Board uses a nominal 3.3 V analog rail. */
#define REFERENCE_MAX_VOLTS 3.6f
#define REFERENCE_AVERAGE_SAMPLES 64U
#define REFERENCE_FILTER_ALPHA 0.125f /* ~0.24 s time constant at 2 kHz / 64. */
#define REFERENCE_TIMEOUT_MS 200U
#define SAMPLE_TIMEOUT_MS 5U
#define BUS_SETTLE_MS 100U
#define BUS_FRAME_DIVIDER 5U
#define CURRENT_SAMPLE_HZ 10000.0f
/* Excess-I^2 budget, NOT a calibrated winding thermal model. Its reference
 * and duration are motor test settings, separate from rated/dq current. */
#define PHASE_CHANNELS (TIM_CCER_CC1E | TIM_CCER_CC1NE | \
                        TIM_CCER_CC2E | TIM_CCER_CC2NE | \
                        TIM_CCER_CC3E | TIM_CCER_CC3NE)

/* Two complete [VBUS, VREFINT] scans; half/full callbacks never split a scan. */
static volatile uint16_t s_bus_dma[4] __attribute__((aligned(4)));
/* Publish a SINGLE aligned float: each ISR snapshots it once and derives both
 * its current and voltage coefficients, so an update cannot tear the pair. */
static volatile float s_adc_volts = REFERENCE_START_VOLTS / ADC_FULL_SCALE;
static volatile bool s_requested[2] = {true, false};
static bool s_monitored[2] = {true, false};
#define s_output_requested s_requested[0]
#define s_output_arm_frames acquisition.arm_frames[0]
#define s_overload_energy acquisition.energy[0]
#define s_fault_current_raw acquisition.trip_raw[0]
#define s_fault_phase acquisition.trip_phase[0]
#define s_fault_during_zero acquisition.zero_fault[0]
static uint16_t s_ibus_limit_ma=APP_IBUS_DEFAULT_MA,s_ibus_dac_code;
static uint8_t s_six_sector=255U;
static uint32_t s_m1_channels=PHASE_CHANNELS,s_six_commuted;

/* Acquisition state is grouped to share base addresses/literal pools.
 * Every original volatile qualifier and reset assignment is preserved. */
static struct {
    MotorSampleFrame frame;
    uint32_t offset_sum[5];
    float offsets[5];
    volatile uint32_t offset_count;
    uint32_t reference_sum;
    uint16_t reference_count, reference_cal;
    volatile uint16_t reference_raw;
    volatile uint32_t reference_at, reference_updates, reference_errors;
    volatile bool reference_valid;
    volatile float bus_voltage;
    volatile uint32_t bus_at, frame_at;
    volatile uint32_t started_at, bus_good_at, bus_samples;
    volatile uint16_t bus_raw;
    volatile bool bus_seen, bus_valid, running, power_enabled;
    volatile bool bus_good_tracking;
    volatile MotorSamplingFault axis_fault[2];
    volatile uint8_t arm_frames[2];
    volatile MotorSamplingFault fault;
    float energy[2];
    uint16_t trip_raw[2][3];
    uint8_t trip_phase[2];
    bool zero_fault[2];
    uint8_t bus_divider;
    volatile bool ibus_started,ibus_latched,ibus_init_failed,ibus_ref_valid;
    volatile uint32_t ibus_trips;
    uint32_t ibus_reference_updates,ibus_low_at,ibus_changed_at;
    bool ibus_low_tracking;
    volatile bool global_stopped,global_stop_pending,key_releasing;
    uint32_t key_released_at;
    MotorGlobalStopRecord stop;
    bool sixstep,six_timer,six_off;
} acquisition;

bool MotorSampling_SixStepTimer(uint32_t us)
{
    if (!acquisition.six_timer) {
        if (!us) return true;
        if (!acquisition.sixstep) return false;
        __HAL_RCC_TIM7_CLK_ENABLE();
        TIM7->CR1=TIM_CR1_OPM|TIM_CR1_URS;
        TIM7->PSC=169U; /* APB1 timer clock stays 170MHz; 1us ticks. */
        HAL_NVIC_SetPriority(TIM7_IRQn,0U,0U);
        HAL_NVIC_ClearPendingIRQ(TIM7_IRQn);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
        acquisition.six_timer=true;
    }
    TIM7->CR1&=~TIM_CR1_CEN;
    TIM7->DIER=0U; TIM7->SR=0U;
    if (!us) return true;
    if (us>260000U || !acquisition.sixstep || !s_requested[0] || MotorSampling_IsGlobalStopped()) return false;
    /* Slow rev-up needs >65ms steps; retain 1us resolution once faster. */
    uint32_t scale=us>65000U ? 4U : 1U;
    TIM7->PSC=170U*scale-1U;
    TIM7->ARR=(us+scale-1U)/scale-1U; TIM7->CNT=0U; TIM7->EGR=TIM_EGR_UG;
    TIM7->SR=0U; TIM7->DIER=TIM_DIER_UIE; TIM7->CR1|=TIM_CR1_CEN;
    return true;
}
void TIM7_IRQHandler(void)
{
    if (!(TIM7->SR & TIM_SR_UIF)) return;
    TIM7->SR=0U; TIM7->DIER=0U;
    SixStep_OnTimer();
}
bool MotorSampling_SelectSixStep(bool enable)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=!s_requested[0] && !(htim1.Instance->CCER & PHASE_CHANNELS);
    if (ok) {
        (void)MotorSampling_SixStepTimer(0U);
        acquisition.sixstep=enable; s_six_sector=255U;
        acquisition.six_off=false; htim1.Instance->CCR4=1U;
        s_m1_channels=PHASE_CHANNELS;
        MODIFY_REG(htim1.Instance->CCMR1,TIM_CCMR1_OC1M|TIM_CCMR1_OC2M,
                   TIM_OCMODE_PWM1|(TIM_OCMODE_PWM1<<8U));
        MODIFY_REG(htim1.Instance->CCMR2,TIM_CCMR2_OC3M,TIM_OCMODE_PWM1);
    }
    __set_PRIMASK(mask); return ok;
}
bool MotorSampling_SixStepPattern(uint8_t sector,float duty)
{
    if (!acquisition.sixstep || sector>6U || !isfinite(duty) || duty<0.0f || duty>0.85f) return false;
    static const uint8_t hs[]={0,0,1,1,2,2},ls[]={1,2,2,0,0,1};
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (MotorSampling_IsGlobalStopped() || !MotorSampling_IsReady()) { __set_PRIMASK(mask); return false; }
    bool armed=s_requested[0] && !acquisition.arm_frames[0] && (htim1.Instance->CCER & PHASE_CHANNELS);
    /* At low duty the OFF plateau is wider and farther from its PWM edge.
     * CH4 remains PWM1: CCR4=ARR-1 gives its rising edge just after the peak;
     * CCR4=1 gives it near the valley. Hysteresis prevents mode chatter.
     * Sine/FOC/HSO retain their original valley trigger. */
    acquisition.six_off=sector<6U && (duty<0.25f || (acquisition.six_off && duty<0.30f));
    htim1.Instance->CCR4=acquisition.six_off ? htim1.Instance->ARR-1U : 1U;
    uint32_t compare=(uint32_t)(duty*(float)htim1.Instance->ARR);
    htim1.Instance->CCR1=htim1.Instance->CCR2=htim1.Instance->CCR3=compare;
    if (sector!=s_six_sector) {
        /* Disconnect before changing modes; no CCPC shadow can delay trips.
         * OSSR=1 keeps disabled N output inactive while its main E is set:
         * float = forced inactive + E only => both HIN/LIN driven LOW. */
        CLEAR_BIT(htim1.Instance->CCER,PHASE_CHANNELS);
        if(s_six_sector==6U) {
            /* Sine CCRs were unequal and near 50%. Do not energize a newly
             * selected high side with that old width until the next UEV.
             * Load equal six-step compares immediately while disconnected;
             * restore preload without UG/resetting the shared ADC timebase. */
            uint32_t pe1=htim1.Instance->CCMR1 & (TIM_CCMR1_OC1PE|TIM_CCMR1_OC2PE);
            uint32_t pe2=htim1.Instance->CCMR2 & TIM_CCMR2_OC3PE;
            CLEAR_BIT(htim1.Instance->CCMR1,TIM_CCMR1_OC1PE|TIM_CCMR1_OC2PE);
            CLEAR_BIT(htim1.Instance->CCMR2,TIM_CCMR2_OC3PE);
            htim1.Instance->CCR1=htim1.Instance->CCR2=htim1.Instance->CCR3=compare;
            SET_BIT(htim1.Instance->CCMR1,pe1); SET_BIT(htim1.Instance->CCMR2,pe2);
        }
        unsigned h=sector<6U ? hs[sector] : 3U;
        MODIFY_REG(htim1.Instance->CCMR1,TIM_CCMR1_OC1M|TIM_CCMR1_OC2M,
            (h==0U || h==3U ? TIM_OCMODE_PWM1 : TIM_OCMODE_FORCED_INACTIVE) |
            ((h==1U || h==3U ? TIM_OCMODE_PWM1 : TIM_OCMODE_FORCED_INACTIVE)<<8U));
        MODIFY_REG(htim1.Instance->CCMR2,TIM_CCMR2_OC3M,
            h==2U || h==3U ? TIM_OCMODE_PWM1 : TIM_OCMODE_FORCED_INACTIVE);
        /* PWM-phase low side conducts during OFF time, with TIM1 deadtime.
         * This gives deterministic bootstrap refresh/freewheel instead of
         * relying on its body diode. The FLOAT phase still has both gates off. */
        s_m1_channels=h==3U ? PHASE_CHANNELS : TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|
                       (TIM_CCER_CC1NE<<(4U*ls[sector]))|(TIM_CCER_CC1NE<<(4U*h));
        s_six_sector=sector; s_six_commuted=DWT->CYCCNT;
        if (armed && !MotorSampling_IsGlobalStopped() && MotorSampling_IsReady() && s_requested[0])
            SET_BIT(htim1.Instance->CCER,s_m1_channels);
    }
    __set_PRIMASK(mask); return true;
}

static uint8_t stop_key_state(void)
{
    return (LL_GPIO_IsInputPinSet(KEY1_GPIO_Port,KEY1_Pin) ? 1U : 0U) |
        (LL_EXTI_IsActiveFlag_0_31(KEY1_Pin) ? 2U : 0U);
}

void MotorSampling_GlobalStopFromIRQ(MotorGlobalStopSource source)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool first=!acquisition.global_stopped;
    acquisition.global_stopped=acquisition.global_stop_pending=true;
    acquisition.key_releasing=false;
    (void)MotorSampling_SixStepTimer(0U);
    /* EXTI is enabled before MX_TIMx_Init: handles can still be NULL. Keep
     * counters/CH4 alive once initialized, so ADC/diagnostics keep running. */
    TIM_HandleTypeDef *timers[]={&htim1,&htim8};
    for (unsigned a=0;a<MOTOR_AXIS_COUNT;++a) {
        if (timers[a]->Instance) {
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timers[a]);
            CLEAR_BIT(timers[a]->Instance->CCER,PHASE_CHANNELS);
        }
        s_requested[a]=false; acquisition.arm_frames[a]=0U;
    }
    /* Gates first. No SPI, formatting or retained-fault getters in the IRQ.
     * Repeated polling/bounce must not replace the original stop cause. */
    if (first) {
        extern volatile uint32_t g_boot_stage;
        acquisition.stop=(MotorGlobalStopRecord){.at=HAL_GetTick(),.stage=g_boot_stage,
            .source=(uint8_t)source,.key=stop_key_state()};
    }
    __set_PRIMASK(mask);
}

void MotorSampling_GlobalStopIRQ(void)
{
    MotorSampling_GlobalStopFromIRQ(MOTOR_STOP_CPU_BOOT);
}

void MotorSampling_GetGlobalStop(MotorGlobalStopRecord *out)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=acquisition.stop; out->now=stop_key_state();
    __set_PRIMASK(mask);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin==KEY1_Pin) MotorSampling_GlobalStopFromIRQ(MOTOR_STOP_KEY_EDGE);
}

bool MotorSampling_IsGlobalStopped(void)
{
    /* Read the pending edge too: a short press can finish while interrupts
     * are masked. Never let a later MOE/CCER write overtake that event. */
    unsigned key=stop_key_state();
    if ((key!=1U) && (!acquisition.global_stopped || acquisition.key_releasing))
        MotorSampling_GlobalStopFromIRQ((key&1U) ? MOTOR_STOP_KEY_PENDING : MOTOR_STOP_KEY_LOW);
    return acquisition.global_stopped;
}

bool MotorSampling_TakeGlobalStop(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    (void)MotorSampling_IsGlobalStopped();
    bool pending=acquisition.global_stop_pending;
    acquisition.global_stop_pending=false;
    __set_PRIMASK(mask);
    return pending;
}

bool MotorSampling_ResumeGlobalStop(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=!MotorSampling_IsGlobalStopped();
    if (!ok && !acquisition.global_stop_pending && acquisition.key_releasing &&
        HAL_GetTick()-acquisition.key_released_at>=30U) {
        acquisition.global_stopped=false; ok=true;
    }
    __set_PRIMASK(mask);
    return ok;
}

static void global_stop_service(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (MotorSampling_IsGlobalStopped() && !acquisition.key_releasing &&
        LL_GPIO_IsInputPinSet(KEY1_GPIO_Port,KEY1_Pin) &&
        !LL_EXTI_IsActiveFlag_0_31(KEY1_Pin)) {
        acquisition.key_released_at=HAL_GetTick(); acquisition.key_releasing=true;
    }
    __set_PRIMASK(mask);
}

static void ibus_latch(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_BREAK); /* Sustained high must not cause an IRQ storm. */
    CLEAR_BIT(htim1.Instance->CCER,PHASE_CHANNELS);
    s_requested[0]=false; acquisition.arm_frames[0]=0;
    if (!acquisition.ibus_latched) { ++acquisition.ibus_trips; acquisition.ibus_low_tracking=false; }
    acquisition.ibus_latched=true;
    (void)MotorSampling_SixStepTimer(0U);
    if (acquisition.axis_fault[0]==MOTOR_SAMPLE_FAULT_NONE) acquisition.axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS;
    BoardDiag_Fault(1U,DIAG_SAMPLE,"M1 IBUS TRIP");
    __set_PRIMASK(mask);
}

/* BIF is a sticky hardware event, including a pulse that has already ended.
 * Poll at every software output gate as well as in the Break callback: ADC
 * and BRK share priority 0, so the ADC ISR may run before the BRK handler. */
static void ibus_poll(void)
{
    if (acquisition.ibus_started && __HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_BREAK)) ibus_latch();
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==htim1.Instance) MotorSampling_BreakIRQ();
}

void MotorSampling_BreakIRQ(void) { ibus_latch(); }

static bool ibus_write_threshold(float reference)
{
    float threshold=reference*0.5f+(float)s_ibus_limit_ma*0.00025f;
    if (!isfinite(reference) || reference<REFERENCE_MIN_VOLTS || reference>REFERENCE_MAX_VOLTS ||
        threshold>reference-0.25f) return false;
    uint16_t code=(uint16_t)(4096.0f*threshold/reference+0.5f);
    if (HAL_DAC_SetValue(&hdac3,DAC_CHANNEL_2,DAC_ALIGN_12B_R,code)!=HAL_OK) return false;
    s_ibus_dac_code=code;
    return true;
}

static void ibus_init(void)
{
    if (acquisition.ibus_started || acquisition.ibus_init_failed) return;
    /* All six phase outputs are disconnected by Start before reaching here.
     * Load a positive threshold before starting the comparator. */
    if (!ibus_write_threshold(REFERENCE_START_VOLTS) ||
        HAL_DAC_Start(&hdac3,DAC_CHANNEL_2)!=HAL_OK) goto failed;
    HAL_Delay(1U); /* Startup only, before ADC/TIM  */
    __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_BREAK);
    if (HAL_COMP_Start(&hcomp4)!=HAL_OK) goto failed;
    /* HAL waits for the comparator's specified startup delay. Its output
     * before that point is not a qualified current indication. All power
     * CCER bits/MOE are still OFF, so discard only this startup flag. */
    acquisition.ibus_started=true;
    if (HAL_COMP_GetOutputLevel(&hcomp4)==COMP_OUTPUT_LEVEL_HIGH) ibus_latch();
    else {
        __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_BREAK);
        __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_BREAK);
    }
    ibus_poll();
    return;
failed:
    acquisition.ibus_init_failed=true;
    __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_BREAK);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    s_requested[0]=false; acquisition.arm_frames[0]=0;
    if (acquisition.axis_fault[0]==MOTOR_SAMPLE_FAULT_NONE) acquisition.axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS_INIT;
    BoardDiag_Fault(1U,DIAG_SAMPLE,"M1 IBUS INIT");
}

uint16_t MotorSampling_GetIbusLimit(void) { return s_ibus_limit_ma; }
bool MotorSampling_IsIbusTripped(void) { ibus_poll(); return acquisition.ibus_latched; }
bool MotorSampling_SetIbusLimit(uint16_t ma)
{
    if (ma<APP_IBUS_MIN_MA || ma>APP_IBUS_MAX_MA || ma%100U) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=(!s_requested[0] || (!acquisition.running && !acquisition.ibus_started)) &&
        !(htim1.Instance->CCER & PHASE_CHANNELS) && !acquisition.ibus_init_failed;
    if (ok) {
        uint16_t previous=s_ibus_limit_ma;
        s_ibus_limit_ma=ma;
        acquisition.ibus_low_tracking=false; acquisition.ibus_ref_valid=false; acquisition.ibus_changed_at=HAL_GetTick();
        if (acquisition.ibus_started && !ibus_write_threshold(s_adc_volts*ADC_FULL_SCALE)) {
            s_ibus_limit_ma=previous; acquisition.ibus_init_failed=true;
            if (acquisition.axis_fault[0]==MOTOR_SAMPLE_FAULT_NONE) acquisition.axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS_INIT;
            BoardDiag_Fault(1U,DIAG_SAMPLE,"M1 IBUS INIT");
            ok=false;
        }
        ibus_poll();
    }
    __set_PRIMASK(mask);
    return ok;
}

static void trigger_bus_after_injected(void)
{
    if (++acquisition.bus_divider < BUS_FRAME_DIVIDER) return;
    acquisition.bus_divider = 0U;
    /* ES0431 2.5.10: a regular conversion interrupted by an injected trigger
     * can make the first injected result (M1 IU) zero on affected silicon.
     * TIM6 never free-runs. Issue its UPDATE/TRGO only after both injected
     * sequences finish, every fifth frame. Accept only the first 25us after
     * the selected PWM center: valley/up for ON, peak/down for six-step OFF.
     * Steady operation leaves >75us; a center change can shorten the next
     * injected interval to 50us, still leaving >25us for the ~16.78us regular
     * sequence. A requested/active-center mismatch skips this regular scan.
     * Skip a late ISR rather than start a conversion near the next trigger.
     * This also runs during calibration, at 0 V and after a latched trip. */
    uint32_t direction = htim1.Instance->CR1 & TIM_CR1_DIR;
    uint32_t counter = htim1.Instance->CNT;
    bool off=acquisition.sixstep && acquisition.six_off;
    if ((off && direction && counter>=htim1.Instance->ARR/2U) ||
        (!off && !direction && counter<=htim1.Instance->ARR/2U))
        htim6.Instance->EGR = TIM_EVENTSOURCE_UPDATE;
}

static void trip(MotorSamplingFault fault)
{
    /* ISR 内只撤掉输出，不调用可能等待转换结束的 HAL Stop。 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* Keep CH4/OC4REF alive for ADC triggers and diagnostics. Disconnect all
     * six power outputs, as during zero calibration; do not stop  */
    CLEAR_BIT(htim1.Instance->CCER, PHASE_CHANNELS);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    acquisition.power_enabled = false;
    if (acquisition.fault == MOTOR_SAMPLE_FAULT_NONE) acquisition.fault = fault;
    BoardDiag_Fault(0U,DIAG_SAMPLE,MotorSampling_GetStatusText());
    __set_PRIMASK(primask);
}

static TIM_HandleTypeDef *axis_timer(MotorAxis axis)
{
    return axis == MOTOR_AXIS_1 ? &htim1 : &htim8;
}

static void trip_axis(MotorAxis axis, MotorSamplingFault fault, unsigned phase, bool zero)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (acquisition.axis_fault[axis] == MOTOR_SAMPLE_FAULT_NONE) {
        for (unsigned i=0; i<3; ++i)
            acquisition.trip_raw[axis][i] = axis == MOTOR_AXIS_1 ? acquisition.frame.m1_current_raw[i] :
                                  (i<2 ? acquisition.frame.m2_current_raw[i] : 0U);
        acquisition.trip_phase[axis] = (uint8_t)phase;
        acquisition.zero_fault[axis] = zero;
        acquisition.axis_fault[axis] = fault;
    }
    CLEAR_BIT(axis_timer(axis)->Instance->CCER, PHASE_CHANNELS);
    if (axis==MOTOR_AXIS_2) __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    acquisition.arm_frames[axis] = 0U;
    /* An absent, unused M2 front end must not consume the first event slot.
     * Its readiness fault remains intact and still prevents enabling M2. */
    if (s_monitored[axis]) BoardDiag_Fault((unsigned)axis+1U,DIAG_SAMPLE,MotorSampling_GetAxisStatusText(axis));
    __set_PRIMASK(mask);
}

static bool frame_fresh(void)
{
    /* Read the ISR-owned timestamp BEFORE the clock. Reversing this order
     * can underflow if an IRQ publishes a newer timestamp during GetTick. */
    uint32_t at = acquisition.frame_at;
    return HAL_GetTick() - at <= SAMPLE_TIMEOUT_MS;
}

static bool bus_fresh(void)
{
    bool seen = acquisition.bus_seen;
    uint32_t at = acquisition.bus_at;
    return seen && HAL_GetTick() - at <= SAMPLE_TIMEOUT_MS;
}

static bool reference_fresh(void)
{
    bool valid = acquisition.reference_valid;
    uint32_t at = acquisition.reference_at;
    return valid && HAL_GetTick() - at <= REFERENCE_TIMEOUT_MS;
}

static void ibus_service(void)
{
    ibus_poll();
    if (!acquisition.ibus_started || acquisition.ibus_init_failed) return;
    if (reference_fresh() && (!acquisition.ibus_ref_valid || acquisition.ibus_reference_updates!=acquisition.reference_updates)) {
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if (ibus_write_threshold(s_adc_volts*ADC_FULL_SCALE)) {
            acquisition.ibus_reference_updates=acquisition.reference_updates;
            acquisition.ibus_ref_valid=HAL_GetTick()-acquisition.ibus_changed_at>=2U;
        } else {
            acquisition.ibus_init_failed=true;
            CLEAR_BIT(htim1.Instance->CCER,PHASE_CHANNELS);
            s_requested[0]=false; acquisition.arm_frames[0]=0;
            if (acquisition.axis_fault[0]==MOTOR_SAMPLE_FAULT_NONE) acquisition.axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS_INIT;
            BoardDiag_Fault(1U,DIAG_SAMPLE,"M1 IBUS INIT");
        }
        ibus_poll();
        __set_PRIMASK(mask);
    }
    bool pulse=__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_BREAK)!=0U;
    if (acquisition.ibus_latched && pulse) {
        /* Acknowledge pulses only AFTER latching/disconnecting. This does
         * not clear the software fault or re-enable MOE/Break interrupts. */
        __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_BREAK);
        acquisition.ibus_low_tracking=false;
    }
    if (HAL_COMP_GetOutputLevel(&hcomp4)==COMP_OUTPUT_LEVEL_HIGH) {
        ibus_latch(); acquisition.ibus_low_tracking=false;
    } else if (!acquisition.ibus_low_tracking) {
        acquisition.ibus_low_at=HAL_GetTick(); acquisition.ibus_low_tracking=true;
    }
}

bool MotorSampling_ClearIbusTrip(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=acquisition.ibus_latched && acquisition.ibus_started && !acquisition.ibus_init_failed && acquisition.ibus_ref_valid &&
        !s_requested[0] && !(htim1.Instance->CCER & PHASE_CHANNELS) &&
        acquisition.fault==MOTOR_SAMPLE_FAULT_NONE && acquisition.axis_fault[0]==MOTOR_SAMPLE_FAULT_IBUS &&
        acquisition.running && reference_fresh() && frame_fresh() && bus_fresh() && acquisition.bus_valid &&
        acquisition.ibus_low_tracking && HAL_GetTick()-acquisition.ibus_low_at>=100U &&
        !__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_BREAK) &&
        HAL_COMP_GetOutputLevel(&hcomp4)==COMP_OUTPUT_LEVEL_LOW;
    if (ok) {
        /* Service acknowledged older pulses while the output was OFF.
         * Never clear BIF here: a NEW pulse arriving after the quiet check
         * must remain pending and immediately re-latch the protection. */
        acquisition.ibus_latched=false; acquisition.axis_fault[0]=MOTOR_SAMPLE_FAULT_NONE;
        __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_BREAK);
        /* Leave MOE, CCER, requests and preload countdown OFF. Only a new
         * explicit control command can arm M1 after this manual clear. */
        ibus_poll();
        ok=!acquisition.ibus_latched;
    }
    __set_PRIMASK(mask);
    return ok;
}

/* MOE is written while all power channels are disconnected, then BIF is
 * checked AGAIN before enabling CCER. A pulse between the first check and
 * the MOE write therefore cannot energize a phase. A later pulse clears MOE
 * in hardware, with no subsequent MOE write in this transaction. */
static void arm_axis(MotorAxis axis)
{
    if (MotorSampling_IsGlobalStopped()) return;
    if (axis==MOTOR_AXIS_1) {
        ibus_poll();
        if (!acquisition.ibus_started || !acquisition.ibus_ref_valid || acquisition.ibus_latched || acquisition.ibus_init_failed) return;
    }
    if (!s_requested[axis] || acquisition.fault!=MOTOR_SAMPLE_FAULT_NONE ||
        acquisition.axis_fault[axis]!=MOTOR_SAMPLE_FAULT_NONE) return;
    CLEAR_BIT(axis_timer(axis)->Instance->CCER,PHASE_CHANNELS);
    __HAL_TIM_MOE_ENABLE(axis_timer(axis));
    if (axis==MOTOR_AXIS_1) ibus_poll();
    if (!MotorSampling_IsGlobalStopped() && s_requested[axis] && acquisition.axis_fault[axis]==MOTOR_SAMPLE_FAULT_NONE)
        SET_BIT(axis_timer(axis)->Instance->CCER,axis==MOTOR_AXIS_1 ? s_m1_channels : PHASE_CHANNELS);
}

bool MotorSampling_IsAxisReady(MotorAxis axis)
{
    ibus_poll();
    return (unsigned)axis < MOTOR_AXIS_COUNT && acquisition.running && acquisition.power_enabled &&
           (axis!=MOTOR_AXIS_1 || (acquisition.ibus_started && acquisition.ibus_ref_valid && !acquisition.ibus_latched && !acquisition.ibus_init_failed)) &&
           acquisition.fault == MOTOR_SAMPLE_FAULT_NONE && acquisition.axis_fault[axis] == MOTOR_SAMPLE_FAULT_NONE &&
           acquisition.bus_valid && bus_fresh() && frame_fresh() && reference_fresh();
}

bool MotorSampling_IsReady(void) { return MotorSampling_IsAxisReady(MOTOR_AXIS_1); }
bool MotorSampling_IsCalibrated(void) { return acquisition.offset_count >= OFFSET_SAMPLES; }
bool MotorSampling_IsAxisOutputRequested(MotorAxis axis)
{
    return (unsigned)axis < MOTOR_AXIS_COUNT && s_requested[axis];
}
bool MotorSampling_IsOutputRequested(void) { return s_requested[0]; }
bool MotorSampling_IsAxisOutputEnabled(MotorAxis axis)
{
    return MotorSampling_IsAxisReady(axis) &&
           (axis_timer(axis)->Instance->CCER & PHASE_CHANNELS) == (axis==MOTOR_AXIS_1 ? s_m1_channels : PHASE_CHANNELS) &&
           (axis_timer(axis)->Instance->BDTR & TIM_BDTR_MOE) != 0U;
}
bool MotorSampling_IsOutputEnabled(void) { return MotorSampling_IsAxisOutputEnabled(MOTOR_AXIS_1); }

bool MotorSampling_SetAxisOutputEnabled(MotorAxis axis, bool enable)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT) return false;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool ok = true;
    if (!enable) {
        if (axis==MOTOR_AXIS_1) (void)MotorSampling_SixStepTimer(0U);
        s_requested[axis] = false;
        acquisition.arm_frames[axis] = 0U;
        CLEAR_BIT(axis_timer(axis)->Instance->CCER, PHASE_CHANNELS);
        if (axis==MOTOR_AXIS_2) __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    } else if (MotorSampling_IsGlobalStopped() || !MotorSampling_IsAxisReady(axis) || !MotorSampling_IsCalibrated()) {
        ok = false;
    } else if (!s_requested[axis]) {
        s_requested[axis] = true;
        s_monitored[axis] = true;
        acquisition.arm_frames[axis] = 2U;
    }
    __set_PRIMASK(mask);
    return ok;
}
bool MotorSampling_SetOutputEnabled(bool enable)
{
    return MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_1, enable);
}

float MotorSampling_GetBusVoltage(void)
{
    return bus_fresh() ? acquisition.bus_voltage : 0.0f;
}
bool MotorSampling_IsBusFresh(void) { return bus_fresh(); }

void MotorSampling_GetAxisDiagnostics(MotorAxis axis, MotorSamplingDiagnostics *out)
{
    if (out == NULL || (unsigned)axis >= MOTOR_AXIS_COUNT) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->fault = acquisition.fault != MOTOR_SAMPLE_FAULT_NONE ? acquisition.fault : acquisition.axis_fault[axis];
    out->bus_raw = acquisition.bus_raw;
    out->bus_samples = acquisition.bus_samples;
    out->frame_samples = acquisition.frame.sequence;
    out->adc1_error = hadc1.ErrorCode;
    out->adc2_error = hadc2.ErrorCode;
    out->bus_voltage = acquisition.bus_voltage;
    out->power_enabled = acquisition.power_enabled && out->fault == MOTOR_SAMPLE_FAULT_NONE;
    for (unsigned i = 0U; i < 3U; ++i) {
        out->current_raw[i] = axis == MOTOR_AXIS_1 ? acquisition.frame.m1_current_raw[i] :
                              (i<2 ? acquisition.frame.m2_current_raw[i] : 0U);
        out->current_zero[i] = axis == MOTOR_AXIS_1 ? (uint16_t)(acquisition.offsets[i]+0.5f) :
                               (i<2 ? (uint16_t)(acquisition.offsets[i+3]+0.5f) : 0U);
        out->fault_current_raw[i] = acquisition.trip_raw[axis][i];
    }
    out->fault_phase = acquisition.trip_phase[axis];
    out->fault_during_zero = acquisition.zero_fault[axis];
    out->silicon_revision = (uint16_t)HAL_GetREVID();
    out->reference_voltage = s_adc_volts * ADC_FULL_SCALE;
    out->reference_raw = acquisition.reference_raw;
    out->reference_cal = acquisition.reference_cal;
    out->reference_updates = acquisition.reference_updates;
    out->reference_errors = acquisition.reference_errors;
    out->ibus_limit_ma=s_ibus_limit_ma;
    out->ibus_dac_code=s_ibus_dac_code;
    out->ibus_trips=acquisition.ibus_trips;
    out->ibus_latched=acquisition.ibus_latched;
    out->ibus_ready=acquisition.ibus_started && acquisition.ibus_ref_valid && !acquisition.ibus_init_failed;
    out->ibus_high=acquisition.ibus_started && HAL_COMP_GetOutputLevel(&hcomp4)==COMP_OUTPUT_LEVEL_HIGH;
    bool reference_valid = acquisition.reference_valid;
    uint32_t reference_at = acquisition.reference_at;
    bool seen = acquisition.bus_seen;
    uint32_t bus_at = acquisition.bus_at, frame_at = acquisition.frame_at;
    __set_PRIMASK(primask);
    uint32_t now = HAL_GetTick();
    out->bus_age_ms = seen ? now - bus_at : UINT32_MAX;
    out->frame_age_ms = now - frame_at;
    out->bus_fresh = seen && out->bus_age_ms <= SAMPLE_TIMEOUT_MS;
    out->reference_age_ms = reference_valid ? now - reference_at : UINT32_MAX;
    out->reference_fresh = reference_valid && out->reference_age_ms <= REFERENCE_TIMEOUT_MS;
    out->ibus_threshold_v=(float)out->ibus_dac_code*out->reference_voltage/4096.0f;
    out->ibus_ready=out->ibus_ready && out->reference_fresh;
}

void MotorSampling_GetDiagnostics(MotorSamplingDiagnostics *out)
{
    MotorSampling_GetAxisDiagnostics(MOTOR_AXIS_1, out);
}
const char *MotorSampling_GetStatusText(void)
{
    return MotorSampling_GetAxisStatusText(MOTOR_AXIS_1);
}
const char *MotorSampling_GetAxisStatusText(MotorAxis axis)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT) return "INVALID AXIS";
    MotorSamplingFault fault = acquisition.fault != MOTOR_SAMPLE_FAULT_NONE ? acquisition.fault : acquisition.axis_fault[axis];
    if (axis == MOTOR_AXIS_2) {
        if (fault == MOTOR_SAMPLE_FAULT_OVERCURRENT) return "M2 OVERCURRENT";
        if (fault == MOTOR_SAMPLE_FAULT_OVERLOAD) return "M2 OVERLOAD";
        if (fault == MOTOR_SAMPLE_FAULT_CURRENT_RANGE)
            return acquisition.zero_fault[axis] ? "M2 ZERO ADC" : "M2 CURRENT ADC";
    }
    switch (fault) {
    case MOTOR_SAMPLE_FAULT_INIT:          return "INIT ERROR";
    case MOTOR_SAMPLE_FAULT_ADC:           return "ADC ERROR";
    case MOTOR_SAMPLE_FAULT_PAIR:          return "ADC PAIR";
    case MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT: return "FRAME TIMEOUT";
    case MOTOR_SAMPLE_FAULT_BUS_TIMEOUT:   return "BUS TIMEOUT";
    case MOTOR_SAMPLE_FAULT_BUS_LOST:      return "BUS LOST";
    case MOTOR_SAMPLE_FAULT_BUS_RANGE:     return "BUS RANGE";
    case MOTOR_SAMPLE_FAULT_OVERCURRENT:   return "M1 OVERCURRENT";
    case MOTOR_SAMPLE_FAULT_CURRENT_RANGE: {
        static const char *const zero[] = {"M1 ZERO", "M1 ZERO U", "M1 ZERO V", "M1 ZERO W"};
        static const char *const run[] = {"M1 CURRENT ADC", "M1 ADC U", "M1 ADC V", "M1 ADC W"};
        return s_fault_during_zero ? zero[s_fault_phase] : run[s_fault_phase];
    }
    case MOTOR_SAMPLE_FAULT_OVERLOAD:      return "M1 OVERLOAD";
    case MOTOR_SAMPLE_FAULT_REFERENCE:     return "ADC REF";
    case MOTOR_SAMPLE_FAULT_IBUS:          return "M1 IBUS TRIP";
    case MOTOR_SAMPLE_FAULT_IBUS_INIT:     return "M1 IBUS INIT";
    default: break;
    }
    if (!acquisition.running) return "STOPPED";
    if (!bus_fresh()) return "WAIT ADC";
    if (!acquisition.bus_valid) return acquisition.bus_raw >= 4095U ? "BUS RANGE" : "WAIT BUS";
    if (!reference_fresh()) return "WAIT VREF";
    if (axis==MOTOR_AXIS_1 && !acquisition.ibus_ref_valid) return "WAIT IBUS";
    if (!MotorSampling_IsCalibrated()) return "ZERO CAL";
    if (!acquisition.power_enabled) return "BUS SETTLE";
    return "READY";
}

static void publish_reference(uint16_t raw)
{
    acquisition.reference_raw = raw;
    /* Avoid division by zero and stop isolated invalid/rail readings from
     * becoming a conversion coefficient. Reject the partial average too. */
    float numerator = (float)acquisition.reference_cal * ((float)VREFINT_CAL_VREF * 0.001f);
    if (raw == 0U || raw >= 4095U ||
        numerator < REFERENCE_MIN_VOLTS * (float)raw ||
        numerator > REFERENCE_MAX_VOLTS * (float)raw) {
        ++acquisition.reference_errors;
        acquisition.reference_sum = 0U;
        acquisition.reference_count = 0U;
        return;
    }
    acquisition.reference_sum += raw;
    if (++acquisition.reference_count < REFERENCE_AVERAGE_SAMPLES) return;
    float estimate = numerator * (float)REFERENCE_AVERAGE_SAMPLES / (float)acquisition.reference_sum;
    float filtered = acquisition.reference_valid ? s_adc_volts * ADC_FULL_SCALE : estimate;
    filtered += REFERENCE_FILTER_ALPHA * (estimate - filtered);
    /* The first complete block seeds the filter, never a hardcoded 3.3 V.
     * Motor power is inhibited until a measured reference is available. */
    s_adc_volts = filtered / ADC_FULL_SCALE;
    acquisition.reference_at = HAL_GetTick();
    acquisition.reference_valid = true;
    ++acquisition.reference_updates;
    acquisition.reference_sum = 0U;
    acquisition.reference_count = 0U;
}

static void publish_regular(const volatile uint16_t *scan)
{
    uint16_t raw = scan[0], reference_raw = scan[1];
    publish_reference(reference_raw);
    float voltage = (float)raw * (s_adc_volts * 11.0f);
    /* 这是测量有效性判断，不替代硬件过压/过流保护。 */
    /* DMA IRQ can be preempted by ADC JEOS. Publish one coherent value. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    acquisition.bus_voltage = voltage;
    acquisition.bus_at = HAL_GetTick();
    acquisition.bus_raw = raw;
    ++acquisition.bus_samples;
    acquisition.bus_seen = true;
    acquisition.bus_valid = raw < 4095U && voltage >= 1.0f;
    if (acquisition.bus_valid && !acquisition.bus_good_tracking) {
        acquisition.bus_good_at = acquisition.bus_at;
        acquisition.bus_good_tracking = true;
    } else if (!acquisition.bus_valid) {
        acquisition.bus_good_tracking = false;
    }
    if (acquisition.power_enabled && !acquisition.bus_valid)
        trip(raw >= 4095U ? MOTOR_SAMPLE_FAULT_BUS_RANGE : MOTOR_SAMPLE_FAULT_BUS_LOST);
    __set_PRIMASK(primask);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) publish_regular(&s_bus_dma[0]);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) publish_regular(&s_bus_dma[2]);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1 || hadc->Instance == ADC2) MotorSampling_AdcErrorIRQ();
}

void MotorSampling_AdcErrorIRQ(void) { trip(MOTOR_SAMPLE_FAULT_ADC); }

void MotorSampling_Stop(void)
{
    (void)MotorSampling_SixStepTimer(0U);
    acquisition.running = false;
    acquisition.power_enabled = false;
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    /* HAL Base_Stop leaves CEN set while any CCER channel is enabled. */
    CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_CEN);
    CLEAR_BIT(htim8.Instance->CR1, TIM_CR1_CEN);
    (void)HAL_TIM_Base_Stop(&htim1);
    (void)HAL_TIM_Base_Stop(&htim8);
    (void)HAL_TIM_Base_Stop(&htim6);
    (void)HAL_ADCEx_InjectedStop_IT(&hadc1);
    (void)HAL_ADCEx_InjectedStop(&hadc2);
    (void)HAL_ADC_Stop_DMA(&hadc1);
    acquisition.bus_valid = false;
    acquisition.bus_seen = false;
    acquisition.reference_valid = false;
}

bool MotorSampling_Start(void)
{
    if (acquisition.running) MotorSampling_Stop();
    acquisition.running = false;
    acquisition.fault = MOTOR_SAMPLE_FAULT_NONE;
    memset((void *)acquisition.axis_fault, 0, sizeof(acquisition.axis_fault));
    if (acquisition.ibus_latched) acquisition.axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS;
    else if (acquisition.ibus_init_failed) acquisition.axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS_INIT;
    acquisition.ibus_ref_valid=false; acquisition.ibus_reference_updates=0;
    memset(acquisition.energy, 0, sizeof(acquisition.energy));
    memset(acquisition.trip_raw, 0, sizeof(acquisition.trip_raw));
    memset(acquisition.trip_phase, 0, sizeof(acquisition.trip_phase));
    memset(acquisition.zero_fault, 0, sizeof(acquisition.zero_fault));
    memset((void *)acquisition.arm_frames, 0, sizeof(acquisition.arm_frames));
    s_requested[1] = s_monitored[1] = false;
    acquisition.bus_valid = false;
    acquisition.bus_seen = false;
    acquisition.bus_good_tracking = false;
    acquisition.power_enabled = false;
    acquisition.bus_samples = 0U;
    acquisition.bus_raw = 0U;
    acquisition.bus_voltage = 0.0f;
    s_adc_volts = REFERENCE_START_VOLTS / ADC_FULL_SCALE;
    acquisition.reference_valid = false;
    acquisition.reference_sum = acquisition.reference_count = 0U;
    acquisition.reference_at = acquisition.reference_updates = acquisition.reference_errors = 0U;
    acquisition.reference_raw = 0U;
    acquisition.reference_cal = *VREFINT_CAL_ADDR;
    acquisition.offset_count = 0U;
    s_overload_energy = 0.0f;
    s_fault_phase = 0U;
    s_fault_during_zero = false;
    acquisition.bus_divider = 0U;
    memset(s_fault_current_raw, 0, sizeof(s_fault_current_raw));
    memset(acquisition.offsets, 0, sizeof(acquisition.offsets));
    memset(&acquisition.frame, 0, sizeof(acquisition.frame));
    memset(acquisition.offset_sum, 0, sizeof(acquisition.offset_sum));

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    /* 两计数器在主机开始前都保持停止；从机只能由 ITR0 启动。 */
    CLEAR_BIT(htim1.Instance->CR1, TIM_CR1_CEN);
    CLEAR_BIT(htim8.Instance->CR1, TIM_CR1_CEN);
    /* Keep TIM6 stopped. Its software UPDATE event remains the regular ADC
     * trigger selected by CubeMX; acquisition owns the event scheduling. */
    CLEAR_BIT(htim6.Instance->CR1, TIM_CR1_CEN);
    /* DIR is writable only outside center-aligned mode. Restore both counters
     * to up-counting as well as CNT=0 when restarting after a Stop. */
    uint32_t cr1 = htim1.Instance->CR1;
    uint32_t cr8 = htim8.Instance->CR1;
    htim1.Instance->CR1 = cr1 & ~(TIM_CR1_CMS | TIM_CR1_DIR);
    htim8.Instance->CR1 = cr8 & ~(TIM_CR1_CMS | TIM_CR1_DIR);
    htim1.Instance->CR1 = cr1 & ~TIM_CR1_DIR;
    htim8.Instance->CR1 = cr8 & ~TIM_CR1_DIR;
    htim1.Instance->CNT = htim8.Instance->CNT = 0U;
    htim1.Instance->CCR1 = htim1.Instance->CCR2 = htim1.Instance->CCR3 =
        htim1.Instance->ARR / 2U;
    htim8.Instance->CCR1 = htim8.Instance->CCR2 = htim8.Instance->CCR3 =
        htim8.Instance->ARR / 2U;
    htim1.Instance->EGR = TIM_EVENTSOURCE_UPDATE;
    htim8.Instance->EGR = TIM_EVENTSOURCE_UPDATE;
    /* Only the ADC-trigger channel is enabled during acquisition/zeroing.
     * Keep all six M1 phase outputs disconnected even when MOE is enabled
     * for the timer's internal trigger path. */
    CLEAR_BIT(htim1.Instance->CCER, PHASE_CHANNELS);
    SET_BIT(htim1.Instance->CCER, TIM_CCER_CC4E);
    CLEAR_BIT(htim8.Instance->CCER, PHASE_CHANNELS);

    ibus_init(); /* DAC -> settle -> COMP -> Break, before any PWM request. */

    if (acquisition.reference_cal < 1000U || acquisition.reference_cal > 2000U) {
        trip(MOTOR_SAMPLE_FAULT_REFERENCE);
        goto failed;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(void *)s_bus_dma, 4U) != HAL_OK) goto failed;

    /* VBUS may be 0 V or absent for an arbitrary time. Keep acquisition alive
     * and start the injected group with both power stages disabled. */
    if (acquisition.fault != MOTOR_SAMPLE_FAULT_NONE || HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) goto failed;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (acquisition.fault != MOTOR_SAMPLE_FAULT_NONE) {
        __set_PRIMASK(primask);
        goto failed;
    }
    acquisition.frame_at = HAL_GetTick();
    acquisition.started_at = acquisition.frame_at;
    acquisition.running = true;
    HAL_StatusTypeDef status = HAL_TIM_Base_Start(&htim1);
    ibus_poll();
    if (status == HAL_OK && !MotorSampling_IsGlobalStopped() && !acquisition.ibus_latched && !acquisition.ibus_init_failed)
        __HAL_TIM_MOE_ENABLE(&htim1);
    ibus_poll();
    __set_PRIMASK(primask);
    if (status != HAL_OK) goto failed;
    /* TIM8 counts from the master's trigger. M2 power stays disabled until
     * its motor parameters and closed-loop controller have been configured. */
    return true;

failed:
    trip(MOTOR_SAMPLE_FAULT_INIT);
    MotorSampling_Stop();
    return false;
}

void MotorSampling_Service(void)
{
    global_stop_service();
    ibus_service();
    if (!acquisition.running) return;
    if (!frame_fresh()) trip(MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT);
    if (!bus_fresh() && HAL_GetTick() - acquisition.started_at > SAMPLE_TIMEOUT_MS)
        trip(MOTOR_SAMPLE_FAULT_BUS_TIMEOUT);
    if (!reference_fresh() && HAL_GetTick() - acquisition.started_at > REFERENCE_TIMEOUT_MS)
        trip(MOTOR_SAMPLE_FAULT_REFERENCE);

    /* First power-up only. A fault remains latched, but measurements continue
     * so a restored supply is visible without silently restarting the motor. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t good_at = acquisition.bus_good_at;
    if (acquisition.fault == MOTOR_SAMPLE_FAULT_NONE && !acquisition.power_enabled &&
        MotorSampling_IsCalibrated() && frame_fresh() && bus_fresh() &&
        acquisition.bus_valid && acquisition.bus_good_tracking && reference_fresh() &&
        HAL_GetTick() - good_at >= BUS_SETTLE_MS) {
        acquisition.power_enabled = true;
        arm_axis(MOTOR_AXIS_1);
    }
    __set_PRIMASK(primask);
}

__attribute__((noinline,noclone)) bool MotorSampling_Capture(void)
{
    (void)MotorSampling_IsGlobalStopped();
    ibus_poll();
    if (!acquisition.running) return false;
    uint32_t now_cycles=DWT->CYCCNT,counter=htim1.Instance->CNT;
    bool off=acquisition.sixstep && acquisition.six_off;
    bool down=(htim1.Instance->CR1 & TIM_CR1_DIR)!=0U;
    uint32_t center_age=off ? htim1.Instance->ARR-counter : counter;
    uint32_t width=off ? htim1.Instance->ARR-htim1.Instance->CCR1 : htim1.Instance->CCR1;
    acquisition.frame.m1_sixstep_off=off;
    acquisition.frame.m1_sixstep_valid=acquisition.sixstep && s_requested[0] &&
        (htim1.Instance->CCER & PHASE_CHANNELS)==s_m1_channels &&
        (htim1.Instance->BDTR & TIM_BDTR_MOE) && down==off &&
        center_age<htim1.Instance->ARR/2U && width>800U &&
        now_cycles-s_six_commuted>center_age+850U;
    acquisition.frame.m1_sixstep_stamp=now_cycles-center_age+530U; /* Rank3 end, ~3.118us after sampled center. */
    /* Snapshot BEFORE this callback can arm outputs or FOC can update CCRs.
     * CH4=1 triggers near CNT=0; all three PWM1 high pulses must extend past
     * the last voltage acquisition (~3.12 us + trigger/deadtime margin).
     * 800 ticks at 170 MHz = 4.71 us. Never extrapolate across an edge/coast. */
    acquisition.frame.m1_voltage_high_plateau =
        !off && (htim1.Instance->CCER & PHASE_CHANNELS)==PHASE_CHANNELS &&
        (htim1.Instance->BDTR & TIM_BDTR_MOE)!=0U &&
        htim1.Instance->CCR1>800U && htim1.Instance->CCR2>800U && htim1.Instance->CCR3>800U;
    if (!frame_fresh()) trip(MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT);
    if (!__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOS)) {
        trip(MOTOR_SAMPLE_FAULT_PAIR);
        return false;
    }
    if (acquisition.power_enabled && !bus_fresh()) trip(MOTOR_SAMPLE_FAULT_BUS_TIMEOUT);
    if (acquisition.power_enabled && !reference_fresh()) trip(MOTOR_SAMPLE_FAULT_REFERENCE);
    float adc_volts = s_adc_volts;
    float current_amps = adc_volts / (0.01f * 50.0f);
    float voltage_volts = adc_volts * 11.0f;
    acquisition.frame.m1_current_raw[0] = (uint16_t)ADC1->JDR1;
    acquisition.frame.m1_current_raw[1] = (uint16_t)ADC2->JDR1;
    acquisition.frame.m1_current_raw[2] = (uint16_t)ADC1->JDR2;
    acquisition.frame.m1_voltage_raw[0] = (uint16_t)ADC1->JDR3;
    acquisition.frame.m1_voltage_raw[1] = (uint16_t)ADC2->JDR2;
    acquisition.frame.m1_voltage_raw[2] = (uint16_t)ADC2->JDR3;
    acquisition.frame.m2_current_raw[0] = (uint16_t)ADC1->JDR4;
    acquisition.frame.m2_current_raw[1] = (uint16_t)ADC2->JDR4;
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS | ADC_FLAG_JEOC);
    trigger_bus_after_injected();

    uint16_t raw[5] = {acquisition.frame.m1_current_raw[0], acquisition.frame.m1_current_raw[1],
                      acquisition.frame.m1_current_raw[2], acquisition.frame.m2_current_raw[0],
                      acquisition.frame.m2_current_raw[1]};
    bool valid_bus = acquisition.bus_valid && bus_fresh();
    if (!acquisition.power_enabled && acquisition.fault == MOTOR_SAMPLE_FAULT_NONE &&
        !valid_bus && acquisition.offset_count != 0U) {
        acquisition.offset_count = 0U;
        memset(acquisition.offset_sum, 0, sizeof(acquisition.offset_sum));
    }
    bool calibrating = !MotorSampling_IsCalibrated();
    uint32_t good_at = acquisition.bus_good_at;
    /* A late-connected motor supply may also power the analog front end.
     * Do not use its unpowered/settling output as the current zero offset. */
    bool collect_zero = calibrating && valid_bus && reference_fresh() && acquisition.bus_good_tracking &&
                        HAL_GetTick() - good_at >= BUS_SETTLE_MS;
    for (unsigned i = 0U; i < 5U; ++i) {
        if (collect_zero) acquisition.offset_sum[i] += raw[i];
        float amps = calibrating ? 0.0f :
                     ((float)raw[i] - acquisition.offsets[i]) * current_amps;
        if (i < 3U) acquisition.frame.m1_current[i] = amps;
        else acquisition.frame.m2_current[i - 3U] = amps;
    }
    if (collect_zero && ++acquisition.offset_count == OFFSET_SAMPLES) {
        for (unsigned i = 0U; i < 5U; ++i)
            acquisition.offsets[i] = (float)acquisition.offset_sum[i] / (float)OFFSET_SAMPLES;
        for (unsigned i = 0U; i < 3U; ++i) {
            if (acquisition.offsets[i] <= 10.0f || acquisition.offsets[i] >= 4085.0f)
                trip_axis(MOTOR_AXIS_1, MOTOR_SAMPLE_FAULT_CURRENT_RANGE, i + 1U, true);
        }
    }
    /* A missing M2 front end must not prevent M1 operation. Its own zero
     * validation still rejects enabling M2 until valid offsets exist. */
    if (!calibrating) {
        for (unsigned i=3; i<5; ++i)
            if (acquisition.offsets[i]<=10.0f || acquisition.offsets[i]>=4085.0f)
                trip_axis(MOTOR_AXIS_2, MOTOR_SAMPLE_FAULT_CURRENT_RANGE, i-2U, true);
    }
    if (acquisition.power_enabled && !calibrating) {
        for (unsigned a=0; a<MOTOR_AXIS_COUNT; ++a) {
            if (!s_monitored[a] || acquisition.axis_fault[a] != MOTOR_SAMPLE_FAULT_NONE) continue;
            const MotorParams *motor = MotorParams_GetAxis((MotorAxis)a);
            bool esc=a==0 && SixStep_IsGeneric();
            float iu = a == 0 ? acquisition.frame.m1_current[0] : acquisition.frame.m2_current[0];
            float iv = a == 0 ? acquisition.frame.m1_current[1] : acquisition.frame.m2_current[1];
            float iw = a == 0 ? acquisition.frame.m1_current[2] : -(iu+iv);
            float currents[3] = {iu, iv, iw};
            for (unsigned i=0; i<3; ++i) {
                uint16_t code = a == 0 ? raw[i] : (i<2 ? raw[i+3] : 2048U);
                if (code<=10U || code>=4085U)
                    trip_axis((MotorAxis)a, MOTOR_SAMPLE_FAULT_CURRENT_RANGE, i+1U, false);
                else if (fabsf(currents[i]) >= (esc ? 2.0f : motor->max_current))
                    trip_axis((MotorAxis)a, MOTOR_SAMPLE_FAULT_OVERCURRENT, i+1U, false);
            }
            float squared = (2.0f/3.0f)*(iu*iu+iv*iv+iw*iw);
            float reference = esc ? 0.64f : motor->overload_current * motor->overload_current;
            acquisition.energy[a] = fmaxf(0.0f, acquisition.energy[a]+squared-reference);
            if (acquisition.energy[a] >= reference * CURRENT_SAMPLE_HZ * (esc ? 3.0f : motor->overload_budget_s))
                trip_axis((MotorAxis)a, MOTOR_SAMPLE_FAULT_OVERLOAD, 0U, false);
        }
    }
    for (unsigned a=0; a<MOTOR_AXIS_COUNT; ++a) {
        if (s_requested[a] && acquisition.power_enabled && acquisition.fault == MOTOR_SAMPLE_FAULT_NONE &&
            acquisition.axis_fault[a] == MOTOR_SAMPLE_FAULT_NONE && acquisition.arm_frames[a] != 0U &&
            --acquisition.arm_frames[a] == 0U) {
            arm_axis((MotorAxis)a);
        }
    }
    for (unsigned i = 0U; i < 3U; ++i)
        acquisition.frame.m1_voltage[i] = (float)acquisition.frame.m1_voltage_raw[i] * voltage_volts;
    ++acquisition.frame.sequence;
    acquisition.frame_at = HAL_GetTick();
    return true;
}

const MotorSampleFrame *MotorSampling_GetFrame(void)
{
    return &acquisition.frame;
}

void MotorSampling_CopyFrame(MotorSampleFrame *out)
{
    if (out == NULL) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = acquisition.frame;
    __set_PRIMASK(primask);
}
