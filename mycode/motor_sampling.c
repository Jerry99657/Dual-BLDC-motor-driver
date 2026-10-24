#include "motor_sampling.h"

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

static MotorSampleFrame s_frame;
static uint32_t s_offset_sum[5];
static float s_offsets[5];
static volatile uint32_t s_offset_count;
/* Two complete [VBUS, VREFINT] scans; half/full callbacks never split a scan. */
static volatile uint16_t s_bus_dma[4] __attribute__((aligned(4)));
/* Publish a SINGLE aligned float: each ISR snapshots it once and derives both
 * its current and voltage coefficients, so an update cannot tear the pair. */
static volatile float s_adc_volts = REFERENCE_START_VOLTS / ADC_FULL_SCALE;
static uint32_t s_reference_sum;
static uint16_t s_reference_count, s_reference_cal;
static volatile uint16_t s_reference_raw;
static volatile uint32_t s_reference_at, s_reference_updates, s_reference_errors;
static volatile bool s_reference_valid;
static volatile float s_bus_voltage;
static volatile uint32_t s_bus_at, s_frame_at;
static volatile uint32_t s_started_at, s_bus_good_at, s_bus_samples;
static volatile uint16_t s_bus_raw;
static volatile bool s_bus_seen, s_bus_valid, s_running, s_power_enabled;
static volatile bool s_bus_good_tracking;
static volatile bool s_requested[2] = {true, false};
static bool s_monitored[2] = {true, false};
static volatile MotorSamplingFault s_axis_fault[2];
#define s_output_requested s_requested[0]
static volatile uint8_t s_arm_frames[2];
#define s_output_arm_frames s_arm_frames[0]
static volatile MotorSamplingFault s_fault;
static float s_energy[2];
#define s_overload_energy s_energy[0]
static uint16_t s_trip_raw[2][3];
#define s_fault_current_raw s_trip_raw[0]
static uint8_t s_trip_phase[2];
#define s_fault_phase s_trip_phase[0]
static bool s_zero_fault[2];
#define s_fault_during_zero s_zero_fault[0]
static uint8_t s_bus_divider;
static uint16_t s_ibus_limit_ma=APP_IBUS_DEFAULT_MA,s_ibus_dac_code;
static volatile bool s_ibus_started,s_ibus_latched,s_ibus_init_failed,s_ibus_ref_valid;
static volatile uint32_t s_ibus_trips;
static uint32_t s_ibus_reference_updates,s_ibus_low_at,s_ibus_changed_at;
static bool s_ibus_low_tracking;
static volatile bool s_global_stopped,s_global_stop_pending,s_key_releasing;
static uint32_t s_key_released_at;
static bool s_sixstep,s_six_timer,s_six_off;
static uint8_t s_six_sector=255U;
static uint32_t s_m1_channels=PHASE_CHANNELS,s_six_commuted;

bool MotorSampling_SixStepTimer(uint32_t us)
{
    if (!s_six_timer) {
        if (!us) return true;
        if (!s_sixstep) return false;
        __HAL_RCC_TIM7_CLK_ENABLE();
        TIM7->CR1=TIM_CR1_OPM|TIM_CR1_URS;
        TIM7->PSC=169U; /* APB1 timer clock stays 170MHz; 1us ticks. */
        HAL_NVIC_SetPriority(TIM7_IRQn,0U,0U);
        HAL_NVIC_ClearPendingIRQ(TIM7_IRQn);
        HAL_NVIC_EnableIRQ(TIM7_IRQn);
        s_six_timer=true;
    }
    TIM7->CR1&=~TIM_CR1_CEN;
    TIM7->DIER=0U; TIM7->SR=0U;
    if (!us) return true;
    if (us>260000U || !s_sixstep || !s_requested[0] || MotorSampling_IsGlobalStopped()) return false;
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
        s_sixstep=enable; s_six_sector=255U;
        s_six_off=false; htim1.Instance->CCR4=1U;
        s_m1_channels=PHASE_CHANNELS;
        MODIFY_REG(htim1.Instance->CCMR1,TIM_CCMR1_OC1M|TIM_CCMR1_OC2M,
                   TIM_OCMODE_PWM1|(TIM_OCMODE_PWM1<<8U));
        MODIFY_REG(htim1.Instance->CCMR2,TIM_CCMR2_OC3M,TIM_OCMODE_PWM1);
    }
    __set_PRIMASK(mask); return ok;
}
bool MotorSampling_SixStepPattern(uint8_t sector,float duty)
{
    if (!s_sixstep || sector>6U || !isfinite(duty) || duty<0.0f || duty>0.85f) return false;
    static const uint8_t hs[]={0,0,1,1,2,2},ls[]={1,2,2,0,0,1};
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (MotorSampling_IsGlobalStopped() || !MotorSampling_IsReady()) { __set_PRIMASK(mask); return false; }
    bool armed=s_requested[0] && !s_arm_frames[0] && (htim1.Instance->CCER & PHASE_CHANNELS);
    /* At low duty the OFF plateau is wider and farther from its PWM edge.
     * CH4 remains PWM1: CCR4=ARR-1 gives its rising edge just after the peak;
     * CCR4=1 gives it near the valley. Hysteresis prevents mode chatter.
     * Sine/FOC/HSO retain their original valley trigger. */
    s_six_off=sector<6U && (duty<0.25f || (s_six_off && duty<0.30f));
    htim1.Instance->CCR4=s_six_off ? htim1.Instance->ARR-1U : 1U;
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

void MotorSampling_GlobalStopIRQ(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    s_global_stopped=s_global_stop_pending=true;
    s_key_releasing=false;
    (void)MotorSampling_SixStepTimer(0U);
    /* EXTI is enabled before MX_TIMx_Init: handles can still be NULL. Keep
     * counters/CH4 alive once initialized, so ADC/diagnostics keep running. */
    TIM_HandleTypeDef *timers[]={&htim1,&htim8};
    for (unsigned a=0;a<MOTOR_AXIS_COUNT;++a) {
        if (timers[a]->Instance) {
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(timers[a]);
            CLEAR_BIT(timers[a]->Instance->CCER,PHASE_CHANNELS);
        }
        s_requested[a]=false; s_arm_frames[a]=0U;
    }
    __set_PRIMASK(mask);
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
    if (pin==KEY1_Pin) MotorSampling_GlobalStopIRQ();
}

bool MotorSampling_IsGlobalStopped(void)
{
    /* Read the pending edge too: a short press can finish while interrupts
     * are masked. Never let a later MOE/CCER write overtake that event. */
    if ((HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET ||
        __HAL_GPIO_EXTI_GET_IT(KEY1_Pin)) && (!s_global_stopped || s_key_releasing))
        MotorSampling_GlobalStopIRQ();
    return s_global_stopped;
}

bool MotorSampling_TakeGlobalStop(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    (void)MotorSampling_IsGlobalStopped();
    bool pending=s_global_stop_pending;
    s_global_stop_pending=false;
    __set_PRIMASK(mask);
    return pending;
}

bool MotorSampling_ResumeGlobalStop(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=!MotorSampling_IsGlobalStopped();
    if (!ok && !s_global_stop_pending && s_key_releasing &&
        HAL_GetTick()-s_key_released_at>=30U) {
        s_global_stopped=false; ok=true;
    }
    __set_PRIMASK(mask);
    return ok;
}

static void global_stop_service(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (MotorSampling_IsGlobalStopped() && !s_key_releasing &&
        HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_SET &&
        !__HAL_GPIO_EXTI_GET_IT(KEY1_Pin)) {
        s_key_released_at=HAL_GetTick(); s_key_releasing=true;
    }
    __set_PRIMASK(mask);
}

static void ibus_latch(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_BREAK); /* Sustained high must not cause an IRQ storm. */
    CLEAR_BIT(htim1.Instance->CCER,PHASE_CHANNELS);
    s_requested[0]=false; s_arm_frames[0]=0;
    if (!s_ibus_latched) { ++s_ibus_trips; s_ibus_low_tracking=false; }
    s_ibus_latched=true;
    (void)MotorSampling_SixStepTimer(0U);
    if (s_axis_fault[0]==MOTOR_SAMPLE_FAULT_NONE) s_axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS;
    __set_PRIMASK(mask);
}

/* BIF is a sticky hardware event, including a pulse that has already ended.
 * Poll at every software output gate as well as in the Break callback: ADC
 * and BRK share priority 0, so the ADC ISR may run before the BRK handler. */
static void ibus_poll(void)
{
    if (s_ibus_started && __HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_BREAK)) ibus_latch();
}

void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance==htim1.Instance) ibus_latch(); /* HAL already cleared BIF. */
}

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
    if (s_ibus_started || s_ibus_init_failed) return;
    /* All six phase outputs are disconnected by Start before reaching here.
     * Load a positive threshold before starting the comparator. */
    if (!ibus_write_threshold(REFERENCE_START_VOLTS) ||
        HAL_DAC_Start(&hdac3,DAC_CHANNEL_2)!=HAL_OK) goto failed;
    HAL_Delay(1U); /* Startup only, before ADC/TIM acquisition. */
    __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_BREAK);
    if (HAL_COMP_Start(&hcomp4)!=HAL_OK) goto failed;
    /* HAL waits for the comparator's specified startup delay. Its output
     * before that point is not a qualified current indication. All power
     * CCER bits/MOE are still OFF, so discard only this startup flag. */
    s_ibus_started=true;
    if (HAL_COMP_GetOutputLevel(&hcomp4)==COMP_OUTPUT_LEVEL_HIGH) ibus_latch();
    else {
        __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_BREAK);
        __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_BREAK);
    }
    ibus_poll();
    return;
failed:
    s_ibus_init_failed=true;
    __HAL_TIM_DISABLE_IT(&htim1,TIM_IT_BREAK);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    s_requested[0]=false; s_arm_frames[0]=0;
    if (s_axis_fault[0]==MOTOR_SAMPLE_FAULT_NONE) s_axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS_INIT;
}

uint16_t MotorSampling_GetIbusLimit(void) { return s_ibus_limit_ma; }
bool MotorSampling_IsIbusTripped(void) { ibus_poll(); return s_ibus_latched; }
bool MotorSampling_SetIbusLimit(uint16_t ma)
{
    if (ma<APP_IBUS_MIN_MA || ma>APP_IBUS_MAX_MA || ma%100U) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=(!s_requested[0] || (!s_running && !s_ibus_started)) &&
        !(htim1.Instance->CCER & PHASE_CHANNELS) && !s_ibus_init_failed;
    if (ok) {
        uint16_t previous=s_ibus_limit_ma;
        s_ibus_limit_ma=ma;
        s_ibus_low_tracking=false; s_ibus_ref_valid=false; s_ibus_changed_at=HAL_GetTick();
        if (s_ibus_started && !ibus_write_threshold(s_adc_volts*ADC_FULL_SCALE)) {
            s_ibus_limit_ma=previous; s_ibus_init_failed=true;
            if (s_axis_fault[0]==MOTOR_SAMPLE_FAULT_NONE) s_axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS_INIT;
            ok=false;
        }
        ibus_poll();
    }
    __set_PRIMASK(mask);
    return ok;
}

static void trigger_bus_after_injected(void)
{
    if (++s_bus_divider < BUS_FRAME_DIVIDER) return;
    s_bus_divider = 0U;
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
    bool off=s_sixstep && s_six_off;
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
     * six power outputs, as during zero calibration; do not stop acquisition. */
    CLEAR_BIT(htim1.Instance->CCER, PHASE_CHANNELS);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    s_power_enabled = false;
    if (s_fault == MOTOR_SAMPLE_FAULT_NONE) s_fault = fault;
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
    if (s_axis_fault[axis] == MOTOR_SAMPLE_FAULT_NONE) {
        for (unsigned i=0; i<3; ++i)
            s_trip_raw[axis][i] = axis == MOTOR_AXIS_1 ? s_frame.m1_current_raw[i] :
                                  (i<2 ? s_frame.m2_current_raw[i] : 0U);
        s_trip_phase[axis] = (uint8_t)phase;
        s_zero_fault[axis] = zero;
        s_axis_fault[axis] = fault;
    }
    CLEAR_BIT(axis_timer(axis)->Instance->CCER, PHASE_CHANNELS);
    if (axis==MOTOR_AXIS_2) __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    s_arm_frames[axis] = 0U;
    __set_PRIMASK(mask);
}

static bool frame_fresh(void)
{
    /* Read the ISR-owned timestamp BEFORE the clock. Reversing this order
     * can underflow if an IRQ publishes a newer timestamp during GetTick. */
    uint32_t at = s_frame_at;
    return HAL_GetTick() - at <= SAMPLE_TIMEOUT_MS;
}

static bool bus_fresh(void)
{
    bool seen = s_bus_seen;
    uint32_t at = s_bus_at;
    return seen && HAL_GetTick() - at <= SAMPLE_TIMEOUT_MS;
}

static bool reference_fresh(void)
{
    bool valid = s_reference_valid;
    uint32_t at = s_reference_at;
    return valid && HAL_GetTick() - at <= REFERENCE_TIMEOUT_MS;
}

static void ibus_service(void)
{
    ibus_poll();
    if (!s_ibus_started || s_ibus_init_failed) return;
    if (reference_fresh() && (!s_ibus_ref_valid || s_ibus_reference_updates!=s_reference_updates)) {
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        if (ibus_write_threshold(s_adc_volts*ADC_FULL_SCALE)) {
            s_ibus_reference_updates=s_reference_updates;
            s_ibus_ref_valid=HAL_GetTick()-s_ibus_changed_at>=2U;
        } else {
            s_ibus_init_failed=true;
            CLEAR_BIT(htim1.Instance->CCER,PHASE_CHANNELS);
            s_requested[0]=false; s_arm_frames[0]=0;
            if (s_axis_fault[0]==MOTOR_SAMPLE_FAULT_NONE) s_axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS_INIT;
        }
        ibus_poll();
        __set_PRIMASK(mask);
    }
    bool pulse=__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_BREAK)!=0U;
    if (s_ibus_latched && pulse) {
        /* Acknowledge pulses only AFTER latching/disconnecting. This does
         * not clear the software fault or re-enable MOE/Break interrupts. */
        __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_BREAK);
        s_ibus_low_tracking=false;
    }
    if (HAL_COMP_GetOutputLevel(&hcomp4)==COMP_OUTPUT_LEVEL_HIGH) {
        ibus_latch(); s_ibus_low_tracking=false;
    } else if (!s_ibus_low_tracking) {
        s_ibus_low_at=HAL_GetTick(); s_ibus_low_tracking=true;
    }
}

bool MotorSampling_ClearIbusTrip(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=s_ibus_latched && s_ibus_started && !s_ibus_init_failed && s_ibus_ref_valid &&
        !s_requested[0] && !(htim1.Instance->CCER & PHASE_CHANNELS) &&
        s_fault==MOTOR_SAMPLE_FAULT_NONE && s_axis_fault[0]==MOTOR_SAMPLE_FAULT_IBUS &&
        s_running && reference_fresh() && frame_fresh() && bus_fresh() && s_bus_valid &&
        s_ibus_low_tracking && HAL_GetTick()-s_ibus_low_at>=100U &&
        !__HAL_TIM_GET_FLAG(&htim1,TIM_FLAG_BREAK) &&
        HAL_COMP_GetOutputLevel(&hcomp4)==COMP_OUTPUT_LEVEL_LOW;
    if (ok) {
        /* Service acknowledged older pulses while the output was OFF.
         * Never clear BIF here: a NEW pulse arriving after the quiet check
         * must remain pending and immediately re-latch the protection. */
        s_ibus_latched=false; s_axis_fault[0]=MOTOR_SAMPLE_FAULT_NONE;
        __HAL_TIM_ENABLE_IT(&htim1,TIM_IT_BREAK);
        /* Leave MOE, CCER, requests and preload countdown OFF. Only a new
         * explicit control command can arm M1 after this manual clear. */
        ibus_poll();
        ok=!s_ibus_latched;
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
        if (!s_ibus_started || !s_ibus_ref_valid || s_ibus_latched || s_ibus_init_failed) return;
    }
    if (!s_requested[axis] || s_fault!=MOTOR_SAMPLE_FAULT_NONE ||
        s_axis_fault[axis]!=MOTOR_SAMPLE_FAULT_NONE) return;
    CLEAR_BIT(axis_timer(axis)->Instance->CCER,PHASE_CHANNELS);
    __HAL_TIM_MOE_ENABLE(axis_timer(axis));
    if (axis==MOTOR_AXIS_1) ibus_poll();
    if (!MotorSampling_IsGlobalStopped() && s_requested[axis] && s_axis_fault[axis]==MOTOR_SAMPLE_FAULT_NONE)
        SET_BIT(axis_timer(axis)->Instance->CCER,axis==MOTOR_AXIS_1 ? s_m1_channels : PHASE_CHANNELS);
}

bool MotorSampling_IsAxisReady(MotorAxis axis)
{
    ibus_poll();
    return (unsigned)axis < MOTOR_AXIS_COUNT && s_running && s_power_enabled &&
           (axis!=MOTOR_AXIS_1 || (s_ibus_started && s_ibus_ref_valid && !s_ibus_latched && !s_ibus_init_failed)) &&
           s_fault == MOTOR_SAMPLE_FAULT_NONE && s_axis_fault[axis] == MOTOR_SAMPLE_FAULT_NONE &&
           s_bus_valid && bus_fresh() && frame_fresh() && reference_fresh();
}

bool MotorSampling_IsReady(void) { return MotorSampling_IsAxisReady(MOTOR_AXIS_1); }
bool MotorSampling_IsCalibrated(void) { return s_offset_count >= OFFSET_SAMPLES; }
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
        s_arm_frames[axis] = 0U;
        CLEAR_BIT(axis_timer(axis)->Instance->CCER, PHASE_CHANNELS);
        if (axis==MOTOR_AXIS_2) __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
    } else if (MotorSampling_IsGlobalStopped() || !MotorSampling_IsAxisReady(axis) || !MotorSampling_IsCalibrated()) {
        ok = false;
    } else if (!s_requested[axis]) {
        s_requested[axis] = true;
        s_monitored[axis] = true;
        s_arm_frames[axis] = 2U;
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
    return bus_fresh() ? s_bus_voltage : 0.0f;
}

void MotorSampling_GetAxisDiagnostics(MotorAxis axis, MotorSamplingDiagnostics *out)
{
    if (out == NULL || (unsigned)axis >= MOTOR_AXIS_COUNT) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->fault = s_fault != MOTOR_SAMPLE_FAULT_NONE ? s_fault : s_axis_fault[axis];
    out->bus_raw = s_bus_raw;
    out->bus_samples = s_bus_samples;
    out->frame_samples = s_frame.sequence;
    out->adc1_error = hadc1.ErrorCode;
    out->adc2_error = hadc2.ErrorCode;
    out->bus_voltage = s_bus_voltage;
    out->power_enabled = s_power_enabled && out->fault == MOTOR_SAMPLE_FAULT_NONE;
    for (unsigned i = 0U; i < 3U; ++i) {
        out->current_raw[i] = axis == MOTOR_AXIS_1 ? s_frame.m1_current_raw[i] :
                              (i<2 ? s_frame.m2_current_raw[i] : 0U);
        out->current_zero[i] = axis == MOTOR_AXIS_1 ? (uint16_t)(s_offsets[i]+0.5f) :
                               (i<2 ? (uint16_t)(s_offsets[i+3]+0.5f) : 0U);
        out->fault_current_raw[i] = s_trip_raw[axis][i];
    }
    out->fault_phase = s_trip_phase[axis];
    out->fault_during_zero = s_zero_fault[axis];
    out->silicon_revision = (uint16_t)HAL_GetREVID();
    out->reference_voltage = s_adc_volts * ADC_FULL_SCALE;
    out->reference_raw = s_reference_raw;
    out->reference_cal = s_reference_cal;
    out->reference_updates = s_reference_updates;
    out->reference_errors = s_reference_errors;
    out->ibus_limit_ma=s_ibus_limit_ma;
    out->ibus_dac_code=s_ibus_dac_code;
    out->ibus_trips=s_ibus_trips;
    out->ibus_latched=s_ibus_latched;
    out->ibus_ready=s_ibus_started && s_ibus_ref_valid && !s_ibus_init_failed;
    out->ibus_high=s_ibus_started && HAL_COMP_GetOutputLevel(&hcomp4)==COMP_OUTPUT_LEVEL_HIGH;
    bool reference_valid = s_reference_valid;
    uint32_t reference_at = s_reference_at;
    bool seen = s_bus_seen;
    uint32_t bus_at = s_bus_at, frame_at = s_frame_at;
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
    MotorSamplingFault fault = s_fault != MOTOR_SAMPLE_FAULT_NONE ? s_fault : s_axis_fault[axis];
    if (axis == MOTOR_AXIS_2) {
        if (fault == MOTOR_SAMPLE_FAULT_OVERCURRENT) return "M2 OVERCURRENT";
        if (fault == MOTOR_SAMPLE_FAULT_OVERLOAD) return "M2 OVERLOAD";
        if (fault == MOTOR_SAMPLE_FAULT_CURRENT_RANGE)
            return s_zero_fault[axis] ? "M2 ZERO ADC" : "M2 CURRENT ADC";
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
    if (!s_running) return "STOPPED";
    if (!bus_fresh()) return "WAIT ADC";
    if (!s_bus_valid) return s_bus_raw >= 4095U ? "BUS RANGE" : "WAIT BUS";
    if (!reference_fresh()) return "WAIT VREF";
    if (axis==MOTOR_AXIS_1 && !s_ibus_ref_valid) return "WAIT IBUS";
    if (!MotorSampling_IsCalibrated()) return "ZERO CAL";
    if (!s_power_enabled) return "BUS SETTLE";
    return "READY";
}

static void publish_reference(uint16_t raw)
{
    s_reference_raw = raw;
    /* Avoid division by zero and stop isolated invalid/rail readings from
     * becoming a conversion coefficient. Reject the partial average too. */
    float numerator = (float)s_reference_cal * ((float)VREFINT_CAL_VREF * 0.001f);
    if (raw == 0U || raw >= 4095U ||
        numerator < REFERENCE_MIN_VOLTS * (float)raw ||
        numerator > REFERENCE_MAX_VOLTS * (float)raw) {
        ++s_reference_errors;
        s_reference_sum = 0U;
        s_reference_count = 0U;
        return;
    }
    s_reference_sum += raw;
    if (++s_reference_count < REFERENCE_AVERAGE_SAMPLES) return;
    float estimate = numerator * (float)REFERENCE_AVERAGE_SAMPLES / (float)s_reference_sum;
    float filtered = s_reference_valid ? s_adc_volts * ADC_FULL_SCALE : estimate;
    filtered += REFERENCE_FILTER_ALPHA * (estimate - filtered);
    /* The first complete block seeds the filter, never a hardcoded 3.3 V.
     * Motor power is inhibited until a measured reference is available. */
    s_adc_volts = filtered / ADC_FULL_SCALE;
    s_reference_at = HAL_GetTick();
    s_reference_valid = true;
    ++s_reference_updates;
    s_reference_sum = 0U;
    s_reference_count = 0U;
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
    s_bus_voltage = voltage;
    s_bus_at = HAL_GetTick();
    s_bus_raw = raw;
    ++s_bus_samples;
    s_bus_seen = true;
    s_bus_valid = raw < 4095U && voltage >= 1.0f;
    if (s_bus_valid && !s_bus_good_tracking) {
        s_bus_good_at = s_bus_at;
        s_bus_good_tracking = true;
    } else if (!s_bus_valid) {
        s_bus_good_tracking = false;
    }
    if (s_power_enabled && !s_bus_valid)
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
    if (hadc->Instance == ADC1 || hadc->Instance == ADC2) trip(MOTOR_SAMPLE_FAULT_ADC);
}

void MotorSampling_Stop(void)
{
    (void)MotorSampling_SixStepTimer(0U);
    s_running = false;
    s_power_enabled = false;
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
    s_bus_valid = false;
    s_bus_seen = false;
    s_reference_valid = false;
}

bool MotorSampling_Start(void)
{
    if (s_running) MotorSampling_Stop();
    s_running = false;
    s_fault = MOTOR_SAMPLE_FAULT_NONE;
    memset((void *)s_axis_fault, 0, sizeof(s_axis_fault));
    if (s_ibus_latched) s_axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS;
    else if (s_ibus_init_failed) s_axis_fault[0]=MOTOR_SAMPLE_FAULT_IBUS_INIT;
    s_ibus_ref_valid=false; s_ibus_reference_updates=0;
    memset(s_energy, 0, sizeof(s_energy));
    memset(s_trip_raw, 0, sizeof(s_trip_raw));
    memset(s_trip_phase, 0, sizeof(s_trip_phase));
    memset(s_zero_fault, 0, sizeof(s_zero_fault));
    memset((void *)s_arm_frames, 0, sizeof(s_arm_frames));
    s_requested[1] = s_monitored[1] = false;
    s_bus_valid = false;
    s_bus_seen = false;
    s_bus_good_tracking = false;
    s_power_enabled = false;
    s_bus_samples = 0U;
    s_bus_raw = 0U;
    s_bus_voltage = 0.0f;
    s_adc_volts = REFERENCE_START_VOLTS / ADC_FULL_SCALE;
    s_reference_valid = false;
    s_reference_sum = s_reference_count = 0U;
    s_reference_at = s_reference_updates = s_reference_errors = 0U;
    s_reference_raw = 0U;
    s_reference_cal = *VREFINT_CAL_ADDR;
    s_offset_count = 0U;
    s_overload_energy = 0.0f;
    s_fault_phase = 0U;
    s_fault_during_zero = false;
    s_bus_divider = 0U;
    memset(s_fault_current_raw, 0, sizeof(s_fault_current_raw));
    memset(s_offsets, 0, sizeof(s_offsets));
    memset(&s_frame, 0, sizeof(s_frame));
    memset(s_offset_sum, 0, sizeof(s_offset_sum));

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

    if (s_reference_cal < 1000U || s_reference_cal > 2000U) {
        trip(MOTOR_SAMPLE_FAULT_REFERENCE);
        goto failed;
    }
    if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK ||
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)(void *)s_bus_dma, 4U) != HAL_OK) goto failed;

    /* VBUS may be 0 V or absent for an arbitrary time. Keep acquisition alive
     * and start the injected group with both power stages disabled. */
    if (s_fault != MOTOR_SAMPLE_FAULT_NONE || HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK ||
        HAL_ADCEx_InjectedStart_IT(&hadc1) != HAL_OK) goto failed;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_fault != MOTOR_SAMPLE_FAULT_NONE) {
        __set_PRIMASK(primask);
        goto failed;
    }
    s_frame_at = HAL_GetTick();
    s_started_at = s_frame_at;
    s_running = true;
    HAL_StatusTypeDef status = HAL_TIM_Base_Start(&htim1);
    ibus_poll();
    if (status == HAL_OK && !MotorSampling_IsGlobalStopped() && !s_ibus_latched && !s_ibus_init_failed)
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
    if (!s_running) return;
    if (!frame_fresh()) trip(MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT);
    if (!bus_fresh() && HAL_GetTick() - s_started_at > SAMPLE_TIMEOUT_MS)
        trip(MOTOR_SAMPLE_FAULT_BUS_TIMEOUT);
    if (!reference_fresh() && HAL_GetTick() - s_started_at > REFERENCE_TIMEOUT_MS)
        trip(MOTOR_SAMPLE_FAULT_REFERENCE);

    /* First power-up only. A fault remains latched, but measurements continue
     * so a restored supply is visible without silently restarting the motor. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t good_at = s_bus_good_at;
    if (s_fault == MOTOR_SAMPLE_FAULT_NONE && !s_power_enabled &&
        MotorSampling_IsCalibrated() && frame_fresh() && bus_fresh() &&
        s_bus_valid && s_bus_good_tracking && reference_fresh() &&
        HAL_GetTick() - good_at >= BUS_SETTLE_MS) {
        s_power_enabled = true;
        arm_axis(MOTOR_AXIS_1);
    }
    __set_PRIMASK(primask);
}

bool MotorSampling_Capture(void)
{
    (void)MotorSampling_IsGlobalStopped();
    ibus_poll();
    if (!s_running) return false;
    uint32_t now_cycles=DWT->CYCCNT,counter=htim1.Instance->CNT;
    bool off=s_sixstep && s_six_off;
    bool down=(htim1.Instance->CR1 & TIM_CR1_DIR)!=0U;
    uint32_t center_age=off ? htim1.Instance->ARR-counter : counter;
    uint32_t width=off ? htim1.Instance->ARR-htim1.Instance->CCR1 : htim1.Instance->CCR1;
    s_frame.m1_sixstep_off=off;
    s_frame.m1_sixstep_valid=s_sixstep && s_requested[0] &&
        (htim1.Instance->CCER & PHASE_CHANNELS)==s_m1_channels &&
        (htim1.Instance->BDTR & TIM_BDTR_MOE) && down==off &&
        center_age<htim1.Instance->ARR/2U && width>800U &&
        now_cycles-s_six_commuted>center_age+850U;
    s_frame.m1_sixstep_stamp=now_cycles-center_age+530U; /* Rank3 end, ~3.118us after sampled center. */
    /* Snapshot BEFORE this callback can arm outputs or FOC can update CCRs.
     * CH4=1 triggers near CNT=0; all three PWM1 high pulses must extend past
     * the last voltage acquisition (~3.12 us + trigger/deadtime margin).
     * 800 ticks at 170 MHz = 4.71 us. Never extrapolate across an edge/coast. */
    s_frame.m1_voltage_high_plateau =
        !off && (htim1.Instance->CCER & PHASE_CHANNELS)==PHASE_CHANNELS &&
        (htim1.Instance->BDTR & TIM_BDTR_MOE)!=0U &&
        htim1.Instance->CCR1>800U && htim1.Instance->CCR2>800U && htim1.Instance->CCR3>800U;
    if (!frame_fresh()) trip(MOTOR_SAMPLE_FAULT_FRAME_TIMEOUT);
    if (!__HAL_ADC_GET_FLAG(&hadc2, ADC_FLAG_JEOS)) {
        trip(MOTOR_SAMPLE_FAULT_PAIR);
        return false;
    }
    if (s_power_enabled && !bus_fresh()) trip(MOTOR_SAMPLE_FAULT_BUS_TIMEOUT);
    if (s_power_enabled && !reference_fresh()) trip(MOTOR_SAMPLE_FAULT_REFERENCE);
    float adc_volts = s_adc_volts;
    float current_amps = adc_volts / (0.01f * 50.0f);
    float voltage_volts = adc_volts * 11.0f;
    s_frame.m1_current_raw[0] = (uint16_t)ADC1->JDR1;
    s_frame.m1_current_raw[1] = (uint16_t)ADC2->JDR1;
    s_frame.m1_current_raw[2] = (uint16_t)ADC1->JDR2;
    s_frame.m1_voltage_raw[0] = (uint16_t)ADC1->JDR3;
    s_frame.m1_voltage_raw[1] = (uint16_t)ADC2->JDR2;
    s_frame.m1_voltage_raw[2] = (uint16_t)ADC2->JDR3;
    s_frame.m2_current_raw[0] = (uint16_t)ADC1->JDR4;
    s_frame.m2_current_raw[1] = (uint16_t)ADC2->JDR4;
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOS | ADC_FLAG_JEOC);
    trigger_bus_after_injected();

    uint16_t raw[5] = {s_frame.m1_current_raw[0], s_frame.m1_current_raw[1],
                      s_frame.m1_current_raw[2], s_frame.m2_current_raw[0],
                      s_frame.m2_current_raw[1]};
    bool valid_bus = s_bus_valid && bus_fresh();
    if (!s_power_enabled && s_fault == MOTOR_SAMPLE_FAULT_NONE &&
        !valid_bus && s_offset_count != 0U) {
        s_offset_count = 0U;
        memset(s_offset_sum, 0, sizeof(s_offset_sum));
    }
    bool calibrating = !MotorSampling_IsCalibrated();
    uint32_t good_at = s_bus_good_at;
    /* A late-connected motor supply may also power the analog front end.
     * Do not use its unpowered/settling output as the current zero offset. */
    bool collect_zero = calibrating && valid_bus && reference_fresh() && s_bus_good_tracking &&
                        HAL_GetTick() - good_at >= BUS_SETTLE_MS;
    for (unsigned i = 0U; i < 5U; ++i) {
        if (collect_zero) s_offset_sum[i] += raw[i];
        float amps = calibrating ? 0.0f :
                     ((float)raw[i] - s_offsets[i]) * current_amps;
        if (i < 3U) s_frame.m1_current[i] = amps;
        else s_frame.m2_current[i - 3U] = amps;
    }
    if (collect_zero && ++s_offset_count == OFFSET_SAMPLES) {
        for (unsigned i = 0U; i < 5U; ++i)
            s_offsets[i] = (float)s_offset_sum[i] / (float)OFFSET_SAMPLES;
        for (unsigned i = 0U; i < 3U; ++i) {
            if (s_offsets[i] <= 10.0f || s_offsets[i] >= 4085.0f)
                trip_axis(MOTOR_AXIS_1, MOTOR_SAMPLE_FAULT_CURRENT_RANGE, i + 1U, true);
        }
    }
    /* A missing M2 front end must not prevent M1 operation. Its own zero
     * validation still rejects enabling M2 until valid offsets exist. */
    if (!calibrating) {
        for (unsigned i=3; i<5; ++i)
            if (s_offsets[i]<=10.0f || s_offsets[i]>=4085.0f)
                trip_axis(MOTOR_AXIS_2, MOTOR_SAMPLE_FAULT_CURRENT_RANGE, i-2U, true);
    }
    if (s_power_enabled && !calibrating) {
        for (unsigned a=0; a<MOTOR_AXIS_COUNT; ++a) {
            if (!s_monitored[a] || s_axis_fault[a] != MOTOR_SAMPLE_FAULT_NONE) continue;
            const MotorParams *motor = MotorParams_GetAxis((MotorAxis)a);
            bool esc=a==0 && SixStep_IsGeneric();
            float iu = a == 0 ? s_frame.m1_current[0] : s_frame.m2_current[0];
            float iv = a == 0 ? s_frame.m1_current[1] : s_frame.m2_current[1];
            float iw = a == 0 ? s_frame.m1_current[2] : -(iu+iv);
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
            s_energy[a] = fmaxf(0.0f, s_energy[a]+squared-reference);
            if (s_energy[a] >= reference * CURRENT_SAMPLE_HZ * (esc ? 3.0f : motor->overload_budget_s))
                trip_axis((MotorAxis)a, MOTOR_SAMPLE_FAULT_OVERLOAD, 0U, false);
        }
    }
    for (unsigned a=0; a<MOTOR_AXIS_COUNT; ++a) {
        if (s_requested[a] && s_power_enabled && s_fault == MOTOR_SAMPLE_FAULT_NONE &&
            s_axis_fault[a] == MOTOR_SAMPLE_FAULT_NONE && s_arm_frames[a] != 0U &&
            --s_arm_frames[a] == 0U) {
            arm_axis((MotorAxis)a);
        }
    }
    for (unsigned i = 0U; i < 3U; ++i)
        s_frame.m1_voltage[i] = (float)s_frame.m1_voltage_raw[i] * voltage_volts;
    ++s_frame.sequence;
    s_frame_at = HAL_GetTick();
    return true;
}

const MotorSampleFrame *MotorSampling_GetFrame(void)
{
    return &s_frame;
}

void MotorSampling_CopyFrame(MotorSampleFrame *out)
{
    if (out == NULL) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = s_frame;
    __set_PRIMASK(primask);
}
