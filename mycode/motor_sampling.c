#include "motor_sampling.h"

#include <string.h>
#include "adc.h"
#include "tim.h"
#include "motor_params.h"
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
static volatile bool s_output_requested = true;
static volatile uint8_t s_output_arm_frames;
static volatile MotorSamplingFault s_fault;
static float s_overload_energy;
static uint16_t s_fault_current_raw[3];
static uint8_t s_fault_phase;
static bool s_fault_during_zero;
static uint8_t s_bus_divider;

static void trigger_bus_after_injected(void)
{
    if (++s_bus_divider < BUS_FRAME_DIVIDER) return;
    s_bus_divider = 0U;
    /* ES0431 2.5.10: a regular conversion interrupted by an injected trigger
     * can make the first injected result (M1 IU) zero on affected silicon.
     * TIM6 never free-runs. Issue its UPDATE/TRGO only after both injected
     * sequences finish, every fifth frame. With the current PWM1 CH4=1
     * rising edge near CNT=0, the first half of up-counting leaves >75 us
     * before the next injected trigger (VBUS + VREFINT take ~16.78 us).
     * Skip a late ISR rather than start a conversion near the next trigger.
     * This also runs during calibration, at 0 V and after a latched trip. */
    uint32_t direction = htim1.Instance->CR1 & TIM_CR1_DIR;
    uint32_t counter = htim1.Instance->CNT;
    if (direction == 0U && counter <= htim1.Instance->ARR / 2U)
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

static void trip_current(MotorSamplingFault fault, unsigned phase, bool during_zero)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_fault == MOTOR_SAMPLE_FAULT_NONE) {
        memcpy(s_fault_current_raw, s_frame.m1_current_raw, sizeof(s_fault_current_raw));
        s_fault_phase = (uint8_t)phase;
        s_fault_during_zero = during_zero;
    }
    trip(fault);
    __set_PRIMASK(primask);
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

bool MotorSampling_IsReady(void)
{
    return s_running && s_power_enabled && s_fault == MOTOR_SAMPLE_FAULT_NONE &&
           s_bus_valid && bus_fresh() && frame_fresh() && reference_fresh();
}

bool MotorSampling_IsCalibrated(void)
{
    return s_offset_count >= OFFSET_SAMPLES;
}

bool MotorSampling_IsOutputRequested(void) { return s_output_requested; }

bool MotorSampling_IsOutputEnabled(void)
{
    return s_power_enabled && s_fault == MOTOR_SAMPLE_FAULT_NONE &&
           (htim1.Instance->CCER & PHASE_CHANNELS) == PHASE_CHANNELS;
}

bool MotorSampling_SetOutputEnabled(bool enable)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool ok = true;
    if (!enable) {
        s_output_requested = false;
        s_output_arm_frames = 0U;
        CLEAR_BIT(htim1.Instance->CCER, PHASE_CHANNELS);
    } else if (!MotorSampling_IsReady() || !MotorSampling_IsCalibrated()) {
        ok = false; /* A failed request must never become a delayed restart. */
    } else if (!s_output_requested) {
        s_output_requested = true;
        s_output_arm_frames = 2U;
    }
    __set_PRIMASK(mask);
    return ok;
}

float MotorSampling_GetBusVoltage(void)
{
    return bus_fresh() ? s_bus_voltage : 0.0f;
}

void MotorSampling_GetDiagnostics(MotorSamplingDiagnostics *out)
{
    if (out == NULL) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->fault = s_fault;
    out->bus_raw = s_bus_raw;
    out->bus_samples = s_bus_samples;
    out->frame_samples = s_frame.sequence;
    out->adc1_error = hadc1.ErrorCode;
    out->adc2_error = hadc2.ErrorCode;
    out->bus_voltage = s_bus_voltage;
    out->power_enabled = s_power_enabled;
    for (unsigned i = 0U; i < 3U; ++i) {
        out->current_raw[i] = s_frame.m1_current_raw[i];
        out->current_zero[i] = (uint16_t)(s_offsets[i] + 0.5f);
        out->fault_current_raw[i] = s_fault_current_raw[i];
    }
    out->fault_phase = s_fault_phase;
    out->fault_during_zero = s_fault_during_zero;
    out->silicon_revision = (uint16_t)HAL_GetREVID();
    out->reference_voltage = s_adc_volts * ADC_FULL_SCALE;
    out->reference_raw = s_reference_raw;
    out->reference_cal = s_reference_cal;
    out->reference_updates = s_reference_updates;
    out->reference_errors = s_reference_errors;
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
}

const char *MotorSampling_GetStatusText(void)
{
    switch (s_fault) {
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
    default: break;
    }
    if (!s_running) return "STOPPED";
    if (!bus_fresh()) return "WAIT ADC";
    if (!s_bus_valid) return s_bus_raw >= 4095U ? "BUS RANGE" : "WAIT BUS";
    if (!reference_fresh()) return "WAIT VREF";
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
    SET_BIT(htim8.Instance->CCER, PHASE_CHANNELS);

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
    if (status == HAL_OK) __HAL_TIM_MOE_ENABLE(&htim1);
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
        if (s_output_requested) SET_BIT(htim1.Instance->CCER, PHASE_CHANNELS);
        s_power_enabled = true;
        __HAL_TIM_MOE_ENABLE(&htim1);
    }
    __set_PRIMASK(primask);
}

bool MotorSampling_Capture(void)
{
    if (!s_running) return false;
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
                trip_current(MOTOR_SAMPLE_FAULT_CURRENT_RANGE, i + 1U, true);
        }
    }
    /* M1 only: M2 power remains disabled. Check unfiltered phase samples so
     * voltage mode, diagnostic open loop and alignment cannot bypass this.
     * This is a sampled software trip, not hardware cycle-by-cycle limiting. */
    if (s_power_enabled && !calibrating) {
        const MotorParams *motor = MotorParams_GetActive();
        for (unsigned i = 0U; i < 3U; ++i) {
            if (raw[i] <= 10U || raw[i] >= 4085U)
                trip_current(MOTOR_SAMPLE_FAULT_CURRENT_RANGE, i + 1U, false);
            else if (fabsf(s_frame.m1_current[i]) >= motor->max_current)
                trip_current(MOTOR_SAMPLE_FAULT_OVERCURRENT, i + 1U, false);
        }
        if (s_power_enabled) {
            float iu = s_frame.m1_current[0], iv = s_frame.m1_current[1];
            float iw = s_frame.m1_current[2];
            /* For balanced phases this equals Id^2+Iq^2 (peak convention).
             * Compare all three measured phases; no square root/filter delay. */
            float current_squared = (2.0f / 3.0f) * (iu * iu + iv * iv + iw * iw);
            float reference_squared = motor->overload_current * motor->overload_current;
            s_overload_energy += current_squared - reference_squared;
            if (s_overload_energy < 0.0f) s_overload_energy = 0.0f;
            if (s_overload_energy >= reference_squared * CURRENT_SAMPLE_HZ * motor->overload_budget_s)
                trip_current(MOTOR_SAMPLE_FAULT_OVERLOAD, 0U, false);
        }
    }
    /* Wait one full PWM period with fresh compare preloads before reconnecting
     * the bridge. Protection above can cancel this; CH4 was never stopped. */
    if (s_output_requested && s_power_enabled && s_fault == MOTOR_SAMPLE_FAULT_NONE &&
        s_output_arm_frames != 0U && --s_output_arm_frames == 0U)
        SET_BIT(htim1.Instance->CCER, PHASE_CHANNELS);
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
