#include "st_hso.h"
#include "main.h"
#include "hso.h"
#include "impedcorr.h"
#include "arm_math.h"
#include <math.h>
#include <string.h>

#define TWO_PI 6.28318530718f
#define INV_TWO_PI 0.15915494309f
#define Q30_SCALE 1073741824.0f
#define VOLTAGE_SCALE 72.6f /* Twice 3.3 V * divider 11. */
#define CURRENT_SCALE 6.6f /* Twice +/-3.3 A nominal measurement range. */
#define FREQUENCY_SCALE 1000.0f
#define MIN_ELECTRICAL_HZ 5.0f
#define QUALIFY_SAMPLES 2000U
#define QUALIFY_PAUSE_TOTAL 50U /* <=5 ms excluded; >=97.5% strict-good samples. */
#define QUALIFY_PAUSE_RUN 10U   /* <=1 ms consecutive, angle glitches only. */
#define HOLD_BAD_SAMPLES 20U /* 2 ms; hard faults bypass this confirmation. */
#define LEARN_SAMPLES 10000U

/* The published opaque structs have uint16_t alignment; their binary contents
 * include 32-bit values. Explicit alignment also permits future double fields. */
static HSO_Obj observer __attribute__((aligned(8)));
static IMPEDCORR_Obj impedance __attribute__((aligned(8)));
static StHsoConfig requested, active;
static volatile bool enabled = true, config_pending, learn_requested;
static bool initialized, was_ready;
static uint32_t last_sequence, qualified, learn_count;
static float learn_sum, learn_square_sum, learn_speed;
static int pole_pairs;
static StHsoSnapshot live;

static void reset_admission(void)
{
    qualified = 0U;
    live.qualify_paused_samples = live.qualify_bad_run = 0U;
}

static void reject_now(StHsoReason reason)
{
    live.usable = false;
    reset_admission();
    live.qualified_samples = live.bad_samples = 0U;
    live.reason = reason;
    live.entry_reason = reason;
}

static float wrap(float a)
{
    if (!isfinite(a)) return NAN;
    while (a > 3.14159265359f) a -= TWO_PI;
    while (a < -3.14159265359f) a += TWO_PI;
    return a;
}

static fixp30_t to_q30(float x) { return (fixp30_t)(x * Q30_SCALE); }
static float from_q30(fixp30_t x) { return (float)x / Q30_SCALE; }

static bool valid_config(const StHsoConfig *c)
{
    return c && isfinite(c->rs_ohm) && c->rs_ohm >= 0.01f && c->rs_ohm <= 100.0f &&
        isfinite(c->ls_h) && c->ls_h >= 0.000001f && c->ls_h <= 0.1f &&
        isfinite(c->flux_wb) && c->flux_wb >= 0.0f && c->flux_wb <= 0.2f &&
        isfinite(c->voltage_pole_rps) && c->voltage_pole_rps >= 100.0f &&
        c->voltage_pole_rps <= 1000000.0f && isfinite(c->sample_delay_cycles) &&
        fabsf(c->sample_delay_cycles) <= 0.25f;
}

void StHso_Init(const MotorParams *motor)
{
    __HAL_RCC_CORDIC_CLK_ENABLE();
    pole_pairs = motor->pole_pairs > 0 ? motor->pole_pairs : 1;
    requested = (StHsoConfig){motor->Rs, motor->Ls, motor->psi_f,
                             25000.0f, 0.019f};
    /* (20k || 2k)*22nF = 40 us. Mean phase-voltage sampling is ~1.9 us
     * later than IU/IV; this is ADC skew, NOT the analog filter time constant. */
    config_pending = true;
    enabled = true;
    live.state = ST_HSO_WAIT_PARAM;
}

bool StHso_SetConfig(const StHsoConfig *config)
{
    if (!valid_config(config)) return false;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    requested = *config;
    learn_requested = false;
    config_pending = true;
    reject_now(ST_HSO_REASON_PARAM);
    __set_PRIMASK(mask);
    return true;
}

void StHso_GetConfig(StHsoConfig *config)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *config = requested;
    __set_PRIMASK(mask);
}

void StHso_Enable(bool enable)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    enabled = enable;
    config_pending = true;
    reject_now(enable ? ST_HSO_REASON_PARAM : ST_HSO_REASON_OFF);
    learn_requested = false;
    __set_PRIMASK(mask);
}

bool StHso_LearnFlux(void)
{
    if (!enabled) return false;
    learn_requested = true;
    return true;
}

static void configure(void)
{
    active = requested;
    initialized = false;
    qualified = learn_count = 0U;
    reject_now(ST_HSO_REASON_PARAM);
    live.reference_flux_wb = active.flux_wb;
    live.qualify_reset_angle_rad = 0.0f;
    config_pending = false;
    if (!valid_config(&active) || active.flux_wb < 0.00001f) return;

    HSO_init(&observer, sizeof(observer));
    IMPEDCORR_init(&impedance, sizeof(impedance));
    IMPEDCORR_Params ip = {
        .voltageFilterPole_rps = active.voltage_pole_rps,
        .cycleFreq_Hz = 10000.0f, .fullScaleFreq_Hz = FREQUENCY_SCALE,
        .KsampleDelay = active.sample_delay_cycles,
        .fullScaleCurrent_A = CURRENT_SCALE, .fullScaleVoltage_V = VOLTAGE_SCALE
    };
    IMPEDCORR_setParams(&impedance, &ip);
    IMPEDCORR_setRs_si(&impedance, active.rs_ohm);
    IMPEDCORR_setLs_si(&impedance, active.ls_h);
    HSO_Params hp = {
        .Flux_Wb = active.flux_wb, .CrossOver_Hz = 20.0f,
        .isrTime_s = 0.0001f, .FullScaleVoltage_V = VOLTAGE_SCALE,
        .FullScaleFreq_Hz = FREQUENCY_SCALE, .speedPole_rps = 100.0f,
        .flag_FilterAdaptive = true, .Filter_adapt_fmin_Hz = 10.0f,
        .Filter_adapt_fmax_Hz = 500.0f, .CheckDirBW_Hz = 0.5f
    };
    HSO_setParams(&observer, &hp);
    HSO_setFlag_TrackAngle(&observer, true);
    HSO_setFlag_EnableOffsetUpdate(&observer, true);
    initialized = true;
}

static void learn(float ua, float ub, float ia, float ib, float angle, float speed,
                  bool encoder_ready)
{
    live.state = ST_HSO_LEARNING;
    float omega = speed * (float)pole_pairs;
    if (!encoder_ready || fabsf(omega) < TWO_PI * MIN_ELECTRICAL_HZ ||
        (learn_count && (omega * learn_speed <= 0.0f ||
         fabsf(omega - learn_speed) > fabsf(learn_speed) * 0.1f))) {
        learn_count = 0U;
        return;
    }
    float sn = arm_sin_f32(angle), cs = arm_cos_f32(angle);
    float vd = ua * cs + ub * sn;
    float vq = ub * cs - ua * sn;
    float id = ia * cs + ib * sn;
    float iq = ib * cs - ia * sn;
    /* Undo RC lag, then rotate the later voltage sample BACK to the current
     * sample instant. This is an encoder-assisted estimate, not ST's
     * Motor Profiler; validate Rs/Ls, phase polarity and the resulting flux. */
    float advance = omega * active.sample_delay_cycles * 0.0001f;
    float lag = omega / active.voltage_pole_rps;
    float restored_d = vd - lag * vq;
    float restored_q = vq + lag * vd;
    float corrected_vq = restored_q * arm_cos_f32(advance) -
                         restored_d * arm_sin_f32(advance);
    float flux = (corrected_vq - active.rs_ohm * iq) / omega - active.ls_h * id;
    if (!isfinite(flux) || flux < 0.00001f || flux > 0.2f) {
        learn_count = 0U;
        return;
    }
    if (!learn_count) { learn_sum = learn_square_sum = 0.0f; learn_speed = omega; }
    learn_sum += flux;
    learn_square_sum += flux * flux;
    if (++learn_count >= LEARN_SAMPLES) {
        float mean = learn_sum / (float)LEARN_SAMPLES;
        float variance = learn_square_sum / (float)LEARN_SAMPLES - mean * mean;
        if (variance <= mean * mean * 0.01f) {
            requested.flux_wb = mean;
            config_pending = true;
            learn_requested = false;
        }
        learn_count = 0U;
    }
}

void StHso_Process(const MotorSampleFrame *frame, bool ready, float encoder_angle,
                   float encoder_velocity, bool encoder_ready)
{
    uint32_t start = DWT->CYCCNT;
    if (!enabled) {
        reject_now(ST_HSO_REASON_OFF);
        live.state = ST_HSO_OFF; was_ready = false; return;
    }
    if (!ready || !frame) {
        live.state = ST_HSO_WAIT_SAMPLE;
        was_ready = false;
        qualified = learn_count = 0U;
        reject_now(ST_HSO_REASON_SAMPLE);
        return;
    }
    bool sequence_gap = was_ready && frame->sequence != last_sequence + 1U;
    if (!was_ready || sequence_gap) config_pending = true;
    was_ready = true;
    last_sequence = frame->sequence;
    if (config_pending) configure();
    if (sequence_gap) live.reason = ST_HSO_REASON_SEQUENCE;
    for (unsigned i = 0U; i < 3U; ++i) {
        if (!isfinite(frame->m1_voltage[i]) || frame->m1_voltage_raw[i] >= 4090U ||
            !isfinite(frame->m1_current[i]) || frame->m1_current_raw[i] < 10U ||
            frame->m1_current_raw[i] > 4085U) {
            live.state = ST_HSO_INPUT_RANGE;
            qualified = learn_count = 0U;
            reject_now(ST_HSO_REASON_RANGE);
            was_ready = false;
            return;
        }
    }
    float ia = frame->m1_current[0];
    float ib = (ia + 2.0f * frame->m1_current[1]) * 0.57735026919f;
    /* Phase-to-ground voltages include PWM common mode: remove it by Clarke.
     * IU/IV are simultaneous; use those rather than the later IW rank. */
    float ua = (2.0f * frame->m1_voltage[0] - frame->m1_voltage[1] -
                frame->m1_voltage[2]) / 3.0f;
    float ub = (frame->m1_voltage[1] - frame->m1_voltage[2]) * 0.57735026919f;
    if (fabsf(ia) >= CURRENT_SCALE || fabsf(ib) >= CURRENT_SCALE ||
        fabsf(ua) >= VOLTAGE_SCALE || fabsf(ub) >= VOLTAGE_SCALE) {
        live.state = ST_HSO_INPUT_RANGE;
        qualified = learn_count = 0U;
        reject_now(ST_HSO_REASON_RANGE);
        was_ready = false;
        return;
    }
    if (learn_requested) learn(ua, ub, ia, ib, encoder_angle, encoder_velocity, encoder_ready);
    if (initialized) {
        Currents_Iab_t iab = {to_q30(ia / CURRENT_SCALE), to_q30(ib / CURRENT_SCALE)};
        Voltages_Uab_t uab = {to_q30(ua / VOLTAGE_SCALE), to_q30(ub / VOLTAGE_SCALE)};
        Voltages_Uab_t emf = IMPEDCORR_run(&impedance, &iab, &uab,
                                          HSO_getEmfSpeed_pu(&observer));
        HSO_run(&observer, &emf, 0);
        live.angle_rad = from_q30(HSO_getAngle_pu(&observer)) * TWO_PI;
        live.emf_hz = from_q30(HSO_getEmfSpeed_pu(&observer)) * FREQUENCY_SCALE;
        live.velocity_rad_s = from_q30(HSO_getSpeedLP_pu(&observer)) *
                               FREQUENCY_SCALE * TWO_PI / (float)pole_pairs;
        live.flux_wb = from_q30(HSO_getFluxAmpl_Wb(&observer));
        live.angle_error_rad = wrap(live.angle_rad - encoder_angle);
        float encoder_hz = encoder_velocity * (float)pole_pairs * INV_TWO_PI;
        float frequency_error = fabsf(live.emf_hz - encoder_hz);
        float angle_error = fabsf(live.angle_error_rad);
        StHsoReason hard = ST_HSO_REASON_NONE;
        if (sequence_gap) hard = ST_HSO_REASON_SEQUENCE;
        else if (!encoder_ready) hard = ST_HSO_REASON_ENCODER;
        else if (!isfinite(encoder_hz) || !isfinite(live.angle_rad) ||
                 !isfinite(live.angle_error_rad) || !isfinite(live.emf_hz) ||
                 !isfinite(live.velocity_rad_s) || !isfinite(live.flux_wb)) hard = ST_HSO_REASON_NUMERIC;
        else if (fabsf(encoder_hz) < MIN_ELECTRICAL_HZ ||
                 fabsf(encoder_hz) >= 0.4f * FREQUENCY_SCALE) hard = ST_HSO_REASON_SPEED;
        else if (angle_error >= 0.785398163f) hard = ST_HSO_REASON_ANGLE; /* 45 deg */
        else if (live.emf_hz * encoder_hz < 0.0f ||
                 frequency_error >= fmaxf(10.0f, fabsf(encoder_hz) * 0.5f)) hard = ST_HSO_REASON_FREQUENCY;
        else if (live.flux_wb <= active.flux_wb * 0.1f ||
                 live.flux_wb >= active.flux_wb * 3.0f) hard = ST_HSO_REASON_FLUX;

        /* Strict admission, separate hold envelope. Crossing the admission
         * boundary once must not require requalification while controlling. */
        bool holding = live.usable;
        StHsoReason entry = hard;
        if (entry == ST_HSO_REASON_NONE) {
            if (angle_error >= 0.261799388f) entry = ST_HSO_REASON_ANGLE;
            else if (frequency_error >= fmaxf(2.0f, fabsf(encoder_hz) * 0.15f)) entry = ST_HSO_REASON_FREQUENCY;
            else if (live.flux_wb <= active.flux_wb * 0.5f ||
                     live.flux_wb >= active.flux_wb * 1.5f) entry = ST_HSO_REASON_FLUX;
        }
        live.entry_reason = entry;
        StHsoReason soft = ST_HSO_REASON_NONE;
        if (angle_error >= (holding ? 0.436332313f : 0.261799388f)) soft = ST_HSO_REASON_ANGLE;
        else if (frequency_error >= fmaxf(holding ? 3.0f : 2.0f,
                     fabsf(encoder_hz) * (holding ? 0.25f : 0.15f))) soft = ST_HSO_REASON_FREQUENCY;
        else if (live.flux_wb <= active.flux_wb * (holding ? 0.4f : 0.5f) ||
                 live.flux_wb >= active.flux_wb * (holding ? 1.6f : 1.5f)) soft = ST_HSO_REASON_FLUX;
        if (hard != ST_HSO_REASON_NONE) {
            if (hard == ST_HSO_REASON_ANGLE) live.qualify_reset_angle_rad = live.angle_error_rad;
            reject_now(hard);
        }
        else if (holding) {
            live.reason = soft;
            live.bad_samples = soft == ST_HSO_REASON_NONE ? 0U : live.bad_samples + 1U;
            if (live.bad_samples >= HOLD_BAD_SAMPLES) {
                live.usable = false;
                reset_admission();
            }
        } else {
            live.reason = soft;
            live.bad_samples = 0U;
            if (soft == ST_HSO_REASON_NONE) {
                if (qualified < QUALIFY_SAMPLES) ++qualified;
                live.qualify_bad_run = 0U;
            } else {
                /* Only a brief 15..20 deg angle excursion can pause admission.
                 * Never hide simultaneous frequency/flux failures behind the
                 * first-reason ANGLE label. Excluded frames earn no progress. */
                bool small_angle_glitch = soft == ST_HSO_REASON_ANGLE &&
                    angle_error < 0.349065850f &&
                    frequency_error < fmaxf(2.0f, fabsf(encoder_hz) * 0.15f) &&
                    live.flux_wb > active.flux_wb * 0.5f && live.flux_wb < active.flux_wb * 1.5f;
                if (qualified != 0U && small_angle_glitch &&
                    live.qualify_paused_samples < QUALIFY_PAUSE_TOTAL &&
                    live.qualify_bad_run < QUALIFY_PAUSE_RUN) {
                    ++live.qualify_paused_samples;
                    ++live.qualify_bad_run;
                } else {
                    if (soft == ST_HSO_REASON_ANGLE) live.qualify_reset_angle_rad = live.angle_error_rad;
                    reset_admission();
                }
            }
            live.usable = qualified >= QUALIFY_SAMPLES;
        }
        if (learn_requested || config_pending) reject_now(ST_HSO_REASON_PARAM);
        live.qualified_samples = qualified;
        if (!learn_requested) live.state = live.usable ? ST_HSO_TRACKING : ST_HSO_OBSERVING;
    } else if (!learn_requested) live.state = ST_HSO_WAIT_PARAM;
    ++live.samples;
    uint32_t elapsed = DWT->CYCCNT - start;
    if (elapsed > live.max_cycles) live.max_cycles = elapsed;
}

void StHso_GetSnapshot(StHsoSnapshot *snapshot)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    *snapshot = live;
    __set_PRIMASK(mask);
}

bool StHso_IsUsable(void) { return live.usable && enabled && !config_pending && !learn_requested; }
bool StHso_CanTakeover(void) { return StHso_IsUsable() && live.entry_reason == ST_HSO_REASON_NONE; }
float StHso_Angle(void) { return live.angle_rad; }
float StHso_Velocity(void) { return live.velocity_rad_s; }

void StHso_RecordControlCycles(uint32_t cycles)
{
    if (cycles > live.max_control_cycles) live.max_control_cycles = cycles;
    if (cycles >= 17000U) {
        ++live.overruns;
        reject_now(ST_HSO_REASON_TIMING);
        live.state = ST_HSO_TIMING;
    }
}

const char *StHso_StateText(StHsoState state)
{
    switch (state) {
    case ST_HSO_OFF: return "OFF";
    case ST_HSO_WAIT_PARAM: return "WAIT FLUX";
    case ST_HSO_WAIT_SAMPLE: return "WAIT ADC";
    case ST_HSO_LEARNING: return "LEARNING";
    case ST_HSO_TRACKING: return "TRACKING";
    case ST_HSO_INPUT_RANGE: return "ADC RANGE";
    case ST_HSO_TIMING: return "LOOP OVERRUN";
    default: return "OBSERVING";
    }
}

const char *StHso_ReasonText(StHsoReason reason)
{
    switch (reason) {
    case ST_HSO_REASON_NONE: return "OK";
    case ST_HSO_REASON_OFF: return "OFF";
    case ST_HSO_REASON_PARAM: return "PARAM";
    case ST_HSO_REASON_SAMPLE: return "SAMPLE";
    case ST_HSO_REASON_SEQUENCE: return "SEQUENCE";
    case ST_HSO_REASON_RANGE: return "ADC RANGE";
    case ST_HSO_REASON_ENCODER: return "ENCODER";
    case ST_HSO_REASON_SPEED: return "SPEED";
    case ST_HSO_REASON_ANGLE: return "ANGLE";
    case ST_HSO_REASON_FREQUENCY: return "FREQUENCY";
    case ST_HSO_REASON_FLUX: return "FLUX";
    case ST_HSO_REASON_NUMERIC: return "NUMERIC";
    case ST_HSO_REASON_TIMING: return "TIMING";
    default: return "UNKNOWN";
    }
}
