#include "st_hso.h"
#include "main.h"
#include "hso.h"
#include "impedcorr.h"
#include "motor_math.h"
#include <math.h>
#include <string.h>

#define TWO_PI 6.28318530718f
#define Q30_SCALE 1073741824.0f
#define VOLTAGE_SCALE 72.6f
#define CURRENT_SCALE 6.6f
#define FREQUENCY_SCALE 1000.0f
#define QUALIFY_SAMPLES 2000U
#define LEARN_SAMPLES 10000U
#define START_LIMIT 60000U
#define CAPTURE_LIMIT 20000U /* Independent 2 s after alignment/ramp/learning, total <=8 s. */
#define CAPTURE_SETTLE_SAMPLES 1000U
#define CAPTURE_BAD_BUDGET 50U /* <=2.5% of a 200 ms window, never unbounded pauses. */
#define CAPTURE_ANGLE_LIMIT 1.0471975512f /* 60 deg: margin to the 90 deg I/F pull-out angle. */
#define CAPTURE_DRIFT_LIMIT 0.1745329252f /* 10 deg from a FROZEN window reference. */
#define VOLTAGE_RANK_SKEW_S (60.0f/42500000.0f) /* 47.5 sample + 12.5 conversion. */

static HSO_Obj observer __attribute__((aligned(8)));
static IMPEDCORR_Obj impedance __attribute__((aligned(8)));
static StHsoConfig requested, active;
static volatile bool enabled=true, config_pending;
static bool initialized, was_ready, running, previous_valid;
static uint32_t last_sequence, quiet, elapsed, qualified, learn_count, blend;
static float learn_sum, learn_square_sum, previous_angle;
static float learn_d_sum,learn_q_sum;
static uint32_t learn_cycle_count,learn_cycle_length;
static unsigned learn_settle;
static uint32_t capture_samples,capture_bad,capture_settle,capture_started;
static float capture_hz,capture_strength,capture_anchor,capture_phase;
static float voltage_align_gain;
static float forced_angle, forced_speed, capture_speed, align_current, angle_offset;
static int pole_pairs=1;
static StHsoState phase;
static StHsoSnapshot live;

__attribute__((optimize("O2"))) static float wrap(float a)
{
    if (!isfinite(a)) return NAN;
    while (a>3.14159265359f) a-=TWO_PI;
    while (a< -3.14159265359f) a+=TWO_PI;
    return a;
}
static fixp30_t to_q30(float x) { return (fixp30_t)(x*Q30_SCALE); }
static float from_q30(fixp30_t x) { return (float)x/Q30_SCALE; }
__attribute__((noinline,optimize("Os"))) static void reject(StHsoReason reason)
{
    running=live.usable=false;
    live.reason=reason;
    /* Keep the rejected measurement condition visible after a timeout. */
    if (reason!=ST_HSO_REASON_STARTUP && reason!=ST_HSO_REASON_LEARN_TIMEOUT &&
        reason!=ST_HSO_REASON_CAPTURE_TIMEOUT) live.entry_reason=reason;
    live.state=phase=ST_HSO_FAULT;
    qualified=0;
}
__attribute__((optimize("Os"))) static bool valid_config(const StHsoConfig *c)
{
    return c && c->rs_ohm>=0.01f && c->rs_ohm<=100 &&
        c->ls_h>=0.000001f && c->ls_h<=0.1f &&
        c->flux_wb>=0 && c->flux_wb<=0.2f &&
        c->voltage_pole_rps>=100 && c->voltage_pole_rps<=1000000 &&
        fabsf(c->sample_delay_cycles)<=0.25f;
}
__attribute__((optimize("Os"))) void StHso_Init(const MotorParams *motor)
{
    __HAL_RCC_CORDIC_CLK_ENABLE();
    memset(&live,0,sizeof(live));
    initialized=was_ready=running=previous_valid=false;
    quiet=elapsed=qualified=learn_count=blend=0;
    forced_angle=forced_speed=angle_offset=0;
    pole_pairs=motor->pole_pairs>0 ? motor->pole_pairs : 1;
    align_current=0.5f*motor->rated_current;
    requested=(StHsoConfig){motor->Rs,motor->Ls,motor->psi_f,25000.0f,VOLTAGE_RANK_SKEW_S*10000};
    config_pending=enabled=true;
    live.state=phase=ST_HSO_WAIT_SAMPLE;
    live.reason=live.entry_reason=ST_HSO_REASON_SAMPLE;
    live.angle_error_rad=NAN;
}
__attribute__((optimize("Os"))) bool StHso_SetConfig(const StHsoConfig *c)
{
    if (!valid_config(c)) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=!running;
    if (ok) { requested=*c; config_pending=true; quiet=0; }
    __set_PRIMASK(mask); return ok;
}
__attribute__((optimize("Os"))) void StHso_GetConfig(StHsoConfig *c)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq(); *c=requested; __set_PRIMASK(mask);
}
__attribute__((noinline,optimize("Os"))) void StHso_Stop(void)
{
    running=live.usable=false;
    quiet=qualified=learn_count=0;
    forced_speed=0;
}
__attribute__((optimize("Os"))) void StHso_Enable(bool enable)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    StHso_Stop(); enabled=enable; config_pending=true;
    live.state=phase=enable ? ST_HSO_WAIT_SAMPLE : ST_HSO_OFF;
    live.reason=live.entry_reason=enable ? ST_HSO_REASON_SAMPLE : ST_HSO_REASON_OFF;
    __set_PRIMASK(mask);
}
__attribute__((optimize("Os"))) bool StHso_LearnFlux(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=enabled && !running;
    if (ok) { requested.flux_wb=0; config_pending=true; quiet=0; }
    __set_PRIMASK(mask); return ok;
}
/* Round the 5-electrical-Hz command floor upward to a 5-rad/s multiple.
 * ceil(2*pi/poles) == 6/poles+1 for positive integer pole counts. */
float StHso_MinSpeed(void) { return (float)(5*(6/pole_pairs+1)); }
bool StHso_AtRest(void) { return !running && quiet>=2000U; }
__attribute__((optimize("Os"))) bool StHso_ValidTarget(float target)
{
    float hz=target*(float)pole_pairs/TWO_PI;
    return isfinite(target) && fabsf(target)>=StHso_MinSpeed() && fabsf(hz)>=5 && fabsf(hz)<400 &&
        (!running || target*capture_speed>0);
}
__attribute__((optimize("Os"))) bool StHso_Start(float target)
{
    if (!enabled || !valid_config(&requested) || !StHso_ValidTarget(target)) return false;
    if (running) return target*capture_speed>0;
    if (!was_ready || quiet<2000U || config_pending) return false;
    running=true; phase=ST_HSO_ALIGN; live.state=phase;
    elapsed=qualified=learn_count=blend=0;
    learn_cycle_count=learn_cycle_length=0;
    learn_d_sum=learn_q_sum=0;
    learn_settle=2U;
    capture_samples=capture_bad=capture_settle=capture_started=0;
    capture_hz=capture_strength=capture_phase=0;
    live.capture_resets=0; live.capture_reason=ST_HSO_REASON_NONE;
    live.capture_angle_rad=live.capture_frequency_error_hz=live.capture_emf_ratio=live.capture_drift_rad=0;
    quiet=0;
    forced_angle=forced_speed=angle_offset=0;
    capture_speed=copysignf(fminf(fabsf(target),TWO_PI*15/(float)pole_pairs),target);
    config_pending=true; previous_valid=false;
    live.usable=false; live.bad_samples=0; live.reason=live.entry_reason=ST_HSO_REASON_NONE;
    return true;
}
__attribute__((optimize("Os"))) static void configure(void)
{
    active=requested; config_pending=false; initialized=false; previous_valid=false;
    live.reference_flux_wb=active.flux_wb;
    live.emf_offset_v=live.emf_offset_limit_v=0;
    /* exp(x)-1, Pade [2/2]. At this board's x=0.035294 the approximation
     * error in exact arithmetic is <1 nV at 20 V; ADC/float rounding dominate.
     * No expf or additional coefficient work in the hot path. */
    float x=active.voltage_pole_rps*VOLTAGE_RANK_SKEW_S;
    voltage_align_gain=12*x/(12-6*x+x*x);
    IMPEDCORR_init(&impedance,sizeof(impedance));
    IMPEDCORR_Params ip={active.voltage_pole_rps,10000.0f,FREQUENCY_SCALE,
                         active.sample_delay_cycles,CURRENT_SCALE,VOLTAGE_SCALE};
    IMPEDCORR_setParams(&impedance,&ip);
    IMPEDCORR_setRs_si(&impedance,active.rs_ohm);
    IMPEDCORR_setLs_si(&impedance,active.ls_h);
    if (active.flux_wb<0.00001f) return;
    HSO_init(&observer,sizeof(observer));
    HSO_Params hp={.Flux_Wb=active.flux_wb,.CrossOver_Hz=20,.isrTime_s=0.0001f,
        .FullScaleVoltage_V=VOLTAGE_SCALE,.FullScaleFreq_Hz=FREQUENCY_SCALE,
        .speedPole_rps=100,.flag_FilterAdaptive=true,.Filter_adapt_fmin_Hz=10,
        .Filter_adapt_fmax_Hz=500,.CheckDirBW_Hz=0.5f};
    HSO_setParams(&observer,&hp);
    HSO_setAngle_pu(&observer,to_q30(forced_angle/TWO_PI));
    HSO_setFlag_EnableOffsetUpdate(&observer,false);
    initialized=true;
}
__attribute__((optimize("Os"))) static void learn_flux(Voltages_Uab_t emf,float hz)
{
    /* Average corrected EMF in the rotating virtual frame over a WHOLE
     * electrical cycle. Include every sample: do not select only favourable
     * PWM/noise samples, or require 10000 perfect consecutive instantaneous
     * phase tests. Direction/phase/strength and variance still gate admission. */
    if (!learn_cycle_length) learn_cycle_length=(uint32_t)(10000.0f/fabsf(hz)+0.5f);
    float a=from_q30(emf.A)*VOLTAGE_SCALE,b=from_q30(emf.B)*VOLTAGE_SCALE;
    float sn,cs;
    MotorMath_SinCos(forced_angle,&sn,&cs);
    learn_d_sum+=a*cs+b*sn; learn_q_sum+=b*cs-a*sn;
    if (++learn_cycle_count<learn_cycle_length) return;
    float d=learn_d_sum/(float)learn_cycle_count,q=learn_q_sum/(float)learn_cycle_count;
    learn_d_sum=learn_q_sum=0; learn_cycle_count=0;
    float flux=sqrtf(d*d+q*q)/(fabsf(hz)*TWO_PI);
    live.flux_wb=flux; /* Useful even when the estimate cannot be accepted. */
    StHsoReason bad=ST_HSO_REASON_NONE;
    if (learn_settle) { --learn_settle; bad=ST_HSO_REASON_SETTLE; }
    else if (!isfinite(flux) || flux<0.00001f || flux>0.2f) bad=ST_HSO_REASON_FLUX;
    else if (q*hz<=0) bad=ST_HSO_REASON_FREQUENCY;
    else if (fabsf(q)<0.2f) bad=ST_HSO_REASON_EMF_LOW;
    else if (fabsf(d)>=fabsf(q)*0.577350269f) bad=ST_HSO_REASON_ANGLE;
    if (bad==ST_HSO_REASON_NONE) {
        if (!learn_count) learn_sum=learn_square_sum=0;
        learn_sum+=flux*(float)learn_cycle_length;
        learn_square_sum+=flux*flux*(float)learn_cycle_length;
        learn_count+=learn_cycle_length;
        if (learn_count>=LEARN_SAMPLES) {
            float mean=learn_sum/(float)learn_count;
            float variance=learn_square_sum/(float)learn_count-mean*mean;
            if (variance<=mean*mean*0.01f) {
                requested.flux_wb=mean; config_pending=true; phase=ST_HSO_CAPTURE;
            } else bad=ST_HSO_REASON_FLUX_RIPPLE;
            learn_count=0;
        }
    } else learn_count=0;
    live.reason=bad;
    if (bad!=ST_HSO_REASON_NONE || phase==ST_HSO_CAPTURE) live.entry_reason=bad;
}
__attribute__((optimize("Os"))) static void qualify_capture(StHsoReason bad,float hz,float expected,
                                                         bool raw_entry,bool raw_frequency_ok)
{
    if (capture_settle<CAPTURE_SETTLE_SAMPLES) {
        if (++capture_settle==CAPTURE_SETTLE_SAMPLES) live.entry_reason=ST_HSO_REASON_NONE;
        live.reason=ST_HSO_REASON_SETTLE;
    } else {
        /* The I/F current-vector angle is not a measured rotor angle. A
         * bounded steady load angle is legitimate; phase slip is not. Freeze
         * this reference for the window, never chase a drift with a low-pass
         * average or reset the reference on each bad sample. */
        if (!capture_samples) capture_anchor=capture_phase;
        float drift=wrap(live.angle_error_rad-capture_anchor);
        if (bad==ST_HSO_REASON_NONE && fabsf(drift)>CAPTURE_DRIFT_LIMIT)
            bad=ST_HSO_REASON_PHASE_DRIFT;
        live.reason=bad;
        if (bad!=ST_HSO_REASON_NONE) live.entry_reason=bad;
        ++capture_samples;
        if (bad==ST_HSO_REASON_NONE) { ++qualified; live.bad_samples=0; }
        else { ++capture_bad; ++live.bad_samples; }
        if (capture_bad>CAPTURE_BAD_BUDGET || live.bad_samples>=20U) {
            ++live.capture_resets; live.capture_reason=bad;
            live.capture_angle_rad=live.angle_error_rad;
            live.capture_drift_rad=drift;
            live.capture_frequency_error_hz=capture_hz-hz;
            live.capture_emf_ratio=sqrtf(capture_strength)/fabsf(expected);
            capture_samples=capture_bad=qualified=live.bad_samples=0;
        }
    }
    if (capture_samples>=QUALIFY_SAMPLES && qualified>=QUALIFY_SAMPLES-CAPTURE_BAD_BUDGET &&
        bad==ST_HSO_REASON_NONE) {
        if (raw_entry) {
            angle_offset=wrap(live.angle_rad-forced_angle);
            phase=ST_HSO_CONTROL; qualified=QUALIFY_SAMPLES; blend=0; live.usable=true;
        } else live.reason=live.entry_reason=raw_frequency_ok ? ST_HSO_REASON_FLUX : ST_HSO_REASON_FREQUENCY;
    }
}
/* Admission and diagnostics share size-optimized code; the Clarke,
 * impedance compensation and official observer arithmetic stay at -O2. */
__attribute__((noinline,optimize("Os"))) static void check_observer(Voltages_Uab_t emf,float forced_hz)
{
    bool control=phase==ST_HSO_CONTROL;
    float hz=control ? live.velocity_rad_s*(float)pole_pairs/TWO_PI : forced_hz;
    live.angle_error_rad=wrap(live.angle_rad-(control ?
        previous_angle+live.emf_hz*TWO_PI*0.0001f : forced_angle));
    float raw_ferr=fabsf(live.emf_hz-hz),ferr=raw_ferr,aerr=fabsf(live.angle_error_rad);
    float frequency_tolerance=fmaxf(control ? 3 : 1,fabsf(hz)*(control ? 0.25f : 0.15f));
    StHsoReason bad=ST_HSO_REASON_NONE;
    if (!isfinite(live.angle_rad) || !isfinite(live.emf_hz) ||
        !isfinite(live.velocity_rad_s) || !isfinite(live.flux_wb)) bad=ST_HSO_REASON_NUMERIC;
    else if (fabsf(live.emf_hz)>=400 || (control && fabsf(live.emf_hz)<1)) bad=ST_HSO_REASON_SPEED;
    else if (control && (live.flux_wb<active.flux_wb*0.1f || live.flux_wb>active.flux_wb*3)) bad=ST_HSO_REASON_FLUX;
    else if (control && previous_valid && aerr>=0.785398163f) bad=ST_HSO_REASON_ANGLE;
    previous_angle=live.angle_rad; previous_valid=true;
    if (bad!=ST_HSO_REASON_NONE && (phase!=ST_HSO_ALIGN || bad==ST_HSO_REASON_NUMERIC)) { reject(bad); return; }
    float ea=from_q30(emf.A)*VOLTAGE_SCALE,eb=from_q30(emf.B)*VOLTAGE_SCALE;
    float expected=active.flux_wb*hz*TWO_PI;
    float raw_strength=ea*ea+eb*eb,strength=raw_strength;
    float minimum=fmaxf(0.04f,expected*expected*0.16f),maximum=expected*expected*2.56f;
    /* Capture evaluates sustained independent observer frequency and EMF
     * energy, rather than raw derivative/PWM ripple. CONTROL keeps its
     * existing per-frame protection. Last-frame strict entry remains. */
    if (phase==ST_HSO_CAPTURE) {
        capture_hz+=0.005f*(live.emf_hz-capture_hz); /* ~20 ms. */
        capture_strength+=0.02f*(strength-capture_strength); /* ~5 ms. */
        capture_phase=wrap(capture_phase+0.005f*wrap(live.angle_error_rad-capture_phase));
        ferr=fabsf(capture_hz-hz); strength=capture_strength;
    }
    if (aerr>(control ? 0.436332313f : CAPTURE_ANGLE_LIMIT)) bad=ST_HSO_REASON_ANGLE;
    else if (live.emf_hz*hz<=0 || ferr>frequency_tolerance) bad=ST_HSO_REASON_FREQUENCY;
    else if (live.flux_wb<active.flux_wb*(control ? 0.4f : 0.5f) ||
             live.flux_wb>active.flux_wb*(control ? 1.6f : 1.5f)) bad=ST_HSO_REASON_FLUX;
    if (strength<minimum || strength>maximum)
        bad=ST_HSO_REASON_FLUX;
    live.reason=bad;
    if (control || bad!=ST_HSO_REASON_NONE) live.entry_reason=bad;
    if (phase==ST_HSO_CAPTURE) {
        bool raw_frequency_ok=raw_ferr<=frequency_tolerance;
        qualify_capture(bad,hz,expected,
            raw_frequency_ok && raw_strength>=minimum && raw_strength<=maximum,raw_frequency_ok);
    } else if (control) {
        if (bad!=ST_HSO_REASON_NONE) { if (++live.bad_samples>=20U) { reject(bad); return; } }
        else live.bad_samples=0;
        if (blend<1000U) ++blend;
    }
}
__attribute__((optimize("O2"))) void StHso_Process(const MotorSampleFrame *f,bool ready,float encoder_angle,
                   float encoder_velocity,bool encoder_ready)
{
    (void)encoder_angle; (void)encoder_velocity; (void)encoder_ready;
    uint32_t start=DWT->CYCCNT;
    if (!ready || !f) {
        quiet=0; was_ready=false;
        if (running) reject(ST_HSO_REASON_SAMPLE);
        else { live.state=ST_HSO_WAIT_SAMPLE; live.reason=ST_HSO_REASON_SAMPLE; }
        return;
    }
    bool gap=was_ready && f->sequence!=last_sequence+1U;
    was_ready=true; last_sequence=f->sequence;
    if (gap) quiet=0;
    if (gap && running) { reject(ST_HSO_REASON_SEQUENCE); return; }
    for (unsigned i=0;i<3;++i) {
        if (!isfinite(f->m1_voltage[i]) || f->m1_voltage_raw[i]>=4090U ||
            !isfinite(f->m1_current[i]) || f->m1_current_raw[i]<10U || f->m1_current_raw[i]>4085U) {
            quiet=0; if (running) reject(ST_HSO_REASON_RANGE);
            else { live.state=ST_HSO_INPUT_RANGE; live.reason=ST_HSO_REASON_RANGE; }
            return;
        }
    }
    if (config_pending && enabled) configure();
    float ia=f->m1_current[0],ib=(ia+2*f->m1_current[1])*0.57735026919f;
    float vu=f->m1_voltage[0],vv=f->m1_voltage[1],vw=f->m1_voltage[2];
    if (f->m1_voltage_high_plateau) {
        /* V was acquired at rank 2, U/W at rank 3. While all inputs equal
         * VBUS, invert the known RC evolution to put U/W at V's timestamp.
         * Differencing mismatched timestamps otherwise converts PWM common
         * mode into a false DC EMF proportional to VBUS. Raw ADC is untouched. */
        float bus=MotorSampling_GetBusVoltage();
        if (!(bus>0 && bus<VOLTAGE_SCALE)) { reject(ST_HSO_REASON_SAMPLE); return; }
        vu+=(vu-bus)*voltage_align_gain; vw+=(vw-bus)*voltage_align_gain;
    }
    float ua=(2*vu-vv-vw)/3;
    float ub=(vv-vw)*0.57735026919f;
    if (fabsf(ia)>=CURRENT_SCALE || fabsf(ib)>=CURRENT_SCALE ||
        fabsf(ua)>=VOLTAGE_SCALE || fabsf(ub)>=VOLTAGE_SCALE) {
        quiet=0; reject(ST_HSO_REASON_RANGE); return;
    }
    if (!running) {
        if (ua*ua+ub*ub<0.09f && ia*ia+ib*ib<0.01f) { if (quiet<2000U) ++quiet; }
        else quiet=0;
        if (!enabled) live.state=ST_HSO_OFF;
        else if (phase!=ST_HSO_FAULT) {
            live.state=quiet>=2000U ? ST_HSO_READY : ST_HSO_WAIT_SAMPLE;
            live.reason=quiet>=2000U ? ST_HSO_REASON_NONE : ST_HSO_REASON_STANDSTILL;
        }
        return;
    }
    ++elapsed;
    if (phase==ST_HSO_CAPTURE && !capture_started) capture_started=elapsed;
    if (phase!=ST_HSO_CONTROL && (phase==ST_HSO_CAPTURE ?
        elapsed-capture_started>CAPTURE_LIMIT : elapsed>START_LIMIT)) {
        reject(phase==ST_HSO_LEARNING ? ST_HSO_REASON_LEARN_TIMEOUT :
               phase==ST_HSO_CAPTURE ? ST_HSO_REASON_CAPTURE_TIMEOUT : ST_HSO_REASON_STARTUP);
        return;
    }
    if (phase==ST_HSO_ALIGN && elapsed>=10000U) {
        if (ia<align_current*0.5f || fabsf(ib)>align_current*0.5f) {
            reject(ST_HSO_REASON_STARTUP); return;
        }
        phase=ST_HSO_RAMP;
    }
    if (phase==ST_HSO_RAMP) {
        float delta=capture_speed-forced_speed;
        forced_speed+=fmaxf(-0.002f,fminf(0.002f,delta)); /* 20 rad/s^2. */
        if (fabsf(delta)<=0.002f) {
            forced_speed=capture_speed;
            phase=active.flux_wb<0.00001f ? ST_HSO_LEARNING : ST_HSO_CAPTURE;
            if (phase==ST_HSO_LEARNING) live.reason=live.entry_reason=ST_HSO_REASON_SETTLE;
        }
    }
    if (phase!=ST_HSO_ALIGN && phase!=ST_HSO_CONTROL) {
        forced_angle+=forced_speed*(float)pole_pairs*0.0001f;
        if (forced_angle>=TWO_PI) forced_angle-=TWO_PI;
        if (forced_angle<0) forced_angle+=TWO_PI;
    }
    float forced_hz=forced_speed*(float)pole_pairs/TWO_PI;
    Currents_Iab_t iab={to_q30(ia/CURRENT_SCALE),to_q30(ib/CURRENT_SCALE)};
    Voltages_Uab_t uab={to_q30(ua/VOLTAGE_SCALE),to_q30(ub/VOLTAGE_SCALE)};
    fixp30_t frequency=phase==ST_HSO_CONTROL ? HSO_getEmfSpeed_pu(&observer) : to_q30(forced_hz/FREQUENCY_SCALE);
    Voltages_Uab_t emf=IMPEDCORR_run(&impedance,&iab,&uab,frequency);
    if (phase==ST_HSO_LEARNING) {
        learn_flux(emf,forced_hz);
        live.angle_error_rad=NAN;
    }
    if (initialized) {
        /* MCSDK mc_tasks_hso: offset adaptation is also enabled in open-loop
         * current mode. Waiting for CONTROL creates a circular admission gate:
         * DC EMF bias distorts flux/phase, so CAPTURE can never reach CONTROL.
         * Keep alignment frozen; all rotating phases use independent HSO
         * adaptation with its original public-library limits. */
        HSO_setFlag_EnableOffsetUpdate(&observer,phase!=ST_HSO_ALIGN);
        HSO_setFlag_TrackAngle(&observer,phase!=ST_HSO_ALIGN);
        if (phase==ST_HSO_ALIGN) HSO_setAngle_pu(&observer,0);
        HSO_run(&observer,&emf,0);
        live.angle_rad=from_q30(HSO_getAngle_pu(&observer))*TWO_PI;
        live.emf_hz=from_q30(HSO_getEmfSpeed_pu(&observer))*FREQUENCY_SCALE;
        live.velocity_rad_s=from_q30(HSO_getSpeedLP_pu(&observer))*FREQUENCY_SCALE*TWO_PI/(float)pole_pairs;
        live.flux_wb=from_q30(HSO_getFluxAmpl_Wb(&observer));
        Vector_ab_t offset=HSO_getOffset_ab_pu(&observer);
        float ox=from_q30(offset.A)*VOLTAGE_SCALE,oy=from_q30(offset.B)*VOLTAGE_SCALE;
        live.emf_offset_v=fmaxf(fabsf(ox),fabsf(oy));
        live.emf_offset_limit_v=from_q30(HSO_getOffsetLimit(&observer))*VOLTAGE_SCALE;
        check_observer(emf,forced_hz);
        if (!running) return;
    }
    live.state=phase; live.qualified_samples=phase==ST_HSO_LEARNING ? learn_count :
        phase==ST_HSO_CAPTURE ? (capture_samples<QUALIFY_SAMPLES ? capture_samples : QUALIFY_SAMPLES) : qualified;
    ++live.samples;
    uint32_t duration=DWT->CYCCNT-start;
    if (duration>live.max_cycles) live.max_cycles=duration;
}
__attribute__((optimize("Os"))) void StHso_GetSnapshot(StHsoSnapshot *s)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq(); *s=live; __set_PRIMASK(mask);
}
bool StHso_IsUsable(void) { return running && live.usable && enabled && !config_pending; }
bool StHso_CanTakeover(void) { return StHso_IsUsable(); }
bool StHso_IsStarting(void) { return running && phase!=ST_HSO_CONTROL; }
bool StHso_IsRunning(void) { return running; }
float StHso_Angle(void) { return live.angle_rad; }
float StHso_Velocity(void) { return live.velocity_rad_s; }
float StHso_ControlAngle(void)
{
    return phase==ST_HSO_CONTROL ? live.angle_rad-angle_offset*(1-fminf((float)blend/500,1)) : forced_angle;
}
float StHso_ControlVelocity(void)
{
    if (!running) return quiet>=2000U ? 0.0f : live.velocity_rad_s;
    return phase==ST_HSO_CONTROL ? live.velocity_rad_s : forced_speed;
}
float StHso_CurrentD(void)
{
    float weight=phase==ST_HSO_ALIGN ? fminf((float)elapsed/2500,1) :
        phase==ST_HSO_CONTROL ? 1-(float)blend/1000 : 1;
    return running ? align_current*weight : 0;
}
__attribute__((optimize("Os"))) void StHso_BlendCurrent(float *d,float *q)
{
    float amplitude=StHso_CurrentD();
    if (phase==ST_HSO_CONTROL && blend<1000U) {
        /* Preserve the physical I/F current vector as the reference frame
         * rotates. Crossfade it to speed-PI torque over 100 ms; changing only
         * angle/Id would discard the startup torque at a load angle. */
        float turn=angle_offset*fminf((float)blend/500,1);
        float sn,cs;
        MotorMath_SinCos(turn,&sn,&cs);
        *d=amplitude*cs;
        *q=*q*((float)blend/1000)-amplitude*sn;
    } else { *d=amplitude; if (phase!=ST_HSO_CONTROL) *q=0; }
}
void StHso_RecordControlCycles(uint32_t cycles)
{
    if (cycles>live.max_control_cycles) live.max_control_cycles=cycles;
    if (cycles>=17000U) { ++live.overruns; if (running) reject(ST_HSO_REASON_TIMING); }
}
__attribute__((optimize("Os"))) const char *StHso_StateText(StHsoState state)
{
    switch (state) {
    case ST_HSO_OFF:return "OFF";
    case ST_HSO_WAIT_PARAM:return "WAIT PARAM";
    case ST_HSO_WAIT_SAMPLE:return "WAIT ADC/STOP";
    case ST_HSO_READY:return "READY";
    case ST_HSO_ALIGN:return "DC ALIGN";
    case ST_HSO_RAMP:return "I/F RAMP";
    case ST_HSO_LEARNING:return "I/F FLUX";
    case ST_HSO_CAPTURE:return "CAPTURE";
    case ST_HSO_CONTROL:return "CONTROL";
    case ST_HSO_FAULT:return "START/HSO FAIL";
    case ST_HSO_INPUT_RANGE:return "ADC RANGE";
    default:return "OBSERVING";
    }
}
__attribute__((optimize("Os"))) const char *StHso_ReasonText(StHsoReason reason)
{
    switch (reason) {
    case ST_HSO_REASON_NONE:return "OK";
    case ST_HSO_REASON_OFF:return "OFF";
    case ST_HSO_REASON_PARAM:return "PARAM";
    case ST_HSO_REASON_SAMPLE:return "SAMPLE";
    case ST_HSO_REASON_SEQUENCE:return "SEQUENCE";
    case ST_HSO_REASON_RANGE:return "ADC RANGE";
    case ST_HSO_REASON_ENCODER:return "ENCODER";
    case ST_HSO_REASON_SPEED:return "SPEED";
    case ST_HSO_REASON_ANGLE:return "ANGLE";
    case ST_HSO_REASON_FREQUENCY:return "FREQUENCY";
    case ST_HSO_REASON_FLUX:return "FLUX";
    case ST_HSO_REASON_NUMERIC:return "NUMERIC";
    case ST_HSO_REASON_TIMING:return "TIMING";
    case ST_HSO_REASON_STARTUP:return "START TIMEOUT";
    case ST_HSO_REASON_STANDSTILL:return "WAIT FOR STOP";
    case ST_HSO_REASON_SETTLE:return "EMF SETTLE";
    case ST_HSO_REASON_EMF_LOW:return "EMF LOW";
    case ST_HSO_REASON_FLUX_RIPPLE:return "FLUX RIPPLE";
    case ST_HSO_REASON_LEARN_TIMEOUT:return "FLUX TIMEOUT";
    case ST_HSO_REASON_CAPTURE_TIMEOUT:return "LOCK TIMEOUT";
    case ST_HSO_REASON_PHASE_DRIFT:return "PHASE DRIFT";
    default:return "UNKNOWN";
    }
}
