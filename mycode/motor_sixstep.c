#include "motor_sixstep.h"
#include "board_diag.h"
#include "main.h"
#include "JerryFOC.h"
#include "motor_math.h"
#include <math.h>

/* 10kHz ADC/current-vector rev-up; TIM7 owns six-step commutation after
 * capture. Raw-cycle differences tolerate DWT wrap at ~25s. */
#define CYCLES_US 170U
static SixStepSnapshot s={.gate='-',.status="6STEP READY"};
static uint32_t started,stepped,previous_stamp,cross_stamp,last_cross;
static uint16_t quiet;
static float period_us,frequency,integral,voltage,previous_bemf,current;
static float forced_angle, id_sum, iq_sum, emf_voltage,tracking_error;
static bool negative,opposite,cross_pending,seen,commute_due;
static bool generic;
bool SixStep_IsGeneric(void) { return generic; }
void SixStep_SetGeneric(bool enable) { generic=enable; if(enable) s.speed_control=false; }
static uint8_t positive_samples,outer_divider;
static const uint8_t high_phase[]={0,0,1,1,2,2};
static const uint8_t low_phase[]={1,2,2,0,0,1};

static float clamp(float x,float lo,float hi) { return fminf(hi,fmaxf(lo,x)); }
static uint16_t percent(float value)
{
    /* Timing ratios are nonnegative and bounded by uint32 DWT / >=833us.
     * Saturate the diagnostic integer without adding float-library calls. */
    uint32_t n=(uint32_t)(value*100.0f);
    return (uint16_t)(n>999U ? 999U : n);
}
static bool active(void) { return s.state>=SIX_ALIGN && s.state<=SIX_RUN; }
/* Shared UI/command boundaries, not part of the commutation ISR. */
/* Round the 12-electrical-Hz command floor upward to a 5-rad/s multiple.
 * ceil(2*pi*12/(5*poles)) == 15/poles+1 for positive integer pole counts. */
__attribute__((noinline)) float SixStep_MinSpeed(void) { return (float)(5*(15/MotorParams_GetActive()->pole_pairs+1)); }
__attribute__((noinline)) float SixStep_MaxSpeed(void) { return 1256.637061f/MotorParams_GetActive()->pole_pairs; } /* 200 Hz */
bool SixStep_AtRest(void) { return !active() && quiet>=2000U; }
void SixStep_GetSnapshot(SixStepSnapshot *out)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq(); *out=s; __set_PRIMASK(mask);
    /* Private state counts 10kHz samples; convert only the public snapshot.
     * Avoid a division and a duplicate counter/store in every fast IRQ. */
    out->startup_ms/=10U;
}

void SixStep_Stop(void)
{
    (void)MotorSampling_SixStepTimer(0U);
    (void)MotorSampling_SetOutputEnabled(false);
    quiet=0U; /* Re-entering this drive must establish fresh electrical rest. */
    if (s.state!=SIX_FAULT) { s.state=SIX_IDLE; s.status="6STEP READY"; s.duty=0; }
    s.target=s.reference=0; integral=0;
}
static void fail(const char *reason)
{
    (void)MotorSampling_SetOutputEnabled(false);
    BoardDiag_Fault(1U,DIAG_SIXSTEP,reason);
    s.state=SIX_FAULT; s.status=reason; SixStep_Stop();
}
static bool schedule(uint32_t microseconds)
{
    if (microseconds<20U || !MotorSampling_SixStepTimer(microseconds)) {
        fail("6STEP TIMING"); return false;
    }
    return true;
}
static void change_step(void)
{
    s.sector=(uint8_t)((s.sector+(negative ? 5U : 1U))%6U);
    stepped=DWT->CYCCNT;
    opposite=cross_pending=seen=false; positive_samples=0;
    previous_stamp=0;
    if (!MotorSampling_SixStepPattern(s.sector,s.duty)) fail("6STEP OUTPUT");
}
void SixStep_OnTimer(void)
{
    if (s.state<SIX_SYNC || s.state>SIX_RUN) return;
    if (MotorSampling_IsGlobalStopped() || !MotorSampling_IsReady() ||
        !MotorSampling_IsOutputRequested()) { SixStep_Stop(); return; }
    if (!commute_due) { fail("6STEP NO ZC"); return; }
    commute_due=false;
    change_step();
    if (!active()) return;
    (void)schedule((uint32_t)(period_us*1.5f)); /* Missing-crossing watchdog, never blind advance. */
}
static bool command(float target,bool speed)
{
    if (!isfinite(target)) return false;
    if (target==0) { SixStep_Stop(); s.speed_control=speed && !generic; return true; }
    if (generic && speed) return false; /* Unknown poles: throttle and electrical Hz only. */
    if ((speed ? fabsf(target)<SixStep_MinSpeed() || fabsf(target)>SixStep_MaxSpeed() : target<0 || target>100) ||
        !MotorSampling_IsReady() || !MotorSampling_IsCalibrated() || !MotorSampling_ResumeGlobalStop()) return false;
    if (active()) {
        if (speed!=s.speed_control || (target<0)!=negative) return false;
        s.target=target; return true;
    }
    if (!SixStep_AtRest()) return false;
    negative=target<0;
    s=(SixStepSnapshot){.state=SIX_ALIGN,.target=target,.sector=0,.duty=0.01f,.gate='A',.speed_control=speed,
        .status="6STEP ALIGN",.startup_current=0.25f,.ramp_hz_s=20};
    if(!generic) s.startup_current=0.5f*MotorParams_GetActive()->rated_current;
    started=stepped=DWT->CYCCNT;
    last_cross=previous_stamp=0; s.quality=outer_divider=0;
    frequency=0; forced_angle=0; id_sum=iq_sum=emf_voltage=tracking_error=0;
    voltage=integral=current=0; commute_due=opposite=cross_pending=seen=false;
    if (!MotorSampling_SixStepPattern(6U,0.5f) || !MotorSampling_SetOutputEnabled(true)) {
        fail("6STEP OUTPUT"); return false;
    }
    return true;
}
bool SixStep_Start(float target) { return command(target,true); }
bool SixStep_Throttle(float percent) { return command(percent,false); }

static void zero_crossing(const MotorSampleFrame *f,float bus)
{
    uint32_t now=DWT->CYCCNT;
    float age=(float)(now-stepped)/CYCLES_US;
    if (!f->m1_sixstep_valid) { s.gate='W'; return; }
    if (seen || age<fmaxf(200.0f,period_us*0.15f)) return;
    unsigned h=high_phase[s.sector],l=low_phase[s.sector],z=3U-h-l;
    const float *v=f->m1_voltage;
    /* Project early Rank2 V to Rank3, 1.412us later, using the 40us RC.
     * OFF: both driven phases and zero-BEMF float target are GND.
     * ON: float target is VBUS/2 at zero BEMF;
     * the correction attenuates its nonzero EMF slightly without moving zero. */
    float level=f->m1_sixstep_off ? 0.0f : h==1U ? bus : l==1U ? 0.0f : bus*0.5f;
    float e=v[z]-(v[h]+v[l])*0.5f;
    e+=(level-v[1])*0.03469f*(z==1U ? 1.0f : -0.5f);
    bool rising=((s.sector & 1U)!=0U)!=negative;
    if (!rising) e=-e;
    s.bemf=rising ? e : -e;
    float hysteresis=fmaxf(0.03f,bus*0.0025f);
    if (e< -hysteresis) { opposite=true; cross_pending=false; positive_samples=0; }
    s.gate=opposite ? 'Z' : 'P';
    if (opposite && !cross_pending && previous_stamp && previous_bemf<0 && e>=0) {
        float fraction=-previous_bemf/(e-previous_bemf);
        /* Remove the low-frequency RC lag; PWM common mode was cancelled
         * using time-aligned phases, not compared with unfiltered VBUS/2. */
        cross_stamp=previous_stamp+(uint32_t)(fraction*(float)(f->m1_sixstep_stamp-previous_stamp))-40U*CYCLES_US;
        cross_pending=true;
    }
    previous_bemf=e; previous_stamp=f->m1_sixstep_stamp;
    if (!cross_pending || e<hysteresis) { positive_samples=0; return; }
    if (++positive_samples<2U) return;
    seen=true;
    float interval=last_cross ? (float)(cross_stamp-last_cross)/CYCLES_US : period_us;
    /* Once BEMF owns timing,
     * normalize by THIS measured interval: at 137% period, a legitimate
     * deceleration puts ZC at 87% of OLD period but only 63.5% of THIS one.
     * The independent 55..145% inter-crossing gate remains unchanged. */
    float ratio=interval/period_us;
    float phase=(float)(cross_stamp-stepped)/(CYCLES_US*interval);
    s.phase_pct=percent(phase);
    s.period_pct=percent(ratio);
    s.period_fraction=(uint8_t)((uint32_t)(ratio*1000.0f)%10U);
    for(unsigned i=0;i<5;++i) s.period_10us[i]=s.period_10us[i+1];
    s.period_10us[5]=(uint16_t)fminf(interval*0.1f,65535.0f);
    s.gate='T';
    if (phase<0.10f || phase>0.85f || ratio<0.55f || ratio>1.45f) {
        s.reject=phase<0.10f ? SIX_REJECT_EARLY : phase>0.85f ? SIX_REJECT_LATE :
                 ratio<0.55f ? SIX_REJECT_SHORT : SIX_REJECT_LONG;
        s.quality=0; last_cross=0;
        fail("6STEP PHASE");
        return;
    }
    /* ADC/RC bandwidth is finite even when throttle has no speed PI. */
    if (interval<833.333f) { fail("6STEP FAST"); return; }
    last_cross=cross_stamp; ++s.crossings;
    s.gate='K';
    /* Commutation must follow the latest validated period. Filtering this
     * delay drags the next switch behind an accelerating rotor. Smooth
     * the reported/PI speed separately, never the 30-degree timer. */
    period_us=interval;
    float velocity=generic ? 166666.667f/interval :
        (negative ? -1.0f : 1.0f)*1047197.551f/(interval*MotorParams_GetActive()->pole_pairs);
    s.velocity+=0.5f*(velocity-s.velocity);
    if (s.state!=SIX_RUN) {
        if (s.quality<12U) ++s.quality;
        /* Only actual six-step BEMF qualifies the speed/throttle handover. */
        if (s.quality>=12U) {
            const MotorParams *m=MotorParams_GetActive();
            s.state=SIX_RUN; s.status="6STEP RUN";
            s.reference=fabsf(s.velocity);
            voltage=s.duty*bus;
            if(s.speed_control) integral=voltage-m->bus_voltage*s.reference/m->rated_speed_mech;
        }
    }
    int32_t delay=(int32_t)((cross_stamp+((uint32_t)(period_us*0.5f)*CYCLES_US))-DWT->CYCCNT);
    if (delay<20*(int32_t)CYCLES_US) { fail("6STEP LATE ZC"); return; }
    commute_due=true; (void)schedule((uint32_t)delay/CYCLES_US);
}
__attribute__((noinline)) static void sine_start(const MotorSampleFrame *f,float bus,const MotorParams *m,uint32_t elapsed)
{
    /* I/F starts independently of the encoder/HSO. Continuous current angle
     * avoids 111ms dwells and ringing of the old 1.5Hz six-step drag. */
    /* All six-step starts use the ESC strength/frequency admission.
     * A preset supplies units/current limits, not a weaker BEMF threshold.
     * Speed targets still cap the forced frequency; never overspeed them
     * merely to obtain stronger BEMF. Throttle needs no pole count here. */
    float capture=emf_voltage>=0.7f && frequency>=12.0f ? frequency : 60.0f;
    if(s.speed_control) capture=fminf(capture,fabsf(s.target)*m->pole_pairs/6.283185307f);
    float step=s.ramp_hz_s*0.0001f;
    if(s.state==SIX_RAMP) frequency+=clamp(capture-frequency,-step,step);
    float omega=(negative ? -6.283185307f : 6.283185307f)*frequency;
    forced_angle+=omega*0.0001f;
    if(forced_angle>=6.283185307f) forced_angle-=6.283185307f;
    if(forced_angle<0) forced_angle+=6.283185307f;
    s.reference=generic ? frequency : omega/m->pole_pairs;
    float sn,cs;
    MotorMath_SinCos(forced_angle,&sn,&cs);
    float ia=f->m1_current[0], ib=(ia+2*f->m1_current[1])*0.577350269f;
    float id=ia*cs+ib*sn, iq=ib*cs-ia*sn;
    float reference=s.startup_current*fminf(1.0f,(float)elapsed/400000.0f);
    float ed=reference-id, eq=-iq;
    float limit=s.state==SIX_ALIGN ? fminf(bus*0.15f,3.0f) : fminf(bus*0.5f,12.0f);
    float ki=0.012f, kp=0.2f;
    id_sum=clamp(id_sum+ki*ed,-limit,limit);
    iq_sum=clamp(iq_sum+ki*eq,-limit,limit);
    float ud=clamp(kp*ed+id_sum,-limit,limit);
    float uq=clamp(kp*eq+iq_sum,-limit,limit);
    float actual_q,actual_d;
    JerryFOC_setAxisVoltage(MOTOR_AXIS_1,uq,ud,forced_angle,&actual_q,&actual_d);
    id_sum+=actual_d-ud; iq_sum+=actual_q-uq;
    /* Uq includes inductive voltage: a strength proxy, never proof of rotor
     * motion. Only actual floating-phase crossings qualify SIX_RUN. Filter
     * tracking error so phase-dependent PWM ripple cannot endlessly reset
     * the 500ms hold despite a stable average current vector. */
    emf_voltage+=0.001f*(fabsf(actual_q)-emf_voltage);
    tracking_error+=0.02f*(fabsf(ed)+fabsf(eq)-tracking_error);
    s.duty=1.732050808f*sqrtf(actual_d*actual_d+actual_q*actual_q)/bus;
    float boundary=negative ? 0.523598776f : 5.759586532f;
    bool tracking=tracking_error<0.3f*reference;
    if(s.state==SIX_ALIGN && elapsed>=1000000U) {
        if(!tracking) { fail("6STEP CURRENT"); return; }
        s.state=SIX_RAMP; s.status="6STEP SINE";
    }
    bool strength=emf_voltage>=fmaxf(0.6f,bus*0.01f);
    if(frequency<12.0f || frequency>capture+0.001f || !tracking || !strength) s.startup_ms=0;
    else if(s.startup_ms<5000U) ++s.startup_ms;
    s.startup_emf=emf_voltage;
    s.gate=!tracking ? 'C' : !strength ? 'E' : 'S';
    if(generic && s.state==SIX_RAMP) {
        /* Current error is a delivery/ripple metric, NOT a rotor observer.
         * Stop accelerating when delivery becomes poor or voltage runs out.
         * With voltage headroom, a repeatedly interrupted capture window
         * searches gradually for a stronger bounded current vector. Freeze
         * its amplitude once 100ms has qualified; retain the full 500ms gate
         * and the following twelve REAL floating-phase zero crossings. */
        bool room=s.duty<0.7f;
        s.ramp_hz_s=tracking && room ? 20.0f : 0;
        if(room && s.startup_ms<1000U)
            s.startup_current=fminf(0.5f,s.startup_current+0.000005f);
    }
    if(s.startup_ms>=5000U && fabsf(forced_angle-boundary)<0.013f) {
        s.state=SIX_SYNC; s.status="6STEP SYNC";
        period_us=1000000.0f/(6.0f*frequency);
        /* A modest resistive seed measured from Ud, independent of Rs.
         * Full Ud causes an excessive torque step in the coupled model. */
        voltage=1.732050808f*(fabsf(actual_q)+0.2f*fabsf(actual_d));
        s.duty=clamp(voltage/bus,0.10f,0.85f);
        /* Rotor/current angle crosses -30deg (forward) or +30deg (reverse).
         * Begin the corresponding 60deg conduction interval, rather than
         * dropping into an arbitrary sector at the sine PWM phase. */
        s.sector=negative ? 0U : 1U;
        change_step();
        if(active()) { commute_due=false; (void)schedule((uint32_t)(period_us*1.5f)); }
    }
}
void SixStep_Process(const MotorSampleFrame *f)
{
    float peak=0;
    for (unsigned i=0;i<3;++i) {
        if (!isfinite(f->m1_current[i]) || !isfinite(f->m1_voltage[i])) { quiet=0; if(active()) fail("6STEP ADC"); return; }
        float a=fabsf(f->m1_current[i]);
        if (a>peak) peak=a;
    }
    if (!active()) {
        const float *v=f->m1_voltage;
        if (peak<0.1f && fabsf(v[0]-v[1])<0.6f && fabsf(v[1]-v[2])<0.6f &&
            fabsf(v[0]-v[2])<0.6f) { if(quiet<2000U) ++quiet; }
        else quiet=0;
        s.velocity=0; return;
    }
    if (MotorSampling_IsGlobalStopped() || !MotorSampling_IsReady() || !MotorSampling_IsOutputRequested()) {
        SixStep_Stop(); return;
    }
    float bus=MotorSampling_GetBusVoltage();
    s.sampling_off=f->m1_sixstep_off;
    const MotorParams *m=MotorParams_GetActive();
    current+=0.1f*(peak-current);
    uint32_t elapsed=(DWT->CYCCNT-started)/CYCLES_US;
    if (s.state!=SIX_RUN && elapsed>8000000U) { fail("6STEP START TIME"); return; }
    if(s.state<=SIX_RAMP) { sine_start(f,bus,m,elapsed); return; }
    if (++outer_divider>=10U) {
        outer_divider=0;
        float requested=fabsf(s.target);
        float feedforward;
        if (s.state==SIX_RUN && s.speed_control) {
            /* Limit relative speed change while BEMF updates are far apart:
             * <= about 5% per 60deg at low speed, up to the old 40rad/s^2.
             * Zero is still immediate coast, handled in command(). */
            float rise=fminf(0.04f,0.00005f*s.velocity*s.velocity*m->pole_pairs);
            float delta=clamp(requested-s.reference,-2.0f*rise,rise);
            s.reference+=delta;
            float error=s.reference-fabsf(s.velocity);
            float candidate=integral+0.001f*m->vel_I*error;
            feedforward=m->bus_voltage*s.reference/m->rated_speed_mech;
            float desired=feedforward+m->vel_P*error+candidate;
            float limit=fminf(bus*0.85f,m->bus_voltage);
            if (desired>0 && desired<limit && peak<m->rated_current) integral=candidate;
            voltage=clamp(desired,0,limit);
        } else if (s.state==SIX_RUN) {
            /* Normalized throttle, bounded by bootstrap/nominal voltage and
             * the same current protection as Speed. No speed PI here. */
            voltage=(generic ? bus*0.85f : fminf(bus*0.85f,m->bus_voltage))*requested*0.01f;
            s.reference=fabsf(s.velocity);
        }
        float cap=generic ? (s.state==SIX_RUN ? 0.5f : fminf(0.5f,1.2f*s.startup_current)) :
            m->rated_current*(s.state==SIX_RUN ? 1.0f : 0.6f);
        float desired=clamp(voltage/bus,0.01f,0.85f);
        if (current>cap) desired=fminf(desired,s.duty*cap/current);
        s.duty+=clamp(desired-s.duty,-0.02f,0.002f);
        if (!MotorSampling_SixStepPattern(s.sector,s.duty)) { fail("6STEP OUTPUT"); return; }
    }
    if (s.state>=SIX_SYNC && s.state<=SIX_RUN) zero_crossing(f,bus);
}
