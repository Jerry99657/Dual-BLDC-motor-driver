/* Real six-step controller, synthetic terminal-voltage/current traces.
 * No encoder or ST observer symbol is linked; not a mechanical motor model. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "motor_sixstep.h"

HostDwt host_dwt;
uint32_t host_primask;
static uint64_t us,deadline,step_at;
static float bus=20,duty,injected_current;
static unsigned sector;
static bool requested,ready=true,stopped,inject_signal=true,window=true;
static unsigned timer_calls;
static bool sine_drive;
static bool sample_off;
static float sine_phase[3];
static float sine_angle,sine_alpha,sine_beta,model_alpha,model_beta;
static float sine_flux=0.0075f;
static const uint8_t hp[]={0,0,1,1,2,2},lp[]={1,2,2,0,0,1};
static MotorSampleFrame frame;
static float held_period=8333.0f;
static float cross_fraction=0.5f;
static unsigned previous_sector=255;
static SixStepState last_state;
uint32_t HAL_GetTick(void) { return (uint32_t)(us/1000U); }
bool MotorSampling_IsReady(void) { return ready; }
bool MotorSampling_IsCalibrated(void) { return true; }
bool MotorSampling_IsGlobalStopped(void) { return stopped; }
bool MotorSampling_ResumeGlobalStop(void) { return !stopped; }
bool MotorSampling_IsOutputRequested(void) { return requested; }
float MotorSampling_GetBusVoltage(void) { return bus; }
bool MotorSampling_SetOutputEnabled(bool e) { requested=e; return true; }
bool MotorSampling_SixStepPattern(uint8_t s,float d) {
    assert(s<=6 && d>=0 && d<=0.85f);
    sine_drive=s==6;
    sample_off=!sine_drive && (d<0.25f || (sample_off && d<0.30f));
#ifdef SIXSTEP_FORCE_ON
    sample_off=false;
#endif
    if(sine_drive) return ready && !stopped;
    if(s!=previous_sector) { step_at=us; previous_sector=s; }
    sector=s; duty=d; return ready && !stopped;
}
void JerryFOC_setAxisVoltage(MotorAxis axis,float uq,float ud,float angle,float *aq,float *ad) {
    (void)axis;
    if(aq) *aq=uq;
    if(ad) *ad=ud;
    float alpha=ud*cosf(angle)-uq*sinf(angle), beta=ud*sinf(angle)+uq*cosf(angle);
    sine_angle=angle; sine_alpha=alpha; sine_beta=beta;
    sine_phase[0]=0.5f+alpha/bus;
    sine_phase[1]=0.5f+(-0.5f*alpha+0.866025404f*beta)/bus;
    sine_phase[2]=0.5f+(-0.5f*alpha-0.866025404f*beta)/bus;
}
bool MotorSampling_SixStepTimer(uint32_t delay) {
    assert(!delay || (delay>=20 && delay<=260000));
    if(delay) { deadline=us+delay; ++timer_calls; }
    else deadline=0;
    return true;
}
static SixStepSnapshot state(void) { SixStepSnapshot s; SixStep_GetSnapshot(&s); return s; }
static void sample(void) {
    SixStepSnapshot s=state();
    if(s.state==SIX_RAMP && last_state==SIX_ALIGN) step_at=us-100U;
    last_state=s.state;
    frame=(MotorSampleFrame){.m1_sixstep_valid=window,.m1_sixstep_off=sample_off,.m1_sixstep_stamp=host_dwt.CYCCNT};
    frame.m1_current[0]=injected_current; frame.m1_current[1]=-injected_current;
    for(unsigned i=0;i<3;++i) frame.m1_voltage[i]=bus*0.5f;
    if(requested && sine_drive) {
        /* Averaged RL circuit, with an imposed rotor for timing-unit tests.
         * The separate plant test does not impose rotor motion. */
        const MotorParams *m=MotorParams_GetActive();
        float omega=SixStep_IsGeneric() ? s.reference*6.283185307f : s.reference*m->pole_pairs;
        float flux=inject_signal ? sine_flux : 0;
        float ea=-flux*omega*sinf(sine_angle), eb=flux*omega*cosf(sine_angle);
        model_alpha+=(sine_alpha-m->Rs*model_alpha-ea)*0.0001f/m->Ls;
        model_beta+=(sine_beta-m->Rs*model_beta-eb)*0.0001f/m->Ls;
        frame.m1_current[0]=model_alpha;
        frame.m1_current[1]=-0.5f*model_alpha+0.866025404f*model_beta;
        frame.m1_current[2]=-frame.m1_current[0]-frame.m1_current[1];
    }
    if(requested && !sine_drive && s.state>=SIX_SYNC && s.state<=SIX_RUN) {
        unsigned h=hp[sector],l=lp[sector],z=3-h-l;
        /* RC-filtered, rank-aligned ON-time terminal voltages with controllable
         * phase crossing. The V sample is 1.412us EARLIER than U/W. */
        float hz=fmaxf(1.5f,SixStep_IsGeneric() ? fabsf(s.reference) : fabsf(s.reference)*MotorParams_GetActive()->pole_pairs/6.283185307f);
        bool revup=s.state==SIX_RAMP || s.state==SIX_SYNC;
        float period=revup ? 1000000.0f/(6.0f*hz) : held_period;
        if(revup) held_period=period;
        float sign=((sector&1U)!=0U) != (s.target<0) ? 1.0f : -1.0f;
        float phase=((float)(us-step_at)-40.0f-period*cross_fraction)/period;
        float e=inject_signal ? sign*4.0f*sinf(phase*3.14159265f) : 0;
        frame.m1_voltage[h]=bus*0.75f;
        frame.m1_voltage[l]=bus*0.25f;
        frame.m1_voltage[z]=bus*0.5f+e;
        float level=sample_off ? 0 : h==1 ? bus : l==1 ? 0 : bus*0.5f;
        frame.m1_voltage[1]=(frame.m1_voltage[1]-level*0.03469f)/(1.0f-0.03469f);
    }
    SixStep_Process(&frame);
}
static void advance(unsigned n) {
    for(unsigned i=0;i<n;++i) {
        us+=100; host_dwt.CYCCNT+=(uint32_t)(170U*100U);
        if(deadline && us>=deadline) {
            deadline=0; SixStep_OnTimer();
        }
        sample();
    }
}
static void idle(void) {
    SixStep_Stop(); requested=false; stopped=false; ready=true;
    inject_signal=window=true; previous_sector=255; injected_current=0; cross_fraction=0.5f;
    model_alpha=model_beta=sine_alpha=sine_beta=0;
    advance(2100); assert(SixStep_AtRest());
}
static void check_alignment_current(void) {
    /* Current-controlled continuous vector alignment, no six-step dwell. */
    for(unsigned profile=0;profile<2;++profile) for(unsigned supply=12; supply<=20;supply+=8) for(unsigned oil=0;oil<2;++oil) {
        MotorParams_Select((MotorProfile)profile); bus=(float)supply; idle();
        const MotorParams *m=MotorParams_GetActive();
        assert(oil ? SixStep_Throttle(100) : SixStep_Start(30));
        float max_amps=0;
        for(unsigned k=0;k<9500;++k) {
            advance(1);
            max_amps=fmaxf(max_amps,fabsf(model_alpha));
            assert(state().state==SIX_ALIGN && requested);
        }
        assert(model_alpha>0.45f*m->rated_current && model_alpha<0.55f*m->rated_current);
        assert(max_amps<0.61f*m->rated_current);
        advance(510); assert(state().state==SIX_RAMP && !deadline && sine_drive);
        assert(state().duty<0.5f);
    }
    puts("PASS: continuous current-vector alignment at 12/20V, both profiles: bounded current/ramp and no forced 60-degree timer during startup");
}
static void check_throttle(void) {
    const float bad[]={NAN,INFINITY,-1,100.1f};
    for(unsigned profile=0;profile<2;++profile) for(unsigned supply=12;supply<=20;supply+=8) {
        MotorParams_Select((MotorProfile)profile); bus=(float)supply; idle();
        for(unsigned k=0;k<sizeof(bad)/sizeof(*bad);++k) assert(!SixStep_Throttle(bad[k]) && !requested);
        assert(SixStep_Throttle(100));
        assert(!state().speed_control && state().target==100 && !SixStep_Start(30));
        for(unsigned k=0;k<70000 && state().state!=SIX_RUN && state().state!=SIX_FAULT;++k) advance(1);
        assert(state().state==SIX_RUN && state().quality==12 && requested);
        float at_handover=duty;
        advance(10); assert(fabsf(duty-at_handover)<=0.0021f); /* No full duty jump. */
        advance(5000);
        float limit=fminf(0.85f,MotorParams_GetActive()->bus_voltage/bus);
        assert(fabsf(duty-limit)<0.001f);
        assert(SixStep_Throttle(20)); advance(1000);
        assert(fabsf(duty-0.2f*limit)<0.001f);
        injected_current=2*MotorParams_GetActive()->rated_current; advance(100);
        assert(duty<0.1f*limit); /* Soft amplitude cap remains active in throttle. */
        assert(!SixStep_Throttle(-1) && state().target==20);
        stopped=true; assert(SixStep_Throttle(0) && !requested && !deadline);
        advance(1); assert(!requested); /* Zero cannot acknowledge/restart KEY1. */
        assert(!SixStep_Throttle(20));
    }
    puts("PASS: real throttle both profiles at 12/20V: independent 0..100 target, bounded align/handover/slew/voltage/current, no Speed switch while active, zero coast even with KEY1 held, invalid input unchanged");
}
static void check_generic_capture_floor(void) {
    MotorParams_Select(MOTOR_PROFILE_4015); bus=12; idle();
    SixStep_SetGeneric(true); sine_flux=0.03f;
    assert(SixStep_Throttle(30));
    for(unsigned k=0;k<70000 && state().state<SIX_SYNC;++k) advance(1);
    /* High flux meets the strength threshold below 12 Hz. It must not
     * freeze the ramp below the independent minimum capture frequency. */
    assert(state().state==SIX_SYNC && state().reference>=12 && state().startup_ms==500);
    idle(); SixStep_SetGeneric(false); sine_flux=0.0075f;
    puts("PASS: generic high-strength startup still reaches minimum capture frequency before freezing the ramp");
}
static void check_shared_capture(void) {
    float hz[2],strength[2];
    uint64_t duration[2];
    MotorParams_Select(MOTOR_PROFILE_2804); bus=12; sine_flux=0.0032f;
    for(unsigned esc=0;esc<2;++esc) {
        idle(); SixStep_SetGeneric(esc!=0);
        uint64_t began=us;
        assert(SixStep_Throttle(20));
        for(unsigned k=0;k<70000 && state().state<SIX_SYNC;++k) advance(1);
        SixStepSnapshot ss=state();
        assert(ss.state==SIX_SYNC && ss.startup_ms==500 && ss.startup_emf>=0.6f);
        hz[esc]=esc ? ss.reference : ss.reference*7.0f/6.283185307f;
        strength[esc]=ss.startup_emf; duration[esc]=us-began;
    }
    assert(fabsf(hz[0]-hz[1])<0.01f && fabsf(strength[0]-strength[1])<0.001f);
    assert(duration[0]+1000>duration[1] && duration[1]+1000>duration[0]);
    /* Low requested Speed must not cause a hidden high-speed startup.
     * With weak EMF at that target, hold in SINE and time out safely. */
    idle(); SixStep_SetGeneric(false);
    assert(SixStep_Start(15));
    for(unsigned k=0;k<81000 && state().state!=SIX_FAULT;++k) {
        advance(1);
        assert(fabsf(state().reference)<=15.001f);
        assert(state().state!=SIX_SYNC && state().state!=SIX_RUN);
    }
    assert(state().state==SIX_FAULT && !requested && !deadline && state().gate=='E');
    idle(); sine_flux=0.0075f;
    puts("PASS: 2804 preset/ESC 20% share strong capture frequency/hold/voltage; low Speed never exceeds target or enters weak-signal SYNC");
}
static void check_rounded_speed_floor(void) {
    SixStep_SetGeneric(false);
    for(unsigned p=0;p<2;++p) {
        MotorParams_Select((MotorProfile)p);
        float minimum=p ? 10.0f : 15.0f;
        assert(SixStep_MinSpeed()==minimum);
        idle();
        assert(!SixStep_Start(minimum-0.1f) && !SixStep_Start(-minimum+0.1f) && !requested);
        for(int sign=-1;sign<=1;sign+=2) {
            assert(SixStep_Start(sign*minimum) && state().target==sign*minimum);
            idle();
        }
    }
    puts("PASS: rounded six-step Speed command floors 2804=15 / 4015=10 rad/s; both signs inclusive, smaller nonzero commands refused");
}
int main(void) {
    check_rounded_speed_floor();
    check_shared_capture();
    check_generic_capture_floor();
    check_alignment_current();
    check_throttle();
    MotorParams_Select(MOTOR_PROFILE_4015); bus=12; idle();
    sine_flux=0.0032f; /* Keep the synthetic ramp moving above the new 10-rad/s floor. */
    assert(SixStep_Start(30)); advance(20000);
    assert(state().state==SIX_RAMP && state().reference>SixStep_MinSpeed());
    float sine_ref=state().reference;
    assert(SixStep_Start(SixStep_MinSpeed())); advance(1);
    assert(state().state==SIX_RAMP && state().startup_ms==0);
    assert(sine_ref-state().reference<0.002f && state().reference<sine_ref);
    sine_flux=0.0075f;
    const float invalid[]={NAN,INFINITY,1,-1,1000};
    for(unsigned profile=0;profile<2;++profile) for(int direction=-1;direction<=1;direction+=2) {
        MotorParams_Select((MotorProfile)profile); idle();
        for(unsigned i=0;i<sizeof(invalid)/sizeof(*invalid);++i) assert(!SixStep_Start(invalid[i]));
        assert(state().state!=SIX_ALIGN);
        float target=direction*50.0f;
        assert(SixStep_Start(target) && requested && state().state==SIX_ALIGN);
        assert(!SixStep_Start(-target));
        for(unsigned i=0;i<70000 && state().state!=SIX_RUN && state().state!=SIX_FAULT;++i) advance(1);
        if(state().state!=SIX_RUN) fprintf(stderr,"capture failed: %s qual %u crosses %lu sector %u at %llu\n",state().status,state().quality,(unsigned long)state().crossings,sector,(unsigned long long)us);
        assert(state().state==SIX_RUN && state().crossings>=12 && state().velocity*direction>0);
        float takeover_duty=state().duty;
        float takeover_ref=state().reference;
        advance(10); /* Preloaded speed PI must not discard the startup drive. */
        assert(state().state==SIX_RUN && fabsf(state().duty-takeover_duty)<0.005f);
        float rise=fminf(0.04f,0.00005f*state().velocity*state().velocity*MotorParams_GetActive()->pole_pairs);
        assert(state().reference<=takeover_ref+rise+0.0001f && rise<=0.04f);
        assert(SixStep_Start(direction*60.0f));
        /* Once locked, missing BEMF must stop via TIM7 without any new ADC call. */
        inject_signal=false; window=false;
        for(unsigned i=0;i<3000 && requested;++i) advance(1);
        assert(!requested && state().state==SIX_FAULT && !deadline);
        assert(!strcmp(state().status,"6STEP NO ZC") && state().duty>0);
        idle(); assert(SixStep_Start(target));
        stopped=true; advance(1); assert(!requested && !deadline);
        assert(!SixStep_Start(target));
        idle(); assert(SixStep_Start(target)); ready=false;
        advance(1); assert(!requested && !deadline);
    }
    idle(); inject_signal=false;
    assert(SixStep_Start(50)); advance(81000);
    assert(!requested && !strcmp(state().status,"6STEP START TIME") && state().duty>0);
    /* Phase synchronization has its own missing-crossing watchdog BEFORE
     * closing the speed loop; a stale timer must never blindly commute. */
    idle(); assert(SixStep_Start(30));
    for(unsigned k=0;k<70000 && state().state!=SIX_SYNC && state().state!=SIX_FAULT;++k) advance(1);
    assert(state().state==SIX_SYNC && state().quality==0 && requested);
    window=false; inject_signal=false;
    for(unsigned k=0;k<3000 && requested;++k) advance(1);
    assert(!requested && state().state==SIX_FAULT && !strcmp(state().status,"6STEP NO ZC"));
    assert(state().gate=='W');
    /* A 145.x% interval used to display only 145 despite exceeding 1.45.
     * Keep that boundary and the grossly late case; preserve exact cause,
     * decimal and the last six candidate intervals across fault/stop. */
    const float late[]={0.955f,1.10f};
    for(unsigned n=0;n<2;++n) {
        idle(); assert(SixStep_Start(30));
        assert(state().reject==SIX_REJECT_NONE && !state().period_10us[5]);
        for(unsigned k=0;k<70000 && state().state!=SIX_RUN && state().state!=SIX_FAULT;++k) advance(1);
        assert(state().state==SIX_RUN);
        cross_fraction=late[n];
        for(unsigned k=0;k<3000 && requested;++k) advance(1);
        SixStepSnapshot rejected=state();
        assert(!requested && rejected.state==SIX_FAULT && !strcmp(rejected.status,"6STEP PHASE"));
        assert(rejected.gate=='T' && rejected.quality==0 && !deadline && rejected.reject==SIX_REJECT_LONG);
        assert(rejected.phase_pct>=10 && rejected.phase_pct<=85 && rejected.period_10us[0]);
        if(n==0) assert(rejected.period_pct==145 && rejected.period_fraction>0);
        else assert(rejected.period_pct>145);
        advance(3000);
        SixStepSnapshot retained=state();
        assert(retained.phase_pct==rejected.phase_pct && retained.period_pct==rejected.period_pct && !requested);
        assert(retained.period_fraction==rejected.period_fraction && retained.reject==rejected.reject);
        assert(!memcmp(retained.period_10us,rejected.period_10us,sizeof(rejected.period_10us)));
    }
    idle(); host_dwt.CYCCNT=UINT32_MAX-170000000U;
    assert(SixStep_Start(50));
    for(unsigned i=0;i<70000 && state().state!=SIX_RUN && state().state!=SIX_FAULT;++i) advance(1);
    assert(state().state==SIX_RUN); SixStep_Stop();
    puts("PASS: encoder-free real six-step: both profiles/directions, align/ramp/sync/run, low-speed reference slew, unchanged PHASE shutdown with retained phase/period diagnostics, signed targets, RC/rank correction, TIM7 loss watchdog, GPIO/ADC stop, startup timeout and DWT wrap");
    return 0;
}
