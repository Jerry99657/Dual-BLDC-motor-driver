/* Test the actual adapter; substitute only the ARM-only ST binary boundary.
 * This does not execute or validate the proprietary observer on a host PC. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "st_hso.h"
#include "hso.h"
#include "impedcorr.h"

HostDwt host_dwt;
uint32_t host_primask;
static HSO_Params params;
static IMPEDCORR_Params iparams;
static Currents_Iab_t measured_i;
static Voltages_Uab_t measured_v;
static float angle, speed_hz=10, flux;
static unsigned runs, configurations;
static uint32_t execution_cycles=1000U;
static int32_t q(float x) { return (int32_t)(x*1073741824.0f); }
void HSO_setParams(HSO_Handle h,const HSO_Params *p) { (void)h; params=*p; flux=p->Flux_Wb; ++configurations; }
void HSO_setFlag_TrackAngle(HSO_Handle h,bool x) { (void)h; assert(x); }
void HSO_setFlag_EnableOffsetUpdate(HSO_Handle h,bool x) { (void)h; assert(x); }
void HSO_run(HSO_Handle h,Voltages_Uab_t *e,fixp30_t c) {
    (void)h; (void)e; assert(c==0); ++runs; host_dwt.CYCCNT+=execution_cycles;
}
fixp30_t HSO_getAngle_pu(const HSO_Handle h) { (void)h; return q(angle/6.28318530718f); }
fixp30_t HSO_getEmfSpeed_pu(const HSO_Handle h) { (void)h; return q(speed_hz/1000); }
fixp30_t HSO_getSpeedLP_pu(const HSO_Handle h) { return HSO_getEmfSpeed_pu(h); }
fixp30_t HSO_getFluxAmpl_Wb(const HSO_Handle h) { (void)h; return q(flux); }
IMPEDCORR_Handle IMPEDCORR_init(void *m,size_t n) { assert(n>=sizeof(IMPEDCORR_Obj)); return m; }
void IMPEDCORR_setParams(IMPEDCORR_Handle h,const IMPEDCORR_Params *p) { (void)h; iparams=*p; }
void IMPEDCORR_setRs_si(IMPEDCORR_Handle h,float x) { (void)h; assert(x==MOTOR_2804.Rs); }
void IMPEDCORR_setLs_si(IMPEDCORR_Handle h,float x) { (void)h; assert(x==MOTOR_2804.Ls); }
Voltages_Uab_t IMPEDCORR_run(IMPEDCORR_Handle h,const Currents_Iab_t *i,const Voltages_Uab_t *v,fixp30_t w) {
    (void)h; (void)w; measured_i=*i; measured_v=*v; return *v;
}
static MotorSampleFrame frame;
static float encoder_speed;
static void step(bool ready,bool encoder) {
    ++frame.sequence;
    StHso_Process(&frame,ready,0,encoder_speed,encoder);
}
int main(void) {
    StHso_Init(&MOTOR_2804);
    for(unsigned i=0;i<3;++i) { frame.m1_current_raw[i]=2048; frame.m1_voltage_raw[i]=1350; }
    frame.m1_voltage[0]=15; frame.m1_voltage[1]=9; frame.m1_voltage[2]=12;
    frame.m1_current[0]=0.5f; frame.m1_current[1]=-0.25f;
    StHsoSnapshot s;
    step(false,true); StHso_GetSnapshot(&s); assert(s.state==ST_HSO_WAIT_SAMPLE && runs==0);
    step(true,true); StHso_GetSnapshot(&s); assert(s.state==ST_HSO_WAIT_PARAM && runs==0);
    StHsoConfig c; StHso_GetConfig(&c);
    assert(c.voltage_pole_rps==25000 && fabsf(c.sample_delay_cycles-0.019f)<1e-6f);
    c.flux_wb=NAN; assert(!StHso_SetConfig(&c));
    c.flux_wb=0.003f; assert(StHso_SetConfig(&c));
    step(true,true); assert(runs==1 && configurations==1);
    assert(params.Flux_Wb==0.003f && params.FullScaleVoltage_V==72.6f);
    assert(iparams.cycleFreq_Hz==10000 && iparams.fullScaleCurrent_A==6.6f);
    assert(fabsf(measured_i.A/1073741824.0f-0.5f/6.6f)<1e-6f && measured_i.B==0);
    assert(fabsf(measured_v.A/1073741824.0f-3.0f/72.6f)<1e-6f);
    assert(fabsf(measured_v.B/1073741824.0f+3.0f*0.577350269f/72.6f)<1e-6f);
    /* Common-mode shift cannot change alpha/beta. */
    Voltages_Uab_t before=measured_v;
    for(unsigned i=0;i<3;++i) frame.m1_voltage[i]+=5;
    step(true,true); assert(before.A==measured_v.A && before.B==measured_v.B);
    encoder_speed=10.0f*6.28318530718f/7;
    /* Reproduce the user's ~24 ms reset loop: occasional small angle spikes
     * pause, do not earn progress, and do not erase the existing good samples. */
    step(true,false);
    for(unsigned i=0;i<1999;++i) {
        angle=(i&1)?-0.08f:0.24f; step(true,true);
        if((i+1)%240==0) { angle=0.30f; step(true,true); }
    }
    StHso_GetSnapshot(&s);
    assert(!StHso_IsUsable() && s.qualified_samples==1999 && s.qualify_paused_samples==8);
    angle=0; step(true,true); assert(StHso_IsUsable() && StHso_CanTakeover());
    /* Never qualify on a sustained 17-degree error. */
    step(true,false); angle=0.30f;
    for(unsigned i=0;i<3000;++i) step(true,true);
    assert(!StHso_IsUsable()); StHso_GetSnapshot(&s); assert(s.qualified_samples==0);
    angle=0;
    for(unsigned i=0;i<100;++i) step(true,true);
    angle=0.30f;
    for(unsigned i=0;i<10;++i) step(true,true);
    StHso_GetSnapshot(&s);
    assert(s.qualified_samples==100 && s.qualify_bad_run==10);
    step(true,true); StHso_GetSnapshot(&s);
    assert(s.qualified_samples==0 && s.qualify_bad_run==0);
    assert(fabsf(s.qualify_reset_angle_rad-0.30f)<0.00001f);
    /* An attempt has a fixed total pause budget, even for isolated spikes. */
    for(unsigned i=0;i<50;++i) { angle=0; step(true,true); angle=0.30f; step(true,true); }
    StHso_GetSnapshot(&s); assert(s.qualified_samples==50 && s.qualify_paused_samples==50);
    angle=0; step(true,true); angle=0.30f; step(true,true);
    StHso_GetSnapshot(&s); assert(s.qualified_samples==0 && s.qualify_paused_samples==0);
    angle=0; step(true,true); angle=0.36f; step(true,true);
    StHso_GetSnapshot(&s); assert(s.qualified_samples==0); /* >=20 deg resets immediately. */
    /* ANGLE's first-reason label cannot mask a simultaneous frequency/flux failure. */
    angle=0; step(true,true); angle=0.30f; speed_hz=13.1f; step(true,true);
    StHso_GetSnapshot(&s); assert(s.qualified_samples==0 && s.qualify_paused_samples==0);
    speed_hz=10; angle=0; step(true,true); angle=0.30f; flux=0.005f; step(true,true);
    StHso_GetSnapshot(&s); assert(s.qualified_samples==0 && s.qualify_paused_samples==0);
    flux=c.flux_wb; angle=0;
    for(unsigned i=0;i<2000;++i) step(true,true);
    assert(StHso_IsUsable() && StHso_CanTakeover());
    /* Admission remains strict, but 15-degree boundary jitter no longer
     * destroys a qualified observer. A new Use must still check this frame. */
    angle=0.28f;
    for(unsigned i=0;i<200;++i) step(true,true);
    assert(StHso_IsUsable() && !StHso_CanTakeover());
    StHso_GetSnapshot(&s);
    assert(s.bad_samples==0 && s.entry_reason==ST_HSO_REASON_ANGLE);
    angle=0.1f; step(true,true); assert(StHso_CanTakeover());
    /* Brief hold-envelope spikes recover; 20 consecutive failures lose lock. */
    angle=0.53f;
    for(unsigned i=0;i<19;++i) step(true,true);
    assert(StHso_IsUsable()); StHso_GetSnapshot(&s); assert(s.bad_samples==19);
    angle=0; step(true,true);
    assert(StHso_IsUsable()); StHso_GetSnapshot(&s); assert(s.bad_samples==0);
    angle=0.53f;
    for(unsigned i=0;i<20;++i) step(true,true);
    assert(!StHso_IsUsable()); StHso_GetSnapshot(&s);
    assert(s.reason==ST_HSO_REASON_ANGLE && s.bad_samples==20);
    angle=0;
    for(unsigned i=0;i<1999;++i) step(true,true);
    assert(!StHso_IsUsable());
    step(true,true);
    assert(StHso_IsUsable());
    /* A severe angle error must not wait for the 2 ms confirmation. */
    angle=0.80f; step(true,true); assert(!StHso_IsUsable());
    StHso_GetSnapshot(&s); assert(s.reason==ST_HSO_REASON_ANGLE);
    angle=0;
    for(unsigned i=0;i<2000;++i) step(true,true);
    /* Frequency/flux hold failures have their own diagnosable causes. */
    speed_hz=13.1f;
    for(unsigned i=0;i<19;++i) step(true,true);
    assert(StHso_IsUsable()); step(true,true); assert(!StHso_IsUsable());
    StHso_GetSnapshot(&s); assert(s.reason==ST_HSO_REASON_FREQUENCY);
    speed_hz=10;
    for(unsigned i=0;i<2000;++i) step(true,true);
    flux=0.005f;
    for(unsigned i=0;i<20;++i) step(true,true);
    assert(!StHso_IsUsable()); StHso_GetSnapshot(&s); assert(s.reason==ST_HSO_REASON_FLUX);
    flux=c.flux_wb;
    for(unsigned i=0;i<2000;++i) step(true,true);
    speed_hz=-10; step(true,true); assert(!StHso_IsUsable());
    StHso_GetSnapshot(&s); assert(s.reason==ST_HSO_REASON_FREQUENCY);
    speed_hz=10;
    for(unsigned i=0;i<2000;++i) step(true,true);
    step(true,false); assert(!StHso_IsUsable());
    StHso_GetSnapshot(&s); assert(s.reason==ST_HSO_REASON_ENCODER);
    frame.sequence+=3; step(true,true); assert(configurations==2 && !StHso_IsUsable());
    StHso_GetSnapshot(&s); assert(s.reason==ST_HSO_REASON_SEQUENCE);
    frame.m1_voltage_raw[1]=4095; unsigned old_runs=runs;
    step(true,true); StHso_GetSnapshot(&s); assert(s.state==ST_HSO_INPUT_RANGE && runs==old_runs);
    frame.m1_voltage_raw[1]=1350; step(true,true); assert(configurations==3);
    execution_cycles=18000; step(true,true); StHso_GetSnapshot(&s); assert(s.max_cycles==18000);
    StHso_RecordControlCycles(18000); StHso_GetSnapshot(&s); assert(s.overruns==1);
    execution_cycles=1000;
    StHso_Enable(false); step(true,true); StHso_GetSnapshot(&s); assert(s.state==ST_HSO_OFF && !StHso_LearnFlux());
    StHso_Enable(true);
    /* Synthetic steady state: q EMF = omega*psi; include the known RC lag.
     * Learner must recover physical Wb, not 2*pi-scaled MCSDK ratedFlux. */
    c.flux_wb=0; c.sample_delay_cycles=0.2f; assert(StHso_SetConfig(&c));
    float omega=encoder_speed*7, true_flux=0.003f, lag=omega/c.voltage_pole_rps;
    float id=-0.1f, iq=0.2f;
    float actual_d=c.rs_ohm*id-omega*c.ls_h*iq;
    float actual_q=c.rs_ohm*iq+omega*(true_flux+c.ls_h*id);
    float filtered_d=(actual_d+lag*actual_q)/(1+lag*lag);
    float filtered_q=(actual_q-lag*actual_d)/(1+lag*lag);
    float advance=omega*c.sample_delay_cycles*0.0001f;
    float vd=filtered_d*cosf(advance)-filtered_q*sinf(advance);
    float vq=filtered_d*sinf(advance)+filtered_q*cosf(advance);
    frame.m1_voltage[0]=12+vd;
    frame.m1_voltage[1]=12-0.5f*vd+0.86602540378f*vq;
    frame.m1_voltage[2]=12-0.5f*vd-0.86602540378f*vq;
    frame.m1_current[0]=id;
    frame.m1_current[1]=-0.5f*id+0.86602540378f*iq;
    frame.m1_current[2]=-frame.m1_current[0]-frame.m1_current[1];
    assert(StHso_LearnFlux());
    for(unsigned i=0;i<10000;++i) step(true,true);
    StHso_GetConfig(&c); assert(fabsf(c.flux_wb-true_flux)<0.000002f);
    step(true,true); StHso_GetSnapshot(&s); assert(s.reference_flux_wb>0 && s.state!=ST_HSO_LEARNING);
    /* Negative rotation uses signed omega, not absolute speed. */
    encoder_speed=-encoder_speed; speed_hz=-speed_hz;
    for(unsigned i=0;i<2000;++i) step(true,true);
    assert(StHso_IsUsable() && StHso_Velocity()<0);
    /* Hard safety conditions bypass confirmation even after qualification. */
    float saved_encoder_speed=encoder_speed;
    encoder_speed=NAN; step(true,true); StHso_GetSnapshot(&s);
    assert(!StHso_IsUsable() && s.reason==ST_HSO_REASON_NUMERIC);
    encoder_speed=0; step(true,true); StHso_GetSnapshot(&s);
    assert(!StHso_IsUsable() && s.reason==ST_HSO_REASON_SPEED);
    encoder_speed=saved_encoder_speed;
    for(unsigned i=0;i<2000;++i) step(true,true);
    assert(StHso_IsUsable());
    frame.sequence+=2; step(true,true); StHso_GetSnapshot(&s);
    assert(!StHso_IsUsable() && s.reason==ST_HSO_REASON_SEQUENCE);
    for(unsigned i=0;i<2000;++i) step(true,true);
    assert(StHso_IsUsable());
    step(false,true); StHso_GetSnapshot(&s);
    assert(!StHso_IsUsable() && s.reason==ST_HSO_REASON_SAMPLE);
    for(unsigned i=0;i<2000;++i) step(true,true);
    assert(StHso_IsUsable());
    StHso_RecordControlCycles(17500); StHso_GetSnapshot(&s);
    assert(!StHso_IsUsable() && s.state==ST_HSO_TIMING && s.max_control_cycles==18000);
    assert(s.reason==ST_HSO_REASON_TIMING);
    host_primask=1; StHso_GetSnapshot(&s); assert(host_primask==1);
    puts("PASS: HSO scaling/learning, strict admission, hold hysteresis/2ms confirmation, hard faults, reason diagnostics, sequence gaps, ADC clipping, timing (ST binary mocked)");
    return 0;
}
