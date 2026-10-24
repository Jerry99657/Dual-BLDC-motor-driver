/* Coupled electrical/mechanical regression. The rotor is NEVER assigned the
 * commanded speed, sector or estimated angle. This is an assumed balanced
 * star-equivalent PMSM, not an identification of the physical 4015. */
#define main legacy_sixstep_main
#include "test_sixstep.c"
#undef main
#include <stdlib.h>

static double amps[3], capv[3], rotor, speed;
static double inertia=0.00002, friction=0.00003;
static double flux=0.0075, max_current;
static bool locked,tolerance;
static const MotorParams *plant_motor;

static void plant_tick(void)
{
    const MotorParams *m=plant_motor;
    const double dt=0.000001;
    double emf[3], pin[3]={0,0,0}, sine[3];
    for(unsigned k=0;k<3;++k) {
        sine[k]=sin(rotor-2.094395102393195*k);
        emf[k]=-flux*speed*m->pole_pairs*sine[k];
    }
    unsigned h=hp[sector], l=lp[sector], z=3-h-l;
    bool on=(us%100)<duty*50.0 || (us%100)>=100.0-duty*50.0;
    pin[h]=requested && on ? bus : 0;
    pin[l]=0;
    bool floating=fabs(amps[z])<1e-8;
    double free_voltage=(pin[h]+pin[l]+3*emf[z])*0.5;
    if(floating && free_voltage>=-0.65 && free_voltage<=bus+0.65) pin[z]=free_voltage;
    else { pin[z]=floating ? (free_voltage<0 ? -0.65 : bus+0.65) : amps[z]>0 ? -0.65 : bus+0.65; floating=false; }
    if(sine_drive) {
        floating=false;
        for(unsigned k=0;k<3;++k) pin[k]=(us%100)<sine_phase[k]*50 || (us%100)>=100-sine_phase[k]*50 ? bus : 0;
    }
    double neutral=(pin[0]+pin[1]+pin[2])/3.0;
    double before=amps[z];
    double torque=0;
    for(unsigned k=0;k<3;++k) {
        if(k!=z || !floating) amps[k]+=(pin[k]-neutral-m->Rs*amps[k]-emf[k])*dt/m->Ls;
        double tau=tolerance ? (k==0 ? 36e-6 : k==1 ? 44e-6 : 40e-6) : 40e-6;
        capv[k]+=(pin[k]-capv[k])*(1.0-exp(-dt/tau));
    }
    if(!sine_drive && !floating && before*amps[z]<0) {
        amps[h]+=amps[z]*0.5; amps[l]+=amps[z]*0.5; amps[z]=0;
    }
    for(unsigned k=0;k<3;++k) {
        torque-=flux*m->pole_pairs*sine[k]*amps[k];
        max_current=fmax(max_current,fabs(amps[k]));
    }
    if(!locked) {
        speed+=(torque-friction*speed)*dt/inertia;
        rotor+=speed*m->pole_pairs*dt;
    } else speed=0;
}
static float plant_adc(double volts)
{
    const double lsb=3.26*11.0/4095.0;
    return (float)(fmax(0,fmin(4095,round(volts/lsb)))*lsb);
}
int main(int argc,char **argv)
{
    setvbuf(stdout,NULL,_IONBF,0);
    MotorProfile physical=argc>7 && !strcmp(argv[7],"2804") ? MOTOR_PROFILE_2804 : MOTOR_PROFILE_4015;
    plant_motor=MotorParams_GetProfile(physical);
    MotorParams_Select(physical); bus=argc>1 ? (float)atof(argv[1]) : 12;
    idle();
    SixStep_SetGeneric(argc>8 && !strcmp(argv[8],"generic"));
    if(SixStep_IsGeneric()) MotorParams_Select(argc>9 && !strcmp(argv[9],"2804") ? MOTOR_PROFILE_2804 : MOTOR_PROFILE_4015);
    rotor=-0.5235987755982988; speed=0;
    int direction=argc>2 ? atoi(argv[2]) : 1;
    if(argc>3) inertia=atof(argv[3]);
    const char *scenario=argc>4 ? argv[4] : "normal";
    float target=argc>5 ? (float)atof(argv[5]) : 30;
    if(argc>6) flux=atof(argv[6]);
    locked=!strcmp(scenario,"stall"); tolerance=!strcmp(scenario,"rc");
    bool no_current=!strcmp(scenario,"no-current"), key=!strcmp(scenario,"key");
    bool oil=SixStep_IsGeneric() || !strncmp(scenario,"throttle",8);
    assert(oil ? SixStep_Throttle(target) : SixStep_Start(direction*target));
    unsigned run_frames=0;
    bool reduced=false,adc_off=false;
    uint64_t sample_at=UINT64_MAX;
    SixStepState old=SIX_IDLE;
    bool entered_sync=false,entered_run=false;
    unsigned reversals=0,averages=0;
    double speed_sum=0,estimate_sum=0;
    bool trace=getenv("SIXSTEP_TRACE")!=NULL;
    const uint64_t began=us;
    for(unsigned n=0;n<9000000 && state().state!=SIX_FAULT;++n) {
        if(deadline && us>=deadline) { deadline=0; SixStep_OnTimer(); }
        /* CCR4 preload loads at both timer extrema. OFF->ON loaded at a
         * valley does not produce another rising edge there: OC4REF was
         * already high. Preserve that one 150us ADC interval, and the
         * 50us interval when an ON->OFF change takes effect at a peak. */
        if(us%50==0) {
            bool previous=adc_off; adc_off=sample_off;
            if((us%100==50 && adc_off) || (us%100==0 && !adc_off && !previous)) {
                sample_at=us; frame.m1_sixstep_off=adc_off;
            }
        }
        uint64_t sample_age=us-sample_at;
        double old_caps[3],old_amps[3];
        memcpy(old_caps,capv,sizeof(capv)); memcpy(old_amps,amps,sizeof(amps));
        plant_tick();
        if(sample_age==0) for(unsigned k=0;k<2;++k)
            frame.m1_current[k]=no_current ? 0 : (float)(old_amps[k]+0.294117647*(amps[k]-old_amps[k]));
        if(sample_age==1) {
            frame.m1_voltage[1]=plant_adc(old_caps[1]+0.705882353*(capv[1]-old_caps[1]));
            frame.m1_current[2]=no_current ? 0 : (float)(old_amps[2]+0.705882353*(amps[2]-old_amps[2]));
        }
        if(sample_age==3) {
            for(unsigned k=0;k<3;k+=2)
                frame.m1_voltage[k]=plant_adc(old_caps[k]+0.117647059*(capv[k]-old_caps[k]));
            frame.m1_sixstep_stamp=host_dwt.CYCCNT+20;
            frame.m1_sixstep_valid=requested && adc_off==sample_off &&
                (adc_off ? duty<0.905f : duty>0.095f) && us-step_at>=10;
        }
        ++us; host_dwt.CYCCNT+=170;
        if(us==sample_at+4) {
            if(key && state().state==SIX_RAMP && fabsf(state().reference)>5) stopped=true;
            SixStep_Process(&frame);
            if(state().state==SIX_RUN && ++run_frames==10000 && !strcmp(scenario,"throttle-drop")) {
                assert(SixStep_Throttle(20)); reduced=true;
            }
        }
        SixStepSnapshot ss=state();
        if(ss.state==SIX_SYNC) entered_sync=true;
        if(ss.state==SIX_RUN) entered_run=true;
        float reference=SixStep_IsGeneric() ? ss.reference*6.283185307f/plant_motor->pole_pairs : ss.reference;
        if(ss.state>=SIX_RAMP && ss.state<=SIX_RUN && fabsf(reference)>2 && speed*direction< -0.5) ++reversals;
        if(n>=8500000) {
            speed_sum+=speed;
            estimate_sum+=SixStep_IsGeneric() ? ss.velocity*6.283185307f/plant_motor->pole_pairs : ss.velocity;
            ++averages;
        }
        if(trace && (ss.state!=old || us%100000==0)) {
            printf("t=%.3f %s q%u zc%lu sector%u actual=%.3f estimate=%.3f ref=%.3f duty=%.3f I=%.3f E=%.3f gate=%c timing=%u/%u\n",
                us*1e-6,ss.status,ss.quality,(unsigned long)ss.crossings,sector,speed,ss.velocity,ss.reference,duty,max_current,ss.bemf,ss.gate,ss.phase_pct,ss.period_pct);
        }
        old=ss.state;
        if(key && stopped && !requested) break;
    }
    SixStepSnapshot ss=state();
    printf("plant %.0fV dir%d J%.6f %s: %s, actual/estimate means %.3f/%.3f, reversals=%u, Imax=%.3f, %.3fs\n",
        bus,direction,inertia,scenario,ss.status,averages ? speed_sum/averages : speed,
        averages ? estimate_sum/averages : ss.velocity,reversals,max_current,(us-began)*1e-6);
#ifndef SIXSTEP_BASELINE
    if(locked || no_current) {
        assert(!entered_sync && !entered_run && !requested && !deadline && ss.state==SIX_FAULT);
        assert(!strcmp(ss.status,no_current ? "6STEP CURRENT" : "6STEP START TIME"));
    } else if(key) assert(!entered_sync && !entered_run && !requested && !deadline && ss.state==SIX_IDLE);
    else if(tolerance && ss.state==SIX_FAULT) {
        /* A deliberately mismatched input network is NOT claimed to work:
         * require truthful loss detection, power off and no automatic retry.
         * A real board with this failure needs measured terminal waveforms. */
        assert(entered_sync && !requested && !deadline);
        assert(!strcmp(ss.status,"6STEP NO ZC") || !strcmp(ss.status,"6STEP PHASE"));
    }
    else {
        assert(entered_run && ss.state==SIX_RUN && requested && ss.quality==12 && averages);
        assert(!reversals);
        if(oil) {
            assert(fabs(duty-(reduced ? 20 : target)*0.0085f)<0.001f);
            assert(speed_sum/averages>5);
        } else assert(fabs(speed_sum/averages-direction*target)<1.5);
        assert(fabs((speed_sum-estimate_sum)/averages)<1.0);
        assert(max_current<(SixStep_IsGeneric() ? 0.5 : 0.7*plant_motor->rated_current));
    }
#else
    (void)entered_sync; (void)entered_run; (void)reduced;
#endif
    return 0;
}
