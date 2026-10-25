/* Electrical signal regression, NOT an identified mechanical motor model.
 * The prescribed rotor advances continuously from its aligned flux position;
 * sine BEMF follows that position, NOT the detector's parity/sign formula.
 * Actual commanded phase pins drive a 40us RC and ADC quantization model. */
#define main legacy_sixstep_main
#include "test_sixstep.c"
#undef main

static double capacitor[3],emf[3];
static double cursor,half_on;
static unsigned wave_h,wave_l;
static bool clipped,noisy;

static void circuit_to(double end)
{
    while(cursor<end-1e-9) {
        double edge=cursor<half_on-1e-9 ? half_on :
                    cursor<100.0-half_on-1e-9 ? 100.0-half_on : 100.0;
        double next=fmin(end,edge);
        bool high=cursor<half_on-1e-9 || cursor>=100.0-half_on-1e-9;
        unsigned z=3-wave_h-wave_l;
        double pin[3]={0,0,0};
        pin[wave_h]=high ? bus : 0;
        pin[z]=pin[wave_h]*0.5+1.5*emf[z];
        if(clipped) pin[z]=fmax(-0.65,fmin(bus+0.65,pin[z]));
        double decay=exp(-(next-cursor)/40.0);
        for(unsigned k=0;k<3;++k) capacitor[k]=pin[k]+(capacitor[k]-pin[k])*decay;
        cursor=next;
    }
}
static float adc_sample(unsigned phase)
{
    const double lsb=3.26*11.0/4096.0;
    double disturbance=noisy ? 0.008*sin((double)(us%10000)*0.371+phase*2.3) : 0;
    return (float)(fmax(0,fmin(4095,round((capacitor[phase]+disturbance)/lsb)))*lsb);
}
static void wave_trial(float supply,int direction,bool clamp_diodes,unsigned scenario)
{
    MotorParams_Select(MOTOR_PROFILE_4015); bus=supply; idle();
    clipped=clamp_diodes;
    noisy=scenario==7;
    memset(capacitor,0,sizeof(capacitor));
    assert(scenario==4 ? SixStep_Throttle(20) : SixStep_Start(direction*30));
    unsigned run_samples=0;
    unsigned settled_crossings=0;
    bool braking=false,brake_checked=false;
    double brake_time=0,brake_duration=0,brake_accel=0;
    double theta=0,omega_e=0;
    for(unsigned n=0;n<121000 && (scenario>=5 || state().state!=SIX_RUN) && state().state!=SIX_FAULT;++n) {
        if(sine_drive) {
            advance(1);
            theta=sine_angle;
            omega_e=state().reference*11.0;
            continue;
        }
        if(deadline && us>=deadline) { deadline=0; SixStep_OnTimer(); }
        SixStepSnapshot ss=state();
        if(ss.state==SIX_SYNC) {
            /* Once phase control starts, rotor motion is an independent,
             * continuous trajectory, not the commutation watchdog period. */
            theta+=omega_e*0.0001;
            if(scenario!=3) omega_e=ss.reference*11.0;
        } else if(ss.state==SIX_RUN && scenario==9) {
            if(!settled_crossings) settled_crossings=ss.crossings+6;
            if(!braking && ss.crossings>=settled_crossings) {
                /* Analytic rotor trajectory, not a detector timestamp:
                 * decelerate continuously so the next electrical 60deg
                 * takes 137% of the preceding steady-speed interval. */
                unsigned z=3-hp[sector]-lp[sector];
                double base=2.094395102393195*z;
                double turns=(theta-base)/3.141592653589793;
                double last_zero=base+3.141592653589793*(direction>0 ? floor(turns) : ceil(turns));
                double old_period=1.047197551196598/fabs(omega_e);
                double since_zero=(theta-last_zero)/omega_e;
                double remaining=1.37*old_period-since_zero;
                brake_duration=0.15*old_period;
                double distance=last_zero+direction*1.047197551196598-theta;
                double final_speed=(distance-0.5*omega_e*brake_duration)/(remaining-0.5*brake_duration);
                brake_accel=(final_speed-omega_e)/brake_duration;
                braking=true;
            }
            double a=braking && brake_time<brake_duration ? brake_accel : 0;
            theta+=omega_e*0.0001+0.5*a*0.0001*0.0001;
            omega_e+=a*0.0001;
            if(braking) brake_time+=0.0001;
            if(braking && ss.crossings>settled_crossings && !brake_checked) {
                assert(ss.period_pct>=133 && ss.period_pct<=141);
                assert(ss.phase_pct>=60 && ss.phase_pct<=68);
                brake_checked=true;
            }
            if(++run_samples>=20000) break;
        } else if(ss.state==SIX_RUN) {
            /* Continue a prescribed, continuous rotor trajectory AFTER
             * handover. Never infer its angle from the detector's sector. */
            theta+=omega_e*0.0001;
            /* Fixed external trajectory: gentle low-speed capture, then a
             * 40rad/s^2 acceleration. Independent of the reference limiter. */
            double acceleration=scenario==8 || fabs(omega_e)>=6.0*11.0 ? 40.0 : 5.0;
            double speed=fmin(30.0*11.0,fabs(omega_e)+acceleration*11.0*0.0001);
            if(scenario==6 && run_samples>=10000) {
                if(run_samples==10000) assert(SixStep_Start(direction*10.0f));
                speed=fmax(10.0*11.0,fabs(omega_e)-40.0*11.0*0.0001);
            }
            omega_e=direction*speed;
            if(++run_samples>=20000) break;
        }
        double rotor_theta=theta,rotor_speed=omega_e;
        if(scenario==1) { rotor_theta=-3.141592653589793/3.0-theta; rotor_speed=-omega_e; }
        if(scenario==2) rotor_speed=0;
        for(unsigned k=0;k<3;++k) emf[k]=-0.0075*rotor_speed*sin(rotor_theta-2.094395102393195*k);
        wave_h=hp[sector]; wave_l=lp[sector];
        cursor=0; half_on=(double)duty*50.0;
        unsigned offset=sample_off ? 50 : 0;
        frame=(MotorSampleFrame){.m1_sixstep_valid=sample_off ? duty<0.905f : duty>0.095f,
            .m1_sixstep_off=sample_off,.m1_sixstep_stamp=host_dwt.CYCCNT+offset*170U+530};
        circuit_to(offset+72.5/42.5); frame.m1_voltage[1]=adc_sample(1);
        circuit_to(offset+132.5/42.5);
        frame.m1_voltage[0]=adc_sample(0); frame.m1_voltage[2]=adc_sample(2);
        host_dwt.CYCCNT+=170U*(offset+4U); us+=offset+4U;
        SixStep_Process(&frame);
        circuit_to(100);
        host_dwt.CYCCNT+=170U*(96U-offset); us+=96U-offset;
    }
    if(scenario==1 || scenario==2) {
        assert(state().state==SIX_FAULT && !requested);
        return;
    }
    if(state().state!=SIX_RUN) fprintf(stderr,"wave failed %.0fV dir %d scenario %u diode %d: %s q%u zc%lu emf %.3f duty %.3f phase/period %u/%u\n",
        supply,direction,scenario,clamp_diodes,state().status,state().quality,(unsigned long)state().crossings,state().bemf,state().duty,state().phase_pct,state().period_pct);
    assert(state().state==SIX_RUN && state().quality==12 && state().velocity*direction>0);
    /* Actual capture can be below the rounded command floor while speeding up. */
    if(scenario==3) assert(fabsf(state().velocity)*MotorParams_GetActive()->pole_pairs>=75.398224f);
    assert(state().crossings>=12);
    assert(state().speed_control==(scenario!=4));
    if(scenario>=5) assert(run_samples>=20000);
    if(scenario==9) assert(brake_checked && state().crossings>=settled_crossings+6);
    SixStep_Stop();
}
int main(void)
{
    for(unsigned v=12;v<=20;v+=8) for(int d=-1;d<=1;d+=2) for(unsigned s=0;s<4;++s)
        wave_trial((float)v,d,true,s);
    wave_trial(12,1,true,4); wave_trial(20,1,true,4);
    for(unsigned v=12;v<=20;v+=8) for(int d=-1;d<=1;d+=2) for(unsigned s=5;s<=7;++s)
        wave_trial((float)v,d,true,s);
    wave_trial(12,1,true,8); /* Abrupt 40rad/s^2: R63 loses ZC immediately. */
    wave_trial(12,1,true,9); wave_trial(12,-1,true,9);
    puts("PASS: independent rotor/PWM/40us RC/skewed ADC/diode traces: 12/20V both directions; 2s after RUN with 5/40rad/s^2 acceleration and voltage noise, adaptive OFF/ON sampling through deceleration to 10rad/s, throttle/fixed-speed handover; reversed/frozen rotor refused (not a mechanical model)");
    return 0;
}
