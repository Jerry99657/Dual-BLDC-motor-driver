/* Actual detector + coordinator + FOC modulator. The independent plant uses
 * averaged inverter voltages, RL windings, PM torque/BEMF and rotor inertia;
 * no result/angle is copied from the detector's internal reference. */
#define MOTOR_HOST_DETECT_TEST
#include "test_control.c"
#include "motor_detect.h"
static double plant_r,plant_p,plant_l,plant_flux,rotor,omega,ia,ib;
static unsigned sensor_ticks;
static int encoder_sign=1;
static bool locked;
static bool reject_zero;
static unsigned zero_age,zero_bridges;
static unsigned voltage_delay;
static double delayed_voltage[3][2];
static bool noisy;
static bool flat_current;
static void plant_tick(void) {
    double ua=0,ub=0;
    if(output_requested) {
        double a=(double)tim1.CCR1/tim1.ARR,b=(double)tim1.CCR2/tim1.ARR,c=(double)tim1.CCR3/tim1.ARR;
        ua=bus*(2*a-b-c)/3; ub=bus*(b-c)/sqrt(3);
    }
    for(unsigned k=2;k>0;--k) {
        delayed_voltage[k][0]=delayed_voltage[k-1][0];
        delayed_voltage[k][1]=delayed_voltage[k-1][1];
    }
    delayed_voltage[0][0]=ua;delayed_voltage[0][1]=ub;
    ua=delayed_voltage[voltage_delay][0];ub=delayed_voltage[voltage_delay][1];
    for(unsigned k=0;k<10;++k) {
        double angle=plant_p*rotor,emf=plant_p*plant_flux*omega;
        double a=ia+(ua-plant_r*ia+emf*sin(angle))*0.00001/plant_l;
        double b=ib+(ub-plant_r*ib-emf*cos(angle))*0.00001/plant_l;
        double torque=1.5*plant_p*plant_flux*(ib*cos(angle)-ia*sin(angle));
        ia=a;ib=b;
        if(!locked) { omega+=(torque-0.002*omega)*0.00001/0.00002;rotor+=omega*0.00001; }
    }
    frame.m1_current[0]=(float)(round(ia/0.0016)*0.0016);
    frame.m1_current[1]=(float)(round((-0.5*ia+sqrt(3)*0.5*ib)/0.0016)*0.0016);
    if(noisy) { /* independent deterministic ADC noise, below one LSB */
        frame.m1_current[0]+=(float)((int)(sensor_ticks*17U%101)-50)*0.000016f;
        frame.m1_current[1]+=(float)((int)(sensor_ticks*29U%101)-50)*0.000016f;
    }
    if(flat_current) { frame.m1_current[0]=0.15f;frame.m1_current[1]=-0.075f; }
    frame.m1_current[2]=-frame.m1_current[0]-frame.m1_current[1];
    /* Independent one-degree mechanical encoder INL; the deliberately
     * nonzero phase makes a short partial-turn estimate biased. */
    double sensor=fmod(encoder_sign*rotor+6.27+0.01745*sin(rotor+0.7),2*PI);if(sensor<0) sensor+=2*PI;
    uint16_t sample=(uint16_t)(sensor*16384/(2*PI));
    if(reject_zero) {
        if(sample==0) {
            ++zero_age;
            /* Rejected zero + ADC service's next age increment. The last
             * verified raw angle is retained, just as the real driver does. */
            healthy=zero_age+1<=3;
            encoder_zero_gap=zero_age+1<=50 && (((unsigned)encoder_raw+2)&16383)<=4;
            if(!healthy && encoder_zero_gap) ++zero_bridges;
        } else { encoder_raw=sample;zero_age=0;healthy=1;encoder_zero_gap=0; }
    } else encoder_raw=sample;
    if(reject_zero && !healthy && !encoder_zero_gap && MotorDetect_Active())
        fprintf(stderr,"zero deadline: %s age %u sample %u last %u rotor %.6f omega %.6f\n",MotorDetect_Status(),zero_age,sample,encoder_raw,rotor,omega);
    encoder_velocity=(float)(encoder_sign*omega);
    hand_position=(float)(encoder_sign*rotor);
    if(++sensor_ticks%10==0) {
        ++now; HAL_TIM_PeriodElapsedCallback(&(TIM_HandleTypeDef){.Instance=TIM2});
    }
    ++frame.sequence; HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
}
static void setup(double r,double p,int sign) {
    MotorDetect_Stop();boot(MOTOR_WORK_SINGLE_M1);MotorControl_StopAll();
    MotorDetect_Reset();plant_r=r;plant_p=p;plant_l=r>2 ? .00086 : .0013;encoder_sign=sign;
    plant_flux=.0075;
    ia=ib=rotor=omega=0;locked=false;encoder_velocity=0;encoder_raw=16350;encoder_zero_gap=0;
    reject_zero=false;zero_age=zero_bridges=0;
    voltage_delay=0;noisy=flat_current=false;memset(delayed_voltage,0,sizeof(delayed_voltage));
    assert(!MotorDetect_Start(99) && !MotorDetect_Start(501));
    assert(MotorDetect_Start(300));
}
int main(void) {
    setbuf(stdout,NULL);
    for(unsigned motor=0;motor<2;++motor) for(unsigned reverse=0;reverse<2;++reverse) {
        bus=motor ? 20 : 12;
        setup(motor ? 1.8 : 2.3,motor ? 11 : 7,reverse ? -1 : 1);
        reject_zero=true;
        for(unsigned t=0;t<500000 && MotorDetect_Active();++t) plant_tick();
        uint32_t r=MotorDetect_Result();
        printf("detect plant: R %.3f p %.0f sign %d -> %s, result %08x, bridged %u\n",plant_r,plant_p,encoder_sign,MotorDetect_Status(),r,zero_bridges);
        printf("Ls %u uH, flux %u uWb\n",MotorDetect_Inductance(),MotorDetect_Flux());
        assert(!MotorDetect_Active() && !output_requested && !output2 && r);
        assert((int8_t)r==(int)(plant_p*encoder_sign));
        assert(fabs((double)((r>>8)&65535)/1000-plant_r)<plant_r*.03);
        assert(fabs((double)MotorDetect_Inductance()*1e-6-plant_l)<plant_l*.12);
        assert(fabs((double)MotorDetect_Flux()*1e-6-.0075)<.0075*.10);
        assert(MotorParams_GetActive()->pole_pairs==7);
        if(motor) assert(zero_bridges); /* Regression: real crossing formerly aborted. */
    }
    for(unsigned motor=0;motor<2;++motor) for(unsigned delay=1;delay<=2;++delay) {
        bus=motor ? 12 : 20;
        setup(motor ? 1.8 : 2.3,motor ? 11 : 7,1);
        voltage_delay=delay;noisy=true;reject_zero=true;
        for(unsigned t=0;t<500000 && MotorDetect_Active();++t) plant_tick();
        printf("delayed %u: %s, L %u flux %u\n",delay,MotorDetect_Status(),MotorDetect_Inductance(),MotorDetect_Flux());
        assert(MotorDetect_Result() && !output_requested);
        assert(fabs((double)MotorDetect_Inductance()*1e-6-plant_l)<plant_l*.12);
        assert(fabs((double)MotorDetect_Flux()*1e-6-.0075)<.0075*.10);
    }
    setup(1.8,11,1);
    while(MotorDetect_Active() && strcmp(MotorDetect_Status(),"LS PULSE")) plant_tick();
    assert(MotorDetect_Active());
    MotorDetect_Stop(); assert(!output_requested && !MotorDetect_Result() && !MotorDetect_Inductance());
    setup(1.8,11,1);
    while(MotorDetect_Active() && strcmp(MotorDetect_Status(),"FLUX TEST")) plant_tick();
    assert(MotorDetect_Active()); locked=true; omega=0;
    for(unsigned t=0;t<30000 && MotorDetect_Active();++t) plant_tick();
    assert(!output_requested && !MotorDetect_Result() && !MotorDetect_Flux());
    printf("locked flux rejected: %s\n",MotorDetect_Status());
    assert(!strcmp(MotorDetect_Status(),"FLUX SLIP"));
    setup(1.8,11,1);
    while(MotorDetect_Active() && strcmp(MotorDetect_Status(),"LS PULSE")) plant_tick();
    assert(MotorDetect_Active());flat_current=true;
    for(unsigned t=0;t<6000 && MotorDetect_Active();++t) plant_tick();
    assert(!MotorDetect_Result() && !MotorDetect_Inductance() && !output_requested);
    assert(!strcmp(MotorDetect_Status(),"LS SIGNAL"));
    setup(1.8,11,1);plant_flux=.012;
    for(unsigned t=0;t<500000 && MotorDetect_Active();++t) plant_tick();
    assert(MotorDetect_Result() && fabs((double)MotorDetect_Flux()*1e-6-plant_flux)<plant_flux*.10);
    setup(1.8,11,1);
    while(MotorDetect_Active() && strcmp(MotorDetect_Status(),"FLUX RAMP")) plant_tick();
    assert(MotorDetect_Active());global_stopped=global_pending=true;output_requested=false;MotorControl_Service(now);
    assert(!MotorDetect_Active() && !MotorDetect_Result() && !MotorDetect_Flux());
    assert(!strcmp(MotorDetect_Status(),"GLOBAL STOP"));
    setup(1.8,11,1);locked=true;
    for(unsigned t=0;t<500000 && MotorDetect_Active();++t) plant_tick();
    assert(!MotorDetect_Result() && !output_requested && !strcmp(MotorDetect_Status(),"POLES / SLIP"));
    setup(1.8,11,1);plant_tick();healthy=false;plant_tick();
    assert(!MotorDetect_Result() && !output_requested && !strcmp(MotorDetect_Status(),"ENCODER1 LOST"));
    healthy=true;for(unsigned t=0;t<100;++t) plant_tick();assert(!output_requested);
    setup(1.8,11,1);healthy=false;encoder_zero_gap=1;plant_tick();
    assert(!MotorDetect_Active() && !output_requested && !MotorDetect_Result()); /* No RS-stage grace. */
    setup(1.8,11,1);for(unsigned t=0;t<16001;++t) plant_tick();
    assert(!strcmp(MotorDetect_Status(),"POLES +"));
    healthy=false;encoder_zero_gap=1;plant_tick();assert(MotorDetect_Active() && output_requested);
    encoder_zero_gap=0;plant_tick();
    assert(!MotorDetect_Active() && !output_requested && !MotorDetect_Result()); /* Gap deadline still stops. */
    setup(1.8,11,1);for(unsigned t=0;t<357103;++t) plant_tick();
    assert(!strcmp(MotorDetect_Status(),"POLES -") && MotorDetect_Active());
    uint32_t hold_pwm[]={tim1.CCR1,tim1.CCR2,tim1.CCR3};
    healthy=false;encoder_zero_gap=1;
    for(unsigned t=0;t<5;++t) plant_tick();
    assert(MotorDetect_Active() && !MotorDetect_Result()); /* Never finish under gap-only admission. */
    assert(tim1.CCR1==hold_pwm[0] && tim1.CCR2==hold_pwm[1] && tim1.CCR3==hold_pwm[2]);
    healthy=true;encoder_zero_gap=0;plant_tick();
    assert(MotorDetect_Active() && !MotorDetect_Result());
    for(unsigned t=0;t<120000 && MotorDetect_Active();++t) plant_tick();
    assert(!MotorDetect_Active() && MotorDetect_Result() && !output_requested);
    setup(1.8,11,1);for(unsigned t=0;t<16001;++t) plant_tick();
    healthy=false;encoder_zero_gap=1;plant_tick();assert(MotorDetect_Active());
    ready=false;plant_tick();assert(!MotorDetect_Active() && !output_requested && !MotorDetect_Result());
    assert(!strcmp(MotorDetect_Status(),"ADC TRIP")); /* ADC protection precedes the zero-gap exception. */
    setup(1.8,11,1);++frame.sequence;plant_tick();
    assert(!output_requested && !MotorDetect_Result() && !strcmp(MotorDetect_Status(),"ADC SEQ"));
    setup(1.8,11,1);ready=false;MotorDetect_Service(now+6);
    assert(!output_requested && !MotorDetect_Result() && !strcmp(MotorDetect_Status(),"ADC TRIP"));
    setup(1.8,11,1);
    command("Speed:10");assert(Protocol_GetControlResult()==-1 && MotorDetect_Active());
    command("OpenLoop:10,2");assert(Protocol_GetControlResult()==-1 && MotorDetect_Active());
    assert(!MotorControl_SelectWork(MOTOR_WORK_INDEPENDENT));
    assert(!MotorControl_SelectProfile(MOTOR_AXIS_1,MOTOR_PROFILE_4015));
    command("Stop");assert(!output_requested && !MotorDetect_Active());
    setup(1.8,11,1);global_stopped=global_pending=true;output_requested=false;MotorControl_Service(now);
    assert(!MotorDetect_Active() && !MotorDetect_Result() && !strcmp(MotorDetect_Status(),"GLOBAL STOP"));
    boot((MotorWorkMode)(MOTOR_WORK_SINGLE_M1|8U)); MotorControl_StopAll();
    assert(MotorControl_HsoAllowed() && !MotorDetect_Start(300) && !output_requested);
    boot(MOTOR_WORK_SINGLE_M1); MotorControl_StopAll();
    assert(MotorControl_SelectSixStep(true));
    assert(!MotorDetect_Start(300) && !output_requested);
    setup(1.8,11,1); MotorDetect_Stop(); healthy=false;
    assert(!MotorDetect_Start(300) && !output_requested); healthy=true;
    calibrated=false;assert(!MotorDetect_Start(300) && !output_requested);calibrated=true;
    encoder_velocity=5; assert(!MotorDetect_Start(300) && !output_requested);encoder_velocity=0;
    setup(1.8,11,1); frame.m1_current[0]=1;
    ++frame.sequence;HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(!MotorDetect_Active() && !MotorDetect_Result() && !output_requested);
    assert(!strcmp(MotorDetect_Status(),"CURRENT"));
    puts("PASS: actual M1 detection on independent RL/PM rotor plant, preset-independent resistance/poles, reverse encoder/wrap, locked rotor rejection, ADC sequence/timeout, encoder loss, command exclusion, Stop and KEY1; no auto-apply/restart");
}
