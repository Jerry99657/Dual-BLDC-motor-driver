/* Real current control/modulator/parameter loader; only hardware and the
 * proprietary HSO binary are replaced by the existing FOC fixture. */
#define main legacy_foc_main
#include "test_foc.c"
#undef main
#include "motor_control.h"

static void applied_dq(MotorAxis axis,float angle,float *d,float *q)
{
    TIM_TypeDef *tim=axis==MOTOR_AXIS_1 ? &tim1 : &tim8;
    float a=(float)tim->CCR1/tim->ARR,b=(float)tim->CCR2/tim->ARR,c=(float)tim->CCR3/tim->ARR;
    assert(a>=0 && a<=1 && b>=0 && b<=1 && c>=0 && c<=1);
    float common=(a+b+c)/3;
    float alpha=(a-common)*bus,beta=(b-c)*bus/sqrtf(3);
    *d=alpha*cosf(angle)+beta*sinf(angle);
    *q=beta*cosf(angle)-alpha*sinf(angle);
}
static void measured_model(MotorAxis axis,unsigned ls,unsigned flux)
{
    MotorCalibration r={(2UL<<24)|(1920UL<<8)|(uint8_t)-11,ls,flux};
    assert(MotorParams_SelectAxis(axis,MOTOR_PROFILE_4015));
    assert(MotorParams_SetCalibration(axis,&r));
}
static void bumpless_tests(void)
{
    for(unsigned axis=0;axis<2;++axis) for(int dir=-1;dir<=1;dir+=2) {
        measured_model(axis,440,7645);
        MotorControllers c;JerryFOC_InitControllers(&c,MotorParams_GetAxis(axis));
        c.q.I=c.d.I=0;c.q.integral_prev=dir*2.8f;c.d.integral_prev=-dir*.1f;
        float d,q,peak=0;bus=12;
        /* PI already provides the voltage at handover. A change of FF enable
         * gain alone must redistribute PI/FF, not add voltage to the motor. */
        for(unsigned i=0;i<600;++i) {
            JerryFOC_CurrentControl(axis,&c,.4f,dir*25.9f,.1f,.3f,.1f,.3f);
            applied_dq(axis,.4f,&d,&q);
            peak=fmaxf(peak,hypotf(d+dir*.1f,q-dir*2.8f));
        }
        printf("GAIN TRANSFER ONLY: M%u dir=%d max dq voltage change=%.6f V\n",axis+1,dir,peak);
        fflush(stdout);assert(peak<.005f);
        assert(c.current_feedforward==1);
        /* Once enabled, a REAL change of model speed must still act as FF.
         * Cancelling every FF delta would silently turn feedforward off. */
        float oldq=q;
        JerryFOC_CurrentControl(axis,&c,.4f,dir*35.9f,.1f,.3f,.1f,.3f);
        applied_dq(axis,.4f,&d,&q);
        assert(fabsf(q-oldq-dir*110*(.007645f+.00044f*.1f))<.005f);
    }
    puts("PASS: gain-only PI/FF transfer preserves voltage; actual speed feedforward remains active");
}
static void feedforward_tests(void)
{
    measured_model(MOTOR_AXIS_1,440,9290);
    measured_model(MOTOR_AXIS_2,880,6000);
    for(unsigned axis=0;axis<2;++axis) for(int dir=-1;dir<=1;dir+=2)
    for(unsigned v=0;v<2;++v) for(unsigned a=0;a<12;++a) {
        MotorControllers c;JerryFOC_InitControllers(&c,MotorParams_GetAxis(axis));
        c.q.P=c.q.I=c.d.P=c.d.I=0;c.current_feedforward=1;
        bus=v ? 20 : 12;
        float angle=(float)a*0.53f,speed=dir*40.0f,d,q;
        JerryFOC_Target_Velocity=-dir*500; /* Target speed cannot set FF. */
        JerryFOC_CurrentControl(axis,&c,angle,speed,.1f,.4f,.1f,.4f);
        applied_dq(axis,angle,&d,&q);
        float l=axis ? .00088f : .00044f,flux=axis ? .006f : .00929f;
        assert(fabsf(d-(-11*speed*l*.4f))<.005f);
        assert(fabsf(q-(11*speed*(flux+l*.1f)))<.005f);
    }
    MotorControllers c;JerryFOC_InitControllers(&c,MotorParams_GetAxis(0));
    c.q.P=c.q.I=c.d.P=c.d.I=0;
    float d,q;bus=20;
    for(unsigned i=0;i<600;++i) {
        JerryFOC_CurrentControl(0,&c,0,40,0,0,0,0);
        applied_dq(0,0,&d,&q);
        assert(fabsf(q)<.005f && fabsf(d)<.005f);
    }
    assert(c.current_feedforward==1 && fabsf(q)<.005f);
    assert(fabsf(c.q.integral_prev+4.0876f)<.001f);
    JerryFOC_ResetControllers(&c);
    assert(c.current_feedforward==0 && c.q.integral_prev==0 && c.d.integral_prev==0);
    MotorCalibration empty={0};assert(MotorParams_SetCalibration(0,&empty));
    JerryFOC_InitControllers(&c,MotorParams_GetAxis(0));
    c.q.P=c.q.I=c.d.P=c.d.I=0;c.current_feedforward=1;
    JerryFOC_CurrentControl(0,&c,0,40,0,0,0,0);
    zero_pwm(); /* Unknown factory PM flux supplies no invented back EMF. */
    JerryFOC_CurrentControl(0,&c,0,NAN,0,0,0,0);zero_pwm();
    measured_model(0,440,9290);
    puts("PASS: actual dq/PWM FF at 12/20 V, both axes and directions, phase angles, per-axis measured Ls/flux, actual vs target speed, bumpless 50 ms gain ramp, reset and unknown-flux/speed fallback");
}
static void saturation_tests(void)
{
    for(unsigned axis=0;axis<2;++axis) for(unsigned v=0;v<2;++v) {
        const MotorParams *m=MotorParams_GetAxis(axis);
        MotorControllers c;JerryFOC_InitControllers(&c,m);
        c.q.P=c.d.P=80;c.q.I=c.d.I=0;c.current_feedforward=1;
        bus=v ? 20 : 12;
        float angle=.74f,d,q;
        float requested_q=80+550*m->psi_f,requested_d=24;
        float limit=fminf(bus,m->bus_voltage)*.95f/sqrtf(3);
        float scale=limit/hypotf(requested_q,requested_d);
        JerryFOC_CurrentControl(axis,&c,angle,50,.3f,1,0,0);
        applied_dq(axis,angle,&d,&q);
        assert(fabsf(d-requested_d*scale)<.005f && fabsf(q-requested_q*scale)<.005f);
        assert(hypotf(d,q)<=limit+.005f);
        assert(fabsf(c.q.integral_prev-.2f*(requested_q*scale-requested_q))<.001f);
        assert(fabsf(c.d.integral_prev-.2f*(requested_d*scale-requested_d))<.001f);
        /* Invalid bus or a numerical PI request must give neutral PWM, never
         * NaN-to-integer compare writes or a permanently poisoned integrator. */
        bus=NAN;JerryFOC_CurrentControl(axis,&c,angle,0,0,0,0,0);
        TIM_TypeDef *tim=axis ? &tim8 : &tim1;
        assert(tim->CCR1==tim->CCR2 && tim->CCR2==tim->CCR3);
        bus=12;JerryFOC_CurrentControl(axis,&c,angle,0,0,NAN,0,0);
        assert(tim->CCR1==tim->CCR2 && tim->CCR2==tim->CCR3);
        assert(isfinite(c.q.integral_prev) && isfinite(c.d.integral_prev));
        JerryFOC_ResetControllers(&c);JerryFOC_CurrentControl(axis,&c,angle,0,0,0,0,0);
        assert(tim->CCR1==tim->CCR2 && tim->CCR2==tim->CCR3);
    }
    puts("PASS: total PI+FF vector saturation, untruncated request back-calculation, actual VBUS ceilings, neutral invalid-bus/NaN output and integrator recovery");
}
static void plant_tests(void)
{
    for(unsigned axis=0;axis<2;++axis) for(int dir=-1;dir<=1;dir+=2) {
        measured_model(axis,440,9290);
        MotorControllers c;JerryFOC_InitControllers(&c,MotorParams_GetAxis(axis));
        float id=0,iq=0,md=0,mq=0,angle=0,peak=0,settled=0;
        for(unsigned i=0;i<14000;++i) {
            float speed=dir*(i<5000 ? (float)i*.008f : i<7000 ? 40 : 20);
            float target=dir*(i<9000 ? .4f : i<11000 ? -.4f : 0);
            bus=i>=5000 && i<7000 ? 6 : 20;
            md=.5f*(md+id);mq=.5f*(mq+iq);
            JerryFOC_CurrentControl(axis,&c,angle,speed,0,target,md,mq);
            float d,q;applied_dq(axis,angle,&d,&q);
            assert(hypotf(d,q)<=bus*.95f/sqrtf(3)+.005f);
            float w=11*speed;
            float dd=(d-1.92f*id+w*.00044f*iq)/.00044f;
            float dq=(q-1.92f*iq-w*(.00044f*id+.00929f))/.00044f;
            id+=.0001f*dd;iq+=.0001f*dq;
            assert(isfinite(id) && isfinite(iq));
            assert(fabsf(c.q.integral_prev)<=24 && fabsf(c.d.integral_prev)<=24);
            peak=fmaxf(peak,hypotf(id,iq));
            if(i==4999 || i==8999 || i==10999 || i==13999) {
                float error=hypotf(id,iq-target);assert(error<.01f);
                settled=fmaxf(settled,error);
            }
            angle=fmodf(angle+w*.0001f,2*PI);
        }
        assert(peak<2.5f);
        printf("RL MODEL ONLY: M%u dir=%d current peak=%.4f A, settled dq error=%.5f A\n",axis+1,dir,peak,settled);
    }
    puts("PASS: actual PI/FF/PWM against independent RL+dq/back-EMF plant, current LPF, prescribed +/-speed ramps, bus sag/restoration, braking and zero-current recovery; no board timing claim");
}
static void m1_path_tests(void)
{
    assert(JerryFOC_selectMotor(MOTOR_PROFILE_4015));
    measured_model(0,440,9290);JerryFOC_ReloadMotor();
    aligned=healthy=1;output_requested=ready=calibrated=true;bus=20;
    frame.m1_current[0]=frame.m1_current[1]=0;
    JerryFOC_useCurrentLoop(1);
    m1_controllers.q.P=m1_controllers.q.I=m1_controllers.d.P=m1_controllers.d.I=0;
    m1_controllers.current_feedforward=1;filtered_velocity_global=40;
    JerryFOC_Target_Velocity=-100;JerryFOC_Target_Iq=0;
    JerryFOC_run();float d,q;applied_dq(0,0,&d,&q);
    assert(fabsf(q-4.0876f)<.005f && fabsf(d)<.005f);
    JerryFOC_useCurrentLoop(0);JerryFOC_Target_Iq=2;
    JerryFOC_run();applied_dq(0,0,&d,&q);
    assert(fabsf(q-2)<.005f && fabsf(d)<.005f);
    JerryFOC_stop();zero_pwm();assert(m1_controllers.current_feedforward==0);
    assert(!output_requested);
    puts("PASS: actual M1 run adapter uses applied calibration and feedback speed, voltage mode remains direct volts, Stop clears feedforward and disconnects output");
}
#ifdef MOTOR_INVERTER_HOST
#define main legacy_current_main
#endif
int main(void)
{
    tim1.ARR=tim8.ARR=8499;healthy=aligned=1;ready=calibrated=output_requested=true;
    JerryFOC_setAngleSource(&source);
    bumpless_tests();feedforward_tests();saturation_tests();plant_tests();m1_path_tests();
    return 0;
}
#undef main
