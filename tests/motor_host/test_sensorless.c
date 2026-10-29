/* Actual adapter + actual M1 FOC + coordinator + protocol. Peripherals and ST
 * binaries are mocked; these assertions do not establish real motor stability. */
#define MOTOR_HOST_REAL_HSO
#include "test_control.c"
#include "hso_binary_stub.h"
/* These startup/recovery tests use one USB owner throughout; the coordinator
 * suite separately verifies that LOCAL/USB/UART cannot steal an active axis. */
static bool sensor_command(MotorAxis axis,JerryFOC_ControlMode mode,float target,
                           float position_speed,bool current_loop)
{
    return MotorControl_CommandFrom(axis,mode,target,position_speed,current_loop,MOTOR_INPUT_USB);
}
static unsigned sample_ticks;
static void sensor_step(unsigned count) {
    for(unsigned i=0;i<count;++i) {
        float a=StHso_ControlAngle(),d=0,q=JerryFOC_Target_Iq;
        StHso_BlendCurrent(&d,&q);
        if (binary_zero_current) d=q=0;
        float ca=cosf(a),sa=sinf(a),ia=d*ca-q*sa,ib=d*sa+q*ca;
        frame.m1_current[0]=ia;
        frame.m1_current[1]=-0.5f*ia+0.86602540378f*ib;
        frame.m1_current[2]=-frame.m1_current[0]-frame.m1_current[1];
        for(unsigned j=0;j<3;++j) { frame.m1_current_raw[j]=2048;frame.m1_voltage_raw[j]=1350; }
        ++frame.sequence; ++sample_ticks;
        HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
        if(sample_ticks%10==0) { ++now;HAL_TIM_PeriodElapsedCallback(&(TIM_HandleTypeDef){.Instance=TIM2}); }
    }
}
static void sensor_control(void) {
    for(unsigned i=0;i<79000 && !StHso_IsUsable();++i) sensor_step(1);
    assert(StHso_IsUsable() && output_requested && JerryFOC_isHSOActive());
}
/* R110 current PI calculation, retained here as an independent pre-FF oracle.
 * Its component output clip is inactive for the normal-current trace below;
 * the final voltage-vector saturation is exercised at each bus voltage. */
static float pre_ff_pi(PIDController *p,float error) {
    float integral=p->integral_prev+p->I*.00005f*(error+p->error_prev);
    p->integral_prev=fminf(p->limit,fmaxf(-p->limit,integral));p->error_prev=error;
    float requested=p->P*error+p->integral_prev;
    assert(fabsf(requested)<p->limit);
    return fminf(p->limit,fmaxf(-p->limit,requested));
}
static void pure_pi_control(void) {
    sensor_step(1600); /* Beyond every previous automatic FF enable delay. */
    assert(StHso_IsUsable() && !healthy && !aligned);
    assert(m1_controllers.current_feedforward==0);
    MotorControllers c;JerryFOC_InitControllers(&c,MotorParams_GetAxis(MOTOR_AXIS_1));
    c.q.I=c.d.I=0;c.q.integral_prev=.8f;c.d.integral_prev=-.1f;
    c.current_feedforward=1; /* Even stale encoder-mode gain must be discarded. */
    for(unsigned i=0;i<100;++i) {
        float speed=i%3==0 ? NAN : i%3==1 ? 500 : -500;
        float angle=StHso_ControlAngle();
        JerryFOC_CurrentControl(MOTOR_AXIS_1,&c,angle,speed,-.1f,.3f,-.1f,.3f);
        float a=(float)tim1.CCR1/tim1.ARR,b=(float)tim1.CCR2/tim1.ARR,v=(float)tim1.CCR3/tim1.ARR;
        float alpha=(a-(a+b+v)/3)*bus,beta=(b-v)*bus/sqrtf(3);
        float d=alpha*cosf(angle)+beta*sinf(angle),q=beta*cosf(angle)-alpha*sinf(angle);
        assert(fabsf(d+.1f)<.005f && fabsf(q-.8f)<.005f);
        assert(fabsf(c.q.integral_prev-.8f)<.0001f && fabsf(c.d.integral_prev+.1f)<.0001f);
        assert(c.current_feedforward==0);
    }
    JerryFOC_InitControllers(&c,MotorParams_GetAxis(MOTOR_AXIS_1));
    MotorControllers legacy=c;float saved_bus=bus;
    for(unsigned i=0;i<3000;++i) {
        bus=i<1000 ? 12 : i<2000 ? 6 : 20;
        float angle=(float)i*.013f,id=.1f*sinf((float)i*.007f),iq=.3f*cosf((float)i*.009f);
        float target_d=.2f*sinf((float)i*.011f),target_q=.5f*cosf((float)i*.005f);
        float requested_q=pre_ff_pi(&legacy.q,target_q-iq),requested_d=pre_ff_pi(&legacy.d,target_d-id);
        JerryFOC_CurrentControl(MOTOR_AXIS_1,&c,angle,i%2 ? 80 : -80,target_d,target_q,id,iq);
        uint32_t a=tim1.CCR1,b=tim1.CCR2,v=tim1.CCR3;float aq,ad;
        JerryFOC_setAxisVoltage(MOTOR_AXIS_1,requested_q,requested_d,angle,&aq,&ad);
        assert(tim1.CCR1==a && tim1.CCR2==b && tim1.CCR3==v);
        legacy.q.integral_prev=fminf(legacy.q.limit,fmaxf(-legacy.q.limit,legacy.q.integral_prev+.2f*(aq-requested_q)));
        legacy.d.integral_prev=fminf(legacy.d.limit,fmaxf(-legacy.d.limit,legacy.d.integral_prev+.2f*(ad-requested_d)));
        assert(c.q.integral_prev==legacy.q.integral_prev && c.d.integral_prev==legacy.d.integral_prev);
        assert(c.current_feedforward==0);
    }
    bus=saved_bus;
    puts("PASS: real encoder-free HSO CONTROL stays pure PI beyond 150 ms, ignores model-speed FF and stale gain, retains PI voltage/integral and protection");
    puts("PASS: 3000 normal-current HSO steps equal pre-FF R110 PI/PWM/integral trace at 6/12/20 V with vector saturation; encoder remains absent");
}
int main(void) {
    boot((MotorWorkMode)(MOTOR_WORK_SINGLE_M1|8U)); /* Persisted HSO at boot: no encoder alignment. */
    healthy=aligned=0;
    assert(MotorControl_HsoAllowed() && !JerryFOC_needsAlignment());
    sensor_step(2000);
    assert(MotorControl_SelectProfile(MOTOR_AXIS_1,MOTOR_PROFILE_4015));
    assert(MotorControl_HsoAllowed()); /* Profile reload restarts its quiet admission window. */
    assert(!JerryFOC_needsAlignment());
    assert(!sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    sensor_step(2000);assert(StHso_AtRest());
    float pending_target=JerryFOC_Target_Velocity;
    command("Speed:4.9"); assert(Protocol_GetControlResult()==-1 && !output_requested && JerryFOC_Target_Velocity==pending_target);
    command("Speed:-4.9"); assert(Protocol_GetControlResult()==-1 && !output_requested && JerryFOC_Target_Velocity==pending_target);
    binary_jump=-0.502654825f; /* Real FOC handover at the reported -28.8 deg offset. */
    assert(sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    assert(output_requested && state(MOTOR_AXIS_1).current_loop);
    sensor_control(); assert(!healthy && !aligned);
    pure_pi_control();
    JerryFOC_Target_Iq=0.4f;
    for(unsigned i=0;i<10;++i) JerryFOC_run(); /* No outer-loop tick: no recursive blending. */
    assert(fabsf(JerryFOC_Target_Iq-0.4f)<1e-6f);
    assert(!strcmp(JerryFOC_getFeedbackText(),"HSO"));
    assert(!sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_POSITION,1,10,false));
    assert(!sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_TORQUE,1,0,false));
    float target=JerryFOC_Target_Velocity;
    command("Speed:4.9"); assert(JerryFOC_Target_Velocity==target && output_requested);
    assert(!sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,-10,0,false));
    assert(JerryFOC_Target_Velocity==target);
    assert(!sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,1,0,false));
    assert(JerryFOC_Target_Velocity==target);
    assert(sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,20,0,false));
    assert(state(MOTOR_AXIS_1).current_loop); /* Settings Off cannot turn HSO into voltage mode. */
    sensor_step(2000);assert(output_requested && !healthy);
    binary_jump=1; sensor_step(1);
    assert(!output_requested && JerryFOC_hasHSOFault() && !JerryFOC_isHSOActive());
    binary_jump=0;sensor_step(3000);assert(!output_requested);
    assert(sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    sensor_control();
    ready=false;ibus_trip=true;sensor_step(1);
    assert(!strcmp(state(MOTOR_AXIS_1).status,"M1 IBUS TRIP")); /* Do not hide hardware trip under HSO STOPPED. */
    assert(!output_requested && JerryFOC_hasHSOFault()); /* ADC/bus fault wins over sensorless operation. */
    ready=true;ibus_trip=false;sensor_step(2500);
    assert(sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    sensor_control();
    command("Speed:0");assert(!output_requested && !JerryFOC_isHSOActive());
    sensor_step(2000);assert(!output_requested);
    assert(sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,-10,0,false));
    sensor_control(); assert(StHso_ControlVelocity()<0);
    command("HSO:Off");assert(!output_requested && !JerryFOC_isHSOActive());
    sensor_step(2500);assert(StHso_AtRest()); /* Disabled HSO still monitors coast admission. */
    command("HSO:On"); sensor_step(2500);
    for(unsigned control=0;control<2;++control) {
        assert(sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
        if(control) sensor_control();
        else assert(StHso_IsStarting());
        global_stopped=global_pending=true; global_released=false; output_requested=false;
        if(control) {
            binary_jump=1; sensor_step(1500); binary_jump=0;
            assert(!JerryFOC_hasHSOFault()); /* Intentional coast is not a new observer fault. */
        }
        MotorControl_Service(now);
        assert(!JerryFOC_isHSOActive() && !StHso_IsStarting() && !StHso_IsRunning());
        assert(!output_requested && JerryFOC_Target_Velocity==0);
        assert(!sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
        sensor_step(2500); global_released=true; MotorControl_Service(now);
        assert(global_stopped && !output_requested);
        assert(sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
        assert(!global_stopped && output_requested);
        MotorControl_StopAll(); sensor_step(2500);
    }
    puts("PASS: KEY1 cancels real independent HSO startup and CONTROL without an encoder; no release restart, held command refusal and explicit restart after quiet admission");
    assert(MotorControl_SelectFeedback(false));
    assert(!sensor_command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    puts("PASS: real encoder-disconnected FOC/coordinator startup + I/F learning + HSO, enforced current loop, speed limits/reversal rejection without target mutation, coast Stop/Speed0, fault no-auto-restart + explicit restart, encoder mode still requires encoder");
    return 0;
}
