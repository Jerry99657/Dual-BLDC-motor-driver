/* Actual adapter + actual M1 FOC + coordinator + protocol. Peripherals and ST
 * binaries are mocked; these assertions do not establish real motor stability. */
#define MOTOR_HOST_REAL_HSO
#include "test_control.c"
#include "hso_binary_stub.h"
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
int main(void) {
    boot((MotorWorkMode)(MOTOR_WORK_SINGLE_M1|8U)); /* Persisted HSO at boot: no encoder alignment. */
    healthy=aligned=0;
    assert(MotorControl_HsoAllowed() && !JerryFOC_needsAlignment());
    sensor_step(2000);
    assert(MotorControl_SelectProfile(MOTOR_AXIS_1,MOTOR_PROFILE_4015));
    assert(MotorControl_HsoAllowed()); /* Profile reload restarts its quiet admission window. */
    assert(!JerryFOC_needsAlignment());
    assert(!MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    sensor_step(2000);assert(StHso_AtRest());
    binary_jump=-0.502654825f; /* Real FOC handover at the reported -28.8 deg offset. */
    assert(MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    assert(output_requested && state(MOTOR_AXIS_1).current_loop);
    sensor_control(); assert(!healthy && !aligned);
    JerryFOC_Target_Iq=0.4f;
    for(unsigned i=0;i<10;++i) JerryFOC_run(); /* No outer-loop tick: no recursive blending. */
    assert(fabsf(JerryFOC_Target_Iq-0.4f)<1e-6f);
    assert(!strcmp(JerryFOC_getFeedbackText(),"HSO"));
    assert(!MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_POSITION,1,10,false));
    assert(!MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_TORQUE,1,0,false));
    float target=JerryFOC_Target_Velocity;
    assert(!MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,-10,0,false));
    assert(JerryFOC_Target_Velocity==target);
    assert(!MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,1,0,false));
    assert(JerryFOC_Target_Velocity==target);
    assert(MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,20,0,false));
    assert(state(MOTOR_AXIS_1).current_loop); /* Settings Off cannot turn HSO into voltage mode. */
    sensor_step(2000);assert(output_requested && !healthy);
    binary_jump=1; sensor_step(1);
    assert(!output_requested && JerryFOC_hasHSOFault() && !JerryFOC_isHSOActive());
    binary_jump=0;sensor_step(3000);assert(!output_requested);
    assert(MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    sensor_control();
    ready=false;ibus_trip=true;sensor_step(1);
    assert(!strcmp(state(MOTOR_AXIS_1).status,"M1 IBUS TRIP")); /* Do not hide hardware trip under HSO STOPPED. */
    assert(!output_requested && JerryFOC_hasHSOFault()); /* ADC/bus fault wins over sensorless operation. */
    ready=true;ibus_trip=false;sensor_step(2500);
    assert(MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    sensor_control();
    command("Speed:0");assert(!output_requested && !JerryFOC_isHSOActive());
    sensor_step(2000);assert(!output_requested);
    assert(MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,-10,0,false));
    sensor_control(); assert(StHso_ControlVelocity()<0);
    command("HSO:Off");assert(!output_requested && !JerryFOC_isHSOActive());
    sensor_step(2500);assert(StHso_AtRest()); /* Disabled HSO still monitors coast admission. */
    command("HSO:On"); sensor_step(2500);
    for(unsigned control=0;control<2;++control) {
        assert(MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
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
        assert(!MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
        sensor_step(2500); global_released=true; MotorControl_Service(now);
        assert(global_stopped && !output_requested);
        assert(MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
        assert(!global_stopped && output_requested);
        MotorControl_StopAll(); sensor_step(2500);
    }
    puts("PASS: KEY1 cancels real independent HSO startup and CONTROL without an encoder; no release restart, held command refusal and explicit restart after quiet admission");
    assert(MotorControl_SelectFeedback(false));
    assert(!MotorControl_Command(MOTOR_AXIS_1,JERRYFOC_MODE_VELOCITY,10,0,false));
    puts("PASS: real encoder-disconnected FOC/coordinator startup + I/F learning + HSO, enforced current loop, speed limits/reversal rejection without target mutation, coast Stop/Speed0, fault no-auto-restart + explicit restart, encoder mode still requires encoder");
    return 0;
}
