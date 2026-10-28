#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "JerryFOC.h"
#include "motor_sampling.h"
#include "adc.h"
#include "angle_src_encoder.h"
#include "st_hso.h"
#include "protocol.h"
#include "MT6701.h"
#include "motor_sixstep.h"
#include "uart_port.h"
#include "usb_device.h"
#include "app_storage.h"
AppSaveState AppStorage_GetSaveState(void) { return APP_SAVE_IDLE; }

USBD_HandleTypeDef hUsbDeviceFS;
volatile uint8_t g_usb_cdc_ready;
static float telemetry_reference;
bool Uart1_TrySend(const uint8_t *p,uint16_t n) {
    if(n==104) memcpy(&telemetry_reference,p+7*sizeof(float),sizeof(float));
    return true;
}
uint8_t CDC_Transmit_FS(uint8_t *p,uint16_t n) { (void)p; (void)n; return 0; }
uint8_t CDC_TxReady_FS(void) { return 0; }
#ifndef MOTOR_HOST_REAL_HSO
void StHso_Enable(bool en) { (void)en; }
bool StHso_LearnFlux(void) { return true; }
void StHso_GetConfig(StHsoConfig *out) { *out=(StHsoConfig){0}; }
bool StHso_SetConfig(const StHsoConfig *c) { (void)c; return true; }
#endif
static void command(const char *s) {
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)s,(uint32_t)strlen(s));
    Protocol_Feed(PROTO_SRC_USB,(const uint8_t *)"\n",1);
    Protocol_Service();
}

HostDwt host_dwt;
uint32_t host_primask;
ADC_TypeDef adc1,adc2;
ADC_HandleTypeDef hadc1={.Instance=ADC1},hadc2={.Instance=ADC2};
TIM_TypeDef tim1,tim8,tim6;
TIM_HandleTypeDef htim1={&tim1},htim8={&tim8},htim6={&tim6};
static MotorSampleFrame frame;
static float bus=12.0f;
static bool ready=true, calibrated=true, complete=true;
static bool output_requested=true;
static bool global_stopped,global_pending,global_released=true;
bool MotorSampling_SelectSixStep(bool enable) { (void)enable; return !output_requested; }
bool MotorSampling_SixStepTimer(uint32_t us) { (void)us; return true; }
bool MotorSampling_SixStepPattern(uint8_t sector,float duty) { (void)sector;(void)duty; return ready && !global_stopped; }
bool MotorSampling_IsGlobalStopped(void) { return global_stopped; }
bool MotorSampling_TakeGlobalStop(void) { bool p=global_pending; global_pending=false; return p; }
bool MotorSampling_ResumeGlobalStop(void) {
    if(global_stopped && (!global_released || global_pending)) return false;
    global_stopped=false; return true;
}
static unsigned captures, encoder_services, aligns;
static uint8_t healthy=1, aligned=1;
static unsigned hso_processes;
#ifndef MOTOR_HOST_REAL_HSO
static bool hso_usable,hso_running;
static float hso_velocity;
bool StHso_IsUsable(void) { return hso_usable; }
bool StHso_CanTakeover(void) { return hso_usable; }
bool StHso_Start(float t) { hso_running=hso_usable && fabsf(t)>3; return hso_running; }
void StHso_Stop(void) { hso_running=false; }
bool StHso_IsStarting(void) { return hso_running && !hso_usable; }
bool StHso_IsRunning(void) { return hso_running && hso_usable; }
bool StHso_AtRest(void) { return true; }
bool StHso_ValidTarget(float t) { return isfinite(t) && fabsf(t)>=3; }
float StHso_MinSpeed(void) { return 3; }
float StHso_Angle(void) { return 0.1f; }
float StHso_Velocity(void) { return hso_velocity; }
float StHso_Position(void) { return NAN; }
float StHso_ControlAngle(void) { return 0.1f; }
float StHso_ControlVelocity(void) { return hso_velocity; }
float StHso_CurrentD(void) { return 0; }
void StHso_BlendCurrent(float *d,float *q) { *d=0; (void)q; }
void StHso_GetSnapshot(StHsoSnapshot *out) { *out=(StHsoSnapshot){.reason=ST_HSO_REASON_TIMING}; }
const char *StHso_ReasonText(StHsoReason reason) { (void)reason; return "TIMING"; }
void StHso_Process(const MotorSampleFrame *f,bool r,float a,float v,bool e) {
    (void)f;(void)r;(void)a;(void)v;(void)e; ++hso_processes;
}
void StHso_RecordControlCycles(uint32_t c) { (void)c; }
#endif
static float encoder_velocity;
static float zero(void) { return 0; }
static float encoder_speed(void) { return encoder_velocity; }
static AngleSource source={zero,zero,encoder_speed,1};
static int selected_pp;
static const MotorParams *hso_motor;
AngleSource *AngleSrc_Encoder_Init(int pp) { selected_pp=pp; aligned=0; return &source; }
#ifndef MOTOR_HOST_REAL_HSO
void StHso_Init(const MotorParams *motor) { hso_motor=motor; hso_usable=hso_running=false; }
#endif
uint32_t HAL_GetTick(void) { return 0; }
bool MotorSampling_IsBusFresh(void) { return true; }
void MT6701_GetDiagnostics(MT6701Diagnostics *s) { *s=(MT6701Diagnostics){0}; }
#ifdef MOTOR_HOST_REAL_CONTROL
bool MotorSampling_IsAxisOutputEnabled(MotorAxis a) { return MotorSampling_IsAxisOutputRequested(a); }
void MT6701_2_GetDiagnostics(MT6701Diagnostics *s) { *s=(MT6701Diagnostics){0}; }
#else
void MT6701_2_GetSnapshot(MT6701_2_Snapshot *s) { *s=(MT6701_2_Snapshot){0}; }
void SixStep_GetSnapshot(SixStepSnapshot *s) { *s=(SixStepSnapshot){.status="READY"}; }
#endif
void HAL_Delay(uint32_t ms) { (void)ms; }
bool MotorSampling_IsReady(void) { return ready; }
bool MotorSampling_IsCalibrated(void) { return calibrated; }
bool MotorSampling_IsOutputRequested(void) { return output_requested; }
bool MotorSampling_SetOutputEnabled(bool enable) {
    if(enable && (global_stopped || !ready || !calibrated)) return false;
    output_requested=enable; return true;
}
float MotorSampling_GetBusVoltage(void) { return bus; }
bool MotorSampling_Capture(void) { ++captures; return complete; }
const MotorSampleFrame *MotorSampling_GetFrame(void) { return &frame; }
void MotorSampling_CopyFrame(MotorSampleFrame *out) { *out=frame; }
uint8_t MT6701_IsContinuous(void) { return 1; }
uint8_t MT6701_IsHealthy(void) { return healthy; }
static uint8_t encoder_zero_gap;
uint8_t MT6701_HasNearZeroGap(void) { return encoder_zero_gap; }
static uint16_t encoder_raw=1000;
uint16_t MT6701_GetRawAngle(void) { return encoder_raw; }
uint8_t MT6701_GetStatus(void) { return 0; }
uint16_t MT6701_GetSampleAgeCycles(void) { return 0; }
uint32_t MT6701_GetErrorCounter(void) { return 0; }
void MT6701_Service(void) { ++encoder_services; }
void AngleSrc_Encoder_Align(float angle) { (void)angle; ++aligns; }
void AngleSrc_Encoder_Update(float dt) { (void)dt; }
uint8_t AngleSrc_Encoder_IsSource(const AngleSource *s) { return s==&source; }
uint8_t AngleSrc_Encoder_IsAligned(void) { return aligned; }
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *);
static void zero_pwm(void) { assert(tim1.CCR1==tim1.CCR2 && tim1.CCR2==tim1.CCR3); }
int main(void) {
    tim1.ARR=8499;
    JerryFOC_setAngleSource(&source);
    JerryFOC_setPhaseVoltage(3,0,0);
    float spread12=(float)(tim1.CCR2-tim1.CCR3);
    bus=24;
    JerryFOC_setPhaseVoltage(3,0,0);
    float spread24=(float)(tim1.CCR2-tim1.CCR3);
    assert(fabsf(spread12-2*spread24)<=2.0f);
    assert(JerryFOC_getBusVoltage()==24 && JerryFOC_isDriveReady());
    /* Keep the motor's configured voltage ceiling even on a 24 V bus. */
    JerryFOC_setPhaseVoltage(100,0,0);
    float applied=(float)(tim1.CCR2-tim1.CCR3)/tim1.ARR*bus;
    assert(applied<=MOTOR_2804.bus_voltage*0.95f+0.01f);
    calibrated=false; JerryFOC_setPhaseVoltage(3,0,0); zero_pwm();
    JerryFOC_alignStart(); assert(aligns==0); assert(!JerryFOC_isDriveReady());
    calibrated=true;
    frame.m1_current[0]=0.5f; frame.m1_current[1]=-0.25f;
    adc1.JDR1=4095; adc2.JDR1=0; /* Old direct reads would give the wrong current. */
    JerryFOC_setCurrent(1); HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(captures==1 && encoder_services==1 && hso_processes==0);
    assert(JerryFOC_getPhaseCurrent_A()==0.5f && JerryFOC_getPhaseCurrent_B()==-0.25f);
    complete=false; frame.m1_current[0]=1.5f;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(captures==2 && encoder_services==1 && JerryFOC_getPhaseCurrent_A()==0.5f);
    HAL_ADCEx_InjectedConvCpltCallback(&hadc2); assert(captures==2);
    complete=true; JerryFOC_alignStart();
    float align_ud=(float)(tim1.CCR1-tim1.CCR2)*bus/(1.5f*tim1.ARR);
    assert(fabsf(align_ud-MOTOR_2804.Rs*MOTOR_2804.rated_current)<0.01f);
    unsigned duty=tim1.CCR1;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc1);
    assert(captures==3 && encoder_services==2 && tim1.CCR1==duty); /* Align voltage is not overwritten. */
    ready=false; JerryFOC_alignFinish(); zero_pwm(); assert(aligns==0);
    ready=true; JerryFOC_alignStart(); JerryFOC_alignFinish(); zero_pwm(); assert(aligns==1);
    assert(!output_requested && JerryFOC_startOutput());
    calibrated=false; JerryFOC_run(); zero_pwm(); assert(JerryFOC_Target_Iq==0);
    calibrated=true; healthy=0; JerryFOC_setCurrent(3); JerryFOC_run(); zero_pwm(); assert(JerryFOC_Target_Iq==0);
    healthy=1; aligned=0; JerryFOC_setCurrent(3); JerryFOC_run(); zero_pwm(); assert(JerryFOC_Target_Iq==0);
    aligned=1; ready=false;
    JerryFOC_setMode(JERRYFOC_MODE_VELOCITY); JerryFOC_setVelocity(100);
    TIM_HandleTypeDef outer={TIM2}; HAL_TIM_PeriodElapsedCallback(&outer);
    assert(JerryFOC_Target_Iq==0 && vel_loop_M0.integral_prev==0);
    ready=true;
    /* UI and protocol are not the only callers: enforce the same continuous
     * target in setter, outer speed PI and the fast current-loop boundary. */
    JerryFOC_useCurrentLoop(1);
    assert(vel_loop_M0.limit==0.5f);
    JerryFOC_setCurrent(10); assert(JerryFOC_Target_Iq==0.5f);
    JerryFOC_setCurrent(-10); assert(JerryFOC_Target_Iq==-0.5f);
    JerryFOC_setCurrent(NAN); assert(JerryFOC_Target_Iq==0);
    JerryFOC_setVelocity(100);
    for(unsigned i=0;i<1000;++i) HAL_TIM_PeriodElapsedCallback(&outer);
    assert(fabsf(JerryFOC_Target_Iq)<=0.5f);
    JerryFOC_Target_Iq=10; JerryFOC_run(); assert(JerryFOC_Target_Iq==0.5f);
    JerryFOC_Target_Iq=-10; JerryFOC_run(); assert(JerryFOC_Target_Iq==-0.5f);
    JerryFOC_useCurrentLoop(0);
    JerryFOC_setCurrent(3); assert(JerryFOC_Target_Iq==3); /* Voltage command units stay V. */
    assert(!JerryFOC_useHSO(1)); /* Encoder mode never silently selects sensorless. */
    /* A profile change is one stopped transaction: no old target, PI state,
     * HSO takeover or encoder alignment may carry into the other motor. */
    encoder_velocity=0; healthy=1;
    assert(!JerryFOC_selectMotor(MOTOR_PROFILE_COUNT));
    encoder_velocity=2;
    assert(!JerryFOC_selectMotor(MOTOR_PROFILE_4015));
    assert(MotorParams_GetActiveProfile()==MOTOR_PROFILE_2804);
    encoder_velocity=0; healthy=0;
    assert(!JerryFOC_selectMotor(MOTOR_PROFILE_4015)); healthy=1;
    vel_loop_M0.integral_prev=4; iq_loop_M0.integral_prev=5;
    assert(JerryFOC_selectMotor(MOTOR_PROFILE_4015)); zero_pwm();
    assert(JerryFOC_getMotorParams()==&MOTOR_4015 && selected_pp==11 && hso_motor==&MOTOR_4015);
    assert(!aligned && JerryFOC_needsAlignment() && !JerryFOC_isHSOActive());
    assert(JerryFOC_Target_Iq==0 && JerryFOC_Target_Velocity==0 && JerryFOC_Target_Position==0);
    assert(vel_loop_M0.P==MOTOR_4015.vel_P && vel_loop_M0.limit==MOTOR_4015.vel_limit);
    assert(vel_loop_M0.integral_prev==0 && iq_loop_M0.integral_prev==0);
    assert(fabsf(iq_loop_M0.P-MOTOR_4015.Ls*MOTOR_CURRENT_BANDWIDTH_RAD_S)<1e-6f && id_loop_M0.I==MOTOR_4015.Rs*MOTOR_CURRENT_BANDWIDTH_RAD_S);
    JerryFOC_setCurrent(99); assert(JerryFOC_Target_Iq==MOTOR_4015.vel_limit);
    JerryFOC_setCurrent(1); JerryFOC_run(); zero_pwm(); /* Alignment is mandatory. */
    JerryFOC_alignStart(); assert(!JerryFOC_needsAlignment());
    assert(!JerryFOC_selectMotor(MOTOR_PROFILE_2804)); /* No switch during alignment. */
    JerryFOC_alignFinish(); aligned=1;
    JerryFOC_useCurrentLoop(1); JerryFOC_setCurrent(9);
    assert(JerryFOC_Target_Iq==MOTOR_4015.rated_current && vel_loop_M0.limit==MOTOR_4015.rated_current);
    command("Speed:999"); assert(JerryFOC_Target_Velocity==600);
    JerryFOC_setAbsolutePosition(1,999); assert(pos_loop_M0.limit==600);
    ready=false; assert(JerryFOC_selectMotor(MOTOR_PROFILE_2804));
    assert(!JerryFOC_isDriveReady()); /* Switching never clears a sampling fault. */
    assert(selected_pp==7 && hso_motor==&MOTOR_2804);
    assert(vel_loop_M0.limit==6.93f && fabsf(iq_loop_M0.P-0.215f)<1e-6f && !aligned);
    JerryFOC_setCurrent(-99); assert(JerryFOC_Target_Iq==-MOTOR_2804.vel_limit);
    ready=true; aligned=1; encoder_velocity=0;
    command("Speed:10"); assert(output_requested);
    encoder_velocity=10;
    JerryFOC_setVelocity(0); /* A zero speed command retains active deceleration. */
    assert(output_requested);
    vel_loop_M0.integral_prev=3; iq_loop_M0.integral_prev=4;
    command("Stop"); assert(!output_requested);
    assert(JerryFOC_Target_Iq==0 && JerryFOC_Target_Velocity==0 && JerryFOC_Target_Position==0);
    assert(vel_loop_M0.integral_prev==0 && iq_loop_M0.integral_prev==0);
    for(unsigned i=0;i<10;++i) { HAL_TIM_PeriodElapsedCallback(&outer); JerryFOC_run(); }
    assert(!output_requested && JerryFOC_Target_Iq==0 && vel_loop_M0.integral_prev==0);
    command("Speed:10"); assert(!output_requested); /* Still coasting: wait, never auto-restart. */
    encoder_velocity=0; command("Speed:10"); assert(output_requested && JerryFOC_Target_Velocity==10);
    command("Stop"); command("Speed:nan"); assert(!output_requested);
    command("Torque:nan"); command("Angle:bad"); command("OpenLoop:bad,5"); assert(!output_requested);
    command("OpenLoop:2,0.8"); assert(output_requested && g_openloop_debug);
    command("Stop"); ready=false; assert(!JerryFOC_startOutput());
    ready=true; healthy=0; assert(!JerryFOC_startOutput()); healthy=1;
    aligned=0; assert(!JerryFOC_startOutput());
    JerryFOC_alignStart(); assert(JerryFOC_isAligning() && output_requested);
    unsigned before_align_stop=aligns;
    command("Stop"); assert(!output_requested && !JerryFOC_isAligning());
    JerryFOC_alignFinish(); assert(aligns==before_align_stop); /* Cancelled align cannot certify zero. */
    JerryFOC_requestAlignment(); assert(JerryFOC_needsAlignment());
    JerryFOC_stop(); assert(!JerryFOC_needsAlignment() && !output_requested);
    puts("PASS: protocol/FOC, encoder control, voltage/current clamps, atomic 2804/4015 controller reload, stop/align gates and fault retention");
    return 0;
}
