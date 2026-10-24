/* Legacy M1/parser/UI regression adapter. The new real dual controller is
 * exercised separately in test_control.c, not replaced by this adapter there. */
#include "motor_control.h"
#include "app_storage.h"
#include "motor_sampling.h"
#include "MT6701.h"
#include "angle_src_encoder.h"
#include "motor_sixstep.h"
#define WEAK __attribute__((weak))
#ifndef CONTROL_STUB_UI
bool SixStep_IsGeneric(void) { return false; }
void SixStep_SetGeneric(bool enable) { (void)enable; }
#endif
bool MotorControl_IsFastFollow(MotorAxis a) { (void)a; return false; }
float MotorControl_SpeedReference(MotorAxis a,float previous,float target)
{
    (void)a;
    float step=(previous*target<0 || fabsf(target)<fabsf(previous) ? 150 : 100)*0.001f;
    if (previous*target<0) target=0;
    return previous+fminf(step,fmaxf(-step,target-previous));
}
float MotorControl_VelocityControl(MotorAxis a,PIDController *p,float reference,float measured)
{ (void)a; return JerryFOC_PID_Calc(p,reference-measured); }
extern float JerryFOC_getAngle(void) WEAK;
extern float JerryFOC_getIq(void) WEAK;
extern float JerryFOC_getId(void) WEAK;
extern float JerryFOC_getPhaseCurrent_A(void) WEAK;
extern float JerryFOC_getPhaseCurrent_B(void) WEAK;
extern float JerryFOC_getElectricalAngle(void) WEAK;
extern float JerryFOC_getTargetVelocity(void) WEAK;
extern JerryFOC_ControlMode JerryFOC_getMode(void) WEAK;
extern uint8_t JerryFOC_isAligning(void) WEAK;
extern uint8_t JerryFOC_selectMotor(MotorProfile) WEAK;
extern float JerryFOC_PID_Calc(PIDController *,float) WEAK;
extern void JerryFOC_setAbsolutePosition(float,float) WEAK;
extern void JerryFOC_setPosition(float,float) WEAK;
extern uint8_t MT6701_IsHealthy(void) WEAK;
extern uint8_t AngleSrc_Encoder_IsAligned(void) WEAK;
extern bool MotorSampling_IsOutputRequested(void) WEAK;
extern const char *MotorSampling_GetStatusText(void) WEAK;
extern bool MotorSampling_IsOutputEnabled(void) WEAK;
extern void MotorSampling_GetDiagnostics(MotorSamplingDiagnostics *) WEAK;
extern volatile float JerryFOC_Target_Iq WEAK, JerryFOC_Target_Position WEAK;
#ifndef CONTROL_STUB_UI
static AppSettings settings;
AppSettings *AppStorage_Get(void) { return &settings; }
bool AppStorage_RequestSave(void) { return false; }
#endif
#ifdef CONTROL_STUB_UI
static bool storage_deferred;
void ControlStub_SetDeferred(bool value) { storage_deferred=value; }
bool AppStorage_IsDeferred(void) { return storage_deferred; }
#else
bool AppStorage_IsDeferred(void) { return false; }
#endif
static MotorWorkMode work;
static MotorAxis focus;
static bool hso=false;
static bool following;
#ifdef CONTROL_STUB_UI
static MotorAxisSnapshot second_ui={.healthy=true,.aligned=true,.ready=true,.status="READY"};
void ControlStub_SetSecondState(MotorAxisSnapshot s) { second_ui=s; }
int8_t Protocol_GetControlResult(void) { return 1; }
#endif
bool MotorControl_HsoAllowed(void) { return work==MOTOR_WORK_SINGLE_M1 && hso; }
static bool sixstep;
bool MotorControl_SixStepAllowed(void) { return work==MOTOR_WORK_SINGLE_M1 && sixstep; }
bool MotorControl_SelectSixStep(bool e) {
    if(work!=MOTOR_WORK_SINGLE_M1) return false;
    sixstep=e; if(e) hso=false;
    SixStep_SetGeneric(e && (AppStorage_Get()->control_config & APP_CONFIG_ESC));
    AppStorage_Get()->control_config=(AppStorage_Get()->control_config & ~(APP_CONFIG_SIXSTEP|8U)) |
        (e ? APP_CONFIG_SIXSTEP : 0U);
    return true;
}
bool MotorControl_IsHandFollow(void) { return work>=MOTOR_WORK_MASTER_M1; }
bool MotorControl_IsPassive(MotorAxis a) { return work==MOTOR_WORK_MASTER_M1+(unsigned)a; }
float MotorControl_PositionVelocity(MotorAxis a,PIDController *p,float error)
{ (void)a; return JerryFOC_PID_Calc ? JerryFOC_PID_Calc(p,error) : p->P*error; }
bool MotorControl_StartFollow(void) {
    if (!MotorControl_IsHandFollow()) return false;
    if (following) return true;
    MotorAxis a=work==MOTOR_WORK_MASTER_M1 ? MOTOR_AXIS_2 : MOTOR_AXIS_1;
    following=MotorControl_Command(a,JERRYFOC_MODE_POSITION,0,30,false); return following;
}
bool MotorControl_MotionAllowed(MotorAxis a) { (void)a; return true; }
void MotorControl_Fast(void) {}
void MotorControl_Outer(void) {}
void MotorControl_PrepareOuter(void) {}
void MotorControl_RecordCycles(uint32_t c) { (void)c; }
void MotorControl_Telemetry(float d[25]) { (void)d; }
MotorWorkMode MotorControl_GetWork(void) { return work; }
MotorAxis MotorControl_GetFocus(void) { return focus; }
void MotorControl_SetFocus(MotorAxis a) {
    focus=a;
    if(work==MOTOR_WORK_INDEPENDENT)
        AppStorage_Get()->control_config=(AppStorage_Get()->control_config & ~APP_CONFIG_FOCUS_M2) |
            (a==MOTOR_AXIS_2 ? APP_CONFIG_FOCUS_M2 : 0U);
}
const char *MotorControl_WorkText(void) {
    static const char *const names[]={"Single M1","Single M2","Independent","Sync speed","Sync position","Master M1","Master M2"};
    return names[work];
}
const char *MotorControl_CommandText(void) { return "READY"; }
const char *MotorControl_EncoderLostText(MotorAxis a) { return a==MOTOR_AXIS_1 ? "ENCODER1 LOST" : "ENCODER2 LOST"; }
bool MotorControl_IsCoupled(void) { return work>=MOTOR_WORK_SYNC_SPEED; }
bool MotorControl_IsGroupActive(void) { return following; }
float MotorControl_GroupError(void) { return 0; }
uint32_t MotorControl_MaxCycles(void) { return 0; }
bool MotorControl_SelectWork(MotorWorkMode m) {
    work=m;
    SixStep_SetGeneric(MotorControl_SixStepAllowed() && (AppStorage_Get()->control_config & APP_CONFIG_ESC));
    focus=m==MOTOR_WORK_SINGLE_M2 || (m==MOTOR_WORK_INDEPENDENT && (AppStorage_Get()->control_config & APP_CONFIG_FOCUS_M2)) ? MOTOR_AXIS_2 : MOTOR_AXIS_1;
    AppStorage_Get()->control_config=(AppStorage_Get()->control_config & ~7U) | m;
    return true;
}
bool MotorControl_SelectFeedback(bool e) {
    sixstep=false;
    SixStep_SetGeneric(false);
    AppStorage_Get()->control_config&=~APP_CONFIG_SIXSTEP;
    hso=e;
    AppStorage_Get()->control_config=(AppStorage_Get()->control_config & ~8U) | (e ? 8U : 0U);
    return true;
}
bool MotorControl_SelectProfile(MotorAxis a,MotorProfile p)
{
    bool ok=a==MOTOR_AXIS_1 && JerryFOC_selectMotor ? JerryFOC_selectMotor(p)!=0 : MotorParams_SelectAxis(a,p)!=0;
    if(ok && a==MOTOR_AXIS_1) {
        AppStorage_Get()->control_config&=~APP_CONFIG_ESC;
        SixStep_SetGeneric(false);
    }
    return ok;
}
bool MotorControl_SelectGeneric(void) {
    if(!MotorControl_SixStepAllowed() || MotorSampling_IsOutputRequested()) return false;
    AppStorage_Get()->control_config|=APP_CONFIG_ESC;
    SixStep_SetGeneric(true);
    return true;
}
bool MotorControl_SetGear(int32_t r,int32_t o) { (void)o; return r!=0; }
bool MotorControl_RequestAlign(MotorAxis a) { (void)a; return true; }
bool MotorControl_ModeAllowed(JerryFOC_ControlMode m) {
    if(MotorControl_SixStepAllowed()) return m==JERRYFOC_MODE_TORQUE ||
        (!SixStep_IsGeneric() && m==JERRYFOC_MODE_VELOCITY);
    if(MotorControl_HsoAllowed()) return m==JERRYFOC_MODE_VELOCITY;
    return !MotorControl_IsHandFollow() || m==JERRYFOC_MODE_POSITION;
}
void MotorControl_Stop(MotorAxis a) {
    if (MotorControl_IsCoupled()) { MotorControl_StopAll(); return; }
#ifdef CONTROL_STUB_UI
    if (a==MOTOR_AXIS_2) { second_ui.active=false; second_ui.target_iq=second_ui.target_velocity=second_ui.target_position=0; return; }
#endif
    (void)a; JerryFOC_stop();
}
void MotorControl_StopAll(void) {
    following=false;
    JerryFOC_stop();
#ifdef CONTROL_STUB_UI
    second_ui.active=false; second_ui.target_iq=second_ui.target_velocity=second_ui.target_position=0;
#endif
}
bool MotorControl_Command(MotorAxis a,JerryFOC_ControlMode m,float t,float l,bool c)
{
#ifdef CONTROL_STUB_UI
    if (MotorControl_SixStepAllowed() && t==0) { JerryFOC_setMode(m); JerryFOC_stop(); return true; }
    if (a==MOTOR_AXIS_1 && MotorControl_HsoAllowed() && m==JERRYFOC_MODE_VELOCITY && t==0) {
        JerryFOC_stop(); return true; /* Real HSO zero is coast. */
    }
    if (a==MOTOR_AXIS_2) {
        second_ui.active=true; second_ui.mode=m; second_ui.current_loop=c;
        if (m==JERRYFOC_MODE_VELOCITY) second_ui.target_velocity=t;
        else if (m==JERRYFOC_MODE_POSITION) second_ui.target_position=t;
        else second_ui.target_iq=t;
        return true;
    }
#endif
    (void)a;
    g_openloop_debug=0;
    JerryFOC_useCurrentLoop(c || MotorControl_HsoAllowed()); JerryFOC_setMode(m);
    if (m==JERRYFOC_MODE_VELOCITY) JerryFOC_setVelocity(t);
    else if (m==JERRYFOC_MODE_POSITION) {
        if (JerryFOC_setAbsolutePosition) JerryFOC_setAbsolutePosition(t,l);
        else if (JerryFOC_setPosition) JerryFOC_setPosition(t,l);
    } else JerryFOC_setCurrent(t);
    return JerryFOC_startOutput()!=0;
}
void MotorControl_GetAxis(MotorAxis a,MotorAxisSnapshot *s)
{
#ifdef CONTROL_STUB_UI
    if (a==MOTOR_AXIS_2) { *s=second_ui; return; }
#endif
    (void)a;
    *s=(MotorAxisSnapshot){.ready=JerryFOC_isDriveReady()!=0,
        .healthy=MT6701_IsHealthy ? MT6701_IsHealthy()!=0 : true,
        .aligned=AngleSrc_Encoder_IsAligned ? AngleSrc_Encoder_IsAligned()!=0 : true,
        .aligning=JerryFOC_isAligning ? JerryFOC_isAligning()!=0 : false,
        .active=MotorSampling_IsOutputRequested ? MotorSampling_IsOutputRequested() : true,
        .velocity=JerryFOC_getVelocity(),.position=JerryFOC_getAngle ? JerryFOC_getAngle() : 0,
        .mode=JerryFOC_getMode ? JerryFOC_getMode() : JERRYFOC_MODE_TORQUE,
        .iq=JerryFOC_getIq ? JerryFOC_getIq() : 0,.id=JerryFOC_getId ? JerryFOC_getId() : 0,
        .ia=JerryFOC_getPhaseCurrent_A ? JerryFOC_getPhaseCurrent_A() : 0,
        .ib=JerryFOC_getPhaseCurrent_B ? JerryFOC_getPhaseCurrent_B() : 0,
        .electrical_angle=JerryFOC_getElectricalAngle ? JerryFOC_getElectricalAngle() : 0,
        .target_velocity=JerryFOC_getTargetVelocity ? JerryFOC_getTargetVelocity() : 0,
        .target_iq=&JerryFOC_Target_Iq ? JerryFOC_Target_Iq : 0,
        .target_position=&JerryFOC_Target_Position ? JerryFOC_Target_Position : 0,
        .status=MotorSampling_GetStatusText ? MotorSampling_GetStatusText() : "READY"};
}
void MotorSampling_GetAxisDiagnostics(MotorAxis a,MotorSamplingDiagnostics *o)
{ (void)a; if (MotorSampling_GetDiagnostics) MotorSampling_GetDiagnostics(o); }
bool MotorSampling_IsAxisOutputRequested(MotorAxis a)
{
#ifdef CONTROL_STUB_UI
    if (a==MOTOR_AXIS_2) return second_ui.active;
#endif
    (void)a; return MotorSampling_IsOutputRequested ? MotorSampling_IsOutputRequested() : false;
}
bool MotorSampling_IsAxisOutputEnabled(MotorAxis a)
{
#ifdef CONTROL_STUB_UI
    if (a==MOTOR_AXIS_2) return second_ui.active;
#endif
    (void)a; return MotorSampling_IsOutputEnabled ? MotorSampling_IsOutputEnabled() : false;
}
bool MotorSampling_IsAxisReady(MotorAxis a) { (void)a; return JerryFOC_isDriveReady()!=0; }
void MT6701_2_FastService(void) {}
#ifndef CONTROL_STUB_UI
void MT6701_2_GetDiagnostics(MT6701Diagnostics *o) { *o=(MT6701Diagnostics){0}; }
#endif
