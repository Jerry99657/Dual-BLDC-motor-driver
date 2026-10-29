#include "compact_text.h"
#include "motor_control.h"
#include "board_diag.h"
#include "motor_sampling.h"
#include "MT6701.h"
#include "angle_src_encoder.h"
#include "encoder_estimator.h"
#include "app_storage.h"
#include "st_hso.h"
#include "motor_sixstep.h"
#include "motor_detect.h"
#include <string.h>
#include "motor_math.h"

/* Both axes share encoder estimation code, with separate state. M1 retains
 * its encoder/HSO mux; M2 owns its PI, calibration and run state below. */
typedef struct {
    MotorControllers loops;
    LowPassFilter speed_filter;
    EncoderEstimator encoder;
    float velocity, electrical_zero, electrical_scale;
    float reference, target_speed, target_position, target_q, position_speed;
    float iq, id, ia, ib;
    uint32_t align_at;
    bool aligned, aligning, pending_align, loop;
    JerryFOC_ControlMode mode;
    const char *stop_reason;
} Motor2Controller;
static const char *command_text="READY";
/* Profile and actuator mode cannot change during a follow session. Bind the
 * confirmed preset when arming; a settings draft must not retune a live loop. */

/* Shared storage base; axes still own distinct controller/estimator state. */
static struct {
    volatile MotorWorkMode work;
    MotorAxis focus;
    uint8_t drive;
    uint8_t input[2];
    volatile bool group_active, timing_fault;
    volatile uint32_t axis_flags[2];
    float gear_ratio,gear_offset;
    Motor2Controller m2;
    JerryFOC_ControlMode group_mode;
    float group_target, group_reference, group_speed, origins[2], group_error;
    uint32_t group_started, error_ms, max_cycles;
    const char *m1_stop_reason,*group_stop_reason;
    float group_position_limit;
    const MotorFollowParams *group_follow;
    float follow_estimate[2],follow_velocity[2];
    float follow_weight;
    float follow_activity,follow_gain;
    float follow_motion_anchor,follow_acceleration;
    uint16_t follow_quiet_ms;
    uint16_t follow_count_interval;
    bool follow_quiet,follow_hold;
    float follow_command_position,follow_command_velocity,follow_master_velocity;
} control;

static bool resume_global_stop(void)
{
    if (MotorSampling_ResumeGlobalStop()) return true;
    command_text="GLOBAL STOP"; return false;
}

#define clamp MotorMath_Clamp
static float norm(float a) { return MotorMath_Wrap(a); }
static float slew(float actual,float target,float accel,float decel)
{
    bool braking=actual*target<0.0f || fabsf(target)<fabsf(actual);
    float step=(braking ? decel : accel)*0.001f;
    if (actual*target<0.0f) target=0.0f;
    return actual+clamp(target-actual,-step,step);
}
/* Stopping-distance envelope with a finite response allowance. Shared by
 * moving-target Follow and finite synchronous-position trajectories. */
static float braking_speed(float distance,float decel,float response)
{
    float margin=decel*response;
    return copysignf(sqrtf(margin*margin+2.0f*decel*fabsf(distance))-margin,distance);
}
static float ratio(void) { return control.gear_ratio; }
static float offset(void) { return control.gear_offset; }
__attribute__((noinline,noclone)) MotorInput MotorControl_GetInput(MotorAxis axis)
{
    return (unsigned)axis<MOTOR_AXIS_COUNT && MotorSampling_IsAxisOutputRequested(axis) ?
        (MotorInput)control.input[axis] : MOTOR_INPUT_NONE;
}
const char *MotorControl_InputText(MotorAxis axis)
{
    static const char names[]="--\0LOCAL\0USB\0UART\0FOLLOW";
    return CompactText_At(names,MotorControl_GetInput(axis));
}
static bool input_available(MotorAxis axis,MotorInput input)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT || input<=MOTOR_INPUT_NONE || input>MOTOR_INPUT_FOLLOW) return false;
    for (unsigned i=0;i<2;++i) if (i==(unsigned)axis || MotorControl_IsCoupled()) {
        MotorInput owner=MotorControl_GetInput((MotorAxis)i);
        if (owner!=MOTOR_INPUT_NONE && owner!=input) { command_text="CONTROL BUSY"; return false; }
    }
    return true;
}
static void set_input(MotorAxis axis,MotorInput input)
{
    control.input[axis]=(uint8_t)input;
    if (MotorControl_IsCoupled()) control.input[1U-axis]=(uint8_t)input;
}
bool MotorControl_ClaimInput(MotorAxis axis,MotorInput input)
{
    bool ok=input_available(axis,input);
    if (ok) set_input(axis,input);
    return ok;
}
uint32_t MotorControl_GetOptions(MotorAxis axis)
{
    return (unsigned)axis<MOTOR_AXIS_COUNT ? control.axis_flags[axis] : 0;
}
__attribute__((noinline,noclone)) static float speed_limit(MotorAxis axis)
{
    uint32_t flags=MotorControl_GetOptions(axis);
    unsigned lim=(flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT;
    if (!lim || lim>APP_SPEED_LIMIT_MAX) lim=APP_SPEED_LIMIT_DEFAULT;
    return fminf((float)lim,MotorParams_GetAxis(axis)->max_speed_mech);
}
__attribute__((noinline,noclone)) static float position_speed(MotorAxis axis)
{
    unsigned limit=AppSettings_GetPositionSpeed(MotorControl_GetOptions(axis));
    if (!limit || limit>APP_SPEED_LIMIT_MAX) limit=30;
    return fminf((float)limit,speed_limit(axis));
}
__attribute__((noinline,noclone)) float MotorControl_LimitSpeed(MotorAxis axis,float value)
{
    float cap=speed_limit(axis);
    return value>cap ? cap : value< -cap ? -cap : value;
}
bool MotorControl_SetOptions(MotorAxis axis,uint32_t flags)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    uint32_t changed=flags^control.axis_flags[axis];
    bool powered=MotorSampling_IsAxisOutputRequested(axis);
    if (((changed & APP_FLAG_CURRENT_LOOP) && (powered || control.group_active ||
            (axis==MOTOR_AXIS_1 ? JerryFOC_isAligning()!=0 : control.m2.aligning))) ||
        ((changed & (APP_SPEED_LIMIT_MASK|APP_POSITION_SPEED_MASK)) && powered && axis==MOTOR_AXIS_1 && MotorControl_GetDrive(axis)!=MOTOR_DRIVE_ENCODER)) {
        command_text="Stop / check axis"; __set_PRIMASK(mask); return false;
    }
    control.axis_flags[axis]=flags;
    __set_PRIMASK(mask); return true;
}
__attribute__((noinline,noclone)) bool MotorControl_AxisEnabled(MotorAxis axis)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT) return false;
    return control.work>=MOTOR_WORK_INDEPENDENT ||
        (axis==MOTOR_AXIS_1 ? control.work==MOTOR_WORK_SINGLE_M1 : control.work==MOTOR_WORK_SINGLE_M2);
}
bool MotorControl_IsCoupled(void) { return control.work>=MOTOR_WORK_SYNC_SPEED; }
bool MotorControl_IsHandFollow(void) { return control.work>=MOTOR_WORK_MASTER_M1; }
bool MotorControl_IsPassive(MotorAxis axis) { return control.work==MOTOR_WORK_MASTER_M1+(unsigned)axis; }
__attribute__((noinline,noclone)) MotorDriveMode MotorControl_GetDrive(MotorAxis axis)
{
    return axis!=MOTOR_AXIS_1 || control.work!=MOTOR_WORK_SINGLE_M1 ? MOTOR_DRIVE_ENCODER :
        (MotorDriveMode)control.drive;
}
__attribute__((noinline)) bool MotorControl_HsoAllowed(void) { return MotorControl_GetDrive(MOTOR_AXIS_1)==MOTOR_DRIVE_HSO; }
__attribute__((noinline)) bool MotorControl_SixStepAllowed(void) { return MotorControl_GetDrive(MOTOR_AXIS_1)==MOTOR_DRIVE_SIXSTEP; }
bool MotorControl_MotionAllowed(MotorAxis axis) { return MotorControl_AxisEnabled(axis) && !MotorControl_IsPassive(axis) && !control.timing_fault; }
MotorWorkMode MotorControl_GetWork(void) { return control.work; }
MotorAxis MotorControl_GetFocus(void) { return control.focus; }
void MotorControl_SetFocus(MotorAxis axis)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT) return;
    control.focus=axis;
    if (control.work==MOTOR_WORK_INDEPENDENT) {
        AppSettings *s=AppStorage_Get();
        s->control_config=(s->control_config & ~APP_CONFIG_FOCUS_M2) |
            (axis==MOTOR_AXIS_2 ? APP_CONFIG_FOCUS_M2 : 0U);
    }
}
static MotorAxis work_focus(void)
{
    return control.work==MOTOR_WORK_SINGLE_M2 || (control.work==MOTOR_WORK_INDEPENDENT &&
        (AppStorage_Get()->control_config & APP_CONFIG_FOCUS_M2)) ? MOTOR_AXIS_2 : MOTOR_AXIS_1;
}
bool MotorControl_IsGroupActive(void) { return control.group_active; }
float MotorControl_GroupError(void) { return control.group_error; }
float MotorControl_GroupTarget(void) { return control.group_target; }
uint32_t MotorControl_MaxCycles(void) { return control.max_cycles; }
const char *MotorControl_CommandText(void) { return command_text; }
const char *MotorControl_EncoderLostText(MotorAxis axis)
{
    return axis==MOTOR_AXIS_1 ? "ENCODER1 LOST" : "ENCODER2 LOST";
}
const char *MotorControl_WorkText(void)
{
    static const char names[]=
        "Single M1\0"
        "Single M2\0"
        "Independent\0"
        "Sync speed\0"
        "Sync position\0"
        "Master M1\0"
        "Master M2";
    return CompactText_At(names,control.work);
}
bool MotorControl_ModeAllowed(JerryFOC_ControlMode mode)
{
    if ((unsigned)mode>JERRYFOC_MODE_POSITION) return false;
    if (MotorControl_SixStepAllowed()) return mode==JERRYFOC_MODE_TORQUE ||
        (!SixStep_IsGeneric() && mode==JERRYFOC_MODE_VELOCITY);
    if (MotorControl_HsoAllowed()) return mode==JERRYFOC_MODE_VELOCITY;
    if (control.work==MOTOR_WORK_SYNC_SPEED) return mode==JERRYFOC_MODE_VELOCITY;
    if (control.work==MOTOR_WORK_SYNC_POSITION || MotorControl_IsHandFollow()) return mode==JERRYFOC_MODE_POSITION;
    return !MotorControl_IsCoupled() || mode!=JERRYFOC_MODE_TORQUE;
}
__attribute__((noinline,noclone)) static void reset_m2(void)
{
    const MotorParams *m=MotorParams_GetAxis(MOTOR_AXIS_2);
    memset(&control.m2,0,sizeof(control.m2));
    JerryFOC_InitControllers(&control.m2.loops,m);
    control.m2.loops.d.output_ramp=control.m2.loops.q.output_ramp=0;
    control.m2.speed_filter.Tf=m->vel_filter_Tf;
    control.m2.position_speed=MOTOR_DEFAULT_POSITION_SPEED;
    control.m2.electrical_scale=(float)(m->pole_pairs*m->encoder_direction);
    control.m2.pending_align=!MotorSampling_IsGlobalStopped();
}
__attribute__((noinline,noclone)) static void stop_axis(MotorAxis axis)
{
    control.input[axis]=MOTOR_INPUT_NONE;
    if (axis==MOTOR_AXIS_1) { SixStep_Stop(); JerryFOC_stop(); return; }
    (void)MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,false);
    (void)MT6701_2_Reserve(false);
    control.m2.aligning=false; control.m2.pending_align=false;
    control.m2.target_q=control.m2.target_speed=control.m2.target_position=0.0f;
    control.m2.reference=control.m2.velocity;
    control.m2.mode=JERRYFOC_MODE_TORQUE;
    JerryFOC_ResetControllers(&control.m2.loops);
    JerryFOC_setAxisVoltage(MOTOR_AXIS_2,0,0,0,NULL,NULL);
}
void MotorControl_StopAll(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    control.group_active=false;
    stop_axis(MOTOR_AXIS_1); stop_axis(MOTOR_AXIS_2);
    __set_PRIMASK(mask);
}
void MotorControl_Stop(MotorAxis axis)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT) return;
    if (MotorControl_IsCoupled()) { MotorControl_StopAll(); return; }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    stop_axis(axis);
    __set_PRIMASK(mask);
}
__attribute__((noinline,noclone)) static bool stationary(void)
{
    if (MotorDetect_Active()) return false;
    if (MotorControl_SixStepAllowed()) return SixStep_AtRest() && !control.m2.aligning &&
        !MotorSampling_IsOutputRequested() && !MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2);
    if (MotorControl_HsoAllowed() && !StHso_AtRest()) return false;
    if (JerryFOC_isAligning() || control.m2.aligning || MotorSampling_IsOutputRequested() ||
        MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2)) return false;
    return fabsf(JerryFOC_getVelocity())<=1.0f && fabsf(control.m2.velocity)<=1.0f;
}
__attribute__((noinline,noclone)) static void save_config(void)
{
    AppSettings *s=AppStorage_Get();
    s->control_config=(s->control_config & (APP_CONFIG_ESC | APP_CONFIG_FOCUS_M2 | APP_CONFIG_IBUS_MASK | APP_CONFIG_THROTTLE_MASK)) | (uint32_t)control.work | (control.drive==MOTOR_DRIVE_HSO ? 8U : 0U) |
        (control.drive==MOTOR_DRIVE_SIXSTEP ? APP_CONFIG_SIXSTEP : 0U) |
        (MotorParams_BaseProfile(MotorParams_GetAxisProfile(MOTOR_AXIS_2))==MOTOR_PROFILE_4015 ? 16U : 0U);
    for(unsigned a=0;a<MOTOR_AXIS_COUNT;++a) {
        unsigned p=MotorParams_GetAxisProfile((MotorAxis)a);
        if(p>=MOTOR_PROFILE_CUSTOM1) s->control_config|=(p-MOTOR_PROFILE_CUSTOM1+1U)<<APP_CONFIG_CUSTOM_SHIFT(a);
    }
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_SaveCustom(unsigned slot,const MotorCalibration *record)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=slot<MOTOR_CUSTOM_COUNT && AppStorage_IsReady() && stationary() && !control.group_active &&
        MotorParams_GetAxisProfile(MOTOR_AXIS_1)!=slot+MOTOR_PROFILE_CUSTOM1 &&
        MotorParams_GetAxisProfile(MOTOR_AXIS_2)!=slot+MOTOR_PROFILE_CUSTOM1 && record && record->poles_rs;
    if(ok) ok=MotorParams_SetCustom(slot,record)!=0;
    if(ok) AppStorage_Get()->custom[slot]=*record;
    __set_PRIMASK(mask);return ok;
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_ApplyCalibration(MotorAxis axis,const MotorCalibration *record)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT || !record) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    MT6701_2_Snapshot enc;
    if(axis==MOTOR_AXIS_2) MT6701_2_GetSnapshot(&enc); else enc.healthy=MT6701_IsHealthy();
    bool ok=stationary() && !control.group_active && enc.healthy &&
        MotorControl_GetDrive(axis)==MOTOR_DRIVE_ENCODER &&
        MotorParams_GetAxisProfile(axis)<MOTOR_PROFILE_ESC &&
        (!record->poles_rs || record->poles_rs>>24==(unsigned)MotorParams_GetAxisProfile(axis)+1U);
    if(ok) ok=MotorParams_SetCalibration(axis,record)!=0;
    if(ok) {
        stop_axis(axis);
        if(axis==MOTOR_AXIS_1) JerryFOC_ReloadMotor(); else reset_m2();
        AppStorage_Get()->calibration[axis]=*record;
    }
    __set_PRIMASK(mask); return ok;
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_SelectWork(MotorWorkMode mode)
{
    if ((unsigned)mode>=MOTOR_WORK_COUNT) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!stationary()) { __set_PRIMASK(mask); return false; }
    MotorControl_StopAll();
    control.work=mode;
    (void)MotorSampling_SelectSixStep(MotorControl_SixStepAllowed());
    SixStep_SetGeneric(MotorControl_SixStepAllowed() && (AppStorage_Get()->control_config & APP_CONFIG_ESC));
    control.group_stop_reason=NULL;
    control.focus=work_focus();
    if (!MotorControl_HsoAllowed()) { (void)JerryFOC_useHSO(0); StHso_Enable(false); }
    else StHso_Enable(true);
    MT6701_2_SetRealtime(MotorControl_AxisEnabled(MOTOR_AXIS_2));
    control.m2.pending_align=MotorControl_AxisEnabled(MOTOR_AXIS_2) && !control.m2.aligned && !MotorSampling_IsGlobalStopped();
    save_config();
    __set_PRIMASK(mask);
    return true;
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_SelectFeedback(bool hso)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (control.work!=MOTOR_WORK_SINGLE_M1 || !stationary()) { __set_PRIMASK(mask); return false; }
    control.drive=hso ? MOTOR_DRIVE_HSO : MOTOR_DRIVE_ENCODER;
    SixStep_SetGeneric(false);
    SixStep_Stop(); (void)MotorSampling_SelectSixStep(false);
    (void)JerryFOC_useHSO(0); StHso_Enable(hso);
    control.m1_stop_reason=control.group_stop_reason=NULL;
    if (!hso) JerryFOC_requestAlignment();
    save_config(); __set_PRIMASK(mask); return true;
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_SelectSixStep(bool enable)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (control.work!=MOTOR_WORK_SINGLE_M1 || !stationary()) { __set_PRIMASK(mask); return false; }
    MotorControl_StopAll();
    if (!MotorSampling_SelectSixStep(enable)) { __set_PRIMASK(mask); return false; }
    control.drive=enable ? MOTOR_DRIVE_SIXSTEP : MOTOR_DRIVE_ENCODER;
    SixStep_SetGeneric(enable && (AppStorage_Get()->control_config & APP_CONFIG_ESC));
    (void)JerryFOC_useHSO(0U); StHso_Enable(false);
    if (!enable) JerryFOC_requestAlignment();
    control.m1_stop_reason=control.group_stop_reason=NULL;
    save_config(); __set_PRIMASK(mask); return true;
}
__attribute__((noinline,noclone)) static void clear_saved_axis(MotorAxis axis)
{
    AppSettings *s=AppStorage_Get();
    unsigned position=(unsigned)MOTOR_DEFAULT_POSITION_SPEED;
    if(position<1U) position=1U;
    if(position>APP_SPEED_LIMIT_DEFAULT) position=APP_SPEED_LIMIT_DEFAULT;
    uint32_t defaults=(s->flags & APP_FLAG_LED_ENABLE)|AppSettings_EncodePositionSpeed(position)|
        ((uint32_t)APP_SPEED_LIMIT_DEFAULT<<APP_SPEED_LIMIT_SHIFT)|
        (MotorParams_BaseProfile(MotorParams_GetAxisProfile(axis))==MOTOR_PROFILE_4015 ? APP_FLAG_MOTOR_4015 : 0U);
    if (axis==MOTOR_AXIS_1) {
        s->speed_x10=s->voltage_x100=s->position_x100=0;
        s->control_config&=~APP_CONFIG_THROTTLE_MASK;
        s->flags=defaults;
    } else {
        s->axis2_speed_x10=s->axis2_voltage_x100=s->axis2_position_x100=0;
        s->axis2_flags=defaults;
    }
    /* Profile/generic selection has already certified both axes stopped. */
    control.axis_flags[axis]=axis==MOTOR_AXIS_1 ? s->flags : s->axis2_flags;
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_SelectProfile(MotorAxis axis,MotorProfile profile)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT || !MotorParams_GetProfile(profile)) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!stationary()) { __set_PRIMASK(mask); return false; }
    bool ok=true;
    if (profile!=MotorParams_GetAxisProfile(axis) ||
        (axis==MOTOR_AXIS_1 && (AppStorage_Get()->control_config & APP_CONFIG_ESC))) {
        if (axis==MOTOR_AXIS_1) {
            ok=JerryFOC_selectMotor(profile)!=0U;
            if (ok) {
                AppStorage_Get()->control_config&=~APP_CONFIG_ESC;
                SixStep_SetGeneric(false);
                StHso_Enable(MotorControl_HsoAllowed());
            }
        } else {
            stop_axis(axis); (void)MotorParams_SelectAxis(axis,profile);
            reset_m2(); save_config();
        }
        if (ok) { clear_saved_axis(axis);save_config(); }
    }
    __set_PRIMASK(mask); return ok;
}
bool MotorControl_SelectGeneric(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool ok=MotorControl_SixStepAllowed() && stationary();
    if(ok) {
        AppStorage_Get()->control_config|=APP_CONFIG_ESC;
        SixStep_SetGeneric(true); clear_saved_axis(MOTOR_AXIS_1);
    }
    __set_PRIMASK(mask); return ok;
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_SetGear(int32_t r,int32_t o)
{
    if (r==0 || r< -1000 || r>1000 || o< -6283 || o>6283) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!stationary()) { __set_PRIMASK(mask); return false; }
    AppStorage_Get()->gear_ratio_x100=r; AppStorage_Get()->gear_offset_x100=o;
    control.gear_ratio=(float)r/100.0f; control.gear_offset=(float)o/100.0f;
    __set_PRIMASK(mask); return true;
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_RequestAlign(MotorAxis axis)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if ((axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed()) ||
        !MotorControl_AxisEnabled(axis) || !resume_global_stop() || !stationary() ||
        ((axis==MOTOR_AXIS_1 || MotorControl_IsCoupled()) && MotorSampling_IsIbusTripped())) {
        __set_PRIMASK(mask); return false;
    }
    if (MotorControl_IsPassive(axis)) {
        if (axis==MOTOR_AXIS_2) MT6701_2_Retry();
        __set_PRIMASK(mask); return true; /* Encoder retry only; never align the hand shaft. */
    }
    if (axis==MOTOR_AXIS_1) { JerryFOC_requestAlignment(); control.m1_stop_reason=NULL; }
    else { MT6701_2_Retry(); control.m2.pending_align=true; control.m2.aligned=false; control.m2.stop_reason=NULL; }
    __set_PRIMASK(mask); return true;
}
__attribute__((noinline,noclone,optimize("Oz"))) void MotorControl_Init(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    AppSettings *s=AppStorage_Get();
    control.work=(MotorWorkMode)(s->control_config & 7U);
    if (control.work>=MOTOR_WORK_COUNT) control.work=MOTOR_WORK_SINGLE_M1;
    control.drive=(s->control_config & APP_CONFIG_SIXSTEP) ? MOTOR_DRIVE_SIXSTEP :
        (s->control_config & 8U) ? MOTOR_DRIVE_HSO : MOTOR_DRIVE_ENCODER;
    control.focus=work_focus();
    MotorCalibration empty={0};
    for(unsigned i=0;i<MOTOR_CUSTOM_COUNT;++i) {
        if(!MotorParams_SetCustom(i,&s->custom[i])) (void)MotorParams_SetCustom(i,&empty);
    }
    for(unsigned a=0;a<MOTOR_AXIS_COUNT;++a) {
        if(!MotorParams_SetCalibration((MotorAxis)a,&s->calibration[a]))
            (void)MotorParams_SetCalibration((MotorAxis)a,&empty);
    }
    for(unsigned a=0;a<MOTOR_AXIS_COUNT;++a) {
        unsigned n=(s->control_config>>APP_CONFIG_CUSTOM_SHIFT(a))&3U;
        MotorProfile preset=(a ? s->control_config&16U : s->flags&APP_FLAG_MOTOR_4015) ? MOTOR_PROFILE_4015 : MOTOR_PROFILE_2804;
        if(!n || !MotorParams_SelectAxis((MotorAxis)a,(MotorProfile)(n+MOTOR_PROFILE_CUSTOM1-1U)))
            (void)MotorParams_SelectAxis((MotorAxis)a,preset);
    }
    JerryFOC_ReloadMotor();
    reset_m2();
    (void)MotorSampling_SelectSixStep(MotorControl_SixStepAllowed());
    SixStep_SetGeneric(MotorControl_SixStepAllowed() && (s->control_config & APP_CONFIG_ESC));
    if (!s->gear_ratio_x100 || s->gear_ratio_x100< -1000 || s->gear_ratio_x100>1000) s->gear_ratio_x100=100;
    if (s->gear_offset_x100< -6283 || s->gear_offset_x100>6283) s->gear_offset_x100=0;
    control.axis_flags[0]=s->flags; control.axis_flags[1]=s->axis2_flags;
    control.gear_ratio=(float)s->gear_ratio_x100/100.0f;
    control.gear_offset=(float)s->gear_offset_x100/100.0f;
    control.group_active=false; /* Deadline fault is reset only by MCU reset/BSS. */
    control.input[0]=control.input[1]=MOTOR_INPUT_NONE;
    control.m1_stop_reason=control.group_stop_reason=NULL;
    MT6701_2_SetRealtime(MotorControl_AxisEnabled(MOTOR_AXIS_2));
    StHso_Enable(MotorControl_HsoAllowed());
    save_config();
    __set_PRIMASK(mask);
}
__attribute__((noinline,noclone)) void MotorControl_GetAxis(MotorAxis axis,MotorAxisSnapshot *out)
{
    if (!out || (unsigned)axis>=MOTOR_AXIS_COUNT) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    const char *sampling=MotorSampling_GetAxisStatusText(axis);
    if (axis==MOTOR_AXIS_1) {
        extern volatile uint8_t flag_use_current_loop;

        out->mode=JerryFOC_getMode();
        out->velocity=JerryFOC_getVelocity();
        out->position=JerryFOC_getAngle();
        out->electrical_angle=JerryFOC_getElectricalAngle();
        out->iq=JerryFOC_getIq();
        out->id=JerryFOC_getId();
        out->ia=JerryFOC_getPhaseCurrent_A();
        out->ib=JerryFOC_getPhaseCurrent_B();
        out->target_velocity=JerryFOC_getTargetVelocity();
        out->target_position=JerryFOC_getTargetPosition();
        out->target_iq=JerryFOC_Target_Iq;
        out->healthy=MT6701_IsHealthy()!=0;
        out->aligned=AngleSrc_Encoder_IsAligned()!=0;
        out->active=MotorSampling_IsOutputRequested();
        out->aligning=JerryFOC_isAligning()!=0;
        out->current_loop=flag_use_current_loop!=0;
        out->ready=JerryFOC_isDriveReady()!=0;
        out->status=sampling;
        if (JerryFOC_hasHSOFault() && MotorSampling_IsAxisReady(axis)) out->status="HSO STOPPED";
        else if (control.m1_stop_reason && out->ready) out->status=control.m1_stop_reason;
        if (MotorControl_HsoAllowed()) {
            out->healthy=!JerryFOC_hasHSOFault();
            out->aligned=true; /* Sensorless alignment belongs to its startup FSM. */
        }
    } else {
        MT6701_2_Snapshot enc; MT6701_2_GetSnapshot(&enc);

        out->mode=control.m2.mode;
        out->velocity=control.m2.velocity;
        out->position=control.m2.encoder.position;
        out->electrical_angle=norm(control.m2.electrical_scale*enc.angle-control.m2.electrical_zero);
        out->iq=control.m2.iq;
        out->id=control.m2.id;
        out->ia=control.m2.ia;
        out->ib=control.m2.ib;
        out->target_velocity=control.m2.target_speed;
        out->target_position=control.m2.target_position;
        out->target_iq=control.m2.target_q;
        out->healthy=enc.healthy!=0;
        out->aligned=control.m2.aligned;
        out->active=MotorSampling_IsAxisOutputRequested(axis);
        out->aligning=control.m2.aligning;
        out->current_loop=control.m2.loop;
        out->ready=MotorSampling_IsAxisReady(axis);
        out->status=sampling;
        if (control.m2.stop_reason && out->ready) out->status=control.m2.stop_reason;
    }
    if (MotorControl_IsPassive(axis)) {
        out->ready=out->healthy;
        out->status=out->healthy ? "HAND MASTER" : MotorControl_EncoderLostText(axis);
    }
    if (out->ready && !out->healthy) out->status=MotorControl_EncoderLostText(axis);
    if (control.group_stop_reason && MotorControl_IsCoupled() && !out->active &&
        (out->ready || MotorControl_IsPassive(axis))) out->status=control.group_stop_reason;
    if (control.timing_fault) { out->ready=false; out->status="CONTROL OVERRUN"; }
    if (axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed()) {
        SixStepSnapshot ss; SixStep_GetSnapshot(&ss);
        out->mode=ss.speed_control ? JERRYFOC_MODE_VELOCITY : JERRYFOC_MODE_TORQUE; out->velocity=ss.velocity;
        out->target_velocity=ss.speed_control ? ss.target : 0;
        out->target_iq=ss.speed_control ? 0 : ss.target; out->position=out->electrical_angle=0;
        const MotorSampleFrame *frame=MotorSampling_GetFrame();
        out->ia=frame->m1_current[0]; out->ib=frame->m1_current[1]; out->iq=out->id=0;
        out->healthy=out->aligned=true; out->current_loop=false;
        out->aligning=false;
        out->ready=MotorSampling_IsReady() && !control.timing_fault;
        if (out->ready) out->status=ss.status;
    }
    if (!MotorControl_AxisEnabled(axis)) { out->ready=false; out->status="AXIS DISABLED"; }
    if (MotorSampling_IsGlobalStopped()) out->status="GLOBAL STOP";
    __set_PRIMASK(mask);
}
static bool can_start(MotorAxis axis)
{
    if (MotorControl_IsCoupled() && MotorSampling_IsIbusTripped()) {
        command_text="M1 IBUS TRIP"; return false;
    }
    MotorAxisSnapshot s; MotorControl_GetAxis(axis,&s);
    if (axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed()) {
        if (!MotorSampling_IsReady() || !MotorSampling_IsCalibrated()) {
            command_text=MotorSampling_GetAxisStatusText(axis); return false;
        }
        return !control.timing_fault;
    }
    if (!MotorControl_AxisEnabled(axis)) command_text="AXIS DISABLED";
    else if (!s.ready && !(axis==MOTOR_AXIS_1 && JerryFOC_hasHSOFault() &&
             MotorSampling_IsAxisReady(axis))) command_text=s.status;
    else if (!s.healthy) command_text=MotorControl_EncoderLostText(axis);
    else if (axis==MOTOR_AXIS_2 && control.m2.encoder.count<ENCODER_VELOCITY_WINDOW) command_text="ENCODER SETTLE";
    else if (!isfinite(s.position) || !isfinite(s.velocity)) command_text="ENCODER VALUE";
    else if (MotorControl_IsPassive(axis)) {
        if (!s.active && !s.aligning) return true;
        command_text="MASTER NOT FREE";
    }
    else if (!s.aligned || s.aligning) command_text="NOT ALIGNED";
    else if (!s.active && (!isfinite(s.velocity) || fabsf(s.velocity)>1.0f)) command_text="Wait for stop";
    else if (control.timing_fault) command_text="CONTROL OVERRUN";
    else return true;
    return false;
}
__attribute__((noinline,noclone)) static void set_mode(MotorAxis axis,JerryFOC_ControlMode mode,bool loop)
{
    if (axis==MOTOR_AXIS_1) {
        g_openloop_debug=0U; JerryFOC_useCurrentLoop(loop); JerryFOC_setMode(mode); return;
    }
    if (control.m2.loop!=loop || control.m2.mode!=mode) {
        float hold=control.m2.loop==loop && MotorSampling_IsAxisOutputRequested(axis) ? control.m2.target_q : 0.0f;
        JerryFOC_ResetControllers(&control.m2.loops);
        control.m2.reference=control.m2.velocity;
        control.m2.loops.vel.integral_prev=control.m2.loops.vel.output_prev=hold;
        control.m2.target_q=hold;
    }
    control.m2.loop=loop; control.m2.mode=mode;
    const MotorParams *m=MotorParams_GetAxis(axis);
    control.m2.loops.vel.limit=loop ? m->rated_current : m->vel_limit;
    control.m2.loops.vel.P=loop ? m->vel_current_P : m->vel_P;
    control.m2.loops.vel.I=loop ? m->vel_current_I : m->vel_I;
}
__attribute__((noinline,noclone)) static void set_target(MotorAxis axis,JerryFOC_ControlMode mode,float value,float limit)
{
    if (axis==MOTOR_AXIS_1) {
        if (mode==JERRYFOC_MODE_VELOCITY) JerryFOC_setVelocity(value);
        else if (mode==JERRYFOC_MODE_POSITION) JerryFOC_setAbsolutePosition(value,
            fminf(limit>0 ? limit : position_speed(axis),position_speed(axis)));
        else JerryFOC_setCurrent(value);
    } else {
        const MotorParams *m=MotorParams_GetAxis(axis);
        if (mode==JERRYFOC_MODE_VELOCITY) control.m2.target_speed=clamp(value,-speed_limit(axis),speed_limit(axis));
        else if (mode==JERRYFOC_MODE_POSITION) {
            control.m2.target_position=value;
            control.m2.position_speed=clamp(limit>0 ? limit : position_speed(axis),0.1f,position_speed(axis));
        } else {
            float max=control.m2.loop ? m->rated_current : m->vel_limit;
            control.m2.target_q=clamp(value,-max,max);
        }
    }
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_StartFollow(void)
{
    if (!MotorControl_IsHandFollow()) { command_text="SELECT MASTER"; return false; }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!resume_global_stop()) { __set_PRIMASK(mask); return false; }
    if (control.group_active) { __set_PRIMASK(mask); return true; } /* Do not re-zero an active session. */
    MotorAxis master=control.work==MOTOR_WORK_MASTER_M2 ? MOTOR_AXIS_2 : MOTOR_AXIS_1;
    MotorAxis slave=(MotorAxis)(1U-master);
    if (!can_start(master) || !can_start(slave)) { __set_PRIMASK(mask); return false; }
    stop_axis(master);
    if (!MT6701_2_Reserve(true)) { command_text="SPI2 BUSY"; __set_PRIMASK(mask); return false; }
    control.origins[0]=JerryFOC_getAngle(); control.origins[1]=control.m2.encoder.position;
    control.group_reference=control.origins[slave];
    control.group_speed=slave==MOTOR_AXIS_1 ? JerryFOC_getVelocity() : control.m2.velocity;
    uint32_t flags=MotorControl_GetOptions(slave);
    float limit=position_speed(slave);
    set_mode(slave,JERRYFOC_MODE_POSITION,(flags & APP_FLAG_CURRENT_LOOP)!=0U);
    const MotorParams *m=MotorParams_GetAxis(slave);
    control.group_follow=(flags & APP_FLAG_CURRENT_LOOP) ? m->follow_current : m->follow_voltage;
    for (unsigned i=0;i<2;++i) {
        control.follow_estimate[i]=control.origins[i];
        control.follow_velocity[i]=i==MOTOR_AXIS_1 ? JerryFOC_getVelocity() : control.m2.velocity;
    }
    control.follow_weight=0;
    control.follow_activity=1; control.follow_acceleration=0;
    control.follow_gain=m->vel_current_P;
    control.follow_motion_anchor=control.origins[master];
    control.follow_quiet_ms=0; control.follow_count_interval=1;
    control.follow_quiet=control.follow_hold=false;
    control.follow_command_position=control.origins[master];
    control.follow_command_velocity=control.follow_master_velocity=control.follow_velocity[master];
    set_target(slave,JERRYFOC_MODE_POSITION,control.origins[slave],limit);
    control.group_mode=JERRYFOC_MODE_POSITION; control.group_error=0; control.error_ms=0; control.group_started=HAL_GetTick();
    control.group_stop_reason=NULL;
    control.group_active=true;
    bool ok=slave==MOTOR_AXIS_1 ? JerryFOC_startOutput()!=0U : MotorSampling_SetAxisOutputEnabled(slave,true);
    if (!ok) MotorControl_StopAll();
    else { set_input(slave,MOTOR_INPUT_FOLLOW); control.m1_stop_reason=NULL; control.m2.stop_reason=NULL; }
    command_text=ok ? "READY" : "START REJECTED";
    __set_PRIMASK(mask); return ok;
}
bool MotorControl_IsFastFollow(MotorAxis axis)
{
    return (unsigned)axis<MOTOR_AXIS_COUNT && control.group_active && MotorControl_IsHandFollow() &&
        !MotorControl_IsPassive(axis) && control.group_follow->fast_tracking;
}
float MotorControl_VelocityControl(MotorAxis axis,PIDController *velocity,float reference,float measured)
{
    bool fast=MotorControl_IsFastFollow(axis);
    float error=reference-(fast ? control.follow_velocity[axis] : measured);
    float integral=velocity->integral_prev;
    float gain=velocity->P;
    float igain=velocity->I;
    /* Retain the R35 gain range. Smooth its error-driven scheduling over
     * about 4 ms instead of modulating P directly with each encoder sample. */
    if (fast) {
        control.follow_gain+=0.2f*(gain+(control.group_follow->velocity_P-gain)*
            clamp(fabsf(error),0,1)-control.follow_gain);
        velocity->P=control.follow_gain;
        /* Hysteresis in follower radians: enter within four encoder counts,
         * leave beyond eight. Preserve the holding integral/load torque;
         * suppress tiny speed-noise corrections instead of disabling PWM. */
        if (!control.follow_quiet || fabsf(control.group_error)>0.003068f || fabsf(reference)>0.15f)
            control.follow_hold=false;
        else if (control.follow_quiet_ms>60 && fabsf(control.group_error)<0.001534f &&
                 fabsf(reference)<0.05f && fabsf(control.follow_velocity[axis])<0.08f)
            control.follow_hold=true;
        if (control.follow_hold) {
            velocity->I=0;
            error=copysignf(fmaxf(0,fabsf(error)-0.05f),error);
        }
    }
    float output=JerryFOC_PID_Calc(velocity,error);
    velocity->P=gain; velocity->I=igain;
    /* Every encoder speed PI must reject further winding into saturation,
     * including ordinary Position and voltage mode. Retain R113 HSO PI. */
    if (!(axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed()) &&
        output*error>0 && fabsf(output)>=velocity->limit)
        velocity->integral_prev=integral;
    return output;
}
float MotorControl_SpeedReference(MotorAxis axis,float previous,float target)
{
    float weight=MotorControl_IsFastFollow(axis) ? control.follow_weight : 0;
    return slew(previous,MotorControl_LimitSpeed(axis,target),100+200*weight,150+450*weight);
}
float MotorControl_PositionVelocity(MotorAxis axis,PIDController *position,float error)
{
    if (!control.group_active || !MotorControl_IsHandFollow() || MotorControl_IsPassive(axis))
    {
        position->limit=fminf(position->limit,position_speed(axis));
        float feedforward=control.group_active && control.work==MOTOR_WORK_SYNC_POSITION ?
            control.group_speed*(axis==MOTOR_AXIS_1 ? 1.0f : ratio()) : 0.0f;
        return feedforward+JerryFOC_PID_Calc(position,error);
    }
    const MotorFollowParams *m=control.group_follow;
    float velocity=axis==MOTOR_AXIS_1 ? JerryFOC_getVelocity() : control.m2.velocity;
    if (m->fast_tracking) velocity=control.follow_velocity[axis];
    /* Feed the velocity loop a planned speed, with a small position correction
     * and relative-speed damping. No position integral or step derivative. */
    /* Keep the planned position/speed pair through braking and settling.
     * Do not switch to a second raw-position PD controller at the endpoint. */
    /* Bounded 4 ms reference lead from the existing 10 Hz acceleration
     * estimate. Fast motion only, in follower mechanical units; never raw
     * angle differentiation or an unmeasured inertia/current feedforward. */
    float lead=m->fast_tracking && !control.follow_quiet ?
        clamp(0.004f*ratio()*control.follow_acceleration,-0.5f,0.5f)*
        control.follow_weight*clamp((control.follow_activity-0.3f)/0.4f,0,1) : 0;
    return control.group_speed+lead+m->P*error+m->damping*(control.group_speed-velocity);
}
__attribute__((noinline,noclone,optimize("Oz"))) bool MotorControl_CommandFrom(MotorAxis axis,JerryFOC_ControlMode mode,float value,float limit,bool loop,MotorInput input)
{
    if (MotorDetect_Active()) { command_text="M1 DETECT BUSY"; return false; }
    if (MotorControl_IsHandFollow()) { command_text="Use Follow"; return false; }
    if ((unsigned)axis>=MOTOR_AXIS_COUNT || !isfinite(value) || !isfinite(limit) || !MotorControl_ModeAllowed(mode)) {
        command_text="MODE / VALUE"; return false;
    }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!input_available(axis,input)) { __set_PRIMASK(mask); return false; }
    if (axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed() && value==0.0f) {
        MotorControl_Stop(axis); command_text="READY"; __set_PRIMASK(mask); return true;
    }
    if (axis==MOTOR_AXIS_1 && MotorControl_SixStepAllowed()) {
        if (mode==JERRYFOC_MODE_VELOCITY && fabsf(value)>speed_limit(axis)) value=copysignf(speed_limit(axis),value);
        bool ok=(value==0 || (!control.timing_fault && resume_global_stop())) &&
            (mode==JERRYFOC_MODE_VELOCITY ? SixStep_Start(value) : SixStep_Throttle(value));
        command_text=ok ? "READY" : "6STEP RANGE/STOP";
        if (ok) set_input(axis,input);
        __set_PRIMASK(mask); return ok;
    }
    if (!resume_global_stop()) { __set_PRIMASK(mask); return false; }
    if (axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed()) {
        value=clamp(value,-speed_limit(axis),speed_limit(axis));
        if (!StHso_ValidTarget(value)) {
            command_text="HSO SPEED/DIR"; __set_PRIMASK(mask); return false;
        }
    }
    bool coupled=MotorControl_IsCoupled();
    if (coupled && control.group_active && control.group_mode!=mode) {
        command_text="Stop before mode"; __set_PRIMASK(mask); return false;
    }
    if (!can_start(axis) || (coupled && (!can_start(MOTOR_AXIS_1) || !can_start(MOTOR_AXIS_2)))) {
        __set_PRIMASK(mask); return false;
    }
    if ((axis==MOTOR_AXIS_2 || coupled) && !MT6701_2_Reserve(true)) {
        command_text="SPI2 BUSY"; __set_PRIMASK(mask); return false;
    }
    if (coupled) {
        if (!control.group_active || control.group_mode!=mode) {
            MotorAxisSnapshot a,b; MotorControl_GetAxis(MOTOR_AXIS_1,&a); MotorControl_GetAxis(MOTOR_AXIS_2,&b);
            control.origins[0]=a.position; control.origins[1]=b.position;
            control.group_reference=0.0f; control.group_speed=0.0f; control.error_ms=0U; control.group_started=HAL_GetTick();
        }
        control.group_stop_reason=NULL;
        control.group_mode=mode; control.group_target=value;
        control.group_position_limit=limit>0.0f ? limit : position_speed(MOTOR_AXIS_1);
        if (mode==JERRYFOC_MODE_VELOCITY) {
            float cap=fminf(speed_limit(MOTOR_AXIS_1),speed_limit(MOTOR_AXIS_2)/fabsf(ratio()));
            control.group_target=clamp(value,-cap,cap);
        }
        bool loop1=(MotorControl_GetOptions(MOTOR_AXIS_1) & APP_FLAG_CURRENT_LOOP)!=0U;
        bool loop2=(MotorControl_GetOptions(MOTOR_AXIS_2) & APP_FLAG_CURRENT_LOOP)!=0U;
        set_mode(MOTOR_AXIS_1,mode,loop1); set_mode(MOTOR_AXIS_2,mode,loop2);
        /* Neutral targets before the two-frame output arming delay. */
        if (!control.group_active) {
            set_target(MOTOR_AXIS_1,mode,mode==JERRYFOC_MODE_POSITION ? control.origins[0] : 0,limit);
            set_target(MOTOR_AXIS_2,mode,mode==JERRYFOC_MODE_POSITION ? control.origins[1] : 0,limit);
        }
        control.group_active=true;
        bool ok=JerryFOC_startOutput() && MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true);
        if (!ok) { MotorControl_StopAll(); command_text="START REJECTED"; }
        else { set_input(axis,input); command_text="READY"; control.m1_stop_reason=control.m2.stop_reason=NULL; }
        __set_PRIMASK(mask); return ok;
    }
    set_mode(axis,mode,loop);
    if (mode==JERRYFOC_MODE_VELOCITY) value=clamp(value,-speed_limit(axis),speed_limit(axis));
    set_target(axis,mode,value,limit);
    bool ok=axis==MOTOR_AXIS_1 ? JerryFOC_startOutput()!=0U : MotorSampling_SetAxisOutputEnabled(axis,true);
    if (!ok && axis==MOTOR_AXIS_2) { stop_axis(axis); command_text="START REJECTED"; }
    else {
        command_text=ok ? "READY" : MotorControl_HsoAllowed() ? "HSO START GATE" : "START REJECTED";
        if (ok) { if (axis==MOTOR_AXIS_2) control.m2.stop_reason=NULL; else control.m1_stop_reason=NULL; }
    }
    if (ok) set_input(axis,input);
    __set_PRIMASK(mask); return ok;
}
void MotorControl_Service(uint32_t now)
{
    MotorDetect_Service(now);
    if (MotorSampling_TakeGlobalStop()) {
        MotorControl_StopAll();
        command_text=control.m1_stop_reason=control.m2.stop_reason=control.group_stop_reason="GLOBAL STOP";
    }
    if (MotorSampling_IsGlobalStopped()) return;
    if (!MotorControl_MotionAllowed(MOTOR_AXIS_2)) return;
    MT6701_2_Snapshot enc; MT6701_2_GetSnapshot(&enc);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool m1_align_done=!MotorControl_MotionAllowed(MOTOR_AXIS_1) ||
                       (!JerryFOC_isAligning() && !JerryFOC_needsAlignment());
    if (m1_align_done && control.m2.pending_align && !control.m2.aligning &&
        (!MotorControl_IsCoupled() || !MotorSampling_IsIbusTripped()) && MotorSampling_IsAxisReady(MOTOR_AXIS_2) &&
        enc.healthy && fabsf(control.m2.velocity)<=1.0f && MT6701_2_Reserve(true)) {
        control.m2.pending_align=false; control.m2.aligning=true; control.m2.aligned=false; control.m2.align_at=now;
        if (!MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true)) stop_axis(MOTOR_AXIS_2);
    }
    if (control.m2.aligning && (now-control.m2.align_at>=3000U || !enc.healthy || !MotorSampling_IsAxisReady(MOTOR_AXIS_2))) {
        bool ok=enc.healthy && MotorSampling_IsAxisReady(MOTOR_AXIS_2) && now-control.m2.align_at>=3000U;
        if (ok) {
            control.m2.electrical_zero=norm(control.m2.electrical_scale*enc.angle);
            EncoderEstimator_Reset(&control.m2.encoder,enc.angle,enc.samples,true);
            control.m2.velocity=0.0f;
            control.m2.speed_filter.y_prev=0.0f; control.m2.aligned=true; control.m2.stop_reason=NULL;
        }
        stop_axis(MOTOR_AXIS_2);
    }
    __set_PRIMASK(mask);
}
void MotorControl_Fast(void)
{
    if (MotorControl_SixStepAllowed()) { SixStep_Process(MotorSampling_GetFrame()); return; }
    if (MotorSampling_IsGlobalStopped()) return; /* Main owns controller/SPI cleanup. */
    if (MotorControl_IsCoupled() &&
        (control.group_active || control.m2.aligning || MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2)) &&
        MotorSampling_IsIbusTripped()) {
        command_text=control.group_stop_reason="M1 IBUS TRIP";
        MotorControl_StopAll(); return;
    }
    if (MotorControl_AxisEnabled(MOTOR_AXIS_1) && MotorSampling_IsOutputRequested() &&
        !g_openloop_debug && !MotorControl_HsoAllowed() &&
        (!MT6701_IsHealthy() || !MotorSampling_IsAxisReady(MOTOR_AXIS_1))) {
        control.m1_stop_reason=MT6701_IsHealthy() ? MotorSampling_GetAxisStatusText(MOTOR_AXIS_1) : MotorControl_EncoderLostText(MOTOR_AXIS_1);
        command_text=control.group_stop_reason=control.m1_stop_reason;
        (void)MotorSampling_SetOutputEnabled(false);
        BoardDiag_Fault(1U,DIAG_ENCODER,control.m1_stop_reason);
        MotorControl_Stop(MOTOR_AXIS_1);
    }
    if (!MotorControl_AxisEnabled(MOTOR_AXIS_2)) return;
    MT6701_2_Snapshot enc; MT6701_2_GetSnapshot(&enc);
    bool active=MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2);
    /* A failed startup probe must not auto-align/energize after reconnection. */
    if (enc.stopped) control.m2.pending_align=false;
    if (active && (!enc.healthy || !MotorSampling_IsAxisReady(MOTOR_AXIS_2))) {
        control.m2.stop_reason=enc.healthy ? MotorSampling_GetAxisStatusText(MOTOR_AXIS_2) : MotorControl_EncoderLostText(MOTOR_AXIS_2);
        if (!enc.healthy) control.m2.aligned=false;
        command_text=control.group_stop_reason=control.m2.stop_reason;
        (void)MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,false);
        BoardDiag_Fault(2U,DIAG_ENCODER,control.m2.stop_reason);
        MotorControl_Stop(MOTOR_AXIS_2); return;
    }
    if (control.group_active && (!MT6701_IsHealthy() || !enc.healthy ||
        MotorSampling_IsOutputRequested()==MotorControl_IsPassive(MOTOR_AXIS_1) ||
        active==MotorControl_IsPassive(MOTOR_AXIS_2) ||
        (!MotorControl_IsPassive(MOTOR_AXIS_1) && !JerryFOC_isDriveReady()))) {
        command_text=control.group_stop_reason=!MT6701_IsHealthy() ? MotorControl_EncoderLostText(MOTOR_AXIS_1) :
            !enc.healthy ? MotorControl_EncoderLostText(MOTOR_AXIS_2) : "GROUP OUTPUT OFF";
        (void)MotorSampling_SetOutputEnabled(false);
        (void)MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,false);
        BoardDiag_Fault(!MT6701_IsHealthy() ? 1U : !enc.healthy ? 2U : 0U,DIAG_CONTROL,command_text);
        MotorControl_StopAll(); return;
    }
    const MotorSampleFrame *f=MotorSampling_GetFrame();
    control.m2.ia=f->m2_current[0]; control.m2.ib=f->m2_current[1];
    float angle=norm(control.m2.electrical_scale*enc.angle-control.m2.electrical_zero);
    float sensor;uint32_t at;
    if (f->timing_valid && MT6701_GetTimedAngle(1,&sensor,&at)) {
        angle=MotorMath_PredictAngle(control.m2.electrical_scale*sensor-control.m2.electrical_zero,
            (float)MotorParams_GetAxis(MOTOR_AXIS_2)->pole_pairs*control.m2.velocity,
            (int32_t)(f->current_cycles[1]-at));
        angle=norm(angle);
    }
    float alpha,beta,d,q;
    JerryFOC_Clarke(control.m2.ia,control.m2.ib,&alpha,&beta); JerryFOC_Park(alpha,beta,angle,&d,&q);
    control.m2.id=0.5f*(control.m2.id+d); control.m2.iq=0.5f*(control.m2.iq+q);
    if (!active) return;
    if (control.m2.aligning) {
        const MotorParams *m=MotorParams_GetAxis(MOTOR_AXIS_2);
        JerryFOC_setAxisVoltage(MOTOR_AXIS_2,0,fminf(3.0f,m->Rs*m->rated_current),0,NULL,NULL);
        return;
    }
    if (!control.m2.aligned) { MotorControl_Stop(MOTOR_AXIS_2); return; }
    if (control.m2.loop) {
        float cap=MotorParams_GetAxis(MOTOR_AXIS_2)->rated_current;
        JerryFOC_CurrentControl(MOTOR_AXIS_2,&control.m2.loops,angle,control.m2.velocity,
            0.0f,clamp(control.m2.target_q,-cap,cap),control.m2.id,control.m2.iq);
    } else JerryFOC_setEncoderVoltage(MOTOR_AXIS_2,control.m2.target_q,0,angle,control.m2.velocity,NULL,NULL);
}
static void update_encoder2(void)
{
    MT6701_2_Snapshot enc; MT6701_2_GetSnapshot(&enc);
    bool accepted=EncoderEstimator_Update(&control.m2.encoder,enc.angle,enc.samples,
        enc.healthy,MotorParams_GetAxis(MOTOR_AXIS_2)->encoder_direction,0.001f);
    if (!control.m2.encoder.synced || !control.m2.encoder.count) {
        control.m2.velocity=control.m2.speed_filter.y_prev=0.0f;
    } else if (accepted) {
        control.m2.velocity=JerryFOC_LPF_Calc(&control.m2.speed_filter,control.m2.encoder.velocity);
    }
}
static void coordinate(void)
{
    if (!control.group_active) return;
    float positions[2]={JerryFOC_getAngle(),control.m2.encoder.position};
    float velocities[2]={JerryFOC_getVelocity(),control.m2.velocity};
    unsigned master=control.work==MOTOR_WORK_MASTER_M2 ? 1U : 0U, slave=1U-master;
    float k=ratio();
    if (MotorControl_IsHandFollow()) {
        float target=control.origins[slave]+offset()+k*(positions[master]-control.origins[master]);
        control.group_error=positions[slave]-target;
        const MotorFollowParams *m=control.group_follow;
        float weight=m->fast_tracking ? clamp((0.2f-fabsf(control.group_error))/0.1f,0,1) : 0;
        control.follow_weight=weight;
        if (m->fast_tracking) {
            /* Keep follower feedback bandwidth: extra lag inside the speed
             * loop can turn low-speed smoothing into a limit cycle. */
            float previous_master=control.follow_velocity[master];
            for (unsigned i=0;i<2;++i) {
                control.follow_estimate[i]+=0.001f*control.follow_velocity[i];
                float residual=positions[i]-control.follow_estimate[i];
                control.follow_estimate[i]+=0.5f*residual;
                control.follow_velocity[i]+=80*residual;
            }
            /* Tolerate one-count encoder jitter at rest. The motion anchor
             * is separate from the observer's sample-by-sample velocity. */
            if (fabsf(positions[master]-control.follow_motion_anchor)>0.0008628642f) {
                control.follow_count_interval=control.follow_quiet_ms>100 ? 1 : control.follow_quiet_ms+1;
                control.follow_motion_anchor=positions[master]; control.follow_quiet_ms=0;
            } else if (control.follow_quiet_ms<1000) ++control.follow_quiet_ms;
            /* Filter signed acceleration before taking its magnitude. A
             * steady hand turn stays smooth; fast reversals retain the low
             * latency path even when their instantaneous speed is small.
             * The same 50ms activity drives BOTH cutoff and channel blend,
             * avoiding fast cutoff modulation in differentiated setpoints. */
            control.follow_acceleration+=0.0591174f*(1000*(control.follow_velocity[master]-previous_master)-control.follow_acceleration);
            float demand=fmaxf(0.25f*fabsf(control.follow_velocity[master]),0.04f*fabsf(control.follow_acceleration));
            control.follow_activity+=0.02f*(clamp(demand,0,1)-control.follow_activity);
            bool quiet=control.follow_quiet_ms>30 && control.follow_quiet_ms>2U*control.follow_count_interval;
            float fast_blend=quiet ? 0 : clamp((control.follow_activity-0.3f)/0.4f,0,1);
            float cutoff=5+95*fast_blend;
            control.follow_quiet=quiet;
            if (quiet) {
                /* Do not let input smoothing prolong a confirmed stop. Sparse
                 * count intervals retain sub-count motion instead of a fixed timeout. */
                cutoff=100;
            }
            float rate=0.0062831853f*cutoff;
            float step=rate/(1+rate)*(positions[master]-control.follow_command_position);
            control.follow_command_position+=step;
            control.follow_command_velocity=quiet ? control.follow_command_velocity*0.5f : step*1000;
            control.follow_master_velocity=control.follow_command_velocity+fast_blend*(control.follow_velocity[master]-control.follow_command_velocity);
            target+=(1-fast_blend)*k*(control.follow_command_position-positions[master]);
        }
        float cap=position_speed((MotorAxis)slave);
        float error=target-control.group_reference;
        /* Replan in follower mechanical units every millisecond. Reserve a
         * speed-response time in the braking-distance envelope (v^2/2a+Tv).
         * The linear region plus damped reference speed prevents bang-bang
         * chasing around the goal. The master velocity enters BEFORE all
         * trajectory limits, never as an extra unbounded final command. */
        float deceleration=m->fast_tracking ? 100+(m->decel-100)*weight : m->decel;
        float response_time=m->fast_tracking ? 0.021833334f+0.003166666f*weight : 0.5f/m->response+0.001f;
        float master_speed=velocities[master];
        if (m->fast_tracking) master_speed+=weight*(control.follow_master_velocity-master_speed);
        float terminal=clamp(k*master_speed,-cap,cap);
        /* Compare stopping positions, including the moving goal's speed.
         * Adding sqrt(2*a*error) to terminal speed would overestimate the
         * available braking distance during a fast catch-up/reversal. */
        float distance=error+terminal*fabsf(terminal)/(2.0f*deceleration)+terminal*response_time;
        float envelope=braking_speed(distance,deceleration,response_time);
        float linear=terminal+(m->fast_tracking ? 12+(m->response-12)*weight : 0.5f*m->response)*error;
        float desired=clamp(error>0 ? fminf(linear,envelope) : fmaxf(linear,envelope),-cap,cap);
        float previous=control.group_speed;
        if (m->fast_tracking) {
            /* Small following errors use a single pursuit stage. Blend back
             * to the damped R34 reference as the shaft falls 0.1..0.2rad behind,
             * preserving braking distance, offsets and continuous reversals. */
            float acceleration=55+(m->accel-55)*weight;
            float filter=0.048f+0.952f*weight;
            control.group_speed=slew(previous,previous+filter*(desired-previous),acceleration,deceleration);
        } else control.group_speed=slew(previous,previous+2.0f*m->response*0.001f*(desired-previous),
            m->accel,m->decel);
        control.group_reference+=0.0005f*(previous+control.group_speed);
        /* Never snap the reference to a new goal: reversals and offsets must
         * brake through zero, and a reduced cap must not reset the trajectory. */
        set_target((MotorAxis)slave,JERRYFOC_MODE_POSITION,control.group_reference,cap);
    } else {
        float accel=100.0f/fmaxf(1.0f,fabsf(k));
        float decel=150.0f/fmaxf(1.0f,fabsf(k));
        float cap=fminf(speed_limit((MotorAxis)master),speed_limit((MotorAxis)slave)/fabsf(k));
        if (control.group_mode==JERRYFOC_MODE_POSITION)
            cap=fminf(cap,fminf(control.group_position_limit,
                fminf(position_speed((MotorAxis)master),position_speed((MotorAxis)slave)/fabsf(k))));
        float desired=control.group_target;
        if (control.group_mode==JERRYFOC_MODE_POSITION) {
            float error=control.group_target-control.group_reference;
            /* 30 ms is a conservative planning allowance, not a measured
             * motor time constant. Feedforward must reserve braking time. */
            desired=clamp(braking_speed(error,decel,0.03f),-cap,cap);
        }
        control.group_speed=slew(control.group_speed,clamp(desired,-cap,cap),accel,decel);
        float targets[2];
        if (control.group_mode==JERRYFOC_MODE_POSITION) {
            float step=control.group_speed*0.001f;
            if (fabsf(step)>=fabsf(control.group_target-control.group_reference) && step*(control.group_target-control.group_reference)>=0) {
                control.group_reference=control.group_target; control.group_speed=0;
            } else control.group_reference+=step;
            targets[master]=control.origins[master]+control.group_reference;
            targets[slave]=control.origins[slave]+offset()+k*control.group_reference;
            control.group_error=(positions[slave]-control.origins[slave])-offset()-k*(positions[master]-control.origins[master]);
        } else {
            targets[master]=control.group_speed;
            targets[slave]=k*control.group_speed;
            control.group_error=velocities[slave]-k*velocities[master];
        }
        float limits[2]; limits[master]=cap; limits[slave]=cap*fabsf(k);
        set_target(MOTOR_AXIS_1,control.group_mode,targets[0],limits[0]);
        set_target(MOTOR_AXIS_2,control.group_mode,targets[1],limits[1]);
    }
    bool hand=MotorControl_IsHandFollow();
    float tolerance=control.group_mode==JERRYFOC_MODE_POSITION ? 1.0f : 10.0f;
    bool bad=fabsf(control.group_error)>tolerance;
    if (hand) {
        /* Hand motion is not a preplanned synchronous trajectory. Permit
         * transient lag and catch-up, but cap backlog at one mechanical turn
         * and detect a stationary master with a follower making no progress. */
        bool catching=control.group_error*velocities[slave]<0 && fabsf(velocities[slave])>0.1f;
        bad=fabsf(control.group_error)>2.0f*PI ||
            (bad && fabsf(velocities[master])<0.5f && !catching);
    }
    if (HAL_GetTick()-control.group_started>3000U && bad) ++control.error_ms;
    else control.error_ms=0U;
    if (control.error_ms>=500U) {
        command_text=hand ? "FOLLOW ERROR" : "SYNC ERROR";
        control.group_stop_reason=control.m2.stop_reason=command_text;
        (void)MotorSampling_SetOutputEnabled(false);
        (void)MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,false);
        BoardDiag_Fault(0U,DIAG_CONTROL,command_text); MotorControl_StopAll();
    }
}
void MotorControl_Outer(void)
{
    if (!MotorControl_AxisEnabled(MOTOR_AXIS_2)) return;
    BoardDiag_Motion(1U,control.m2.mode,control.m2.target_speed,control.m2.target_position,
        control.m2.target_q,control.m2.velocity,control.m2.encoder.position);
    if (!MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2) || control.m2.aligning) {
        control.m2.reference=control.m2.velocity; JerryFOC_ResetPID(&control.m2.loops.vel); JerryFOC_ResetPID(&control.m2.loops.pos); return;
    }
    if (control.m2.mode==JERRYFOC_MODE_POSITION) {
        control.m2.loops.pos.limit=control.m2.position_speed;
        control.m2.target_speed=clamp(MotorControl_PositionVelocity(MOTOR_AXIS_2,&control.m2.loops.pos,
            control.m2.target_position-control.m2.encoder.position),-control.m2.position_speed,control.m2.position_speed);
    }
    if (control.m2.mode!=JERRYFOC_MODE_TORQUE) {
        control.m2.reference=MotorControl_SpeedReference(MOTOR_AXIS_2,control.m2.reference,control.m2.target_speed);
        control.m2.target_q=MotorControl_VelocityControl(MOTOR_AXIS_2,&control.m2.loops.vel,control.m2.reference,control.m2.velocity);
    }
}
void MotorControl_PrepareOuter(void)
{
    if (!MotorControl_AxisEnabled(MOTOR_AXIS_2)) return;
    update_encoder2(); coordinate();
}
void MotorControl_RecordCycles(uint32_t cycles)
{
    if (cycles>control.max_cycles) control.max_cycles=cycles;
    if (cycles>=17000U && (MotorSampling_IsOutputRequested() ||
        MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2))) {
        control.timing_fault=true;
        (void)MotorSampling_SetOutputEnabled(false);
        (void)MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,false);
        BoardDiag_Fault(0U,DIAG_CONTROL,"CONTROL OVERRUN"); MotorControl_StopAll();
    }
}
