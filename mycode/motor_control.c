#include "motor_control.h"
#include "motor_sampling.h"
#include "MT6701.h"
#include "angle_src_encoder.h"
#include "app_storage.h"
#include "st_hso.h"
#include <string.h>

/* M1 retains its proven encoder/HSO controller. M2 owns all estimator, PI,
 * calibration and run state below; only math/modulation and sampling are shared. */
typedef struct {
    PIDController vel, pos, q, d;
    LowPassFilter speed_filter;
    float sensor_prev, position, velocity, electrical_zero, history[4];
    float reference, target_speed, target_position, target_q, position_speed;
    float iq, id, ia, ib;
    uint32_t samples, align_at;
    uint8_t history_index, history_count;
    bool initialized, was_healthy, aligned, aligning, pending_align, loop;
    JerryFOC_ControlMode mode;
    const char *stop_reason;
} Motor2Controller;
static Motor2Controller m2;
static volatile MotorWorkMode work;
static MotorAxis focus;
static bool hso_selected;
static volatile bool group_active, timing_fault;
static JerryFOC_ControlMode group_mode;
static float group_target, group_reference, group_speed, origins[2], group_error;
static uint32_t group_started, error_ms, max_cycles;
static const char *command_text="READY";
static const char *m1_stop_reason,*group_stop_reason;
static float group_position_limit;
/* Profile and actuator mode cannot change during a follow session. Bind the
 * confirmed preset when arming; a settings draft must not retune a live loop. */
static const MotorFollowParams *group_follow;
static float follow_estimate[2],follow_velocity[2];
static float follow_weight;
static float follow_activity;
static float follow_motion_anchor,follow_acceleration;
static uint16_t follow_quiet_ms;
static uint16_t follow_count_interval;
static bool follow_quiet,follow_hold;
static float follow_command_position,follow_command_velocity,follow_master_velocity;

static float clamp(float x,float lo,float hi) { return fminf(hi,fmaxf(lo,x)); }
static float norm(float a) { a=fmodf(a,2.0f*PI); return a<0 ? a+2.0f*PI : a; }
static float slew(float actual,float target,float accel,float decel)
{
    bool braking=actual*target<0.0f || fabsf(target)<fabsf(actual);
    float step=(braking ? decel : accel)*0.001f;
    if (actual*target<0.0f) target=0.0f;
    return actual+clamp(target-actual,-step,step);
}
static float ratio(void) { return (float)AppStorage_Get()->gear_ratio_x100/100.0f; }
static float offset(void) { return (float)AppStorage_Get()->gear_offset_x100/100.0f; }
static float speed_limit(MotorAxis axis)
{
    uint32_t flags=axis==MOTOR_AXIS_1 ? AppStorage_Get()->flags : AppStorage_Get()->axis2_flags;
    unsigned lim=(flags & APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT;
    if (!lim || lim>APP_SPEED_LIMIT_MAX) lim=APP_SPEED_LIMIT_DEFAULT;
    return fminf((float)lim,MotorParams_GetAxis(axis)->max_speed_mech);
}
static float position_speed(MotorAxis axis)
{
    uint32_t flags=axis==MOTOR_AXIS_1 ? AppStorage_Get()->flags : AppStorage_Get()->axis2_flags;
    unsigned limit=AppSettings_GetPositionSpeed(flags);
    if (!limit || limit>APP_SPEED_LIMIT_MAX) limit=30;
    return fminf((float)limit,speed_limit(axis));
}
bool MotorControl_AxisEnabled(MotorAxis axis)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT) return false;
    return work>=MOTOR_WORK_INDEPENDENT ||
        (axis==MOTOR_AXIS_1 ? work==MOTOR_WORK_SINGLE_M1 : work==MOTOR_WORK_SINGLE_M2);
}
bool MotorControl_IsCoupled(void) { return work>=MOTOR_WORK_SYNC_SPEED; }
bool MotorControl_IsHandFollow(void) { return work>=MOTOR_WORK_MASTER_M1; }
bool MotorControl_IsPassive(MotorAxis axis) { return work==MOTOR_WORK_MASTER_M1+(unsigned)axis; }
bool MotorControl_HsoAllowed(void) { return work==MOTOR_WORK_SINGLE_M1 && hso_selected; }
bool MotorControl_MotionAllowed(MotorAxis axis) { return MotorControl_AxisEnabled(axis) && !MotorControl_IsPassive(axis) && !timing_fault; }
MotorWorkMode MotorControl_GetWork(void) { return work; }
MotorAxis MotorControl_GetFocus(void) { return focus; }
void MotorControl_SetFocus(MotorAxis axis)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT) return;
    focus=axis;
    if (work==MOTOR_WORK_INDEPENDENT) {
        AppSettings *s=AppStorage_Get();
        s->control_config=(s->control_config & ~APP_CONFIG_FOCUS_M2) |
            (axis==MOTOR_AXIS_2 ? APP_CONFIG_FOCUS_M2 : 0U);
    }
}
static MotorAxis work_focus(void)
{
    return work==MOTOR_WORK_SINGLE_M2 || (work==MOTOR_WORK_INDEPENDENT &&
        (AppStorage_Get()->control_config & APP_CONFIG_FOCUS_M2)) ? MOTOR_AXIS_2 : MOTOR_AXIS_1;
}
bool MotorControl_IsGroupActive(void) { return group_active; }
float MotorControl_GroupError(void) { return group_error; }
uint32_t MotorControl_MaxCycles(void) { return max_cycles; }
const char *MotorControl_CommandText(void) { return command_text; }
const char *MotorControl_EncoderLostText(MotorAxis axis)
{
    return axis==MOTOR_AXIS_1 ? "ENCODER1 LOST" : "ENCODER2 LOST";
}
const char *MotorControl_WorkText(void)
{
    static const char *const names[]={"Single M1","Single M2","Independent","Sync speed",
        "Sync position","Master M1","Master M2"};
    return names[work];
}
bool MotorControl_ModeAllowed(JerryFOC_ControlMode mode)
{
    if ((unsigned)mode>JERRYFOC_MODE_POSITION) return false;
    if (MotorControl_HsoAllowed()) return mode==JERRYFOC_MODE_VELOCITY;
    if (work==MOTOR_WORK_SYNC_SPEED) return mode==JERRYFOC_MODE_VELOCITY;
    if (work==MOTOR_WORK_SYNC_POSITION || MotorControl_IsHandFollow()) return mode==JERRYFOC_MODE_POSITION;
    return !MotorControl_IsCoupled() || mode!=JERRYFOC_MODE_TORQUE;
}
static void reset_m2(void)
{
    const MotorParams *m=MotorParams_GetAxis(MOTOR_AXIS_2);
    memset(&m2,0,sizeof(m2));
    m2.vel=(PIDController){.P=m->vel_P,.I=m->vel_I,.limit=m->vel_limit,.output_ramp=m->vel_ramp};
    m2.pos=(PIDController){.P=m->pos_P,.limit=m->pos_limit,.output_ramp=m->pos_ramp};
    m2.d=m2.q=(PIDController){.P=m->cur_P,.I=m->cur_I,.limit=m->cur_limit};
    m2.speed_filter.Tf=m->vel_filter_Tf;
    m2.position_speed=m->pos_limit;
    m2.pending_align=true;
}
static void reset_pi(PIDController *p) { p->error_prev=p->integral_prev=p->output_prev=0.0f; }
static void stop_axis(MotorAxis axis)
{
    if (axis==MOTOR_AXIS_1) { JerryFOC_stop(); return; }
    (void)MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,false);
    (void)MT6701_2_Reserve(false);
    m2.aligning=false; m2.pending_align=false;
    m2.target_q=m2.target_speed=m2.target_position=0.0f;
    m2.reference=m2.velocity;
    m2.mode=JERRYFOC_MODE_TORQUE;
    reset_pi(&m2.vel); reset_pi(&m2.pos); reset_pi(&m2.q); reset_pi(&m2.d);
    JerryFOC_setAxisVoltage(MOTOR_AXIS_2,0,0,0,NULL,NULL);
}
void MotorControl_StopAll(void)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    group_active=false;
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
static bool stationary(void)
{
    if (MotorControl_HsoAllowed() && !StHso_AtRest()) return false;
    if (JerryFOC_isAligning() || m2.aligning || MotorSampling_IsOutputRequested() ||
        MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2)) return false;
    return fabsf(JerryFOC_getVelocity())<=1.0f && fabsf(m2.velocity)<=1.0f;
}
static void save_config(void)
{
    AppSettings *s=AppStorage_Get();
    s->control_config=(s->control_config & APP_CONFIG_FOCUS_M2) | (uint32_t)work | (hso_selected ? 8U : 0U) |
        (MotorParams_GetAxisProfile(MOTOR_AXIS_2)==MOTOR_PROFILE_4015 ? 16U : 0U);
}
bool MotorControl_SelectWork(MotorWorkMode mode)
{
    if ((unsigned)mode>=MOTOR_WORK_COUNT) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!stationary()) { __set_PRIMASK(mask); return false; }
    MotorControl_StopAll();
    work=mode;
    group_stop_reason=NULL;
    focus=work_focus();
    if (!MotorControl_HsoAllowed()) { (void)JerryFOC_useHSO(0); StHso_Enable(false); }
    else StHso_Enable(true);
    MT6701_2_SetRealtime(MotorControl_AxisEnabled(MOTOR_AXIS_2));
    m2.pending_align=MotorControl_AxisEnabled(MOTOR_AXIS_2) && !m2.aligned;
    save_config();
    __set_PRIMASK(mask);
    return true;
}
bool MotorControl_SelectFeedback(bool hso)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (work!=MOTOR_WORK_SINGLE_M1 || !stationary()) { __set_PRIMASK(mask); return false; }
    hso_selected=hso;
    (void)JerryFOC_useHSO(0); StHso_Enable(hso);
    m1_stop_reason=group_stop_reason=NULL;
    if (!hso) JerryFOC_requestAlignment();
    save_config(); __set_PRIMASK(mask); return true;
}
static void clear_saved_axis(MotorAxis axis)
{
    AppSettings *s=AppStorage_Get();
    uint32_t defaults=(30UL<<APP_POSITION_SPEED_SHIFT)|((uint32_t)APP_SPEED_LIMIT_DEFAULT<<APP_SPEED_LIMIT_SHIFT);
    if (axis==MOTOR_AXIS_1) {
        s->speed_x10=s->voltage_x100=s->position_x100=0;
        s->flags=(s->flags & (APP_FLAG_LED_ENABLE|APP_FLAG_MOTOR_4015)) | defaults;
    } else {
        s->axis2_speed_x10=s->axis2_voltage_x100=s->axis2_position_x100=0;
        s->axis2_flags=defaults;
    }
}
bool MotorControl_SelectProfile(MotorAxis axis,MotorProfile profile)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT || (unsigned)profile>=MOTOR_PROFILE_COUNT) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!stationary()) { __set_PRIMASK(mask); return false; }
    bool ok=true;
    if (profile!=MotorParams_GetAxisProfile(axis)) {
        if (axis==MOTOR_AXIS_1) {
            ok=JerryFOC_selectMotor(profile)!=0U;
            if (ok) {
                AppStorage_Get()->flags=(AppStorage_Get()->flags & ~APP_FLAG_MOTOR_4015) |
                    (profile==MOTOR_PROFILE_4015 ? APP_FLAG_MOTOR_4015 : 0U);
                StHso_Enable(MotorControl_HsoAllowed());
            }
        } else {
            stop_axis(axis); (void)MotorParams_SelectAxis(axis,profile);
            reset_m2(); save_config();
        }
        if (ok) clear_saved_axis(axis);
    }
    __set_PRIMASK(mask); return ok;
}
bool MotorControl_SetGear(int32_t r,int32_t o)
{
    if (r==0 || r< -1000 || r>1000 || o< -6283 || o>6283) return false;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!stationary()) { __set_PRIMASK(mask); return false; }
    AppStorage_Get()->gear_ratio_x100=r; AppStorage_Get()->gear_offset_x100=o;
    __set_PRIMASK(mask); return true;
}
bool MotorControl_RequestAlign(MotorAxis axis)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (!MotorControl_AxisEnabled(axis) || !stationary()) { __set_PRIMASK(mask); return false; }
    if (MotorControl_IsPassive(axis)) {
        if (axis==MOTOR_AXIS_2) MT6701_2_Retry();
        __set_PRIMASK(mask); return true; /* Encoder retry only; never align the hand shaft. */
    }
    if (axis==MOTOR_AXIS_1) JerryFOC_requestAlignment();
    else { MT6701_2_Retry(); m2.pending_align=true; m2.aligned=false; m2.stop_reason=NULL; }
    __set_PRIMASK(mask); return true;
}
void MotorControl_Init(void)
{
    AppSettings *s=AppStorage_Get();
    work=(MotorWorkMode)(s->control_config & 7U);
    if (work>=MOTOR_WORK_COUNT) work=MOTOR_WORK_SINGLE_M1;
    hso_selected=(s->control_config & 8U)!=0U;
    focus=work_focus();
    (void)JerryFOC_selectMotor((s->flags & APP_FLAG_MOTOR_4015) ? MOTOR_PROFILE_4015 : MOTOR_PROFILE_2804);
    (void)MotorParams_SelectAxis(MOTOR_AXIS_2,(s->control_config & 16U) ? MOTOR_PROFILE_4015 : MOTOR_PROFILE_2804);
    reset_m2();
    if (!s->gear_ratio_x100 || s->gear_ratio_x100< -1000 || s->gear_ratio_x100>1000) s->gear_ratio_x100=100;
    if (s->gear_offset_x100< -6283 || s->gear_offset_x100>6283) s->gear_offset_x100=0;
    group_active=timing_fault=false;
    m1_stop_reason=group_stop_reason=NULL;
    MT6701_2_SetRealtime(MotorControl_AxisEnabled(MOTOR_AXIS_2));
    StHso_Enable(MotorControl_HsoAllowed());
    save_config();
}
void MotorControl_GetAxis(MotorAxis axis,MotorAxisSnapshot *out)
{
    if (!out || (unsigned)axis>=MOTOR_AXIS_COUNT) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    const char *sampling=MotorSampling_GetAxisStatusText(axis);
    if (axis==MOTOR_AXIS_1) {
        extern volatile uint8_t flag_use_current_loop;
        *out=(MotorAxisSnapshot){.mode=JerryFOC_getMode(),.velocity=JerryFOC_getVelocity(),
            .position=JerryFOC_getAngle(),.electrical_angle=JerryFOC_getElectricalAngle(),
            .iq=JerryFOC_getIq(),.id=JerryFOC_getId(),.ia=JerryFOC_getPhaseCurrent_A(),.ib=JerryFOC_getPhaseCurrent_B(),
            .target_velocity=JerryFOC_getTargetVelocity(),.target_position=JerryFOC_getTargetPosition(),
            .target_iq=JerryFOC_Target_Iq,.healthy=MT6701_IsHealthy()!=0,.aligned=AngleSrc_Encoder_IsAligned()!=0,
            .active=MotorSampling_IsOutputRequested(),.aligning=JerryFOC_isAligning()!=0,
            .current_loop=flag_use_current_loop!=0,.ready=JerryFOC_isDriveReady()!=0,.status=sampling};
        if (JerryFOC_hasHSOFault()) out->status="HSO STOPPED";
        else if (m1_stop_reason && out->ready) out->status=m1_stop_reason;
        if (MotorControl_HsoAllowed()) {
            out->healthy=!JerryFOC_hasHSOFault();
            out->aligned=true; /* Sensorless alignment belongs to its startup FSM. */
        }
    } else {
        MT6701_2_Snapshot enc; MT6701_2_GetSnapshot(&enc);
        *out=(MotorAxisSnapshot){.mode=m2.mode,.velocity=m2.velocity,.position=m2.position,
            .electrical_angle=norm(MotorParams_GetAxis(axis)->pole_pairs*enc.angle-m2.electrical_zero),
            .iq=m2.iq,.id=m2.id,.ia=m2.ia,.ib=m2.ib,.target_velocity=m2.target_speed,
            .target_position=m2.target_position,.target_iq=m2.target_q,.healthy=enc.healthy!=0,
            .aligned=m2.aligned,.active=MotorSampling_IsAxisOutputRequested(axis),.aligning=m2.aligning,
            .current_loop=m2.loop,.ready=MotorSampling_IsAxisReady(axis),.status=sampling};
        if (m2.stop_reason && out->ready) out->status=m2.stop_reason;
    }
    if (MotorControl_IsPassive(axis)) {
        out->ready=out->healthy;
        out->status=out->healthy ? "HAND MASTER" : MotorControl_EncoderLostText(axis);
    }
    if (out->ready && !out->healthy) out->status=MotorControl_EncoderLostText(axis);
    if (group_stop_reason && MotorControl_IsCoupled() && !out->active &&
        (out->ready || MotorControl_IsPassive(axis))) out->status=group_stop_reason;
    if (timing_fault) { out->ready=false; out->status="CONTROL OVERRUN"; }
    if (!MotorControl_AxisEnabled(axis)) { out->ready=false; out->status="AXIS DISABLED"; }
    __set_PRIMASK(mask);
}
static bool can_start(MotorAxis axis)
{
    MotorAxisSnapshot s; MotorControl_GetAxis(axis,&s);
    if (axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed()) {
        if (!MotorSampling_IsReady() || !MotorSampling_IsCalibrated()) {
            command_text=MotorSampling_GetAxisStatusText(axis); return false;
        }
        return !timing_fault;
    }
    if (!MotorControl_AxisEnabled(axis)) command_text="AXIS DISABLED";
    else if (!s.ready && !(axis==MOTOR_AXIS_1 && JerryFOC_hasHSOFault() &&
             MotorSampling_IsAxisReady(axis))) command_text=s.status;
    else if (!s.healthy) command_text=MotorControl_EncoderLostText(axis);
    else if (axis==MOTOR_AXIS_2 && m2.history_count<4U) command_text="ENCODER SETTLE";
    else if (!isfinite(s.position) || !isfinite(s.velocity)) command_text="ENCODER VALUE";
    else if (MotorControl_IsPassive(axis)) {
        if (!s.active && !s.aligning) return true;
        command_text="MASTER NOT FREE";
    }
    else if (!s.aligned || s.aligning) command_text="NOT ALIGNED";
    else if (!s.active && (!isfinite(s.velocity) || fabsf(s.velocity)>1.0f)) command_text="Wait for stop";
    else if (timing_fault) command_text="CONTROL OVERRUN";
    else return true;
    return false;
}
static void set_mode(MotorAxis axis,JerryFOC_ControlMode mode,bool loop)
{
    if (axis==MOTOR_AXIS_1) {
        g_openloop_debug=0U; JerryFOC_useCurrentLoop(loop); JerryFOC_setMode(mode); return;
    }
    if (m2.loop!=loop || m2.mode!=mode) {
        float hold=m2.loop==loop && MotorSampling_IsAxisOutputRequested(axis) ? m2.target_q : 0.0f;
        reset_pi(&m2.vel); reset_pi(&m2.pos); reset_pi(&m2.q); reset_pi(&m2.d);
        m2.reference=m2.velocity;
        m2.vel.integral_prev=m2.vel.output_prev=hold;
        m2.target_q=hold;
    }
    m2.loop=loop; m2.mode=mode;
    const MotorParams *m=MotorParams_GetAxis(axis);
    m2.vel.limit=loop ? m->rated_current : m->vel_limit;
    m2.vel.P=loop ? m->vel_current_P : m->vel_P;
    m2.vel.I=loop ? m->vel_current_I : m->vel_I;
}
static void set_target(MotorAxis axis,JerryFOC_ControlMode mode,float value,float limit)
{
    if (axis==MOTOR_AXIS_1) {
        if (mode==JERRYFOC_MODE_VELOCITY) JerryFOC_setVelocity(value);
        else if (mode==JERRYFOC_MODE_POSITION) JerryFOC_setAbsolutePosition(value,
            fminf(limit>0 ? limit : MotorParams_GetAxis(axis)->pos_limit,speed_limit(axis)));
        else JerryFOC_setCurrent(value);
    } else {
        const MotorParams *m=MotorParams_GetAxis(axis);
        if (mode==JERRYFOC_MODE_VELOCITY) m2.target_speed=clamp(value,-speed_limit(axis),speed_limit(axis));
        else if (mode==JERRYFOC_MODE_POSITION) {
            m2.target_position=value;
            m2.position_speed=clamp(limit>0 ? limit : m->pos_limit,0.1f,speed_limit(axis));
        } else {
            float max=m2.loop ? m->rated_current : m->vel_limit;
            m2.target_q=clamp(value,-max,max);
        }
    }
}
bool MotorControl_StartFollow(void)
{
    if (!MotorControl_IsHandFollow()) { command_text="SELECT MASTER"; return false; }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (group_active) { __set_PRIMASK(mask); return true; } /* Do not re-zero an active session. */
    MotorAxis master=work==MOTOR_WORK_MASTER_M2 ? MOTOR_AXIS_2 : MOTOR_AXIS_1;
    MotorAxis slave=(MotorAxis)(1U-master);
    if (!can_start(master) || !can_start(slave)) { __set_PRIMASK(mask); return false; }
    stop_axis(master);
    if (!MT6701_2_Reserve(true)) { command_text="SPI2 BUSY"; __set_PRIMASK(mask); return false; }
    origins[0]=JerryFOC_getAngle(); origins[1]=m2.position;
    group_reference=origins[slave];
    group_speed=slave==MOTOR_AXIS_1 ? JerryFOC_getVelocity() : m2.velocity;
    uint32_t flags=slave==MOTOR_AXIS_1 ? AppStorage_Get()->flags : AppStorage_Get()->axis2_flags;
    float limit=position_speed(slave);
    set_mode(slave,JERRYFOC_MODE_POSITION,(flags & APP_FLAG_CURRENT_LOOP)!=0U);
    const MotorParams *m=MotorParams_GetAxis(slave);
    group_follow=(flags & APP_FLAG_CURRENT_LOOP) ? &m->follow_current : &m->follow_voltage;
    for (unsigned i=0;i<2;++i) {
        follow_estimate[i]=origins[i];
        follow_velocity[i]=i==MOTOR_AXIS_1 ? JerryFOC_getVelocity() : m2.velocity;
    }
    follow_weight=0;
    follow_activity=1; follow_acceleration=0;
    follow_motion_anchor=origins[master];
    follow_quiet_ms=0; follow_count_interval=1;
    follow_quiet=follow_hold=false;
    follow_command_position=origins[master];
    follow_command_velocity=follow_master_velocity=follow_velocity[master];
    set_target(slave,JERRYFOC_MODE_POSITION,origins[slave],limit);
    group_mode=JERRYFOC_MODE_POSITION; group_error=0; error_ms=0; group_started=HAL_GetTick();
    group_stop_reason=NULL;
    group_active=true;
    bool ok=slave==MOTOR_AXIS_1 ? JerryFOC_startOutput()!=0U : MotorSampling_SetAxisOutputEnabled(slave,true);
    if (!ok) MotorControl_StopAll();
    else { m1_stop_reason=NULL; m2.stop_reason=NULL; }
    command_text=ok ? "READY" : "START REJECTED";
    __set_PRIMASK(mask); return ok;
}
bool MotorControl_IsFastFollow(MotorAxis axis)
{
    return (unsigned)axis<MOTOR_AXIS_COUNT && group_active && MotorControl_IsHandFollow() &&
        !MotorControl_IsPassive(axis) && group_follow->fast_tracking;
}
float MotorControl_VelocityControl(MotorAxis axis,PIDController *velocity,float reference,float measured)
{
    bool fast=MotorControl_IsFastFollow(axis);
    float error=reference-(fast ? follow_velocity[axis] : measured);
    float integral=velocity->integral_prev;
    float gain=velocity->P;
    float igain=velocity->I;
    /* Retain R35 moving gains. Withdraw R36's extra low-speed P and terminal
     * damping after the real-board vibration report and encoder-noise tests. */
    if (fast) {
        velocity->P=gain+(group_follow->velocity_P-gain)*clamp(fabsf(error),0,1);
        /* Hysteresis in follower radians: enter within four encoder counts,
         * leave beyond eight. Preserve the holding integral/load torque;
         * suppress tiny speed-noise corrections instead of disabling PWM. */
        if (!follow_quiet || fabsf(group_error)>0.003068f || fabsf(reference)>0.15f)
            follow_hold=false;
        else if (follow_quiet_ms>60 && fabsf(group_error)<0.001534f &&
                 fabsf(reference)<0.05f && fabsf(follow_velocity[axis])<0.08f)
            follow_hold=true;
        if (follow_hold) {
            velocity->I=0;
            error=copysignf(fmaxf(0,fabsf(error)-0.05f),error);
        }
    }
    float output=JerryFOC_PID_Calc(velocity,error);
    velocity->P=gain; velocity->I=igain;
    /* Conditional integration during fast-follow current saturation. Allow
     * reverse error to unwind; never carry a saturated acceleration into stop. */
    if (fast && output*error>0 && fabsf(output)>=velocity->limit)
        velocity->integral_prev=integral;
    return output;
}
float MotorControl_SpeedReference(MotorAxis axis,float previous,float target)
{
    float weight=MotorControl_IsFastFollow(axis) ? follow_weight : 0;
    return slew(previous,target,100+200*weight,150+450*weight);
}
float MotorControl_PositionVelocity(MotorAxis axis,PIDController *position,float error)
{
    if (!group_active || !MotorControl_IsHandFollow() || MotorControl_IsPassive(axis))
        return JerryFOC_PID_Calc(position,error);
    const MotorFollowParams *m=group_follow;
    float velocity=axis==MOTOR_AXIS_1 ? JerryFOC_getVelocity() : m2.velocity;
    if (m->fast_tracking) velocity=follow_velocity[axis];
    /* Feed the velocity loop a planned speed, with a small position correction
     * and relative-speed damping. No position integral or step derivative. */
    /* Keep the planned position/speed pair through braking and settling.
     * Do not switch to a second raw-position PD controller at the endpoint. */
    return group_speed+m->P*error+m->damping*(group_speed-velocity);
}
bool MotorControl_Command(MotorAxis axis,JerryFOC_ControlMode mode,float value,float limit,bool loop)
{
    if (MotorControl_IsHandFollow()) { command_text="Use Follow"; return false; }
    if ((unsigned)axis>=MOTOR_AXIS_COUNT || !isfinite(value) || !isfinite(limit) || !MotorControl_ModeAllowed(mode)) {
        command_text="MODE / VALUE"; return false;
    }
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if (axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed() && value==0.0f) {
        MotorControl_Stop(axis); command_text="READY"; __set_PRIMASK(mask); return true;
    }
    if (axis==MOTOR_AXIS_1 && MotorControl_HsoAllowed()) {
        value=clamp(value,-speed_limit(axis),speed_limit(axis));
        if (!StHso_ValidTarget(value)) {
            command_text="HSO SPEED/DIR"; __set_PRIMASK(mask); return false;
        }
    }
    bool coupled=MotorControl_IsCoupled();
    if (coupled && group_active && group_mode!=mode) {
        command_text="Stop before mode"; __set_PRIMASK(mask); return false;
    }
    if (!can_start(axis) || (coupled && (!can_start(MOTOR_AXIS_1) || !can_start(MOTOR_AXIS_2)))) {
        __set_PRIMASK(mask); return false;
    }
    if ((axis==MOTOR_AXIS_2 || coupled) && !MT6701_2_Reserve(true)) {
        command_text="SPI2 BUSY"; __set_PRIMASK(mask); return false;
    }
    if (coupled) {
        if (!group_active || group_mode!=mode) {
            MotorAxisSnapshot a,b; MotorControl_GetAxis(MOTOR_AXIS_1,&a); MotorControl_GetAxis(MOTOR_AXIS_2,&b);
            origins[0]=a.position; origins[1]=b.position;
            group_reference=0.0f; group_speed=0.0f; error_ms=0U; group_started=HAL_GetTick();
        }
        group_stop_reason=NULL;
        group_mode=mode; group_target=value;
        group_position_limit=limit>0.0f ? limit : 30.0f;
        if (mode==JERRYFOC_MODE_VELOCITY) {
            float cap=fminf(speed_limit(MOTOR_AXIS_1),speed_limit(MOTOR_AXIS_2)/fabsf(ratio()));
            group_target=clamp(value,-cap,cap);
        }
        bool loop1=(AppStorage_Get()->flags & APP_FLAG_CURRENT_LOOP)!=0U;
        bool loop2=(AppStorage_Get()->axis2_flags & APP_FLAG_CURRENT_LOOP)!=0U;
        set_mode(MOTOR_AXIS_1,mode,loop1); set_mode(MOTOR_AXIS_2,mode,loop2);
        /* Neutral targets before the two-frame output arming delay. */
        if (!group_active) {
            set_target(MOTOR_AXIS_1,mode,mode==JERRYFOC_MODE_POSITION ? origins[0] : 0,limit);
            set_target(MOTOR_AXIS_2,mode,mode==JERRYFOC_MODE_POSITION ? origins[1] : 0,limit);
        }
        group_active=true;
        bool ok=JerryFOC_startOutput() && MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true);
        if (!ok) { MotorControl_StopAll(); command_text="START REJECTED"; }
        else { command_text="READY"; m1_stop_reason=m2.stop_reason=NULL; }
        __set_PRIMASK(mask); return ok;
    }
    set_mode(axis,mode,loop);
    if (mode==JERRYFOC_MODE_VELOCITY) value=clamp(value,-speed_limit(axis),speed_limit(axis));
    set_target(axis,mode,value,limit);
    bool ok=axis==MOTOR_AXIS_1 ? JerryFOC_startOutput()!=0U : MotorSampling_SetAxisOutputEnabled(axis,true);
    if (!ok && axis==MOTOR_AXIS_2) { stop_axis(axis); command_text="START REJECTED"; }
    else {
        command_text=ok ? "READY" : MotorControl_HsoAllowed() ? "HSO START GATE" : "START REJECTED";
        if (ok) { if (axis==MOTOR_AXIS_2) m2.stop_reason=NULL; else m1_stop_reason=NULL; }
    }
    __set_PRIMASK(mask); return ok;
}
void MotorControl_Service(uint32_t now)
{
    if (!MotorControl_MotionAllowed(MOTOR_AXIS_2)) return;
    MT6701_2_Snapshot enc; MT6701_2_GetSnapshot(&enc);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    bool m1_align_done=!MotorControl_MotionAllowed(MOTOR_AXIS_1) ||
                       (!JerryFOC_isAligning() && !JerryFOC_needsAlignment());
    if (m1_align_done && m2.pending_align && !m2.aligning && MotorSampling_IsAxisReady(MOTOR_AXIS_2) &&
        enc.healthy && fabsf(m2.velocity)<=1.0f && MT6701_2_Reserve(true)) {
        m2.pending_align=false; m2.aligning=true; m2.aligned=false; m2.align_at=now;
        if (!MotorSampling_SetAxisOutputEnabled(MOTOR_AXIS_2,true)) stop_axis(MOTOR_AXIS_2);
    }
    if (m2.aligning && (now-m2.align_at>=3000U || !enc.healthy || !MotorSampling_IsAxisReady(MOTOR_AXIS_2))) {
        bool ok=enc.healthy && MotorSampling_IsAxisReady(MOTOR_AXIS_2) && now-m2.align_at>=3000U;
        if (ok) {
            m2.electrical_zero=norm(MotorParams_GetAxis(MOTOR_AXIS_2)->pole_pairs*enc.angle);
            m2.position=m2.velocity=0.0f; m2.sensor_prev=enc.angle;
            memset(m2.history,0,sizeof(m2.history)); m2.history_count=m2.history_index=0;
            m2.speed_filter.y_prev=0.0f; m2.aligned=true; m2.stop_reason=NULL;
        }
        stop_axis(MOTOR_AXIS_2);
    }
    __set_PRIMASK(mask);
}
static float current_pi(PIDController *p,float error)
{
    p->integral_prev=clamp(p->integral_prev+p->I*0.00005f*(error+p->error_prev),-p->limit,p->limit);
    p->error_prev=error;
    return clamp(p->P*error+p->integral_prev,-p->limit,p->limit);
}
void MotorControl_Fast(void)
{
    if (MotorControl_AxisEnabled(MOTOR_AXIS_1) && MotorSampling_IsOutputRequested() &&
        !g_openloop_debug && !MotorControl_HsoAllowed() &&
        (!MT6701_IsHealthy() || !MotorSampling_IsAxisReady(MOTOR_AXIS_1))) {
        m1_stop_reason=MT6701_IsHealthy() ? MotorSampling_GetAxisStatusText(MOTOR_AXIS_1) : MotorControl_EncoderLostText(MOTOR_AXIS_1);
        command_text=group_stop_reason=m1_stop_reason;
        MotorControl_Stop(MOTOR_AXIS_1);
    }
    if (!MotorControl_AxisEnabled(MOTOR_AXIS_2)) return;
    MT6701_2_Snapshot enc; MT6701_2_GetSnapshot(&enc);
    bool active=MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2);
    /* A failed startup probe must not auto-align/energize after reconnection. */
    if (enc.stopped) m2.pending_align=false;
    if (active && (!enc.healthy || !MotorSampling_IsAxisReady(MOTOR_AXIS_2))) {
        m2.stop_reason=enc.healthy ? MotorSampling_GetAxisStatusText(MOTOR_AXIS_2) : MotorControl_EncoderLostText(MOTOR_AXIS_2);
        if (!enc.healthy) m2.aligned=false;
        command_text=group_stop_reason=m2.stop_reason;
        MotorControl_Stop(MOTOR_AXIS_2); return;
    }
    if (group_active && (!MT6701_IsHealthy() || !enc.healthy ||
        MotorSampling_IsOutputRequested()==MotorControl_IsPassive(MOTOR_AXIS_1) ||
        active==MotorControl_IsPassive(MOTOR_AXIS_2) ||
        (!MotorControl_IsPassive(MOTOR_AXIS_1) && !JerryFOC_isDriveReady()))) {
        command_text=group_stop_reason=!MT6701_IsHealthy() ? MotorControl_EncoderLostText(MOTOR_AXIS_1) :
            !enc.healthy ? MotorControl_EncoderLostText(MOTOR_AXIS_2) : "GROUP OUTPUT OFF";
        MotorControl_StopAll(); return;
    }
    const MotorSampleFrame *f=MotorSampling_GetFrame();
    m2.ia=f->m2_current[0]; m2.ib=f->m2_current[1];
    float angle=norm(MotorParams_GetAxis(MOTOR_AXIS_2)->pole_pairs*enc.angle-m2.electrical_zero);
    float alpha,beta,d,q;
    JerryFOC_Clarke(m2.ia,m2.ib,&alpha,&beta); JerryFOC_Park(alpha,beta,angle,&d,&q);
    m2.id=0.5f*(m2.id+d); m2.iq=0.5f*(m2.iq+q);
    if (!active) return;
    if (m2.aligning) {
        const MotorParams *m=MotorParams_GetAxis(MOTOR_AXIS_2);
        JerryFOC_setAxisVoltage(MOTOR_AXIS_2,0,fminf(3.0f,m->Rs*m->rated_current),0,NULL,NULL);
        return;
    }
    if (!m2.aligned) { MotorControl_Stop(MOTOR_AXIS_2); return; }
    float uq=m2.target_q,ud=0,aq,ad;
    if (m2.loop) {
        float cap=MotorParams_GetAxis(MOTOR_AXIS_2)->rated_current;
        uq=current_pi(&m2.q,clamp(m2.target_q,-cap,cap)-m2.iq);
        ud=current_pi(&m2.d,-m2.id);
    }
    JerryFOC_setAxisVoltage(MOTOR_AXIS_2,uq,ud,angle,&aq,&ad);
    if (m2.loop) {
        m2.q.integral_prev=clamp(m2.q.integral_prev+0.2f*(aq-uq),-m2.q.limit,m2.q.limit);
        m2.d.integral_prev=clamp(m2.d.integral_prev+0.2f*(ad-ud),-m2.d.limit,m2.d.limit);
    }
}
static void update_encoder2(void)
{
    MT6701_2_Snapshot enc; MT6701_2_GetSnapshot(&enc);
    if (!enc.healthy) { m2.was_healthy=false; m2.velocity=0.0f; return; }
    if (enc.samples==m2.samples) return;
    m2.samples=enc.samples;
    if (!m2.initialized || !m2.was_healthy) {
        m2.initialized=m2.was_healthy=true;
        m2.sensor_prev=enc.angle;
        for (unsigned i=0;i<4;++i) m2.history[i]=m2.position;
        m2.history_count=m2.history_index=0; m2.speed_filter.y_prev=0; return;
    }
    float delta=enc.angle-m2.sensor_prev;
    if (delta>PI) delta-=2.0f*PI;
    if (delta< -PI) delta+=2.0f*PI;
    m2.sensor_prev=enc.angle; m2.position+=delta;
    float old=m2.history[m2.history_index];
    m2.history[m2.history_index]=m2.position;
    m2.history_index=(m2.history_index+1U)%4U;
    if (m2.history_count<4U) ++m2.history_count;
    m2.velocity=JerryFOC_LPF_Calc(&m2.speed_filter,(m2.position-old)/(0.001f*m2.history_count));
}
static void coordinate(void)
{
    if (!group_active) return;
    float positions[2]={JerryFOC_getAngle(),m2.position};
    float velocities[2]={JerryFOC_getVelocity(),m2.velocity};
    unsigned master=work==MOTOR_WORK_MASTER_M2 ? 1U : 0U, slave=1U-master;
    float k=ratio();
    if (MotorControl_IsHandFollow()) {
        float target=origins[slave]+offset()+k*(positions[master]-origins[master]);
        group_error=positions[slave]-target;
        const MotorFollowParams *m=group_follow;
        float weight=m->fast_tracking ? clamp((0.2f-fabsf(group_error))/0.1f,0,1) : 0;
        follow_weight=weight;
        if (m->fast_tracking) {
            /* Keep follower feedback bandwidth: extra lag inside the speed
             * loop can turn low-speed smoothing into a limit cycle. */
            float previous_master=follow_velocity[master];
            for (unsigned i=0;i<2;++i) {
                follow_estimate[i]+=0.001f*follow_velocity[i];
                float residual=positions[i]-follow_estimate[i];
                follow_estimate[i]+=0.5f*residual;
                follow_velocity[i]+=80*residual;
            }
            /* Tolerate one-count encoder jitter at rest. The motion anchor
             * is separate from the observer's sample-by-sample velocity. */
            if (fabsf(positions[master]-follow_motion_anchor)>0.0008628642f) {
                follow_count_interval=follow_quiet_ms>100 ? 1 : follow_quiet_ms+1;
                follow_motion_anchor=positions[master]; follow_quiet_ms=0;
            } else if (follow_quiet_ms<1000) ++follow_quiet_ms;
            /* Filter signed acceleration before taking its magnitude. A
             * steady hand turn stays smooth; fast reversals retain the low
             * latency path even when their instantaneous speed is small.
             * The same 50ms activity drives BOTH cutoff and channel blend,
             * avoiding fast cutoff modulation in differentiated setpoints. */
            follow_acceleration+=0.0591174f*(1000*(follow_velocity[master]-previous_master)-follow_acceleration);
            float demand=fmaxf(0.25f*fabsf(follow_velocity[master]),0.04f*fabsf(follow_acceleration));
            follow_activity+=0.02f*(clamp(demand,0,1)-follow_activity);
            bool quiet=follow_quiet_ms>30 && follow_quiet_ms>2U*follow_count_interval;
            float fast_blend=quiet ? 0 : clamp((follow_activity-0.3f)/0.4f,0,1);
            float cutoff=5+95*fast_blend;
            follow_quiet=quiet;
            if (quiet) {
                /* Do not let input smoothing prolong a confirmed stop. Sparse
                 * count intervals retain sub-count motion instead of a fixed timeout. */
                cutoff=100;
            }
            float rate=0.0062831853f*cutoff;
            float step=rate/(1+rate)*(positions[master]-follow_command_position);
            follow_command_position+=step;
            follow_command_velocity=quiet ? follow_command_velocity*0.5f : step*1000;
            follow_master_velocity=follow_command_velocity+fast_blend*(follow_velocity[master]-follow_command_velocity);
            target+=(1-fast_blend)*k*(follow_command_position-positions[master]);
        }
        float cap=position_speed((MotorAxis)slave);
        float error=target-group_reference;
        /* Replan in follower mechanical units every millisecond. Reserve a
         * speed-response time in the braking-distance envelope (v^2/2a+Tv).
         * The linear region plus damped reference speed prevents bang-bang
         * chasing around the goal. The master velocity enters BEFORE all
         * trajectory limits, never as an extra unbounded final command. */
        float deceleration=m->fast_tracking ? 100+(m->decel-100)*weight : m->decel;
        float response_time=m->fast_tracking ? 0.021833334f+0.003166666f*weight : 0.5f/m->response+0.001f;
        float margin=deceleration*response_time;
        float master_speed=velocities[master];
        if (m->fast_tracking) master_speed+=weight*(follow_master_velocity-master_speed);
        float terminal=clamp(k*master_speed,-cap,cap);
        /* Compare stopping positions, including the moving goal's speed.
         * Adding sqrt(2*a*error) to terminal speed would overestimate the
         * available braking distance during a fast catch-up/reversal. */
        float distance=error+terminal*fabsf(terminal)/(2.0f*deceleration)+terminal*response_time;
        float envelope=copysignf(sqrtf(margin*margin+2.0f*deceleration*fabsf(distance))-margin,distance);
        float linear=terminal+(m->fast_tracking ? 12+(m->response-12)*weight : 0.5f*m->response)*error;
        float desired=clamp(error>0 ? fminf(linear,envelope) : fmaxf(linear,envelope),-cap,cap);
        float previous=group_speed;
        if (m->fast_tracking) {
            /* Small following errors use a single pursuit stage. Blend back
             * to the damped R34 reference as the shaft falls 0.1..0.2rad behind,
             * preserving braking distance, offsets and continuous reversals. */
            float acceleration=55+(m->accel-55)*weight;
            float filter=0.048f+0.952f*weight;
            group_speed=slew(previous,previous+filter*(desired-previous),acceleration,deceleration);
        } else group_speed=slew(previous,previous+2.0f*m->response*0.001f*(desired-previous),
            m->accel,m->decel);
        group_reference+=0.0005f*(previous+group_speed);
        /* Never snap the reference to a new goal: reversals and offsets must
         * brake through zero, and a reduced cap must not reset the trajectory. */
        set_target((MotorAxis)slave,JERRYFOC_MODE_POSITION,group_reference,cap);
    } else {
        float accel=100.0f/fmaxf(1.0f,fabsf(k));
        float decel=150.0f/fmaxf(1.0f,fabsf(k));
        float cap=fminf(speed_limit((MotorAxis)master),speed_limit((MotorAxis)slave)/fabsf(k));
        if (group_mode==JERRYFOC_MODE_POSITION) cap=fminf(cap,group_position_limit);
        float desired=group_target;
        if (group_mode==JERRYFOC_MODE_POSITION) {
            float error=group_target-group_reference;
            desired=copysignf(fminf(cap,sqrtf(2.0f*decel*fabsf(error))),error);
        }
        group_speed=slew(group_speed,clamp(desired,-cap,cap),accel,decel);
        float targets[2];
        if (group_mode==JERRYFOC_MODE_POSITION) {
            float step=group_speed*0.001f;
            if (fabsf(step)>=fabsf(group_target-group_reference) && step*(group_target-group_reference)>=0) {
                group_reference=group_target; group_speed=0;
            } else group_reference+=step;
            targets[master]=origins[master]+group_reference;
            targets[slave]=origins[slave]+offset()+k*group_reference;
            group_error=(positions[slave]-origins[slave])-offset()-k*(positions[master]-origins[master]);
        } else {
            targets[master]=group_speed;
            targets[slave]=k*group_speed;
            group_error=velocities[slave]-k*velocities[master];
        }
        float limits[2]; limits[master]=cap; limits[slave]=cap*fabsf(k);
        set_target(MOTOR_AXIS_1,group_mode,targets[0],limits[0]);
        set_target(MOTOR_AXIS_2,group_mode,targets[1],limits[1]);
    }
    bool hand=MotorControl_IsHandFollow();
    float tolerance=group_mode==JERRYFOC_MODE_POSITION ? 1.0f : 10.0f;
    bool bad=fabsf(group_error)>tolerance;
    if (hand) {
        /* Hand motion is not a preplanned synchronous trajectory. Permit
         * transient lag and catch-up, but cap backlog at one mechanical turn
         * and detect a stationary master with a follower making no progress. */
        bool catching=group_error*velocities[slave]<0 && fabsf(velocities[slave])>0.1f;
        bad=fabsf(group_error)>2.0f*PI ||
            (bad && fabsf(velocities[master])<0.5f && !catching);
    }
    if (HAL_GetTick()-group_started>3000U && bad) ++error_ms;
    else error_ms=0U;
    if (error_ms>=500U) {
        command_text=hand ? "FOLLOW ERROR" : "SYNC ERROR";
        group_stop_reason=m2.stop_reason=command_text; MotorControl_StopAll();
    }
}
void MotorControl_Outer(void)
{
    if (!MotorControl_AxisEnabled(MOTOR_AXIS_2)) return;
    if (!MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2) || m2.aligning) {
        m2.reference=m2.velocity; reset_pi(&m2.vel); reset_pi(&m2.pos); return;
    }
    if (m2.mode==JERRYFOC_MODE_POSITION) {
        m2.pos.limit=m2.position_speed;
        m2.target_speed=clamp(MotorControl_PositionVelocity(MOTOR_AXIS_2,&m2.pos,
            m2.target_position-m2.position),-m2.position_speed,m2.position_speed);
    }
    if (m2.mode!=JERRYFOC_MODE_TORQUE) {
        m2.reference=MotorControl_SpeedReference(MOTOR_AXIS_2,m2.reference,m2.target_speed);
        m2.target_q=MotorControl_VelocityControl(MOTOR_AXIS_2,&m2.vel,m2.reference,m2.velocity);
    }
}
void MotorControl_PrepareOuter(void)
{
    if (!MotorControl_AxisEnabled(MOTOR_AXIS_2)) return;
    update_encoder2(); coordinate();
}
void MotorControl_RecordCycles(uint32_t cycles)
{
    if (cycles>max_cycles) max_cycles=cycles;
    if (cycles>=17000U && (MotorSampling_IsOutputRequested() ||
        MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2))) {
        timing_fault=true; MotorControl_StopAll();
    }
}
void MotorControl_Telemetry(float d[25])
{
    if (!MotorControl_AxisEnabled(MOTOR_AXIS_2)) return;
    d[16]=(float)work; d[17]=m2.velocity; d[18]=m2.target_speed;
    d[19]=m2.position; d[20]=m2.iq; d[21]=m2.id;
    d[22]=(float)max_cycles/170.0f;
    d[23]=(float)((m2.aligned ? 1U : 0U) | (MotorSampling_IsAxisOutputRequested(MOTOR_AXIS_2) ? 2U : 0U));
    d[24]=group_error;
}
