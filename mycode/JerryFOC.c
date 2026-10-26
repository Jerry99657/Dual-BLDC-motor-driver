#include "JerryFOC.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "usart.h"
#include "angle_src_encoder.h"
#include "MT6701.h"
#include "motor_sampling.h"
#include "st_hso.h"
#include "motor_detect.h"
#include "protocol.h"
#include "motor_math.h"
#include "motor_control.h"
#include "motor_sixstep.h"

/* 定义放入 RAM 执行的宏（链接脚本已把 .RamFunc 放进 .data） */
#ifndef __RAM_FUNC
#define __RAM_FUNC __attribute__((section(".RamFunc")))
#endif

/* ================= DWT 微秒级延时 ================= */
static void delay_us(uint32_t us)
{
    uint32_t ticks = us * 170U; /* 170 MHz 下 1 us = 170 ticks */
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

/* ================= 大疆启动音效 ================= */
void JerryFOC_playStartupSound(void)
{
    uint16_t notes[] = {523, 659, 783, 1046};
    uint16_t durations[] = {150, 150, 150, 300};

    for (int i = 0; i < 4; i++) {
        uint32_t start_time = HAL_GetTick();
        uint32_t period_us = 1000000U / notes[i];
        uint32_t half_period_us = period_us / 2U;

        while ((HAL_GetTick() - start_time) < durations[i]) {
            JerryFOC_setPhaseVoltage(0.0f, 2.0f, 0.0f);
            delay_us(half_period_us);
            JerryFOC_setPhaseVoltage(0.0f, -2.0f, 0.0f);
            delay_us(half_period_us);
        }

        JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        HAL_Delay(20);
    }
}

/* ================= 参数与全局变量 ================= */
#define active_motor (MotorParams_GetActive())
/* Compact private storage; externally visible state and all arithmetic are unchanged. */
static struct {
    AngleSource *angle_source;
    uint8_t motor_profile_initialized;
    volatile uint8_t motor_alignment_pending;
    float g_shared_prev_Uq;
    float g_shared_prev_Ud;
    float g_shared_prev_Ualpha;
    float g_shared_prev_Ubeta;
    float id_filter_prev, debug_id_filter_prev;
    volatile uint8_t hso_feedback, hso_feedback_fault;
    StHsoReason hso_fault_reason;
    uint8_t hso_loss_recorded;
    volatile uint8_t hso_stop_inhibit;
    float sensored_velocity_reference;
    volatile JerryFOC_ControlMode current_control_mode;
} foc;


/* 纯开环诊断模式 */
volatile uint8_t g_openloop_debug = 0;
volatile float g_openloop_angle = 0.0f;
static volatile uint8_t foc_fast_loop_enabled = 1U;
#define HSO_DECEL_LIMIT 50.0f /* rad/s^2; independent of encoder deceleration. */

/* 有感速度指令规划：反转时先减速到零，再沿相反方向加速。 */
#define SENSORED_ACCEL_LIMIT 100.0f   /* rad/s^2 */
#define SENSORED_DECEL_LIMIT 150.0f   /* rad/s^2 */

/* ================= 模式控制 ================= */

/* 电流环使能标志（默认 0：电压控制模式；2804 推荐电压模式） */
volatile uint8_t flag_use_current_loop = 0;

/* ================= 角度源 ================= */
void JerryFOC_setAngleSource(AngleSource *src)
{
    foc.angle_source = src;
}

AngleSource* JerryFOC_getAngleSource(void)
{
    return foc.angle_source;
}

const MotorParams* JerryFOC_getMotorParams(void)
{
    return active_motor;
}

uint8_t JerryFOC_isDriveReady(void)
{
    return MotorControl_MotionAllowed(MOTOR_AXIS_1) && MotorSampling_IsReady() &&
           MotorSampling_IsCalibrated() && !foc.hso_feedback_fault;
}

uint8_t JerryFOC_isHSOActive(void) { return foc.hso_feedback; }
uint8_t JerryFOC_hasHSOFault(void) { return foc.hso_feedback_fault; }
const char *JerryFOC_getHSOFaultText(void)
{
    return foc.hso_loss_recorded ? StHso_ReasonText(foc.hso_fault_reason) : "--";
}
uint8_t JerryFOC_isHSOReturning(void) { return 0U; }
const char *JerryFOC_getFeedbackText(void)
{
    if (MotorControl_SixStepAllowed()) return "6STEP";
    if (foc.hso_feedback_fault) return "HSO STOPPED";
    if (MotorControl_HsoAllowed()) return foc.hso_feedback ?
        (StHso_IsStarting() ? "HSO START" : "HSO") : "HSO READY";
    return "ENCODER";
}
uint8_t JerryFOC_useHSO(uint8_t enable)
{
    if (enable) return MotorControl_HsoAllowed() && JerryFOC_startOutput();
    JerryFOC_stop(); foc.hso_feedback_fault=foc.hso_stop_inhibit=0;
    return 1U;
}

float JerryFOC_getBusVoltage(void)
{
    return MotorSampling_GetBusVoltage();
}

void JerryFOC_useCurrentLoop(uint8_t enable)
{
    enable = enable || MotorControl_HsoAllowed() ? 1U : 0U;
    if (flag_use_current_loop == enable) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* 速度环输出在电压模式下是 V，电流模式下是 A，限幅也必须切换。 */
    vel_loop_M0.limit = enable ? active_motor->rated_current : active_motor->vel_limit;
    vel_loop_M0.P = enable ? active_motor->vel_current_P : active_motor->vel_P;
    vel_loop_M0.I = enable ? active_motor->vel_current_I : active_motor->vel_I;
    JerryFOC_ResetPID(&vel_loop_M0);
    JerryFOC_ResetPID(&iq_loop_M0);
    JerryFOC_ResetPID(&id_loop_M0);
    JerryFOC_Target_Iq = 0.0f;
    flag_use_current_loop = enable;
    __set_PRIMASK(primask);
}

/* ================= 速度/电流/位置状态 ================= */
volatile float filtered_velocity_global = 0.0f;
volatile float JerryFOC_Target_Position = 0.0f;
volatile float JerryFOC_Target_Velocity = 0.0f;
volatile float JerryFOC_Target_Iq = 0.0f;
float velocity_limit_global = 30.0f;

volatile float phase_current_a = 0.0f;
volatile float phase_current_b = 0.0f;
volatile float id_current = 0.0f;
volatile float iq_current = 0.0f;

/* 速度低通滤波器，切换电机时从参数表重新加载。 */
LowPassFilter M0_Vel_Flt = { .Tf = 0.005f, .y_prev = 0.0f };
/* 电流低通滤波器（0.1 ms） */
LowPassFilter M0_Curr_Flt = { .Tf = 0.0001f, .y_prev = 0.0f };

/* 位置外环 P */
PIDController pos_loop_M0 = {
    .P = 6.0f, .I = 0.0f, .D = 0.0f,
    .output_ramp = 0.0f, .limit = 30.0f,
    .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f
};

/* 速度外环 PI（电压模式时直接输出 Uq） */
PIDController vel_loop_M0 = {
    .P = 0.05f, .I = 0.5f, .D = 0.0f,
    .output_ramp = 0.0f, .limit = 6.93f,
    .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f
};

/* 电流最内环 PI */
PIDController id_loop_M0 = {
    .P = 0.215f, .I = 575.0f, .D = 0.0f,
    .output_ramp = 100000.0f, .limit = 12.0f,
    .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f
};
PIDController iq_loop_M0 = {
    .P = 0.215f, .I = 575.0f, .D = 0.0f,
    .output_ramp = 100000.0f, .limit = 12.0f,
    .error_prev = 0.0f, .output_prev = 0.0f, .integral_prev = 0.0f
};

/* ================= 工具函数 ================= */
uint8_t JerryFOC_needsAlignment(void) { return !MotorControl_SixStepAllowed() && !MotorControl_HsoAllowed() && foc.motor_alignment_pending; }
void JerryFOC_requestAlignment(void)
{
    JerryFOC_stop();
    foc.motor_alignment_pending=!MotorSampling_IsGlobalStopped();
}
uint8_t JerryFOC_isAligning(void) { return !foc_fast_loop_enabled; }

uint8_t JerryFOC_selectMotor(MotorProfile profile)
{
    if ((unsigned)profile >= MOTOR_PROFILE_COUNT) return 0U;
    if (foc.motor_profile_initialized && profile == MotorParams_GetActiveProfile()) return 1U;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Read the source directly, before reinitializing its velocity to zero.
     * A disconnected encoder cannot certify that the rotor has stopped. */
    if (!MotorControl_HsoAllowed() && !MotorControl_SixStepAllowed() && foc.angle_source && (!foc_fast_loop_enabled || !MT6701_IsHealthy() ||
        !isfinite(foc.angle_source->getVelocity()) || fabsf(foc.angle_source->getVelocity()) > 1.0f)) {
        __set_PRIMASK(mask);
        return 0U;
    }
    JerryFOC_stop();
    MotorParams_Select(profile);
    const MotorParams *m = active_motor;
    flag_use_current_loop = 0U;
    foc.current_control_mode = JERRYFOC_MODE_TORQUE;
    JerryFOC_Target_Position = JerryFOC_Target_Velocity = JerryFOC_Target_Iq = 0.0f;
    g_openloop_debug = 0U;
    g_openloop_angle = foc.sensored_velocity_reference = 0.0f;
    foc.g_shared_prev_Uq = foc.g_shared_prev_Ud = foc.g_shared_prev_Ualpha = foc.g_shared_prev_Ubeta = 0.0f;
    foc.hso_feedback = foc.hso_feedback_fault = foc.hso_stop_inhibit = 0U;
    foc.hso_loss_recorded = 0U;
    JerryFOC_InitPID(&vel_loop_M0,m->vel_P,m->vel_I,m->vel_limit,m->vel_ramp);
    JerryFOC_InitPID(&pos_loop_M0,m->pos_P,0,m->pos_limit,m->pos_ramp);
    JerryFOC_InitPID(&id_loop_M0,m->cur_P,m->cur_I,m->cur_limit,m->cur_ramp);
    iq_loop_M0=id_loop_M0;
    velocity_limit_global = m->pos_limit;
    M0_Vel_Flt = (LowPassFilter){.Tf=m->vel_filter_Tf};
    M0_Curr_Flt.y_prev = filtered_velocity_global = id_current = iq_current = 0.0f;
    foc.id_filter_prev = foc.debug_id_filter_prev = 0.0f;
    foc.angle_source = AngleSrc_Encoder_Init(m->pole_pairs);
    StHso_Init(m);
    foc.motor_profile_initialized = 1U;
    foc.motor_alignment_pending = !MotorSampling_IsGlobalStopped();
    __set_PRIMASK(mask);
    return 1U;
}

/* Share the repeated finite-controller saturation sequence; retain its NaN
 * behavior as well (unlike replacing it with fminf/fmaxf). */
__attribute__((noinline)) static float _constrain(float amt,float low,float high)
{ return amt<low ? low : amt>high ? high : amt; }
__attribute__((noinline)) static float symmetric_limit(float amt,float limit)
{ return amt< -limit ? -limit : amt>limit ? limit : amt; }
__attribute__((noinline)) float JerryFOC_CurrentPI(PIDController *pid,float error)
{
    float integral=pid->integral_prev+pid->I*0.00005f*(error+pid->error_prev);
    pid->integral_prev=symmetric_limit(integral,pid->limit);
    pid->error_prev=error;
    return symmetric_limit(pid->P*error+pid->integral_prev,pid->limit);
}
__attribute__((noinline)) void JerryFOC_TrackCurrent(PIDController *pid,float applied,float requested)
{
    pid->integral_prev=symmetric_limit(pid->integral_prev+0.2f*(applied-requested),pid->limit);
}

__RAM_FUNC static float _normalizeAngle(float angle)
{
    /* 使用浮点->整型转换求余，替代 fmod */
    float a = angle - (int)(angle * 0.159154943091895f) * (2.0f * PI);
    return a >= 0.0f ? a : (a + 2.0f * PI);
}

__RAM_FUNC float JerryFOC_LPF_Calc(LowPassFilter* filter, float x)
{
    const float Ts = 0.001f;
    float alpha = filter->Tf / (filter->Tf + Ts);
    float y = alpha * filter->y_prev + (1.0f - alpha) * x;
    filter->y_prev = y;
    return y;
}

__RAM_FUNC float JerryFOC_PID_Calc(PIDController* pid, float error)
{
    const float Ts = 0.001f;

    float proportional = pid->P * error;

    float integral = pid->integral_prev +
        pid->I * Ts * 0.5f * (error + pid->error_prev);
    integral = symmetric_limit(integral,pid->limit);

    float derivative = pid->D * (error - pid->error_prev) / Ts;

    float output = proportional + integral + derivative;
    output = symmetric_limit(output,pid->limit);

    if (pid->output_ramp > 0.0f) {
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp) {
            output = pid->output_prev + pid->output_ramp * Ts;
        } else if (output_rate < -pid->output_ramp) {
            output = pid->output_prev - pid->output_ramp * Ts;
        }
    }

    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;

    return output;
}

/* ================= 底层 FOC 换相 ================= */
__RAM_FUNC static void setAxisPwm(MotorAxis axis, float da, float db, float dc)
{
    float dc_a = _constrain(da, 0.0f, 1.0f);
    float dc_b = _constrain(db, 0.0f, 1.0f);
    float dc_c = _constrain(dc, 0.0f, 1.0f);

    TIM_HandleTypeDef *timer = axis == MOTOR_AXIS_1 ? &htim1 : &htim8;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(timer);

    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_1, (uint32_t)(dc_a * (float)arr));
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_2, (uint32_t)(dc_b * (float)arr));
    __HAL_TIM_SET_COMPARE(timer, TIM_CHANNEL_3, (uint32_t)(dc_c * (float)arr));
}

__RAM_FUNC void JerryFOC_setAxisVoltage(MotorAxis axis, float Uq, float Ud, float angle_el,
                                       float *applied_q, float *applied_d)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT) return;
    const MotorParams *motor=MotorParams_GetAxis(axis);
    float bus_voltage = MotorSampling_GetBusVoltage();
    if (!MotorSampling_IsAxisReady(axis) || bus_voltage < 1.0f ||
        !isfinite(Uq) || !isfinite(Ud) || !isfinite(angle_el)) {
        setAxisPwm(axis,0.5f,0.5f,0.5f);
        if (applied_q) *applied_q=0.0f;
        if (applied_d) *applied_d=0.0f;
        if (axis==MOTOR_AXIS_1) {
            foc.g_shared_prev_Ualpha=foc.g_shared_prev_Ubeta=0.0f;
            foc.g_shared_prev_Ud=foc.g_shared_prev_Uq=0.0f;
        }
        return;
    }
    angle_el = _normalizeAngle(angle_el);

    /* d/q 电压矢量限幅，保留 5% 裕量给死区和母线波动。 */
    /* Measured VBUS sets duty scaling. Retain the configured motor voltage
     * ceiling when a 12 V motor is operated from a higher supply. */
    float voltage_limit = (axis==MOTOR_AXIS_1 && SixStep_IsGeneric() ? bus_voltage : fminf(bus_voltage, motor->bus_voltage)) *
                          0.548482756f; /* 0.95/sqrt(3) */
    float voltage_sq = Uq * Uq + Ud * Ud;
    float voltage_limit_sq = voltage_limit * voltage_limit;
    if (voltage_sq > voltage_limit_sq) {
        float scale = voltage_limit / sqrtf(voltage_sq);
        Uq *= scale;
        Ud *= scale;
    }

    float sin_angle,cos_angle;
    MotorMath_SinCos(angle_el,&sin_angle,&cos_angle);

    /* 逆 Park 变换 */
    float Ualpha = -Uq * sin_angle + Ud * cos_angle;
    float Ubeta  =  Uq * cos_angle + Ud * sin_angle;

    uint32_t start_cycles = DWT->CYCCNT;

    /* 逆 Clarke 变换 */
    float Ua = Ualpha;
    float Ub = -0.5f * Ualpha + 0.86602540378f * Ubeta;
    float Uc = -0.5f * Ualpha - 0.86602540378f * Ubeta;

    /* 相电压转 0~1 占空比 */
    float inv_Vbus = 1.0f / bus_voltage;
    float da = Ua * inv_Vbus + 0.5f;
    float db = Ub * inv_Vbus + 0.5f;
    float dc = Uc * inv_Vbus + 0.5f;

    /* 连续 SVPWM（Min-Max 零序注入） */
    float duty_max = da > db ? (da > dc ? da : dc) : (db > dc ? db : dc);
    float duty_min = da < db ? (da < dc ? da : dc) : (db < dc ? db : dc);
    float d_com = 0.5f - 0.5f * (duty_max + duty_min);

    da += d_com;
    db += d_com;
    dc += d_com;

    da = _constrain(da, 0.0f, 1.0f);
    db = _constrain(db, 0.0f, 1.0f);
    dc = _constrain(dc, 0.0f, 1.0f);

    /* 用最终占空比反算实际施加的 alpha/beta 电压（诊断用） */
    float duty_common = (da + db + dc) * 0.3333333333f;
    float Ua_actual = (da - duty_common) * bus_voltage;
    float Ub_actual = (db - duty_common) * bus_voltage;
    float Uc_actual = (dc - duty_common) * bus_voltage;
    float actual_alpha=Ua_actual;
    float actual_beta=(Ub_actual-Uc_actual)*0.5773502692f;
    float actual_d=actual_alpha*cos_angle+actual_beta*sin_angle;
    float actual_q=actual_beta*cos_angle-actual_alpha*sin_angle;
    if (applied_q) *applied_q=actual_q;
    if (applied_d) *applied_d=actual_d;
    if (axis==MOTOR_AXIS_1) {
        foc.g_shared_prev_Ualpha=actual_alpha; foc.g_shared_prev_Ubeta=actual_beta;
        foc.g_shared_prev_Ud=actual_d; foc.g_shared_prev_Uq=actual_q;
    }
    setAxisPwm(axis,da,db,dc);

    (void)start_cycles;
}

__RAM_FUNC void JerryFOC_setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    if (!JerryFOC_isDriveReady()) Uq=Ud=0.0f;
    JerryFOC_setAxisVoltage(MOTOR_AXIS_1,Uq,Ud,angle_el,NULL,NULL);
}

/* ================= 传感器标定 ================= */
void JerryFOC_alignStart(void)
{
    if (MotorSampling_IsGlobalStopped() || MotorControl_HsoAllowed() || MotorControl_SixStepAllowed()) return;
    if (foc.hso_feedback || foc.hso_feedback_fault) (void)JerryFOC_useHSO(0U);
    if (!JerryFOC_isDriveReady()) return;
    if (!MotorSampling_SetOutputEnabled(true)) return;
    foc.motor_alignment_pending = 0U;
    /* 标定期间暂停 ADC FOC 回调，否则 10 kHz 电流环会覆盖标定电压。 */
    foc_fast_loop_enabled = 0U;

    /* 在 D 轴施加电压，迫使转子物理 D 轴对齐到电角度 0 */
    /* At standstill Id ~= Ud/Rs. Use the continuous-current target instead
     * of the old fixed 3 V (~1.3 A on this 2.3-ohm motor). */
    float align_voltage = fminf(3.0f, active_motor->Rs * active_motor->rated_current);
    JerryFOC_setPhaseVoltage(0.0f, align_voltage, 0.0f);
}

void JerryFOC_alignFinish(void)
{
    /* 只有拿到新鲜编码器数据才允许记录零点。 */
    if (!foc_fast_loop_enabled && JerryFOC_isDriveReady() && AngleSrc_Encoder_IsSource(foc.angle_source) &&
        MT6701_IsContinuous() && MT6701_IsHealthy()) {
        AngleSrc_Encoder_Align(0.0f);
    }

    JerryFOC_stop();
    foc_fast_loop_enabled = 1U;
}

void JerryFOC_alignSensor(void)
{
    JerryFOC_alignStart();
    HAL_Delay(3000);
    JerryFOC_alignFinish();
}

float JerryFOC_getAngle(void)
{
    if (MotorControl_HsoAllowed()) return StHso_Position();
    if (foc.angle_source) {
        return foc.angle_source->getMechanicalAngle();
    }
    return 0.0f;
}

static uint8_t sensored_active(void)
{
    return (foc.angle_source && AngleSrc_Encoder_IsSource(foc.angle_source)) ? 1U : 0U;
}

static uint8_t sensored_control_ready(void)
{
    return (JerryFOC_isDriveReady() && MT6701_IsContinuous() && MT6701_IsHealthy() &&
            AngleSrc_Encoder_IsAligned()) ? 1U : 0U;
}

__attribute__((noinline,noclone))
void JerryFOC_InitPID(PIDController *pid,float p,float i,float limit,float ramp)
{
    *pid=(PIDController){.P=p,.I=i,.limit=limit,.output_ramp=ramp};
}

__attribute__((noinline)) void JerryFOC_ResetPID(PIDController *pid)
{
    pid->integral_prev = 0.0f;
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
}

void JerryFOC_stop(void)
{
    MotorDetect_Stop();
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Equal 50% PWM is a zero vector / dynamic brake, not a coast state.
     * Disconnect the bridge FIRST; keep CH4 and ADC protection alive. */
    (void)MotorSampling_SetOutputEnabled(false);
    foc.motor_alignment_pending = 0U; /* Explicit Stop cancels queued as well as active alignment. */
    if (!foc_fast_loop_enabled) {
        foc_fast_loop_enabled = 1U;
    }
    foc.hso_feedback = 0U;
    StHso_Stop();
    g_openloop_debug = 0U;
    foc.current_control_mode = JERRYFOC_MODE_TORQUE;
    JerryFOC_Target_Position = JerryFOC_Target_Velocity = JerryFOC_Target_Iq = 0.0f;
    JerryFOC_ResetPID(&vel_loop_M0);
    JerryFOC_ResetPID(&pos_loop_M0);
    JerryFOC_ResetPID(&id_loop_M0);
    JerryFOC_ResetPID(&iq_loop_M0);
    foc.sensored_velocity_reference = filtered_velocity_global;
    foc.g_shared_prev_Uq = foc.g_shared_prev_Ud = foc.g_shared_prev_Ualpha = foc.g_shared_prev_Ubeta = 0.0f;
    setAxisPwm(MOTOR_AXIS_1,0.5f,0.5f,0.5f); /* Neutral preloads, with outputs disconnected. */
    __set_PRIMASK(mask);
}

uint8_t JerryFOC_startOutput(void)
{
    if (MotorDetect_Active()) return 0U;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (MotorControl_SixStepAllowed()) { __set_PRIMASK(mask); return 0U; }
    if (!MotorSampling_ResumeGlobalStop()) { __set_PRIMASK(mask); return 0U; }
    /* Do not reconnect a neutral voltage vector to a fast coasting rotor.
     * This is a fresh-start gate, not a change to running Speed:0 ramps. */
    if (MotorControl_HsoAllowed()) {
        if (!foc_fast_loop_enabled || !MotorSampling_IsReady() || !MotorSampling_IsCalibrated() ||
            foc.current_control_mode!=JERRYFOC_MODE_VELOCITY || !StHso_Start(JerryFOC_Target_Velocity)) {
            __set_PRIMASK(mask); return 0U;
        }
        if (!foc.hso_feedback) {
            JerryFOC_useCurrentLoop(1U);
            foc.hso_feedback_fault=foc.hso_stop_inhibit=foc.hso_loss_recorded=0;
            foc.sensored_velocity_reference=M0_Vel_Flt.y_prev=filtered_velocity_global=0;
            JerryFOC_ResetPID(&iq_loop_M0); JerryFOC_ResetPID(&id_loop_M0); JerryFOC_ResetPID(&vel_loop_M0);
            if (!MotorSampling_SetOutputEnabled(true)) {
                StHso_Stop(); __set_PRIMASK(mask); return 0U;
            }
            foc.hso_feedback=1U;
            foc.motor_alignment_pending=0U;
        }
        __set_PRIMASK(mask); return 1U;
    }
    float speed = foc.angle_source ? foc.angle_source->getVelocity() : NAN;
    bool stopped = MotorSampling_IsOutputRequested() ||
                   (isfinite(speed) && fabsf(speed) <= 1.0f);
    uint8_t ok = foc_fast_loop_enabled && JerryFOC_isDriveReady() && !foc.hso_stop_inhibit &&
        stopped && (g_openloop_debug || sensored_control_ready()) && MotorSampling_SetOutputEnabled(true);
    __set_PRIMASK(mask);
    return ok;
}

static void seed_sensored_velocity_pi(void)
{
    foc.sensored_velocity_reference = filtered_velocity_global;
    float error = foc.sensored_velocity_reference - filtered_velocity_global;
    vel_loop_M0.integral_prev = symmetric_limit(
        JerryFOC_Target_Iq - vel_loop_M0.P * error,vel_loop_M0.limit);
    vel_loop_M0.error_prev = error;
    vel_loop_M0.output_prev = JerryFOC_Target_Iq;
}

static void service_hso_feedback(void)
{
    /* An intentional coast can make the observer lose phase before main has
     * cleaned up its startup/control state. Do not turn KEY1 into HSO LOST. */
    if (!foc.hso_feedback || MotorSampling_IsGlobalStopped() || StHso_IsRunning()) return;
    StHsoSnapshot snapshot; StHso_GetSnapshot(&snapshot);
    JerryFOC_stop();
    foc.hso_feedback_fault=foc.hso_stop_inhibit=foc.hso_loss_recorded=1U;
    foc.hso_fault_reason=snapshot.reason;
}
static void recover_hso_stop_for_command(void)
{
    /* Independent sensorless starts clear their fault only after quiet idle
     * admission. An encoder command can clear it after an explicit mode switch. */
    if (!MotorControl_HsoAllowed()) foc.hso_feedback_fault=foc.hso_stop_inhibit=0;
}

static float update_velocity_reference(float command)
{
    command=MotorControl_LimitSpeed(MOTOR_AXIS_1,command);
    if (!sensored_active()) {
        foc.sensored_velocity_reference = command;
        return command;
    }

    /* 正反转不能一步跨过零速。方向相反时先以减速度收敛到 0，
     * 下一控制周期再按加速度进入目标方向。 */
    float desired = command;
    float rate = SENSORED_ACCEL_LIMIT;
    float deceleration = foc.hso_feedback ? HSO_DECEL_LIMIT : SENSORED_DECEL_LIMIT;
    if (MotorControl_IsFastFollow(MOTOR_AXIS_1)) {
        foc.sensored_velocity_reference=MotorControl_SpeedReference(MOTOR_AXIS_1,
            foc.sensored_velocity_reference,command);
        return foc.sensored_velocity_reference;
    }
    if (foc.sensored_velocity_reference * command < 0.0f &&
        fabsf(foc.sensored_velocity_reference) > 1.0e-4f) {
        desired = 0.0f;
        rate = deceleration;
    } else if (fabsf(command) < fabsf(foc.sensored_velocity_reference)) {
        rate = deceleration;
    }

    const float max_step = rate * 0.001f;
    float delta = desired - foc.sensored_velocity_reference;
    if (delta > max_step) {
        delta = max_step;
    } else if (delta < -max_step) {
        delta = -max_step;
    }
    foc.sensored_velocity_reference += delta;

    if (fabsf(desired - foc.sensored_velocity_reference) < 1.0e-4f) {
        foc.sensored_velocity_reference = desired;
    }
    return foc.sensored_velocity_reference;
}

/* ================= 闭环模式设置 ================= */
void JerryFOC_setMode(JerryFOC_ControlMode mode)
{
    if (mode == JERRYFOC_MODE_POSITION && foc.hso_feedback) (void)JerryFOC_useHSO(0U);
    if (mode == foc.current_control_mode) {
        return;
    }

    if (sensored_active()) {
        if (mode == JERRYFOC_MODE_VELOCITY ||
            mode == JERRYFOC_MODE_POSITION) {
            seed_sensored_velocity_pi();
        } else {
            foc.sensored_velocity_reference = filtered_velocity_global;
            JerryFOC_ResetPID(&vel_loop_M0);
        }
        JerryFOC_ResetPID(&pos_loop_M0);
    }
    foc.current_control_mode = mode;
}

void JerryFOC_setPosition(float target_pos, float limit)
{
    if (!isfinite(target_pos)) return;
    recover_hso_stop_for_command();
    float current_abs_pos = JerryFOC_getAngle();
    float base_angle = floorf(current_abs_pos / (2.0f * PI)) * 2.0f * PI;

    JerryFOC_Target_Position = base_angle + target_pos;

    if (limit <= 0.0f) limit = active_motor->pos_limit;
    limit = fminf(limit, active_motor->max_speed_mech);
    velocity_limit_global = limit;
    pos_loop_M0.limit = limit;
}

void JerryFOC_setAbsolutePosition(float target_pos, float limit)
{
    if (!isfinite(target_pos)) return;
    recover_hso_stop_for_command();
    JerryFOC_Target_Position = target_pos;
    if (limit <= 0.0f) limit = active_motor->pos_limit;
    limit = fminf(limit, active_motor->max_speed_mech);
    velocity_limit_global = limit;
    pos_loop_M0.limit = limit;
}

void JerryFOC_setVelocity(float target_vel)
{
    if (!isfinite(target_vel)) return;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    target_vel=symmetric_limit(target_vel,active_motor->max_speed_mech);
    if (MotorControl_HsoAllowed()) {
        if (target_vel==0.0f) { JerryFOC_stop(); __set_PRIMASK(mask); return; }
        if (!StHso_ValidTarget(target_vel)) { __set_PRIMASK(mask); return; }
    }
    if (foc.current_control_mode == JERRYFOC_MODE_VELOCITY) {
        /* A pending observer loss belongs to the OLD motion. Process it first
         * so this new explicit command cannot be erased by the next ADC IRQ. */
        service_hso_feedback();
        foc.current_control_mode = JERRYFOC_MODE_VELOCITY;
        recover_hso_stop_for_command();
    }
    JerryFOC_Target_Velocity = symmetric_limit(target_vel,active_motor->max_speed_mech);
    __set_PRIMASK(mask);
}
void JerryFOC_setCurrent(float target_Iq)
{
    if (!isfinite(target_Iq)) target_Iq = 0.0f;
    if (foc.current_control_mode == JERRYFOC_MODE_TORQUE && target_Iq != 0.0f)
        recover_hso_stop_for_command();
    if (flag_use_current_loop)
        target_Iq = symmetric_limit(target_Iq,active_motor->rated_current);
    else
        target_Iq = symmetric_limit(target_Iq,active_motor->vel_limit);
    JerryFOC_Target_Iq = target_Iq;
}
JerryFOC_ControlMode JerryFOC_getMode(void) { return foc.current_control_mode; }
float JerryFOC_getVelocity(void) { return filtered_velocity_global; }
float JerryFOC_getPhaseCurrent_A(void) { return phase_current_a; }
float JerryFOC_getPhaseCurrent_B(void) { return phase_current_b; }
float JerryFOC_getIq(void) { return iq_current; }
float JerryFOC_getId(void) { return id_current; }
float JerryFOC_getElectricalAngle(void)
{
    if (MotorControl_HsoAllowed()) return StHso_IsUsable() ? StHso_Angle() : NAN;
    if (MotorControl_HsoAllowed()) return StHso_ControlAngle();
    return foc.angle_source ? foc.angle_source->getElectricalAngle() : 0.0f;
}
float JerryFOC_getTargetVelocity(void) { return JerryFOC_Target_Velocity; }
float JerryFOC_getTargetPosition(void) { return JerryFOC_Target_Position; }

/* ================= Clarke / Park ================= */
__RAM_FUNC void JerryFOC_Clarke(float Ia, float Ib, float* Ialpha, float* Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = 0.577350269f * Ia + 1.154700538f * Ib;
}

__RAM_FUNC void JerryFOC_Park(float Ialpha, float Ibeta, float angle_el, float *Id, float *Iq)
{
    float ct,st;
    MotorMath_SinCos(angle_el,&st,&ct);
    *Id = Ialpha * ct + Ibeta * st;
    *Iq = Ibeta * ct - Ialpha * st;
}

/* ================= 10 kHz 快速环 ================= */
__RAM_FUNC void JerryFOC_run(void)
{
    if (MotorControl_SixStepAllowed()) return;
    const float Ts_curr = 0.0001f;
    service_hso_feedback();
    if (!MotorSampling_IsOutputRequested()) {
        const MotorSampleFrame *sample = MotorSampling_GetFrame();
        phase_current_a = sample->m1_current[0];
        phase_current_b = sample->m1_current[1];
        if (foc.angle_source && MT6701_IsHealthy()) {
            float ia, ib, raw_d, raw_q;
            JerryFOC_Clarke(phase_current_a, phase_current_b, &ia, &ib);
            JerryFOC_Park(ia, ib, foc.angle_source->getElectricalAngle(), &raw_d, &raw_q);
            float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
            iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f-alpha) * raw_q;
            M0_Curr_Flt.y_prev = iq_current;
            id_current = alpha * foc.id_filter_prev + (1.0f-alpha) * raw_d;
            foc.id_filter_prev = id_current;
        }
        return;
    }

    /* All five current offsets are captured before rotor alignment. */
    if (!JerryFOC_isDriveReady() || foc.hso_stop_inhibit) {
        JerryFOC_Target_Iq = 0.0f;
        JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return;
    }

    /* 正常闭环要求编码器在线且已完成零点标定；
     * 纯开环诊断不需要编码器零点。 */
    if (!g_openloop_debug && !(MotorControl_HsoAllowed() ? StHso_IsRunning() : sensored_control_ready())) {
        JerryFOC_Target_Iq = 0.0f;
        JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return;
    }

    const MotorSampleFrame *sample = MotorSampling_GetFrame();
    phase_current_a = sample->m1_current[0];
    phase_current_b = sample->m1_current[1];

    float Ialpha, Ibeta;
    JerryFOC_Clarke(phase_current_a, phase_current_b, &Ialpha, &Ibeta);

    /* 纯开环诊断：固定电压 + 旋转角度 */
    if (g_openloop_debug) {
        float speed_mech = JerryFOC_Target_Velocity;
        float uq_voltage = JerryFOC_Target_Iq;

        g_openloop_angle = _normalizeAngle(
            g_openloop_angle + speed_mech *
            (float)active_motor->pole_pairs * Ts_curr);

        float raw_Id, raw_Iq;
        JerryFOC_Park(Ialpha, Ibeta, g_openloop_angle, &raw_Id, &raw_Iq);

        float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
        iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f - alpha) * raw_Iq;
        M0_Curr_Flt.y_prev = iq_current;
        id_current = alpha * foc.debug_id_filter_prev + (1.0f - alpha) * raw_Id;
        foc.debug_id_filter_prev = id_current;

        JerryFOC_setPhaseVoltage(uq_voltage, 0.0f, g_openloop_angle);
        return;
    }

    float electrical_angle = MotorControl_HsoAllowed() ? StHso_ControlAngle() :
        foc.angle_source ? foc.angle_source->getElectricalAngle() : 0.0f;

    float raw_Id, raw_Iq;
    JerryFOC_Park(Ialpha, Ibeta, electrical_angle, &raw_Id, &raw_Iq);

    float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
    iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f - alpha) * raw_Iq;
    M0_Curr_Flt.y_prev = iq_current;
    id_current = alpha * foc.id_filter_prev + (1.0f - alpha) * raw_Id;
    foc.id_filter_prev = id_current;

    float target_iq = JerryFOC_Target_Iq;
    if (!isfinite(target_iq)) target_iq = 0.0f;
    float target_id = 0.0f;
    if (MotorControl_HsoAllowed()) StHso_BlendCurrent(&target_id,&target_iq);
    float Uq = 0.0f;
    float Ud = 0.0f;

    if (!flag_use_current_loop) {
        Uq = target_iq;
    } else {
        /* Also enforce this at the fast-loop boundary: the velocity outer
         * loop and external application code can write Target_Iq directly. */
        float iq_cap=sqrtf(fmaxf(0.0f,active_motor->rated_current*active_motor->rated_current-target_id*target_id));
        target_iq = symmetric_limit(target_iq,iq_cap);
        /* HSO's dq crossfade is an applied current, not the 1 kHz speed-PI
         * request. Feeding it back here would blend the same request again
         * on each of the ten fast-loop ticks and distort startup torque. */
        if (!MotorControl_HsoAllowed()) JerryFOC_Target_Iq = target_iq;
        Uq=JerryFOC_CurrentPI(&iq_loop_M0,target_iq-iq_current);
        Ud=JerryFOC_CurrentPI(&id_loop_M0,target_id-id_current);
    }

    JerryFOC_setPhaseVoltage(Uq, Ud, electrical_angle);

    /* 矢量限幅后的实际电压回算到 PI，抑制饱和时的积分累积。 */
    if (flag_use_current_loop) {
        JerryFOC_TrackCurrent(&iq_loop_M0,foc.g_shared_prev_Uq,Uq);
        JerryFOC_TrackCurrent(&id_loop_M0,foc.g_shared_prev_Ud,Ud);
    }
}

/* ================= ADC 注入完成中断：10 kHz ================= */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) JerryFOC_FastIRQ();
}

void JerryFOC_FastIRQ(void)
{
    uint32_t control_start = DWT->CYCCNT;
    if (!MotorSampling_Capture()) return;
    /* SPI DMA 不再在完成中断里自触发；由 ADC 节拍固定为 10 kHz。 */
    MT6701_Service();
    MT6701_2_FastService();
    const MotorSampleFrame *frame = MotorSampling_GetFrame();
    if (!MotorDetect_Process(frame)) {
        if (MotorControl_HsoAllowed()) StHso_Process(frame,
            MotorSampling_IsReady() && MotorSampling_IsCalibrated(),0.0f,0.0f,false);
        if (foc_fast_loop_enabled) {
            JerryFOC_run();
        }
        MotorControl_Fast();
    }
    StHso_RecordControlCycles(DWT->CYCCNT - control_start);
    MotorControl_RecordCycles(DWT->CYCCNT - control_start);
    service_hso_feedback();
}

/* ================= TIM2 1 ms 外环 ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) JerryFOC_OuterIRQ();
}

void JerryFOC_OuterIRQ(void)
{
    const float Ts = 0.001f;

    /* 编码器多圈机械角与速度只在这里更新 */
    AngleSrc_Encoder_Update(Ts);

    uint8_t control_ready = MotorControl_HsoAllowed() ? StHso_IsRunning() : sensored_control_ready();

    /* 速度：编码器角度源直接提供机械角速度 */
    float raw_velocity = MotorControl_HsoAllowed() ? StHso_ControlVelocity() : 0.0f;
    if (!MotorControl_HsoAllowed() && foc.angle_source) {
        if (foc.angle_source->providesVelocity) raw_velocity=foc.angle_source->getVelocity();
        else {
            static float last_mechanical_angle;
            float position=foc.angle_source->getMechanicalAngle();
            raw_velocity=(position-last_mechanical_angle)/Ts; last_mechanical_angle=position;
        }
    }
    filtered_velocity_global = JerryFOC_LPF_Calc(&M0_Vel_Flt, raw_velocity);

    float current_mechanical_angle = 0.0f;
    if (foc.angle_source) {
        current_mechanical_angle = foc.angle_source->getMechanicalAngle();
    }

    MotorControl_PrepareOuter();

    if (MotorDetect_Active()) return;

    if (MotorControl_SixStepAllowed() || !MotorSampling_IsOutputRequested() ||
        (!g_openloop_debug && (!control_ready || foc.hso_stop_inhibit))) {
        /* 编码器异常时清空外环状态，防止恢复后误差一次性施加到电机。 */
        JerryFOC_Target_Iq = 0.0f;
        foc.sensored_velocity_reference = filtered_velocity_global;
        JerryFOC_ResetPID(&vel_loop_M0);
        JerryFOC_ResetPID(&pos_loop_M0);
    } else if (g_openloop_debug || (MotorControl_HsoAllowed() && StHso_IsStarting())) {
        if (!g_openloop_debug) { JerryFOC_Target_Iq=0; foc.sensored_velocity_reference=raw_velocity; JerryFOC_ResetPID(&vel_loop_M0); }
        /* 开环诊断：目标电压由协议命令直接设置，外环不介入 */
    } else if (foc.current_control_mode == JERRYFOC_MODE_POSITION) {
        float error_pos = JerryFOC_Target_Position - current_mechanical_angle;
        JerryFOC_Target_Velocity = symmetric_limit(MotorControl_PositionVelocity(
            MOTOR_AXIS_1, &pos_loop_M0, error_pos),pos_loop_M0.limit);

        float velocity_reference =
            update_velocity_reference(JerryFOC_Target_Velocity);
        JerryFOC_Target_Iq = MotorControl_VelocityControl(MOTOR_AXIS_1,&vel_loop_M0,
            velocity_reference,filtered_velocity_global);
    } else if (foc.current_control_mode == JERRYFOC_MODE_VELOCITY) {
        float velocity_reference =
            update_velocity_reference(JerryFOC_Target_Velocity);
        float error_vel = velocity_reference - filtered_velocity_global;
        JerryFOC_Target_Iq = JerryFOC_PID_Calc(&vel_loop_M0, error_vel);
    }

    MotorControl_Outer();

    /* R86 telemetry is produced in main-loop Telemetry_Service, including Detect. */
}
