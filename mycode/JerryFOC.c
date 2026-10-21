#include "JerryFOC.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "usart.h"
#include "angle_src_encoder.h"
#include "MT6701.h"
#include "motor_sampling.h"
#include "st_hso.h"
#include "protocol.h"
#include "arm_math.h"
#include "motor_control.h"

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
static AngleSource *angle_source = NULL;
static uint8_t motor_profile_initialized;
static volatile uint8_t motor_alignment_pending;

/* 纯开环诊断模式 */
volatile uint8_t g_openloop_debug = 0;
volatile float g_openloop_angle = 0.0f;
static float g_shared_prev_Uq = 0.0f;
static float g_shared_prev_Ud = 0.0f;
static float g_shared_prev_Ualpha = 0.0f;
static float g_shared_prev_Ubeta = 0.0f;
static float id_filter_prev, debug_id_filter_prev;
static volatile uint8_t foc_fast_loop_enabled = 1U;
static volatile uint8_t hso_feedback, hso_feedback_fault;
static volatile float hso_blend; /* ADC writes; TIM2 reads for speed handover. */
static StHsoReason hso_fault_reason;
static uint8_t hso_loss_recorded;
static volatile uint8_t hso_returning, hso_stop_inhibit;
static volatile float hso_return_weight;
static float hso_return_angle_delta, hso_return_speed_delta;
static volatile float hso_control_angle;
/* Mechanical boundary: 45 rpm = 4.712389 rad/s, 5.25 electrical Hz at 7 pp.
 * A speed-command step alone must not change a healthy HSO feedback source. */
#define HSO_SWITCH_SPEED_RAD_S (45.0f * (2.0f * PI) / 60.0f)
#define HSO_DECEL_LIMIT 50.0f /* rad/s^2; independent of encoder deceleration. */

/* 有感速度指令规划：反转时先减速到零，再沿相反方向加速。 */
#define SENSORED_ACCEL_LIMIT 100.0f   /* rad/s^2 */
#define SENSORED_DECEL_LIMIT 150.0f   /* rad/s^2 */
static float sensored_velocity_reference = 0.0f;

/* ================= 模式控制 ================= */
static volatile JerryFOC_ControlMode current_control_mode = JERRYFOC_MODE_TORQUE;

/* 电流环使能标志（默认 0：电压控制模式；2804 推荐电压模式） */
volatile uint8_t flag_use_current_loop = 0;

/* ================= 角度源 ================= */
void JerryFOC_setAngleSource(AngleSource *src)
{
    angle_source = src;
}

AngleSource* JerryFOC_getAngleSource(void)
{
    return angle_source;
}

const MotorParams* JerryFOC_getMotorParams(void)
{
    return active_motor;
}

uint8_t JerryFOC_isDriveReady(void)
{
    return MotorControl_MotionAllowed(MOTOR_AXIS_1) && MotorSampling_IsReady() &&
           MotorSampling_IsCalibrated() && !hso_feedback_fault;
}

uint8_t JerryFOC_isHSOActive(void) { return hso_feedback; }
uint8_t JerryFOC_hasHSOFault(void) { return hso_feedback_fault; }
const char *JerryFOC_getHSOFaultText(void)
{
    return hso_loss_recorded ? StHso_ReasonText(hso_fault_reason) : "--";
}
uint8_t JerryFOC_isHSOReturning(void) { return hso_returning; }
const char *JerryFOC_getFeedbackText(void)
{
    if (hso_feedback_fault) return "STOPPED";
    if (hso_returning) return "TO ENCODER";
    return hso_feedback ? "HSO" : "ENCODER";
}

static bool encoder_feedback_healthy(void)
{
    return MotorSampling_IsReady() && MotorSampling_IsCalibrated() &&
        angle_source && AngleSrc_Encoder_IsSource(angle_source) &&
        MT6701_IsHealthy() && MT6701_IsContinuous() && AngleSrc_Encoder_IsAligned() &&
        isfinite(angle_source->getElectricalAngle()) && isfinite(angle_source->getVelocity());
}

uint8_t JerryFOC_useHSO(uint8_t enable)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (enable) {
        if (!MotorControl_HsoAllowed()) { __set_PRIMASK(mask); return 0U; }
        float mechanical_speed = encoder_feedback_healthy() ? angle_source->getVelocity() : 0.0f;
        if (!foc_fast_loop_enabled || !MotorSampling_IsOutputRequested() ||
            !JerryFOC_isDriveReady() || hso_returning || hso_stop_inhibit ||
            fabsf(mechanical_speed) <= HSO_SWITCH_SPEED_RAD_S ||
            (current_control_mode == JERRYFOC_MODE_VELOCITY &&
             (fabsf(JerryFOC_Target_Velocity) <= HSO_SWITCH_SPEED_RAD_S ||
              JerryFOC_Target_Velocity * mechanical_speed <= 0.0f)) ||
            !MT6701_IsHealthy() || !AngleSrc_Encoder_IsAligned() ||
            !AngleSrc_Encoder_IsSource(angle_source) || g_openloop_debug ||
            current_control_mode == JERRYFOC_MODE_POSITION || !StHso_CanTakeover()) {
            __set_PRIMASK(mask);
            return 0U;
        }
        if (!hso_feedback) {
            hso_control_angle = angle_source->getElectricalAngle();
            hso_blend = 0.0f;
            hso_feedback = 1U;
            hso_loss_recorded = 0U;
            hso_fault_reason = ST_HSO_REASON_NONE;
        }
    } else {
        hso_feedback = hso_feedback_fault = 0U;
        hso_returning = 0U;
        hso_return_weight = 0.0f;
        hso_blend = 0.0f;
        JerryFOC_Target_Iq = JerryFOC_Target_Velocity = 0.0f;
        g_openloop_debug = 0U;
        current_control_mode = JERRYFOC_MODE_TORQUE;
        vel_loop_M0.integral_prev = vel_loop_M0.output_prev = vel_loop_M0.error_prev = 0.0f;
        iq_loop_M0.integral_prev = iq_loop_M0.output_prev = iq_loop_M0.error_prev = 0.0f;
        id_loop_M0.integral_prev = id_loop_M0.output_prev = id_loop_M0.error_prev = 0.0f;
        JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
    }
    __set_PRIMASK(mask);
    return 1U;
}

float JerryFOC_getBusVoltage(void)
{
    return MotorSampling_GetBusVoltage();
}

void JerryFOC_useCurrentLoop(uint8_t enable)
{
    enable = enable ? 1U : 0U;
    if (flag_use_current_loop == enable) return;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* 速度环输出在电压模式下是 V，电流模式下是 A，限幅也必须切换。 */
    vel_loop_M0.limit = enable ? active_motor->rated_current : active_motor->vel_limit;
    vel_loop_M0.P = enable ? active_motor->vel_current_P : active_motor->vel_P;
    vel_loop_M0.I = enable ? active_motor->vel_current_I : active_motor->vel_I;
    vel_loop_M0.integral_prev = vel_loop_M0.output_prev = vel_loop_M0.error_prev = 0.0f;
    iq_loop_M0.integral_prev = iq_loop_M0.output_prev = iq_loop_M0.error_prev = 0.0f;
    id_loop_M0.integral_prev = id_loop_M0.output_prev = id_loop_M0.error_prev = 0.0f;
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
uint8_t JerryFOC_needsAlignment(void) { return motor_alignment_pending; }
void JerryFOC_requestAlignment(void)
{
    JerryFOC_stop();
    motor_alignment_pending=1U;
}
uint8_t JerryFOC_isAligning(void) { return !foc_fast_loop_enabled; }

uint8_t JerryFOC_selectMotor(MotorProfile profile)
{
    if ((unsigned)profile >= MOTOR_PROFILE_COUNT) return 0U;
    if (motor_profile_initialized && profile == MotorParams_GetActiveProfile()) return 1U;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Read the source directly, before reinitializing its velocity to zero.
     * A disconnected encoder cannot certify that the rotor has stopped. */
    if (angle_source && (!foc_fast_loop_enabled || !MT6701_IsHealthy() ||
        !isfinite(angle_source->getVelocity()) || fabsf(angle_source->getVelocity()) > 1.0f)) {
        __set_PRIMASK(mask);
        return 0U;
    }
    JerryFOC_stop();
    MotorParams_Select(profile);
    const MotorParams *m = active_motor;
    flag_use_current_loop = 0U;
    current_control_mode = JERRYFOC_MODE_TORQUE;
    JerryFOC_Target_Position = JerryFOC_Target_Velocity = JerryFOC_Target_Iq = 0.0f;
    g_openloop_debug = 0U;
    g_openloop_angle = sensored_velocity_reference = 0.0f;
    g_shared_prev_Uq = g_shared_prev_Ud = g_shared_prev_Ualpha = g_shared_prev_Ubeta = 0.0f;
    hso_feedback = hso_feedback_fault = hso_returning = hso_stop_inhibit = 0U;
    hso_loss_recorded = 0U;
    hso_blend = hso_return_weight = hso_control_angle = 0.0f;
    vel_loop_M0 = (PIDController){.P=m->vel_P, .I=m->vel_I,
        .output_ramp=m->vel_ramp, .limit=m->vel_limit};
    pos_loop_M0 = (PIDController){.P=m->pos_P, .limit=m->pos_limit,
        .output_ramp=m->pos_ramp};
    id_loop_M0 = iq_loop_M0 = (PIDController){.P=m->cur_P, .I=m->cur_I,
        .output_ramp=m->cur_ramp, .limit=m->cur_limit};
    velocity_limit_global = m->pos_limit;
    M0_Vel_Flt = (LowPassFilter){.Tf=m->vel_filter_Tf};
    M0_Curr_Flt.y_prev = filtered_velocity_global = id_current = iq_current = 0.0f;
    id_filter_prev = debug_id_filter_prev = 0.0f;
    angle_source = AngleSrc_Encoder_Init(m->pole_pairs);
    StHso_Init(m);
    motor_alignment_pending = motor_profile_initialized = 1U;
    __set_PRIMASK(mask);
    return 1U;
}

#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

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
    integral = _constrain(integral, -pid->limit, pid->limit);

    float derivative = pid->D * (error - pid->error_prev) / Ts;

    float output = proportional + integral + derivative;
    output = _constrain(output, -pid->limit, pid->limit);

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
            g_shared_prev_Ualpha=g_shared_prev_Ubeta=0.0f;
            g_shared_prev_Ud=g_shared_prev_Uq=0.0f;
        }
        return;
    }
    angle_el = _normalizeAngle(angle_el);

    /* d/q 电压矢量限幅，保留 5% 裕量给死区和母线波动。 */
    /* Measured VBUS sets duty scaling. Retain the configured motor voltage
     * ceiling when a 12 V motor is operated from a higher supply. */
    float voltage_limit = fminf(bus_voltage, motor->bus_voltage) *
                          0.548482756f; /* 0.95/sqrt(3) */
    float voltage_sq = Uq * Uq + Ud * Ud;
    float voltage_limit_sq = voltage_limit * voltage_limit;
    if (voltage_sq > voltage_limit_sq) {
        float scale = voltage_limit / sqrtf(voltage_sq);
        Uq *= scale;
        Ud *= scale;
    }

    float sin_angle = arm_sin_f32(angle_el);
    float cos_angle = arm_cos_f32(angle_el);

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
        g_shared_prev_Ualpha=actual_alpha; g_shared_prev_Ubeta=actual_beta;
        g_shared_prev_Ud=actual_d; g_shared_prev_Uq=actual_q;
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
    if (hso_feedback || hso_feedback_fault) (void)JerryFOC_useHSO(0U);
    if (!JerryFOC_isDriveReady()) return;
    if (!MotorSampling_SetOutputEnabled(true)) return;
    motor_alignment_pending = 0U;
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
    if (!foc_fast_loop_enabled && JerryFOC_isDriveReady() && AngleSrc_Encoder_IsSource(angle_source) &&
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
    if (angle_source) {
        return angle_source->getMechanicalAngle();
    }
    return 0.0f;
}

static uint8_t sensored_active(void)
{
    return (angle_source && AngleSrc_Encoder_IsSource(angle_source)) ? 1U : 0U;
}

static uint8_t sensored_control_ready(void)
{
    return (JerryFOC_isDriveReady() && MT6701_IsContinuous() && MT6701_IsHealthy() &&
            AngleSrc_Encoder_IsAligned()) ? 1U : 0U;
}

static void reset_pid_state(PIDController *pid)
{
    pid->integral_prev = 0.0f;
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
}

void JerryFOC_stop(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Equal 50% PWM is a zero vector / dynamic brake, not a coast state.
     * Disconnect the bridge FIRST; keep CH4 and ADC protection alive. */
    (void)MotorSampling_SetOutputEnabled(false);
    if (!foc_fast_loop_enabled) {
        foc_fast_loop_enabled = 1U;
        motor_alignment_pending = 0U; /* Explicit Stop cancels an alignment. */
    }
    hso_feedback = hso_returning = 0U;
    hso_blend = hso_return_weight = 0.0f;
    g_openloop_debug = 0U;
    current_control_mode = JERRYFOC_MODE_TORQUE;
    JerryFOC_Target_Position = JerryFOC_Target_Velocity = JerryFOC_Target_Iq = 0.0f;
    reset_pid_state(&vel_loop_M0);
    reset_pid_state(&pos_loop_M0);
    reset_pid_state(&id_loop_M0);
    reset_pid_state(&iq_loop_M0);
    sensored_velocity_reference = filtered_velocity_global;
    g_shared_prev_Uq = g_shared_prev_Ud = g_shared_prev_Ualpha = g_shared_prev_Ubeta = 0.0f;
    setAxisPwm(MOTOR_AXIS_1,0.5f,0.5f,0.5f); /* Neutral preloads, with outputs disconnected. */
    __set_PRIMASK(mask);
}

uint8_t JerryFOC_startOutput(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* Do not reconnect a neutral voltage vector to a fast coasting rotor.
     * This is a fresh-start gate, not a change to running Speed:0 ramps. */
    float speed = angle_source ? angle_source->getVelocity() : NAN;
    bool stopped = MotorSampling_IsOutputRequested() ||
                   (isfinite(speed) && fabsf(speed) <= 1.0f);
    uint8_t ok = foc_fast_loop_enabled && JerryFOC_isDriveReady() && !hso_stop_inhibit &&
        stopped && (g_openloop_debug || sensored_control_ready()) && MotorSampling_SetOutputEnabled(true);
    __set_PRIMASK(mask);
    return ok;
}

static void seed_sensored_velocity_pi(void)
{
    sensored_velocity_reference = filtered_velocity_global;
    float error = sensored_velocity_reference - filtered_velocity_global;
    vel_loop_M0.integral_prev = _constrain(
        JerryFOC_Target_Iq - vel_loop_M0.P * error,
        -vel_loop_M0.limit, vel_loop_M0.limit);
    vel_loop_M0.error_prev = error;
    vel_loop_M0.output_prev = JerryFOC_Target_Iq;
}

static void stop_after_hso_loss(StHsoReason reason)
{
    if (!hso_loss_recorded) {
        hso_fault_reason = reason;
        hso_loss_recorded = 1U;
    }
    hso_feedback = hso_returning = 0U;
    hso_blend = hso_return_weight = 0.0f;
    hso_stop_inhibit = 1U; /* Cleared by a fresh valid motion command only. */
    hso_feedback_fault = encoder_feedback_healthy() ? 0U : 1U;
    JerryFOC_Target_Iq = JerryFOC_Target_Velocity = 0.0f;
    current_control_mode = JERRYFOC_MODE_TORQUE;
    g_openloop_debug = 0U;
    sensored_velocity_reference = 0.0f;
    reset_pid_state(&vel_loop_M0);
    reset_pid_state(&iq_loop_M0);
    reset_pid_state(&id_loop_M0);
    reset_pid_state(&pos_loop_M0);
    M0_Vel_Flt.y_prev = filtered_velocity_global =
        encoder_feedback_healthy() ? angle_source->getVelocity() : 0.0f;
    JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
}

static void begin_encoder_return(void)
{
    /* Freeze only the last applied phase correction, never a rotor angle.
     * Subsequent frames follow the live encoder and remove this correction.
     * Observer loss below its speed floor cannot corrupt this transition. */
    float delta = hso_control_angle - angle_source->getElectricalAngle();
    if (delta > PI) delta -= 2.0f * PI;
    if (delta < -PI) delta += 2.0f * PI;
    hso_return_angle_delta = delta;
    hso_return_speed_delta = (StHso_Velocity() - angle_source->getVelocity()) * hso_blend;
    hso_return_weight = 1.0f;
    hso_returning = 1U;
}

static void service_hso_feedback(void)
{
    if (!hso_feedback) return;
    if (!encoder_feedback_healthy()) {
        stop_after_hso_loss(MotorSampling_IsReady() ? ST_HSO_REASON_ENCODER : ST_HSO_REASON_SAMPLE);
        return;
    }
    StHsoReason reason = ST_HSO_REASON_NONE;
    if (!StHso_IsUsable()) {
        StHsoSnapshot snapshot;
        StHso_GetSnapshot(&snapshot);
        reason = snapshot.reason;
        if (reason == ST_HSO_REASON_NONE) reason = ST_HSO_REASON_PARAM;
        if (hso_returning) {
            /* The return uses encoder + frozen correction, not observer data.
             * Acquisition and whole-loop timing faults still stop immediately. */
            if (reason == ST_HSO_REASON_TIMING || reason == ST_HSO_REASON_SAMPLE ||
                reason == ST_HSO_REASON_SEQUENCE || reason == ST_HSO_REASON_RANGE ||
                reason == ST_HSO_REASON_ENCODER || reason == ST_HSO_REASON_NUMERIC)
                stop_after_hso_loss(reason);
            return;
        }
    }
    float encoder_speed = angle_source->getVelocity();
    /* Encoder selects the measured mechanical boundary and validates safety;
     * its speed does not enter the HSO speed controller above that boundary.
     * Do not wait for the observer's low-pass speed to catch a deceleration. */
    bool below_boundary = fabsf(encoder_speed) <= HSO_SWITCH_SPEED_RAD_S;
    bool speed_floor = reason == ST_HSO_REASON_SPEED && below_boundary;
    if (reason != ST_HSO_REASON_NONE &&
        !speed_floor) {
        stop_after_hso_loss(reason);
        return;
    }
    if (!hso_returning && (below_boundary || speed_floor))
        begin_encoder_return();
}

static void recover_hso_stop_for_command(void)
{
    if ((hso_stop_inhibit || hso_feedback_fault) && encoder_feedback_healthy()) {
        hso_feedback_fault = hso_stop_inhibit = 0U;
        /* Seed from actual speed with zero output; no old HSO PI state. */
        M0_Vel_Flt.y_prev = filtered_velocity_global = angle_source->getVelocity();
        seed_sensored_velocity_pi();
    }
}

static float update_velocity_reference(float command)
{
    if (!sensored_active()) {
        sensored_velocity_reference = command;
        return command;
    }

    /* 正反转不能一步跨过零速。方向相反时先以减速度收敛到 0，
     * 下一控制周期再按加速度进入目标方向。 */
    float desired = command;
    float rate = SENSORED_ACCEL_LIMIT;
    float deceleration = hso_feedback && !hso_returning ? HSO_DECEL_LIMIT : SENSORED_DECEL_LIMIT;
    if (MotorControl_IsFastFollow(MOTOR_AXIS_1)) {
        sensored_velocity_reference=MotorControl_SpeedReference(MOTOR_AXIS_1,
            sensored_velocity_reference,command);
        return sensored_velocity_reference;
    }
    if (sensored_velocity_reference * command < 0.0f &&
        fabsf(sensored_velocity_reference) > 1.0e-4f) {
        desired = 0.0f;
        rate = deceleration;
    } else if (fabsf(command) < fabsf(sensored_velocity_reference)) {
        rate = deceleration;
    }

    const float max_step = rate * 0.001f;
    float delta = desired - sensored_velocity_reference;
    if (delta > max_step) {
        delta = max_step;
    } else if (delta < -max_step) {
        delta = -max_step;
    }
    sensored_velocity_reference += delta;

    if (fabsf(desired - sensored_velocity_reference) < 1.0e-4f) {
        sensored_velocity_reference = desired;
    }
    return sensored_velocity_reference;
}

/* ================= 闭环模式设置 ================= */
void JerryFOC_setMode(JerryFOC_ControlMode mode)
{
    if (mode == JERRYFOC_MODE_POSITION && hso_feedback) (void)JerryFOC_useHSO(0U);
    if (mode == current_control_mode) {
        return;
    }

    if (sensored_active()) {
        if (mode == JERRYFOC_MODE_VELOCITY ||
            mode == JERRYFOC_MODE_POSITION) {
            seed_sensored_velocity_pi();
        } else {
            sensored_velocity_reference = filtered_velocity_global;
            reset_pid_state(&vel_loop_M0);
        }
        reset_pid_state(&pos_loop_M0);
    }
    current_control_mode = mode;
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
    if (current_control_mode == JERRYFOC_MODE_VELOCITY) {
        /* A pending observer loss belongs to the OLD motion. Process it first
         * so this new explicit command cannot be erased by the next ADC IRQ. */
        service_hso_feedback();
        current_control_mode = JERRYFOC_MODE_VELOCITY;
        recover_hso_stop_for_command();
    }
    JerryFOC_Target_Velocity = _constrain(target_vel,
        -active_motor->max_speed_mech, active_motor->max_speed_mech);
    __set_PRIMASK(mask);
}
void JerryFOC_setCurrent(float target_Iq)
{
    if (!isfinite(target_Iq)) target_Iq = 0.0f;
    if (current_control_mode == JERRYFOC_MODE_TORQUE && target_Iq != 0.0f)
        recover_hso_stop_for_command();
    if (flag_use_current_loop)
        target_Iq = _constrain(target_Iq, -active_motor->rated_current, active_motor->rated_current);
    else
        target_Iq = _constrain(target_Iq, -active_motor->vel_limit, active_motor->vel_limit);
    JerryFOC_Target_Iq = target_Iq;
}
JerryFOC_ControlMode JerryFOC_getMode(void) { return current_control_mode; }
float JerryFOC_getVelocity(void) { return filtered_velocity_global; }
float JerryFOC_getPhaseCurrent_A(void) { return phase_current_a; }
float JerryFOC_getPhaseCurrent_B(void) { return phase_current_b; }
float JerryFOC_getIq(void) { return iq_current; }
float JerryFOC_getId(void) { return id_current; }
float JerryFOC_getElectricalAngle(void)
{
    if (hso_feedback) return hso_control_angle;
    return angle_source ? angle_source->getElectricalAngle() : 0.0f;
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
    float ct = arm_cos_f32(angle_el);
    float st = arm_sin_f32(angle_el);
    *Id = Ialpha * ct + Ibeta * st;
    *Iq = Ibeta * ct - Ialpha * st;
}

/* ================= 10 kHz 快速环 ================= */
__RAM_FUNC void JerryFOC_run(void)
{
    const float Ts_curr = 0.0001f;
    service_hso_feedback();
    if (!MotorSampling_IsOutputRequested()) {
        const MotorSampleFrame *sample = MotorSampling_GetFrame();
        phase_current_a = sample->m1_current[0];
        phase_current_b = sample->m1_current[1];
        if (angle_source && MT6701_IsHealthy()) {
            float ia, ib, raw_d, raw_q;
            JerryFOC_Clarke(phase_current_a, phase_current_b, &ia, &ib);
            JerryFOC_Park(ia, ib, angle_source->getElectricalAngle(), &raw_d, &raw_q);
            float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
            iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f-alpha) * raw_q;
            M0_Curr_Flt.y_prev = iq_current;
            id_current = alpha * id_filter_prev + (1.0f-alpha) * raw_d;
            id_filter_prev = id_current;
        }
        return;
    }

    /* All five current offsets are captured before rotor alignment. */
    if (!JerryFOC_isDriveReady() || hso_stop_inhibit) {
        JerryFOC_Target_Iq = 0.0f;
        JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return;
    }

    /* 正常闭环要求编码器在线且已完成零点标定；
     * 纯开环诊断不需要编码器零点。 */
    if (!g_openloop_debug && !sensored_control_ready()) {
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
        id_current = alpha * debug_id_filter_prev + (1.0f - alpha) * raw_Id;
        debug_id_filter_prev = id_current;

        JerryFOC_setPhaseVoltage(uq_voltage, 0.0f, g_openloop_angle);
        return;
    }

    float electrical_angle = 0.0f;
    if (angle_source) {
        electrical_angle = angle_source->getElectricalAngle();
    }
    if (hso_returning) {
        hso_return_weight -= 0.01f;
        if (hso_return_weight < 0.0f) hso_return_weight = 0.0f;
        electrical_angle = _normalizeAngle(electrical_angle + hso_return_angle_delta * hso_return_weight);
        hso_control_angle = electrical_angle;
        if (hso_return_weight == 0.0f) {
            hso_feedback = hso_returning = 0U;
            hso_blend = 0.0f;
        }
    } else if (hso_feedback) {
        /* Blend only a qualified small phase difference over 10 ms. */
        if (hso_blend < 1.0f) hso_blend += 0.01f;
        if (hso_blend > 1.0f) hso_blend = 1.0f;
        if (hso_blend >= 1.0f) {
            electrical_angle = _normalizeAngle(StHso_Angle());
        } else {
            float delta = StHso_Angle() - electrical_angle;
            if (delta > PI) delta -= 2.0f * PI;
            if (delta < -PI) delta += 2.0f * PI;
            electrical_angle = _normalizeAngle(electrical_angle + delta * hso_blend);
        }
        hso_control_angle = electrical_angle;
    }

    float raw_Id, raw_Iq;
    JerryFOC_Park(Ialpha, Ibeta, electrical_angle, &raw_Id, &raw_Iq);

    float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
    iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f - alpha) * raw_Iq;
    M0_Curr_Flt.y_prev = iq_current;
    id_current = alpha * id_filter_prev + (1.0f - alpha) * raw_Id;
    id_filter_prev = id_current;

    float target_iq = JerryFOC_Target_Iq;
    if (!isfinite(target_iq)) target_iq = 0.0f;
    float target_id = 0.0f;
    float Uq = 0.0f;
    float Ud = 0.0f;

    if (!flag_use_current_loop) {
        Uq = target_iq;
    } else {
        /* Also enforce this at the fast-loop boundary: the velocity outer
         * loop and external application code can write Target_Iq directly. */
        target_iq = _constrain(target_iq, -active_motor->rated_current, active_motor->rated_current);
        JerryFOC_Target_Iq = target_iq;
        float err_q = target_iq - iq_current;
        float integ_q = iq_loop_M0.integral_prev +
            iq_loop_M0.I * Ts_curr * 0.5f * (err_q + iq_loop_M0.error_prev);
        integ_q = _constrain(integ_q, -iq_loop_M0.limit, iq_loop_M0.limit);
        iq_loop_M0.integral_prev = integ_q;
        iq_loop_M0.error_prev = err_q;
        Uq = _constrain(iq_loop_M0.P * err_q + integ_q,
                        -iq_loop_M0.limit, iq_loop_M0.limit);

        float err_d = target_id - id_current;
        float integ_d = id_loop_M0.integral_prev +
            id_loop_M0.I * Ts_curr * 0.5f * (err_d + id_loop_M0.error_prev);
        integ_d = _constrain(integ_d, -id_loop_M0.limit, id_loop_M0.limit);
        id_loop_M0.integral_prev = integ_d;
        id_loop_M0.error_prev = err_d;
        Ud = _constrain(id_loop_M0.P * err_d + integ_d,
                        -id_loop_M0.limit, id_loop_M0.limit);
    }

    JerryFOC_setPhaseVoltage(Uq, Ud, electrical_angle);

    /* 矢量限幅后的实际电压回算到 PI，抑制饱和时的积分累积。 */
    if (flag_use_current_loop) {
        iq_loop_M0.integral_prev = _constrain(
            iq_loop_M0.integral_prev + 0.2f * (g_shared_prev_Uq - Uq),
            -iq_loop_M0.limit, iq_loop_M0.limit);
        id_loop_M0.integral_prev = _constrain(
            id_loop_M0.integral_prev + 0.2f * (g_shared_prev_Ud - Ud),
            -id_loop_M0.limit, id_loop_M0.limit);
    }
}

/* ================= ADC 注入完成中断：10 kHz ================= */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        uint32_t control_start = DWT->CYCCNT;
        if (!MotorSampling_Capture()) return;
        /* SPI DMA 不再在完成中断里自触发；由 ADC 节拍固定为 10 kHz。 */
        MT6701_Service();
        MT6701_2_FastService();
        const MotorSampleFrame *frame = MotorSampling_GetFrame();
        bool encoder_ready = angle_source && MT6701_IsHealthy() &&
            AngleSrc_Encoder_IsAligned() && AngleSrc_Encoder_IsSource(angle_source);
        if (MotorControl_HsoAllowed()) StHso_Process(frame, MotorSampling_IsReady() && MotorSampling_IsCalibrated(),
                      encoder_ready ? angle_source->getElectricalAngle() : 0.0f,
                      encoder_ready ? angle_source->getVelocity() : 0.0f, encoder_ready);
        if (foc_fast_loop_enabled) {
            JerryFOC_run();
        }
        MotorControl_Fast();
        StHso_RecordControlCycles(DWT->CYCCNT - control_start);
        MotorControl_RecordCycles(DWT->CYCCNT - control_start);
        service_hso_feedback();
    }
}

/* ================= TIM2 1 ms 外环 ================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2) {
        return;
    }

    const float Ts = 0.001f;

    /* 编码器多圈机械角与速度只在这里更新 */
    AngleSrc_Encoder_Update(Ts);

    uint8_t control_ready = sensored_control_ready();

    /* 速度：编码器角度源直接提供机械角速度 */
    float raw_velocity = 0.0f;
    if (angle_source) {
        if (angle_source->providesVelocity) {
            raw_velocity = angle_source->getVelocity();
        } else {
            static float last_mechanical_angle = 0.0f;
            static AngleSource *last_velocity_source = NULL;
            float current_mechanical_angle =
                angle_source->getMechanicalAngle();
            if (last_velocity_source == angle_source) {
                raw_velocity = (current_mechanical_angle -
                                last_mechanical_angle) / Ts;
            }
            last_mechanical_angle = current_mechanical_angle;
            last_velocity_source = angle_source;
        }
    }

    if (hso_returning) {
        raw_velocity += hso_return_speed_delta * hso_return_weight;
    } else if (hso_feedback && !hso_feedback_fault) {
        raw_velocity = hso_blend >= 1.0f ? StHso_Velocity() :
            raw_velocity + (StHso_Velocity() - raw_velocity) * hso_blend;
    }
    filtered_velocity_global = JerryFOC_LPF_Calc(&M0_Vel_Flt, raw_velocity);

    float current_mechanical_angle = 0.0f;
    if (angle_source) {
        current_mechanical_angle = angle_source->getMechanicalAngle();
    }

    MotorControl_PrepareOuter();

    if (!MotorSampling_IsOutputRequested() ||
        (!g_openloop_debug && (!control_ready || hso_stop_inhibit))) {
        /* 编码器异常时清空外环状态，防止恢复后误差一次性施加到电机。 */
        JerryFOC_Target_Iq = 0.0f;
        sensored_velocity_reference = filtered_velocity_global;
        reset_pid_state(&vel_loop_M0);
        reset_pid_state(&pos_loop_M0);
    } else if (g_openloop_debug) {
        /* 开环诊断：目标电压由协议命令直接设置，外环不介入 */
    } else if (current_control_mode == JERRYFOC_MODE_POSITION) {
        float error_pos = JerryFOC_Target_Position - current_mechanical_angle;
        JerryFOC_Target_Velocity = _constrain(MotorControl_PositionVelocity(
            MOTOR_AXIS_1, &pos_loop_M0, error_pos), -pos_loop_M0.limit, pos_loop_M0.limit);

        float velocity_reference =
            update_velocity_reference(JerryFOC_Target_Velocity);
        JerryFOC_Target_Iq = MotorControl_VelocityControl(MOTOR_AXIS_1,&vel_loop_M0,
            velocity_reference,filtered_velocity_global);
    } else if (current_control_mode == JERRYFOC_MODE_VELOCITY) {
        float velocity_reference =
            update_velocity_reference(JerryFOC_Target_Velocity);
        float error_vel = velocity_reference - filtered_velocity_global;
        JerryFOC_Target_Iq = JerryFOC_PID_Calc(&vel_loop_M0, error_vel);
    }

    MotorControl_Outer();

    /* 每 10 ms 通过 USART1 + USB CDC 发送一帧 JustFloat */
    static int print_counter = 0;
    if (++print_counter >= 10) {
        print_counter = 0;

        struct Frame {
            float fdata[25];
            unsigned char tail[4];
        } frame;

        frame.fdata[0] = JerryFOC_Target_Velocity;      /* 目标速度 */
        frame.fdata[1] = filtered_velocity_global;      /* 实际速度 */
        frame.fdata[2] = iq_current;                    /* 实际 Iq */
        frame.fdata[3] = id_current;                    /* 实际 Id */
        frame.fdata[4] = phase_current_a;               /* A 相电流 */
        frame.fdata[5] = phase_current_b;               /* B 相电流 */
        frame.fdata[6] = JerryFOC_Target_Iq;            /* 目标 Iq */
        frame.fdata[7] = sensored_velocity_reference;   /* 斜坡后速度给定 */
        frame.fdata[8] = (float)MT6701_IsHealthy();     /* 编码器健康 */
        frame.fdata[9] = (float)MT6701_GetSampleAgeCycles();
        frame.fdata[10] = (float)MT6701_GetRawAngle();  /* 14 位原始角度 */
        frame.fdata[11] = (float)MT6701_GetErrorCounter();
        frame.fdata[12] = angle_source ?
            angle_source->getElectricalAngle() : 0.0f;
        frame.fdata[13] = current_mechanical_angle;     /* 多圈机械角 */
        frame.fdata[14] = (float)MT6701_GetStatus();    /* Mg[3:0] */
        frame.fdata[15] = (float)AngleSrc_Encoder_IsAligned();
        StHsoSnapshot hso;
        StHso_GetSnapshot(&hso);
        frame.fdata[16] = hso.angle_rad;
        frame.fdata[17] = hso.velocity_rad_s;
        frame.fdata[18] = hso.angle_error_rad;
        frame.fdata[19] = hso.flux_wb;
        frame.fdata[20] = hso.reference_flux_wb;
        frame.fdata[21] = (float)hso.state;
        frame.fdata[22] = (float)hso.max_control_cycles / 170.0f;
        frame.fdata[23] = hso_feedback_fault ? -1.0f : (float)hso_feedback;
        frame.fdata[24] = (float)Protocol_GetHsoResult();
        MotorControl_Telemetry(frame.fdata);

        frame.tail[0] = 0x00;
        frame.tail[1] = 0x00;
        frame.tail[2] = 0x80;
        frame.tail[3] = 0x7f;

        Telemetry_Send((const uint8_t*)&frame, (uint16_t)sizeof(frame));
    }
}
