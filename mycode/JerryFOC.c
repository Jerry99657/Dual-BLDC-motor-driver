#include "JerryFOC.h"
#include <stdint.h>
#include <stdio.h>
#include "adc.h"
#include "usart.h"
#include "angle_src_encoder.h"
#include "MT6701.h"
#include "protocol.h"
#include "arm_math.h"

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
static const MotorParams *active_motor = &MOTOR_2804;
static AngleSource *angle_source = NULL;

/* 纯开环诊断模式 */
volatile uint8_t g_openloop_debug = 0;
volatile float g_openloop_angle = 0.0f;
static float g_shared_prev_Uq = 0.0f;
static float g_shared_prev_Ud = 0.0f;
static float g_shared_prev_Ualpha = 0.0f;
static float g_shared_prev_Ubeta = 0.0f;
static volatile uint8_t foc_fast_loop_enabled = 1U;

/* 有感速度指令规划：反转时先减速到零，再沿相反方向加速。 */
#define SENSORED_ACCEL_LIMIT 100.0f   /* rad/s^2 */
#define SENSORED_DECEL_LIMIT 150.0f   /* rad/s^2 */
static float sensored_velocity_reference = 0.0f;

/* ================= 电流采样硬件常数 ================= */
#define SHUNT_RESISTOR 0.01f
#define AMP_GAIN 50.0f
#define VOLTS_TO_AMPS (1.0f / SHUNT_RESISTOR / AMP_GAIN) /* 2.0 A/V */
#define ADC_TO_VOLTS (3.3f / 4095.0f)
#define CURRENT_OFFSET_SAMPLES 2000U

static float adc_offset_count_a = 2047.5f;
static float adc_offset_count_b = 2047.5f;
static uint32_t adc_offset_accum_a = 0U;
static uint32_t adc_offset_accum_b = 0U;
static uint32_t adc_offset_samples = 0U;

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

void JerryFOC_useCurrentLoop(uint8_t enable)
{
    flag_use_current_loop = enable;
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

/* 速度低通滤波器（5 ms，对应 MOTOR_2804.vel_filter_Tf） */
LowPassFilter M0_Vel_Flt = { .Tf = 0.005f, .y_prev = 0.0f };
/* 电流低通滤波器（0.1 ms） */
LowPassFilter M0_Curr_Flt = { .Tf = 0.0001f, .y_prev = 0.0f };

/* 位置外环 P */
PIDController pos_loop_M0 = {
    .P = 2.0f, .I = 0.0f, .D = 0.0f,
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
__RAM_FUNC static void setPwm(float da, float db, float dc)
{
    float dc_a = _constrain(da, 0.0f, 1.0f);
    float dc_b = _constrain(db, 0.0f, 1.0f);
    float dc_c = _constrain(dc, 0.0f, 1.0f);

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(dc_a * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(dc_b * (float)arr));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(dc_c * (float)arr));
}

__RAM_FUNC void JerryFOC_setPhaseVoltage(float Uq, float Ud, float angle_el)
{
    angle_el = _normalizeAngle(angle_el);

    /* d/q 电压矢量限幅，保留 5% 裕量给死区和母线波动。 */
    float voltage_limit = active_motor->bus_voltage * 0.548482756f; /* 0.95/sqrt(3) */
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
    float inv_Vbus = 1.0f / active_motor->bus_voltage;
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
    float Ua_actual = (da - duty_common) * active_motor->bus_voltage;
    float Ub_actual = (db - duty_common) * active_motor->bus_voltage;
    float Uc_actual = (dc - duty_common) * active_motor->bus_voltage;
    g_shared_prev_Ualpha = Ua_actual;
    g_shared_prev_Ubeta = (Ub_actual - Uc_actual) * 0.5773502692f;
    g_shared_prev_Ud = g_shared_prev_Ualpha * cos_angle +
                       g_shared_prev_Ubeta * sin_angle;
    g_shared_prev_Uq = g_shared_prev_Ubeta * cos_angle -
                       g_shared_prev_Ualpha * sin_angle;

    setPwm(da, db, dc);

    (void)start_cycles;
}

/* ================= 传感器标定 ================= */
void JerryFOC_alignSensor(void)
{
    /* 阻塞式标定期间暂停 ADC FOC 回调，否则 10 kHz 电流环会覆盖标定电压。 */
    foc_fast_loop_enabled = 0U;

    /* 在 D 轴施加电压，迫使转子物理 D 轴对齐到电角度 0 */
    JerryFOC_setPhaseVoltage(0.0f, 3.0f, 0.0f);
    HAL_Delay(3000);

    /* 只有拿到新鲜编码器数据才允许记录零点。 */
    if (AngleSrc_Encoder_IsSource(angle_source) &&
        MT6701_IsContinuous() && MT6701_IsHealthy()) {
        AngleSrc_Encoder_Align(0.0f);
    }

    JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
    foc_fast_loop_enabled = 1U;
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
    return (MT6701_IsContinuous() && MT6701_IsHealthy() &&
            AngleSrc_Encoder_IsAligned()) ? 1U : 0U;
}

static void reset_pid_state(PIDController *pid)
{
    pid->integral_prev = 0.0f;
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
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
    if (sensored_velocity_reference * command < 0.0f &&
        fabsf(sensored_velocity_reference) > 1.0e-4f) {
        desired = 0.0f;
        rate = SENSORED_DECEL_LIMIT;
    } else if (fabsf(command) < fabsf(sensored_velocity_reference)) {
        rate = SENSORED_DECEL_LIMIT;
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
    float current_abs_pos = JerryFOC_getAngle();
    float base_angle = floorf(current_abs_pos / (2.0f * PI)) * 2.0f * PI;

    JerryFOC_Target_Position = base_angle + target_pos;

    if (limit <= 0.0f) {
        limit = 30.0f;
    }
    velocity_limit_global = limit;
    pos_loop_M0.limit = limit;
}

void JerryFOC_setAbsolutePosition(float target_pos, float limit)
{
    JerryFOC_Target_Position = target_pos;
    if (limit <= 0.0f) {
        limit = 30.0f;
    }
    velocity_limit_global = limit;
    pos_loop_M0.limit = limit;
}

void JerryFOC_setVelocity(float target_vel) { JerryFOC_Target_Velocity = target_vel; }
void JerryFOC_setCurrent(float target_Iq) { JerryFOC_Target_Iq = target_Iq; }
JerryFOC_ControlMode JerryFOC_getMode(void) { return current_control_mode; }
float JerryFOC_getVelocity(void) { return filtered_velocity_global; }
float JerryFOC_getPhaseCurrent_A(void) { return phase_current_a; }
float JerryFOC_getPhaseCurrent_B(void) { return phase_current_b; }
float JerryFOC_getIq(void) { return iq_current; }
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

    /* 上电后在零输出状态采集两路电流的真实零点 */
    if (adc_offset_samples < CURRENT_OFFSET_SAMPLES) {
        adc_offset_accum_a += ADC1->JDR1;
        adc_offset_accum_b += ADC2->JDR1;
        adc_offset_samples++;
        if (adc_offset_samples == CURRENT_OFFSET_SAMPLES) {
            adc_offset_count_a =
                (float)adc_offset_accum_a / (float)CURRENT_OFFSET_SAMPLES;
            adc_offset_count_b =
                (float)adc_offset_accum_b / (float)CURRENT_OFFSET_SAMPLES;
            JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        }
        return;
    }

    /* 正常闭环要求编码器在线且已完成零点标定；
     * 纯开环诊断不需要编码器零点。 */
    if (!g_openloop_debug && !sensored_control_ready()) {
        JerryFOC_Target_Iq = 0.0f;
        JerryFOC_setPhaseVoltage(0.0f, 0.0f, 0.0f);
        return;
    }

    float current_leg1 = ((float)ADC1->JDR1 - adc_offset_count_a) *
                         ADC_TO_VOLTS * VOLTS_TO_AMPS;
    float current_leg2 = ((float)ADC2->JDR1 - adc_offset_count_b) *
                         ADC_TO_VOLTS * VOLTS_TO_AMPS;

    phase_current_a = current_leg1;
    phase_current_b = current_leg2;

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
        static float debug_id_prev = 0.0f;
        id_current = alpha * debug_id_prev + (1.0f - alpha) * raw_Id;
        debug_id_prev = id_current;

        JerryFOC_setPhaseVoltage(uq_voltage, 0.0f, g_openloop_angle);
        return;
    }

    float electrical_angle = 0.0f;
    if (angle_source) {
        electrical_angle = angle_source->getElectricalAngle();
    }

    float raw_Id, raw_Iq;
    JerryFOC_Park(Ialpha, Ibeta, electrical_angle, &raw_Id, &raw_Iq);

    float alpha = M0_Curr_Flt.Tf / (M0_Curr_Flt.Tf + Ts_curr);
    iq_current = alpha * M0_Curr_Flt.y_prev + (1.0f - alpha) * raw_Iq;
    M0_Curr_Flt.y_prev = iq_current;
    static float id_prev = 0.0f;
    id_current = alpha * id_prev + (1.0f - alpha) * raw_Id;
    id_prev = id_current;

    float target_iq = JerryFOC_Target_Iq;
    float target_id = 0.0f;
    float Uq = 0.0f;
    float Ud = 0.0f;

    if (!flag_use_current_loop) {
        Uq = target_iq;
    } else {
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
        /* SPI DMA 不再在完成中断里自触发；由 ADC 节拍固定为 10 kHz。 */
        MT6701_Service();
        if (foc_fast_loop_enabled) {
            JerryFOC_run();
        }
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

    filtered_velocity_global = JerryFOC_LPF_Calc(&M0_Vel_Flt, raw_velocity);

    float current_mechanical_angle = 0.0f;
    if (angle_source) {
        current_mechanical_angle = angle_source->getMechanicalAngle();
    }

    if (!g_openloop_debug && !control_ready) {
        /* 编码器异常时清空外环状态，防止恢复后误差一次性施加到电机。 */
        JerryFOC_Target_Iq = 0.0f;
        sensored_velocity_reference = filtered_velocity_global;
        reset_pid_state(&vel_loop_M0);
        reset_pid_state(&pos_loop_M0);
    } else if (g_openloop_debug) {
        /* 开环诊断：目标电压由协议命令直接设置，外环不介入 */
    } else if (current_control_mode == JERRYFOC_MODE_POSITION) {
        float error_pos = JerryFOC_Target_Position - current_mechanical_angle;
        JerryFOC_Target_Velocity = JerryFOC_PID_Calc(&pos_loop_M0, error_pos);

        float velocity_reference =
            update_velocity_reference(JerryFOC_Target_Velocity);
        float error_vel = velocity_reference - filtered_velocity_global;
        JerryFOC_Target_Iq = JerryFOC_PID_Calc(&vel_loop_M0, error_vel);
    } else if (current_control_mode == JERRYFOC_MODE_VELOCITY) {
        float velocity_reference =
            update_velocity_reference(JerryFOC_Target_Velocity);
        float error_vel = velocity_reference - filtered_velocity_global;
        JerryFOC_Target_Iq = JerryFOC_PID_Calc(&vel_loop_M0, error_vel);
    }

    /* 每 10 ms 通过 USART1 + USB CDC 发送一帧 JustFloat */
    static int print_counter = 0;
    if (++print_counter >= 10) {
        print_counter = 0;

        struct Frame {
            float fdata[16];
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

        frame.tail[0] = 0x00;
        frame.tail[1] = 0x00;
        frame.tail[2] = 0x80;
        frame.tail[3] = 0x7f;

        Telemetry_Send((const uint8_t*)&frame, (uint16_t)sizeof(frame));
    }
}
