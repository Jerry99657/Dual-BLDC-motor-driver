#ifndef __JERRYFOC_H
#define __JERRYFOC_H

#include "main.h"
#include "tim.h"
#include <math.h>
#include "angle_source.h"
#include "motor_params.h"

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* ================= 低通滤波器 ================= */
typedef struct {
    float Tf;             /* 滤波时间常数 */
    float y_prev;         /* 上一次的输出值 */
} LowPassFilter;

/* ================= PID 控制器 ================= */
typedef struct {
    float P;              /* 比例系数 */
    float I;              /* 积分系数 */
    float D;              /* 微分系数 */
    float output_ramp;    /* 输出变化率限幅 */
    float limit;          /* 输出绝对值限幅 */

    float error_prev;     /* 上一次的误差 */
    float output_prev;    /* 上一次的输出 */
    float integral_prev;  /* 上一次的积分项 */
} PIDController;

/* ================= 控制模式 ================= */
typedef enum {
    JERRYFOC_MODE_TORQUE = 0,    /* 纯扭矩/电流模式 (只闭环 Iq) */
    JERRYFOC_MODE_VELOCITY = 1,  /* 速度模式 (速度外环 + Iq 内环) */
    JERRYFOC_MODE_POSITION = 2   /* 位置模式 (位置外环 + 速度内环 + Iq 最内环) */
} JerryFOC_ControlMode;

/* ================= 全局控制器对象 ================= */
extern LowPassFilter M0_Vel_Flt;
extern PIDController vel_loop_M0;

extern PIDController pos_loop_M0;

extern LowPassFilter M0_Curr_Flt;
extern PIDController id_loop_M0;
extern PIDController iq_loop_M0;

/* ================= 供协议/遥测使用的状态 ================= */
extern volatile float JerryFOC_Target_Velocity;
extern volatile float JerryFOC_Target_Position;
extern volatile float JerryFOC_Target_Iq;
extern volatile float filtered_velocity_global;

/* 纯开环诊断模式 */
extern volatile uint8_t g_openloop_debug;
extern volatile float g_openloop_angle;

/* ================= 接口函数 ================= */
void JerryFOC_playStartupSound(void);

void JerryFOC_setMode(JerryFOC_ControlMode mode);
void JerryFOC_useCurrentLoop(uint8_t enable); /* 0: 电压控制(默认), 1: PI 电流环 */
void JerryFOC_setVelocity(float target_vel);
void JerryFOC_setCurrent(float target_Iq);
void JerryFOC_alignSensor(void);
void JerryFOC_setPhaseVoltage(float Uq, float Ud, float angle_el);

/* PID 与滤波计算 */
float JerryFOC_LPF_Calc(LowPassFilter* filter, float x);
float JerryFOC_PID_Calc(PIDController* pid, float error);

/* Clarke 与 Park 变换 */
void JerryFOC_Clarke(float Ia, float Ib, float* Ialpha, float* Ibeta);
void JerryFOC_Park(float Ialpha, float Ibeta, float angle_el, float *Id, float *Iq);

void JerryFOC_setPosition(float target_pos, float limit);
void JerryFOC_setAbsolutePosition(float target_pos, float limit);

/* 状态读取 */
JerryFOC_ControlMode JerryFOC_getMode(void);
float JerryFOC_getVelocity(void);
float JerryFOC_getAngle(void);
float JerryFOC_getTargetVelocity(void);
float JerryFOC_getTargetPosition(void);
float JerryFOC_getPhaseCurrent_A(void);
float JerryFOC_getPhaseCurrent_B(void);
float JerryFOC_getIq(void);

/* 10 kHz 快速环入口 */
void JerryFOC_run(void);

/* 角度源 */
void JerryFOC_setAngleSource(AngleSource *src);
const MotorParams* JerryFOC_getMotorParams(void);
AngleSource* JerryFOC_getAngleSource(void);

#endif /* __JERRYFOC_H */
