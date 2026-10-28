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
typedef struct { PIDController vel,pos,q,d; } MotorControllers;

/* ================= 控制模式 ================= */
typedef enum {
    JERRYFOC_MODE_TORQUE = 0,    /* 纯扭矩/电流模式 (只闭环 Iq) */
    JERRYFOC_MODE_VELOCITY = 1,  /* 速度模式 (速度外环 + Iq 内环) */
    JERRYFOC_MODE_POSITION = 2   /* 位置模式 (位置外环 + 速度内环 + Iq 最内环) */
} JerryFOC_ControlMode;

/* ================= 全局控制器对象 ================= */
extern LowPassFilter M0_Vel_Flt;
extern MotorControllers m1_controllers;
#define vel_loop_M0 (m1_controllers.vel)
#define pos_loop_M0 (m1_controllers.pos)

extern LowPassFilter M0_Curr_Flt;
#define id_loop_M0 (m1_controllers.d)
#define iq_loop_M0 (m1_controllers.q)

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
/* Main-loop only. Existing output is stopped; a moving/unhealthy encoder or
 * ongoing alignment rejects a live switch. Sampling faults remain latched. */
uint8_t JerryFOC_selectMotor(MotorProfile profile);
void JerryFOC_ReloadMotor(void); /* Caller certified stopped and masks IRQs. */
uint8_t JerryFOC_needsAlignment(void);
void JerryFOC_requestAlignment(void);
uint8_t JerryFOC_isAligning(void);
/* Coast with all six gates disabled. All ADC trips remain active. */
void JerryFOC_stop(void);
/* Explicit new motion command only; refuses unready feedback/latched faults. */
uint8_t JerryFOC_startOutput(void);

void JerryFOC_setMode(JerryFOC_ControlMode mode);
void JerryFOC_useCurrentLoop(uint8_t enable); /* 0: 电压控制(默认), 1: PI 电流环 */
void JerryFOC_setVelocity(float target_vel);
void JerryFOC_setCurrent(float target_Iq);
void JerryFOC_alignSensor(void);
void JerryFOC_alignStart(void);
void JerryFOC_alignFinish(void);
void JerryFOC_setPhaseVoltage(float Uq, float Ud, float angle_el);
/* Shared modulator: axis-specific timer, voltage ceiling and applied dq for
 * anti-windup. M2 calls do not overwrite M1's HSO voltage history. */
void JerryFOC_setAxisVoltage(MotorAxis axis, float Uq, float Ud, float angle_el,
                            float *applied_q, float *applied_d);

/* PID 与滤波计算 */
float JerryFOC_LPF_Calc(LowPassFilter* filter, float x);
float JerryFOC_PID_Calc(PIDController* pid, float error);
/* Shared stopped-controller initialization; states remain independent per axis. */
void JerryFOC_ResetPID(PIDController *pid);
void JerryFOC_InitPID(PIDController *pid,float p,float i,float limit,float ramp);
void JerryFOC_InitControllers(MotorControllers *loops,const MotorParams *motor);
void JerryFOC_ResetControllers(MotorControllers *loops);

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
float JerryFOC_getId(void);
float JerryFOC_getElectricalAngle(void);
uint8_t JerryFOC_isDriveReady(void);
float JerryFOC_getBusVoltage(void);
/* Experimental spinning takeover; encoder remains connected as a watchdog.
 * Disabling returns to encoder with zero targets. Position uses encoder only. */
uint8_t JerryFOC_useHSO(uint8_t enable);
uint8_t JerryFOC_isHSOActive(void);
uint8_t JerryFOC_hasHSOFault(void);
const char *JerryFOC_getHSOFaultText(void); /* Loss history, cleared by next successful Use. */
const char *JerryFOC_getFeedbackText(void); /* ENCODER / HSO / TO ENCODER / STOPPED. */
uint8_t JerryFOC_isHSOReturning(void);

/* 10 kHz 快速环入口 */
void JerryFOC_run(void);
/* Application scheduling entries; HAL compatibility callbacks delegate here. */
void JerryFOC_FastIRQ(void); /* Complete ADC1/2 injected pair, 10 kHz. */
void JerryFOC_OuterIRQ(void); /* TIM2 update, 1 kHz. */

/* 角度源 */
void JerryFOC_setAngleSource(AngleSource *src);
const MotorParams* JerryFOC_getMotorParams(void);
AngleSource* JerryFOC_getAngleSource(void);

/* Shared 10 kHz current PI and applied-voltage antiwindup. */
float JerryFOC_CurrentPI(PIDController *pid,float error);
void JerryFOC_TrackCurrent(PIDController *pid,float applied,float requested);

#endif /* __JERRYFOC_H */
