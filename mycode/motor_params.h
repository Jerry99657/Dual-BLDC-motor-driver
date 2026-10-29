#ifndef __MOTOR_PARAMS_H
#define __MOTOR_PARAMS_H

#include <stdint.h>
#define MOTOR_DEFAULT_POSITION_SPEED 30.0f
#define MOTOR_CURRENT_BANDWIDTH_RAD_S 250.0f

/* 电机参数结构体 */
typedef struct {
    float accel, decel;         /* Reference acceleration limits, rad/s^2. */
    float P, damping;           /* Position correction and filtered speed damping. */
    float response;             /* Reference response, 1/s. */
    uint8_t fast_tracking;       /* Bounded position tracking; low-lag speed observer. */
    float velocity_P;           /* Maximum fast-follow speed P, A/(rad/s). */
} MotorFollowParams;

typedef struct {
    const char *name;           /* 电机标识名 */
    uint8_t pole_pairs;             /* 极对数 */
    int8_t encoder_direction;     /* +1/-1, independent of the positive pole count. */

    /* === 电气参数 === */
    float Rs;                   /* 相电阻 (ohm) */
    float Ls;                   /* 相电感 (H) */
    float psi_f;                /* 永磁体磁链 (Wb)，有感模式可填 0 */
    float bus_voltage;          /* 电机配置电压上限 (V)；实时 VBUS 由 ADC 测量 */

    /* === 额定值 === */
    float rated_current;        /* 额定值/保守 dq 电流目标上限 (A) */
    float max_current;          /* 峰值软件停机门限 (A)，不作为持续目标 */
    float overload_current;     /* 过载累计基准 (A)，不是铭牌额定值 */
    float overload_budget_s;    /* 超额 I^2 预算 = 基准电流平方 * 此秒数 */
    float rated_speed_mech;     /* Mechanical rated speed used by six-step feedforward. */
    float max_speed_mech;       /* 最高机械角速度 (rad/s) */

    /* === 电流环 PI（d/q 轴共用） === */

    /* === 速度环 PI === */
    float vel_P;
    float vel_I;
    float vel_current_P;        /* Speed PI outputs A with current loop enabled. */
    float vel_current_I;
    float vel_limit;            /* 输出限幅 (V)，通常 = bus_voltage / sqrt(3) */
    float vel_filter_Tf;        /* 速度 LPF 时间常数 (s) */

    /* === 位置环 P === */
    float pos_P;

    /* Current and voltage actuators have different dynamics. Choose using
     * the running controller's mode, not an unconfirmed settings draft. */
    const MotorFollowParams *follow_voltage, *follow_current;
} MotorParams;

typedef enum {
    MOTOR_PROFILE_2804 = 0,
    MOTOR_PROFILE_4015,
    MOTOR_PROFILE_ESC, /* Protocol selector only; not a physical parameter set. */
    MOTOR_PROFILE_CUSTOM1,
    MOTOR_PROFILE_CUSTOM2,
    MOTOR_PROFILE_CUSTOM3,
    MOTOR_PROFILE_COUNT
} MotorProfile;
#define MOTOR_CUSTOM_COUNT 3U

/* Physical bridge/encoder identity, independent of the attached motor model. */
typedef enum {
    MOTOR_AXIS_1 = 0,
    MOTOR_AXIS_2,
    MOTOR_AXIS_COUNT
} MotorAxis;

/* Equivalent-star result. Zero denotes factory parameters.
 * poles_rs low signed byte: native MT6701 raw-count direction * pole pairs,
 * not the runtime Angle direction (Angle=2pi-raw). Bits 23:8: Rs in mOhm;
 * high byte: base preset + 1. Kept unchanged for saved v6/v7 records. */
typedef struct { uint32_t poles_rs,ls_uh,flux_uwb; } MotorCalibration;
uint8_t MotorParams_SetCalibration(MotorAxis axis,const MotorCalibration *record);
uint8_t MotorParams_IsCalibrated(MotorAxis axis);
uint8_t MotorParams_SetCustom(unsigned slot,const MotorCalibration *record);
unsigned MotorParams_CustomMask(void);
MotorProfile MotorParams_BaseProfile(MotorProfile profile);

extern const MotorParams MOTOR_2804;
extern const MotorParams MOTOR_4015;
const MotorParams *MotorParams_GetProfile(MotorProfile profile);
/* GetProfile returns the immutable control/protection template. Live measured
 * electrical parameters, including Custom slots, are returned by GetAxis. */
/* Invalid axes return NULL / MOTOR_PROFILE_COUNT and cannot change either
 * axis. These functions publish configuration only: the controller must stop
 * the affected axis, reload its PI/limits and invalidate alignment itself. */
const MotorParams *MotorParams_GetAxis(MotorAxis axis);
MotorProfile MotorParams_GetAxisProfile(MotorAxis axis);
uint8_t MotorParams_SelectAxis(MotorAxis axis, MotorProfile profile);
/* Legacy single-motor interface always addresses M1, never the UI focus. */
const MotorParams *MotorParams_GetActive(void);
MotorProfile MotorParams_GetActiveProfile(void);
/* Application changes must go through JerryFOC_selectMotor(), which stops
 * output, reloads controllers and invalidates the encoder alignment. */
void MotorParams_Select(MotorProfile profile);

#endif /* __MOTOR_PARAMS_H */
