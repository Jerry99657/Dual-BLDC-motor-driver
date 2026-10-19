#ifndef __MOTOR_PARAMS_H
#define __MOTOR_PARAMS_H

#include <stdint.h>

/* 电机参数结构体 */
typedef struct {
    const char *name;           /* 电机标识名 */
    int pole_pairs;             /* 极对数 */

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
    float rated_speed_mech;     /* 额定机械角速度 (rad/s) */
    float max_speed_mech;       /* 最高机械角速度 (rad/s) */

    /* === 电流环 PI（d/q 轴共用） === */
    float cur_bandwidth;        /* 目标带宽 (rad/s) */
    float cur_P;                /* 比例增益 (V/A) */
    float cur_I;                /* 积分增益 (V/(A*s)) */
    float cur_limit;            /* 输出电压限幅 (V) */
    float cur_ramp;             /* 输出变化率限幅 (V/s)，0=不限幅 */

    /* === 速度环 PI === */
    float vel_P;
    float vel_I;
    float vel_limit;            /* 输出限幅 (V)，通常 = bus_voltage / sqrt(3) */
    float vel_ramp;             /* 输出变化率限幅 (V/s) */
    float vel_filter_Tf;        /* 速度 LPF 时间常数 (s) */

    /* === 位置环 P === */
    float pos_P;
    float pos_limit;            /* 输出限幅 (rad/s) */
    float pos_ramp;
} MotorParams;

/* 2804 云台电机 */
extern const MotorParams MOTOR_2804;

#endif /* __MOTOR_PARAMS_H */
