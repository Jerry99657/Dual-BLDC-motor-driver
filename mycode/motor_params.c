#include "motor_params.h"

/*
 * 2804 云台电机，参数按当前电机铭牌。
 * Rs=2.3 ohm, Ls=0.86 mH, 极对数 7, 母线 12 V
 * 电流环增益由 Ls * 250 rad/s 和 Rs * 250 rad/s 反推得到。
 */
const MotorParams MOTOR_2804 = {
    .name = "2804",
    .pole_pairs = 7,
    .Rs = 2.3f,          .Ls = 0.00086f,       .psi_f = 0.0f,
    .bus_voltage = 12.0f,
    /* Conservative dq target limit; nameplate peak is a trip threshold,
     * not a continuous current command. RMS/DC definitions need board checks. */
    .rated_current = 0.5f,     .max_current = 2.0f,
    /* Manual-load test allowance, independent of nameplate/align/dq limits.
     * This software budget is not a measured winding-temperature model. */
    .overload_current = 0.8f,  .overload_budget_s = 3.0f,
    .rated_speed_mech = 300.0f, .max_speed_mech = 600.0f,
    .cur_bandwidth = 250.0f,
    .cur_P = 0.215f,     .cur_I = 575.0f,
    .cur_limit = 12.0f,  .cur_ramp = 100000.0f,
    .vel_P = 0.05f,      .vel_I = 0.5f,
    .vel_limit = 6.93f,  .vel_ramp = 0.0f,
    .vel_filter_Tf = 0.005f,
    .pos_P = 2.0f,       .pos_limit = 30.0f,   .pos_ramp = 0.0f,
};
