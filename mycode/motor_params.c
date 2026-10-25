#include "motor_params.h"
#include <stddef.h>

/* Cold application services: prioritize Flash; official sources/flags are unchanged. */
#if defined(__arm__) && defined(__GNUC__)
#pragma GCC optimize ("Oz")
#endif

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
    .vel_current_P = 0.05f, .vel_current_I = 0.5f,
    .vel_limit = 6.93f,  .vel_ramp = 0.0f,
    .vel_filter_Tf = 0.005f,
    .pos_P = 6.0f,       .pos_limit = 30.0f,   .pos_ramp = 0.0f,
    .follow_voltage = {.accel=40, .decel=40, .P=2, .damping=2, .response=16},
    .follow_current = {.accel=40, .decel=40, .P=2, .damping=2, .response=16},
};

/* 4015 / 24N22P, delta: 3.6-ohm terminal resistance -> 1.8-ohm star model.
 * Vendor L=1.3 mH has no stated measurement convention: provisional ONLY.
 * Current settings use phase-peak/dq units, NOT the unspecified nameplate
 * 1.4 A convention. Start sensored in voltage mode; validate PI/temperature.
 * Flux stays unknown: never inherit a learned 2804 flux on a profile change. */
const MotorParams MOTOR_4015 = {
    .name = "4015",
    .pole_pairs = 11,
    .Rs = 1.8f, .Ls = 0.0013f, .psi_f = 0.0f,
    .bus_voltage = 24.0f,
    .rated_current = 1.2f, .max_current = 2.5f,
    .overload_current = 1.4f, .overload_budget_s = 3.0f,
    .rated_speed_mech = 104.72f, .max_speed_mech = 600.0f,
    .cur_bandwidth = 250.0f,
    .cur_P = 0.325f, .cur_I = 450.0f,
    .cur_limit = 24.0f, .cur_ramp = 100000.0f,
    /* Voltage-mode response trial; do not copy V gains into the A controller. */
    .vel_P = 0.1f, .vel_I = 1.0f,
    .vel_current_P = 0.05f, .vel_current_I = 0.5f,
    .vel_limit = 13.86f, .vel_ramp = 0.0f,
    .vel_filter_Tf = 0.005f,
    .pos_P = 6.0f, .pos_limit = 30.0f, .pos_ramp = 0.0f,
    .follow_voltage = {.accel=40, .decel=40, .P=2, .damping=2, .response=16},
    /* +/-10deg at 4Hz requires 110rad/s^2. Keep correction headroom in the
     * 300/600 speed-command slew; current/overload limits remain independent. */
    .follow_current = {.accel=180, .decel=500, .P=3, .damping=0, .response=120,
                       .fast_tracking=1, .velocity_P=0.1f},
};

/* Main loop publishes inside the controller switch critical section; ADC
 * reads. Presets are immutable and can be shared; axis selections cannot. */
static volatile MotorProfile axis_profiles[MOTOR_AXIS_COUNT] = {
    MOTOR_PROFILE_2804, MOTOR_PROFILE_2804
};

const MotorParams *MotorParams_GetProfile(MotorProfile profile)
{
    return profile == MOTOR_PROFILE_4015 ? &MOTOR_4015 : &MOTOR_2804;
}

const MotorParams *MotorParams_GetActive(void)
{
    return MotorParams_GetAxis(MOTOR_AXIS_1);
}

const MotorParams *MotorParams_GetAxis(MotorAxis axis)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT) return NULL;
    return MotorParams_GetProfile(axis_profiles[axis]);
}

MotorProfile MotorParams_GetAxisProfile(MotorAxis axis)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT) return MOTOR_PROFILE_COUNT;
    return axis_profiles[axis];
}

uint8_t MotorParams_SelectAxis(MotorAxis axis, MotorProfile profile)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT ||
        (unsigned)profile >= MOTOR_PROFILE_COUNT) return 0U;
    axis_profiles[axis] = profile;
    return 1U;
}

MotorProfile MotorParams_GetActiveProfile(void)
{
    return MotorParams_GetAxisProfile(MOTOR_AXIS_1);
}

void MotorParams_Select(MotorProfile profile)
{
    (void)MotorParams_SelectAxis(MOTOR_AXIS_1, profile);
}
