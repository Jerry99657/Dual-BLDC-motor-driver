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
static const MotorFollowParams follow_default={.accel=40,.decel=40,.P=2,.damping=2,.response=16};
static const MotorFollowParams follow_4015_current={.accel=180,.decel=500,.P=3,.damping=0,.response=120,.fast_tracking=1,.velocity_P=0.1f};
const MotorParams MOTOR_2804 = {
    .name = "2804",
    .pole_pairs = 7,
    .encoder_direction = 1,
    .Rs = 2.3f,          .Ls = 0.00086f,       .psi_f = 0.0f,
    .bus_voltage = 12.0f,
    /* Conservative dq target limit; nameplate peak is a trip threshold,
     * not a continuous current command. RMS/DC definitions need board checks. */
    .rated_current = 0.5f,     .max_current = 2.0f,
    /* Manual-load test allowance, independent of nameplate/align/dq limits.
     * This software budget is not a measured winding-temperature model. */
    .overload_current = 0.8f,  .overload_budget_s = 3.0f,
    .rated_speed_mech = 300.0f, .max_speed_mech = 600.0f,
    .vel_P = 0.05f,      .vel_I = 0.5f,
    .vel_current_P = 0.05f, .vel_current_I = 0.5f,
    .vel_limit = 6.93f,
    .vel_filter_Tf = 0.005f,
    .pos_P = 6.0f,
    .follow_voltage = &follow_default,
    .follow_current = &follow_default,
};

/* 4015 / 24N22P, delta: 3.6-ohm terminal resistance -> 1.8-ohm star model.
 * Vendor L=1.3 mH has no stated measurement convention: provisional ONLY.
 * Current settings use phase-peak/dq units, NOT the unspecified nameplate
 * 1.4 A convention. Start sensored in voltage mode; validate PI/temperature.
 * Flux stays unknown: never inherit a learned 2804 flux on a profile change. */
const MotorParams MOTOR_4015 = {
    .name = "4015",
    .pole_pairs = 11,
    .encoder_direction = 1,
    .Rs = 1.8f, .Ls = 0.0013f, .psi_f = 0.0f,
    .bus_voltage = 24.0f,
    .rated_current = 1.2f, .max_current = 2.5f,
    .overload_current = 1.4f, .overload_budget_s = 3.0f,
    .rated_speed_mech = 104.72f, .max_speed_mech = 600.0f,
    /* Voltage-mode response trial; do not copy V gains into the A controller. */
    .vel_P = 0.1f, .vel_I = 1.0f,
    .vel_current_P = 0.05f, .vel_current_I = 0.5f,
    .vel_limit = 13.86f,
    .vel_filter_Tf = 0.005f,
    .pos_P = 6.0f,
    .follow_voltage = &follow_default,
    /* +/-10deg at 4Hz requires 110rad/s^2. Keep correction headroom in the
     * 300/600 speed-command slew; current/overload limits remain independent. */
    .follow_current = &follow_4015_current,
};

/* Main loop publishes inside the controller switch critical section; ADC
 * reads. Presets are immutable and can be shared; axis selections cannot. */
static volatile MotorProfile axis_profiles[MOTOR_AXIS_COUNT] = {
    MOTOR_PROFILE_2804, MOTOR_PROFILE_2804
};
static const MotorParams *axis_motors[2]={&MOTOR_2804,&MOTOR_2804};
static MotorParams calibrated[MOTOR_AXIS_COUNT];
static MotorCalibration records[MOTOR_AXIS_COUNT+MOTOR_CUSTOM_COUNT];
#define calibration records
#define custom (records+MOTOR_AXIS_COUNT)
static const char custom_names[][9]={"Custom 1","Custom 2","Custom 3"};
static uint8_t custom_mask;

__attribute__((noinline,noclone)) static uint8_t valid_record(const MotorCalibration *r)
{
    int p=(int8_t)r->poles_rs;
    unsigned poles=(unsigned)(p<0 ? -p : p),rs=(r->poles_rs>>8)&65535U;
    return (r->poles_rs>>24)-1U<2U && poles-1U<32U && rs>=100 && rs<=50000 &&
        r->ls_uh>=50 && r->ls_uh<=50000 && r->flux_uwb>=100 && r->flux_uwb<=200000;
}
__attribute__((noinline,noclone)) static uint8_t set_record(unsigned index,const MotorCalibration *r)
{
    if(!r || (r->poles_rs && !valid_record(r))) return 0;
    records[index]=*r;return 1;
}
__attribute__((noinline,noclone)) uint8_t MotorParams_SetCustom(unsigned slot,const MotorCalibration *r)
{
    if(slot>=MOTOR_CUSTOM_COUNT || !set_record(slot+MOTOR_AXIS_COUNT,r)) return 0;
    custom_mask=(uint8_t)((custom_mask&~(1U<<slot))|(r->poles_rs ? 1U<<slot : 0U));return 1;
}
unsigned MotorParams_CustomMask(void)
{
    return custom_mask;
}
__attribute__((noinline,noclone)) MotorProfile MotorParams_BaseProfile(MotorProfile profile)
{
    if((unsigned)profile>=MOTOR_PROFILE_CUSTOM1 && (unsigned)profile<MOTOR_PROFILE_COUNT)
        return (MotorProfile)((custom[profile-MOTOR_PROFILE_CUSTOM1].poles_rs>>24)-1U);
    return profile;
}

uint8_t MotorParams_IsCalibrated(MotorAxis axis)
{
    return (unsigned)axis<MOTOR_AXIS_COUNT && axis_motors[axis]==&calibrated[axis];
}
__attribute__((noinline,noclone,optimize("Os"))) uint8_t MotorParams_SetCalibration(MotorAxis axis,const MotorCalibration *r)
{
    if ((unsigned)axis>=MOTOR_AXIS_COUNT || !set_record((unsigned)axis,r)) return 0;
    (void)MotorParams_SelectAxis(axis,axis_profiles[axis]);
    return 1;
}

__attribute__((noinline,noclone)) const MotorParams *MotorParams_GetProfile(MotorProfile profile)
{
    MotorProfile base=MotorParams_BaseProfile(profile);
    return base==MOTOR_PROFILE_4015 ? &MOTOR_4015 : base==MOTOR_PROFILE_2804 ? &MOTOR_2804 : NULL;
}

const MotorParams *MotorParams_GetActive(void)
{
    return MotorParams_GetAxis(MOTOR_AXIS_1);
}

const MotorParams *MotorParams_GetAxis(MotorAxis axis)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT) return NULL;
    return axis_motors[axis];
}

MotorProfile MotorParams_GetAxisProfile(MotorAxis axis)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT) return MOTOR_PROFILE_COUNT;
    return axis_profiles[axis];
}

__attribute__((noinline,noclone)) uint8_t MotorParams_SelectAxis(MotorAxis axis, MotorProfile profile)
{
    if ((unsigned)axis >= MOTOR_AXIS_COUNT) return 0U;
    const MotorParams *base=MotorParams_GetProfile(profile);
    if(!base) return 0U;
    axis_profiles[axis] = profile;
    const MotorCalibration *r=&calibration[axis];
    if(profile>=MOTOR_PROFILE_CUSTOM1) r=&custom[profile-MOTOR_PROFILE_CUSTOM1];
    axis_motors[axis]=base;
    if(profile>=MOTOR_PROFILE_CUSTOM1 || r->poles_rs>>24==(unsigned)profile+1U) {
        MotorParams *m=&calibrated[axis];int p=(int8_t)r->poles_rs;
        *m=*base; m->pole_pairs=(uint8_t)(p<0 ? -p : p);
        /* Detection signs native counts; MT6701's published Angle reverses
         * those counts. Convert once here for both applied and Custom records. */
        if(axis==MOTOR_AXIS_1) m->encoder_direction=p<0 ? 1 : -1;
        m->Rs=(float)((r->poles_rs>>8)&65535U)*0.001f; m->Ls=(float)r->ls_uh*0.000001f;
        m->psi_f=(float)r->flux_uwb*0.000001f;
        if(profile>=MOTOR_PROFILE_CUSTOM1) m->name=custom_names[profile-MOTOR_PROFILE_CUSTOM1];
        axis_motors[axis]=m;
    }
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
