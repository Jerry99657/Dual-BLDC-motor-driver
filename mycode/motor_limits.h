#ifndef MOTOR_LIMITS_H
#define MOTOR_LIMITS_H
#include "app_storage.h"

/* Pure interpretation of published flags, with the same legacy defaults in
 * settings and control. Each caller retains its integer/float cap enforcement
 * and the control loops still read live published options, without a cache. */
static inline unsigned MotorLimits_Speed(uint32_t flags)
{
    unsigned value=(flags&APP_SPEED_LIMIT_MASK)>>APP_SPEED_LIMIT_SHIFT;
    if(!value || value>APP_SPEED_LIMIT_MAX) value=APP_SPEED_LIMIT_DEFAULT;
    return value;
}
static inline unsigned MotorLimits_Position(uint32_t flags)
{
    unsigned value=AppSettings_GetPositionSpeed(flags);
    if(!value || value>APP_SPEED_LIMIT_MAX) value=30;
    return value;
}
#endif
