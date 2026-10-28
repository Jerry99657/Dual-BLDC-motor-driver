#ifndef MOTOR_MATH_H
#define MOTOR_MATH_H
#include "main.h"
#include <math.h>
/* Positive-turn normalization with the same binary32 remainder as fmodf. */
float MotorMath_Wrap(float angle);
float MotorMath_Clamp(float value,float low,float high);
/* Both encoder paths use the same four-sample difference. Caller owns reset,
 * sample admission and period; no extra filtering or prediction is added. */
float MotorMath_WindowVelocity(float position,float history[4],uint8_t *index,
                               uint8_t *count,float period);
#if defined(CORDIC)
void MotorMath_SinCos(float angle,float *sine,float *cosine);
#else
/* Host regressions have no hardware CORDIC. */
static inline void MotorMath_SinCos(float angle,float *sine,float *cosine)
{
    *sine=sinf(angle); *cosine=cosf(angle);
}
#endif
#endif
