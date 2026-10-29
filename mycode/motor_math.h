#ifndef MOTOR_MATH_H
#define MOTOR_MATH_H
#include "main.h"
#include <math.h>
/* Positive-turn normalization with the same binary32 remainder as fmodf. */
float MotorMath_Wrap(float angle);
float MotorMath_Clamp(float value,float low,float high);
/* Signed DWT delta, modulo wrap; reject a prediction outside +/-1 ms.
 * Does not wrap the result: the caller normalizes once at its own boundary. */
float MotorMath_PredictAngle(float angle,float electrical_speed,int32_t cycles);
uint32_t MotorMath_DeadTicks(uint32_t dtg,uint32_t clock_division);
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
