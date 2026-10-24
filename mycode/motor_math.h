#ifndef MOTOR_MATH_H
#define MOTOR_MATH_H
#include "main.h"
#include <math.h>
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
