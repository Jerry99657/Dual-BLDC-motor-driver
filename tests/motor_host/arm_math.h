#ifndef HOST_ARM_MATH_H
#define HOST_ARM_MATH_H
#include <math.h>
static inline float arm_sin_f32(float x) { return sinf(x); }
static inline float arm_cos_f32(float x) { return cosf(x); }
#endif
