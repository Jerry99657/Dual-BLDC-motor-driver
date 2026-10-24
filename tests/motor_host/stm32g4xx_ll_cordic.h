#ifndef HOST_LL_CORDIC_H
#define HOST_LL_CORDIC_H
#include "main.h"
#define LL_CORDIC_FUNCTION_COSINE 0U
#define LL_CORDIC_PRECISION_6CYCLES (6U<<4)
#define LL_CORDIC_SCALE_0 0U
#define LL_CORDIC_NBWRITE_2 (1U<<21)
#define LL_CORDIC_NBREAD_2 (1U<<19)
#define LL_CORDIC_INSIZE_32BITS 0U
#define LL_CORDIC_OUTSIZE_32BITS 0U
void LL_CORDIC_WriteData(HostCordic *device,uint32_t data);
uint32_t LL_CORDIC_ReadData(HostCordic *device);
#endif
