#include "motor_math.h"
extern uint32_t SystemCoreClock;

float MotorMath_PredictAngle(float angle,float speed,int32_t cycles)
{
    int32_t maximum=(int32_t)(SystemCoreClock/1000U);
    if (!isfinite(speed) || cycles>maximum || cycles< -maximum) return angle;
    return angle+speed*((float)cycles/(float)SystemCoreClock);
}
uint32_t MotorMath_DeadTicks(uint32_t dtg,uint32_t ckd)
{
    if (ckd>2U) return 0U;
    dtg&=255U;
    uint32_t ticks=dtg<128U ? dtg : dtg<192U ? (64U+(dtg&63U))*2U :
        dtg<224U ? (32U+(dtg&31U))*8U : (32U+(dtg&31U))*16U;
    return ticks*(1U<<ckd);
}

/* User-side hot arithmetic only; no changes to libm, HAL or ST HSO.
 * This toolchain's min/max return b for equality, including signed zeros. */
__attribute__((noinline,noclone)) float MotorMath_Min(float a,float b)
{ return isnan(a) ? b : isnan(b) || a<b ? a : b; }
__attribute__((noinline,noclone)) float MotorMath_Max(float a,float b)
{ return isnan(a) ? b : isnan(b) || a>b ? a : b; }

/* Preserve fmin/fmax NaN semantics in all three existing callers. */
__attribute__((noinline,noclone,optimize("Os")))
float MotorMath_Clamp(float value,float low,float high)
{ return MotorMath_Min(high,MotorMath_Max(low,value)); }

/* Fixed divisor 2*pi: retain exact binary32 remainder, including very large
 * multi-turn positions. Integer mantissa division avoids retaining the
 * general two-argument libm fmod/errno implementation for these three users. */
float MotorMath_Wrap(float angle)
{
    union { float f; uint32_t u; } value={.f=angle};
    unsigned sign=value.u>>31U;
    value.u&=0x7FFFFFFFU;
    if(value.u>=0x7F800000U) return NAN;
    if(value.u>=0x40C90FDBU) {
        int shift=(int)(value.u>>23U)-129;
        uint32_t mantissa=(value.u&0x7FFFFFU)|0x800000U;
        while(shift-->0) {
            if(mantissa>=0xC90FDBU) mantissa-=0xC90FDBU;
            mantissa<<=1U;
        }
        if(mantissa>=0xC90FDBU) mantissa-=0xC90FDBU;
        value.f=(float)mantissa*0x1p-21f;
    }
    if(sign) value.f=-value.f;
    return value.f<0 ? value.f+0x1.921fb6p+2f : value.f;
}
#if defined(CORDIC)
#include "stm32g4xx_ll_cordic.h"

/* STM32G431 already has the CORDIC used by ST HSO. Share one sine/cosine
 * operation instead of retaining the 2052-byte CMSIS lookup table. Every
 * transaction sets its own function, format AND modulus; HSO's previous
 * register state cannot leak into the result. Restore the caller's PRIMASK. */
void MotorMath_SinCos(float angle,float *sine,float *cosine)
{
    float turns=angle*0.159154943091895f;
    turns-=floorf(turns);
    if(turns>=0.5f) turns-=1.0f;
    int32_t phase=(int32_t)(turns*4294967296.0f);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    __HAL_RCC_CORDIC_CLK_ENABLE();
    CORDIC->CSR=LL_CORDIC_FUNCTION_COSINE|LL_CORDIC_PRECISION_6CYCLES|
        LL_CORDIC_SCALE_0|LL_CORDIC_NBWRITE_2|LL_CORDIC_NBREAD_2|
        LL_CORDIC_INSIZE_32BITS|LL_CORDIC_OUTSIZE_32BITS;
    LL_CORDIC_WriteData(CORDIC,(uint32_t)phase);
    LL_CORDIC_WriteData(CORDIC,0x7FFFFFFFU);
    while(!(CORDIC->CSR & CORDIC_CSR_RRDY)) { }
    *cosine=(float)(int32_t)LL_CORDIC_ReadData(CORDIC)*4.656612873077393e-10f;
    *sine=(float)(int32_t)LL_CORDIC_ReadData(CORDIC)*4.656612873077393e-10f;
    __set_PRIMASK(mask);
}
#endif
