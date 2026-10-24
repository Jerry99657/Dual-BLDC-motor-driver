#include "motor_math.h"
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
