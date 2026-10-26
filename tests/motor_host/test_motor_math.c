#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "motor_math.h"
#include "stm32g4xx_ll_cordic.h"

HostCordic host_cordic;
uint32_t host_primask;
static unsigned writes,reads;
static int32_t phase;
static int32_t output[2];
void LL_CORDIC_WriteData(HostCordic *device,uint32_t data)
{
    assert(host_primask==1 && device==CORDIC);
    if(!writes++) phase=(int32_t)data;
    else {
        assert(writes==2 && data==0x7fffffffU);
        double a=(double)phase*3.14159265358979323846/2147483648.0;
        output[0]=(int32_t)(cos(a)*2147483647.0);
        output[1]=(int32_t)(sin(a)*2147483647.0);
        device->CSR|=CORDIC_CSR_RRDY;
    }
}
uint32_t LL_CORDIC_ReadData(HostCordic *device)
{
    assert(host_primask==1 && device==CORDIC && writes==2 && reads<2);
    return (uint32_t)output[reads++];
}
static void check(float angle,unsigned masked)
{
    host_primask=masked; writes=reads=0;
    host_cordic.CSR=0x55555555U; /* Unrelated HSO function/format/modulus. */
    float sn,cs;
    MotorMath_SinCos(angle,&sn,&cs);
    assert(reads==2 && writes==2 && host_primask==masked);
    assert((host_cordic.CSR & ~CORDIC_CSR_RRDY)==
        (LL_CORDIC_PRECISION_6CYCLES|LL_CORDIC_NBWRITE_2|LL_CORDIC_NBREAD_2));
    assert(fabsf(sn-sinf(angle))<2e-6f && fabsf(cs-cosf(angle))<2e-6f);
}
int main(void)
{
    uint32_t random=1;
    for(unsigned i=0;i<1000000;++i) {
        random=random*1664525U+1013904223U;
        union { uint32_t u; float f; } input={.u=random},actual,expected;
        actual.f=MotorMath_Wrap(input.f);
        expected.f=fmodf(input.f,0x1.921fb6p+2f);
        if(expected.f<0) expected.f+=0x1.921fb6p+2f;
        assert(isnan(expected.f) ? isnan(actual.f) : actual.u==expected.u);
    }
    const float boundary[]={0.0f,-0.0f,0x1p-149f,-0x1p-149f,0x1.921fb6p+2f,-0x1.921fb6p+2f,INFINITY,-INFINITY,NAN};
    for(unsigned i=0;i<sizeof(boundary)/sizeof(*boundary);++i) {
        float expected=fmodf(boundary[i],0x1.921fb6p+2f);if(expected<0)expected+=0x1.921fb6p+2f;
        float actual=MotorMath_Wrap(boundary[i]);
        assert(isnan(expected) ? isnan(actual) : actual==expected && !!signbit(actual)==!!signbit(expected));
    }
    puts("PASS: fixed-turn normalization vs independent libm: 1000000 binary32 bit patterns, subnormals, large turns, signed zero and nonfinite values");
    for(int k=-10000;k<=10000;++k) check((float)k*0.001f,(unsigned)k&1U);
    const float pi=3.14159265358979323846f;
    for(int k=-3;k<=3;++k) {
        float a=k*pi;
        check(a,0); check(nextafterf(a,INFINITY),1); check(nextafterf(a,-INFINITY),0);
    }
    puts("PASS: actual CORDIC adapter: signed Q1.31 angle/modulus, quadrant boundaries, 20001 angles, independent HSO register state and PRIMASK restoration (hardware arithmetic emulated)");
}
