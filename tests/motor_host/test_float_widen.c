#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <stdio.h>
uint64_t App_FloatToDoubleBits(uint32_t bits);
static void check(uint32_t bits)
{
    float f;memcpy(&f,&bits,4);
    volatile float input=f;
    double d=(double)input;uint64_t expected;memcpy(&expected,&d,8);
    assert(App_FloatToDoubleBits(bits)==expected);
}
int main(void)
{
    for (unsigned e=0;e<256;++e) for(unsigned sign=0;sign<2;++sign) {
        uint32_t b=(sign<<31)|(e<<23);
        check(b);check(b|1);check(b|0x3fffff);check(b|0x400000);check(b|0x7fffff);
    }
    uint32_t random=19870217;
    for(unsigned i=0;i<1000000;++i) { random=random*1664525U+1013904223U;check(random); }
    puts("PASS: compact float widening vs independent binary64 conversion; signed zero, subnormals, exponents, NaN payloads and 1M random patterns");
}
