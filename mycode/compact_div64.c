#include <stdint.h>

/* GCC runtime ABI, used only by UART setup and UI animation arithmetic.
 * A fixed 64-iteration restoring division avoids libgcc's large speed-tuned
 * implementation. Motor/observer arithmetic uses the M4's 32-bit divider.
 * The overflow bit preserves a 65-bit intermediate for full-range operands.
 * As with C division, a zero denominator is outside the callers' contract. */
__attribute__((used,externally_visible))
uint64_t __udivmoddi4(uint64_t numerator,uint64_t denominator,uint64_t *remainder)
{
    uint64_t quotient=numerator,rest=0;
    for(unsigned count=64;count;--count) {
        unsigned overflow=(unsigned)(rest>>63);
        rest=(rest<<1)|(quotient>>63);
        quotient<<=1;
        if(overflow || rest>=denominator) { rest-=denominator; quotient|=1; }
    }
    if(remainder) *remainder=rest;
    return quotient;
}

/* Exact binary32 -> binary64 widening. libgcc groups its float-widening
 * entry with double addition/subtraction; this project only needs widening.
 * Providing the base-PCS runtime entry avoids pulling in that whole object.
 * No ST function is replaced. Preserve signed zero, all subnormals, infinity
 * and NaN payloads, quieting signalling NaNs exactly like libgcc. */
uint64_t App_FloatToDoubleBits(uint32_t bits)
{
    uint32_t fraction=bits&0x7fffffU,exponent=(bits>>23)&255U;
    uint64_t sign=(uint64_t)(bits>>31)<<63;
    if (exponent==255U) {
        exponent=2047U;
        if (fraction) fraction|=0x400000U;
    } else if (exponent) exponent+=896U;
    else if (fraction) {
        unsigned shift=(unsigned)__builtin_clz(fraction)-8U;
        fraction=(fraction<<shift)&0x7fffffU; exponent=897U-shift;
    }
    return sign|((uint64_t)exponent<<52)|((uint64_t)fraction<<29);
}
#if defined(__arm__)
/* EABI conversion helper uses r0 -> r0/r1 even in the hard-float build. */
uint64_t __aeabi_f2d(uint32_t bits) __attribute__((alias("App_FloatToDoubleBits")));
#endif
