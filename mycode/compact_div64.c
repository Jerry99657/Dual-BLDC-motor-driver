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
