#include <stdint.h>
#include <assert.h>
#include <stdio.h>
#include <stddef.h>
uint64_t __udivmoddi4(uint64_t,uint64_t,uint64_t *);
static uint64_t random64(void) {
    static uint64_t x=UINT64_C(0x16439872feba7362);
    x^=x<<13; x^=x>>7; x^=x<<17; return x;
}
static void check(uint64_t a,uint64_t b) {
    uint64_t r,q=__udivmoddi4(a,b,&r);
    assert(q==a/b && r==a%b && r<b);
    assert(__udivmoddi4(a,b,NULL)==q);
}
int main(void) {
    const uint64_t edges[]={0,1,2,UINT32_MAX,UINT64_C(1)<<32,
        (UINT64_C(1)<<63)-1,UINT64_C(1)<<63,UINT64_MAX-1,UINT64_MAX};
    for(unsigned i=0;i<sizeof(edges)/sizeof(*edges);++i)
        for(unsigned j=1;j<sizeof(edges)/sizeof(*edges);++j) check(edges[i],edges[j]);
    for(unsigned i=0;i<100000;++i) check(random64(),random64()|1U);
    puts("PASS: compact GCC 64-bit division: full-range edges, 100000 independent native-division comparisons, remainder and null output");
}
