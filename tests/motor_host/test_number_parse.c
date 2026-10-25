#include "number_parse.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t rng=0x79260415U;
static unsigned random_word(void) { rng=rng*1664525U+1013904223U; return rng; }
static void check(const char *s) {
    const char *end;
    char *native_end;
    float a=Number_Parse(s,&end),b=strtof(s,&native_end);
    uint32_t x,y; memcpy(&x,&a,4);memcpy(&y,&b,4);
    if(x!=y || end-s!=native_end-s) fprintf(stderr,"parse %s: %08x / %08x\n",s,x,y);
    assert(x==y && end-s==native_end-s);
}
int main(void) {
    const char *edges[]={"-0","-0e9999","1.000000059604644775390625",
        "1.00000005960464477539062499999999999999999999999999999999999",
        "1.00000005960464477539062500000000000000000000000000000000001",
        "1.000000178813934326171875","3.40282346638528859811704183484516925440e38",
        "3.40282356779733661637539395458142568448e38",
        "7.006492321624085354618647916449580656401309709382578858786e-46",
        "1.401298464324817070923729583289916131280261941876515771757e-45",
        "0x1.000001p0","0x1.000003p0","0x1p-150","0x1.00000001p-150",
        "0x1.ffffffp127","0x1.ffffffffffffffffffffffffffffffffffffffffffffffffffffffp-149"};
    for(unsigned i=0;i<sizeof(edges)/sizeof(edges[0]);++i) { assert(strlen(edges[i])<64);check(edges[i]); }
    char line[64];
    for(unsigned i=0;i<30000;++i) {
        unsigned n=1+random_word()%48,pos=0;
        if(random_word()&1) line[pos++]='-';
        line[pos++]=(char)('1'+random_word()%9);line[pos++]='.';
        for(unsigned j=1;j<n;++j) line[pos++]=(char)('0'+random_word()%10);
        snprintf(line+pos,sizeof(line)-pos,"e%d",(int)(random_word()%190)-125);
        check(line);
    }
    puts("PASS: exact binary32 parsing vs native strtof: ties-to-even, long significands, subnormals, signed zero, overflow midpoint and 30000 random decimal strings");
}
