#include "ui_text.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Compare against the host C library, including every truncation boundary.
 * Guards beyond capacity must survive; zero capacity accepts a null buffer. */
#define CHECK(format,...) do { \
    for(size_t cap=0;cap<40;++cap) { \
        unsigned char actual[48],expected[48]; \
        memset(actual,0xA5,sizeof(actual));memset(expected,0xA5,sizeof(expected)); \
        int a=UiText_Format(cap ? (char *)actual : NULL,cap,format,##__VA_ARGS__); \
        int b=snprintf(cap ? (char *)expected : NULL,cap,format,##__VA_ARGS__); \
        assert(a==b); \
        size_t used=cap ? ((size_t)b+1<cap ? (size_t)b+1 : cap) : 0; \
        assert(!memcmp(actual,expected,used)); \
        for(size_t guard=cap;guard<sizeof(actual);++guard) assert(actual[guard]==0xA5); \
    } \
} while(0)
int main(void)
{
    const uint32_t values[]={0,1,9,10,99,100,65535,2147483647U,2147483648U,UINT32_MAX};
    for(unsigned k=0;k<sizeof(values)/sizeof(*values);++k) {
        uint32_t value=values[k];
        int32_t number=(int32_t)value;
        CHECK("%u / %lu",(unsigned)value,(unsigned long)value);
        CHECK("%d / %ld",(int)number,(long)number);
        CHECK("0x%04X %02lX",(unsigned)value,(unsigned long)value);
        for(int width=0;width<=10;++width) {
            CHECK("%0*ld",width,(long)number);
            CHECK("%*u",width,(unsigned)value);
        }
    }
    CHECK("literal %%");
    CHECK("M%u %s %s",2U,"Speed","6STEP");
    CHECK("%s%ld.%0*ld","-",15L,2,1L);
    CHECK("%u.%03u V",3U,26U);
    CHECK("%u/12 %c %u/%u.%c",2U,'P',52U,105U,'1');
    CHECK("%s %c %u","RUN",'W',3319U);
    uint32_t seed=74;
    for(unsigned i=0;i<2000;++i) {
        seed=seed*1664525U+1013904223U;
        CHECK("%ld %lu %04X",(long)(int32_t)seed,(unsigned long)seed,(unsigned)seed);
    }
    char bad[12];
    assert(UiText_Format(bad,sizeof(bad),"before %f",1.0)==-1 && !strcmp(bad,"before "));
    puts("PASS: bounded application formatter vs snprintf: signed/unsigned 32-bit edges, INT_MIN, width/zero padding/hex/string/char, 2000 random values, capacities 0..39 and guards");
    return 0;
}
