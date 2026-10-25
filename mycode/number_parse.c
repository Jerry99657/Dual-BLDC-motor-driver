#include "number_parse.h"
#include <stdint.h>
#include <stdbool.h>

/* A command has at most 63 characters. 320 bits cover every significand and
 * power of five which can round to a finite nonzero binary32. Compare shifted
 * integers virtually so even extreme exponents never require a large stack. */
typedef struct { uint32_t w[10]; } Integer;
static void multiply(Integer *a,uint32_t m,uint32_t add) {
    uint64_t carry=add;
    for(unsigned i=0;i<10;++i) { carry+=(uint64_t)a->w[i]*m; a->w[i]=(uint32_t)carry; carry>>=32; }
}
static int length(const Integer *a) {
    for(int i=9;i>=0;--i) if(a->w[i]) return i*32+32-__builtin_clz(a->w[i]);
    return 0;
}
static int compare(const Integer *n,const Integer *den,int exponent,uint32_t c,int e) {
    Integer rhs=*den; multiply(&rhs,c,0);
    int nl=length(n),rl=length(&rhs),top=nl+exponent;
    if(top!=rl+e) return top>rl+e ? 1 : -1;
    int bottom=exponent<e ? exponent : e;
    for(int bit=top-1;bit>=bottom;--bit) {
        int x=bit-exponent,y=bit-e;
        unsigned a=x>=0 && x<nl ? (n->w[x/32]>>(x%32))&1U : 0;
        unsigned b=y>=0 && y<rl ? (rhs.w[y/32]>>(y%32))&1U : 0;
        if(a!=b) return a>b ? 1 : -1;
    }
    return 0;
}
static int digit(char c,bool hex) {
    if(c>='0' && c<='9') return c-'0';
    c|=32;
    return hex && c>='a' && c<='f' ? c-'a'+10 : -1;
}
static int float_compare(const Integer *n,const Integer *den,int exponent,uint32_t bits,bool midpoint) {
    unsigned field=bits>>23;
    uint32_t c=(bits&0x7fffffU)|(field ? 0x800000U : 0);
    int e=field ? (int)field-150 : -149;
    if(midpoint) { c=2*c+1; --e; }
    return compare(n,den,exponent,c,e);
}
__attribute__((noinline)) float Number_Parse(const char *text,const char **end) {
    const char *p=text;
    while(*p==' ' || (*p>='\t' && *p<='\r')) ++p;
    bool negative=*p=='-';
    if(*p=='-' || *p=='+') ++p;
    bool hex=p[0]=='0' && (p[1]=='x' || p[1]=='X');
    if(hex) p+=2;
    Integer n={{0}},den={{1}};
    unsigned digits=0,significant=0;
    int exponent=0;
    bool dot=false;
    for(;;) {
        int d=digit(*p,hex);
        if(d>=0) {
            multiply(&n,hex ? 16 : 10,(unsigned)d); ++digits; ++p;
            if(d || significant) ++significant;
            if(dot) exponent-=hex ? 4 : 1;
        } else if(*p=='.' && !dot) { dot=true; ++p; }
        else break;
    }
    if(!digits) { *end=text; return 0; }
    if((*p|32)==(hex ? 'p' : 'e')) {
        const char *mark=p++;
        bool minus=*p=='-';
        if(*p=='-' || *p=='+') ++p;
        const char *first=p;
        int extra=0;
        while(*p>='0' && *p<='9') { if(extra<10000) extra=extra*10+*p-'0'; ++p; }
        if(p==first) p=mark;
        else exponent+=minus ? -extra : extra;
    }
    *end=p;
    union { uint32_t bits; float f; } value={.bits=0};
    if(significant) {
        int order=exponent+(int)significant*(hex ? 4 : 1);
        if(order>(hex ? 132 : 39)) value.bits=0x7f800000U;
        else if(order>=(hex ? -153 : -46)) {
            if(!hex) {
                Integer *power=exponent<0 ? &den : &n;
                unsigned steps=exponent<0 ? (unsigned)-exponent : (unsigned)exponent;
                while(steps--) multiply(power,5,0);
            }
            uint32_t lo=0,hi=0x7f800000U;
            while(hi-lo>1U) {
                uint32_t mid=lo+(hi-lo)/2U;
                if(float_compare(&n,&den,exponent,mid,false)>=0) lo=mid;
                else hi=mid;
            }
            int cmp=float_compare(&n,&den,exponent,lo,true);
            value.bits=lo+(cmp>0 || (cmp==0 && (lo&1U)));
        }
    }
    if(negative) value.bits|=0x80000000U;
    return value.f;
}
