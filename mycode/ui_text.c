#include "ui_text.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

typedef struct { char *buffer; size_t capacity,length; } TextWriter;
static void put_char(TextWriter *out,char c)
{
    if(out->length+1<out->capacity) out->buffer[out->length]=c;
    ++out->length;
}
static int finish(TextWriter *out,int result)
{
    if(out->capacity)
        out->buffer[out->length<out->capacity ? out->length : out->capacity-1]='\0';
    return result;
}
__attribute__((noinline,noclone))
static unsigned number_digits(char *digits,uint32_t value,unsigned base,unsigned minimum)
{
    unsigned count=0;
    do { digits[count++]="0123456789ABCDEF"[value%base]; value/=base; }
    while(value || count<minimum);
    return count;
}
__attribute__((noinline))
int UiText_Fixed(char *buffer,size_t capacity,int32_t value,unsigned places,const char *unit)
{
    TextWriter out={buffer,capacity,0};
    uint32_t magnitude=value<0 ? 0U-(uint32_t)value : (uint32_t)value;
    char digits[10];
    /* Include a leading integer zero even for values below one unit. */
    unsigned count=number_digits(digits,magnitude,10U,places+1U);
    if(value<0) put_char(&out,'-');
    while(count) {
        put_char(&out,digits[--count]);
        if(count && count==places) put_char(&out,'.');
    }
    if(unit) {
        put_char(&out,' ');
        while(*unit) put_char(&out,*unit++);
    }
    return finish(&out,(int)out.length);
}
__attribute__((noinline,noclone))
int UiText_Format(char *buffer,size_t capacity,const char *format,...)
{
    TextWriter out={buffer,capacity,0};
    va_list args;
    va_start(args,format);
    while(*format) {
        char type=*format++;
        if(type!='%') { put_char(&out,type); continue; }
        bool zero=*format=='0';
        if(zero) ++format;
        int width=0;
        if(*format=='*') { width=va_arg(args,int); ++format; }
        else while(*format>='0' && *format<='9') width=width*10+(*format++-'0');
        bool wide=*format=='l';
        if(wide) ++format;
        type=*format;
        if(type) ++format;
        if(type=='s') {
            const char *text=va_arg(args,const char *);
            while(*text) put_char(&out,*text++);
        } else if(type=='c' || type=='%') {
            put_char(&out,type=='%' ? '%' : (char)va_arg(args,int));
        } else if(type=='d' || type=='u' || type=='X') {
            bool negative=false;
            uint32_t value;
            if(type=='d') {
                int32_t number=wide ? (int32_t)va_arg(args,long) : va_arg(args,int);
                negative=number<0;
                value=negative ? 0U-(uint32_t)number : (uint32_t)number;
            } else value=wide ? (uint32_t)va_arg(args,unsigned long) : va_arg(args,unsigned);
            char digits[10];
            unsigned count=number_digits(digits,value,type=='X' ? 16U : 10U,1U);
            width-=(int)count+(negative ? 1 : 0);
            if(negative && zero) put_char(&out,'-');
            while(width-->0) put_char(&out,zero ? '0' : ' ');
            if(negative && !zero) put_char(&out,'-');
            while(count) put_char(&out,digits[--count]);
        } else {
            va_end(args);
            return finish(&out,-1);
        }
    }
    va_end(args);
    return finish(&out,(int)out.length);
}
