#ifndef UI_TEXT_H
#define UI_TEXT_H
#include <stddef.h>
#include <stdint.h>

/* Decimal places 0..3; optional unit is separated by one space. */
int UiText_Fixed(char *buffer,size_t capacity,int32_t value,unsigned places,const char *unit);

/* Application display text only: unpadded %s/%c/%% and 32-bit %d/%u/%X.
 * Integers support l, nonnegative numeric width (or *) and zero padding.
 * No floating-point conversions, precision or other printf flags.
 * Returns the untruncated length, or -1 for an unsupported conversion.
 * A nonzero capacity always reserves a terminator, as snprintf does. */
int UiText_Format(char *buffer,size_t capacity,const char *format,...)
    __attribute__((format(printf,3,4)));
#endif
