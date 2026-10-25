#ifndef UI_TEXT_H
#define UI_TEXT_H
#include <stddef.h>

/* Application display text only: unpadded %s/%c/%% and 32-bit %d/%u/%X.
 * Integers support l, nonnegative numeric width (or *) and zero padding.
 * No floating-point conversions, precision or other printf flags.
 * Returns the untruncated length, or -1 for an unsupported conversion.
 * A nonzero capacity always reserves a terminator, as snprintf does. */
int UiText_Format(char *buffer,size_t capacity,const char *format,...)
    __attribute__((format(printf,3,4)));
#endif
