#ifndef COMPACT_TEXT_H
#define COMPACT_TEXT_H
/* Cold diagnostic lookup: concatenated NUL strings avoid 32-bit pointer tables.
 * Callers bound the index. No heap, writable pool or dependency on the LCD. */
static __attribute__((noinline,noclone,optimize("Os"))) const char *CompactText_At(const char *text,unsigned index)
{
    while(index--) while(*text++);
    return text;
}
#endif
