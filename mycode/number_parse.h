#ifndef NUMBER_PARSE_H
#define NUMBER_PARSE_H
/* <=63-byte binary32 wire numbers, decimal or C hexadecimal. No heap/double.
 * Returns infinity for overflow so the protocol can reject it explicitly. */
float Number_Parse(const char *text,const char **end);
#endif
