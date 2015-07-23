#ifndef _LIB_KSTRTOX_H
#define _LIB_KSTRTOX_H

#define KSTRTOX_OVERFLOW	(1U << 31)
unsigned int _parse_integer(const char *s, unsigned int base, unsigned long long *res);

#endif
