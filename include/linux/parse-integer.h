#ifndef _PARSE_INTEGER_H
#define _PARSE_INTEGER_H
#include <linux/compiler.h>
#include <linux/types.h>

/*
 * int parse_integer(const char *s, unsigned int base, T *val);
 *
 * Convert integer string representation to an integer.
 * Range of accepted values equals to that of type T.
 *
 * Conversion to unsigned integer accepts sign "+".
 * Conversion to signed integer accepts sign "+" and sign "-".
 *
 * Radix 0 means autodetection: leading "0x" implies radix 16,
 * leading "0" implies radix 8, otherwise radix is 10.
 * Autodetection hint works after optional sign, but not before.
 *
 * Return number of characters parsed or -E.
 *
 * "T=char" case is not supported because -f{un,}signed-char can silently
 * change range of accepted values.
 */
#define parse_integer(s, base, val)	\
({					\
	const char *_s = (s);		\
	unsigned int _base = (base);	\
	typeof(&(val)[0]) _val = (val);	\
					\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), signed char *),	\
	_parse_integer_sc(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), unsigned char *),	\
	_parse_integer_uc(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), short *),		\
	_parse_integer_s(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), unsigned short *),	\
	_parse_integer_us(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), int *),		\
	_parse_integer_i(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), unsigned int *),	\
	_parse_integer_u(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), long *) && sizeof(long) == 4,\
	_parse_integer_i(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), long *) && sizeof(long) == 8,\
	_parse_integer_ll(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), unsigned long *) && sizeof(unsigned long) == 4,\
	_parse_integer_u(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), unsigned long *) && sizeof(unsigned long) == 8,\
	_parse_integer_ull(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), long long *),	\
	_parse_integer_ll(_s, _base, (void *)_val),			\
	__builtin_choose_expr(						\
	__builtin_types_compatible_p(typeof(_val), unsigned long long *),\
	_parse_integer_ull(_s, _base, (void *)_val),			\
	_parse_integer_link_time_error()))))))))))));	\
})
/* internal, do not use */
int _parse_integer_sc(const char *s, unsigned int base, signed char *val);
int _parse_integer_uc(const char *s, unsigned int base, unsigned char *val);
int _parse_integer_s(const char *s, unsigned int base, short *val);
int _parse_integer_us(const char *s, unsigned int base, unsigned short *val);
int _parse_integer_i(const char *s, unsigned int base, int *val);
int _parse_integer_u(const char *s, unsigned int base, unsigned int *val);
int _parse_integer_ll(const char *s, unsigned int base, long long *val);
int _parse_integer_ull(const char *s, unsigned int base, unsigned long long *val);
void _parse_integer_link_time_error(void);
const char *_parse_integer_fixup_radix(const char *s, unsigned int *base);
#endif
