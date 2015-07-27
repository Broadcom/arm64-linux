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
#define PARSE_INTEGER_NEWLINE 0x80000000u

/*
 * Convert integer string representation terminated by \n\0 or \0 to an integer.
 *
 * Return 0 on success or -E.
 *
 * See parse_integer().
 */
static inline int __must_check kstrtoull(const char *s, unsigned int base, unsigned long long *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtoll(const char *s, unsigned int base, long long *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtoul(const char *s, unsigned int base, unsigned long *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtol(const char *s, unsigned int base, long *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtouint(const char *s, unsigned int base, unsigned int *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtoint(const char *s, unsigned int base, int *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtou64(const char *s, unsigned int base, u64 *res)
{
	return kstrtoull(s, base, res);
}

static inline int __must_check kstrtos64(const char *s, unsigned int base, s64 *res)
{
	return kstrtoll(s, base, res);
}

static inline int __must_check kstrtou32(const char *s, unsigned int base, u32 *res)
{
	return kstrtouint(s, base, res);
}

static inline int __must_check kstrtos32(const char *s, unsigned int base, s32 *res)
{
	return kstrtoint(s, base, res);
}

static inline int __must_check kstrtou16(const char *s, unsigned int base, u16 *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtos16(const char *s, unsigned int base, s16 *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtou8(const char *s, unsigned int base, u8 *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

static inline int __must_check kstrtos8(const char *s, unsigned int base, s8 *res)
{
	return parse_integer(s, base | PARSE_INTEGER_NEWLINE, res);
}

int __must_check kstrtoull_from_user(const char __user *s, size_t count, unsigned int base, unsigned long long *res);
int __must_check kstrtoll_from_user(const char __user *s, size_t count, unsigned int base, long long *res);
int __must_check kstrtoul_from_user(const char __user *s, size_t count, unsigned int base, unsigned long *res);
int __must_check kstrtol_from_user(const char __user *s, size_t count, unsigned int base, long *res);
int __must_check kstrtouint_from_user(const char __user *s, size_t count, unsigned int base, unsigned int *res);
int __must_check kstrtoint_from_user(const char __user *s, size_t count, unsigned int base, int *res);
int __must_check kstrtou16_from_user(const char __user *s, size_t count, unsigned int base, u16 *res);
int __must_check kstrtos16_from_user(const char __user *s, size_t count, unsigned int base, s16 *res);
int __must_check kstrtou8_from_user(const char __user *s, size_t count, unsigned int base, u8 *res);
int __must_check kstrtos8_from_user(const char __user *s, size_t count, unsigned int base, s8 *res);

static inline int __must_check kstrtou64_from_user(const char __user *s, size_t count, unsigned int base, u64 *res)
{
	return kstrtoull_from_user(s, count, base, res);
}

static inline int __must_check kstrtos64_from_user(const char __user *s, size_t count, unsigned int base, s64 *res)
{
	return kstrtoll_from_user(s, count, base, res);
}

static inline int __must_check kstrtou32_from_user(const char __user *s, size_t count, unsigned int base, u32 *res)
{
	return kstrtouint_from_user(s, count, base, res);
}

static inline int __must_check kstrtos32_from_user(const char __user *s, size_t count, unsigned int base, s32 *res)
{
	return kstrtoint_from_user(s, count, base, res);
}
#endif
