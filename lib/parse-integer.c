/*
 * See parse_integer().
 *
 * Individual dispatch functions in this file aren't supposed to be used
 * directly and thus aren't advertised and documented despited being exported.
 *
 * Do not use any function in this file for any reason.
 */
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/parse-integer.h>
#include <asm/bug.h>

const char *_parse_integer_fixup_radix(const char *s, unsigned int *base)
{
	if (*base == 0) {
		if (s[0] == '0') {
			if (_tolower(s[1]) == 'x' && isxdigit(s[2]))
				*base = 16;
			else
				*base = 8;
		} else
			*base = 10;
	}
	if (*base == 16 && s[0] == '0' && _tolower(s[1]) == 'x')
		s += 2;
	BUG_ON(*base < 2 || *base > 16);
	return s;
}

static int ___parse_integer(const char *s, unsigned int base, unsigned long long *val)
{
	const char *s0 = s, *sd;
	unsigned long long acc;

	s = sd = _parse_integer_fixup_radix(s0, &base);
	acc = 0;
	while (*s) {
		unsigned int d;

		if ('0' <= *s && *s <= '9')
			d = *s - '0';
		else if ('a' <= _tolower(*s) && _tolower(*s) <= 'f')
			d = _tolower(*s) - 'a' + 10;
		else
			break;
		if (d >= base)
			break;
		/* Overflow can't happen early enough. */
		if ((acc >> 60) && acc > div_u64(ULLONG_MAX - d, base))
			return -ERANGE;
		acc = acc * base + d;
		s++;
	}
	/* At least one digit has to be converted. */
	if (s == sd)
		return -EINVAL;
	*val = acc;
	/* Radix 1 is not supported otherwise returned length can overflow. */
	return s - s0;
}

static int __parse_integer(const char *s, unsigned int base, unsigned long long *val)
{
	unsigned long long tmp;
	int rv;

	rv = ___parse_integer(s, base & ~PARSE_INTEGER_NEWLINE, &tmp);
	if (rv < 0)
		return rv;
	if (base & PARSE_INTEGER_NEWLINE) {
		/* Accept "integer\0" or "integer\n\0" */
		s += rv;
		if (*s == '\n')
			s++;
		if (*s)
			return -EINVAL;
	}
	*val = tmp;
	return rv;
}

int _parse_integer_ull(const char *s, unsigned int base, unsigned long long *val)
{
	char sign;
	int rv;

	sign = 0;
	if (*s == '-')
		return -EINVAL;
	else if (*s == '+')
		sign = *s++;

	rv = __parse_integer(s, base, val);
	if (rv < 0)
		return rv;
	if (base & PARSE_INTEGER_NEWLINE)
		return 0;
	return rv + !!sign;
}
EXPORT_SYMBOL(_parse_integer_ull);

int _parse_integer_ll(const char *s, unsigned int base, long long *val)
{
	unsigned long long tmp;
	char sign;
	int rv;

	sign = 0;
	if (*s == '-' || *s == '+')
		sign = *s++;

	rv = __parse_integer(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (sign == '-') {
		if ((long long)-tmp > 0)
			return -ERANGE;
		*val = -tmp;
	} else {
		if ((long long)tmp < 0)
			return -ERANGE;
		*val = tmp;
	}
	if (base & PARSE_INTEGER_NEWLINE)
		return 0;
	return rv + !!sign;
}
EXPORT_SYMBOL(_parse_integer_ll);

int _parse_integer_u(const char *s, unsigned int base, unsigned int *val)
{
	unsigned long long tmp;
	int rv;

	rv = _parse_integer_ull(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (tmp != (unsigned int)tmp)
		return -ERANGE;
	*val = tmp;
	return rv;
}
EXPORT_SYMBOL(_parse_integer_u);

int _parse_integer_i(const char *s, unsigned int base, int *val)
{
	long long tmp;
	int rv;

	rv = _parse_integer_ll(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (tmp != (int)tmp)
		return -ERANGE;
	*val = tmp;
	return rv;
}
EXPORT_SYMBOL(_parse_integer_i);

int _parse_integer_us(const char *s, unsigned int base, unsigned short *val)
{
	unsigned long long tmp;
	int rv;

	rv = _parse_integer_ull(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (tmp != (unsigned short)tmp)
		return -ERANGE;
	*val = tmp;
	return rv;
}
EXPORT_SYMBOL(_parse_integer_us);

int _parse_integer_s(const char *s, unsigned int base, short *val)
{
	long long tmp;
	int rv;

	rv = _parse_integer_ll(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (tmp != (short)tmp)
		return -ERANGE;
	*val = tmp;
	return rv;
}
EXPORT_SYMBOL(_parse_integer_s);

int _parse_integer_uc(const char *s, unsigned int base, unsigned char *val)
{
	unsigned long long tmp;
	int rv;

	rv = _parse_integer_ull(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (tmp != (unsigned char)tmp)
		return -ERANGE;
	*val = tmp;
	return rv;
}
EXPORT_SYMBOL(_parse_integer_uc);

int _parse_integer_sc(const char *s, unsigned int base, signed char *val)
{
	long long tmp;
	int rv;

	rv = _parse_integer_ll(s, base, &tmp);
	if (rv < 0)
		return rv;
	if (tmp != (signed char)tmp)
		return -ERANGE;
	*val = tmp;
	return rv;
}
EXPORT_SYMBOL(_parse_integer_sc);
