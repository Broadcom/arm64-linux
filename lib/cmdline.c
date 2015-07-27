/*
 * linux/lib/cmdline.c
 * Helper functions generally used for parsing kernel command line
 * and module options.
 *
 * Code and copyrights come from init/main.c and arch/i386/kernel/setup.c.
 *
 * This source code is licensed under the GNU General Public License,
 * Version 2.  See the file COPYING for more details.
 *
 * GNU Indent formatting options for this file: -kr -i8 -npsl -pcs
 *
 */

#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/string.h>

/*
 *	If a hyphen was found in get_option, this will handle the
 *	range of numbers, M-N.  This will expand the range and insert
 *	the values[M, M+1, ..., N] into the ints array in get_options.
 */

static int get_range(char **str, int *pint)
{
	int x, inc_counter, upper_range;

	(*str)++;
	parse_integer(*str, 0, &upper_range);
	inc_counter = upper_range - *pint;
	for (x = *pint; x < upper_range; x++)
		*pint++ = x;
	return inc_counter;
}

/**
 *	get_option - Parse integer from an option string
 *	@str: option string
 *	@pint: (output) integer value parsed from @str
 *
 *	Read an int from an option string; if available accept a subsequent
 *	comma as well.
 *
 *	Return values:
 *	0 - no int in string
 *	1 - int found, no subsequent comma
 *	2 - int found including a subsequent comma
 *	3 - hyphen found to denote a range
 */

int get_option(char **str, int *pint)
{
	int len;

	if (!str || !*str)
		return 0;
	len = parse_integer(*str, 0, pint);
	if (len < 0)
		return 0;
	*str += len;
	if (**str == ',') {
		(*str)++;
		return 2;
	}
	if (**str == '-')
		return 3;

	return 1;
}
EXPORT_SYMBOL(get_option);

/**
 *	get_options - Parse a string into a list of integers
 *	@str: String to be parsed
 *	@nints: size of integer array
 *	@ints: integer array
 *
 *	This function parses a string containing a comma-separated
 *	list of integers, a hyphen-separated range of _positive_ integers,
 *	or a combination of both.  The parse halts when the array is
 *	full, or when no more numbers can be retrieved from the
 *	string.
 *
 *	Return value is the character in the string which caused
 *	the parse to end (typically a null terminator, if @str is
 *	completely parseable).
 */

char *get_options(const char *str, int nints, int *ints)
{
	int res, i = 1;

	while (i < nints) {
		res = get_option((char **)&str, ints + i);
		if (res == 0)
			break;
		if (res == 3) {
			int range_nums;
			range_nums = get_range((char **)&str, ints + i);
			if (range_nums < 0)
				break;
			/*
			 * Decrement the result by one to leave out the
			 * last number in the range.  The next iteration
			 * will handle the upper number in the range
			 */
			i += (range_nums - 1);
		}
		i++;
		if (res == 1)
			break;
	}
	ints[0] = i - 1;
	return (char *)str;
}
EXPORT_SYMBOL(get_options);

/**
 *	memparse - parse a string with mem suffixes into a number
 *	@ptr: Where parse begins
 *	@retptr: (output) Optional pointer to next char after parse completes
 *
 *	Parses a string into a number.  The number stored at @ptr is
 *	potentially suffixed with K, M, G, T, P, E.
 */

unsigned long long memparse(const char *ptr, char **retptr)
{
	unsigned long long val = 0;
	int len;

	len = parse_integer(ptr, 0, &val);
	if (len < 0)
		goto out;
	ptr += len;
	switch (*ptr) {
	case 'E':
	case 'e':
		val <<= 10;
	case 'P':
	case 'p':
		val <<= 10;
	case 'T':
	case 't':
		val <<= 10;
	case 'G':
	case 'g':
		val <<= 10;
	case 'M':
	case 'm':
		val <<= 10;
	case 'K':
	case 'k':
		val <<= 10;
		ptr++;
	default:
		break;
	}
out:
	if (retptr)
		*retptr = (char *)ptr;

	return val;
}
EXPORT_SYMBOL(memparse);

/**
 *	parse_option_str - Parse a string and check an option is set or not
 *	@str: String to be parsed
 *	@option: option name
 *
 *	This function parses a string containing a comma-separated list of
 *	strings like a=b,c.
 *
 *	Return true if there's such option in the string, or return false.
 */
bool parse_option_str(const char *str, const char *option)
{
	while (*str) {
		if (!strncmp(str, option, strlen(option))) {
			str += strlen(option);
			if (!*str || *str == ',')
				return true;
		}

		while (*str && *str != ',')
			str++;

		if (*str == ',')
			str++;
	}

	return false;
}
