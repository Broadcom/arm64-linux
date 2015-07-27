#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/parse-integer.h>
#include <asm/bug.h>

#define for_each_test(i, test)  \
	for (i = 0; i < ARRAY_SIZE(test); i++)

#define DEFINE_TEST_OK(type, test_type, test)	\
test_type {					\
	const char *str;			\
	unsigned int base;			\
	int expected_rv;			\
	type expected_val;			\
};						\
static const test_type test[] __initconst =

#define TEST_OK(type, fmt, test)				\
{								\
	unsigned int i;						\
								\
	for_each_test(i, test) {				\
		const typeof(test[0]) *t = &test[i];		\
		type val;					\
		int rv;						\
								\
		rv = parse_integer(t->str, t->base, &val);	\
		if (rv != t->expected_rv || val != t->expected_val) {				\
			WARN(1, "str '%s', base %u, expected %d/"fmt", got %d/"fmt"\n",		\
				t->str, t->base, t->expected_rv, t->expected_val, rv, val);	\
		}						\
	}							\
}

struct test_fail {
	const char *str;
	unsigned int base;
};

#define DEFINE_TEST_FAIL(type, test)	\
static const struct test_fail test[] __initconst =

#define TEST_FAIL(type, fmt, test)				\
{								\
	unsigned int i;						\
								\
	for_each_test(i, test) {				\
		const typeof(test[0]) *t = &test[i];		\
		type val;					\
		int rv;						\
								\
		val = 113;					\
		rv = parse_integer(t->str, t->base, &val);	\
		if (rv >= 0 || val != 113) {			\
			WARN(1, "str '%s', base %u, expected -E, got %d/"fmt"\n",\
				t->str, t->base, rv, val);	\
		}						\
	}							\
}

DEFINE_TEST_OK(unsigned long long, struct test_ull, test_ull_ok)
{
	{"0",	10,	1,	0},
	{"1",	10,	1,	1},
	{"2",	10,	1,	2},
	{"3",	10,	1,	3},
	{"4",	10,	1,	4},
	{"5",	10,	1,	5},
	{"6",	10,	1,	6},
	{"7",	10,	1,	7},
	{"8",	10,	1,	8},
	{"9",	10,	1,	9},

	{"0",	8,	1,	0},
	{"1",	8,	1,	1},
	{"2",	8,	1,	2},
	{"3",	8,	1,	3},
	{"4",	8,	1,	4},
	{"5",	8,	1,	5},
	{"6",	8,	1,	6},
	{"7",	8,	1,	7},

	{"0",	16,	1,	0},
	{"1",	16,	1,	1},
	{"2",	16,	1,	2},
	{"3",	16,	1,	3},
	{"4",	16,	1,	4},
	{"5",	16,	1,	5},
	{"6",	16,	1,	6},
	{"7",	16,	1,	7},
	{"8",	16,	1,	8},
	{"9",	16,	1,	9},
	{"a",	16,	1,	10},
	{"b",	16,	1,	11},
	{"c",	16,	1,	12},
	{"d",	16,	1,	13},
	{"e",	16,	1,	14},
	{"f",	16,	1,	15},
	{"A",	16,	1,	10},
	{"B",	16,	1,	11},
	{"C",	16,	1,	12},
	{"D",	16,	1,	13},
	{"E",	16,	1,	14},
	{"F",	16,	1,	15},

	{"127",				10,	3,	127},
	{"128",				10,	3,	128},
	{"255",				10,	3,	255},
	{"256",				10,	3,	256},
	{"32767",			10,	5,	32767},
	{"32768",			10,	5,	32768},
	{"65535",			10,	5,	65535},
	{"65536",			10,	5,	65536},
	{"2147483647",			10,	10,	2147483647},
	{"2147483648",			10,	10,	2147483648ull},
	{"4294967295",			10,	10,	4294967295ull},
	{"4294967296",			10,	10,	4294967296},
	{"9223372036854775807",		10,	19,	9223372036854775807},
	{"9223372036854775808",		10,	19,	9223372036854775808ull},
	{"18446744073709551615",	10,	20,	18446744073709551615ull},

	{"177",				8,	3,	0177},
	{"200",				8,	3,	0200},
	{"377",				8,	3,	0377},
	{"400",				8,	3,	0400},
	{"77777",			8,	5,	077777},
	{"100000",			8,	6,	0100000},
	{"177777",			8,	6,	0177777},
	{"200000",			8,	6,	0200000},
	{"17777777777",			8,	11,	017777777777},
	{"20000000000",			8,	11,	020000000000},
	{"37777777777",			8,	11,	037777777777},
	{"40000000000",			8,	11,	040000000000},
	{"777777777777777777777",	8,	21,	0777777777777777777777},
	{"1000000000000000000000",	8,	22,	01000000000000000000000},
	{"1777777777777777777777",	8,	22,	01777777777777777777777},

	{"7f",			16,	2,	0x7f},
	{"80",			16,	2,	0x80},
	{"ff",			16,	2,	0xff},
	{"100",			16,	3,	0x100},
	{"7fff",		16,	4,	0x7fff},
	{"8000",		16,	4,	0x8000},
	{"ffff",		16,	4,	0xffff},
	{"10000",		16,	5,	0x10000},
	{"7fffffff",		16,	8,	0x7fffffff},
	{"80000000",		16,	8,	0x80000000},
	{"ffffffff",		16,	8,	0xffffffff},
	{"100000000",		16,	9,	0x100000000},
	{"7fffffffffffffff",	16,	16,	0x7fffffffffffffff},
	{"8000000000000000",	16,	16,	0x8000000000000000},
	{"ffffffffffffffff",	16,	16,	0xffffffffffffffff},
	/* test sign */
	{"+0",	10,	2,	0},
	{"+42",	10,	3,	42},
	/* test termination */
	{"42/",	10,	2,	42},
	{"42:",	10,	2,	42},
	{"42/",	8,	2,	042},
	{"428",	8,	2,	042},
	{"42/",	16,	2,	0x42},
	{"42`",	16,	2,	0x42},
	{"42g",	16,	2,	0x42},
	{"42@",	16,	2,	0x42},
	{"42G",	16,	2,	0x42},
	/* base autodetection */
	{"010",		0,	3,	8},
	{"0x10",	0,	4,	16},
	{"0X10",	0,	4,	16},
};

static void __init test_parse_integer_ull_ok(void)
{
	TEST_OK(unsigned long long, "%llu", test_ull_ok);
}

DEFINE_TEST_FAIL(unsigned long long, test_ull_fail)
{
	/* type overflow */
	{"10000000000000000000000000000000000000000000000000000000000000000",	2},
	{"18446744073709551616",	10},
	{"2000000000000000000000",	8},
	{"10000000000000000",		16},

	{"",	0},
	{"",	10},
	{"",	8},
	{"",	16},
	{"+",	0},
	{"+",	10},
	{"+",	8},
	{"+",	16},
	{"-",	0},
	{"-",	10},
	{"-",	8},
	{"-",	16},
	{" ",	0},
	{" ",	10},
	{" ",	8},
	{" ",	16},
	{"\n",	0},
	{"\n",	10},
	{"\n",	8},
	{"\n",	16},
	{" 0",	0},
	{" 0",	10},
	{" 0",	8},
	{" 0",	16},
	{"\n0",	0},
	{"\n0",	10},
	{"\n0",	8},
	{"\n0",	16},
	/* non-digit */
	{"/",	10},
	{":",	10},
	{"/",	8},
	{"8",	8},
	{"/",	16},
	{":",	16},
	{"`",	16},
	{"g",	16},
	{"@",	16},
	{"G",	16},
	{"/0",	10},
	{":0",	10},
	{"/0",	8},
	{"80",	8},
	{"/0",	16},
	{":0",	16},
	{"`0",	16},
	{"g0",	16},
	{"@0",	16},
	{"G0",	16},

	{"-0",	0},
	{"-0",	10},
	{"-0",	8},
	{"-0",	16},
	{"-1",	0},
	{"-1",	10},
	{"-1",	8},
	{"-1",	16},
	/* accept only one sign */
	{"--",	0},
	{"--",	10},
	{"--",	8},
	{"--",	16},
	{"-+",	0},
	{"-+",	10},
	{"-+",	8},
	{"-+",	16},
	{"+-",	0},
	{"+-",	10},
	{"+-",	8},
	{"+-",	16},
	{"++",	0},
	{"++",	10},
	{"++",	8},
	{"++",	16},
	{"--0",	0},
	{"--0",	10},
	{"--0",	8},
	{"--0",	16},
	{"-+0",	0},
	{"-+0",	10},
	{"-+0",	8},
	{"-+0",	16},
	{"+-0",	0},
	{"+-0",	10},
	{"+-0",	8},
	{"+-0",	16},
	{"++0",	0},
	{"++0",	10},
	{"++0",	8},
	{"++0",	16},
};

static void __init test_parse_integer_ull_fail(void)
{
	TEST_FAIL(unsigned long long, "%llu", test_ull_fail);
}

DEFINE_TEST_OK(long long, struct test_ll, test_ll_ok)
{
	{"-9223372036854775808",10,	20,	LLONG_MIN},
	{"-4294967296",		10,	11,	-4294967296},
	{"-2147483648",		10,	11,	-2147483648ll},
	{"-65536",		10,	6,	-65536},
	{"-32768",		10,	6,	-32768},
	{"-256",		10,	4,	-256},
	{"-128",		10,	4,	-128},
	{"-0",			10,	2,	0},
	{"0",			10,	1,	0},
	{"127",			10,	3,	127},
	{"255",			10,	3,	255},
	{"32767",		10,	5,	32767},
	{"65535",		10,	5,	65535},
	{"2147483647",		10,	10,	2147483647},
	{"4294967295",		10,	10,	4294967295ll},
	{"9223372036854775807",	10,	19,	9223372036854775807},
};

static void __init test_parse_integer_ll_ok(void)
{
	TEST_OK(long long, "%lld", test_ll_ok);
}

DEFINE_TEST_FAIL(long long, test_ll_fail)
{
	{"-9223372036854775809",	10},
	{"9223372036854775808",		10},
};

static void __init test_parse_integer_ll_fail(void)
{
	TEST_FAIL(long long, "%lld", test_ll_fail);
}

DEFINE_TEST_OK(unsigned int, struct test_u, test_u_ok)
{
	{"0",		10,	1,	0},
	{"127",		10,	3,	127},
	{"128",		10,	3,	128},
	{"255",		10,	3,	255},
	{"256",		10,	3,	256},
	{"32767",	10,	5,	32767},
	{"32768",	10,	5,	32768},
	{"65535",	10,	5,	65535},
	{"65536",	10,	5,	65536},
	{"2147483647",	10,	10,	2147483647},
	{"2147483648",	10,	10,	2147483648u},
	{"4294967295",	10,	10,	4294967295u},
};

static void __init test_parse_integer_u_ok(void)
{
	TEST_OK(unsigned int, "%u", test_u_ok);
}

DEFINE_TEST_FAIL(unsigned int, test_u_fail)
{
	{"4294967296",			10},
	{"9223372036854775807",		10},
	{"9223372036854775808",		10},
	{"18446744073709551615",	10},
};

static void __init test_parse_integer_u_fail(void)
{
	TEST_FAIL(unsigned int, "%u", test_u_fail);
}

DEFINE_TEST_OK(int, struct test_i, test_i_ok)
{
	{"-2147483648",	10,	11,	INT_MIN},
	{"-65536",	10,	6,	-65536},
	{"-32768",	10,	6,	-32768},
	{"-256",	10,	4,	-256},
	{"-128",	10,	4,	-128},
	{"-0",		10,	2,	0},
	{"0",		10,	1,	0},
	{"127",		10,	3,	127},
	{"255",		10,	3,	255},
	{"32767",	10,	5,	32767},
	{"65535",	10,	5,	65535},
	{"2147483647",	10,	10,	2147483647},
};

static void __init test_parse_integer_i_ok(void)
{
	TEST_OK(int, "%d", test_i_ok);
}

DEFINE_TEST_FAIL(int, test_i_fail)
{
	{"-9223372036854775809",	10},
	{"-9223372036854775808",	10},
	{"-4294967296",			10},
	{"-2147483649",			10},
	{"2147483648",			10},
	{"4294967295",			10},
	{"9223372036854775807",		10},
	{"9223372036854775808",		10},
};

static void __init test_parse_integer_i_fail(void)
{
	TEST_FAIL(int, "%d", test_i_fail);
}

DEFINE_TEST_OK(unsigned short, struct test_us, test_us_ok)
{
	{"0",		10,	1,	0},
	{"127",		10,	3,	127},
	{"128",		10,	3,	128},
	{"255",		10,	3,	255},
	{"256",		10,	3,	256},
	{"32767",	10,	5,	32767},
	{"32768",	10,	5,	32768},
	{"65535",	10,	5,	65535},
};

static void __init test_parse_integer_us_ok(void)
{
	TEST_OK(unsigned short, "%hu", test_us_ok);
}

DEFINE_TEST_FAIL(unsigned short, test_us_fail)
{
	{"65536",			10},
	{"2147483647",			10},
	{"2147483648",			10},
	{"4294967295",			10},
	{"4294967296",			10},
	{"9223372036854775807",		10},
	{"9223372036854775808",		10},
	{"18446744073709551615",	10},
};

static void __init test_parse_integer_us_fail(void)
{
	TEST_FAIL(unsigned short, "%hu", test_us_fail);
}

DEFINE_TEST_OK(short, struct test_s, test_s_ok)
{
	{"-32768",	10,	6,	-32768},
	{"-256",	10,	4,	-256},
	{"-128",	10,	4,	-128},
	{"-0",		10,	2,	0},
	{"0",		10,	1,	0},
	{"127",		10,	3,	127},
	{"255",		10,	3,	255},
	{"32767",	10,	5,	32767},
};

static void __init test_parse_integer_s_ok(void)
{
	TEST_OK(short, "%hd", test_s_ok);
}

DEFINE_TEST_FAIL(short, test_s_fail)
{
	{"-9223372036854775809",	10},
	{"-9223372036854775808",	10},
	{"-4294967296",			10},
	{"-2147483649",			10},
	{"-2147483648",			10},
	{"-65536",			10},
	{"-32769",			10},
	{"32768",			10},
	{"65535",			10},
	{"2147483647",			10},
	{"2147483648",			10},
	{"4294967295",			10},
	{"9223372036854775807",		10},
	{"9223372036854775808",		10},
};

static void __init test_parse_integer_s_fail(void)
{
	TEST_FAIL(short, "%hd", test_s_fail);
}

DEFINE_TEST_OK(unsigned char, struct test_uc, test_uc_ok)
{
	{"0",	10,	1,	0},
	{"127",	10,	3,	127},
	{"128",	10,	3,	128},
	{"255",	10,	3,	255},
};

static void __init test_parse_integer_uc_ok(void)
{
	TEST_OK(unsigned char, "%hhu", test_uc_ok);
}

DEFINE_TEST_FAIL(unsigned char, test_uc_fail)
{
	{"256",				10},
	{"32767",			10},
	{"32768",			10},
	{"65535",			10},
	{"65536",			10},
	{"2147483647",			10},
	{"2147483648",			10},
	{"4294967295",			10},
	{"4294967296",			10},
	{"9223372036854775807",		10},
	{"9223372036854775808",		10},
	{"18446744073709551615",	10},
};

static void __init test_parse_integer_uc_fail(void)
{
	TEST_FAIL(unsigned char, "%hhu", test_uc_fail);
}

DEFINE_TEST_OK(signed char, struct test_sc, test_sc_ok)
{
	{"-128",	10,	4,	-128},
	{"-0",		10,	2,	0},
	{"0",		10,	1,	0},
	{"127",		10,	3,	127},
};

static void __init test_parse_integer_sc_ok(void)
{
	TEST_OK(signed char, "%hhd", test_sc_ok);
}

DEFINE_TEST_FAIL(signed char, test_sc_fail)
{
	{"-9223372036854775809",	10},
	{"-9223372036854775808",	10},
	{"-4294967296",			10},
	{"-2147483649",			10},
	{"-2147483648",			10},
	{"-65536",			10},
	{"-32769",			10},
	{"-32768",			10},
	{"-256",			10},
	{"-129",			10},
	{"128",				10},
	{"255",				10},
	{"32767",			10},
	{"32768",			10},
	{"65535",			10},
	{"2147483647",			10},
	{"2147483648",			10},
	{"4294967295",			10},
	{"9223372036854775807",		10},
	{"9223372036854775808",		10},
};

static void __init test_parse_integer_sc_fail(void)
{
	TEST_FAIL(signed char, "%hhd", test_sc_fail);
}

static int __init test_parse_integer_init(void)
{
	test_parse_integer_ull_ok();
	test_parse_integer_ull_fail();
	test_parse_integer_ll_ok();
	test_parse_integer_ll_fail();
	test_parse_integer_u_ok();
	test_parse_integer_u_fail();
	test_parse_integer_i_ok();
	test_parse_integer_i_fail();
	test_parse_integer_us_ok();
	test_parse_integer_us_fail();
	test_parse_integer_s_ok();
	test_parse_integer_s_fail();
	test_parse_integer_uc_ok();
	test_parse_integer_uc_fail();
	test_parse_integer_sc_ok();
	test_parse_integer_sc_fail();
	return -EINVAL;
}
module_init(test_parse_integer_init);
MODULE_LICENSE("Dual BSD/GPL");
