/* Tests the decoding of all types.
 * This is the counterpart of test_encode3.
 * Run e.g. ./test_encode3 | ./test_decode3
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pb_decode.h>
#include "alltypes.pb.h"
#include "test_helpers.h"

#define TEST(x) if (!(x)) { \
    printf("Test " #x " failed.\n"); \
    return false; \
    }

/* This function is called once from main(), it handles
   the decoding and checks the fields. */
bool check_alltypes(pb_istream_t *stream, int mode)
{
    AllTypes alltypes = AllTypes_init_zero;

    /* Fill with garbage to better detect initialization errors */
    memset(&alltypes, 0xAA, sizeof(alltypes));

    if (!pb_decode(stream, AllTypes_fields, &alltypes))
        return false;

    TEST(alltypes.rep_int32_count == 5 && alltypes.rep_int32[4] == -2001 && alltypes.rep_int32[0] == 0);
    TEST(alltypes.rep_int64_count == 5 && alltypes.rep_int64[4] == -2002 && alltypes.rep_int64[0] == 0);
    TEST(alltypes.rep_uint32_count == 5 && alltypes.rep_uint32[4] == 2003 && alltypes.rep_uint32[0] == 0);
    TEST(alltypes.rep_uint64_count == 5 && alltypes.rep_uint64[4] == 2004 && alltypes.rep_uint64[0] == 0);
    TEST(alltypes.rep_sint32_count == 5 && alltypes.rep_sint32[4] == -2005 && alltypes.rep_sint32[0] == 0);
    TEST(alltypes.rep_sint64_count == 5 && alltypes.rep_sint64[4] == -2006 && alltypes.rep_sint64[0] == 0);
    TEST(alltypes.rep_bool_count == 5 && alltypes.rep_bool[4] == true && alltypes.rep_bool[0] == false);

    TEST(alltypes.rep_fixed32_count == 5 && alltypes.rep_fixed32[4] == 2008 && alltypes.rep_fixed32[0] == 0);
    TEST(alltypes.rep_sfixed32_count == 5 && alltypes.rep_sfixed32[4] == -2009 && alltypes.rep_sfixed32[0] == 0);
    TEST(alltypes.rep_float_count == 5 && alltypes.rep_float[4] == 2010.0f && alltypes.rep_float[0] == 0.0f);

    TEST(alltypes.rep_fixed64_count == 5 && alltypes.rep_fixed64[4] == 2011 && alltypes.rep_fixed64[0] == 0);
    TEST(alltypes.rep_sfixed64_count == 5 && alltypes.rep_sfixed64[4] == -2012 && alltypes.rep_sfixed64[0] == 0);
    TEST(alltypes.rep_double_count == 5 && alltypes.rep_double[4] == 2013.0 && alltypes.rep_double[0] == 0.0);

    TEST(alltypes.rep_string_count == 5 && strcmp(alltypes.rep_string[4], "2014") == 0 && alltypes.rep_string[0][0] == '\0');
    TEST(alltypes.rep_bytes_count == 5 && alltypes.rep_bytes[4].size == 4 && alltypes.rep_bytes[0].size == 0);
    TEST(memcmp(alltypes.rep_bytes[4].bytes, "2015", 4) == 0);

    TEST(alltypes.rep_submsg_count == 5);
    TEST(strcmp(alltypes.rep_submsg[4].substuff1, "2016") == 0 && alltypes.rep_submsg[0].substuff1[0] == '\0');
    TEST(alltypes.rep_submsg[4].substuff2 == 2016 && alltypes.rep_submsg[0].substuff2 == 0);
    TEST(alltypes.rep_submsg[4].substuff3 == 2016 && alltypes.rep_submsg[0].substuff3 == 0);

    TEST(alltypes.rep_enum_count == 5 && alltypes.rep_enum[4] == MyEnum_Truth && alltypes.rep_enum[0] == MyEnum_Zero);
    TEST(alltypes.rep_emptymsg_count == 5);

    TEST(alltypes.rep_fbytes_count == 5);
    TEST(alltypes.rep_fbytes[0][0] == 0 && alltypes.rep_fbytes[0][3] == 0);
    TEST(memcmp(alltypes.rep_fbytes[4], "2019", 4) == 0);

    if (mode == 0)
    {
        /* Expect default values */
        TEST(alltypes.sng_int32         == 0);
        TEST(alltypes.sng_int64         == 0);
        TEST(alltypes.sng_uint32        == 0);
        TEST(alltypes.sng_uint64        == 0);
        TEST(alltypes.sng_sint32        == 0);
        TEST(alltypes.sng_sint64        == 0);
        TEST(alltypes.sng_bool          == false);

        TEST(alltypes.sng_fixed32       == 0);
        TEST(alltypes.sng_sfixed32      == 0);
        TEST(alltypes.sng_float         == 0.0f);

        TEST(alltypes.sng_fixed64       == 0);
        TEST(alltypes.sng_sfixed64      == 0);
        TEST(alltypes.sng_double        == 0.0);

        TEST(strcmp(alltypes.sng_string, "") == 0);
        TEST(alltypes.sng_bytes.size == 0);
        TEST(strcmp(alltypes.sng_submsg.substuff1, "") == 0);
        TEST(alltypes.sng_submsg.substuff2 == 0);
        TEST(alltypes.sng_submsg.substuff3 == 0);
        TEST(alltypes.sng_enum == MyEnum_Zero);
        TEST(alltypes.sng_fbytes[0] == 0 &&
             alltypes.sng_fbytes[1] == 0 &&
             alltypes.sng_fbytes[2] == 0 &&
             alltypes.sng_fbytes[3] == 0);

        TEST(alltypes.which_oneof == 0);
    }
    else
    {
        /* Expect filled-in values */
        TEST(alltypes.sng_int32         == 3041);
        TEST(alltypes.sng_int64         == 3042);
        TEST(alltypes.sng_uint32        == 3043);
        TEST(alltypes.sng_uint64        == 3044);
        TEST(alltypes.sng_sint32        == 3045);
        TEST(alltypes.sng_sint64        == 3046);
        TEST(alltypes.sng_bool          == true);

        TEST(alltypes.sng_fixed32       == 3048);
        TEST(alltypes.sng_sfixed32      == 3049);
        TEST(alltypes.sng_float         == 3050.0f);

        TEST(alltypes.sng_fixed64       == 3051);
        TEST(alltypes.sng_sfixed64      == 3052);
        TEST(alltypes.sng_double        == 3053.0);

        TEST(strcmp(alltypes.sng_string, "3054") == 0);
        TEST(alltypes.sng_bytes.size == 4);
        TEST(memcmp(alltypes.sng_bytes.bytes, "3055", 4) == 0);
        TEST(strcmp(alltypes.sng_submsg.substuff1, "3056") == 0);
        TEST(alltypes.sng_submsg.substuff2 == 3056);
        TEST(alltypes.sng_submsg.substuff3 == 0);
        TEST(alltypes.sng_enum == MyEnum_Truth);
        TEST(memcmp(alltypes.sng_fbytes, "3059", 4) == 0);

        TEST(alltypes.which_oneof == AllTypes_oneof_msg1_tag);
        TEST(strcmp(alltypes.oneof.oneof_msg1.substuff1, "4059") == 0);
        TEST(alltypes.oneof.oneof_msg1.substuff2 == 4059);
    }

    TEST(alltypes.req_limits.int32_min  == INT32_MIN);
    TEST(alltypes.req_limits.int32_max  == INT32_MAX);
    TEST(alltypes.req_limits.uint32_min == 0);
    TEST(alltypes.req_limits.uint32_max == UINT32_MAX);
    TEST(alltypes.req_limits.int64_min  == INT64_MIN);
    TEST(alltypes.req_limits.int64_max  == INT64_MAX);
    TEST(alltypes.req_limits.uint64_min == 0);
    TEST(alltypes.req_limits.uint64_max == UINT64_MAX);
    TEST(alltypes.req_limits.enum_min   == HugeEnum_Negative);
    TEST(alltypes.req_limits.enum_max   == HugeEnum_Positive);

    TEST(alltypes.end == 1099);

    return true;
}

int main(int argc, char **argv)
{
    uint8_t buffer[2048];
    size_t count;
    pb_istream_t stream;

    /* Whether to expect the optional values or the default values. */
    int mode = (argc > 1) ? atoi(argv[1]) : 0;

    /* Read the data into buffer */
    SET_BINARY_MODE(stdin);
    count = fread(buffer, 1, sizeof(buffer), stdin);

    /* Construct a pb_istream_t for reading from the buffer */
    stream = pb_istream_from_buffer(buffer, count);

    /* Decode and print out the stuff */
    if (!check_alltypes(&stream, mode))
    {
        printf("Parsing failed: %s\n", PB_GET_ERROR(&stream));
        return 1;
    } else {
        return 0;
    }
}
