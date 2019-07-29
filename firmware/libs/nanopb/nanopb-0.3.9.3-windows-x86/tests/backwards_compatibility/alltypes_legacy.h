/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.0-dev at Tue Aug 19 17:53:24 2014. */

#ifndef PB_ALLTYPES_LEGACY_H_INCLUDED
#define PB_ALLTYPES_LEGACY_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
typedef enum _HugeEnum {
    HugeEnum_Negative = -2147483647,
    HugeEnum_Positive = 2147483647
} HugeEnum;

typedef enum _MyEnum {
    MyEnum_Zero = 0,
    MyEnum_First = 1,
    MyEnum_Second = 2,
    MyEnum_Truth = 42
} MyEnum;

/* Struct definitions */
typedef struct _EmptyMessage {
    uint8_t dummy_field;
} EmptyMessage;

typedef struct _Limits {
    int32_t int32_min;
    int32_t int32_max;
    uint32_t uint32_min;
    uint32_t uint32_max;
    int64_t int64_min;
    int64_t int64_max;
    uint64_t uint64_min;
    uint64_t uint64_max;
    HugeEnum enum_min;
    HugeEnum enum_max;
} Limits;

typedef struct _SubMessage {
    char substuff1[16];
    int32_t substuff2;
    bool has_substuff3;
    uint32_t substuff3;
} SubMessage;

typedef PB_BYTES_ARRAY_T(16) AllTypes_req_bytes_t;

typedef PB_BYTES_ARRAY_T(16) AllTypes_rep_bytes_t;

typedef PB_BYTES_ARRAY_T(16) AllTypes_opt_bytes_t;

typedef struct _AllTypes {
    int32_t req_int32;
    int64_t req_int64;
    uint32_t req_uint32;
    uint64_t req_uint64;
    int32_t req_sint32;
    int64_t req_sint64;
    bool req_bool;
    uint32_t req_fixed32;
    int32_t req_sfixed32;
    float req_float;
    uint64_t req_fixed64;
    int64_t req_sfixed64;
    double req_double;
    char req_string[16];
    AllTypes_req_bytes_t req_bytes;
    SubMessage req_submsg;
    MyEnum req_enum;
    pb_size_t rep_int32_count;
    int32_t rep_int32[5];
    pb_size_t rep_int64_count;
    int64_t rep_int64[5];
    pb_size_t rep_uint32_count;
    uint32_t rep_uint32[5];
    pb_size_t rep_uint64_count;
    uint64_t rep_uint64[5];
    pb_size_t rep_sint32_count;
    int32_t rep_sint32[5];
    pb_size_t rep_sint64_count;
    int64_t rep_sint64[5];
    pb_size_t rep_bool_count;
    bool rep_bool[5];
    pb_size_t rep_fixed32_count;
    uint32_t rep_fixed32[5];
    pb_size_t rep_sfixed32_count;
    int32_t rep_sfixed32[5];
    pb_size_t rep_float_count;
    float rep_float[5];
    pb_size_t rep_fixed64_count;
    uint64_t rep_fixed64[5];
    pb_size_t rep_sfixed64_count;
    int64_t rep_sfixed64[5];
    pb_size_t rep_double_count;
    double rep_double[5];
    pb_size_t rep_string_count;
    char rep_string[5][16];
    pb_size_t rep_bytes_count;
    AllTypes_rep_bytes_t rep_bytes[5];
    pb_size_t rep_submsg_count;
    SubMessage rep_submsg[5];
    pb_size_t rep_enum_count;
    MyEnum rep_enum[5];
    bool has_opt_int32;
    int32_t opt_int32;
    bool has_opt_int64;
    int64_t opt_int64;
    bool has_opt_uint32;
    uint32_t opt_uint32;
    bool has_opt_uint64;
    uint64_t opt_uint64;
    bool has_opt_sint32;
    int32_t opt_sint32;
    bool has_opt_sint64;
    int64_t opt_sint64;
    bool has_opt_bool;
    bool opt_bool;
    bool has_opt_fixed32;
    uint32_t opt_fixed32;
    bool has_opt_sfixed32;
    int32_t opt_sfixed32;
    bool has_opt_float;
    float opt_float;
    bool has_opt_fixed64;
    uint64_t opt_fixed64;
    bool has_opt_sfixed64;
    int64_t opt_sfixed64;
    bool has_opt_double;
    double opt_double;
    bool has_opt_string;
    char opt_string[16];
    bool has_opt_bytes;
    AllTypes_opt_bytes_t opt_bytes;
    bool has_opt_submsg;
    SubMessage opt_submsg;
    bool has_opt_enum;
    MyEnum opt_enum;
    int32_t end;
    pb_extension_t *extensions;
} AllTypes;

/* Default values for struct fields */
extern const char SubMessage_substuff1_default[16];
extern const int32_t SubMessage_substuff2_default;
extern const uint32_t SubMessage_substuff3_default;
extern const int32_t Limits_int32_min_default;
extern const int32_t Limits_int32_max_default;
extern const uint32_t Limits_uint32_min_default;
extern const uint32_t Limits_uint32_max_default;
extern const int64_t Limits_int64_min_default;
extern const int64_t Limits_int64_max_default;
extern const uint64_t Limits_uint64_min_default;
extern const uint64_t Limits_uint64_max_default;
extern const HugeEnum Limits_enum_min_default;
extern const HugeEnum Limits_enum_max_default;
extern const int32_t AllTypes_opt_int32_default;
extern const int64_t AllTypes_opt_int64_default;
extern const uint32_t AllTypes_opt_uint32_default;
extern const uint64_t AllTypes_opt_uint64_default;
extern const int32_t AllTypes_opt_sint32_default;
extern const int64_t AllTypes_opt_sint64_default;
extern const bool AllTypes_opt_bool_default;
extern const uint32_t AllTypes_opt_fixed32_default;
extern const int32_t AllTypes_opt_sfixed32_default;
extern const float AllTypes_opt_float_default;
extern const uint64_t AllTypes_opt_fixed64_default;
extern const int64_t AllTypes_opt_sfixed64_default;
extern const double AllTypes_opt_double_default;
extern const char AllTypes_opt_string_default[16];
extern const AllTypes_opt_bytes_t AllTypes_opt_bytes_default;
extern const MyEnum AllTypes_opt_enum_default;

/* Initializer values for message structs */
#define SubMessage_init_default                  {"1", 2, false, 3u}
#define EmptyMessage_init_default                {0}
#define Limits_init_default                      {2147483647, -2147483647, 4294967295u, 0u, 9223372036854775807ll, -9223372036854775807ll, 18446744073709551615ull, 0ull, HugeEnum_Positive, HugeEnum_Negative}
#define AllTypes_init_default                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "", {0, {0}}, SubMessage_init_default, (MyEnum)0, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {"", "", "", "", ""}, 0, {{0, {0}}, {0, {0}}, {0, {0}}, {0, {0}}, {0, {0}}}, 0, {SubMessage_init_default, SubMessage_init_default, SubMessage_init_default, SubMessage_init_default, SubMessage_init_default}, 0, {(MyEnum)0, (MyEnum)0, (MyEnum)0, (MyEnum)0, (MyEnum)0}, false, 4041, false, 4042ll, false, 4043u, false, 4044ull, false, 4045, false, 4046, false, false, false, 4048u, false, 4049, false, 4050, false, 4051ull, false, 4052ll, false, 4053, false, "4054", false, {4, {0x34,0x30,0x35,0x35}}, false, SubMessage_init_default, false, MyEnum_Second, 0, NULL}
#define SubMessage_init_zero                     {"", 0, false, 0}
#define EmptyMessage_init_zero                   {0}
#define Limits_init_zero                         {0, 0, 0, 0, 0, 0, 0, 0, (HugeEnum)0, (HugeEnum)0}
#define AllTypes_init_zero                       {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "", {0, {0}}, SubMessage_init_zero, (MyEnum)0, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {0, 0, 0, 0, 0}, 0, {"", "", "", "", ""}, 0, {{0, {0}}, {0, {0}}, {0, {0}}, {0, {0}}, {0, {0}}}, 0, {SubMessage_init_zero, SubMessage_init_zero, SubMessage_init_zero, SubMessage_init_zero, SubMessage_init_zero}, 0, {(MyEnum)0, (MyEnum)0, (MyEnum)0, (MyEnum)0, (MyEnum)0}, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, "", false, {0, {0}}, false, SubMessage_init_zero, false, (MyEnum)0, 0, NULL}

/* Field tags (for use in manual encoding/decoding) */
#define Limits_int32_min_tag                     1
#define Limits_int32_max_tag                     2
#define Limits_uint32_min_tag                    3
#define Limits_uint32_max_tag                    4
#define Limits_int64_min_tag                     5
#define Limits_int64_max_tag                     6
#define Limits_uint64_min_tag                    7
#define Limits_uint64_max_tag                    8
#define Limits_enum_min_tag                      9
#define Limits_enum_max_tag                      10
#define SubMessage_substuff1_tag                 1
#define SubMessage_substuff2_tag                 2
#define SubMessage_substuff3_tag                 3
#define AllTypes_req_int32_tag                   1
#define AllTypes_req_int64_tag                   2
#define AllTypes_req_uint32_tag                  3
#define AllTypes_req_uint64_tag                  4
#define AllTypes_req_sint32_tag                  5
#define AllTypes_req_sint64_tag                  6
#define AllTypes_req_bool_tag                    7
#define AllTypes_req_fixed32_tag                 8
#define AllTypes_req_sfixed32_tag                9
#define AllTypes_req_float_tag                   10
#define AllTypes_req_fixed64_tag                 11
#define AllTypes_req_sfixed64_tag                12
#define AllTypes_req_double_tag                  13
#define AllTypes_req_string_tag                  14
#define AllTypes_req_bytes_tag                   15
#define AllTypes_req_submsg_tag                  16
#define AllTypes_req_enum_tag                    17
#define AllTypes_rep_int32_tag                   21
#define AllTypes_rep_int64_tag                   22
#define AllTypes_rep_uint32_tag                  23
#define AllTypes_rep_uint64_tag                  24
#define AllTypes_rep_sint32_tag                  25
#define AllTypes_rep_sint64_tag                  26
#define AllTypes_rep_bool_tag                    27
#define AllTypes_rep_fixed32_tag                 28
#define AllTypes_rep_sfixed32_tag                29
#define AllTypes_rep_float_tag                   30
#define AllTypes_rep_fixed64_tag                 31
#define AllTypes_rep_sfixed64_tag                32
#define AllTypes_rep_double_tag                  33
#define AllTypes_rep_string_tag                  34
#define AllTypes_rep_bytes_tag                   35
#define AllTypes_rep_submsg_tag                  36
#define AllTypes_rep_enum_tag                    37
#define AllTypes_opt_int32_tag                   41
#define AllTypes_opt_int64_tag                   42
#define AllTypes_opt_uint32_tag                  43
#define AllTypes_opt_uint64_tag                  44
#define AllTypes_opt_sint32_tag                  45
#define AllTypes_opt_sint64_tag                  46
#define AllTypes_opt_bool_tag                    47
#define AllTypes_opt_fixed32_tag                 48
#define AllTypes_opt_sfixed32_tag                49
#define AllTypes_opt_float_tag                   50
#define AllTypes_opt_fixed64_tag                 51
#define AllTypes_opt_sfixed64_tag                52
#define AllTypes_opt_double_tag                  53
#define AllTypes_opt_string_tag                  54
#define AllTypes_opt_bytes_tag                   55
#define AllTypes_opt_submsg_tag                  56
#define AllTypes_opt_enum_tag                    57
#define AllTypes_end_tag                         99

/* Struct field encoding specification for nanopb */
extern const pb_field_t SubMessage_fields[4];
extern const pb_field_t EmptyMessage_fields[1];
extern const pb_field_t Limits_fields[11];
extern const pb_field_t AllTypes_fields[54];

/* Maximum encoded size of messages (where known) */
#define SubMessage_size                          34
#define EmptyMessage_size                        0
#define Limits_size                              90
#define AllTypes_size                            1362

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
