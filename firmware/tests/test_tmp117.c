/* SPDX-License-Identifier: MIT */

#include "simpleclock/tmp117.h"

#include <stdint.h>
#include <stdio.h>

static int check_bool(const char *name, bool actual, bool expected)
{
    if (actual != expected)
    {
        (void)fprintf(stderr, "%s: expected %d, got %d\n", name, expected ? 1 : 0, actual ? 1 : 0);
        return 0;
    }
    return 1;
}

static int check_u16(const char *name, uint16_t actual, uint16_t expected)
{
    if (actual != expected)
    {
        (void)fprintf(stderr, "%s: expected 0x%04X, got 0x%04X\n", name, (unsigned int)expected,
                      (unsigned int)actual);
        return 0;
    }
    return 1;
}

static int check_i16(const char *name, int16_t actual, int16_t expected)
{
    if (actual != expected)
    {
        (void)fprintf(stderr, "%s: expected %d, got %d\n", name, (int)expected, (int)actual);
        return 0;
    }
    return 1;
}

static int check_i32(const char *name, int32_t actual, int32_t expected)
{
    if (actual != expected)
    {
        (void)fprintf(stderr, "%s: expected %ld, got %ld\n", name, (long)expected, (long)actual);
        return 0;
    }
    return 1;
}

static int check_status(const char *name, simpleclock_tmp117_status_t actual,
                        simpleclock_tmp117_status_t expected)
{
    if (actual != expected)
    {
        (void)fprintf(stderr, "%s: expected status %d, got %d\n", name, (int)expected, (int)actual);
        return 0;
    }
    return 1;
}

int main(void)
{
    int passed = 1;
    int32_t milli_c = 0;

    passed &=
        check_u16("big endian", simpleclock_tmp117_decode_u16_be(0x12u, 0x34u), UINT16_C(0x1234));
    passed &= check_i16("positive raw", simpleclock_tmp117_decode_temperature_raw(0x19u, 0x00u),
                        INT16_C(6400));
    passed &= check_i16("negative raw", simpleclock_tmp117_decode_temperature_raw(0xE7u, 0x00u),
                        INT16_C(-6400));
    passed &= check_i16("minimum raw decode",
                        simpleclock_tmp117_decode_temperature_raw(0x80u, 0x00u), INT16_MIN);
    passed &= check_i16("maximum raw decode",
                        simpleclock_tmp117_decode_temperature_raw(0x7Fu, 0xFFu), INT16_MAX);

    passed &= check_bool("startup raw invalid", simpleclock_tmp117_temperature_raw_valid(INT16_MIN),
                         false);
    passed &=
        check_bool("zero raw valid", simpleclock_tmp117_temperature_raw_valid(INT16_C(0)), true);
    passed &=
        check_bool("maximum raw valid", simpleclock_tmp117_temperature_raw_valid(INT16_MAX), true);

    passed &= check_status("25 C conversion",
                           simpleclock_tmp117_temperature_milli_c(INT16_C(3200), &milli_c),
                           SIMPLECLOCK_TMP117_STATUS_OK);
    passed &= check_i32("25 C milli", milli_c, INT32_C(25000));
    passed &= check_status("negative conversion",
                           simpleclock_tmp117_temperature_milli_c(INT16_C(-321), &milli_c),
                           SIMPLECLOCK_TMP117_STATUS_OK);
    passed &= check_i32("negative truncation", milli_c, INT32_C(-2507));
    passed &= check_status("positive conversion",
                           simpleclock_tmp117_temperature_milli_c(INT16_C(321), &milli_c),
                           SIMPLECLOCK_TMP117_STATUS_OK);
    passed &= check_i32("positive truncation", milli_c, INT32_C(2507));
    passed &= check_status("minimum raw conversion",
                           simpleclock_tmp117_temperature_milli_c(INT16_MIN + 1, &milli_c),
                           SIMPLECLOCK_TMP117_STATUS_OK);
    passed &= check_i32("minimum raw milli", milli_c, INT32_C(-255992));
    passed &= check_status("maximum raw conversion",
                           simpleclock_tmp117_temperature_milli_c(INT16_MAX, &milli_c),
                           SIMPLECLOCK_TMP117_STATUS_OK);
    passed &= check_i32("maximum raw milli", milli_c, INT32_C(255992));
    passed &= check_status("invalid startup conversion",
                           simpleclock_tmp117_temperature_milli_c(INT16_MIN, &milli_c),
                           SIMPLECLOCK_TMP117_STATUS_INVALID_TEMPERATURE);
    passed &= check_status("null output", simpleclock_tmp117_temperature_milli_c(0, NULL),
                           SIMPLECLOCK_TMP117_STATUS_INVALID_ARGUMENT);

    passed &=
        check_bool("data ready clear", simpleclock_tmp117_data_ready(UINT16_C(0x0000)), false);
    passed &= check_bool("data ready set", simpleclock_tmp117_data_ready(UINT16_C(0x2000)), true);
    passed &= check_bool("data ready preserves other bits",
                         simpleclock_tmp117_data_ready(UINT16_C(0xA000)), true);

    passed &= check_bool("device id revision zero",
                         simpleclock_tmp117_device_id_valid(UINT16_C(0x0117)), true);
    passed &= check_bool("device id revision A",
                         simpleclock_tmp117_device_id_valid(UINT16_C(0xA117)), true);
    passed &= check_u16("device revision", simpleclock_tmp117_device_revision(UINT16_C(0xA117)),
                        UINT16_C(0x000A));
    passed &= check_status("device id status valid",
                           simpleclock_tmp117_validate_device_id(UINT16_C(0xA117)),
                           SIMPLECLOCK_TMP117_STATUS_OK);
    passed &= check_bool("device id wrong product",
                         simpleclock_tmp117_device_id_valid(UINT16_C(0xA116)), false);
    passed &= check_status("device id status invalid",
                           simpleclock_tmp117_validate_device_id(UINT16_C(0xA116)),
                           SIMPLECLOCK_TMP117_STATUS_INVALID_DEVICE_ID);

    return passed ? 0 : 1;
}
