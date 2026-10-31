/* SPDX-License-Identifier: MIT */

#include "simpleclock/tmp117.h"

#include <stddef.h>

uint16_t simpleclock_tmp117_decode_u16_be(uint8_t msb, uint8_t lsb)
{
    return (uint16_t)(((uint16_t)msb << 8u) | (uint16_t)lsb);
}

int16_t simpleclock_tmp117_decode_temperature_raw(uint8_t msb, uint8_t lsb)
{
    const uint16_t encoded = simpleclock_tmp117_decode_u16_be(msb, lsb);
    const int32_t signed_value = ((encoded & UINT16_C(0x8000)) != 0u)
                                     ? (int32_t)encoded - INT32_C(0x10000)
                                     : (int32_t)encoded;

    /* signed_value is in the complete int16_t range before conversion. */
    return (int16_t)signed_value;
}

bool simpleclock_tmp117_temperature_raw_valid(int16_t raw)
{
    return raw != SIMPLECLOCK_TMP117_TEMPERATURE_INVALID_RAW;
}

simpleclock_tmp117_status_t simpleclock_tmp117_temperature_milli_c(int16_t raw, int32_t *milli_c)
{
    int32_t scaled;

    if (milli_c == NULL)
    {
        return SIMPLECLOCK_TMP117_STATUS_INVALID_ARGUMENT;
    }
    if (!simpleclock_tmp117_temperature_raw_valid(raw))
    {
        return SIMPLECLOCK_TMP117_STATUS_INVALID_TEMPERATURE;
    }

    /* The product is bounded by INT16_MAX * 125 and fits in int32_t. */
    scaled = (int32_t)raw * INT32_C(125);
    *milli_c = scaled / INT32_C(16);
    return SIMPLECLOCK_TMP117_STATUS_OK;
}

bool simpleclock_tmp117_data_ready(uint16_t configuration)
{
    return (configuration & SIMPLECLOCK_TMP117_CONFIG_DATA_READY_MASK) != 0u;
}

bool simpleclock_tmp117_device_id_valid(uint16_t device_id)
{
    return simpleclock_tmp117_validate_device_id(device_id) == SIMPLECLOCK_TMP117_STATUS_OK;
}

uint8_t simpleclock_tmp117_device_revision(uint16_t device_id)
{
    return (uint8_t)(device_id >> 12u);
}

simpleclock_tmp117_status_t simpleclock_tmp117_validate_device_id(uint16_t device_id)
{
    if ((device_id & SIMPLECLOCK_TMP117_DEVICE_ID_MASK) != SIMPLECLOCK_TMP117_DEVICE_ID_VALUE)
    {
        return SIMPLECLOCK_TMP117_STATUS_INVALID_DEVICE_ID;
    }
    return SIMPLECLOCK_TMP117_STATUS_OK;
}
