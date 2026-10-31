/* SPDX-License-Identifier: MIT */

#ifndef SIMPLECLOCK_TMP117_H
#define SIMPLECLOCK_TMP117_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    enum
    {
        SIMPLECLOCK_TMP117_I2C_ADDRESS = 0x48u,
        SIMPLECLOCK_TMP117_REG_TEMPERATURE = 0x00u,
        SIMPLECLOCK_TMP117_REG_CONFIGURATION = 0x01u,
        SIMPLECLOCK_TMP117_REG_DEVICE_ID = 0x0Fu,
        SIMPLECLOCK_TMP117_CONFIG_DATA_READY_MASK = UINT16_C(0x2000),
        SIMPLECLOCK_TMP117_DEVICE_ID_MASK = UINT16_C(0x0FFF),
        SIMPLECLOCK_TMP117_DEVICE_ID_VALUE = UINT16_C(0x0117),
        /* 0x8000 is the raw value used by the device before its first conversion. */
        SIMPLECLOCK_TMP117_TEMPERATURE_INVALID_RAW = INT16_MIN
    };

    typedef enum
    {
        SIMPLECLOCK_TMP117_STATUS_OK = 0,
        SIMPLECLOCK_TMP117_STATUS_INVALID_ARGUMENT,
        SIMPLECLOCK_TMP117_STATUS_INVALID_TEMPERATURE,
        SIMPLECLOCK_TMP117_STATUS_INVALID_DEVICE_ID
    } simpleclock_tmp117_status_t;

    /* Decode a TMP117 register returned MSB first on the I2C wire. */
    uint16_t simpleclock_tmp117_decode_u16_be(uint8_t msb, uint8_t lsb);

    /* Decode the signed two's-complement temperature register without scaling. */
    int16_t simpleclock_tmp117_decode_temperature_raw(uint8_t msb, uint8_t lsb);

    /* Return true if a temperature sample is usable after the first conversion. */
    bool simpleclock_tmp117_temperature_raw_valid(int16_t raw);

    /*
 * Convert a valid raw sample to milli-degrees Celsius.
 *
 * Each raw count is exactly 1/128 degree Celsius. The integer result is
 * truncated toward zero: milli-C = raw * 1000 / 128 = raw * 125 / 16.
 * The output pointer must be non-NULL and raw must not be the startup value.
 */
    simpleclock_tmp117_status_t simpleclock_tmp117_temperature_milli_c(int16_t raw,
                                                                       int32_t *milli_c);

    /* Return the TMP117 Data_Ready status from a decoded configuration register. */
    /*
 * This is a snapshot only: the device may clear Data_Ready when configuration
 * or temperature is read. The transport layer must define the read sequence
 * when it needs a freshness decision.
 */
    bool simpleclock_tmp117_data_ready(uint16_t configuration);

    /*
 * Validate a decoded device-ID register. The lower 12 bits identify TMP117;
 * the upper nibble is the silicon revision and is intentionally preserved by
 * the caller rather than compared here.
 */
    bool simpleclock_tmp117_device_id_valid(uint16_t device_id);

    /* Return the upper-nibble silicon revision from a decoded device ID. */
    uint8_t simpleclock_tmp117_device_revision(uint16_t device_id);

    /* Validate the product portion of a decoded device ID with a status result. */
    simpleclock_tmp117_status_t simpleclock_tmp117_validate_device_id(uint16_t device_id);

#ifdef __cplusplus
}
#endif

#endif
