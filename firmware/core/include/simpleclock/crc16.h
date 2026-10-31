/* SPDX-License-Identifier: MIT */

#ifndef SIMPLECLOCK_CRC16_H
#define SIMPLECLOCK_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    enum
    {
        SIMPLECLOCK_CRC16_CCITT_INIT = 0x0000u,
        SIMPLECLOCK_CRC16_CCITT_POLYNOMIAL = 0x1021u
    };

    typedef struct
    {
        uint16_t value;
    } simpleclock_crc16_context_t;

    /*
 * Initialize a CRC-16/CCITT context.  Parameters are polynomial 0x1021,
 * initial value 0, no input or output reflection, and final XOR 0.
 *
 * For SBF packets, include ID, Length, and all remaining block bytes (offset
 * 4 through the end). Exclude only the two sync bytes and stored two-byte CRC.
 * The context argument must be non-NULL.
 */
    void simpleclock_crc16_ccitt_init(simpleclock_crc16_context_t *context);

    /*
 * Incorporate bytes into a context.
 *
 * Preconditions: context is non-NULL.  If length is non-zero, data points to
 * at least length readable bytes.  When length is zero, data may be NULL.
 */
    void simpleclock_crc16_ccitt_update(simpleclock_crc16_context_t *context, const uint8_t *data,
                                        size_t length);

    /* Return the current CRC value; context must be non-NULL. */
    uint16_t simpleclock_crc16_ccitt_final(const simpleclock_crc16_context_t *context);

    /*
 * Compute a CRC in one call.  The update preconditions apply to data and
 * length; data may be NULL only when length is zero.
 */
    uint16_t simpleclock_crc16_ccitt(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif
