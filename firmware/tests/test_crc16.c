/* SPDX-License-Identifier: MIT */

#include "simpleclock/crc16.h"

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int check_crc(const char *name, const uint8_t *data, size_t length, uint16_t expected)
{
    uint16_t actual = simpleclock_crc16_ccitt(data, length);

    if (actual != expected)
    {
        (void)fprintf(stderr, "%s: expected 0x%04X, got 0x%04X\n", name, (unsigned int)expected,
                      (unsigned int)actual);
        return 0;
    }
    return 1;
}

static int check_incremental(const uint8_t *data, size_t length, uint16_t expected)
{
    size_t split;

    for (split = 0u; split <= length; ++split)
    {
        simpleclock_crc16_context_t context;

        simpleclock_crc16_ccitt_init(&context);
        simpleclock_crc16_ccitt_update(&context, data, split);
        simpleclock_crc16_ccitt_update(&context, data + split, length - split);
        if (simpleclock_crc16_ccitt_final(&context) != expected)
        {
            (void)fprintf(stderr, "incremental split %lu failed\n", (unsigned long)split);
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    static const uint8_t binary[] = {0x00u, 0x01u, 0x02u, 0x7Fu, 0x80u, 0xFFu};
    static const char text[] = "123456789";
    /* Independently verified with Python binascii.crc_hqx(data, 0). */
    const uint16_t binary_crc = UINT16_C(0xB668);
    int passed = 1;

    passed &= check_crc("empty", NULL, 0u, UINT16_C(0x0000));
    passed &= check_crc("123456789", (const uint8_t *)text, strlen(text), UINT16_C(0x31C3));
    passed &= check_crc("binary", binary, sizeof(binary), binary_crc);
    passed &= check_incremental(binary, sizeof(binary), binary_crc);

    return passed ? 0 : 1;
}
