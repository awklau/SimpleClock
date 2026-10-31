/* SPDX-License-Identifier: MIT */

#include "simpleclock/crc16.h"

void simpleclock_crc16_ccitt_init(simpleclock_crc16_context_t *context)
{
    context->value = (uint16_t)SIMPLECLOCK_CRC16_CCITT_INIT;
}

void simpleclock_crc16_ccitt_update(simpleclock_crc16_context_t *context, const uint8_t *data,
                                    size_t length)
{
    size_t index;

    for (index = 0u; index < length; ++index)
    {
        uint8_t bit;

        context->value ^= (uint16_t)((uint16_t)data[index] << 8u);
        for (bit = 0u; bit < 8u; ++bit)
        {
            if ((context->value & UINT16_C(0x8000)) != 0u)
            {
                context->value = (uint16_t)(context->value << 1u);
                context->value ^= (uint16_t)SIMPLECLOCK_CRC16_CCITT_POLYNOMIAL;
            }
            else
            {
                context->value = (uint16_t)(context->value << 1u);
            }
        }
    }
}

uint16_t simpleclock_crc16_ccitt_final(const simpleclock_crc16_context_t *context)
{
    return context->value;
}

uint16_t simpleclock_crc16_ccitt(const uint8_t *data, size_t length)
{
    simpleclock_crc16_context_t context;

    simpleclock_crc16_ccitt_init(&context);
    simpleclock_crc16_ccitt_update(&context, data, length);
    return simpleclock_crc16_ccitt_final(&context);
}
