/* SPDX-License-Identifier: MIT */
/* Separate mixed ASCII/SBF input and validate whole frames with fixed prefix storage. */
#include "simpleclock/sbf.h"
#include <stddef.h>

uint16_t sc_sbf_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint32_t sc_sbf_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* MeasEpoch has nested variable sub-blocks. Check their bounds as bytes pass,
 * including N2 beyond the retained prefix; never allocate the full frame.
 * mosaic-G5 Firmware v1.1.0 Reference Guide pp. 195-198: common header20, Type1 minimum20, Type2 minimum12. */
static void validate_meas_epoch_byte(sc_sbf_t *parser, uint8_t byte)
{
    if (parser->used == 8U)
    {
        const uint16_t id_rev = sc_sbf_u16(parser->prefix + 4);
        parser->meas_check =
            (id_rev & SC_SBF_ID_MASK) == 4027U && (id_rev >> SC_SBF_REVISION_SHIFT) <= 1U;
        parser->meas_bad = parser->meas_check && parser->length < 20U;
        parser->meas_next = 20U;
        parser->meas_remaining = 0U;
    }
    if (!parser->meas_check || parser->meas_bad)
    {
        return;
    }
    if (parser->used == 20U)
    {
        parser->meas_remaining = parser->prefix[14];
        if (parser->meas_remaining != 0U && parser->prefix[15] < 20U)
        {
            parser->meas_bad = true;
        }
    }
    else if (parser->meas_remaining != 0U && parser->used == parser->meas_next + 20U)
    {
        if (byte != 0U && parser->prefix[16] < 12U)
        {
            parser->meas_bad = true;
        }
        parser->meas_next += parser->prefix[15] + (uint32_t)byte * parser->prefix[16];
        parser->meas_remaining--;
        if (parser->meas_next > parser->length)
        {
            parser->meas_bad = true;
        }
    }
}

void sc_sbf_expire(sc_sbf_t *parser, uint32_t now_ms, uint32_t baud)
{
    if (parser->used == 0U || baud == 0U)
    {
        return;
    }
    /* 8N1 needs 10 bits/byte: bytes * 10 * 1000 / baud gives wire milliseconds.
     * Add scheduling slack and independently reject a 200 ms inter-byte stall. */
    const uint32_t allowed_ms = 200U + (uint32_t)parser->length * 10000U / baud;
    if ((uint32_t)(now_ms - parser->last_byte_ms) > 200U ||
        (uint32_t)(now_ms - parser->start_ms) > allowed_ms)
    {
        parser->timeouts++;
        parser->used = 0;
        parser->length = 0;
    }
}

void sc_sbf_feed(sc_sbf_t *parser, const uint8_t *bytes, unsigned count, uint32_t now_ms,
                 sc_sbf_frame_fn frame, sc_sbf_ascii_fn ascii, void *context)
{
    for (unsigned i = 0; i < count; i++)
    {
        const uint8_t byte = bytes[i];
        if (parser->used == 1U && byte != '@')
        {
            if (ascii != NULL)
            {
                ascii(context, '$');
            }
            parser->used = 0;
        }
        if (parser->used == 0U)
        {
            if (byte == '$')
            {
                parser->prefix[0] = byte;
                parser->used = 1;
                parser->length = 0;
                parser->start_ms = now_ms;
                parser->last_byte_ms = now_ms;
            }
            else if (ascii != NULL)
            {
                ascii(context, byte);
            }
            continue;
        }
        if (parser->used < SC_SBF_PREFIX)
        {
            parser->prefix[parser->used] = byte;
        }
        if (parser->used == 4U)
        {
            simpleclock_crc16_ccitt_init(&parser->crc);
        }
        if (parser->used >= 4U)
        {
            simpleclock_crc16_ccitt_update(&parser->crc, &byte, 1);
        }
        parser->used++;
        parser->last_byte_ms = now_ms;
        if (parser->used == 8U)
        {
            parser->length = sc_sbf_u16(parser->prefix + 6);
            if (parser->length < 8U || (parser->length & 3U) != 0U)
            {
                parser->length_errors++;
                parser->used = 0;
                parser->length = 0;
                continue;
            }
        }
        validate_meas_epoch_byte(parser, byte);
        if (parser->length != 0U && parser->used == parser->length)
        {
            if (parser->crc.value == sc_sbf_u16(parser->prefix + 2))
            {
                const bool valid =
                    !(parser->meas_check && (parser->meas_bad || parser->meas_remaining != 0U ||
                                             parser->meas_next > parser->length));
                if (!valid)
                {
                    parser->structure_errors++;
                }
                parser->frames++;
                if (frame != NULL)
                {
                    frame(context, parser->prefix, parser->length, valid);
                }
            }
            else
            {
                parser->crc_errors++;
            }
            parser->used = 0;
            parser->length = 0;
        }
    }
}
