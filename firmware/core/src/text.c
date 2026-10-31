/* SPDX-License-Identifier: MIT */
/* Bounded integer/hex telemetry formatting without printf, allocation or floating point. */
#include "simpleclock/text.h"

void sc_text_i32(sc_text_t *text, int32_t value)
{
    if (value < 0)
    {
        sc_text_string(text, "-");
    }
    sc_text_u32(text, value < 0 ? 0U - (uint32_t)value : (uint32_t)value);
}

void sc_text_q16(sc_text_t *text, int32_t value)
{
    const uint32_t magnitude = value < 0 ? 0U - (uint32_t)value : (uint32_t)value;
    const uint32_t milli = (uint32_t)(((uint64_t)magnitude * 1000U + 32768U) / 65536U);
    if (value < 0 && milli)
    {
        sc_text_string(text, "-");
    }
    sc_text_u32(text, milli / 1000U);
    sc_text_string(text, ".");
    const uint32_t fraction = milli % 1000U;
    if (fraction < 100U)
    {
        sc_text_string(text, "0");
    }
    if (fraction < 10U)
    {
        sc_text_string(text, "0");
    }
    sc_text_u32(text, fraction);
}

static void append_character(sc_text_t *text, char character)
{
    if (!text->ok)
    {
        return;
    }
    if (text->length + 1U >= text->capacity)
    {
        text->ok = false;
        return;
    }
    text->data[text->length++] = character;
    text->data[text->length] = '\0';
}

void sc_text_init(sc_text_t *text, char *data, size_t capacity)
{
    text->data = data;
    text->capacity = capacity;
    text->length = 0;
    text->ok = data != NULL && capacity != 0U;
    if (text->ok)
    {
        data[0] = '\0';
    }
}

void sc_text_string(sc_text_t *text, const char *value)
{
    while (text->ok && *value != '\0')
    {
        append_character(text, *value++);
    }
}

void sc_text_u32(sc_text_t *text, uint32_t value)
{
    char digits[10];
    unsigned count = 0;
    do
    {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count != 0U)
    {
        append_character(text, digits[--count]);
    }
}

void sc_text_hex(sc_text_t *text, uint32_t value, unsigned digits)
{
    static const char hex[] = "0123456789ABCDEF";
    if (digits == 0U || digits > 8U)
    {
        text->ok = false;
        return;
    }
    while (digits != 0U)
    {
        digits--;
        append_character(text, hex[(value >> (digits * 4U)) & 15U]);
    }
}

void sc_text_u64(sc_text_t *text, uint64_t value)
{
    char digits[20];
    unsigned count = 0;
    do
    {
        digits[count++] = (char)('0' + value % 10U);
        value /= 10U;
    } while (value != 0U);
    while (count != 0U)
    {
        append_character(text, digits[--count]);
    }
}

void sc_text_i64(sc_text_t *text, int64_t value)
{
    if (value < 0)
    {
        sc_text_string(text, "-");
    }
    sc_text_u64(text, value < 0 ? UINT64_C(0) - (uint64_t)value : (uint64_t)value);
}

void sc_text_temperature(sc_text_t *text, int16_t raw)
{
    const uint32_t magnitude = raw < 0 ? (uint32_t)(-(int32_t)raw) : (uint32_t)raw;
    if (raw < 0)
    {
        sc_text_string(text, "-");
    }
    sc_text_u32(text, magnitude / 128U);
    sc_text_string(text, ".");
    /* Fraction < 128; multiplication stays below 10^7 and is exact. */
    uint32_t fraction = (magnitude % 128U) * 78125U;
    for (uint32_t divisor = 1000000U; divisor; divisor /= 10U)
    {
        sc_text_u32(text, fraction / divisor);
        fraction %= divisor;
    }
}
