/* SPDX-License-Identifier: MIT */
/* Pure SiT5801 register codecs and paced read scheduling; no bus or oscillator writes. */
#include "simpleclock/sit5801.h"
#include <math.h>
#include <stddef.h>

bool sit5801_fcw_encode(int64_t q, uint16_t words[3])
{
    if (words == NULL || q < SIT5801_FCW_MIN || q > SIT5801_FCW_MAX)
    {
        return false;
    }
    const uint64_t bits = (uint64_t)q & ((UINT64_C(1) << 39) - 1U);
    words[0] = (uint16_t)(bits >> 23);
    words[1] = (uint16_t)(bits >> 7);
    words[2] = (uint16_t)((bits & 0x7fU) << 9);
    return true;
}

int64_t sit5801_fcw_decode(const uint16_t words[3])
{
    const uint64_t bits =
        ((uint64_t)words[0] << 23) | ((uint64_t)words[1] << 7) | ((uint64_t)words[2] >> 9);
    /* bits fits int64_t; subtraction implements sign extension portably. */
    return (int64_t)bits - ((bits & (UINT64_C(1) << 38)) != 0U ? (INT64_C(1) << 39) : 0);
}

bool sit5801_ppb_to_fcw(double ppb, int64_t *q)
{
    if (q == NULL || !isfinite(ppb) || ppb < -800000.0 || ppb > 800000.0)
    {
        return false;
    }
    const double scaled = ppb * ((double)SIT5801_FCW_MAX / 800000.0);
    /* Nearest, ties away from zero; conversion is within int64_t range. */
    *q = (int64_t)(scaled + (scaled < 0.0 ? -0.5 : 0.5));
    return true;
}

double sit5801_fcw_to_ppb(int64_t q)
{
    return (double)q * (800000.0 / (double)SIT5801_FCW_MAX);
}

double sit5801_clip_ppb(uint16_t clip_register)
{
    const uint16_t clip = clip_register & 0x1fffU;
    return clip == 0U ? 800000.0 : 800000.0 * (double)clip / 8192.0;
}

uint32_t sit5801_power_code(uint16_t high, uint16_t low)
{
    return ((uint32_t)high << 2) | ((uint32_t)low >> 14);
}

bool sit5801_probe_begin(sit5801_probe_t *probe, uint32_t now_ms, uint8_t *reg)
{
    static const uint8_t registers[8] = {0x2a, 0x00, 0x13, 0x0c, 0x0d, 0x0e, 0x1e, 0x1f};
    if (probe == NULL || reg == NULL || probe->pending)
    {
        return false;
    }
    if (probe->started && (uint32_t)(now_ms - probe->completed_ms) < SIT5801_READ_GAP_MS)
    {
        return false;
    }
    if (probe->index == 0U)
    {
        probe->valid_mask = 0U;
    }
    *reg = registers[probe->index];
    probe->pending = true;
    probe->started = true;
    return true;
}

void sit5801_probe_complete(sit5801_probe_t *probe, uint32_t now_ms, bool success,
                            const uint8_t bytes[2])
{
    if (probe == NULL || !probe->pending)
    {
        return;
    }
    if (success && bytes != NULL)
    {
        probe->wire[probe->index] = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
        probe->valid_mask |= (uint8_t)(1U << probe->index);
    }
    else
    {
        probe->wire[probe->index] = 0U;
        probe->errors++;
    }
    probe->index++;
    if (probe->index == 8U)
    {
        probe->index = 0U;
        probe->sequence++;
    }
    probe->completed_ms = now_ms;
    probe->pending = false;
}

uint16_t sit5801_probe_word(const sit5801_probe_t *probe, unsigned index, unsigned order)
{
    if (probe == NULL || index >= 8U || order == 0U || order > 2U)
    {
        return 0U;
    }
    const uint16_t raw = probe->wire[index];
    return order == 1U ? raw : (uint16_t)((raw << 8) | (raw >> 8));
}

unsigned sit5801_probe_word_order(const sit5801_probe_t *probe)
{
    if (probe == NULL || probe->valid_mask != 0xffU || probe->index != 0U)
    {
        return 0U;
    }
    unsigned candidate = 0U;
    for (unsigned order = 1U; order <= 2U; order++)
    {
        if ((sit5801_probe_word(probe, 0U, order) & 0xff00U) == 0xd000U &&
            (sit5801_probe_word(probe, 1U, order) & 0x1fffU) == 32U)
        {
            if (candidate != 0U)
            {
                return 0U;
            }
            candidate = order;
        }
    }
    return candidate;
}
