/* SPDX-License-Identifier: MIT */
#include "simpleclock/preheat.h"
#include <string.h>

#define PREHEAT_SENSOR_STALE_MS 1500U  /* Maximum gap between valid power samples. */
#define PREHEAT_SAMPLE_GAP_MS 950U     /* Reject calls that cannot be one-second evidence. */
#define PREHEAT_WINDOW_SPAN_MS 119000U /* Minimum span of the 120-sample retained window. */

void sc_preheat_init(sc_preheat_t *state, uint32_t now_ms)
{
    memset(state, 0, sizeof(*state));
    state->start_ms = now_ms;
}

void sc_preheat_expire(sc_preheat_t *state, uint32_t now_ms, bool sensor_fresh)
{
    if (!sensor_fresh ||
        (state->observed && (uint32_t)(now_ms - state->last_ms) > PREHEAT_SENSOR_STALE_MS))
    {
        state->complete_inferred = false;
        state->plateau = false;
        state->count = state->next = 0;
    }
    if (!state->complete_inferred && (uint32_t)(now_ms - state->start_ms) >= SC_PREHEAT_TIMEOUT_MS)
    {
        state->unconfirmed = true;
    }
}

void sc_preheat_push(sc_preheat_t *state, uint32_t now_ms, uint32_t power_code, bool device_valid,
                     bool sensor_fresh, bool evidence_verified)
{
    sc_preheat_expire(state, now_ms, sensor_fresh);
    if (!evidence_verified)
    {
        state->complete_inferred = false;
    }
    if (!device_valid || !sensor_fresh || !power_code || power_code >= 262143U)
    {
        state->complete_inferred = false;
        state->plateau = false;
        state->count = state->next = 0;
        return;
    }
    /* Refuse burst calls as 120 seconds of evidence. Small acquisition jitter is
     * allowed, but the full window must span at least 119 nominal seconds. */
    if (state->observed && (uint32_t)(now_ms - state->last_ms) < PREHEAT_SAMPLE_GAP_MS)
    {
        return;
    }
    state->observed = true;
    state->last_ms = now_ms;
    if (!state->count)
    {
        state->first_ms = now_ms;
    }
    state->power[state->next] = power_code;
    state->sample_ms[state->next] = now_ms;
    state->next = (state->next + 1U) % SC_PREHEAT_WINDOW;
    if (state->count < SC_PREHEAT_WINDOW)
    {
        state->count++;
    }
    if (state->count < SC_PREHEAT_WINDOW ||
        (uint32_t)(now_ms - state->sample_ms[state->next]) < PREHEAT_WINDOW_SPAN_MS)
    {
        return;
    }
    uint32_t sorted[SC_PREHEAT_WINDOW];
    memcpy(sorted, state->power, sizeof(sorted));
    for (unsigned i = 1; i < SC_PREHEAT_WINDOW; i++)
    {
        const uint32_t value = sorted[i];
        unsigned j = i;
        while (j && sorted[j - 1U] > value)
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }
    /* Nearest-rank percentiles: ceil(p*N)-1. Median uses the two middle values.
     * Codes are 18 bit, so both sum and 100*spread fit in uint32_t. */
    state->p05 = sorted[5];
    state->p95 = sorted[113];
    state->median = (sorted[59] + sorted[60]) / 2U;
    state->plateau = (state->p95 - state->p05) * 100U <= state->median * 2U;
    if (!evidence_verified)
    {
        state->complete_inferred = false;
    }
    else if (!state->unconfirmed && (uint32_t)(now_ms - state->start_ms) >= SC_PREHEAT_MIN_MS &&
             state->plateau)
    {
        state->complete_inferred = true;
    }
    /* After qualification the native power response to external heat must not
     * revoke readiness merely because its plateau moves (plan section 6). */
}
