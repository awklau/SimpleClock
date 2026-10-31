/* SPDX-License-Identifier: MIT */
#include "simpleclock/thermal_boot.h"
#include "simpleclock/hybrid_defaults.h"
#include "simpleclock/thermal_control.h"
#include <limits.h>
#include <string.h>

#define BOOT(name) SC_HYBRID_THERMAL_BOOT_##name
#define MIN_TARGET ((int32_t)(SC_HYBRID_THERMAL_OUTER_SETPOINT_MIN_C * SC_THERMAL_Q16))
#define MAX_TARGET ((int32_t)(SC_HYBRID_THERMAL_OUTER_SETPOINT_MAX_C * SC_THERMAL_Q16))

_Static_assert(SC_THERMAL_BOOT_POINTS >= 3U && SC_THERMAL_BOOT_POINTS <= 31U,
               "Recheck index and int64 slope-product bounds if fit capacity changes");
_Static_assert(SC_THERMAL_BOOT_CAPTURE_SLOPE_Q16_PER_S > 0,
               "The fixed startup slope threshold must be positive");

void sc_thermal_boot_start(sc_thermal_boot_t *s, uint32_t first_power_ms)
{
    memset(s, 0, sizeof(*s));
    s->start_ms = first_power_ms;
}

static bool capture(sc_thermal_boot_t *s, int32_t raw_q16, sc_thermal_boot_phase_t phase)
{
    s->target_q16 =
        raw_q16 < MIN_TARGET ? MIN_TARGET : (raw_q16 > MAX_TARGET ? MAX_TARGET : raw_q16);
    s->phase = phase;
    return true;
}

static int32_t slope(const sc_thermal_boot_t *s)
{
    /* Anchor both axes at the oldest point. Integer least squares uses actual
     * sample milliseconds, handles tick wrap, and preserves TMP117 resolution.
     * At <2.5 s between <=31 points and -55..150 C, all products fit int64. */
    const unsigned first = s->next;
    const uint32_t origin_ms = s->time_ms[first];
    const int32_t origin_q16 = s->temperature_q16[first];
    int64_t st = 0, sy = 0, stt = 0, sty = 0;
    for (unsigned i = 0; i < SC_THERMAL_BOOT_POINTS; i++)
    {
        const int64_t t = (uint32_t)(s->time_ms[i] - origin_ms);
        const int64_t y = (int64_t)s->temperature_q16[i] - origin_q16;
        st += t;
        sy += y;
        stt += t * t;
        sty += t * y;
    }
    const int64_t n = SC_THERMAL_BOOT_POINTS;
    const int64_t denominator = n * stt - st * st;
    const int64_t result = (n * sty - st * sy) * 1000 / denominator;
    return result < -INT32_MAX ? -INT32_MAX : (result > INT32_MAX ? INT32_MAX : (int32_t)result);
}

bool sc_thermal_boot_sample(sc_thermal_boot_t *s, uint32_t sample_ms, int32_t raw_q16)
{
    if (s->phase >= SC_THERMAL_BOOT_CAPTURED)
    {
        return s->phase != SC_THERMAL_BOOT_BAD_DATA;
    }
    if (raw_q16 < -55L * SC_THERMAL_Q16 || raw_q16 > 150L * SC_THERMAL_Q16)
    {
        s->phase = SC_THERMAL_BOOT_BAD_DATA;
        return false;
    }
    if (raw_q16 >= MAX_TARGET)
    {
        return capture(s, raw_q16, SC_THERMAL_BOOT_CEILING);
    }
    const uint32_t age_ms = sample_ms - s->start_ms;
    if (age_ms >= UINT32_C(0x80000000))
    {
        return false; /* Conversion began before the first powered frame. */
    }
    if (s->have_sample)
    {
        const uint32_t elapsed = sample_ms - s->last_ms;
        if (elapsed == 0U)
        {
            return false;
        }
        if (elapsed >= SC_TEMPERATURE_STALE_MS)
        {
            s->phase = SC_THERMAL_BOOT_BAD_DATA;
            return false;
        }
        if (elapsed < 900U)
        {
            return false; /* Cached/oversampled values cannot manufacture a slope. */
        }
    }
    s->last_ms = sample_ms;
    s->have_sample = true;
    if (age_ms < (uint32_t)(BOOT(REFERENCE_DELAY_S) * 1000.0))
    {
        return false;
    }
    s->phase = s->reference_valid ? SC_THERMAL_BOOT_DECAY : SC_THERMAL_BOOT_REFERENCE;
    s->time_ms[s->next] = sample_ms;
    s->temperature_q16[s->next] = raw_q16;
    s->next = (uint8_t)((s->next + 1U) % SC_THERMAL_BOOT_POINTS);
    if (s->count < SC_THERMAL_BOOT_POINTS)
    {
        s->count++;
    }
    if (s->count < SC_THERMAL_BOOT_POINTS)
    {
        return false;
    }
    /* Do not silently turn the agreed reference into a longer
     * experiment if conversions were missed. Normal 1 Hz scheduling jitter
     * is allowed; actual sample times still determine the fitted slope. */
    if (!s->reference_valid)
    {
        const uint32_t span_ms = sample_ms - s->time_ms[s->next];
        const uint32_t window_ms = BOOT(REFERENCE_WINDOW_S) * 1000U;
        if (span_ms < window_ms - 500U || span_ms > window_ms + 500U)
        {
            s->phase = SC_THERMAL_BOOT_BAD_DATA;
            return false;
        }
    }
    s->slope_q16_per_s = slope(s);
    if (!s->reference_valid)
    {
        s->reference_q16_per_s = s->slope_q16_per_s;
        /* Two raw TMP117 counts over the reference form a resolution floor, not
         * a claimed measured thermal gain. Do not divide by zero, or boost
         * indefinitely on a flat/cooling warm start. Enter PID at current T. */
        const int32_t minimum =
            (int32_t)(BOOT(MIN_REFERENCE_RISE_RAW_COUNTS) * 512L / (SC_THERMAL_BOOT_POINTS - 1U));
        if (s->reference_q16_per_s < minimum)
        {
            return capture(s, raw_q16, SC_THERMAL_BOOT_NO_RISE);
        }
        s->reference_valid = true;
        s->phase = SC_THERMAL_BOOT_DECAY;
        /* A warm start may already be below the fixed threshold. The first
         * full valid fit can start confirmation; no reference ratio is used. */
    }
    const bool below = s->slope_q16_per_s <= SC_THERMAL_BOOT_CAPTURE_SLOPE_Q16_PER_S;
    if (!below)
    {
        s->below_count = 0U;
        s->below_timing = false;
        s->below_elapsed_ms = 0U;
        return false;
    }
    if (!s->below_timing)
    {
        s->below_timing = true;
        s->below_since_ms = sample_ms;
    }
    if (s->below_count < UINT8_MAX)
    {
        s->below_count++;
    }
    s->below_elapsed_ms = sample_ms - s->below_since_ms;
    /* Overlapping fits are not independent confirmations. Require elapsed
     * measurement time below the threshold, resetting on any above-threshold
     * fit. Cached samples cannot advance this timer; missing data faults above. */
    if (s->below_elapsed_ms >= BOOT(CAPTURE_CONFIRM_S) * 1000U)
    {
        return capture(s, raw_q16, SC_THERMAL_BOOT_CAPTURED);
    }
    return false;
}
