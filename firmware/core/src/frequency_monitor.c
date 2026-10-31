/* SPDX-License-Identifier: MIT */
#include "simpleclock/frequency_monitor.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define SETPOINT_Q18_SCALE 262144.0 /* 2^18 counts per ppb. */
#define SETPOINT_ROUNDING_PPB (0.5 / SETPOINT_Q18_SCALE)
#define RECEIVER_TIMEOUT_MS ((uint32_t)(SC_HYBRID_QUALIFICATION_RECEIVER_UPDATE_TIMEOUT_S * 1000.0))
#define SAMPLE_INTERVAL_MS 1000U

void sc_frequency_monitor_service(sc_frequency_monitor_t *s, uint32_t now_ms)
{
    if (s->count && (uint32_t)(now_ms - s->last_ms) >= SC_FREQUENCY_HOLDOVER_MS)
    {
        memset(s, 0, sizeof(*s));
    }
}

void sc_frequency_monitor_sample(sc_frequency_monitor_t *s, uint64_t epoch_ms, uint32_t received_ms,
                                 uint32_t now_ms, double drift_ppb, double setpoint_ppb)
{
    sc_frequency_monitor_service(s, now_ms);
    if (!isfinite(drift_ppb) || fabs(drift_ppb) > (double)FLT_MAX || !isfinite(setpoint_ppb) ||
        fabs(setpoint_ppb) > SC_HYBRID_CLOCK_TRACKING_REPORTED_DRIFT_SETPOINT_LIMIT_PPB ||
        fabs(setpoint_ppb * SETPOINT_Q18_SCALE) > INT16_MAX ||
        (uint32_t)(now_ms - received_ms) > RECEIVER_TIMEOUT_MS)
    {
        return;
    }
    if (s->count)
    {
        if (epoch_ms == s->last_epoch_ms)
        {
            return; /* Repeated/cached epochs cannot refresh the indication. */
        }
        /* Backward receiver time wraps this unsigned delta to a large value,
         * taking the same reset path as a gap beyond the holdover allowance. */
        const uint64_t epoch_delta = epoch_ms - s->last_epoch_ms;
        const uint32_t arrival_delta_ms = received_ms - s->last_ms;
        if (epoch_delta < SAMPLE_INTERVAL_MS || epoch_delta >= SC_FREQUENCY_HOLDOVER_MS ||
            arrival_delta_ms >= SC_FREQUENCY_HOLDOVER_MS)
        {
            memset(s, 0, sizeof(*s));
        }
        else if (epoch_delta != SAMPLE_INTERVAL_MS || arrival_delta_ms > RECEIVER_TIMEOUT_MS)
        {
            const uint32_t gap =
                (uint32_t)epoch_delta > arrival_delta_ms ? (uint32_t)epoch_delta : arrival_delta_ms;
            s->gaps_ms += gap - SAMPLE_INTERVAL_MS;
            if (!s->settled)
            {
                s->timing = false; /* A gap cannot manufacture the initial dwell. */
            }
            if (s->gaps_ms + s->previous_gaps_ms > SC_FREQUENCY_HOLDOVER_MS)
            {
                memset(s, 0, sizeof(*s));
            }
        }
    }
    if (!s->count)
    {
        s->first_ms = received_ms;
        s->first_epoch_ms = epoch_ms;
    }
    s->last_epoch_ms = epoch_ms;
    s->last_ms = received_ms;
    if (++s->block_count == SC_FREQUENCY_POINTS)
    {
        /* A trailing window crosses at most two 121-sample blocks. Bounding
         * their combined gaps bounds its age without a timestamp per point.
         * Periodic short dropouts must not accumulate a lifetime gap debt. */
        s->block_count = 0U;
        s->previous_gaps_ms = s->gaps_ms;
        s->gaps_ms = 0U;
    }
    s->drift[s->next] = (float)drift_ppb;
    s->next = (uint16_t)((s->next + 1U) % SC_FREQUENCY_POINTS);
    const double scaled = setpoint_ppb * SETPOINT_Q18_SCALE;
    s->setpoint_q18[s->setpoint_next] = (int16_t)(scaled + (scaled < 0.0 ? -0.5 : 0.5));
    s->setpoint_next = (uint16_t)((s->setpoint_next + 1U) % SC_FREQUENCY_MEAN_POINTS);
    if (s->count < SC_FREQUENCY_POINTS)
    {
        s->count++;
    }
    const unsigned means =
        s->count < SC_FREQUENCY_MEAN_POINTS ? s->count : SC_FREQUENCY_MEAN_POINTS;
    double drift_sum = 0.0, requested_sum = 0.0;
    /* The rings have different lengths but contain the same trailing 'means'
     * observations. Sum order need not match: mean(drift-request) equals the
     * difference of their means, using each sample's contemporaneous request. */
    for (unsigned i = 0U; i < means; i++)
    {
        drift_sum +=
            (double)s->drift[(s->next + SC_FREQUENCY_POINTS - 1U - i) % SC_FREQUENCY_POINTS];
        requested_sum += (double)s->setpoint_q18[i] / SETPOINT_Q18_SCALE;
    }
    s->mean_drift_ppb = drift_sum / means;
    s->mean_error_ppb = (drift_sum - requested_sum) / means;
    const uint32_t span_ms = (SC_FREQUENCY_POINTS - 1U) * SAMPLE_INTERVAL_MS;
    s->window_valid = s->window_valid ||
                      (s->count == SC_FREQUENCY_POINTS && epoch_ms - s->first_epoch_ms >= span_ms &&
                       (uint32_t)(received_ms - s->first_ms) >= span_ms);
    if (!s->window_valid)
    {
        return;
    }
    /* Anchored moments avoid subtracting large nearly equal raw moments. */
    const double base = (double)s->drift[0];
    double sum = 0.0, square = 0.0;
    for (unsigned i = 0U; i < SC_FREQUENCY_POINTS; i++)
    {
        const double delta = (double)s->drift[i] - base;
        sum += delta;
        square += delta * delta;
    }
    s->drift_sd_ppb =
        sqrt(fmax(0.0, square - sum * (sum / SC_FREQUENCY_POINTS)) / (SC_FREQUENCY_POINTS - 1U));
    /* Keep both mean gates: tracking a nonzero request does not by itself
     * establish near-zero drift. The Q18 half-LSB tightens only tracking error. */
    const bool good =
        fabs(s->mean_drift_ppb) <= SC_HYBRID_CLOCK_FREQUENCY_SETTLE_MEAN_DRIFT_LIMIT_PPB &&
        fabs(s->mean_error_ppb) + SETPOINT_ROUNDING_PPB <=
            SC_HYBRID_CLOCK_FREQUENCY_SETTLE_MEAN_ERROR_LIMIT_PPB &&
        s->drift_sd_ppb <= SC_HYBRID_CLOCK_FREQUENCY_SETTLE_DRIFT_STDDEV_LIMIT_PPB;
    if (!good)
    {
        s->timing = s->settled = false;
    }
    else if (!s->timing)
    {
        s->timing = true;
        s->since_ms = now_ms;
        s->since_epoch_ms = epoch_ms;
    }
    else if (!s->settled)
    {
        const uint32_t dwell_ms =
            (uint32_t)(SC_HYBRID_CLOCK_FREQUENCY_SETTLE_GOOD_DWELL_S * 1000.0);
        s->settled = epoch_ms - s->since_epoch_ms >= dwell_ms &&
                     (uint32_t)(now_ms - s->since_ms) >= dwell_ms;
    }
}
