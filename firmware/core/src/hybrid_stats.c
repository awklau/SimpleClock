/* SPDX-License-Identifier: MIT */
#include "simpleclock/hybrid_stats.h"
#include <float.h>
#include <math.h>
#include <string.h>

static bool finite_float_input(double value)
{
    return isfinite(value) && value <= (double)FLT_MAX && value >= -(double)FLT_MAX;
}

static unsigned oldest_index(const sc_hybrid_stats_t *state)
{
    return state->count == SC_HYBRID_STATS_CAPACITY ? state->next : 0U;
}

enum
{
    DRIFT,
    BIAS,
    RESIDUAL,
    CHANNELS
};

static void add_to_block(sc_hybrid_moments_t *m, unsigned count, const double values[CHANNELS])
{
    if (count == 0U)
    {
        memset(m, 0, sizeof(*m));
        memcpy(m->base, values, sizeof(m->base));
    }
    for (unsigned i = 0; i < CHANNELS; i++)
    {
        const double delta = values[i] - m->base[i];
        m->sum[i] += delta;
        if (i != BIAS)
        {
            m->square[i] += delta * delta;
        }
        if (i != DRIFT)
        {
            m->weighted[i] += (double)count * delta;
        }
    }
}

static void slide_window(sc_hybrid_moments_t *m, const double old[CHANNELS],
                         const double values[CHANNELS])
{
    for (unsigned i = 0; i < CHANNELS; i++)
    {
        const double leaving = old[i] - m->base[i];
        const double entering = values[i] - m->base[i];
        /* Old x indices 1..120 shift to 0..119; the entering point has x=120.
         * Use the OLD sum before updating it. All values share this block's
         * nearby anchor, so a large absolute receiver bias is cancelled first. */
        if (i != DRIFT)
        {
            m->weighted[i] +=
                (double)(SC_HYBRID_STATS_CAPACITY - 1U) * entering - (m->sum[i] - leaving);
        }
        m->sum[i] += entering - leaving;
        if (i != BIAS)
        {
            m->square[i] += entering * entering - leaving * leaving;
        }
    }
}

static double nonnegative(double value)
{
    return value < 0.0 ? 0.0 : value;
}

static void full_window(const sc_hybrid_stats_t *s, sc_hybrid_metrics_t *out)
{
    const sc_hybrid_moments_t *m = &s->window;
    const double n = (double)SC_HYBRID_STATS_CAPACITY;
    const double center = (n - 1.0) * 0.5;
    /* Sum of squared centered indices for n contiguous 1 Hz samples:
     * sum((i-(n-1)/2)^2) = n*(n^2-1)/12, in seconds squared. */
    const double denominator = n * (n * n - 1.0) / 12.0;
    const double bias_cross = m->weighted[BIAS] - center * m->sum[BIAS];
    const double residual_cross = m->weighted[RESIDUAL] - center * m->sum[RESIDUAL];
    const double drift_ss = m->square[DRIFT] - m->sum[DRIFT] * (m->sum[DRIFT] / n);
    const double residual_ss = m->square[RESIDUAL] - m->sum[RESIDUAL] * (m->sum[RESIDUAL] / n) -
                               residual_cross * (residual_cross / denominator);
    *out = (sc_hybrid_metrics_t){
        .drift_mean_ppb = m->base[DRIFT] + m->sum[DRIFT] / n,
        .drift_sd_ppb = sqrt(nonnegative(drift_ss) / (n - 1.0)),
        .bias_slope_ppb = bias_cross / denominator,
        .residual_slope_ppb = residual_cross / denominator,
        .residual_detrended_rms_ns = sqrt(nonnegative(residual_ss) / n),
    };
}

void sc_hybrid_stats_reset(sc_hybrid_stats_t *state)
{
    if (state != NULL)
    {
        memset(state, 0, sizeof(*state));
    }
}

bool sc_hybrid_stats_push(sc_hybrid_stats_t *state, double drift_ppb, double bias_offset_ns,
                          double residual_ns)
{
    if (state == NULL || !finite_float_input(drift_ppb) || !isfinite(bias_offset_ns) ||
        !isfinite(residual_ns))
    {
        if (state != NULL)
        {
            state->rejected++;
        }
        return false;
    }

    const unsigned index = state->next;
    /* Cache the same rounded float drift value as the ring, not the
     * higher-precision argument; both public paths must describe the same
     * samples. A rejected sample above changes neither cache nor history. */
    const double values[CHANNELS] = {(double)(float)drift_ppb, bias_offset_ns, residual_ns};
    if (state->count == SC_HYBRID_STATS_CAPACITY)
    {
        const double old[CHANNELS] = {(double)state->drift_ppb[index], state->bias_offset_ns[index],
                                      state->residual_ns[index]};
        slide_window(&state->window, old, values);
    }
    add_to_block(&state->rebuilding, state->rebuilding_count, values);
    state->rebuilding_count++;
    if (state->rebuilding_count == SC_HYBRID_STATS_CAPACITY)
    {
        /* This just-completed block is exactly the current trailing window.
         * Rebase every 121 pushes; do not accumulate numerical drift forever. */
        state->window = state->rebuilding;
        state->rebuilding_count = 0U;
    }
    state->drift_ppb[index] = (float)drift_ppb;
    state->bias_offset_ns[index] = bias_offset_ns;
    state->residual_ns[index] = residual_ns;
    const unsigned next = index + 1U;
    state->next = (uint16_t)(next == SC_HYBRID_STATS_CAPACITY ? 0U : next);
    if (state->count < SC_HYBRID_STATS_CAPACITY)
    {
        state->count++;
    }
    state->accepted++;
    return true;
}

bool sc_hybrid_stats_get(const sc_hybrid_stats_t *state, unsigned points,
                         sc_hybrid_metrics_t *metrics)
{
    if (state == NULL || metrics == NULL || points < 2U || points > SC_HYBRID_STATS_CAPACITY ||
        points > state->count)
    {
        return false;
    }
    if (points == SC_HYBRID_STATS_CAPACITY)
    {
        full_window(state, metrics);
        return true;
    }

    const unsigned first = (oldest_index(state) + state->count - points) % SC_HYBRID_STATS_CAPACITY;
    const double center = ((double)points - 1.0) * 0.5;
    const double denominator = (double)points * ((double)points * (double)points - 1.0) / 12.0;
    /* Offset each channel before accumulating moments. This is algebraically
     * identical to raw sums, but avoids subtracting two large, nearly equal
     * values when drift barely moves. */
    const double drift_base = (double)state->drift_ppb[first];
    const double bias_base = state->bias_offset_ns[first];
    const double residual_base = state->residual_ns[first];

    double drift_delta_sum = 0.0;
    double drift_delta_square_sum = 0.0;
    double bias_cross = 0.0;
    double residual_cross = 0.0;
    double residual_sum = 0.0;
    double residual_square_sum = 0.0;
    unsigned index = first;
    double x = -center;
    for (unsigned i = 0U; i < points; i++)
    {
        const double drift_delta = (double)state->drift_ppb[index] - drift_base;
        const double bias = state->bias_offset_ns[index] - bias_base;
        const double residual = state->residual_ns[index] - residual_base;
        drift_delta_sum += drift_delta;
        drift_delta_square_sum += drift_delta * drift_delta;
        bias_cross += x * bias;
        residual_cross += x * residual;
        residual_sum += residual;
        residual_square_sum += residual * residual;
        index++;
        if (index == SC_HYBRID_STATS_CAPACITY)
        {
            index = 0U;
        }
        x += 1.0;
    }
    const double drift_mean_delta = drift_delta_sum / (double)points;
    const double drift_mean = drift_base + drift_mean_delta;
    double drift_ss = drift_delta_square_sum - drift_delta_sum * drift_mean_delta;
    double residual_ss = residual_square_sum - residual_sum * (residual_sum / (double)points);
    const double bias_slope = bias_cross / denominator;
    const double residual_slope = residual_cross / denominator;
    residual_ss -= residual_cross * residual_slope;
    /* Exact variances and residual sums of squares are nonnegative. Roundoff
     * in the one-pass centered-moment identities can produce a tiny negative. */
    if (drift_ss < 0.0)
    {
        drift_ss = 0.0;
    }
    if (residual_ss < 0.0)
    {
        residual_ss = 0.0;
    }

    *metrics = (sc_hybrid_metrics_t){
        .drift_mean_ppb = drift_mean,
        .drift_sd_ppb = sqrt(drift_ss / (double)(points - 1U)),
        .bias_slope_ppb = bias_slope,
        .residual_slope_ppb = residual_slope,
        .residual_detrended_rms_ns = sqrt(residual_ss / (double)points),
    };
    return true;
}

bool sc_hybrid_stats_bias_slope(const sc_hybrid_stats_t *state, unsigned points, double *slope_ppb)
{
    if (!state || !slope_ppb || points < 2U || points > SC_HYBRID_STATS_CAPACITY ||
        points > state->count)
    {
        return false;
    }
    const unsigned first = (oldest_index(state) + state->count - points) % SC_HYBRID_STATS_CAPACITY;
    const double center = ((double)points - 1.0) * 0.5;
    const double base = state->bias_offset_ns[first];
    double cross = 0.0;
    unsigned index = first;
    double x = -center;
    for (unsigned i = 0; i < points; i++)
    {
        cross += x * (state->bias_offset_ns[index] - base);
        index++;
        if (index == SC_HYBRID_STATS_CAPACITY)
        {
            index = 0U;
        }
        x += 1.0;
    }
    *slope_ppb = cross / ((double)points * ((double)points * (double)points - 1.0) / 12.0);
    return true;
}

bool sc_hybrid_stats_drift_mean(const sc_hybrid_stats_t *state, unsigned points, double *mean_ppb)
{
    if (!state || !mean_ppb || points == 0U || points > SC_HYBRID_STATS_CAPACITY ||
        points > state->count)
    {
        return false;
    }
    const unsigned first = (oldest_index(state) + state->count - points) % SC_HYBRID_STATS_CAPACITY;
    double sum = 0.0;
    unsigned index = first;
    for (unsigned i = 0U; i < points; i++)
    {
        sum += (double)state->drift_ppb[index];
        index++;
        if (index == SC_HYBRID_STATS_CAPACITY)
        {
            index = 0U;
        }
    }
    *mean_ppb = sum / (double)points;
    return true;
}
