/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_HYBRID_STATS_H
#define SIMPLECLOCK_HYBRID_STATS_H

#include <stdbool.h>
#include <stdint.h>

#define SC_HYBRID_STATS_CAPACITY 121U

/* Anchored sums for drift, bias and bias-minus-integrated-drift.
 * A second accumulator rebuilds a fresh 121-point block one point per push;
 * replacing the rolling moments each block bounds accumulated roundoff without
 * a periodic long foreground scan. Storage and all phase arithmetic are double. */
typedef struct
{
    double base[3], sum[3], square[3], weighted[3];
} sc_hybrid_moments_t;

/* Fixed-size trailing clock statistics for contiguous, nominal 1 Hz samples.
 * Temperature is not a clock input. Drift keeps compact precision; phase
 * quantities remain double so a large offset does not erase small changes.
 * Initialize with reset (or zero-initialize); all operations are foreground-only
 * and copy values. No timestamps are stored: the caller must reset on a gap or
 * coordinate change rather than fitting noncontiguous samples as 1 Hz data. */
typedef struct
{
    float drift_ppb[SC_HYBRID_STATS_CAPACITY];
    double bias_offset_ns[SC_HYBRID_STATS_CAPACITY];
    double residual_ns[SC_HYBRID_STATS_CAPACITY];
    sc_hybrid_moments_t window, rebuilding;
    uint32_t accepted, rejected;
    uint16_t count, next, rebuilding_count;
} sc_hybrid_stats_t;

_Static_assert(sizeof(sc_hybrid_stats_t) <= 2816U, "hybrid statistics exceed 2.75 KiB");

/* Outputs are double for stable host tests and phase arithmetic. Slopes use
 * seconds as the independent variable, so ns/s is numerically ppb. */
typedef struct
{
    double drift_mean_ppb;
    double drift_sd_ppb;
    double bias_slope_ppb;
    double residual_slope_ppb;
    double residual_detrended_rms_ns;
} sc_hybrid_metrics_t;

void sc_hybrid_stats_reset(sc_hybrid_stats_t *state);

/* Store one finite sample. The ring retains the latest 121 accepted points;
 * rejected non-finite/out-of-range inputs do not alter the stored window. */
bool sc_hybrid_stats_push(sc_hybrid_stats_t *state, double drift_ppb, double bias_offset_ns,
                          double residual_ns);

/* Compute metrics for the latest points. At least two contiguous points are
 * required because the result includes sample SDs and fitted slopes. Phase
 * offsets are differenced from a nearby sample before fitting. The normal
 * 121-point window uses bounded rolling moments; shorter windows use a scan.
 * All getters return false for unavailable points/NULL arguments and leave
 * their output untouched. A drift mean alone permits a single point. */
bool sc_hybrid_stats_get(const sc_hybrid_stats_t *state, unsigned points,
                         sc_hybrid_metrics_t *metrics);
/* Lightweight bias-only fit permits the 100 s phase window alongside the
 * 121-point tracking window without recomputing every metric twice. */
bool sc_hybrid_stats_bias_slope(const sc_hybrid_stats_t *state, unsigned points, double *slope_ppb);
/* Lightweight mean over the latest contiguous stored drift samples.  The
 * double accumulator matches drift_mean_ppb from sc_hybrid_stats_get(). */
bool sc_hybrid_stats_drift_mean(const sc_hybrid_stats_t *state, unsigned points, double *mean_ppb);

#endif
