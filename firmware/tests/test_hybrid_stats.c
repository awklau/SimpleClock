/* SPDX-License-Identifier: MIT */
#include "simpleclock/hybrid_stats.h"
#include <math.h>
#include <stdio.h>

#define CHECK(e)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(e))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #e);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static int close_to(double actual, double expected, double tolerance)
{
    return fabs(actual - expected) <= tolerance;
}

typedef struct
{
    float drift;
    double bias;
    double residual;
} reference_sample_t;

typedef struct
{
    reference_sample_t samples[SC_HYBRID_STATS_CAPACITY];
    unsigned count;
    unsigned next;
} reference_ring_t;

static void reference_reset(reference_ring_t *reference)
{
    *reference = (reference_ring_t){0};
}

static void reference_push(reference_ring_t *reference, double drift, double bias, double residual)
{
    reference->samples[reference->next] = (reference_sample_t){(float)drift, bias, residual};
    reference->next++;
    if (reference->next == SC_HYBRID_STATS_CAPACITY)
    {
        reference->next = 0U;
    }
    if (reference->count < SC_HYBRID_STATS_CAPACITY)
    {
        reference->count++;
    }
}

/* Independent two-pass regression over the test's own retained inputs. The
 * first included value anchors each channel before calculating means, which
 * preserves small changes on the 1e12 ns phase-offset fixtures. */
static int reference_get(const reference_ring_t *reference, unsigned points,
                         sc_hybrid_metrics_t *metrics)
{
    if (reference == NULL || metrics == NULL || points < 2U || points > reference->count)
    {
        return 0;
    }
    reference_sample_t ordered[SC_HYBRID_STATS_CAPACITY];
    const unsigned first =
        (reference->next + SC_HYBRID_STATS_CAPACITY - points) % SC_HYBRID_STATS_CAPACITY;
    unsigned index = first;
    for (unsigned i = 0U; i < points; i++)
    {
        ordered[i] = reference->samples[index];
        index++;
        if (index == SC_HYBRID_STATS_CAPACITY)
        {
            index = 0U;
        }
    }

    const double center = ((double)points - 1.0) * 0.5;
    const double denominator = (double)points * ((double)points * (double)points - 1.0) / 12.0;
    const double drift_base = (double)ordered[0].drift;
    const double bias_base = ordered[0].bias;
    const double residual_base = ordered[0].residual;
    double drift_sum = 0.0;
    double bias_sum = 0.0;
    double residual_sum = 0.0;
    for (unsigned i = 0U; i < points; i++)
    {
        drift_sum += (double)ordered[i].drift - drift_base;
        bias_sum += ordered[i].bias - bias_base;
        residual_sum += ordered[i].residual - residual_base;
    }
    const double drift_mean_delta = drift_sum / (double)points;
    const double bias_mean = bias_sum / (double)points;
    const double residual_mean = residual_sum / (double)points;
    double drift_ss = 0.0;
    double bias_cross = 0.0;
    double residual_cross = 0.0;
    for (unsigned i = 0U; i < points; i++)
    {
        const double x = (double)i - center;
        const double drift_delta = (double)ordered[i].drift - drift_base - drift_mean_delta;
        const double bias_delta = ordered[i].bias - bias_base - bias_mean;
        const double residual_delta = ordered[i].residual - residual_base - residual_mean;
        drift_ss += drift_delta * drift_delta;
        bias_cross += x * bias_delta;
        residual_cross += x * residual_delta;
    }
    const double residual_slope = residual_cross / denominator;
    double residual_ss = 0.0;
    for (unsigned i = 0U; i < points; i++)
    {
        const double x = (double)i - center;
        const double residual_delta =
            ordered[i].residual - residual_base - residual_mean - x * residual_slope;
        residual_ss += residual_delta * residual_delta;
    }
    *metrics = (sc_hybrid_metrics_t){
        .drift_mean_ppb = drift_base + drift_mean_delta,
        .drift_sd_ppb = sqrt(drift_ss / (double)(points - 1U)),
        .bias_slope_ppb = bias_cross / denominator,
        .residual_slope_ppb = residual_slope,
        .residual_detrended_rms_ns = sqrt(residual_ss / (double)points),
    };
    return 1;
}

static int compare_reference(const sc_hybrid_metrics_t *actual, const sc_hybrid_metrics_t *expected,
                             unsigned run, unsigned sample, unsigned points)
{
    /* Limits stay well below the adjacent qualification boundaries: 0.03 ppb
     * bias slope and 5 ns detrended RMS. Phase values are quantized at about
     * 1.22e-4 ns around 1e12 ns, so RMS tolerance allows several input ULPs. */
    const double drift_mean_tol = 1e-8;
    const double drift_sd_tol = 1e-8;
    const double phase_slope_tol = 1e-6;
    const double residual_rms_tol = 0.001;
#define COMPARE_METRIC(field, tolerance)                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!close_to(actual->field, expected->field, tolerance))                                  \
        {                                                                                          \
            fprintf(stderr,                                                                        \
                    "run %u sample %u window %u %s mismatch: actual %.17g expected %.17g tol "     \
                    "%.3g\n",                                                                      \
                    run, sample, points, #field, actual->field, expected->field, tolerance);       \
            return 0;                                                                              \
        }                                                                                          \
    } while (0)
    COMPARE_METRIC(drift_mean_ppb, drift_mean_tol);
    COMPARE_METRIC(drift_sd_ppb, drift_sd_tol);
    COMPARE_METRIC(bias_slope_ppb, phase_slope_tol);
    COMPARE_METRIC(residual_slope_ppb, phase_slope_tol);
    COMPARE_METRIC(residual_detrended_rms_ns, residual_rms_tol);
#undef COMPARE_METRIC
    return 1;
}

static void generated_sample(unsigned run, unsigned sample_index, double *drift, double *bias,
                             double *residual)
{
    static const int noise[] = {-6, -2, 3, 7, -4, 1, 5, -5, 2, -1, 0};
    const unsigned epoch = sample_index / 500U;
    const unsigned local = sample_index % 500U;
    const int direction = epoch % 2U == 0U ? 1 : -1;
    const double n = (double)noise[sample_index % 11U];
    const double bias_rate = epoch % 2U == 0U ? 0.0298 : -0.0302;
    const double residual_rate = epoch % 2U == 0U ? 0.0297 : -0.0303;
    *drift = 10.0 + (double)run * 0.5 + (double)epoch * 0.1 +
             (double)direction * 0.0003 * (double)local + n * 0.00002;
    *bias = 1.0e12 + (double)run * 16.0 + (double)epoch * 5.0 + bias_rate * (double)local + 0.5 * n;
    *residual =
        1.0e12 + (double)run * 32.0 + (double)epoch * 9.0 + residual_rate * (double)local + 1.2 * n;
}

static int test_analytic(void)
{
    sc_hybrid_stats_t state;
    sc_hybrid_metrics_t metrics;
    sc_hybrid_stats_reset(&state);
    for (unsigned i = 0U; i < 121U; i++)
    {
        const double x = (double)i;
        CHECK(sc_hybrid_stats_push(&state, 1.0 + 0.01 * x, 100000.0 + 2.0 * x, 50.0 + 0.5 * x));
    }
    CHECK(sc_hybrid_stats_get(&state, 121U, &metrics));
    CHECK(close_to(metrics.drift_mean_ppb, 1.6, 1e-7));
    CHECK(close_to(metrics.drift_sd_ppb, 0.01 * sqrt(121.0 * 122.0 / 12.0), 1e-7));
    CHECK(close_to(metrics.bias_slope_ppb, 2.0, 1e-12));
    CHECK(close_to(metrics.residual_slope_ppb, 0.5, 1e-12));
    CHECK(close_to(metrics.residual_detrended_rms_ns, 0.0, 1e-10));
    CHECK(sc_hybrid_stats_get(&state, 101U, &metrics));
    CHECK(close_to(metrics.drift_mean_ppb, 1.7, 1e-7));
    CHECK(close_to(metrics.bias_slope_ppb, 2.0, 1e-12));
    CHECK(close_to(metrics.residual_slope_ppb, 0.5, 1e-12));
    return 0;
}

static int test_ring_and_rejection(void)
{
    sc_hybrid_stats_t state;
    sc_hybrid_metrics_t metrics;
    double drift_mean = -123.0;
    sc_hybrid_stats_reset(&state);
    CHECK(!sc_hybrid_stats_get(&state, 2U, &metrics));
    CHECK(!sc_hybrid_stats_drift_mean(&state, 1U, &drift_mean) && drift_mean == -123.0);
    for (unsigned i = 0U; i < 99U; i++)
    {
        CHECK(sc_hybrid_stats_push(&state, 0.0, 0.0, 0.0));
    }
    CHECK(!sc_hybrid_stats_get(&state, 100U, &metrics));
    sc_hybrid_stats_reset(&state);
    for (unsigned i = 0U; i < 121U; i++)
    {
        CHECK(sc_hybrid_stats_push(&state, (double)i, (double)i, (double)i));
    }
    CHECK(!sc_hybrid_stats_get(&state, 122U, &metrics));
    CHECK(!sc_hybrid_stats_get(&state, 1U, &metrics));
    CHECK(sc_hybrid_stats_push(&state, 121.0, 121.0, 121.0));
    CHECK(sc_hybrid_stats_get(&state, 121U, &metrics));
    CHECK(close_to(metrics.bias_slope_ppb, 1.0, 1e-12)); /* Ring now holds 1..121. */
    CHECK(sc_hybrid_stats_drift_mean(&state, 121U, &drift_mean));
    CHECK(close_to(drift_mean, 61.0, 1e-12));
    CHECK(sc_hybrid_stats_get(&state, 100U, &metrics));
    CHECK(close_to(metrics.bias_slope_ppb, 1.0, 1e-12));
    CHECK(close_to(metrics.drift_mean_ppb, 71.5, 1e-12));
    CHECK(sc_hybrid_stats_drift_mean(&state, 100U, &drift_mean));
    CHECK(close_to(drift_mean, metrics.drift_mean_ppb, 1e-12));
    CHECK(!sc_hybrid_stats_drift_mean(&state, 0U, &drift_mean));
    CHECK(!sc_hybrid_stats_drift_mean(&state, 122U, &drift_mean));
    CHECK(!sc_hybrid_stats_drift_mean(&state, 1U, NULL));
    CHECK(!sc_hybrid_stats_push(&state, NAN, 0.0, 0.0));
    CHECK(!sc_hybrid_stats_push(&state, 0.0, INFINITY, 0.0));
    CHECK(!sc_hybrid_stats_push(&state, 0.0, 0.0, NAN));
    CHECK(state.accepted == 122U && state.rejected == 3U && state.count == 121U);
    return 0;
}

static int test_residual_detrend(void)
{
    sc_hybrid_stats_t state;
    sc_hybrid_metrics_t metrics;
    sc_hybrid_stats_reset(&state);
    for (unsigned i = 0U; i < 121U; i++)
    {
        const double x = (double)i;
        const double noise = (i % 2U == 0U) ? 1.0 : -1.0;
        CHECK(sc_hybrid_stats_push(&state, 0.0, 1.0e12 + 3.0 * x, 1.0e12 + 0.25 * x + noise));
    }
    CHECK(sc_hybrid_stats_get(&state, 121U, &metrics));
    CHECK(close_to(metrics.bias_slope_ppb, 3.0, 1e-9));
    CHECK(close_to(metrics.residual_slope_ppb, 0.25, 1e-9));
    CHECK(close_to(metrics.residual_detrended_rms_ns, 1.0, 1e-4));
    double slope = 0.0;
    CHECK(sc_hybrid_stats_bias_slope(&state, 101U, &slope));
    CHECK(close_to(slope, 3.0, 1e-9));
    CHECK(!sc_hybrid_stats_bias_slope(&state, 1U, &slope));
    CHECK(!sc_hybrid_stats_bias_slope(&state, 122U, &slope));
    return 0;
}

static int test_small_residual_large_phase_offset(void)
{
    sc_hybrid_stats_t state;
    sc_hybrid_metrics_t metrics;
    sc_hybrid_stats_reset(&state);
    for (unsigned i = 0U; i < 121U; i++)
    {
        const double x = (double)i;
        const double noise = (i % 2U == 0U) ? 0.0005 : -0.0005;
        CHECK(sc_hybrid_stats_push(&state, 0.0, 1.0e12 + 0.125 * x, 1.0e12 + 0.025 * x + noise));
    }
    CHECK(sc_hybrid_stats_get(&state, 121U, &metrics));
    CHECK(close_to(metrics.bias_slope_ppb, 0.125, 2e-6));
    CHECK(close_to(metrics.residual_slope_ppb, 0.025, 2e-6));
    CHECK(metrics.residual_detrended_rms_ns > 0.00025 &&
          metrics.residual_detrended_rms_ns < 0.00075);
    return 0;
}

static int test_nearly_linear_window(void)
{
    sc_hybrid_stats_t state;
    sc_hybrid_metrics_t metrics;
    sc_hybrid_stats_reset(&state);
    for (unsigned i = 0U; i < 121U; i++)
    {
        const double x = (double)i;
        const double noise = (i % 2U == 0U) ? 0.0004 : -0.0004;
        CHECK(sc_hybrid_stats_push(&state, 1.0 - 0.003 * x, 1.0e12 + 0.002 * x,
                                   1.0e12 - 0.05 * x + noise));
    }
    CHECK(sc_hybrid_stats_get(&state, 121U, &metrics));
    CHECK(close_to(metrics.drift_mean_ppb, 0.82, 1e-7));
    CHECK(close_to(metrics.drift_sd_ppb, 0.003 * sqrt(121.0 * 122.0 / 12.0), 2e-7));
    CHECK(close_to(metrics.bias_slope_ppb, 0.002, 2e-6));
    CHECK(close_to(metrics.residual_slope_ppb, -0.05, 2e-6));
    CHECK(metrics.residual_detrended_rms_ns > 0.00015 &&
          metrics.residual_detrended_rms_ns < 0.00065);
    return 0;
}

static int test_nearly_constant_spread(void)
{
    sc_hybrid_stats_t state;
    sc_hybrid_metrics_t metrics;
    sc_hybrid_stats_reset(&state);
    const double drift_delta = 0.00001;
    const float drift_positive = (float)(10.0 + drift_delta);
    const float drift_negative = (float)(10.0 - drift_delta);
    /* The statistics ring stores drift as float, so assert against the levels
     * that are actually retained rather than the unquantized input. */
    const double two_level_sample_factor = sqrt((61.0 * 60.0) / (121.0 * 120.0));
    for (unsigned i = 0U; i < 121U; i++)
    {
        const double sign = (i % 2U == 0U) ? 1.0 : -1.0;
        CHECK(sc_hybrid_stats_push(&state, 10.0 + sign * drift_delta, 1.0e12, 1.0e12));
    }
    CHECK(sc_hybrid_stats_get(&state, 121U, &metrics));
    CHECK(close_to(metrics.drift_sd_ppb,
                   ((double)drift_positive - (double)drift_negative) * two_level_sample_factor,
                   (double)(drift_positive - drift_negative) * 1e-6));
    CHECK(metrics.residual_detrended_rms_ns == 0.0);
    return 0;
}

static int test_cached_full_window_against_reference(void)
{
    sc_hybrid_stats_t state;
    reference_ring_t reference;
    sc_hybrid_metrics_t actual, expected;
    sc_hybrid_stats_reset(&state);
    reference_reset(&reference);

    /* Two independent 2,500-sample runs exercise many ring wraps and the
     * implementation's 121-sample cache refresh. The second run starts after
     * reset with unrelated offsets to detect retained cache state. */
    for (unsigned run = 0U; run < 2U; run++)
    {
        if (run != 0U)
        {
            sc_hybrid_stats_reset(&state);
            reference_reset(&reference);
            CHECK(state.count == 0U && state.accepted == 0U && state.next == 0U);
            CHECK(!sc_hybrid_stats_get(&state, SC_HYBRID_STATS_CAPACITY, &actual));
        }
        for (unsigned sample_index = 0U; sample_index < 2500U; sample_index++)
        {
            double drift, bias, residual;
            generated_sample(run, sample_index, &drift, &bias, &residual);
            CHECK(sc_hybrid_stats_push(&state, drift, bias, residual));
            reference_push(&reference, drift, bias, residual);
            if (reference.count < SC_HYBRID_STATS_CAPACITY)
            {
                CHECK(!sc_hybrid_stats_get(&state, SC_HYBRID_STATS_CAPACITY, &actual));
                continue;
            }

            CHECK(reference_get(&reference, SC_HYBRID_STATS_CAPACITY, &expected));
            CHECK(sc_hybrid_stats_get(&state, SC_HYBRID_STATS_CAPACITY, &actual));
            CHECK(
                compare_reference(&actual, &expected, run, sample_index, SC_HYBRID_STATS_CAPACITY));

            /* These windows sit within constant-slope epochs. The finite noise
             * sequence shifts fitted slopes by a few thousandths of a ppb from
             * the nominal 0.03 ppb boundary. RMS remains just below 5 ns. These
             * fixture checks do not relax the tight reference tolerances above. */
            if (run == 0U && (sample_index == 300U || sample_index == 800U))
            {
                CHECK(fabs(expected.bias_slope_ppb) > 0.025 &&
                      fabs(expected.bias_slope_ppb) < 0.035);
                CHECK(fabs(expected.residual_slope_ppb) > 0.025 &&
                      fabs(expected.residual_slope_ppb) < 0.035);
                CHECK(expected.residual_detrended_rms_ns > 4.5 &&
                      expected.residual_detrended_rms_ns < 4.9);
            }

            /* The 100-point acquisition window remains a generic scan and
             * must agree with the same independent reference on wrap. */
            if (sample_index % 17U == 0U || sample_index == 300U || sample_index == 800U)
            {
                CHECK(reference_get(&reference, 100U, &expected));
                CHECK(sc_hybrid_stats_get(&state, 100U, &actual));
                CHECK(compare_reference(&actual, &expected, run, sample_index, 100U));
            }
        }
        CHECK(state.accepted == 2500U && state.count == SC_HYBRID_STATS_CAPACITY);
    }
    return 0;
}

int main(void)
{
    CHECK(test_analytic() == 0);
    CHECK(test_ring_and_rejection() == 0);
    CHECK(test_residual_detrend() == 0);
    CHECK(test_small_residual_large_phase_offset() == 0);
    CHECK(test_nearly_linear_window() == 0);
    CHECK(test_nearly_constant_spread() == 0);
    CHECK(test_cached_full_window_against_reference() == 0);
    return 0;
}
