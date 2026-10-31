/* SPDX-License-Identifier: MIT */
/* Independent adversarial checks for the public hybrid controller API. */
#include "simpleclock/hybrid.h"
#include "simpleclock/sit5801.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define CHECK(expr)                                                                                \
    do                                                                                             \
    {                                                                                              \
        if (!(expr))                                                                               \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #expr);                                     \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

/* Fixture parameters, not compiled defaults: these scenarios exercise BOTH-mode
 * FLL acquisition with PPP 60/600 s and SBAS/standalone 120/3000 s. */
static void legacy_profiles(sc_hybrid_t *h)
{
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_PPP, 60.0, 600.0);
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_SBAS, 120.0, 3000.0);
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_STANDALONE, 120.0, 3000.0);
}

static sc_hybrid_input_t sample_input(uint64_t epoch_ms, uint32_t now_ms, double bias_ns,
                                      double drift_ppb)
{
    sc_hybrid_input_t in = {0};
    in.epoch_ms = epoch_ms;
    in.now_ms = now_ms;
    in.received_ms = now_ms;
    in.bias_ns = bias_ns;
    in.drift_ppb = drift_ppb;
    in.gnss_valid = true;
    return in;
}

/* The public API has no test-only transition into TRACK. These fixtures set
 * state intentionally, with a verified word and the same fields enter_track
 * establishes in production. */
static void force_track(sc_hybrid_t *s, uint32_t now_ms, double acquired_ppb)
{
    int64_t word = 0;
    (void)sit5801_ppb_to_fcw(acquired_ppb, &word);
    sc_hybrid_init(s);
    legacy_profiles(s);
    sc_hybrid_observed(s, true, word, 32U, now_ms);
    s->state = SC_HYBRID_TRACK;
    s->acquired_ppb = s->applied_ppb;
    s->accumulator_ppb = s->requested_ppb = s->applied_ppb;
    s->phase_reference_ns = 0.0;
    s->phase_valid = true;
    s->phase_enabled = s->warm = true;
    s->source_mode = 10U;
    s->phase_integral_ppb = 0.0;
    s->lower_ppb = -10.0;
    s->upper_ppb = 10.0;
    s->last_received_ms = now_ms;
    s->observed_ms = now_ms;
    s->applied_ms = now_ms;
}

static int complete_command(sc_hybrid_t *s, const sc_hybrid_input_t *in, int64_t *word)
{
    if (!sc_hybrid_take_command(s, in->now_ms, word))
    {
        return 0;
    }
    sc_hybrid_written(s, true, in->now_ms);
    sc_hybrid_observed(s, true, *word, 32U, in->now_ms);
    return 1;
}

static int test_subquantum_both_signs(void)
{
    sc_hybrid_t s;
    force_track(&s, 100000U, 0.0);
    s.phase_integral_ppb = 1.0e-6;
    uint64_t epoch = UINT64_C(1000000);
    uint32_t now = 100000U;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        now += 1000U;
        epoch += 1000U;
        sc_hybrid_input_t in = sample_input(epoch, now, 0.0, 0.0);
        sc_hybrid_sample(&s, &in);
        int64_t ignored = 0;
        CHECK(!complete_command(&s, &in, &ignored));
    }
    const double positive = s.accumulator_ppb;
    const double quantum = 800000.0 / (double)SIT5801_FCW_MAX;
    CHECK(positive > 0.0 && positive < quantum && s.applied_word == 0);

    force_track(&s, 100000U, 0.0);
    s.phase_integral_ppb = -1.0e-6;
    epoch = UINT64_C(1000000);
    now = 100000U;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        now += 1000U;
        epoch += 1000U;
        sc_hybrid_input_t in = sample_input(epoch, now, 0.0, 0.0);
        sc_hybrid_sample(&s, &in);
        int64_t ignored = 0;
        CHECK(!complete_command(&s, &in, &ignored));
    }
    CHECK(s.accumulator_ppb < 0.0 && s.accumulator_ppb > -quantum && s.applied_word == 0);
    return 0;
}

static int test_signed_slew_and_hard_span(void)
{
    sc_hybrid_t s;
    uint64_t epoch = UINT64_C(1000000);
    uint32_t now = 100000U;
    force_track(&s, now, 0.0);
    s.accumulator_ppb = 0.1;
    s.phase_integral_ppb = 0.1;
    now += 1000U;
    epoch += 1000U;
    sc_hybrid_input_t in = sample_input(epoch, now, 0.0, 0.0);
    sc_hybrid_sample(&s, &in);
    CHECK(s.slew_limited && s.requested_ppb > 0.0 && s.requested_ppb <= 0.003000001);

    force_track(&s, 100000U, 0.0);
    s.accumulator_ppb = -0.1;
    s.phase_integral_ppb = -0.1;
    in = sample_input(UINT64_C(1001000), 101000U, 0.0, 0.0);
    sc_hybrid_sample(&s, &in);
    CHECK(s.slew_limited && s.requested_ppb < 0.0 && s.requested_ppb >= -0.003000001);

    force_track(&s, 100000U, 0.0);
    s.accumulator_ppb = 1.0;
    s.phase_integral_ppb = 0.1;
    in = sample_input(UINT64_C(1001000), 101000U, 0.0, 0.0);
    sc_hybrid_sample(&s, &in);
    CHECK(s.state == SC_HYBRID_HOLDOVER && s.reason == SC_HYBRID_Q_LIMIT);

    force_track(&s, 100000U, 0.0);
    s.accumulator_ppb = -1.0;
    s.phase_integral_ppb = -0.1;
    in = sample_input(UINT64_C(1001000), 101000U, 0.0, 0.0);
    sc_hybrid_sample(&s, &in);
    CHECK(s.state == SC_HYBRID_HOLDOVER && s.reason == SC_HYBRID_Q_LIMIT);
    return 0;
}

static int test_conditional_antiwindup_releases_both_signs(void)
{
    sc_hybrid_t s;
    uint64_t epoch = UINT64_C(1000000);
    uint32_t now = 100000U;
    force_track(&s, now, 0.0);
    s.phase_integral_ppb = 0.2;
    sc_hybrid_input_t in = sample_input(epoch + 1000U, now + 1000U, -100000.0, 0.0);
    sc_hybrid_sample(&s, &in);
    const double blocked_positive = s.phase_integral_ppb;
    CHECK(fabs(blocked_positive - 0.2) < 1.0e-12);
    s.previous_valid = false;  /* Deliberately bypass continuity for a sign reversal. */
    s.filtered_error_ns = 0.0; /* A recovered estimator points back into the range. */
    in = sample_input(epoch + 2000U, now + 2000U, 1.0, 0.0);
    sc_hybrid_sample(&s, &in);
    CHECK(s.phase_integral_ppb < blocked_positive);

    force_track(&s, now, 0.0);
    s.phase_integral_ppb = -0.2;
    in = sample_input(epoch + 1000U, now + 1000U, 100000.0, 0.0);
    sc_hybrid_sample(&s, &in);
    const double blocked_negative = s.phase_integral_ppb;
    CHECK(fabs(blocked_negative + 0.2) < 1.0e-12);
    s.previous_valid = false;
    s.filtered_error_ns = 0.0;
    in = sample_input(epoch + 2000U, now + 2000U, -1.0, 0.0);
    sc_hybrid_sample(&s, &in);
    CHECK(s.phase_integral_ppb > blocked_negative);
    return 0;
}

static int test_stale_stream_no_catchup(void)
{
    sc_hybrid_t s;
    sc_hybrid_init(&s);
    legacy_profiles(&s);
    sc_hybrid_observed(&s, true, 0, 32U, 100000U);
    sc_hybrid_input_t in = sample_input(UINT64_C(1000000), 100000U, 0.0, 0.0);
    sc_hybrid_sample(&s, &in);
    in.now_ms += 3001U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    sc_hybrid_sample(&s, &in);
    CHECK(s.state == SC_HYBRID_HOLDOVER && !s.qualifying);
    for (unsigned i = 0U; i < 130U; ++i)
    {
        in.now_ms += 1U;
        in.received_ms = in.now_ms;
        in.epoch_ms += 1000U;
        sc_hybrid_sample(&s, &in);
    }
    CHECK(s.state == SC_HYBRID_QUALIFY && s.attempts == 0U && s.writes == 0U);
    return 0;
}

static int test_attempt_latch_and_fault_freeze(void)
{
    sc_hybrid_t s;
    sc_hybrid_init(&s);
    legacy_profiles(&s);
    sc_hybrid_observed(&s, true, 0, 32U, 100000U);
    s.state = SC_HYBRID_VERIFY;
    s.attempts = 3U;
    uint64_t epoch = UINT64_C(1000000);
    uint32_t now = 100000U;
    double bias = 0.0;
    for (unsigned i = 0U; i < 100U; ++i)
    {
        now += 1000U;
        epoch += 1000U;
        bias += 1.0;
        sc_hybrid_input_t in = sample_input(epoch, now, bias, 1.0);
        sc_hybrid_sample(&s, &in);
    }
    CHECK(s.state == SC_HYBRID_ACQUISITION_FAULT && s.acquisition_latched);
    const double target = s.acquire_target_ppb;
    const double accumulator = s.accumulator_ppb;
    now += 1000U;
    epoch += 1000U;
    sc_hybrid_input_t in = sample_input(epoch, now, bias + 100.0, -50.0);
    sc_hybrid_sample(&s, &in);
    int64_t word = 0;
    CHECK(!sc_hybrid_take_command(&s, now, &word));
    CHECK(s.state == SC_HYBRID_ACQUISITION_FAULT && s.acquire_target_ppb == target &&
          s.accumulator_ppb == accumulator);
    return 0;
}

static int test_write_faults_and_bad_point(void)
{
    sc_hybrid_t s;
    int64_t requested = 0;
    CHECK(sit5801_ppb_to_fcw(1.0, &requested));
    sc_hybrid_init(&s);
    legacy_profiles(&s);
    sc_hybrid_observed(&s, true, 0, 32U, 100000U);
    s.requested_word = requested;
    s.pending = true;
    sc_hybrid_observed(&s, true, 0, 32U, 100100U);
    CHECK(!s.known && s.state == SC_HYBRID_ACTUATOR_UNKNOWN);
    sc_hybrid_observed(&s, true, 0, 32U, 100200U);
    sc_hybrid_observed(&s, true, 0, 32U, 100300U);
    CHECK(s.known && s.state == SC_HYBRID_HOLDOVER);

    sc_hybrid_init(&s);

    legacy_profiles(&s);
    sc_hybrid_observed(&s, true, 0, 32U, 1000U);
    s.pending = true;
    s.pending_ms = 1000U;
    sc_hybrid_service(&s, 3501U);
    CHECK(!s.known && s.state == SC_HYBRID_ACTUATOR_UNKNOWN);

    sc_hybrid_init(&s);

    legacy_profiles(&s);
    sc_hybrid_observed(&s, true, 0, 32U, 100000U);
    sc_hybrid_input_t in = sample_input(UINT64_C(1000000), 100000U, 0.0, 0.0);
    sc_hybrid_sample(&s, &in);
    CHECK(s.qualifying && s.history.count == 1U);
    in.now_ms += 1000U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    in.bias_ns = 100.0;
    sc_hybrid_sample(&s, &in);
    /* A 100 ns point is clipped, not rejected: qualification keeps its history
     * but the point enters it, and the controller, as prediction + 2 ns. */
    CHECK(s.state == SC_HYBRID_QUALIFY && s.qualifying && s.history.count == 2U);
    CHECK(s.input_clipped && s.clipped_samples == 1U && s.phase_good_samples == 2U);
    CHECK(fabs(s.previous_bias_ns - 2.0) < 1e-12 && fabs(s.phase_innovation_ns - 100.0) < 1e-12);
    CHECK(!s.phase_enabled && s.last_failure.bias_ns == 100.0);
    return 0;
}

static int test_coordinate_policy(void)
{
    sc_hybrid_t s;
    force_track(&s, 100000U, 0.0);
    sc_hybrid_input_t in = sample_input(UINT64_C(1001000), 101000U, 0.0, 0.0);
    sc_hybrid_sample(&s, &in);
    CHECK(s.phase_valid);
    in.now_ms += 1000U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    in.gnss_valid = false;
    sc_hybrid_sample(&s, &in);
    CHECK(sc_hybrid_tracking(&s) && s.phase_valid && s.reason == SC_HYBRID_Q_GNSS);

    /* A flagged coordinate change is never bridged, even after a short gap. */
    in.gnss_valid = true;
    in.coordinate_changed = true;
    in.now_ms += 1000U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    sc_hybrid_sample(&s, &in);
    CHECK(!s.phase_valid && !s.phase_enabled && s.state == SC_HYBRID_HOLDOVER);
    return 0;
}

static int test_moving_thermal_target_keeps_fll(void)
{
    sc_hybrid_t s;
    force_track(&s, 100000U, 0.0);
    uint64_t epoch = UINT64_C(1000000);
    uint32_t now = 100000U;
    double bias = 0.0;
    unsigned writes = 0U;
    for (unsigned i = 0U; i < 40U; ++i)
    {
        now += 1000U;
        epoch += 1000U;
        bias += 0.05;
        sc_hybrid_input_t in = sample_input(epoch, now, bias, 0.05);
        sc_hybrid_sample(&s, &in);
        int64_t word = 0;
        writes += (unsigned)complete_command(&s, &in, &word);
        CHECK(sc_hybrid_tracking(&s));
    }
    CHECK(writes > 0U && s.accumulator_ppb < 0.0 && s.applied_ppb < 0.0);
    return 0;
}

static int test_frequency_settled_is_separate_from_phase_lock(void)
{
    for (unsigned scenario = 0U; scenario < 4U; ++scenario)
    {
        sc_hybrid_t s;
        uint32_t now = UINT32_MAX - 60000U;
        uint64_t epoch = UINT64_C(1000000);
        force_track(&s, now, 0.0);
        /* A constant 6 ns offset fails the relative phase band. Set v initially
         * to 0, +0.06 or +0.08 ppb to test the independent 0.05 ppb gates. */
        s.phase_error_ns = s.filtered_error_ns = 6.0;
        s.phase_integral_ppb = 0.02 + (scenario == 1U ? 0.06 : scenario == 2U ? 0.08 : 0.0);
        double errors[50] = {0};
        sc_hybrid_input_t in = {0};
        for (unsigned i = 0; i < 270U; ++i)
        {
            now += 1000U;
            epoch += 1000U;
            const double drift = scenario == 1U   ? 0.06
                                 : scenario == 3U ? (i % 2U ? 0.12 : -0.12)
                                                  : 0.0;
            in = sample_input(epoch, now, 6.0, drift);
            sc_hybrid_sample(&s, &in);
            CHECK(sc_hybrid_tracking(&s));
            errors[i % 50U] = drift - s.desired_drift_ppb;
            int64_t proposed = 0;
            (void)complete_command(&s, &in, &proposed);
            sc_hybrid_observed(&s, true, s.applied_word, 32U, now);
            if (i >= 49U)
            {
                double expected = 0.0;
                for (unsigned j = 0; j < 50U; ++j)
                {
                    expected += errors[j] / 50.0;
                }
                CHECK(fabs(s.frequency.mean_error_ppb - expected) < 2e-6);
            }
        }
        CHECK(!s.locked && s.frequency.window_valid);
        CHECK(s.frequency.settled == (scenario == 0U));
        if (scenario == 1U)
        {
            CHECK(fabs(s.frequency.mean_error_ppb) < 0.05);
            CHECK(fabs(s.frequency.mean_drift_ppb - 0.06) < 1e-8);
        }
        if (scenario == 2U)
        {
            CHECK(fabs(s.frequency.mean_drift_ppb) < 1e-8);
            CHECK(fabs(s.frequency.mean_error_ppb) > 0.05);
        }
        if (scenario == 3U)
        {
            CHECK(fabs(s.frequency.mean_drift_ppb) < 1e-8);
            CHECK(s.metrics.drift_sd_ppb > 0.1);
        }
        if (scenario == 0U)
        {
            const uint32_t attempts = s.attempts;
            const uint32_t generation = s.generation;
            /* A single accepted noise sample is averaged, not an automatic
             * acquisition restart. A sustained excursion can remove readiness,
             * but retains the windows and recovers without qualifying again. */
            unsigned recovery = 0U;
            for (unsigned i = 0U; i < 91U; ++i)
            {
                now += 1000U;
                epoch += 1000U;
                const double drift = i == 0U ? 0.12 : i <= 30U ? 0.10 : 0.0;
                in = sample_input(epoch, now, 6.0, drift);
                sc_hybrid_sample(&s, &in);
                int64_t proposed = 0;
                (void)complete_command(&s, &in, &proposed);
                sc_hybrid_observed(&s, true, s.applied_word, 32U, now);
                CHECK(sc_hybrid_tracking(&s) && s.history.count == 121U && s.phase_valid);
                CHECK(s.attempts == attempts && s.generation == generation);
                if (i == 0U)
                {
                    CHECK(s.frequency.settled);
                }
                if (i == 30U)
                {
                    CHECK(!s.frequency.settled && s.frequency.window_valid);
                }
                if (i > 30U && s.frequency.settled)
                {
                    recovery = i - 30U;
                    break;
                }
            }
            CHECK(recovery >= 10U && recovery <= 60U);
        }
        in.now_ms += 1000U;
        in.received_ms = in.now_ms;
        in.epoch_ms += 1000U;
        in.gnss_valid = false;
        sc_hybrid_sample(&s, &in);
        CHECK(s.frequency.window_valid && s.frequency.count == 121U);
        sc_frequency_monitor_service(&s.frequency, in.now_ms + SC_FREQUENCY_HOLDOVER_MS);
        CHECK(!s.frequency.settled && !s.frequency.window_valid && s.frequency.count == 0U);
    }
    return 0;
}

static int test_frequency_threshold_boundaries(void)
{
    /* Exercise each side of BOTH limits in BOTH directions. A matching
     * nonzero setpoint isolates near-zero drift; zero drift with an unmatched
     * setpoint isolates servo error. Neither case changes the phase target. */
    for (unsigned scenario = 0U; scenario < 8U; ++scenario)
    {
        sc_hybrid_t s;
        uint32_t now = UINT32_MAX - 60000U;
        uint64_t epoch = UINT64_C(1000000);
        force_track(&s, now, 0.0);
        const double sign = scenario < 4U ? 1.0 : -1.0;
        const bool inside = scenario % 2U == 0U;
        const double requested = sign * (inside ? 0.049 : 0.051);
        const double drift = scenario % 4U < 2U ? requested : 0.0;
        s.phase_integral_ppb = requested; /* Zero phase error keeps v constant. */
        for (unsigned i = 0U; i < 180U; ++i)
        {
            now += 1000U;
            epoch += 1000U;
            sc_hybrid_input_t in = sample_input(epoch, now, 0.0, drift);
            sc_hybrid_sample(&s, &in);
            int64_t proposed = 0;
            (void)complete_command(&s, &in, &proposed);
            sc_hybrid_observed(&s, true, s.applied_word, 32U, now);
        }
        CHECK(s.frequency.window_valid && s.history.count == 121U);
        CHECK(s.frequency.settled == inside);
        CHECK(s.phase_valid && s.phase_reference_ns == 0.0 && s.attempts == 0U);
        CHECK(s.locked && sc_hybrid_tracking(&s));
    }
    return 0;
}

int main(void)
{
    /* A partial transaction can decode to an in-hardware-range word outside
     * the automatic boot span. Two reads must not expand that envelope. */
    sc_hybrid_t bounded;
    sc_hybrid_init(&bounded);
    legacy_profiles(&bounded);
    sc_hybrid_observed(&bounded, true, 0, 32U, 1000U);
    int64_t outside = 0;
    CHECK(sit5801_ppb_to_fcw(11.0, &outside));
    sc_hybrid_observed(&bounded, false, 0, 32U, 2000U);
    sc_hybrid_observed(&bounded, true, outside, 32U, 3000U);
    sc_hybrid_observed(&bounded, true, outside, 32U, 4000U);
    CHECK(!bounded.known && bounded.state == SC_HYBRID_ACTUATOR_UNKNOWN);
    CHECK((bounded.reason & SC_HYBRID_Q_LIMIT) != 0U);
    for (int sign = -1; sign <= 1; sign += 2)
    {
        sc_hybrid_init(&bounded);
        legacy_profiles(&bounded);
        sc_hybrid_observed(&bounded, true, 0, 32U, 1000U);
        int64_t endpoint = 0;
        CHECK(sit5801_ppb_to_fcw((double)sign * 10.0, &endpoint));
        if (sit5801_fcw_to_ppb(endpoint) < -10.0)
        {
            endpoint++;
        }
        if (sit5801_fcw_to_ppb(endpoint) > 10.0)
        {
            endpoint--;
        }
        bounded.applied_word = endpoint;
        bounded.applied_ppb = sit5801_fcw_to_ppb(endpoint);
        bounded.acquire_target_ppb = (double)sign * 10.0;
        bounded.state = SC_HYBRID_ACQUIRE;
        bounded.gnss_good = true;
        bounded.last_received_ms = bounded.observed_ms = 2000U;
        int64_t next = 0;
        CHECK(!sc_hybrid_take_command(&bounded, 2000U, &next));
        CHECK(bounded.state == SC_HYBRID_VERIFY);
    }
    CHECK(test_subquantum_both_signs() == 0);
    CHECK(test_signed_slew_and_hard_span() == 0);
    CHECK(test_conditional_antiwindup_releases_both_signs() == 0);
    CHECK(test_stale_stream_no_catchup() == 0);
    CHECK(test_attempt_latch_and_fault_freeze() == 0);
    CHECK(test_write_faults_and_bad_point() == 0);
    CHECK(test_coordinate_policy() == 0);
    CHECK(test_moving_thermal_target_keeps_fll() == 0);
    CHECK(test_frequency_settled_is_separate_from_phase_lock() == 0);
    CHECK(test_frequency_threshold_boundaries() == 0);
    puts("Hybrid adversarial fault checks passed");
    return 0;
}
