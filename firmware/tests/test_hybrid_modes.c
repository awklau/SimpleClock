/* SPDX-License-Identifier: MIT */
/* Real controller/FCW, ideal unity plant. These are not hardware ADEV tests. */
#include "simpleclock/hybrid.h"
#include "simpleclock/sit5801.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static sc_hybrid_t s;
static sc_hybrid_input_t in;
static int64_t word;
static double physical, bias, reported_drift_error, bias_step;

static void initialize(sc_hybrid_loop_mode_t mode, double offset, unsigned source)
{
    sc_hybrid_init(&s);
    for (unsigned profile = 0U; profile < 3U; profile++)
    {
        const double tf = profile == 0U ? 60.0 : 120.0;
        const double tp = profile == 0U ? 600.0 : 3000.0;
        (void)sc_hybrid_profile_parameters(
            &s, (sc_hybrid_profile_t)profile,
            mode == SC_HYBRID_LOOP_PHASE || mode == SC_HYBRID_LOOP_OFF ? 0.0 : tf,
            mode == SC_HYBRID_LOOP_FREQUENCY || mode == SC_HYBRID_LOOP_OFF ? 0.0 : tp);
    }
    (void)sit5801_ppb_to_fcw(1.25, &word);
    in = (sc_hybrid_input_t){
        .now_ms = UINT32_MAX - 70000U,
        .epoch_ms = 1000000U,
    };
    in.gnss_valid = source == 10U;
    in.sbas_valid = source == 6U;
    in.standalone_valid = source == 1U;
    in.time_system = source == 10U ? 1U : 0U;
    physical = offset - sit5801_fcw_to_ppb(word);
    bias = -400000.0;
    reported_drift_error = bias_step = 0.0;
    sc_hybrid_observed(&s, true, word, 32U, in.now_ms);
}

static void tick(void)
{
    in.now_ms += 1000U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    const double actual_drift = physical + sit5801_fcw_to_ppb(word);
    bias += actual_drift;
    in.bias_ns = bias + bias_step;
    in.drift_ppb = actual_drift + reported_drift_error;
    sc_hybrid_observed(&s, true, word, 32U, in.now_ms);
    sc_hybrid_service(&s, in.now_ms);
    sc_hybrid_sample(&s, &in);
    int64_t next;
    if (sc_hybrid_take_command(&s, in.now_ms, &next))
    {
        word = next;
        sc_hybrid_written(&s, true, in.now_ms);
        sc_hybrid_observed(&s, true, word, 32U, in.now_ms);
    }
}

static int test_frequency_only(void)
{
    const unsigned sources[] = {10U, 6U, 1U};
    for (unsigned j = 0U; j < 3U; j++)
    {
        initialize(SC_HYBRID_LOOP_FREQUENCY, 0.037, sources[j]);
        for (unsigned i = 0U; i < 500U; i++)
        {
            tick();
        }
        CHECK(s.warm && s.state == SC_HYBRID_FREQUENCY && !s.phase_enabled && !s.phase_valid);
        CHECK(s.desired_drift_ppb == 0.0 && s.phase_integral_ppb == 0.0);
        CHECK(fabs(physical + s.applied_ppb) < 0.00001);
        /* Untrusted phase jumps must not become frequency commands. */
        physical = -s.applied_ppb;
        const int64_t held = word;
        for (unsigned i = 0U; i < 200U; i++)
        {
            bias_step = i % 2U ? 100.0 : -100.0;
            tick();
            CHECK(word == held && !s.phase_enabled && s.desired_drift_ppb == 0.0);
        }
    }
    return 0;
}

static int test_phase_only(void)
{
    const unsigned sources[] = {10U, 6U, 1U};
    for (unsigned j = 0U; j < 3U; j++)
    {
        initialize(SC_HYBRID_LOOP_PHASE, 0.0, sources[j]);
        reported_drift_error = 0.02; /* Within retained bias/drift integrity guard. */
        for (unsigned i = 0U; i < 500U; i++)
        {
            tick();
        }
        /* Every PHASE start acquires frequency once before the phase PI. */
        CHECK(s.phase_enabled && s.phase_valid && s.warm && s.attempts == 1U);
        CHECK(s.desired_drift_ppb == 0.0);
    }
    /* Either sign of a >1 ppb cold offset: one coarse acquisition removes it,
     * then the direct phase PI holds frequency against a fixed reference. */
    const double offsets[] = {2.0, -2.0};
    for (unsigned j = 0U; j < 2U; j++)
    {
        initialize(SC_HYBRID_LOOP_PHASE, offsets[j], 10U);
        for (unsigned i = 0U; i < 1000U && !s.phase_enabled; i++)
        {
            tick();
        }
        CHECK(s.phase_enabled && s.phase_valid && s.attempts == 1U);
        CHECK(fabs(physical + s.applied_ppb) <= 0.05); /* Acquisition residual limit. */
        const double reference = s.phase_reference_ns;
        for (unsigned i = 0U; i < 15000U; i++)
        {
            const double before = s.applied_ppb;
            tick();
            CHECK(s.attempts == 1U && s.phase_reference_ns == reference);
            CHECK(fabs(s.applied_ppb - before) <= 0.003004);
            CHECK(s.applied_ppb >= s.lower_ppb && s.applied_ppb <= s.upper_ppb);
        }
        CHECK(s.phase_enabled && s.locked);
        CHECK(fabs(physical + s.applied_ppb) < 0.00001 && fabs(s.filtered_error_ns) < 0.02);
        /* A persistent 10 ns measurement shift is neither rejected nor
         * requalified: the 2 ns clip passes it at <= 2 ns per sample while
         * phase steering continues on the same reference at the slew cap. */
        const uint32_t clipped = s.clipped_samples;
        bias_step = 10.0;
        tick();
        CHECK(s.phase_enabled && s.input_clipped && s.phase_reference_ns == reference);
        CHECK(fabs(s.phase_innovation_ns - 10.0) < 0.01);
        CHECK(s.last_failure.flags & SC_HYBRID_LOCK_INNOVATION);
        CHECK(fabs(s.last_failure.bias_ns - in.bias_ns) < 1e-9); /* Raw trigger kept. */
        for (unsigned i = 0U; i < 30U; i++)
        {
            const double before = s.applied_ppb;
            tick();
            CHECK(s.phase_enabled && s.phase_reference_ns == reference);
            CHECK(fabs(s.applied_ppb - before) <= 0.003004);
        }
        CHECK(s.clipped_samples - clipped >= 4U && s.clipped_samples - clipped <= 5U);
        CHECK(!s.input_clipped);
        for (unsigned i = 0U; i < 15000U; i++)
        {
            tick();
            CHECK(s.phase_enabled && s.phase_reference_ns == reference);
        }
        CHECK(s.locked && fabs(physical + s.applied_ppb) < 0.00001);
        CHECK(fabs(s.filtered_error_ns) < 0.02);
        const int64_t before_loss = word;
        in.gnss_valid = false;
        tick(); /* Bridged: trim held, authority kept. */
        CHECK(sc_hybrid_tracking(&s) && s.phase_enabled && word == before_loss);
        for (unsigned i = 0U; i < 11U; i++)
        {
            tick();
        }
        CHECK(s.state == SC_HYBRID_HOLDOVER && word == before_loss);
    }
    return 0;
}

static int test_switches_and_parameters(void)
{
    initialize(SC_HYBRID_LOOP_BOTH, 0.0, 10U);
    for (unsigned i = 0U; i < 300U; i++)
    {
        tick();
    }
    CHECK(s.phase_enabled && s.locked);
    physical += 0.02;
    for (unsigned i = 0U; i < 100U; i++)
    {
        tick();
    }
    const double reference = s.phase_reference_ns, accumulator = s.accumulator_ppb;
    const int64_t held = word;
    const unsigned count = s.history.count;
    const double v = s.desired_drift_ppb;
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 600.0, 6000.0));
    CHECK(s.enabled && word == held && s.accumulator_ppb == accumulator &&
          s.history.count == count);
    CHECK(s.phase_reference_ns == reference);
    CHECK(fabs(s.phase_integral_ppb - 2.0 / 6000.0 * s.filtered_error_ns - v) < 1e-14);
    const double integral = s.phase_integral_ppb;
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_SBAS, 1200.0, 12000.0));
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_STANDALONE, 300.0, 3000.0));
    CHECK(s.phase_integral_ppb == integral && sc_hybrid_frequency_time(&s) == 600.0);
    const double bad[][2] = {{NAN, 600}, {60, INFINITY}, {-1, 600},  {9, 600},
                             {60, 9},    {86401, 0},     {0, 86401}, {10.5, 600}};
    for (unsigned i = 0U; i < sizeof(bad) / sizeof(bad[0]); i++)
    {
        CHECK(!sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, bad[i][0], bad[i][1]));
    }
    CHECK(!sc_hybrid_profile_parameters(&s, (sc_hybrid_profile_t)4, 60, 600));
    s.pending = true;
    CHECK(!sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 0, 0));
    s.pending = false;
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 60, 0));
    CHECK(s.loop_mode == SC_HYBRID_LOOP_FREQUENCY && !s.phase_enabled && s.desired_drift_ppb == 0);
    CHECK(word == held && s.accumulator_ppb == accumulator && !s.request_ready);
    for (unsigned i = 0U; i < 1000U; i++)
    {
        tick();
    }
    CHECK(!s.phase_enabled && fabs(physical + s.applied_ppb) < 0.00001);
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 0, 1200));
    CHECK(s.loop_mode == SC_HYBRID_LOOP_PHASE);
    for (unsigned i = 0U; i < 121U; i++)
    {
        tick();
    }
    CHECK(s.phase_enabled);
    const double before = s.accumulator_ppb;
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 60, 600));
    CHECK(s.loop_mode == SC_HYBRID_LOOP_BOTH && s.phase_enabled && s.accumulator_ppb == before);
    const int64_t stopped = word;
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 0, 0));
    CHECK(s.loop_mode == SC_HYBRID_LOOP_OFF && !s.phase_enabled);
    physical += 0.02;
    const unsigned writes = s.writes;
    for (unsigned i = 0U; i < 300U; i++)
    {
        tick();
    }
    CHECK(word == stopped && s.writes == writes && s.state == SC_HYBRID_DISABLED);
    CHECK(s.enabled && s.previous_valid && s.history.count == SC_HYBRID_TRACKING_POINTS);
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 100, 0));
    tick();
    CHECK(s.loop_mode == SC_HYBRID_LOOP_FREQUENCY && sc_hybrid_tracking(&s));
    sc_hybrid_hold(&s);
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 86400, 0));
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 0, 86400));
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 34560, 86400));
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 34561, 86400));
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 86400, 86400));
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 86400, 10));
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_SBAS, 10, 10));
    CHECK(!s.enabled && s.state == SC_HYBRID_MANUAL_HOLD);
    CHECK(strcmp(sc_hybrid_loop_name(SC_HYBRID_LOOP_OFF), "OFF") == 0);
    CHECK(strcmp(sc_hybrid_loop_name((sc_hybrid_loop_mode_t)4), "INVALID") == 0);
    /* Disabled cold startup must not acquire even a large drift offset. */
    initialize(SC_HYBRID_LOOP_OFF, 2.0, 10U);
    for (unsigned i = 0U; i < 500U; i++)
    {
        tick();
    }
    CHECK(s.attempts == 0U && s.writes == 0U && !s.warm);
    return 0;
}

static int test_fast_time_constants(void)
{
    const sc_hybrid_loop_mode_t modes[] = {SC_HYBRID_LOOP_BOTH, SC_HYBRID_LOOP_FREQUENCY,
                                           SC_HYBRID_LOOP_PHASE};
    for (unsigned mode = 0U; mode < 3U; mode++)
    {
        initialize(modes[mode], 0.02, 6U);
        const double phase_time = modes[mode] == SC_HYBRID_LOOP_BOTH ? 25.0 : 10.0;
        CHECK(sc_hybrid_profile_parameters(
            &s, SC_HYBRID_PROFILE_PPP, modes[mode] == SC_HYBRID_LOOP_PHASE ? 0.0 : 10.0,
            modes[mode] == SC_HYBRID_LOOP_FREQUENCY ? 0.0 : phase_time));
        CHECK(sc_hybrid_profile_parameters(
            &s, SC_HYBRID_PROFILE_SBAS, modes[mode] == SC_HYBRID_LOOP_PHASE ? 0.0 : 10.0,
            modes[mode] == SC_HYBRID_LOOP_FREQUENCY ? 0.0 : phase_time));
        CHECK(!sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 9.0, phase_time));
        CHECK(!sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 10.0, 9.0));
        for (unsigned i = 0U; i < 1000U; i++)
        {
            tick();
        }
        CHECK(s.state != SC_HYBRID_ACQUISITION_FAULT && s.known);
        physical += 0.01;
        for (unsigned i = 0U; i < 1500U; i++)
        {
            const double before = s.applied_ppb;
            tick();
            CHECK(fabs(s.applied_ppb - before) <= 0.003004);
        }
        CHECK(fabs(physical + s.applied_ppb) < 0.00001);
        if (modes[mode] != SC_HYBRID_LOOP_FREQUENCY)
        {
            CHECK(s.phase_enabled && s.locked && fabs(s.filtered_error_ns) < 0.02);
        }
    }
    return 0;
}

int main(void)
{
    CHECK(test_frequency_only() == 0);
    CHECK(test_phase_only() == 0);
    CHECK(test_switches_and_parameters() == 0);
    CHECK(test_fast_time_constants() == 0);
    return 0;
}
