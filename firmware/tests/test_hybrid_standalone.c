/* SPDX-License-Identifier: MIT */
/* Actual controller and FCW conversion with an explicitly ideal unity plant. */
#include "simpleclock/hybrid.h"
#include "simpleclock/sit5801.h"
#include <math.h>
#include <stdio.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s state=%s\n", __LINE__, #x,                                \
                    sc_hybrid_state_name(s.state));                                                \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
static sc_hybrid_t s;
static sc_hybrid_input_t in;
static int64_t word;
static double physical, bias, domain_offset, noise;

/* Fixture parameters, not compiled defaults: these scenarios exercise BOTH-mode
 * FLL acquisition with PPP 60/600 s and SBAS/standalone 120/3000 s. */
static void legacy_profiles(sc_hybrid_t *h)
{
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_PPP, 60.0, 600.0);
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_SBAS, 120.0, 3000.0);
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_STANDALONE, 120.0, 3000.0);
}

static void mode(unsigned m)
{
    in.gnss_valid = m == 10U;
    in.standalone_valid = m == 1U;
    in.sbas_valid = m == 6U;
    in.frequency_valid = m != 0U;
    in.time_system = m == 10U ? 1U : 0U;
}

static void tick(void)
{
    in.now_ms += 1000U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    in.drift_ppb = physical + sit5801_fcw_to_ppb(word);
    bias += in.drift_ppb;
    in.bias_ns = bias + domain_offset + (in.epoch_ms % 2000U ? noise : -noise);
    sc_hybrid_observed(&s, true, word, 32U, in.now_ms);
    sc_hybrid_service(&s, in.now_ms);
    sc_hybrid_sample(&s, &in);
    int64_t proposed;
    if (sc_hybrid_take_command(&s, in.now_ms, &proposed))
    {
        word = proposed;
        sc_hybrid_written(&s, true, in.now_ms);
        sc_hybrid_observed(&s, true, word, 32U, in.now_ms);
    }
}

static void initialize(unsigned m, double offset)
{
    sc_hybrid_init(&s);
    legacy_profiles(&s);
    (void)sit5801_ppb_to_fcw(1.25, &word);
    physical = offset - sit5801_fcw_to_ppb(word);
    bias = 200000.0;
    domain_offset = noise = 0.0;
    in = (sc_hybrid_input_t){
        .now_ms = UINT32_MAX - 80000U,
        .epoch_ms = 1000000U,
    };
    mode(m);
}

static void start(unsigned m)
{
    initialize(m, 0.0);
    for (unsigned i = 0U; i < 260U; i++)
    {
        tick();
    }
}

int main(void)
{
    /* Any supported solution can acquire from cold, independent of PPP. */
    const unsigned modes[] = {10U, 6U, 1U};
    for (unsigned k = 0; k < 3U; k++)
    {
        start(modes[k]);
        CHECK(s.warm && s.phase_enabled && s.phase_valid && s.locked);
        CHECK(sc_hybrid_frequency_time(&s) == (k == 0U ? 60.0 : 120.0));
        CHECK(sc_hybrid_phase_time(&s) == (k == 0U ? 600.0 : 3000.0));
    }
    /* Live startup regression: coarse acquisition may move several ppb.
     * Fine entry must initialize from that verified endpoint, not boot trim.
     * Also cover smaller offsets that would evade the +/-1 ppb span guard. */
    const double offsets[] = {4.5, -4.5, 0.4, -0.4};
    for (unsigned k = 0U; k < 3U; k++)
    {
        for (unsigned j = 0U; j < 4U; j++)
        {
            initialize(modes[k], offsets[j]);
            bool entered = false;
            for (unsigned i = 0U; i < 700U; i++)
            {
                const bool was_warm = s.warm;
                const int64_t before = word;
                tick();
                CHECK(!(s.reason & SC_HYBRID_Q_LIMIT));
                if (!was_warm && s.warm)
                {
                    entered = true;
                    CHECK(word == before && s.accumulator_ppb == s.applied_ppb);
                    CHECK(fabs(s.applied_ppb - (1.25 - offsets[j])) < 0.00001);
                }
                if (entered)
                {
                    CHECK(fabs(s.applied_ppb - (1.25 - offsets[j])) < 0.00001);
                }
            }
            CHECK(entered && s.phase_enabled && s.locked);
        }
    }
    start(10U);
    CHECK(s.locked && s.frequency.settled);
    /* Nonzero delivered phase setpoint and fractional remainder survive a
     * contiguous source change. Only an ordinary FLL increment is permitted. */
    s.desired_drift_ppb = s.phase_integral_ppb = 0.012;
    s.accumulator_ppb += 0.0000002;
    const double fractional = s.accumulator_ppb;
    mode(6U);
    domain_offset = 100.0;
    tick();
    CHECK(s.phase_enabled && s.phase_valid && s.desired_drift_ppb == 0.012);
    CHECK(s.state == SC_HYBRID_PHASE_RECOVERING && !s.ramping && !s.locked);
    CHECK(fabs(s.accumulator_ppb - (fractional + 0.012 / 120.0)) < 1e-13);
    CHECK(s.filtered_error_ns == 0.0 && s.phase_integral_ppb == 0.012);
    const double previous_reference = s.phase_reference_ns;
    /* Every usable domain selects its gains and recaptures phase immediately. */
    const unsigned handovers[] = {1U, 10U, 6U, 1U};
    for (unsigned k = 0; k < 4U; k++)
    {
        const double v = s.desired_drift_ppb;
        mode(handovers[k]);
        domain_offset += k % 2U ? -100.0 : 20.0;
        tick();
        CHECK(s.phase_valid && s.phase_enabled && s.desired_drift_ppb == v);
        CHECK(s.phase_reference_ns == in.bias_ns && s.filtered_error_ns == 0.0);
        for (unsigned i = 0; i < 150U; i++)
        {
            const double before = s.applied_ppb;
            tick();
            CHECK(fabs(s.applied_ppb - before) <= 0.003004);
        }
        CHECK(s.phase_enabled && s.generation == k + 3U);
    }
    CHECK(s.phase_reference_ns != previous_reference);
    /* A time-system-only change also recaptures phase without a dwell. */
    in.time_system ^= 1U;
    domain_offset += 100.0;
    tick();
    CHECK(s.phase_valid && s.phase_enabled && s.phase_reference_ns == in.bias_ns);
    /* Very noisy phase must not prevent frequency regulation. Every +/-10 ns
     * sample is clipped; phase steering continues on the bounded estimate. */
    physical += 0.08;
    noise = 10.0;
    const uint32_t clipped = s.clipped_samples;
    for (unsigned i = 0; i < 200U; i++)
    {
        tick();
        CHECK(sc_hybrid_tracking(&s) && s.phase_enabled);
    }
    CHECK(s.clipped_samples - clipped >= 190U);
    CHECK(fabs(in.drift_ppb - s.desired_drift_ppb) < 0.03);
    noise = 0.0;
    for (unsigned i = 0; i < 500U; i++)
    {
        tick();
    }
    CHECK(s.phase_enabled);
    /* A 100 ns bias excursion keeps phase steering and its saved local
     * reference; the controller sees at most the clip on each sample. */
    const double anchor = s.phase_reference_ns, error_before = s.phase_error_ns;
    domain_offset += 100.0;
    tick();
    CHECK(s.phase_enabled && s.phase_valid && s.phase_reference_ns == anchor);
    CHECK(s.input_clipped && fabs(s.phase_error_ns - error_before) < 2.1);
    /* Missing data freezes trim and integrators. Duplicate return data cannot
     * manufacture a frequency dwell or refresh the accepted coordinate. */
    mode(0U);
    const int64_t held = word;
    const double integral = s.phase_integral_ppb;
    for (unsigned i = 0; i < 30U; i++)
    {
        tick();
        CHECK(word == held && s.phase_integral_ppb == integral);
    }
    mode(1U);
    tick();
    const unsigned accepted = s.history.accepted;
    for (unsigned i = 0; i < 10U; i++)
    {
        sc_hybrid_sample(&s, &in);
    }
    CHECK(s.history.accepted == accepted && word == held);
    for (unsigned i = 0; i < 200U; i++)
    {
        tick();
    }
    CHECK(sc_hybrid_tracking(&s));
    /* Manual hold still inhibits commands. */
    for (unsigned i = 0; i < 200U; i++)
    {
        tick();
    }
    sc_hybrid_hold(&s);
    const int64_t manual_word = word;
    for (unsigned i = 0; i < 130U; i++)
    {
        tick();
        CHECK(word == manual_word && !s.phase_enabled);
    }
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 120.0, 1200.0));
    sc_hybrid_resume(&s);
    for (unsigned i = 0; i < 140U; i++)
    {
        tick();
    }
    CHECK(s.phase_enabled && sc_hybrid_phase_time(&s) == 3000.0); /* PPP-only override */
    /* FLL response of the real accumulator with phase authority inhibited:
     * a 0.06 ppb disturbance relaxes by (1-1/T)^120, subject to FCW rounding. */
    for (unsigned k = 0; k < 2U; k++)
    {
        start(k ? 1U : 10U);
        /* Select a FREQUENCY-only loop explicitly; noise no longer disables
         * phase because the per-sample clip keeps it steering. */
        for (unsigned p = 0U; p < 3U; p++)
        {
            CHECK(sc_hybrid_profile_parameters(&s, (sc_hybrid_profile_t)p, p ? 120.0 : 60.0, 0.0));
        }
        for (unsigned i = 0; i < 140U; i++)
        {
            tick();
        }
        CHECK(!s.phase_enabled && !s.ramping && s.state == SC_HYBRID_FREQUENCY);
        physical += 0.06;
        const double error0 = physical + sit5801_fcw_to_ppb(word) - s.desired_drift_ppb;
        for (unsigned i = 0; i < 120U; i++)
        {
            tick();
        }
        const double error1 = physical + sit5801_fcw_to_ppb(word) - s.desired_drift_ppb;
        CHECK(fabs(error1 - error0 * pow(1.0 - 1.0 / (k ? 120.0 : 60.0), 120.0)) < 0.000005);
    }
    /* A gain change during hold, then resume: the restart takes a fresh
     * reference (20 ns logged clock break) and preloads I to the delivered
     * output, so the activation command is identical. */
    start(10U);
    const double saved_reference = s.phase_reference_ns;
    sc_hybrid_hold(&s);
    domain_offset = 20.0;
    for (unsigned i = 0; i < 10U; i++)
    {
        tick();
    }
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 90.0, 900.0));
    const double before_gain_v = s.desired_drift_ppb;
    sc_hybrid_resume(&s);
    for (unsigned i = 0; i < 120U; i++)
    {
        tick();
        CHECK(!s.phase_enabled);
    }
    const int64_t gain_word = word;
    tick();
    CHECK(s.phase_enabled && s.phase_reference_ns != saved_reference && word == gain_word);
    CHECK(s.filtered_error_ns == 0.0 && fabs(s.reanchor_step_ns - 20.0) < 0.5);
    CHECK(fabs(s.phase_integral_ppb - before_gain_v) < 1e-14);
    /* Short and long outages freeze both state and physical trim; old packets
     * cannot switch mode or manufacture qualification on return. */
    const unsigned outages[] = {1U, 3U, 30U, 300U};
    for (unsigned k = 0; k < 4U; k++)
    {
        start(10U);
        const int64_t hold_word = word;
        const double hold_i = s.phase_integral_ppb;
        mode(0U);
        for (unsigned i = 0; i < outages[k]; i++)
        {
            tick();
            CHECK(word == hold_word && s.phase_integral_ppb == hold_i);
        }
        mode(10U);
        tick();
        const sc_hybrid_input_t current = in;
        const unsigned count = s.history.count;
        in.epoch_ms -= 1000U;
        mode(6U);
        sc_hybrid_sample(&s, &in);
        CHECK(s.source_mode == 10U && s.phase_valid && s.history.count == count);
        in = current;
        for (unsigned i = 0; i < 150U; i++)
        {
            tick();
        }
        CHECK(s.phase_enabled);
    }
    /* Rapid valid source changes retain control but never claim relative lock. */
    start(10U);
    const int64_t flapping_word = word;
    for (unsigned i = 0; i < 30U; i++)
    {
        mode(i % 2U ? 10U : 6U);
        domain_offset = i % 2U ? 0.0 : 100.0;
        tick();
        CHECK(word == flapping_word && s.phase_enabled && s.previous_valid && !s.locked);
    }
    puts("Multimode cold starts, continuous handovers, phase recovery and inhibits "
         "passed");
    return 0;
}
