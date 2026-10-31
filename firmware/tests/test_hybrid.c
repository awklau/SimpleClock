/* SPDX-License-Identifier: MIT */
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
            fprintf(stderr, "line %d: %s state=%s\n", __LINE__, #x,                                \
                    sc_hybrid_state_name(s.state));                                                \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
static sc_hybrid_t s;
static sc_hybrid_input_t in;
static int64_t word;
static double physical, bias;

/* Fixture parameters, not compiled defaults: these scenarios exercise BOTH-mode
 * FLL acquisition with PPP 60/600 s and SBAS/standalone 120/3000 s. */
static void legacy_profiles(sc_hybrid_t *h)
{
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_PPP, 60.0, 600.0);
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_SBAS, 120.0, 3000.0);
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_STANDALONE, 120.0, 3000.0);
}

static void start(double initial_pull)
{
    sc_hybrid_init(&s);
    legacy_profiles(&s);
    (void)sit5801_ppb_to_fcw(initial_pull, &word);
    sc_hybrid_observed(&s, true, word, 32, 300000U);
    in = (sc_hybrid_input_t){0};
    in.now_ms = in.received_ms = 300000U;
    in.epoch_ms = UINT64_C(2436) * UINT64_C(604800000) + UINT64_C(604700000);
    in.gnss_valid = true;
    physical = 2.75;
    bias = -400000.0;
}

static void sample(bool actuate)
{
    in.now_ms += 1000U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    in.drift_ppb = physical + sit5801_fcw_to_ppb(word);
    bias += in.drift_ppb;
    in.bias_ns = bias;
    sc_hybrid_observed(&s, true, word, 32, in.now_ms);
    sc_hybrid_service(&s, in.now_ms);
    sc_hybrid_sample(&s, &in);
    int64_t proposed;
    if (actuate && sc_hybrid_take_command(&s, in.now_ms, &proposed))
    {
        word = proposed;
        sc_hybrid_written(&s, true, in.now_ms);
        sc_hybrid_observed(&s, true, word, 32, in.now_ms + 400U);
    }
}

static int test_shortened_dwell_boundaries(void)
{
    start(0);
    physical = 0.0; /* No ramp delay: observe just the requested timing gates. */
    in.now_ms = in.received_ms = UINT32_MAX - 40000U;
    for (unsigned i = 0; i < 61U; ++i)
    {
        sample(true);
    }
    CHECK(s.state == SC_HYBRID_QUALIFY && s.attempts == 0U && s.writes == 0U);
    CHECK(s.history.count == 61U); /* Exactly 60 s is still insufficient. */
    sample(true);
    CHECK(s.state == SC_HYBRID_VERIFY && s.verify_samples == 0U && s.history.count == 0U);
    for (unsigned i = 0; i < 49U; ++i)
    {
        sample(true);
    }
    CHECK(s.state == SC_HYBRID_VERIFY && !s.phase_valid && s.verify_samples == 49U);
    sc_hybrid_sample(&s, &in); /* A duplicate cannot become the 50th fresh sample. */
    CHECK(s.state == SC_HYBRID_VERIFY && s.verify_samples == 49U);
    sample(true);
    CHECK(s.state == SC_HYBRID_FREQUENCY && !s.phase_enabled && !s.phase_valid);
    CHECK(s.history.count == 50U && !s.locking && !s.locked);

    /* Relative-phase readiness still needs the control windows. The frequency
     * observer already has measurements collected during qualification. */
    for (unsigned i = 0; i < 70U; ++i)
    {
        sample(true);
        CHECK(!s.locking && !s.locked);
    }
    sample(true);
    CHECK(s.phase_enabled && !s.locking && !s.locked && s.history.count == 121U);
    sample(true);
    CHECK(s.locking && !s.locked);
    CHECK(s.frequency.timing && s.frequency.settled && s.frequency.count == 121U);
    for (unsigned i = 0; i < 9U; ++i)
    {
        sample(true);
        CHECK(!s.locked);
        CHECK(s.frequency.settled);
    }
    sc_hybrid_sample(&s, &in);
    CHECK(!s.locked); /* Duplicate does not advance the nine elapsed seconds. */
    sample(true);
    CHECK(s.locked && s.state == SC_HYBRID_TRACK);
    CHECK(s.frequency.settled);

    sample(true);
    CHECK(s.locked && s.phase_valid && s.state == SC_HYBRID_TRACK);
    CHECK(s.generation == 1U && s.frequency.settled && s.frequency.timing);
    return 0;
}

int main(void)
{
    CHECK(test_shortened_dwell_boundaries() == 0);
    start(0);
    CHECK(s.enabled && s.known && !s.phase_valid);
    for (unsigned i = 0; i < 60U; i++)
    {
        sample(false);
    }
    CHECK(s.state == SC_HYBRID_QUALIFY && s.history.count == 60U);
    sample(false); /* Elapsed exactly 60 seconds from the first eligible epoch. */
    CHECK(s.state == SC_HYBRID_QUALIFY && s.attempts == 0);
    sample(true);
    CHECK(s.state == SC_HYBRID_ACQUIRE && s.attempts == 1 && word < 0);
    CHECK(fabs(s.applied_ppb) <= 0.050004);
    for (unsigned i = 0; i < 250U && !s.phase_enabled; i++)
    {
        sample(true);
    }
    CHECK(sc_hybrid_tracking(&s) && s.generation == 1 && s.phase_valid);
    CHECK(fabs(physical + s.applied_ppb) < 0.00001);
    CHECK(fabs(s.phase_reference_ns) > 390000.0 && fabs(s.filtered_error_ns) < 0.00001);
    const double reference = s.phase_reference_ns;
    for (unsigned i = 0; i < 230U; i++)
    {
        sample(true);
    }
    CHECK(s.locked && s.state == SC_HYBRID_TRACK);

    /* Small positive frequency disturbance: the 60 s FLL reduces the error,
     * with the slower phase PI active. No artificial 100 s TRACK averaging. */
    physical += 0.02;
    const double before = s.applied_ppb;
    for (unsigned i = 0; i < 60U; i++)
    {
        sample(true);
    }
    CHECK(s.applied_ppb < before && physical + s.applied_ppb > 0.003 &&
          physical + s.applied_ppb < 0.009);
    CHECK(s.phase_reference_ns == reference && s.desired_drift_ppb < 0.0);
    for (unsigned i = 0; i < 2200U; i++)
    {
        sample(true);
    }
    CHECK(sc_hybrid_tracking(&s) && fabs(s.filtered_error_ns) < 0.15);
    CHECK(s.phase_reference_ns == reference);

    in.gnss_valid = false;
    const int64_t held_word = word;
    /* One invalid epoch is a bridged gap: trim held, authority and anchor kept. */
    sample(true);
    CHECK(sc_hybrid_tracking(&s) && s.phase_enabled && word == held_word &&
          s.phase_reference_ns == reference && !s.gnss_good);
    /* Beyond the short-gap bridge it is an outage: holdover, re-anchor on return. */
    for (unsigned i = 0; i < 11U; i++)
    {
        sample(true);
    }
    CHECK(s.state == SC_HYBRID_HOLDOVER && word == held_word && s.phase_valid);
    in.gnss_valid = true;
    for (unsigned i = 0; i < 250U; i++)
    {
        sample(true);
    }
    /* Recovery re-anchors: the holdover phase error is a logged clock break. */
    CHECK(sc_hybrid_tracking(&s) && s.generation == 2 && s.reanchor_count == 1U);
    CHECK(fabs(s.reanchor_step_ns - (s.phase_reference_ns - reference)) < 1e-9);

    in.coordinate_changed = true;
    sample(true);
    in.coordinate_changed = false;
    CHECK(!s.phase_valid);
    for (unsigned i = 0; i < 250U; i++)
    {
        sample(true);
    }
    CHECK(s.phase_valid && s.generation == 3 && s.reanchor_count == 1U); /* Not a restart. */
    sc_hybrid_hold(&s);
    for (unsigned i = 0; i < 200U; i++)
    {
        sample(true);
    }
    CHECK(!s.enabled && s.state == SC_HYBRID_MANUAL_HOLD);
    CHECK(!sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, NAN, 600) &&
          !sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 1, 600));
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 60, 600));
    sc_hybrid_resume(&s);
    sample(true);
    CHECK(s.state == SC_HYBRID_QUALIFY);

    /* A failed multiword transaction cannot claim known-frequency holdover. */
    s.pending = true;
    sc_hybrid_written(&s, false, in.now_ms);
    CHECK(!s.known && s.state == SC_HYBRID_ACTUATOR_UNKNOWN);
    sc_hybrid_observed(&s, true, word, 32, in.now_ms + 1000U);
    CHECK(!s.known);
    sc_hybrid_observed(&s, true, word, 32, in.now_ms + 2000U);
    CHECK(s.known && s.state == SC_HYBRID_HOLDOVER);
    sc_hybrid_service(&s, in.now_ms + 4501U);
    CHECK(!s.known);

    start(1.25); /* MCU-only restart preserves a nonzero boot FCW. */
    CHECK(fabs(s.boot_ppb - 1.25) < 0.000002);
    /* Clock qualification starts at once; it has no thermal input. */
    for (unsigned i = 0; i < 50U; i++)
    {
        sample(false);
    }
    CHECK(s.state == SC_HYBRID_QUALIFY && s.qualifying && s.writes == 0);
    const uint32_t count = s.history.accepted;
    sc_hybrid_sample(&s, &in);
    CHECK(s.history.accepted == count);
    in.epoch_ms += 1000U; /* Missing receiver epoch breaks dwell. */
    sample(false);
    CHECK(!s.qualifying && s.state == SC_HYBRID_HOLDOVER);
    for (unsigned i = 0; i < 61U; i++)
    {
        sample(false);
    }
    CHECK(s.state == SC_HYBRID_QUALIFY);
    sample(false);
    CHECK(s.state == SC_HYBRID_ACQUIRE);

    start(0);
    for (unsigned i = 0; i < 130U; i++)
    {
        /* A queued stream spans receiver time but only130ms of MCU time. */
        in.now_ms++;
        in.received_ms = in.now_ms;
        in.epoch_ms += 1000U;
        in.drift_ppb = 0;
        in.bias_ns = 1000;
        sc_hybrid_observed(&s, true, word, 32, in.now_ms);
        sc_hybrid_sample(&s, &in);
    }
    CHECK(s.state == SC_HYBRID_QUALIFY && s.writes == 0);
    puts("Hybrid boot/acquisition/relative tracking/holdover/fault checks passed");
    start(0);
    for (unsigned i = 0; i < 100U; ++i)
    {
        sample(false);
    }
    for (unsigned i = 0; i < 140U; ++i)
    {
        sample(true);
    }
    CHECK(s.writes >= 1U && s.warm);

    return 0;
}
