/* SPDX-License-Identifier: MIT */
/* Real controller/FCW conversion, ideal unity plant; no hardware stability claim. */
#include "simpleclock/hybrid.h"
#include "simpleclock/sit5801.h"
#include <math.h>
#include <stdio.h>

#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s state=%s reason=%lu\n", __LINE__, #x,                     \
                    sc_hybrid_state_name(s.state), (unsigned long)s.reason);                       \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static sc_hybrid_t s;
static sc_hybrid_input_t in;
static int64_t word;
static double physical, bias, offset;

static void source(unsigned mode)
{
    in.gnss_valid = mode == 10U;
    in.sbas_valid = mode == 6U;
    in.standalone_valid = mode == 1U;
    in.frequency_valid = mode != 0U;
    in.time_system = mode == 10U ? 1U : 0U;
}

static void tick(void)
{
    in.now_ms += 1000U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    in.drift_ppb = physical + sit5801_fcw_to_ppb(word);
    bias += in.drift_ppb;
    in.bias_ns = bias + offset;
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

static void initialize(sc_hybrid_loop_mode_t mode)
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
    physical = -sit5801_fcw_to_ppb(word);
    bias = -400000.0;
    offset = 0.0;
    in = (sc_hybrid_input_t){
        .now_ms = UINT32_MAX - 200000U,
        .epoch_ms = 1000000U,
    };
    source(10U);
}

static void start(sc_hybrid_loop_mode_t mode)
{
    initialize(mode);
    for (unsigned i = 0U; i < 400U; i++)
    {
        tick();
    }
}

static int test_continuous_handover(void)
{
    start(SC_HYBRID_LOOP_BOTH);
    CHECK(s.locked && s.phase_enabled && !s.ramping);
    s.desired_drift_ppb = s.phase_integral_ppb = 0.012;
    s.accumulator_ppb += 0.0000002;
    const double accumulator = s.accumulator_ppb, acquired = s.acquired_ppb;
    const unsigned accepted = s.history.accepted, generation = s.generation;
    physical += 0.06;
    source(6U);
    offset = 100.0;
    tick();
    CHECK(s.phase_valid && s.phase_enabled && !s.locked && !s.ramping);
    CHECK(s.generation == generation + 1U && s.phase_reference_ns == in.bias_ns);
    CHECK(s.filtered_error_ns == 0.0 && s.phase_integral_ppb == 0.012);
    CHECK(s.desired_drift_ppb == 0.012 && s.acquired_ppb == acquired);
    CHECK(s.history.accepted == accepted + 1U && s.history.count == SC_HYBRID_TRACKING_POINTS);
    CHECK(fabs(s.accumulator_ppb - (accumulator + (0.012 - 0.06) / 120.0)) < 1e-13);
    CHECK(s.phase_good_samples == 1U && s.reason == 0U);
    return 0;
}

static int test_flapping(void)
{
    start(SC_HYBRID_LOOP_BOTH);
    const unsigned accepted = s.history.accepted, generation = s.generation;
    physical += 0.06;
    for (unsigned i = 0U; i < 120U; i++)
    {
        const double before = s.applied_ppb;
        source(i % 2U ? 10U : 6U);
        offset = i % 2U ? -50.0 : 100.0;
        tick();
        CHECK(s.phase_enabled && s.previous_valid && !s.ramping && !s.locked);
        CHECK(s.reason == 0U && fabs(s.applied_ppb - before) <= 0.003004);
    }
    const double expected = 0.06 * pow(1.0 - 1.0 / 60.0, 60.0) * pow(1.0 - 1.0 / 120.0, 60.0);
    CHECK(fabs(physical + s.applied_ppb - expected) < 0.000005);
    CHECK(s.history.accepted == accepted + 120U && s.generation == generation + 120U);
    /* Phase control continued, but lock needs a fresh same-domain window. */
    for (unsigned i = 0U; i < 119U; i++)
    {
        tick();
        CHECK(!s.locked);
    }
    return 0;
}

static int test_modes_and_weights(void)
{
    const sc_hybrid_loop_mode_t loops[] = {SC_HYBRID_LOOP_FREQUENCY, SC_HYBRID_LOOP_PHASE,
                                           SC_HYBRID_LOOP_BOTH};
    const unsigned modes[] = {6U, 1U, 10U};
    for (unsigned k = 0U; k < 3U; k++)
    {
        start(loops[k]);
        CHECK(s.warm);
        for (unsigned j = 0U; j < 3U; j++)
        {
            const double accumulator = s.accumulator_ppb;
            const int64_t held = word;
            source(modes[j]);
            offset += 100.0;
            tick();
            CHECK(sc_hybrid_tracking(&s) && s.previous_valid && !s.ramping);
            CHECK(s.phase_enabled == (loops[k] != SC_HYBRID_LOOP_FREQUENCY));
            CHECK(word == held && s.accumulator_ppb == accumulator);
            if (s.phase_enabled)
            {
                CHECK(s.phase_integral_ppb ==
                      (loops[k] == SC_HYBRID_LOOP_PHASE ? accumulator : 0.0));
            }
            tick();
        }
        /* TimeSystem alone also recaptures local phase without a dwell. */
        const unsigned generation = s.generation;
        in.time_system ^= 1U;
        offset += 100.0;
        tick();
        CHECK(sc_hybrid_tracking(&s) && s.previous_valid && !s.ramping);
        CHECK(s.generation == generation + (loops[k] != SC_HYBRID_LOOP_FREQUENCY ? 1U : 0U));
    }
    /* Fallback Tf gives exactly the configured smaller FLL increment. */
    for (unsigned k = 0U; k < 2U; k++)
    {
        start(SC_HYBRID_LOOP_FREQUENCY);
        CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 100.0, 0.0));
        CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_SBAS, 500.0, 0.0));
        const double accumulator = s.accumulator_ppb;
        physical += 0.05;
        source(k ? 6U : 10U);
        tick();
        CHECK(fabs(s.accumulator_ppb - (accumulator - 0.05 / (k ? 500.0 : 100.0))) < 1e-13);
    }
    /* Direct phase PI must use both matched gains, Kp=2/T and Ki=1/T^2.
     * A real 0.02 ns increment after reanchoring receives the slower profile. */
    for (unsigned k = 0U; k < 2U; k++)
    {
        start(SC_HYBRID_LOOP_PHASE);
        source(k ? 6U : 10U);
        tick();
        const double accumulator = s.accumulator_ppb;
        const double phase_time = k ? 3000.0 : 600.0;
        physical += 0.02;
        tick();
        const double filtered = 0.02; /* Default bias filter is 0 s = bypass. */
        const double expected =
            accumulator - (2.0 / phase_time + 1.0 / (phase_time * phase_time)) * filtered;
        CHECK(fabs(s.accumulator_ppb - expected) < 1e-13);
    }
    return 0;
}

static int test_profile_topology_changes(void)
{
    start(SC_HYBRID_LOOP_PHASE);
    CHECK(s.phase_enabled && s.applied_ppb > 1.0);
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_SBAS, 120, 3000));
    const int64_t direct_word = word;
    source(6U);
    offset = 100;
    tick();
    CHECK(word == direct_word && s.reason == 0U && sc_hybrid_tracking(&s));
    CHECK(s.acquired_ppb == s.applied_ppb);
    /* All sixteen warm loop combinations cross an actual receiver-domain
     * boundary. Retained trim must never be scaled by a new time constant. */
    for (unsigned from = 0U; from < 4U; from++)
    {
        for (unsigned to = 0U; to < 4U; to++)
        {
            start(SC_HYBRID_LOOP_BOTH);
            const double ft = to == SC_HYBRID_LOOP_PHASE || to == SC_HYBRID_LOOP_OFF ? 0.0 : 200.0;
            const double pt =
                to == SC_HYBRID_LOOP_FREQUENCY || to == SC_HYBRID_LOOP_OFF ? 0.0 : 2000.0;
            CHECK(sc_hybrid_profile_parameters(
                &s, SC_HYBRID_PROFILE_PPP,
                from == SC_HYBRID_LOOP_PHASE || from == SC_HYBRID_LOOP_OFF ? 0.0 : 100.0,
                from == SC_HYBRID_LOOP_FREQUENCY || from == SC_HYBRID_LOOP_OFF ? 0.0 : 1000.0));
            CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_SBAS, ft, pt));
            CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_STANDALONE, 0, 0));
            tick();
            const int64_t held = word;
            const unsigned accepted = s.history.accepted;
            source(6U);
            offset = 100.0;
            tick();
            CHECK(s.loop_mode == to && word == held && s.previous_valid && s.warm);
            CHECK(s.history.accepted == accepted + 1U);
            CHECK(sc_hybrid_frequency_time(&s) == ft && sc_hybrid_phase_time(&s) == pt);
            const double accumulator = s.accumulator_ppb;
            physical += 0.05;
            tick();
            if (to == SC_HYBRID_LOOP_OFF)
            {
                CHECK(word == held);
            }
            if (to == SC_HYBRID_LOOP_FREQUENCY || to == SC_HYBRID_LOOP_BOTH)
            {
                CHECK(s.accumulator_ppb < accumulator);
            }
            source(1U);
            offset += 100.0;
            const int64_t before_off = word;
            tick();
            CHECK(word == before_off && s.loop_mode == SC_HYBRID_LOOP_OFF);
            CHECK(sc_hybrid_frequency_time(&s) == 0 && sc_hybrid_phase_time(&s) == 0);
        }
    }
    return 0;
}

static int test_recovery_gates(void)
{
    /* No warm handover may bypass startup, even with frequent source changes. */
    initialize(SC_HYBRID_LOOP_BOTH);
    for (unsigned i = 0U; i < 160U; i++)
    {
        source(i % 2U ? 10U : 6U);
        tick();
        CHECK(!s.warm && !s.phase_enabled && s.writes == 0U);
    }
    /* Changes concurrent with gaps, real discontinuities, invalid data, thermal
     * departure, manual hold or a pending transaction must retain recovery. */
    for (unsigned k = 0U; k < 8U; k++)
    {
        start(SC_HYBRID_LOOP_BOTH);
        const int64_t held = word;
        source(6U);
        offset = 100.0;
        if (k == 0U)
        {
            in.epoch_ms += 1000U;
        }
        else if (k == 1U)
        {
            in.coordinate_changed = in.frequency_discontinuity = true;
        }
        else if (k == 2U)
        {
            /* A plain contiguous domain change hands over without recovery. */
            tick();
            CHECK(sc_hybrid_tracking(&s) && s.phase_enabled && s.phase_valid);
            continue;
        }
        else if (k == 3U)
        {
            sc_hybrid_hold(&s);
        }
        else if (k == 4U)
        {
            /* Invalid data alone is a bridged short gap that holds the trim;
             * past the bridge it is holdover with full recovery. */
            source(0U);
            tick();
            CHECK(word == held && s.phase_enabled && !s.request_ready && sc_hybrid_tracking(&s));
            for (unsigned i = 0U; i < 11U; i++)
            {
                tick();
            }
            CHECK(word == held && !s.phase_enabled && s.state == SC_HYBRID_HOLDOVER);
            continue;
        }
        else if (k == 5U)
        {
            physical += 0.3; /* A mode switch must not hide a drift outlier. */
        }
        else if (k == 6U)
        {
            /* Model a write still waiting for readback when the sample arrives. */
            s.pending = true;
            s.requested_word = word;
            s.pending_ms = in.now_ms;
            in.now_ms += 1000U;
            in.received_ms = in.now_ms;
            in.epoch_ms += 1000U;
            in.bias_ns = bias + offset;
            sc_hybrid_sample(&s, &in);
            CHECK(s.pending && !s.phase_enabled && !s.request_ready && !s.phase_valid);
            continue;
        }
        else
        {
            in.coordinate_changed = true; /* Configuration change without clock reset. */
        }
        tick();
        CHECK(word == held && !s.phase_enabled && !s.request_ready);
        CHECK(s.state == SC_HYBRID_HOLDOVER || s.state == SC_HYBRID_MANUAL_HOLD);
        if (k == 5U)
        {
            CHECK(s.reason == SC_HYBRID_Q_INNOVATION);
        }
    }
    /* A clipped sample leaves no rejection state behind: phase keeps steering
     * and a following contiguous domain change hands over normally. */
    start(SC_HYBRID_LOOP_PHASE);
    offset = 20.0;
    tick();
    CHECK(s.phase_enabled && s.input_clipped);
    source(6U);
    offset = 100.0;
    tick();
    CHECK(s.phase_enabled && s.phase_valid && !s.input_clipped);
    /* Cached observations cannot change the domain or recapture an anchor. */
    start(SC_HYBRID_LOOP_BOTH);
    const unsigned generation = s.generation;
    source(6U);
    sc_hybrid_sample(&s, &in);
    CHECK(s.source_mode == 10U && s.generation == generation && s.phase_enabled);
    return 0;
}

static int test_ramp_and_lock(void)
{
    initialize(SC_HYBRID_LOOP_FREQUENCY);
    for (unsigned i = 0U; i < 400U && !s.warm; i++)
    {
        tick();
    }
    CHECK(s.warm && s.ramping);
    for (unsigned i = 0U; i < 10U; i++)
    {
        tick();
    }
    const uint32_t ramp_ms = s.ramp_ms;
    source(6U);
    tick();
    CHECK(s.ramping && s.ramp_ms == ramp_ms && s.state == SC_HYBRID_FREQUENCY);
    /* Reanchoring permits steering immediately, but not an immediate lock claim. */
    start(SC_HYBRID_LOOP_BOTH);
    source(6U);
    offset = 100.0;
    tick();
    CHECK(s.phase_enabled && !s.locked);
    for (unsigned i = 0U; i < 119U; i++)
    {
        tick();
        CHECK(!s.locked && !s.locking);
    }
    /* A late warm entry can still be ramping when phase qualifies: starting
     * the lock dwell must not restart the 60 s FLL authority ramp. */
    s.ramping = true;
    s.ramp_ms = in.now_ms - 30000U;
    tick();
    CHECK(s.locking && !s.locked && s.ramping);
    for (unsigned i = 0U; i < 9U; i++)
    {
        tick();
        CHECK(!s.locked);
    }
    tick();
    CHECK(s.locked);
    for (unsigned i = 0U; i < 18U; i++)
    {
        tick();
    }
    CHECK(s.ramping);
    tick();
    CHECK(!s.ramping && s.locked); /* 60 s after ramp_ms, not after the lock start. */
    return 0;
}

static int test_drift_only_fallback(void)
{
    /* Selection allows PPP: SBAS and standalone are fallbacks. */
    initialize(SC_HYBRID_LOOP_PHASE);
    in.primary_mode = 10U;
    for (unsigned i = 0U; i < 400U; i++)
    {
        tick();
    }
    CHECK(s.locked && s.phase_enabled && s.phase_valid && s.loop_mode == SC_HYBRID_LOOP_PHASE);
    const double reference = s.phase_reference_ns;
    const unsigned reanchors = s.reanchor_count;
    /* PPP -> SBAS with a 10 ns bias jump: frequency loop only, PPP anchor kept. */
    source(6U);
    offset = 10.0;
    tick();
    CHECK(s.fallback && !s.settling && sc_hybrid_tracking(&s));
    CHECK(s.loop_mode == SC_HYBRID_LOOP_FREQUENCY && !s.phase_enabled);
    CHECK(sc_hybrid_frequency_time(&s) == 10.0 && sc_hybrid_phase_time(&s) == 0.0);
    CHECK(s.phase_valid && s.phase_reference_ns == reference);
    /* SBAS -> standalone with another 100 ns jump: still frequency only, no pull. */
    const double before = s.applied_ppb;
    source(1U);
    offset = 110.0;
    for (unsigned i = 0U; i < 30U; i++)
    {
        tick();
        CHECK(s.fallback && !s.phase_enabled && fabs(s.applied_ppb - before) < 0.0001);
    }
    /* A drift error is still corrected, at the drift-only Tf of 10 s (under the slew cap). */
    const double accumulator = s.accumulator_ppb;
    physical += 0.02;
    tick();
    CHECK(fabs(s.accumulator_ppb - (accumulator - (in.drift_ppb - 0.0) / 10.0)) < 1e-9);
    /* Back to PPP, starting 40 ns off and converging: frequency only through
     * the 1000 s settle; then phase restarts from a fresh, logged anchor. */
    source(10U);
    unsigned t = 0U;
    offset = 2.0 + 38.0;
    tick();
    CHECK(!s.fallback && s.settling && s.loop_mode == SC_HYBRID_LOOP_FREQUENCY);
    for (t = 1U; t < 999U; t++)
    {
        offset = 2.0 + 38.0 * exp(-(double)t / 150.0);
        tick();
        CHECK(s.settling && !s.phase_enabled && sc_hybrid_tracking(&s));
    }
    for (; t < 1200U && !s.phase_enabled; t++)
    {
        offset = 2.0 + 38.0 * exp(-(double)t / 150.0);
        tick();
    }
    CHECK(s.phase_enabled && !s.settling && s.loop_mode == SC_HYBRID_LOOP_PHASE);
    CHECK(s.reanchor_count == reanchors + 1U && s.filtered_error_ns == 0.0);
    CHECK(fabs(s.reanchor_step_ns - (s.phase_reference_ns - reference)) < 1e-9);
    /* No pull toward the old reference afterwards. */
    const double anchored = s.applied_ppb;
    for (unsigned i = 0U; i < 60U; i++)
    {
        tick();
    }
    CHECK(fabs(s.applied_ppb - anchored) < 0.001 && fabs(s.filtered_error_ns) < 1.0);

    /* SBAS selection: standalone is the fallback; returning SBAS needs no
     * settle, only the normal phase qualification. */
    initialize(SC_HYBRID_LOOP_PHASE);
    in.primary_mode = 6U;
    source(6U);
    for (unsigned i = 0U; i < 400U; i++)
    {
        tick();
    }
    CHECK(s.locked && s.phase_enabled && !s.fallback);
    const unsigned sbas_reanchors = s.reanchor_count;
    const double sbas_reference = s.phase_reference_ns;
    source(1U);
    offset = 8.0;
    tick();
    CHECK(s.fallback && !s.phase_enabled && s.loop_mode == SC_HYBRID_LOOP_FREQUENCY);
    source(6U);
    offset = 3.0;
    tick();
    CHECK(!s.fallback && !s.settling && s.loop_mode == SC_HYBRID_LOOP_PHASE && !s.phase_enabled);
    unsigned waited = 0U;
    while (!s.phase_enabled && waited < 200U)
    {
        tick();
        waited++;
    }
    CHECK(s.phase_enabled && waited >= 100U && s.reanchor_count == sbas_reanchors + 1U);
    CHECK(fabs(s.reanchor_step_ns - (s.phase_reference_ns - sbas_reference)) < 1e-9);
    /* A new selection: an anchor from the old primary is not comparable. */
    in.primary_mode = 10U;
    tick();
    CHECK(s.fallback && !s.phase_valid && !s.phase_enabled);

    /* A 0/0 pair never steers on its solution type, in a fallback too: the
     * trim is held on SBAS, and steering returns on the next enabled type. */
    initialize(SC_HYBRID_LOOP_PHASE);
    in.primary_mode = 10U;
    for (unsigned i = 0U; i < 400U; i++)
    {
        tick();
    }
    CHECK(s.locked && sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_SBAS, 0.0, 0.0));
    source(6U);
    offset = 10.0;
    tick();
    CHECK(s.fallback && s.loop_mode == SC_HYBRID_LOOP_OFF && s.state == SC_HYBRID_DISABLED);
    const int64_t held = word;
    physical += 0.02;
    for (unsigned i = 0U; i < 30U; i++)
    {
        tick();
        CHECK(word == held && s.state == SC_HYBRID_DISABLED);
    }
    source(1U);
    offset = 20.0;
    tick();
    CHECK(s.fallback && s.loop_mode == SC_HYBRID_LOOP_FREQUENCY && sc_hybrid_tracking(&s));
    for (unsigned i = 0U; i < 30U; i++)
    {
        tick();
    }
    CHECK(word != held && !s.phase_enabled);
    /* The same for the selected type while PPP settles. */
    CHECK(sc_hybrid_profile_parameters(&s, SC_HYBRID_PROFILE_PPP, 0.0, 0.0));
    source(10U);
    offset = 0.0;
    tick();
    CHECK(s.settling && s.loop_mode == SC_HYBRID_LOOP_OFF && s.state == SC_HYBRID_DISABLED);
    return 0;
}

static int test_phase_span_recentres(void)
{
    /* Phase-only tracking is bounded to +/-10 ppb about the trim verified at
     * the last acquisition, not by the wide boot-relative envelope. A slow
     * frequency ramp past that span freezes once, then re-acquires and
     * re-centres the span instead of latching a fault or stopping. */
    start(SC_HYBRID_LOOP_PHASE);
    CHECK(s.locked && s.upper_ppb - s.lower_ppb > 1000.0);
    const double first = s.acquired_ppb;
    unsigned limit_freezes = 0U;
    for (unsigned i = 0U; i < 9000U; i++)
    {
        physical += 0.002; /* 18 ppb in total, below the 0.003 ppb/s slew cap. */
        tick();
        CHECK(fabs(s.applied_ppb - s.acquired_ppb) <= 10.0 + 1e-6 || !sc_hybrid_tracking(&s));
        limit_freezes += (s.reason & SC_HYBRID_Q_LIMIT) && s.state == SC_HYBRID_HOLDOVER ? 1U : 0U;
        CHECK(s.state != SC_HYBRID_ACQUISITION_FAULT);
    }
    for (unsigned i = 0U; i < 4000U; i++) /* Tphi is 600 s in this fixture. */
    {
        tick();
    }
    CHECK(limit_freezes >= 1U && limit_freezes <= 3U);
    CHECK(sc_hybrid_tracking(&s) && s.phase_enabled && fabs(s.acquired_ppb - first) > 9.0);
    CHECK(fabs(physical + s.applied_ppb) < 0.05);
    return 0;
}

int main(void)
{
    CHECK(test_phase_span_recentres() == 0);
    CHECK(test_drift_only_fallback() == 0);
    CHECK(test_continuous_handover() == 0);
    CHECK(test_flapping() == 0);
    CHECK(test_modes_and_weights() == 0);
    CHECK(test_profile_topology_changes() == 0);
    CHECK(test_recovery_gates() == 0);
    CHECK(test_ramp_and_lock() == 0);
    puts("Continuous weighted handovers, phase anchoring and retained recovery gates passed");
    return 0;
}
