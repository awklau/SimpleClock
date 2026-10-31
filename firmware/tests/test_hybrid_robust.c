/* SPDX-License-Identifier: MIT */
/* Real controller/FCW, ideal unity plant with deterministic measurement noise.
 * Regression bounds for anti-windup and the per-sample bias clip; these are
 * not hardware ADEV/TDEV results. */
#include "simpleclock/hybrid.h"
#include "simpleclock/sit5801.h"
#include <math.h>
#include <stdbool.h>
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
/* clock_ns is the true clock phase against GNSS time; the receiver reports it
 * plus bias noise/glitches, and the true frequency plus drift noise. */
static double physical, clock_ns, bias_noise_sd, drift_noise_sd, bias_offset;
static uint64_t rng_state;

static double gaussian(void)
{
    /* Deterministic xorshift64* + Box-Muller: identical on every compiler. */
    double u[2];
    for (unsigned i = 0U; i < 2U; i++)
    {
        rng_state ^= rng_state >> 12U;
        rng_state ^= rng_state << 25U;
        rng_state ^= rng_state >> 27U;
        u[i] = ((double)((rng_state * UINT64_C(2685821657736338717)) >> 11U) + 0.5) /
               9007199254740992.0;
    }
    return sqrt(-2.0 * log(u[0])) * cos(6.283185307179586 * u[1]);
}

/* A negative phase time keeps the compiled default profiles. */
static void initialize(double phase_s, double frequency_s)
{
    sc_hybrid_init(&s);
    for (unsigned profile = 0U; profile < 3U && phase_s >= 0.0; profile++)
    {
        (void)sc_hybrid_profile_parameters(&s, (sc_hybrid_profile_t)profile, frequency_s, phase_s);
    }
    (void)sit5801_ppb_to_fcw(1.25, &word);
    in = (sc_hybrid_input_t){
        .now_ms = UINT32_MAX - 90000U, .epoch_ms = 1000000U, .gnss_valid = true, .time_system = 1U};
    physical = -sit5801_fcw_to_ppb(word);
    clock_ns = 300000.0;
    bias_noise_sd = drift_noise_sd = bias_offset = 0.0;
    rng_state = UINT64_C(0x9e3779b97f4a7c15);
    sc_hybrid_observed(&s, true, word, 32U, in.now_ms);
}

static void tick(void)
{
    in.now_ms += 1000U;
    in.received_ms = in.now_ms;
    in.epoch_ms += 1000U;
    const double actual = physical + sit5801_fcw_to_ppb(word);
    clock_ns += actual;
    in.bias_ns = clock_ns + bias_offset + bias_noise_sd * gaussian();
    in.drift_ppb = actual + drift_noise_sd * gaussian();
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

static int settle(void)
{
    for (unsigned i = 0U; i < 3000U; i++)
    {
        tick();
    }
    CHECK(s.phase_enabled && s.phase_valid);
    return 0;
}

/* True clock phase error against the loop's local reference. */
static double true_error(void)
{
    return clock_ns - s.phase_reference_ns;
}

static int test_retained_error_recovery(void)
{
    /* A gain change preloads I with a real retained error. In PHASE at
     * Tphi=10 s the first I step (e/T^2) exceeds the 0.003 ppb/s slew cap;
     * holding I and recomputing the output from it repeated the preloaded
     * output forever. BOTH uses the SBAS/standalone operating point. */
    const double frequency_times[] = {0.0, 10.0}, phase_times[] = {10.0, 100.0};
    for (unsigned mode = 0U; mode < 2U; mode++)
    {
        for (int sign = -1; sign <= 1; sign += 2)
        {
            initialize(600.0, frequency_times[mode]);
            CHECK(settle() == 0);
            bias_offset = 15.0 * sign; /* Passes the 2 ns clip within 8 s. */
            for (unsigned i = 0U; i < 20U; i++)
            {
                tick();
            }
            const double retained = s.phase_error_ns;
            CHECK(fabs(retained) > 10.0 && retained * sign > 0.0);
            for (unsigned p = 0U; p < 3U; p++)
            {
                CHECK(sc_hybrid_profile_parameters(&s, (sc_hybrid_profile_t)p,
                                                   frequency_times[mode], phase_times[mode]));
            }
            unsigned moved = 0U;
            for (unsigned i = 0U; i < 60U; i++)
            {
                const double before = s.applied_ppb;
                tick();
                CHECK(fabs(s.applied_ppb - before) <= 0.003004);
                moved += s.applied_ppb != before ? 1U : 0U;
            }
            CHECK(moved >= 50U); /* No stall at the preloaded output. */
            for (unsigned i = 0U; i < 6000U; i++)
            {
                tick();
            }
            CHECK(fabs(s.phase_error_ns) < 0.5 && s.phase_enabled);
            CHECK(fabs(physical + s.applied_ppb) < 0.0005);
        }
    }
    return 0;
}

static double rms_true_error(unsigned samples)
{
    double sum = 0.0;
    for (unsigned i = 0U; i < samples; i++)
    {
        tick();
        sum += true_error() * true_error();
    }
    return sqrt(sum / (double)samples);
}

static int test_noise_without_limit_cycle(void)
{
    /* PPP/HAS operating point: PHASE, Tphi=10 s. With 0.2 ns white bias noise
     * the slew cap is active on most samples. Projection anti-windup turned
     * that into a ~100 s, several-ns limit cycle in the analysis model. */
    initialize(10.0, 0.0);
    CHECK(settle() == 0);
    const double reference = s.phase_reference_ns;
    bias_noise_sd = 0.2;
    for (unsigned i = 0U; i < 500U; i++)
    {
        tick();
    }
    const double rms = rms_true_error(4000U);
    CHECK(s.phase_enabled && s.phase_reference_ns == reference);
    printf("PHASE Tphi=10 s, 0.2 ns white bias noise: true phase RMS %.3f ns\n", rms);
    CHECK(rms < 0.25); /* Projection anti-windup: 0.51 ns in this fixture. */
    return 0;
}

static int test_both_drift_noise(void)
{
    /* SBAS/standalone operating point: BOTH, Tf=10 s, Tphi=100 s, with
     * per-sample bias and drift measurement noise. Back-calculating I from
     * the measured drift injected that noise into I under the slew cap. */
    initialize(100.0, 10.0);
    CHECK(settle() == 0);
    bias_noise_sd = 0.15;
    drift_noise_sd = 0.02;
    for (unsigned i = 0U; i < 500U; i++)
    {
        tick();
    }
    const double rms = rms_true_error(6000U);
    CHECK(s.phase_enabled);
    printf("BOTH Tf=10 s Tphi=100 s, 0.15 ns + 0.02 ppb noise: true phase RMS %.3f ns\n", rms);
    CHECK(rms < 0.4); /* Projection anti-windup: 1.97 ns in this fixture. */
    return 0;
}

static int cold_start(unsigned source, double fll_s, double phase_s)
{
    /* No profile edits: compiled defaults from a 2 ppb cold offset. Every
     * source acquires coarsely first, then locks on its compiled pair. */
    initialize(-1.0, 0.0);
    in.gnss_valid = source == 10U;
    in.sbas_valid = source == 6U;
    in.standalone_valid = source == 1U || source == 3U;
    in.static_fix = source == 3U; /* Static PVT fixed-position solution. */
    in.time_system = source == 10U ? 1U : 0U;
    physical += 2.0;
    unsigned elapsed = 0U;
    while (elapsed++ < 6000U && !s.locked)
    {
        tick();
    }
    CHECK(s.locked && s.attempts >= 1U);
    CHECK(s.loop_mode == (fll_s > 0.0 ? SC_HYBRID_LOOP_BOTH : SC_HYBRID_LOOP_PHASE));
    CHECK(sc_hybrid_frequency_time(&s) == fll_s && sc_hybrid_phase_time(&s) == phase_s);
    CHECK(fabs(physical + s.applied_ppb) < 0.001);
    printf("compiled source %u default locked after %u s from a 2 ppb offset\n", source, elapsed);
    return 0;
}

static unsigned run_until_locked(unsigned limit)
{
    unsigned elapsed = 0U;
    while (elapsed++ < limit && !s.locked)
    {
        tick();
    }
    return elapsed;
}

static int test_phase_start_acquisition(void)
{
    /* PHASE-only Tphi 10 s: every start acquires frequency coarsely first, so
     * the direct phase PI never has to pull in a multi-ppb offset itself. */
    const double offsets[] = {2.0, -2.0, 4.0, -4.0};
    for (unsigned k = 0U; k < 4U; k++)
    {
        initialize(10.0, 0.0);
        physical += offsets[k];
        const unsigned elapsed = run_until_locked(6000U);
        CHECK(s.locked && s.loop_mode == SC_HYBRID_LOOP_PHASE && s.attempts >= 1U);
        CHECK(fabs(physical + s.applied_ppb) < 0.001 && fabs(s.phase_error_ns) < 5.0);
        printf("PHASE cold start %+.0f ppb: locked after %u s, %u acquisition(s)\n", offsets[k],
               elapsed, (unsigned)s.attempts);
    }
    /* Software restart: the frequency moves 1 ppb during a 300 s hold. The
     * restart re-acquires frequency and takes a fresh phase reference: the
     * holdover phase error is a logged clock break, never steered back. */
    sc_hybrid_hold(&s);
    physical += 1.0;
    for (unsigned i = 0U; i < 300U; i++)
    {
        tick();
    }
    const uint32_t reanchors = s.reanchor_count;
    sc_hybrid_resume(&s);
    unsigned elapsed = 0U, limit_freezes = 0U;
    double peak_ppb = 0.0;
    while (elapsed++ < 6000U && !s.locked)
    {
        const bool phase_before = s.phase_enabled;
        tick();
        limit_freezes += (s.reason & SC_HYBRID_Q_LIMIT) && !s.phase_enabled && phase_before;
        if (s.phase_enabled)
        {
            peak_ppb = fmax(peak_ppb, fabs(physical + s.applied_ppb));
        }
    }
    CHECK(s.locked && s.attempts >= 1U && limit_freezes == 0U);
    CHECK(s.reanchor_count == reanchors + 1U && fabs(s.reanchor_step_ns) > 100.0);
    CHECK(peak_ppb <= 0.01); /* Re-anchored and re-acquired: no phase pull. */
    printf("PHASE resume after 1 ppb move in hold: re-anchor step %.0f ns (total %.0f ns), "
           "locked after %u s, peak frequency error after phase entry %.4f ppb\n",
           s.reanchor_step_ns, s.reanchor_total_ns, elapsed, peak_ppb);
    for (unsigned i = 0U; i < 600U; i++)
    {
        tick();
    }
    CHECK(s.locked && fabs(physical + s.applied_ppb) < 0.001 && fabs(s.phase_error_ns) < 1.0);
    /* Recovery after a 300 s GNSS outage with a 0.5 ppb frequency move. */
    in.gnss_valid = false;
    physical -= 0.5;
    for (unsigned i = 0U; i < 300U; i++)
    {
        tick();
    }
    in.gnss_valid = true;
    elapsed = run_until_locked(6000U);
    CHECK(s.locked);
    printf("PHASE recovery after 0.5 ppb move in outage: locked after %u s\n", elapsed);
    for (unsigned i = 0U; i < 600U; i++)
    {
        tick();
    }
    CHECK(s.locked && fabs(physical + s.applied_ppb) < 0.001 && fabs(s.phase_error_ns) < 1.0);
    return 0;
}

static int test_restart_acquisition_all_modes(void)
{
    /* FREQUENCY and BOTH also re-acquire on a software restart after the
     * frequency moved 1 ppb during a 300 s hold, instead of leaving the step
     * to the fine loop. */
    const double phase_times[] = {0.0, 100.0};
    for (unsigned k = 0U; k < 2U; k++)
    {
        initialize(phase_times[k], 10.0);
        for (unsigned i = 0U; i < 2000U && !sc_hybrid_tracking(&s); i++)
        {
            tick();
        }
        CHECK(sc_hybrid_tracking(&s));
        sc_hybrid_hold(&s);
        physical += 1.0;
        for (unsigned i = 0U; i < 300U; i++)
        {
            tick();
        }
        sc_hybrid_resume(&s);
        for (unsigned i = 0U; i < 2000U && !sc_hybrid_tracking(&s); i++)
        {
            tick();
        }
        CHECK(sc_hybrid_tracking(&s) && s.attempts >= 1U);
        CHECK(fabs(physical + s.applied_ppb) <= 0.05);
    }
    return 0;
}

static int test_compiled_defaults(void)
{
    CHECK(cold_start(10U, 0.0, 10.0) == 0);  /* PPP/HAS: PHASE Tphi 10 s */
    CHECK(cold_start(6U, 10.0, 100.0) == 0); /* SBAS */
    CHECK(cold_start(1U, 10.0, 100.0) == 0); /* Standalone */
    CHECK(cold_start(3U, 10.0, 100.0) == 0); /* Static PVT fixed position */
    return 0;
}

static int test_isolated_glitch(void)
{
    initialize(10.0, 0.0);
    CHECK(settle() == 0);
    const double start = true_error();
    const uint32_t clipped = s.clipped_samples;
    bias_offset = 50.0;
    tick();
    CHECK(s.input_clipped && s.clipped_samples == clipped + 1U);
    CHECK(fabs(s.phase_innovation_ns - 50.0) < 0.01 && s.last_failure.bias_ns == in.bias_ns);
    bias_offset = 0.0;
    double worst = 0.0;
    for (unsigned i = 0U; i < 300U; i++)
    {
        tick();
        worst = fmax(worst, fabs(true_error() - start));
    }
    /* The loop saw 2 ns for one sample, then -2 ns back: the slew cap bounds
     * the clock excursion far below the raw 50 ns spike. */
    CHECK(worst < 0.1 && s.phase_enabled);
    return 0;
}

/* Epochs the controller never sees (overwritten while an FCW write was
 * pending, assembler drop): the clock keeps running, the actuator is probed. */
static void miss(unsigned epochs)
{
    for (unsigned i = 0U; i < epochs; i++)
    {
        in.now_ms += 1000U;
        in.epoch_ms += 1000U;
        clock_ns += physical + sit5801_fcw_to_ppb(word);
        sc_hybrid_observed(&s, true, word, 32U, in.now_ms);
        sc_hybrid_service(&s, in.now_ms);
    }
}

static int test_short_gap_bridge(void)
{
    /* PPP direct PI and the standalone/SBAS cascade. */
    const double frequency_times[] = {0.0, 10.0}, phase_times[] = {10.0, 100.0};
    for (unsigned mode = 0U; mode < 2U; mode++)
    {
        initialize(phase_times[mode], frequency_times[mode]);
        bias_noise_sd = 0.1;
        drift_noise_sd = 0.001;
        CHECK(settle() == 0);
        (void)run_until_locked(3000U);
        CHECK(s.phase_enabled && s.locked);
        const uint32_t reanchors = s.reanchor_count, generation = s.generation;
        const double reference = s.phase_reference_ns;
        /* Probe cycle slower than GNSS time: one epoch lost every 50 s. */
        bool stayed = true;
        for (unsigned k = 0U; k < 24U; k++)
        {
            miss(1U);
            for (unsigned i = 0U; i < 49U; i++)
            {
                tick();
                stayed = stayed && sc_hybrid_tracking(&s) && s.phase_enabled;
            }
        }
        CHECK(stayed && s.locked && s.bridged_gaps == 24U && s.bridged_epochs == 24U);
        CHECK(s.reanchor_count == reanchors && s.generation == generation &&
              s.phase_reference_ns == reference && fabs(true_error()) < 1.0);
        /* A short run of invalid fixes holds the trim, then steering continues. */
        const int64_t held = word;
        in.gnss_valid = false;
        for (unsigned i = 0U; i < 3U; i++)
        {
            tick();
            CHECK(word == held && sc_hybrid_tracking(&s) && s.phase_enabled && !s.gnss_good);
        }
        in.gnss_valid = true;
        tick();
        CHECK(s.gnss_good && s.phase_enabled && !s.input_clipped && s.bridged_epochs == 27U);
        /* Bridge limit: a 10 s receiver gap is bridged, an 11 s gap is an outage. */
        const int64_t before_gap = word;
        miss(9U);
        CHECK(word == before_gap && sc_hybrid_tracking(&s)); /* No write without data. */
        tick();
        CHECK(sc_hybrid_tracking(&s) && s.phase_enabled && s.bridged_gaps == 26U);
        CHECK(s.reanchor_count == reanchors && s.generation == generation);
        miss(10U);
        tick();
        CHECK(s.state == SC_HYBRID_HOLDOVER && s.reason == SC_HYBRID_Q_EPOCH);
        const unsigned elapsed = run_until_locked(3000U);
        CHECK(s.locked && s.reanchor_count == reanchors + 1U && s.generation == generation + 1U);
        printf("short-gap bridge mode %u: 24 lost epochs and 3 invalid fixes bridged without "
               "re-anchor; 11 s gap re-locked in %u s\n",
               mode, elapsed);
    }
    return 0;
}

static int test_lock_label(void)
{
    /* PPP direct PI at Tphi 10 s: 0.1 ns noise keeps the 0.003 ppb/s slew cap
     * active on most samples. That alone must not deny the lock label... */
    initialize(10.0, 0.0);
    bias_noise_sd = 0.1;
    drift_noise_sd = 0.001;
    CHECK(settle() == 0);
    unsigned slewed = 0U, locked = 0U;
    for (unsigned i = 0U; i < 600U; i++)
    {
        tick();
        slewed += s.slew_limited;
        locked += s.locked;
    }
    CHECK(slewed > 300U && locked == 600U && fabs(true_error()) < 0.5);
    /* ...but a real disturbance still drops it. An instantaneous 1 ppb step
     * fails the 0.25 ppb drift-innovation check twice, freezes, re-acquires on
     * the warm restart and re-anchors once; the label returns after relock. */
    physical += 1.0;
    const uint32_t step_reanchors = s.reanchor_count;
    unsigned lost_after = 0U;
    while (lost_after++ < 60U && s.locked)
    {
        tick();
    }
    CHECK(!s.locked);
    const unsigned relocked = run_until_locked(3000U);
    const double residual = physical + s.applied_ppb; /* Within the 0.02 ppb slope gate. */
    CHECK(s.locked && fabs(residual) < 0.02);
    for (unsigned i = 0U; i < 600U; i++)
    {
        tick();
    }
    CHECK(s.locked && fabs(physical + s.applied_ppb) < 0.001);
    CHECK(s.reanchor_count == step_reanchors + 1U);
    printf("PPP lock label: held with slew cap active on %u/600 samples; 1 ppb step lost it "
           "after %u s, regained after %u s (residual %.4f ppb)\n",
           slewed, lost_after, relocked, residual);
    return 0;
}

int main(void)
{
    sc_hybrid_init(&s);
    CHECK(s.bias_clip_ps == 2000U); /* 2 ns clip. */
    CHECK(test_retained_error_recovery() == 0);
    CHECK(test_noise_without_limit_cycle() == 0);
    CHECK(test_both_drift_noise() == 0);
    CHECK(test_isolated_glitch() == 0);
    CHECK(test_compiled_defaults() == 0);
    CHECK(test_phase_start_acquisition() == 0);
    CHECK(test_restart_acquisition_all_modes() == 0);
    CHECK(test_short_gap_bridge() == 0);
    CHECK(test_lock_label() == 0);
    puts("Hybrid anti-windup, clip and short-gap regressions passed");
    return 0;
}
