/* SPDX-License-Identifier: MIT */
/* Actual clock and thermal controllers coupled to a synthetic thermal plant.
 * These cases exercise software interaction only; plant gain and lag are
 * explicit assumptions, not measurements of this OCXO or enclosure. */
#include "simpleclock/hybrid.h"
#include "simpleclock/sit5801.h"
#include "simpleclock/thermal_control.h"
#include "simpleclock/thermal_outer.h"

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

#define SENSOR_RING 64U
#define SENSOR_MAX_DELAY_S 25U

typedef struct
{
    const char *name;
    double heater_gain_K_per_duty;
    uint32_t plant_lag_s;
    uint32_t sensor_delay_s;
} plant_case_t;

typedef struct
{
    double temperature_C;
    double ambient_C;
    double sensor_ring[SENSOR_RING];
    double free_drift_ppb;
    double thermal_coefficient_ppb_per_C;
    double previous_rx_drift_ppb;
    double bias_ns;
    double boot_ppb;
    uint32_t sample_sequence;
    int64_t physical_word;
    int64_t readback_word;
    bool readback_pending;
} fixture_t;

static int32_t q16(double celsius)
{
    const double scaled = celsius * (double)SC_THERMAL_Q16;
    return (int32_t)(scaled + (scaled < 0.0 ? -0.5 : 0.5));
}

static int16_t raw128(double celsius)
{
    const double scaled = celsius * 128.0;
    return (int16_t)(scaled + (scaled < 0.0 ? -0.5 : 0.5));
}

static uint16_t word_clip(void)
{
    /* The test uses a 32 ppb receiver clip, like the default synthetic clock
     * fixtures; it is deliberately wider than the 10 ppb boot envelope. */
    return 32U;
}

static void plant_step(fixture_t *fixture, uint16_t duty_permille, const plant_case_t *plant)
{
    const double equilibrium_C =
        fixture->ambient_C + plant->heater_gain_K_per_duty * ((double)duty_permille / 1000.0);
    fixture->temperature_C += (equilibrium_C - fixture->temperature_C) / (double)plant->plant_lag_s;
}

static double delayed_temperature(const fixture_t *fixture, uint32_t second, uint32_t delay_s)
{
    const uint32_t slot = (second + SENSOR_RING - delay_s) % SENSOR_RING;
    return fixture->sensor_ring[slot];
}

/* Fixture parameters, not compiled defaults: these scenarios exercise BOTH-mode
 * FLL acquisition with PPP 60/600 s and SBAS/standalone 120/3000 s. */
static void legacy_profiles(sc_hybrid_t *h)
{
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_PPP, 60.0, 600.0);
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_SBAS, 120.0, 3000.0);
    (void)sc_hybrid_profile_parameters(h, SC_HYBRID_PROFILE_STANDALONE, 120.0, 3000.0);
}

static int step_fixture(sc_hybrid_t *hybrid, sc_thermal_control_t *inner,
                        sc_thermal_outer_state_t *outer, fixture_t *fixture,
                        const plant_case_t *plant, uint32_t second, bool ppp_valid,
                        uint16_t *previous_duty, int32_t *previous_target, unsigned *max_fcw_step)
{
    const uint32_t now_ms = second * 1000U;
    const uint64_t epoch_ms = (uint64_t)second * 1000U;

    if (fixture->readback_pending)
    {
        /* A completed write is reported as a full exact three-word readback
         * at the next synthetic receiver epoch. */
        sc_hybrid_observed(hybrid, true, fixture->readback_word, word_clip(), now_ms);
        fixture->readback_pending = false;
    }
    else
    {
        fixture->readback_word = fixture->physical_word;
        sc_hybrid_observed(hybrid, true, fixture->readback_word, word_clip(), now_ms);
    }

    plant_step(fixture, *previous_duty, plant);
    fixture->sensor_ring[second % SENSOR_RING] = fixture->temperature_C;
    const double measured_C = delayed_temperature(fixture, second, plant->sensor_delay_s);
    const sc_temperature_t temperature = {
        .valid = true,
        .raw = raw128(measured_C),
        .sequence = ++fixture->sample_sequence,
        .sample_ms = now_ms,
    };
    if (!sc_thermal_control_update(inner, &temperature, now_ms))
    {
        return 1;
    }

    const double physical_drift_ppb =
        fixture->free_drift_ppb +
        fixture->thermal_coefficient_ppb_per_C *
            (fixture->temperature_C -
             (double)outer->config.initial_target_q16 / SC_THERMAL_OUTER_Q16);
    const double rx_drift_ppb = physical_drift_ppb + sit5801_fcw_to_ppb(fixture->physical_word);
    fixture->bias_ns += 0.5 * (fixture->previous_rx_drift_ppb + rx_drift_ppb);
    fixture->previous_rx_drift_ppb = rx_drift_ppb;

    sc_hybrid_service(hybrid, now_ms);
    sc_hybrid_input_t input = {0};
    input.epoch_ms = epoch_ms;
    input.now_ms = now_ms;
    input.received_ms = now_ms;
    input.bias_ns = fixture->bias_ns;
    input.drift_ppb = rx_drift_ppb;
    input.gnss_valid = ppp_valid; /* Receiver data only; no thermal input. */
    sc_hybrid_sample(hybrid, &input);

    int64_t proposed_word = 0;
    if (sc_hybrid_take_command(hybrid, now_ms, &proposed_word))
    {
        const double before = sit5801_fcw_to_ppb(fixture->physical_word);
        const double after = sit5801_fcw_to_ppb(proposed_word);
        const unsigned step = (unsigned)ceil(fabs(after - before) * 1000.0);
        if (step > *max_fcw_step)
        {
            *max_fcw_step = step;
        }
        fixture->physical_word = proposed_word;
        fixture->readback_word = proposed_word;
        fixture->readback_pending = true;
        sc_hybrid_written(hybrid, true, now_ms + 1U);
    }
    if (fabs(sit5801_fcw_to_ppb(fixture->physical_word) - fixture->boot_ppb) > 10.00001)
    {
        return 1;
    }

    const sc_thermal_outer_input_t outer_input = {
        .raw_temperature_q16 = q16(measured_C),
        .filtered_temperature_q16 = inner->filtered_q16,
        .applied_duty_permille = *previous_duty,
        .actuator_saturated = inner->saturated,
        .applied_duty_valid = true,
        .sensor_valid = true,
        .centering_permitted = inner->running, /* Thermal-only permission. */
    };
    int32_t target_q16 = 0;
    if (!sc_thermal_outer_update(outer, now_ms, &outer_input, &target_q16))
    {
        return 1;
    }
    if (target_q16 != inner->target_q16 && !sc_thermal_control_target(inner, target_q16))
    {
        return 1;
    }
    *previous_duty = inner->duty_permille;
    if (previous_target)
    {
        const int64_t delta = (int64_t)outer->target_q16 - *previous_target;
        const uint32_t magnitude = (uint32_t)(delta < 0 ? -delta : delta);
        if (magnitude > (uint32_t)outer->config.setpoint_slew_q16_per_s + 1U)
        {
            return 1;
        }
        *previous_target = outer->target_q16;
    }
    return 0;
}

static int run_case(const plant_case_t *plant)
{
    sc_hybrid_t hybrid;
    sc_hybrid_init(&hybrid);
    legacy_profiles(&hybrid);
    sc_thermal_outer_config_t outer_config;
    sc_thermal_outer_default_config(&outer_config);
    sc_thermal_outer_state_t outer;
    CHECK(sc_thermal_outer_init(&outer, &outer_config));
    sc_thermal_control_t inner;
    sc_thermal_control_init(&inner);
    CHECK(sc_thermal_control_pid_defaults(&inner));

    fixture_t fixture = {0};
    const double initial_target_C = (double)outer_config.initial_target_q16 / SC_THERMAL_OUTER_Q16;
    fixture.temperature_C = initial_target_C;
    fixture.ambient_C = initial_target_C - 0.5 * plant->heater_gain_K_per_duty;
    fixture.free_drift_ppb = 0.06;
    fixture.thermal_coefficient_ppb_per_C = 0.25;
    fixture.bias_ns = 12000.0;
    for (uint32_t i = 0U; i < SENSOR_RING; ++i)
    {
        fixture.sensor_ring[i] = fixture.temperature_C;
    }

    /* Begin this simulation from a valid warm sample and bumpless 50% PID
     * duty. Full-power slope capture has separate thermal startup tests. */
    sc_temperature_t warm = {
        .valid = true,
        .raw = raw128(fixture.temperature_C),
        .sequence = 1U,
        .sample_ms = 0U,
    };
    fixture.sample_sequence = 1U;
    CHECK(sc_thermal_control_update(&inner, &warm, 0U));
    CHECK(sc_thermal_control_start(&inner, outer_config.initial_target_q16, 500U, 0U));
    uint16_t previous_duty = 500U;
    int32_t previous_target = outer.target_q16;
    unsigned max_fcw_step = 0U;

    CHECK(sit5801_ppb_to_fcw(0.45, &fixture.physical_word));
    fixture.boot_ppb = sit5801_fcw_to_ppb(fixture.physical_word);
    fixture.readback_word = fixture.physical_word;
    fixture.previous_rx_drift_ppb =
        fixture.free_drift_ppb + sit5801_fcw_to_ppb(fixture.physical_word);
    sc_hybrid_observed(&hybrid, true, fixture.readback_word, word_clip(), 0U);

    bool reached_track = false;
    bool saw_pre_acquisition_centering = false;
    bool saw_pre_acquisition_motion = false;
    bool saw_qualification_motion = false;
    bool saw_acquisition_motion = false;
    uint32_t track_start_second = 0U;
    uint32_t outer_start_second = 0U;
    uint32_t acquisition_start_second = 0U;
    uint32_t second = 1U;
    for (; second <= 1600U && !hybrid.locked; ++second)
    {
        if (second == 10U)
        {
            /* A small ambient step produces a real duty imbalance before the
             * clock has ever acquired; centering must already be permitted. */
            fixture.ambient_C += 0.2;
        }
        const bool ppp_valid = second >= 180U;
        const int32_t target_before = outer.target_q16;
        CHECK(step_fixture(&hybrid, &inner, &outer, &fixture, plant, second, ppp_valid,
                           &previous_duty, &previous_target, &max_fcw_step) == 0);
        if (second < 180U && outer.centering_started && hybrid.attempts == 0U)
        {
            saw_pre_acquisition_centering = true;
        }
        if (second < 180U && outer.target_q16 != target_before)
        {
            saw_pre_acquisition_motion = true;
        }
        if (hybrid.qualifying && outer.target_q16 != target_before)
        {
            saw_qualification_motion = true;
        }
        if ((hybrid.state == SC_HYBRID_ACQUIRE || hybrid.state == SC_HYBRID_VERIFY) &&
            outer.target_q16 != target_before)
        {
            saw_acquisition_motion = true;
        }
        if (hybrid.attempts != 0U && !acquisition_start_second)
        {
            acquisition_start_second = second;
        }
        if (sc_hybrid_tracking(&hybrid) && hybrid.phase_valid && hybrid.generation == 1U)
        {
            reached_track = true;
            if (!track_start_second)
            {
                track_start_second = second;
            }
        }
        if (outer.centering_started && !outer_start_second)
        {
            outer_start_second = second;
        }
    }
    CHECK(reached_track && hybrid.locked && hybrid.state == SC_HYBRID_TRACK);
    CHECK(track_start_second && outer.centering_started && outer.window_valid);
    CHECK(saw_pre_acquisition_centering && saw_pre_acquisition_motion);
    CHECK(outer_start_second == SC_THERMAL_OUTER_WINDOW_MS / 1000U + 1U);
    CHECK(acquisition_start_second > outer_start_second && outer_start_second < track_start_second);
    CHECK(saw_qualification_motion && saw_acquisition_motion);
    const double phase_reference = hybrid.phase_reference_ns;
    const int64_t boot_word = fixture.physical_word;

    /* A moderate synthetic ambient step changes the power duty needed to hold
     * temperature. The outer controller slowly moves target; the actual FLL
     * remains live as the thermal plant changes its frequency contribution. */
    fixture.ambient_C += 0.2; /* Moderate +0.2 C step, below the outer pause threshold. */
    const int32_t target_before_ramp = outer.target_q16;
    const double applied_before_ramp = hybrid.applied_ppb;
    const uint32_t writes_before_ramp = hybrid.writes;
    int32_t target_after_ramp = target_before_ramp;
    bool target_moved = false;
    for (; second <= 3000U; ++second)
    {
        CHECK(step_fixture(&hybrid, &inner, &outer, &fixture, plant, second, true, &previous_duty,
                           &previous_target, &max_fcw_step) == 0);
        if (outer.target_q16 > target_before_ramp + q16(0.001))
        {
            target_moved = true;
        }
        target_after_ramp = outer.target_q16;
        CHECK(sc_hybrid_tracking(&hybrid) && hybrid.phase_valid && hybrid.generation == 1U);
        CHECK(!outer.error_paused);
        if (!fixture.readback_pending)
        {
            CHECK(fabs(hybrid.applied_ppb - sit5801_fcw_to_ppb(fixture.physical_word)) < 0.00001);
        }
    }
    CHECK(target_moved && outer.target_q16 > target_before_ramp);
    CHECK(hybrid.writes > writes_before_ramp);
    CHECK(fabs(hybrid.applied_ppb - applied_before_ramp) > 0.0001);
    CHECK(fabs(hybrid.phase_reference_ns - phase_reference) < 1e-9);
    CHECK(fabs(sit5801_fcw_to_ppb(fixture.physical_word) - sit5801_fcw_to_ppb(boot_word)) < 10.0);

    /* Continuous synthetic PPP loss freezes the verified FCW (clock holdover)
     * while temperature conversions, inner PI and outer centering continue. */
    const int64_t held_word = fixture.physical_word;
    const uint32_t updates_before_loss = inner.updates;
    const uint32_t writes_before_loss = hybrid.writes;
    for (unsigned outage = 0U; outage < 15U; ++outage, ++second)
    {
        CHECK(step_fixture(&hybrid, &inner, &outer, &fixture, plant, second, false, &previous_duty,
                           &previous_target, &max_fcw_step) == 0);
        /* Bridged for the first ~11 s, then holdover; the trim is held throughout. */
        CHECK(!hybrid.gnss_good && fixture.physical_word == held_word);
        CHECK(outage < 11U ? sc_hybrid_tracking(&hybrid) : hybrid.state == SC_HYBRID_HOLDOVER);
    }
    CHECK(inner.updates == updates_before_loss + 15U && hybrid.writes == writes_before_loss);
    CHECK(hybrid.phase_valid && hybrid.generation == 1U &&
          fabs(hybrid.phase_reference_ns - phase_reference) < 1e-9);

    bool requalified = false;
    for (unsigned recovery = 0U; recovery < 1000U && !requalified; ++recovery, ++second)
    {
        CHECK(step_fixture(&hybrid, &inner, &outer, &fixture, plant, second, true, &previous_duty,
                           &previous_target, &max_fcw_step) == 0);
        CHECK(hybrid.phase_valid && hybrid.generation == 1U &&
              fabs(hybrid.phase_reference_ns - phase_reference) < 1e-9);
        requalified = sc_hybrid_tracking(&hybrid);
    }
    CHECK(requalified);
    CHECK(fabs(sit5801_fcw_to_ppb(fixture.physical_word) - sit5801_fcw_to_ppb(boot_word)) < 10.0);
    CHECK(max_fcw_step <= 51U); /* <=0.051 ppb per 1 Hz epoch during acquisition. */
    CHECK(outer.target_q16 >= outer.config.target_min_q16 &&
          outer.target_q16 <= outer.config.target_max_q16);
    printf("%s synthetic plant: gain=%.1f K/duty lag=%u s sensor_delay=%u s "
           "target_delta=%.4f C FLL_writes=%u phase_generation=%u\n",
           plant->name, plant->heater_gain_K_per_duty, (unsigned int)plant->plant_lag_s,
           (unsigned int)plant->sensor_delay_s,
           (double)(target_after_ramp - target_before_ramp) / SC_THERMAL_OUTER_Q16,
           (unsigned int)hybrid.writes, (unsigned int)hybrid.generation);
    return 0;
}

int main(void)
{
    const plant_case_t cases[] = {
        {"low_gain_fast_lag", 5.0, 240U, 8U},
        {"high_gain_slow_lag", 14.0, 900U, SENSOR_MAX_DELAY_S},
    };
    CHECK(run_case(&cases[0]) == 0);
    CHECK(run_case(&cases[1]) == 0);
    return 0;
}
