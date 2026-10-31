/* SPDX-License-Identifier: MIT */
#include "simpleclock/thermal_outer.h"

#include <stdio.h>
#include <stdint.h>

#define CHECK(expression)                                                                          \
    do                                                                                             \
    {                                                                                              \
        if (!(expression))                                                                         \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #expression);                               \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static sc_thermal_outer_input_t input(uint16_t duty, int32_t raw_q16, bool healthy)
{
    return (sc_thermal_outer_input_t){raw_q16, raw_q16, duty, false, true, true, healthy};
}

static bool tick(sc_thermal_outer_state_t *state, uint32_t now_ms, uint16_t duty, int32_t raw_q16,
                 bool healthy)
{
    int32_t target;
    sc_thermal_outer_input_t value = input(duty, raw_q16, healthy);
    return sc_thermal_outer_update(state, now_ms, &value, &target);
}

static void quick_config(sc_thermal_outer_config_t *config)
{
    sc_thermal_outer_default_config(config);
    config->pause_error_duration_s = 2U;
    config->resume_good_duration_s = 3U;
    config->centered_dwell_s = 3U;
}

static int sign_and_deadband(void)
{
    sc_thermal_outer_config_t config;
    quick_config(&config);
    config.duty_filter_tau_ms = 100U;
    sc_thermal_outer_state_t low, high, deadband;
    CHECK(sc_thermal_outer_init(&low, &config));
    CHECK(sc_thermal_outer_init(&high, &config));
    CHECK(sc_thermal_outer_init(&deadband, &config));
    const int32_t initial = config.initial_target_q16;
    for (uint32_t second = 0; second <= 35U; second++)
    {
        CHECK(tick(&low, second * 1000U, 300U, initial, true));
        CHECK(tick(&high, second * 1000U, 700U, initial, true));
        CHECK(tick(&deadband, second * 1000U, 505U, initial, true));
    }
    CHECK(low.target_q16 > initial);       /* low demand raises target */
    CHECK(high.target_q16 < initial);      /* high demand lowers target */
    CHECK(deadband.target_q16 == initial); /* continuous 49..51% deadband */
    CHECK(low.target_rate_q16_per_s > 0 && high.target_rate_q16_per_s < 0);
    return 0;
}

static int filter_and_start(void)
{
    sc_thermal_outer_config_t config;
    quick_config(&config);
    sc_thermal_outer_state_t state;
    CHECK(sc_thermal_outer_init(&state, &config));
    const int32_t target = config.initial_target_q16;
    CHECK(tick(&state, 0U, 0U, target, true));
    CHECK(state.filtered_duty_permille == 0U && !state.centering_started);
    CHECK(tick(&state, 1000U, 1000U, target, true));
    CHECK(state.filtered_duty_q16 > 0U && state.filtered_duty_q16 < 1000U * SC_THERMAL_OUTER_Q16);
    for (uint32_t second = 2U; second <= 30U; second++)
    {
        CHECK(tick(&state, second * 1000U, 500U, target, true));
    }
    CHECK(state.centering_started);          /* Fresh full window; no GNSS/tracking dwell. */
    CHECK(state.target_rate_q16_per_s == 0); /* activation epoch has no catch-up */
    CHECK(tick(&state, 33000U, 1000U, target, true));
    CHECK(tick(&state, 34000U, 1000U, target, true));
    CHECK(tick(&state, 35000U, 1000U, target, true));
    CHECK(state.target_q16 < target);
    return 0;
}

static int pause_resume_and_saturation(void)
{
    sc_thermal_outer_config_t config;
    quick_config(&config);
    config.duty_filter_tau_ms = 100U;
    sc_thermal_outer_state_t state;
    CHECK(sc_thermal_outer_init(&state, &config));
    const int32_t target = config.initial_target_q16;
    for (uint32_t second = 0U; second <= 32U; second++)
    {
        CHECK(tick(&state, second * 1000U, 500U, target, true));
    }
    CHECK(state.centering_started);
    CHECK(tick(&state, 33000U, 500U, target, true));
    const int32_t held = state.target_q16;
    CHECK(tick(&state, 34000U, 500U, target + 30000, true));
    CHECK(tick(&state, 35000U, 500U, target + 30000, true));
    CHECK(tick(&state, 36000U, 500U, target + 30000, true));
    CHECK(state.error_paused && state.pause_reason == SC_THERMAL_OUTER_PAUSE_ERROR);
    CHECK(state.target_q16 == held);
    for (uint32_t second = 37U; second <= 40U; second++)
    {
        CHECK(tick(&state, second * 1000U, 500U, target, true));
    }
    CHECK(!state.error_paused);
    CHECK(tick(&state, 41000U, 500U, target, true));
    CHECK(state.target_q16 == held); /* first good epoch only arms adaptation */
    CHECK(tick(&state, 42000U, 500U, target, true));
    CHECK(state.target_q16 == held);
    sc_thermal_outer_pause(&state);
    CHECK(state.explicit_paused);
    CHECK(tick(&state, 43000U, 0U, target, true));
    CHECK(state.target_q16 == held && state.pause_reason == SC_THERMAL_OUTER_PAUSE_EXPLICIT);
    sc_thermal_outer_resume(&state);
    CHECK(!state.explicit_paused);
    CHECK(tick(&state, 44000U, 0U, target, true));
    CHECK(state.target_q16 == held); /* resume epoch cannot catch up pause time */
    CHECK(tick(&state, 45000U, 0U, target, true));
    CHECK(state.target_q16 > held);

    sc_thermal_outer_state_t saturated;
    CHECK(sc_thermal_outer_init(&saturated, &config));
    CHECK(tick(&saturated, 0U, 0U, target, true));
    for (uint32_t second = 1U; second <= 31U; second++)
    {
        CHECK(tick(&saturated, second * 1000U, 0U, target, true));
    }
    CHECK(saturated.window_valid && saturated.window_saturated);
    CHECK(saturated.pause_reason == SC_THERMAL_OUTER_PAUSE_NONE);
    CHECK(saturated.target_q16 > target); /* Recover headroom even at zero duty. */

    sc_thermal_outer_state_t one_frame;
    CHECK(sc_thermal_outer_init(&one_frame, &config));
    CHECK(tick(&one_frame, 0U, 500U, target, true));
    for (uint32_t second = 1U; second <= 31U; second++)
    {
        sc_thermal_outer_input_t value = input(500U, target, true);
        value.actuator_saturated = second == 15U;
        CHECK(sc_thermal_outer_update(&one_frame, second * 1000U, &value, NULL));
    }
    CHECK(one_frame.mean_duty_permille == 500U && !one_frame.window_saturated);
    return 0;
}

/* Exercise both saturation rails, error direction and adoption of a target
 * after 100% boost. A fixed raw sensor is intentional: target motion must be
 * bounded even when the physical plant cannot follow. */
static int recovery_direction_and_target_adoption(void)
{
    sc_thermal_outer_config_t config;
    quick_config(&config);
    config.duty_filter_tau_ms = 100U;
    sc_thermal_outer_state_t low, high;
    CHECK(sc_thermal_outer_init(&low, &config));
    CHECK(sc_thermal_outer_init(&high, &config));
    const int32_t initial = config.initial_target_q16;
    const int32_t half_degree = SC_THERMAL_OUTER_Q16 / 2;
    for (uint32_t sec = 0; sec <= 80U; ++sec)
    {
        const int32_t before_low = low.target_q16, before_high = high.target_q16;
        CHECK(tick(&low, sec * 1000U, 0U, initial + half_degree, true));
        CHECK(tick(&high, sec * 1000U, 1000U, initial - half_degree, true));
        CHECK(low.target_q16 - before_low <= config.setpoint_slew_q16_per_s);
        CHECK(before_high - high.target_q16 <= config.setpoint_slew_q16_per_s);
    }
    CHECK(low.window_saturated && high.window_saturated);
    CHECK(low.error_paused && high.error_paused);
    CHECK(low.target_q16 > initial && high.target_q16 < initial);
    CHECK(low.pause_reason == SC_THERMAL_OUTER_PAUSE_NONE);
    CHECK(high.pause_reason == SC_THERMAL_OUTER_PAUSE_NONE);
    /* Reversing error sign must stop immediately: never push away from raw. */
    const int32_t held_low = low.target_q16, held_high = high.target_q16;
    CHECK(tick(&low, 81000U, 0U, initial - half_degree, true));
    CHECK(tick(&high, 81000U, 1000U, initial + half_degree, true));
    CHECK(low.target_q16 == held_low && high.target_q16 == held_high);
    CHECK(low.direction_paused && high.direction_paused);

    /* Adopt once, flushing old full-power history and its filtered value. */
    const int32_t captured = initial - SC_THERMAL_OUTER_Q16;
    CHECK(sc_thermal_outer_adopt_target(&high, captured));
    CHECK(!high.window_valid && !high.centering_started);
    for (uint32_t sec = 82U; sec <= 112U; ++sec)
    {
        CHECK(tick(&high, sec * 1000U, 0U, captured, true));
        CHECK(high.target_q16 == captured);
    }
    CHECK(high.filtered_duty_permille == 0U && high.window_valid);
    CHECK(tick(&high, 113000U, 0U, captured, true));
    CHECK(high.target_q16 > captured); /* Old boost cannot pull target down. */
    /* A brief PPP-loss event between frames cancels fractional motion. */
    sc_thermal_outer_freeze(&high);
    const int32_t held = high.target_q16;
    CHECK(tick(&high, 114000U, 0U, captured, true));
    CHECK(high.target_q16 == held && high.target_remainder_num == 0);
    CHECK(tick(&high, 115000U, 0U, captured, true));
    CHECK(high.target_q16 > held);
    sc_thermal_outer_pause(&high);
    CHECK(sc_thermal_outer_adopt_target(&high, captured));
    CHECK(high.explicit_paused); /* Target adoption is not an implicit resume. */
    CHECK(!sc_thermal_outer_adopt_target(&high, config.target_max_q16 + 1));
    return 0;
}

static int bounds_wrap_gap_and_status(void)
{
    sc_thermal_outer_config_t config;
    quick_config(&config);
    config.initial_target_q16 = config.target_max_q16 - 1;
    config.nominal_recentering_s = 1U;
    config.gain_q16_per_unit_duty = 100L * SC_THERMAL_OUTER_Q16;
    config.setpoint_slew_q16_per_s = SC_THERMAL_OUTER_Q16;
    sc_thermal_outer_state_t upper;
    CHECK(sc_thermal_outer_init(&upper, &config));
    for (uint32_t second = 0U; second <= 34U; second++)
    {
        CHECK(tick(&upper, second * 1000U, 300U, config.initial_target_q16, true));
    }
    CHECK(upper.target_q16 == config.target_max_q16 && upper.power_target_limited);
    config.initial_target_q16 = config.target_min_q16 + 1;
    sc_thermal_outer_state_t lower;
    CHECK(sc_thermal_outer_init(&lower, &config));
    for (uint32_t second = 0U; second <= 34U; second++)
    {
        CHECK(tick(&lower, second * 1000U, 700U, config.initial_target_q16, true));
    }
    CHECK(lower.target_q16 == config.target_min_q16 && lower.power_target_limited);

    /* A calculated rate below one Q16 count/s must still accumulate through
     * the bounded fractional residual instead of disappearing by truncation. */
    quick_config(&config);
    config.initial_target_q16 = config.target_min_q16 + 100;
    config.nominal_recentering_s = 1U;
    config.gain_q16_per_unit_duty = 1L;
    config.setpoint_slew_q16_per_s = SC_THERMAL_OUTER_Q16;
    sc_thermal_outer_state_t fractional;
    CHECK(sc_thermal_outer_init(&fractional, &config));
    const int32_t fractional_initial = config.initial_target_q16;
    for (uint32_t second = 0U; second <= 40U; second++)
    {
        CHECK(tick(&fractional, second * 1000U, 300U, fractional_initial, true));
    }
    CHECK(fractional.target_rate_q16_per_s == 0 && fractional.target_q16 > fractional_initial);

    sc_thermal_outer_state_t wrap;
    quick_config(&config);
    CHECK(sc_thermal_outer_init(&wrap, &config));
    CHECK(tick(&wrap, UINT32_MAX - 999U, 500U, config.initial_target_q16, true));
    CHECK(tick(&wrap, 0U, 500U, config.initial_target_q16, true));
    CHECK(wrap.have_update && wrap.window_ms == 1000U);
    const int32_t before_gap = wrap.target_q16;
    CHECK(tick(&wrap, 10000U, 300U, config.initial_target_q16, true));
    CHECK(wrap.target_q16 == before_gap && !wrap.centering_started);
    CHECK(!wrap.window_valid && wrap.window_ms == 0U);
    for (uint32_t second = 11U; second < 40U; second++)
    {
        CHECK(tick(&wrap, second * 1000U, 300U, config.initial_target_q16, true));
    }
    CHECK(!wrap.window_valid); /* missing gap cannot be filled with synthetic duty */
    CHECK(tick(&wrap, 40000U, 300U, config.initial_target_q16, true));
    CHECK(wrap.window_valid);

    sc_thermal_outer_state_t bad_frame;
    CHECK(sc_thermal_outer_init(&bad_frame, &config));
    for (uint32_t second = 0U; second <= 32U; second++)
    {
        CHECK(tick(&bad_frame, second * 1000U, 500U, config.initial_target_q16, true));
    }
    sc_thermal_outer_input_t invalid_duty = input(500U, config.initial_target_q16, true);
    invalid_duty.applied_duty_valid = false;
    CHECK(!sc_thermal_outer_update(&bad_frame, 33000U, &invalid_duty, NULL));
    CHECK(!bad_frame.safety_fault && !bad_frame.duty_data_valid);
    CHECK(bad_frame.pause_reason == SC_THERMAL_OUTER_PAUSE_DUTY_DATA);
    const int32_t held_for_frame_loss = bad_frame.target_q16;
    for (uint32_t second = 34U; second <= 64U; second++)
    {
        CHECK(tick(&bad_frame, second * 1000U, 300U, config.initial_target_q16, true));
        CHECK(bad_frame.target_q16 == held_for_frame_loss);
    }
    CHECK(bad_frame.window_valid);
    CHECK(tick(&bad_frame, 65000U, 300U, config.initial_target_q16, true));
    CHECK(bad_frame.target_q16 > held_for_frame_loss);

    sc_thermal_outer_state_t centered;
    CHECK(sc_thermal_outer_init(&centered, &config));
    CHECK(tick(&centered, 0U, 500U, config.initial_target_q16, true));
    for (uint32_t second = 1U; second <= 34U; second++)
    {
        CHECK(tick(&centered, second * 1000U, 500U, config.initial_target_q16, true));
    }
    CHECK(centered.window_valid && centered.duty_centered);
    return 0;
}

int main(void)
{
    sc_thermal_outer_config_t defaults;
    sc_thermal_outer_default_config(&defaults);
    CHECK(defaults.target_duty_permille == 500U && defaults.duty_deadband_permille == 10U);
    CHECK(defaults.target_min_q16 == 0 && defaults.target_max_q16 == 60L * SC_THERMAL_OUTER_Q16 &&
          defaults.overtemperature_trip_q16 == 70L * SC_THERMAL_OUTER_Q16);
    CHECK(sign_and_deadband() == 0);
    CHECK(filter_and_start() == 0);
    CHECK(pause_resume_and_saturation() == 0);
    CHECK(bounds_wrap_gap_and_status() == 0);
    CHECK(recovery_direction_and_target_adoption() == 0);

    sc_thermal_outer_state_t state;
    CHECK(sc_thermal_outer_init(&state, &defaults));
    sc_thermal_outer_input_t trip = input(500U, defaults.overtemperature_trip_q16, true);
    CHECK(!sc_thermal_outer_update(&state, 0U, &trip, NULL));
    CHECK(state.safety_fault && state.pause_reason == SC_THERMAL_OUTER_PAUSE_SAFETY);
    CHECK(sc_thermal_outer_clear_safety_fault(&state));
    CHECK(!state.safety_fault && !state.centering_started);
    defaults.overtemperature_trip_q16 = SC_THERMAL_OUTER_MAX_TRIP_Q16 + 1;
    CHECK(!sc_thermal_outer_init(&state, &defaults));
    defaults.overtemperature_trip_q16 = SC_THERMAL_OUTER_MAX_TRIP_Q16;
    defaults.target_min_q16 = defaults.target_max_q16;
    CHECK(!sc_thermal_outer_init(&state, &defaults));
    CHECK(state.config_fault && !state.enabled);
    return 0;
}
