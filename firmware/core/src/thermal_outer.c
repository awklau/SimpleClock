/* SPDX-License-Identifier: MIT */
#include "simpleclock/thermal_outer.h"
#include "simpleclock/hybrid_defaults.h"

#include <limits.h>
#include <string.h>

static int32_t clamp_i64(int64_t value, int32_t low, int32_t high)
{
    return value < low ? low : (value > high ? high : (int32_t)value);
}

static uint32_t abs_i32(int32_t value)
{
    /* Temperatures and errors are bounded by the configuration, so INT32_MIN
     * is not a valid input to this helper. */
    return (uint32_t)(value < 0 ? -value : value);
}

static uint32_t ema_alpha_q31(uint32_t elapsed_ms, uint32_t tau_ms)
{
    /* Padé x/(1+x/2) approximates 1-exp(-x), with sub-0.01% error over the
     * normal one-second update.  It keeps local-jitter dependence without a
     * floating-point library or a hidden cadence assumption. */
    const uint64_t twice_elapsed = (uint64_t)elapsed_ms * 2U;
    const uint64_t denominator = (uint64_t)tau_ms * 2U + elapsed_ms;
    return twice_elapsed >= denominator
               ? UINT32_C(2147483648)
               : (uint32_t)(twice_elapsed * UINT32_C(2147483648) / denominator);
}

void sc_thermal_outer_default_config(sc_thermal_outer_config_t *config)
{
    if (!config)
    {
        return;
    }
    *config = (sc_thermal_outer_config_t){
        .target_duty_permille =
            (uint16_t)(SC_HYBRID_THERMAL_OUTER_TARGET_EXTERNAL_HEATER_DUTY_FRACTION * 1000.0),
        .duty_deadband_permille =
            (uint16_t)(SC_HYBRID_THERMAL_OUTER_DUTY_DEADBAND_FRACTION * 1000.0),
        .duty_filter_tau_ms =
            (uint32_t)(SC_HYBRID_THERMAL_OUTER_DUTY_FILTER_TIME_CONSTANT_S * 1000.0),
        .nominal_recentering_s = (uint32_t)(SC_HYBRID_THERMAL_OUTER_NOMINAL_RECENTERING_TIME_S),
        .gain_q16_per_unit_duty =
            (int32_t)(SC_HYBRID_THERMAL_OUTER_GAIN_ESTIMATE_K_PER_UNIT_DUTY * SC_THERMAL_OUTER_Q16),
        .setpoint_slew_q16_per_s =
            (int32_t)(SC_HYBRID_THERMAL_OUTER_SETPOINT_SLEW_LIMIT_K_PER_S *
                      SC_THERMAL_OUTER_Q16), /* floor keeps the cap conservative */
        .initial_target_q16 =
            (int32_t)(SC_HYBRID_THERMAL_OUTER_INITIAL_SETPOINT_IF_NO_VALID_EXISTING_VALUE_C *
                      SC_THERMAL_OUTER_Q16),
        .target_min_q16 = (int32_t)(SC_HYBRID_THERMAL_OUTER_SETPOINT_MIN_C * SC_THERMAL_OUTER_Q16),
        .target_max_q16 = (int32_t)(SC_HYBRID_THERMAL_OUTER_SETPOINT_MAX_C * SC_THERMAL_OUTER_Q16),
        .overtemperature_trip_q16 =
            (int32_t)(SC_HYBRID_THERMAL_OUTER_OVERTEMPERATURE_TRIP_C * SC_THERMAL_OUTER_Q16),
        .pause_abs_error_q16 =
            (int32_t)(SC_HYBRID_THERMAL_OUTER_PAUSE_ABS_TRACKING_ERROR_C * SC_THERMAL_OUTER_Q16),
        .pause_error_duration_s = (uint32_t)(SC_HYBRID_THERMAL_OUTER_PAUSE_ERROR_DURATION_S),
        .resume_abs_error_q16 =
            (int32_t)(SC_HYBRID_THERMAL_OUTER_RESUME_ABS_TRACKING_ERROR_C * SC_THERMAL_OUTER_Q16),
        .resume_good_duration_s = (uint32_t)(SC_HYBRID_THERMAL_OUTER_RESUME_GOOD_DURATION_S),
        .saturation_low_permille =
            (uint16_t)(SC_HYBRID_THERMAL_OUTER_SATURATION_LOW_FRACTION * 1000.0),
        .saturation_high_permille =
            (uint16_t)(SC_HYBRID_THERMAL_OUTER_SATURATION_HIGH_FRACTION * 1000.0),
        .centered_dwell_s = (uint32_t)(SC_HYBRID_THERMAL_OUTER_STEADY_POWER_TARGET_DWELL_S),
        .centered_halfwidth_permille =
            (uint16_t)(SC_HYBRID_THERMAL_OUTER_DUTY_CENTERED_STATUS_HALFWIDTH_FRACTION * 1000.0),
    };
}

static bool config_valid(const sc_thermal_outer_config_t *config)
{
    const uint32_t max_seconds = UINT32_MAX / 1000U;
    return config && config->target_duty_permille <= SC_THERMAL_OUTER_DUTY_MAX_PERMILLE &&
           config->duty_deadband_permille < SC_THERMAL_OUTER_DUTY_MAX_PERMILLE &&
           config->duty_filter_tau_ms != 0U && config->nominal_recentering_s != 0U &&
           config->gain_q16_per_unit_duty > 0 && config->setpoint_slew_q16_per_s > 0 &&
           config->initial_target_q16 >= config->target_min_q16 &&
           config->initial_target_q16 <= config->target_max_q16 &&
           config->target_min_q16 < config->target_max_q16 &&
           config->target_max_q16 < config->overtemperature_trip_q16 &&
           config->overtemperature_trip_q16 <= SC_THERMAL_OUTER_MAX_TRIP_Q16 &&
           config->pause_abs_error_q16 > 0 && config->pause_error_duration_s != 0U &&
           config->pause_error_duration_s <= max_seconds && config->resume_abs_error_q16 >= 0 &&
           config->resume_abs_error_q16 < config->pause_abs_error_q16 &&
           config->resume_good_duration_s != 0U && config->resume_good_duration_s <= max_seconds &&
           config->saturation_low_permille < config->saturation_high_permille &&
           config->saturation_high_permille <= SC_THERMAL_OUTER_DUTY_MAX_PERMILLE &&
           config->centered_dwell_s != 0U && config->centered_dwell_s <= max_seconds &&
           config->centered_halfwidth_permille <= SC_THERMAL_OUTER_DUTY_MAX_PERMILLE;
}

static void clear_window(sc_thermal_outer_state_t *state)
{
    state->duty_ms_sum = 0U;
    state->window_ms = 0U;
    state->interval_count = 0U;
    state->interval_next = 0U;
    state->window_valid = false;
    state->window_saturated = false;
    state->mean_duty_permille = 0U;
}

static uint8_t oldest_index(const sc_thermal_outer_state_t *state)
{
    return (uint8_t)((state->interval_next + SC_THERMAL_OUTER_INTERVAL_HISTORY -
                      state->interval_count) %
                     SC_THERMAL_OUTER_INTERVAL_HISTORY);
}

static void remove_oldest(sc_thermal_outer_state_t *state)
{
    if (!state->interval_count)
    {
        return;
    }
    const sc_thermal_outer_interval_t *item = &state->intervals[oldest_index(state)];
    state->window_ms -= item->duration_ms;
    state->duty_ms_sum -= (uint64_t)item->duration_ms * item->duty_permille;
    state->interval_count--;
}

static void refresh_window_flags(sc_thermal_outer_state_t *state)
{
    state->window_valid = state->window_ms >= SC_THERMAL_OUTER_WINDOW_MS;
    if (!state->window_ms)
    {
        state->mean_duty_permille = 0U;
        state->window_saturated = false;
        return;
    }
    state->mean_duty_permille =
        (uint16_t)((state->duty_ms_sum + state->window_ms / 2U) / state->window_ms);
    state->window_saturated =
        state->window_valid && (state->mean_duty_permille < state->config.saturation_low_permille ||
                                state->mean_duty_permille > state->config.saturation_high_permille);
}

static void add_interval(sc_thermal_outer_state_t *state, uint32_t duration_ms,
                         uint16_t duty_permille)
{
    if (!duration_ms)
    {
        return;
    }
    if (duration_ms > SC_THERMAL_OUTER_WINDOW_MS)
    {
        clear_window(state);
        duration_ms = SC_THERMAL_OUTER_WINDOW_MS;
    }
    if (state->interval_count == SC_THERMAL_OUTER_INTERVAL_HISTORY)
    {
        remove_oldest(state);
    }
    state->intervals[state->interval_next] =
        (sc_thermal_outer_interval_t){duration_ms, duty_permille};
    state->interval_next =
        (uint8_t)((state->interval_next + 1U) % SC_THERMAL_OUTER_INTERVAL_HISTORY);
    state->interval_count++;
    state->window_ms += duration_ms;
    state->duty_ms_sum += (uint64_t)duration_ms * duty_permille;
    while (state->window_ms > SC_THERMAL_OUTER_WINDOW_MS)
    {
        uint32_t excess = state->window_ms - SC_THERMAL_OUTER_WINDOW_MS;
        const uint8_t index = oldest_index(state);
        sc_thermal_outer_interval_t *oldest = &state->intervals[index];
        if (oldest->duration_ms <= excess)
        {
            remove_oldest(state);
        }
        else
        {
            oldest->duration_ms -= excess;
            state->window_ms -= excess;
            state->duty_ms_sum -= (uint64_t)excess * oldest->duty_permille;
        }
    }
    refresh_window_flags(state);
}

static void refresh_errors(sc_thermal_outer_state_t *state, const sc_thermal_outer_input_t *input)
{
    state->raw_error_q16 = state->target_q16 - input->raw_temperature_q16;
    state->filtered_error_q16 = state->target_q16 - input->filtered_temperature_q16;
}

static void update_centered_status(sc_thermal_outer_state_t *state, uint32_t now_ms)
{
    const int64_t duty_error_q16 =
        (int64_t)state->config.target_duty_permille * SC_THERMAL_OUTER_Q16 -
        state->filtered_duty_q16;
    const bool in_band =
        state->window_valid &&
        abs_i32(clamp_i64(duty_error_q16, INT32_MIN + 1, INT32_MAX)) <=
            (uint32_t)state->config.centered_halfwidth_permille * SC_THERMAL_OUTER_Q16 &&
        !state->power_target_limited;
    if (!in_band)
    {
        state->centered_timing = false;
        state->duty_centered = false;
    }
    else if (!state->centered_timing)
    {
        state->centered_timing = true;
        state->centered_since_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - state->centered_since_ms) >=
             state->config.centered_dwell_s * 1000U)
    {
        state->duty_centered = true;
    }
}

static void update_pause_reason(sc_thermal_outer_state_t *state)
{
    if (state->config_fault || state->safety_fault)
    {
        state->pause_reason =
            state->config_fault ? SC_THERMAL_OUTER_PAUSE_CONFIG : SC_THERMAL_OUTER_PAUSE_SAFETY;
    }
    else if (state->explicit_paused)
    {
        state->pause_reason = SC_THERMAL_OUTER_PAUSE_EXPLICIT;
    }
    else if (state->direction_paused || (state->error_paused && !state->adaptation_was_allowed))
    {
        state->pause_reason = SC_THERMAL_OUTER_PAUSE_ERROR;
    }
    else if (!state->duty_data_valid || !state->window_valid)
    {
        state->pause_reason = SC_THERMAL_OUTER_PAUSE_DUTY_DATA;
    }
    else if (!state->centering_started)
    {
        state->pause_reason = SC_THERMAL_OUTER_PAUSE_PERMISSION;
    }
    else
    {
        state->pause_reason = SC_THERMAL_OUTER_PAUSE_NONE;
    }
}

bool sc_thermal_outer_init(sc_thermal_outer_state_t *state, const sc_thermal_outer_config_t *config)
{
    if (!state)
    {
        return false;
    }
    memset(state, 0, sizeof(*state));
    if (!config_valid(config))
    {
        state->config_fault = true;
        state->enabled = false;
        state->pause_reason = SC_THERMAL_OUTER_PAUSE_CONFIG;
        return false;
    }
    state->config = *config;
    state->target_q16 = config->initial_target_q16;
    state->enabled = true;
    state->pause_reason = SC_THERMAL_OUTER_PAUSE_BOOT;
    return true;
}

bool sc_thermal_outer_adopt_target(sc_thermal_outer_state_t *state, int32_t target_q16)
{
    if (!state || state->config_fault || target_q16 < state->config.target_min_q16 ||
        target_q16 > state->config.target_max_q16)
    {
        return false;
    }
    state->target_q16 = target_q16;
    clear_window(state);
    state->duty_data_valid = false;
    state->have_update = false;
    state->centering_started = false;
    state->adaptation_was_allowed = false;
    state->target_rate_q16_per_s = 0;
    state->target_remainder_num = 0;
    state->error_paused = state->direction_paused = false;
    state->bad_error_timing = state->recovery_timing = false;
    state->centered_timing = state->duty_centered = state->power_target_limited = false;
    update_pause_reason(state);
    return true;
}

void sc_thermal_outer_pause(sc_thermal_outer_state_t *state)
{
    if (!state)
    {
        return;
    }
    state->explicit_paused = true;
    sc_thermal_outer_freeze(state);
}

void sc_thermal_outer_freeze(sc_thermal_outer_state_t *state)
{
    if (!state)
    {
        return;
    }
    state->centering_started = false;
    state->adaptation_was_allowed = false;
    state->target_rate_q16_per_s = 0;
    state->target_remainder_num = 0;
    update_pause_reason(state);
}

void sc_thermal_outer_resume(sc_thermal_outer_state_t *state)
{
    if (!state)
    {
        return;
    }
    state->explicit_paused = false;
    sc_thermal_outer_freeze(state);
}

bool sc_thermal_outer_clear_safety_fault(sc_thermal_outer_state_t *state)
{
    if (!state || state->config_fault)
    {
        return false;
    }
    state->safety_fault = false;
    state->adaptation_was_allowed = false;
    state->target_remainder_num = 0;
    state->centering_started = false;
    update_pause_reason(state);
    return true;
}

static void update_error_pause(sc_thermal_outer_state_t *state, uint32_t now_ms)
{
    const uint32_t error = abs_i32(state->raw_error_q16);
    if (!state->centering_started)
    {
        state->bad_error_timing = false;
        state->recovery_timing = false;
        return;
    }
    if (!state->error_paused)
    {
        if (error > (uint32_t)state->config.pause_abs_error_q16)
        {
            if (!state->bad_error_timing)
            {
                state->bad_error_timing = true;
                state->bad_error_since_ms = now_ms;
            }
            if ((uint32_t)(now_ms - state->bad_error_since_ms) >=
                state->config.pause_error_duration_s * 1000U)
            {
                state->error_paused = true;
                state->bad_error_timing = false;
                state->recovery_timing = false;
            }
        }
        else
        {
            state->bad_error_timing = false;
        }
        return;
    }
    if (error <= (uint32_t)state->config.resume_abs_error_q16)
    {
        if (!state->recovery_timing)
        {
            state->recovery_timing = true;
            state->recovery_since_ms = now_ms;
        }
        if ((uint32_t)(now_ms - state->recovery_since_ms) >=
            state->config.resume_good_duration_s * 1000U)
        {
            state->error_paused = false;
            state->recovery_timing = false;
        }
    }
    else
    {
        state->recovery_timing = false;
    }
}

static void update_centering_start(sc_thermal_outer_state_t *state, bool permitted)
{
    if (!permitted || !state->duty_data_valid || !state->window_valid)
    {
        state->centering_started = false;
        state->adaptation_was_allowed = false;
        return;
    }
    if (!state->centering_started)
    {
        state->centering_started = true;
        state->adaptation_was_allowed = false;
        /* Prime from a full post-handoff/holdover duty window. Boost history
         * must not bias the slow filter toward an inappropriate lower target. */
        state->filtered_duty_q16 = (uint32_t)state->mean_duty_permille * SC_THERMAL_OUTER_Q16;
        state->filtered_duty_permille = state->mean_duty_permille;
    }
}

bool sc_thermal_outer_update(sc_thermal_outer_state_t *state, uint32_t now_ms,
                             const sc_thermal_outer_input_t *input, int32_t *out_target_q16)
{
    if (!state || !input || state->config_fault)
    {
        return false;
    }
    if (out_target_q16)
    {
        *out_target_q16 = state->target_q16;
    }
    if (!input->sensor_valid || input->applied_duty_permille > SC_THERMAL_OUTER_DUTY_MAX_PERMILLE ||
        input->raw_temperature_q16 >= state->config.overtemperature_trip_q16)
    {
        state->safety_fault = true;
        sc_thermal_outer_freeze(state);
        return false;
    }
    const uint32_t elapsed = state->have_update ? now_ms - state->last_update_ms : 0U;
    if (state->have_update && elapsed == 0U)
    {
        return false;
    }
    const bool previous_duty_valid = state->duty_data_valid;
    if (!input->applied_duty_valid)
    {
        /* A missing/invalid completed timer frame is a benign outer-loop
         * pause.  The sensor remains usable by the independent heater safety
         * path, so this must not become a sensor safety fault. */
        state->duty_data_valid = false;
        state->adaptation_was_allowed = false;
        clear_window(state);
    }
    else if (!previous_duty_valid)
    {
        /* Do not manufacture an interval from an unknown preceding duty. */
        state->filtered_duty_q16 = (uint32_t)input->applied_duty_permille * SC_THERMAL_OUTER_Q16;
        state->filtered_duty_permille = input->applied_duty_permille;
        state->duty_data_valid = true;
        clear_window(state);
    }
    else if (elapsed > SC_THERMAL_OUTER_MAX_GAP_MS)
    {
        /* A scheduling/data gap breaks continuous tracking.  Refresh from the
         * current interval and rebuild the 30 s window; no catch-up move. */
        state->filtered_duty_q16 = (uint32_t)input->applied_duty_permille * SC_THERMAL_OUTER_Q16;
        state->filtered_duty_permille = input->applied_duty_permille;
        clear_window(state);
        state->centering_started = false;
        state->bad_error_timing = false;
        state->recovery_timing = false;
        state->error_paused = false;
        state->adaptation_was_allowed = false;
        state->duty_data_valid = true;
    }
    else
    {
        const uint32_t applied_q16 = (uint32_t)input->applied_duty_permille * SC_THERMAL_OUTER_Q16;
        const uint32_t alpha_q31 = ema_alpha_q31(elapsed, state->config.duty_filter_tau_ms);
        const int64_t filter_delta = ((int64_t)applied_q16 - (int64_t)state->filtered_duty_q16) *
                                     alpha_q31 / INT64_C(2147483648);
        int64_t filtered_q16 = (int64_t)state->filtered_duty_q16 + filter_delta;
        if (filtered_q16 < 0)
        {
            filtered_q16 = 0;
        }
        else if (filtered_q16 > (int64_t)SC_THERMAL_OUTER_DUTY_MAX_PERMILLE * SC_THERMAL_OUTER_Q16)
        {
            filtered_q16 = (int64_t)SC_THERMAL_OUTER_DUTY_MAX_PERMILLE * SC_THERMAL_OUTER_Q16;
        }
        state->filtered_duty_q16 = (uint32_t)filtered_q16;
        state->filtered_duty_permille =
            (uint16_t)((state->filtered_duty_q16 + SC_THERMAL_OUTER_Q16 / 2U) /
                       SC_THERMAL_OUTER_Q16);
        state->duty_data_valid = true;
    }
    if (state->have_update && previous_duty_valid && input->applied_duty_valid &&
        elapsed <= SC_THERMAL_OUTER_MAX_GAP_MS)
    {
        add_interval(state, elapsed, input->applied_duty_permille);
    }
    state->last_update_ms = now_ms;
    state->have_update = true;
    state->power_target_limited = false;
    refresh_errors(state, input);
    update_centering_start(state, input->centering_permitted && state->enabled &&
                                      !state->explicit_paused && !state->safety_fault);
    update_error_pause(state, now_ms);

    const bool timing_gap = elapsed > SC_THERMAL_OUTER_MAX_GAP_MS;
    bool allowed = state->enabled && state->duty_data_valid && !state->explicit_paused &&
                   !state->safety_fault && state->centering_started && state->window_valid &&
                   !timing_gap;
    state->direction_paused = false;
    if (allowed)
    {
        int64_t duty_error_q16 =
            (int64_t)state->config.target_duty_permille * SC_THERMAL_OUTER_Q16 -
            state->filtered_duty_q16;
        const int64_t deadband_q16 =
            (int64_t)state->config.duty_deadband_permille * SC_THERMAL_OUTER_Q16;
        int64_t deadband_error_q16 = duty_error_q16;
        if (duty_error_q16 > deadband_q16)
        {
            deadband_error_q16 -= deadband_q16;
        }
        else if (duty_error_q16 < -deadband_q16)
        {
            deadband_error_q16 += deadband_q16;
        }
        else
        {
            deadband_error_q16 = 0;
        }
        const int64_t rate_denominator =
            INT64_C(1000) * SC_THERMAL_OUTER_Q16 * state->config.nominal_recentering_s;
        int64_t rate_numerator = (int64_t)state->config.gain_q16_per_unit_duty * deadband_error_q16;
        const int64_t slew_numerator =
            (int64_t)state->config.setpoint_slew_q16_per_s * rate_denominator;
        if (rate_numerator > slew_numerator)
        {
            rate_numerator = slew_numerator;
        }
        else if (rate_numerator < -slew_numerator)
        {
            rate_numerator = -slew_numerator;
        }
        /* A single integrator can recover either rail: low duty raises the
         * target, high duty lowers it. Never increase an already excessive
         * target-minus-raw error while the inner PID is catching up. During
         * the sustained-error pause, only motion TOWARD raw temperature is
         * allowed; otherwise a hot board at zero duty could never recover. */
        const bool corrective = (rate_numerator > 0 && state->raw_error_q16 < 0) ||
                                (rate_numerator < 0 && state->raw_error_q16 > 0);
        state->direction_paused =
            (rate_numerator > 0 && state->raw_error_q16 > state->config.resume_abs_error_q16) ||
            (rate_numerator < 0 && state->raw_error_q16 < -state->config.resume_abs_error_q16) ||
            (state->error_paused && !corrective);
        allowed = !state->direction_paused;
        if (!allowed || !state->adaptation_was_allowed)
        {
            rate_numerator = 0; /* No integration across a pause or permission change. */
        }
        if (rate_numerator == 0 || (rate_numerator > 0 && state->target_remainder_num < 0) ||
            (rate_numerator < 0 && state->target_remainder_num > 0))
        {
            state->target_remainder_num = 0;
        }
        state->target_rate_q16_per_s = (int32_t)(rate_numerator / rate_denominator);
        const int64_t target_numerator = rate_numerator * elapsed + state->target_remainder_num;
        const int64_t target_delta = target_numerator / (rate_denominator * 1000L);
        state->target_remainder_num = target_numerator % (rate_denominator * 1000L);
        state->target_q16 = clamp_i64((int64_t)state->target_q16 + target_delta,
                                      state->config.target_min_q16, state->config.target_max_q16);
        if ((state->target_q16 == state->config.target_min_q16 && rate_numerator < 0) ||
            (state->target_q16 == state->config.target_max_q16 && rate_numerator > 0))
        {
            state->power_target_limited = true;
            state->target_remainder_num = 0;
        }
    }
    else
    {
        state->target_rate_q16_per_s = 0;
        state->target_remainder_num = 0;
    }
    state->adaptation_was_allowed = allowed;
    refresh_errors(state, input);
    update_centered_status(state, now_ms);
    update_pause_reason(state);
    if (out_target_q16)
    {
        *out_target_q16 = state->target_q16;
    }
    return input->applied_duty_valid;
}
