/* SPDX-License-Identifier: MIT */
#include "simpleclock/thermal_control.h"
#include "simpleclock/hybrid_defaults.h"
#include <limits.h>
#include <string.h>

#define FILTER_MS 5000U               /* Exponential filter time constant, milliseconds. */
#define CADENCE_MS 1000U              /* Minimum accepted sample cadence, milliseconds. */
#define RAMP_Q16_PER_S 328L           /* 0.00500488 K/s, within 0.1% of 0.005 K/s. */
#define INTEGRAL_BOUND_MICRO 8000000L /* Integral bound in duty millionths. */
/* Legacy strict stability diagnostics. LED3/clock readiness are evaluated
 * separately; these thresholds never gate PID output or automatic steering. */
#define ENTRY_ERROR_Q16 3276L         /* <=0.04999 K, conservative rounding of 0.05 K. */
#define STABLE_SLOPE_Q16 655L         /* <=0.009995 K/min. */
#define SLOPE_WINDOW_MIN_MS 119000U   /* 120 points must span at least 119 s. */
#define SLOPE_WINDOW_MAX_MS 300000U   /* Do not fit a window older than 300 s. */
#define STABLE_ENTRY_DWELL_MS 300000U /* Required continuous entry dwell, milliseconds. */

static int32_t clamp64(int64_t value, int32_t low, int32_t high)
{
    return value < low ? low : (value > high ? high : (int32_t)value);
}

static int32_t magnitude(int32_t value)
{
    /* Callers use bounded temperature differences or a slope clamped to
     * [-INT32_MAX, INT32_MAX]; INT32_MIN is excluded before this helper. */
    return value < 0 ? -value : value;
}

static void reset_window(sc_thermal_control_t *state)
{
    state->count = state->next = 0;
    state->stable = state->slope_valid = false;
    state->entry_timing = state->exit_timing = false;
    state->window_saturated = false;
    state->mean_permille = 0;
}

void sc_thermal_control_init(sc_thermal_control_t *state)
{
    memset(state, 0, sizeof(*state));
    state->gains = (sc_thermal_gains_t){SC_THERMAL_DEFAULT_KP_MICRO, SC_THERMAL_DEFAULT_KI_MICRO,
                                        SC_THERMAL_PI_MAX_PERMILLE};
}

void sc_thermal_control_stop(sc_thermal_control_t *state)
{
    state->running = state->saturated = false;
    state->duty_permille = 0;
    state->integral_micro = 0;
    state->derivative_micro = state->derivative_rate_q16_per_s = 0;
    reset_window(state);
}

bool sc_thermal_control_start(sc_thermal_control_t *state, int32_t target_q16,
                              uint16_t previous_permille, uint32_t now_ms)
{
    if (!state->filter_valid || target_q16 < -55L * SC_THERMAL_Q16 ||
        target_q16 > SC_THERMAL_TARGET_MAX_Q16 || previous_permille > state->gains.max_permille)
    {
        return false;
    }
    state->target_q16 = target_q16;
    state->ramp_q16 = state->filtered_q16;
    state->integral_micro = (int32_t)previous_permille * 1000;
    state->duty_permille = previous_permille;
    state->control_ms = now_ms;
    state->derivative_micro = state->derivative_rate_q16_per_s = 0;
    state->running = true;
    state->saturated = false;
    reset_window(state);
    return true;
}

bool sc_thermal_control_gains(sc_thermal_control_t *state, sc_thermal_gains_t gains)
{
    /* Bounded profile range also bounds all products and the bumpless integral.
     * This API is tested but gains are fixed in the initial board command set. */
    if (gains.kp_micro <= 0 || gains.kp_micro > 8000000 || gains.ki_micro < 0 ||
        gains.ki_micro > 100000 || !gains.max_permille ||
        gains.max_permille > SC_THERMAL_PI_MAX_PERMILLE ||
        state->duty_permille > gains.max_permille)
    {
        return false;
    }
    const int32_t error = state->ramp_q16 - state->filtered_q16;
    const int64_t integral = (int64_t)state->duty_permille * 1000 -
                             (int64_t)gains.kp_micro * error / SC_THERMAL_Q16 -
                             state->derivative_micro;
    if (state->running && (integral < -INTEGRAL_BOUND_MICRO || integral > INTEGRAL_BOUND_MICRO))
    {
        return false; /* Reject a retune that cannot be bumpless within numeric bounds. */
    }
    state->gains = gains;
    state->integral_micro = state->running ? (int32_t)integral : 0;
    reset_window(state);
    return true;
}

bool sc_thermal_control_derivative(sc_thermal_control_t *state, sc_thermal_derivative_t config)
{
    if (state->running || config.kd_micro < 0 || config.kd_micro > 64000000 ||
        config.limit_micro < 0 || config.limit_micro > 500000 ||
        (config.kd_micro &&
         (!config.limit_micro || config.filter_ms < 500U || config.filter_ms > 30000U)))
    {
        return false;
    }
    state->derivative = config;
    state->derivative_micro = state->derivative_rate_q16_per_s = 0;
    return true;
}

bool sc_thermal_control_pid_defaults(sc_thermal_control_t *state)
{
    /* Td=Kd/Kp=6.5 s is a model-tested anticipation time, not an exact
     * cancellation of transport delay. See THERMAL_TUNING.md for the fit,
     * frequency-domain check, noise limits and unverified hardware recovery. */
    const sc_thermal_derivative_t d = {
        (int32_t)(SC_HYBRID_THERMAL_INNER_KD_DUTY_S_PER_K * 1000000.0),
        (int32_t)(SC_HYBRID_THERMAL_INNER_DERIVATIVE_LIMIT_DUTY * 1000000.0),
        (uint32_t)(SC_HYBRID_THERMAL_INNER_DERIVATIVE_FILTER_S * 1000.0)};
    const sc_thermal_gains_t pi = {(int32_t)(SC_HYBRID_THERMAL_INNER_KP_DUTY_PER_K * 1000000.0),
                                   (int32_t)(SC_HYBRID_THERMAL_INNER_KI_DUTY_PER_K_S * 1000000.0),
                                   SC_THERMAL_PI_MAX_PERMILLE};
    return sc_thermal_control_derivative(state, d) && sc_thermal_control_gains(state, pi);
}

static void update_window(sc_thermal_control_t *state, uint32_t now_ms)
{
    state->history[state->next] = (sc_thermal_point_t){
        now_ms, state->filtered_q16 - state->target_q16, state->duty_permille, state->saturated};
    state->next = (uint8_t)((state->next + 1U) % SC_THERMAL_HISTORY);
    if (state->count < SC_THERMAL_HISTORY)
    {
        state->count++;
    }
    int64_t sum_x = 0, sum_y = 0, sum_xx = 0, sum_xy = 0;
    uint32_t sum_duty = 0, oldest_ms = 0;
    state->window_saturated = false;
    for (unsigned i = 0; i < state->count; i++)
    {
        const sc_thermal_point_t *point = &state->history[i];
        const uint32_t age = now_ms - point->ms;
        if (age > oldest_ms)
        {
            oldest_ms = age;
        }
        /* Tenths of a second keep worst-case covariance*600 below 2^59 even
         * across the whole sensor range and a 300 s window. Subtracting the
         * latest temperature removes the irrelevant Celsius offset. */
        /* x is age in tenths of a second. The minus sign below makes a
         * positive physical temperature slope produce a positive result even
         * though older samples have larger age. */
        const int64_t x = (int64_t)(age / 100U);
        const int64_t y =
            (int64_t)point->error_q16 - ((int64_t)state->filtered_q16 - state->target_q16);
        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
        sum_duty += point->duty_permille;
        state->window_saturated |= point->saturated;
    }
    state->mean_permille = (uint16_t)(sum_duty / state->count);
    const int64_t variance = state->count * sum_xx - sum_x * sum_x;
    state->slope_valid = state->count == SC_THERMAL_HISTORY && oldest_ms >= SLOPE_WINDOW_MIN_MS &&
                         oldest_ms < SLOPE_WINDOW_MAX_MS && variance > 0;
    if (state->slope_valid)
    {
        const int64_t covariance = state->count * sum_xy - sum_x * sum_y;
        state->slope_q16_per_min = clamp64(-covariance * 600 / variance, -INT32_MAX, INT32_MAX);
    }
}

static void qualify(sc_thermal_control_t *state, uint32_t now_ms)
{
    const int32_t error = magnitude(state->target_q16 - state->filtered_q16);
    const bool eligible =
        state->ramp_q16 == state->target_q16 && error <= ENTRY_ERROR_Q16 && state->slope_valid &&
        magnitude(state->slope_q16_per_min) <= STABLE_SLOPE_Q16 && !state->window_saturated;
    if (!eligible)
    {
        state->entry_timing = state->stable = false;
    }
    else if (!state->entry_timing)
    {
        state->entry_timing = true;
        state->stable_since_ms = now_ms;
    }
    else if ((uint32_t)(now_ms - state->stable_since_ms) >= STABLE_ENTRY_DWELL_MS)
    {
        state->stable = true;
        state->exit_timing = false;
    }
}

bool sc_thermal_control_target(sc_thermal_control_t *state, int32_t target_q16)
{
    if (!state->running || target_q16 < 0 || target_q16 > SC_THERMAL_TARGET_MAX_Q16)
    {
        return false;
    }
    state->target_q16 = target_q16;
    return true;
}

bool sc_thermal_control_update(sc_thermal_control_t *state, const sc_temperature_t *sample,
                               uint32_t now_ms)
{
    if (!sc_temperature_fresh(sample, now_ms) || sample->raw < -55 * 128 || sample->raw > 150 * 128)
    {
        sc_thermal_control_stop(state);
        state->filter_valid = false;
        return false;
    }
    const uint32_t elapsed = now_ms - state->update_ms;
    if (state->filter_valid && (sample->sequence == state->sequence || elapsed < CADENCE_MS))
    {
        /* Foreground polling retries an early fresh conversion when cadence
         * expires. Do not consume its sequence here or wait for another shot. */
        return false;
    }
    const int32_t measured = (int32_t)sample->raw * 512; /* raw/128 C -> signed Q16 C */
    if (!state->filter_valid || elapsed >= SC_TEMPERATURE_STALE_MS)
    {
        /* A missed interval cannot be integrated as a burst or count as stable.
         * The actuator independently faults an active control-update timeout. */
        sc_thermal_control_stop(state);
        state->filtered_q16 = measured;
        state->filter_valid = true;
    }
    else
    {
        state->filtered_q16 +=
            (int32_t)(((int64_t)measured - state->filtered_q16) * elapsed / (FILTER_MS + elapsed));
    }
    state->sequence = sample->sequence;
    state->update_ms = now_ms;
    state->updates++;
    if (state->running)
    {
        if (state->derivative.kd_micro)
        {
            /* Differentiate raw measurements through a separate low-pass, so
             * P/I smoothing does not add its 5 s delay to disturbance detection.
             * Every term has units Q16 K*ms/s; division yields Q16 K/s.
             * No integration across stale data: the branch above stops first. */
            const int64_t rate_numerator =
                (int64_t)state->derivative_rate_q16_per_s * state->derivative.filter_ms +
                ((int64_t)measured - state->measured_q16) * 1000;
            state->derivative_rate_q16_per_s = clamp64(
                rate_numerator / (state->derivative.filter_ms + elapsed), -INT32_MAX, INT32_MAX);
            state->derivative_micro =
                clamp64(-(int64_t)state->derivative.kd_micro * state->derivative_rate_q16_per_s /
                            SC_THERMAL_Q16,
                        -state->derivative.limit_micro, state->derivative.limit_micro);
        }
        /* Activation can fall between sensor updates. Integrate/ramp only the
         * time since activation, while the filter retains its own cadence. */
        const uint32_t control_elapsed = now_ms - state->control_ms;
        state->control_ms = now_ms;
        const int32_t ramp_step = (int32_t)((int64_t)RAMP_Q16_PER_S * control_elapsed / 1000);
        const int32_t target_delta = state->target_q16 - state->ramp_q16;
        state->ramp_q16 += clamp64(target_delta, -ramp_step, ramp_step);
        const int32_t error = state->ramp_q16 - state->filtered_q16;
        const int64_t proportional = (int64_t)state->gains.kp_micro * error / SC_THERMAL_Q16;
        const int64_t pd = proportional + state->derivative_micro;
        const int32_t upper = (int32_t)state->gains.max_permille * 1000;
        const int64_t raw = pd + state->integral_micro;
        const int64_t increment =
            (int64_t)state->gains.ki_micro * error * control_elapsed / (SC_THERMAL_Q16 * 1000);
        /* Conditional anti-windup: integrate out of either bound, never farther
         * into it. Bound a crossing step at the output limit as well. */
        if (!((raw >= upper && increment > 0) || (raw <= 0 && increment < 0)))
        {
            int64_t candidate = (int64_t)state->integral_micro + increment;
            if (increment > 0 && pd + candidate > upper)
            {
                candidate = upper - pd;
            }
            if (increment < 0 && pd + candidate < 0)
            {
                candidate = -pd;
            }
            state->integral_micro = clamp64(candidate, -INTEGRAL_BOUND_MICRO, INTEGRAL_BOUND_MICRO);
        }
        const int64_t output = pd + state->integral_micro;
        state->duty_permille = (uint16_t)((clamp64(output, 0, upper) + 500) / 1000);
        /* Qualification uses the quantized actuator command: rounding to zero
         * or the cap still means persistent loss of output headroom. */
        state->saturated =
            state->duty_permille == 0 || state->duty_permille == state->gains.max_permille;
    }
    state->measured_q16 = measured;
    update_window(state, now_ms);
    if (state->running)
    {
        qualify(state, now_ms);
    }
    return true;
}
