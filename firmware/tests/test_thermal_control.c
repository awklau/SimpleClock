/* SPDX-License-Identifier: MIT */
#include "simpleclock/thermal_control.h"
#include <stdio.h>
#define CHECK(e)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(e))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #e);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static sc_temperature_t sample(uint32_t ms, uint32_t sequence, int16_t raw)
{
    return (sc_temperature_t){.sample_ms = ms, .sequence = sequence, .raw = raw, .valid = true};
}

static bool step(sc_thermal_control_t *state, uint32_t ms, uint32_t sequence, int16_t raw)
{
    const sc_temperature_t value = sample(ms, sequence, raw);
    return sc_thermal_control_update(state, &value, ms);
}

static int model(double capacity_j_per_k, double loss_w_per_k, unsigned lag_s, double target_c,
                 bool reachable)
{
    /* Host-only one-node model; deliberately separate double arithmetic from
     * the fixed-point controller. This is design evidence, not board tuning.
     * Model time step is 1 s, Pmax=1 W, heater-off operating point=40 C. */
    sc_thermal_control_t state;
    sc_thermal_control_init(&state);
    double plant_c = 40.0, delayed[32], peak_c = plant_c;
    for (unsigned i = 0; i < 32; i++)
    {
        delayed[i] = plant_c;
    }
    CHECK(step(&state, 0, 1, 40 * 128));
    CHECK(sc_thermal_control_start(&state, (int32_t)(target_c * 65536.0), 0, state.update_ms));
    for (uint32_t seconds = 1; seconds <= 7200; seconds++)
    {
        plant_c += ((double)state.duty_permille / 1000.0 - loss_w_per_k * (plant_c - 40.0)) /
                   capacity_j_per_k;
        if (plant_c > peak_c)
        {
            peak_c = plant_c;
        }
        delayed[seconds % 32U] = plant_c;
        const double observed_c = delayed[(seconds + 32U - lag_s) % 32U];
        CHECK(step(&state, seconds * 1000U, seconds + 1U, (int16_t)(observed_c * 128.0 + 0.5)));
        CHECK(state.running && state.duty_permille <= 1000U);
    }
    if (reachable)
    {
        CHECK(plant_c > target_c - 0.05 && plant_c < target_c + 0.05);
        CHECK(peak_c < target_c + 0.30 && state.stable);
    }
    else
    {
        CHECK(!state.stable && state.duty_permille == 1000U && plant_c < target_c - 1.0);
    }
    printf(
        "model C=%.1f J/K G=%.2f W/K lag=%u s target=%.1f C final=%.4f C peak=%.4f C stable=%u\n",
        capacity_j_per_k, loss_w_per_k, lag_s, target_c, plant_c, peak_c, state.stable ? 1U : 0U);
    return 0;
}

static int test_pid_units_filter_and_samples(void)
{
    sc_thermal_control_t state;
    sc_thermal_control_init(&state);
    CHECK(step(&state, 0U, 1U, 25 * 128));
    /* Explicit test gains: Kd=0.2 duty*s/K, a 0.5 duty cap, tau=1 s. */
    CHECK(sc_thermal_control_derivative(&state, (sc_thermal_derivative_t){200000, 500000, 1000U}));
    CHECK(sc_thermal_control_gains(&state, (sc_thermal_gains_t){200000, 0, 1000U}));
    CHECK(sc_thermal_control_start(&state, 25L * SC_THERMAL_Q16, 500U, 0U));
    CHECK(!sc_thermal_control_derivative(&state, (sc_thermal_derivative_t){200000, 500000, 1000U}));

    /* A +1 K observation after 2 s has secant 0.5 K/s; the 1 s backward-
     * Euler filter produces 1/3 K/s, or D=-0.066665 duty. */
    CHECK(step(&state, 2000U, 2U, 26 * 128));
    CHECK(state.derivative_rate_q16_per_s == 21845);
    CHECK(state.derivative_micro == -66665);
    CHECK(state.duty_permille == 376U);

    /* Retuning Kp must include the existing D term in the bumpless integral. */
    const int32_t error = state.ramp_q16 - state.filtered_q16;
    const int32_t new_p = (int32_t)((int64_t)300000 * error / SC_THERMAL_Q16);
    const int32_t expected_integral =
        (int32_t)state.duty_permille * 1000 - new_p - state.derivative_micro;
    CHECK(sc_thermal_control_gains(&state, (sc_thermal_gains_t){300000, 0, 1000U}));
    CHECK(state.integral_micro == expected_integral && state.duty_permille == 376U);

    /* Changing only the setpoint adds no derivative kick: unchanged measured
     * temperature lets the existing filtered rate decay by tau/(tau+dt). */
    CHECK(sc_thermal_control_target(&state, 26L * SC_THERMAL_Q16));
    CHECK(step(&state, 3000U, 3U, 26 * 128));
    CHECK(state.derivative_rate_q16_per_s == 10922);
    CHECK(state.derivative_micro == -33331);

    /* Cooling reverses D's sign. */
    CHECK(step(&state, 5000U, 4U, 25 * 128));
    CHECK(state.derivative_rate_q16_per_s == -18204);
    CHECK(state.derivative_micro == 55554);

    /* An early fresh conversion is retried at cadence with its actual elapsed
     * interval, and a repeated cached sample has no second effect. */
    const sc_temperature_t early = sample(5500U, 5U, 26 * 128);
    const uint32_t updates_before = state.updates;
    CHECK(!sc_thermal_control_update(&state, &early, 5500U));
    CHECK(state.updates == updates_before && state.derivative_rate_q16_per_s == -18204 &&
          state.derivative_micro == 55554);
    CHECK(sc_thermal_control_update(&state, &early, 6000U));
    CHECK(state.derivative_rate_q16_per_s == 23666);
    CHECK(state.derivative_micro == -72222);
    const int32_t rate_before_cached = state.derivative_rate_q16_per_s;
    const int32_t derivative_before_cached = state.derivative_micro;
    CHECK(!sc_thermal_control_update(&state, &early, 6001U));
    CHECK(state.derivative_rate_q16_per_s == rate_before_cached &&
          state.derivative_micro == derivative_before_cached);
    return 0;
}

static int test_pid_cap_fault_resets_and_rollover(void)
{
    sc_thermal_control_t state;
    sc_thermal_control_init(&state);
    CHECK(step(&state, 0U, 1U, 25 * 128));
    CHECK(sc_thermal_control_derivative(&state, (sc_thermal_derivative_t){64000000, 125000, 500U}));
    CHECK(sc_thermal_control_gains(&state, (sc_thermal_gains_t){200000, 0, 1000U}));
    CHECK(sc_thermal_control_start(&state, 25L * SC_THERMAL_Q16, 500U, 0U));
    CHECK(step(&state, 1000U, 2U, 30 * 128));
    CHECK(state.derivative_micro == -125000);
    CHECK(step(&state, 2000U, 3U, 20 * 128));
    CHECK(state.derivative_micro == 125000);

    /* A stale interval stops output and clears derivative history. */
    CHECK(step(&state, 3000U, 4U, 20 * 128));
    CHECK(state.derivative_micro != 0);
    CHECK(step(&state, 5500U, 5U, 20 * 128)); /* elapsed = stale limit */
    CHECK(!state.running && state.duty_permille == 0U && state.integral_micro == 0 &&
          state.derivative_micro == 0 && state.derivative_rate_q16_per_s == 0);

    /* Invalid data likewise stops and clears D; configuration remains set. */
    CHECK(sc_thermal_control_start(&state, state.filtered_q16, 100U, 5500U));
    CHECK(step(&state, 6500U, 6U, 19 * 128));
    CHECK(state.derivative_micro != 0);
    sc_temperature_t invalid = sample(6501U, 7U, 19 * 128);
    invalid.valid = false;
    CHECK(!sc_thermal_control_update(&state, &invalid, 6501U));
    CHECK(!state.running && !state.filter_valid && state.duty_permille == 0U &&
          state.integral_micro == 0 && state.derivative_micro == 0 &&
          state.derivative_rate_q16_per_s == 0);
    CHECK(state.derivative.kd_micro == 64000000);

    /* Derivative filtering uses wrap-safe elapsed milliseconds. */
    sc_thermal_control_init(&state);
    const uint32_t near_wrap = UINT32_MAX - 999U;
    CHECK(step(&state, near_wrap, 1U, 25 * 128));
    CHECK(sc_thermal_control_derivative(&state, (sc_thermal_derivative_t){200000, 500000, 1000U}));
    CHECK(sc_thermal_control_gains(&state, (sc_thermal_gains_t){200000, 0, 1000U}));
    CHECK(sc_thermal_control_start(&state, 25L * SC_THERMAL_Q16, 500U, near_wrap));
    CHECK(step(&state, 0U, 2U, 26 * 128));
    CHECK(state.derivative_rate_q16_per_s == 32768 && state.derivative_micro == -100000);
    return 0;
}

int main(void)
{
    sc_thermal_control_t state;
    sc_thermal_control_init(&state);
    CHECK(state.gains.kp_micro == 1000000 && state.gains.ki_micro == 4000);
    CHECK(state.gains.max_permille == 1000U);
    CHECK(!sc_thermal_control_start(&state, 27L * 65536, 0, state.update_ms));
    CHECK(step(&state, 0, 1, 25 * 128));
    CHECK(state.filtered_q16 == 25L * 65536);
    CHECK(!step(&state, 500, 2, 26 * 128));
    CHECK(step(&state, 1000, 2, 26 * 128));
    CHECK(state.filtered_q16 == 25L * 65536 + 65536 / 6);
    const int32_t filtered_before = state.filtered_q16;
    CHECK(!step(&state, 2000, 2, 26 * 128)); /* Cached sequence never integrates. */
    CHECK(state.filtered_q16 == filtered_before);
    CHECK(step(&state, 2500, 3, 26 * 128));
    CHECK(state.filtered_q16 == filtered_before + (26L * 65536 - filtered_before) * 1500 / 6500);
    CHECK(!sc_thermal_control_start(&state, 60L * 65536 + 1, 0, state.update_ms));
    CHECK(!sc_thermal_control_start(&state, 27L * 65536, 1001, state.update_ms));
    CHECK(sc_thermal_control_start(&state, 27L * 65536, 1000, state.update_ms));
    CHECK(state.integral_micro == 1000000 && state.duty_permille == 1000);
    CHECK(sc_thermal_control_start(&state, 27L * 65536, 250, state.update_ms));
    CHECK(state.ramp_q16 == state.filtered_q16 && state.integral_micro == 250000);
    CHECK(state.duty_permille == 250);

    /* Fixed temperature: independently evaluate the first 1.5 s PI step. */
    sc_thermal_control_init(&state);
    CHECK(step(&state, 0, 1, 25 * 128));
    CHECK(sc_thermal_control_start(&state, 27L * 65536, 250, state.update_ms));
    CHECK(step(&state, 1500, 2, 25 * 128));
    CHECK(state.ramp_q16 == 25L * 65536 + 492);
    CHECK(state.integral_micro == 250045 && state.duty_permille == 258);
    const int32_t integral_before = state.integral_micro;
    CHECK(!step(&state, 2000, 2, 25 * 128));
    CHECK(state.integral_micro == integral_before);

    /* A command halfway between samples must not integrate pre-command time. */
    sc_thermal_control_init(&state);
    CHECK(step(&state, 0, 1, 25 * 128));
    CHECK(sc_thermal_control_start(&state, 27L * 65536, 250, 500));
    CHECK(step(&state, 1000, 2, 25 * 128));
    CHECK(state.ramp_q16 == 25L * 65536 + 164);
    CHECK(state.integral_micro == 250005);

    /* Output saturation cannot wind up, but opposite-sign error can unwind. */
    state.ramp_q16 = state.target_q16 = 30L * 65536;
    state.integral_micro = 0;
    CHECK(step(&state, 2500, 3, 25 * 128));
    CHECK(state.saturated && state.duty_permille == 1000 && state.integral_micro == 0);
    state.ramp_q16 = state.target_q16 = 24L * 65536;
    state.integral_micro = 2500000;
    CHECK(step(&state, 3500, 4, 25 * 128));
    CHECK(state.integral_micro == 2496000 && state.duty_permille == 1000);
    state.integral_micro = 0;
    CHECK(step(&state, 4500, 5, 25 * 128));
    CHECK(state.integral_micro == 0 && state.duty_permille == 0);

    /* Bumpless gain change must use the NEW proportional gain. */
    state.ramp_q16 = state.target_q16 = 26L * 65536;
    state.duty_permille = 400;
    CHECK(sc_thermal_control_gains(&state, (sc_thermal_gains_t){200000, 250, 800}));
    CHECK(state.integral_micro == 200000 && !state.stable && !state.slope_valid);
    CHECK(!sc_thermal_control_gains(&state, (sc_thermal_gains_t){-1, 250, 800}));
    CHECK(!sc_thermal_control_gains(&state, (sc_thermal_gains_t){200000, 250, 300}));

    /* Full 120-point regression window plus 300 s of entry conditions. */
    sc_thermal_control_init(&state);
    CHECK(step(&state, 0, 1, 25 * 128));
    CHECK(sc_thermal_control_start(&state, 25L * 65536, 250, state.update_ms));
    for (uint32_t i = 1; i <= 419; i++)
    {
        CHECK(step(&state, i * 1000, i + 1, 25 * 128));
    }
    CHECK(state.slope_valid && state.slope_q16_per_min == 0 && !state.stable);
    CHECK(step(&state, 420000, 421, 25 * 128));
    CHECK(state.stable && state.duty_permille == 250 && state.mean_permille == 250);
    for (uint32_t i = 421; i <= 426; i++)
    {
        CHECK(step(&state, i * 1000, i + 1, 25 * 128 + 32));
    }
    CHECK(!state.stable); /* Current error/slope clears the label; never latched. */
    for (uint32_t i = 427; i <= 445; i++)
    {
        CHECK(step(&state, i * 1000, i + 1, 25 * 128 + 32));
    }
    CHECK(!state.stable && state.running); /* Label loss must not disable regulation. */
    sc_temperature_t invalid = sample(446000, 447, 25 * 128);
    invalid.valid = false;
    CHECK(!sc_thermal_control_update(&state, &invalid, 446000));
    CHECK(!state.running && !state.stable && !state.filter_valid && state.duty_permille == 0);

    /* Positive/negative regression slopes, with filter settled, have correct
     * K/min units. One raw count per second is exactly 0.46875 K/min. */
    sc_thermal_control_init(&state);
    for (uint32_t i = 0; i < 600; i++)
    {
        CHECK(step(&state, i * 1000, i + 1, (int16_t)(3200 + i)));
    }
    CHECK(state.slope_valid && state.slope_q16_per_min >= 30700 &&
          state.slope_q16_per_min <= 30740);
    for (uint32_t i = 600; i < 1200; i++)
    {
        CHECK(step(&state, i * 1000, i + 1, (int16_t)(4400 - i)));
    }
    CHECK(state.slope_valid && state.slope_q16_per_min <= -30700 &&
          state.slope_q16_per_min >= -30740);
    CHECK(sc_thermal_control_start(&state, 28L * 65536, 250, state.update_ms));
    CHECK(step(&state, 1201500, 1201, 3200)); /* 2.5 s missed update is not caught up. */
    CHECK(!state.running && !state.stable && state.filtered_q16 == 25L * 65536);

    sc_thermal_control_init(&state);
    CHECK(step(&state, UINT32_MAX - 999U, 1, 25 * 128));
    CHECK(sc_thermal_control_start(&state, 27L * 65536, 0, state.update_ms));
    CHECK(step(&state, 0, 2, 25 * 128)); /* Monotonic uint32 rollover. */
    CHECK(state.running && state.ramp_q16 == 25L * 65536 + 328);

    /* Conversions arrive slightly early/late; the foreground retries each
     * early fresh sample without requiring a second conversion or integrating
     * its cached value twice. This is the board adapter's polling contract. */
    sc_thermal_control_init(&state);
    CHECK(step(&state, 0, 1, 25 * 128));
    sc_temperature_t early = sample(997, 2, 25 * 128 + 1);
    CHECK(!sc_thermal_control_update(&state, &early, 997));
    CHECK(state.sequence == 1 && state.updates == 1);
    CHECK(sc_thermal_control_update(&state, &early, 1000));
    CHECK(!sc_thermal_control_update(&state, &early, 1003));
    CHECK(step(&state, 2003, 3, 25 * 128 + 2));
    CHECK(state.updates == 3 && state.sequence == 3);

    sc_thermal_control_init(&state);
    CHECK(step(&state, 0, 1, 25 * 128));
    CHECK(sc_thermal_control_start(&state, 25L * 65536, 0, state.update_ms));
    state.integral_micro = 400; /* Rounds to zero duty: no false headroom/stability. */
    for (uint32_t i = 1; i < 600; i++)
    {
        CHECK(step(&state, i * 1000, i + 1, 25 * 128));
    }
    CHECK(!state.stable && state.window_saturated && state.duty_permille == 0);
    /* Worst sensor range and longest permitted cadence exercise fixed-point
     * regression products. No assumptions about realistic thermal slew here. */
    sc_thermal_control_init(&state);
    for (uint32_t i = 0; i < 240; i++)
    {
        CHECK(step(&state, i * 2499U, i + 1U, (int16_t)(i % 2U ? 150 * 128 : -55 * 128)));
    }
    CHECK(state.slope_valid);
    CHECK(model(48.6, 0.10, 0, 42.0, true) == 0);
    CHECK(model(48.6, 0.10, 15, 42.0, true) == 0);
    CHECK(model(97.2, 0.20, 15, 42.0, true) == 0);
    CHECK(model(48.6, 0.30, 15, 45.0, false) == 0);
    sc_thermal_control_init(&state);
    CHECK(step(&state, 0, 1, 39 * 128));
    CHECK(sc_thermal_control_start(&state, 39L * 65536, 350, 0));
    CHECK(step(&state, 1000, 2, 39 * 128));
    const int32_t before_integral = state.integral_micro;
    const int32_t before_filter = state.filtered_q16;
    const int32_t before_ramp = state.ramp_q16;
    CHECK(sc_thermal_control_target(&state, 39L * 65536 + 65));
    CHECK(state.integral_micro == before_integral && state.filtered_q16 == before_filter &&
          state.ramp_q16 == before_ramp && state.running);
    CHECK(!sc_thermal_control_target(&state, -1));
    CHECK(!sc_thermal_control_target(&state, 60L * 65536 + 1));
    CHECK(step(&state, 2000, 3, 39 * 128));
    CHECK(state.ramp_q16 == state.target_q16 && state.duty_permille >= 350);
    CHECK(test_pid_units_filter_and_samples() == 0);
    CHECK(test_pid_cap_fault_resets_and_rollover() == 0);
    return 0;
}
