/* SPDX-License-Identifier: MIT */
/* Deterministic host model only.  Passing this test is controller behavior in
 * a synthetic plant, not a claim about measured clock or thermal performance. */
#include "simpleclock/thermal_control.h"
#include "simpleclock/thermal_outer.h"
#include "simpleclock/hybrid.h"

#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#define CHECK(e)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(e))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #e);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static int32_t q16(double celsius)
{
    return (int32_t)(celsius * (double)SC_THERMAL_Q16 + (celsius < 0.0 ? -0.5 : 0.5));
}

static int16_t raw128(double celsius)
{
    return (int16_t)(celsius * 128.0 + (celsius < 0.0 ? -0.5 : 0.5));
}

typedef struct
{
    double temperature_c;
    double ambient_c;
    double full_on_power_w;
    double loss_w_per_k;
    double capacity_j_per_k;
} plant_t;

#define SENSOR_DELAY_S 8U

static void plant_step(plant_t *plant, uint16_t duty_permille)
{
    const double heater_w = plant->full_on_power_w * (double)duty_permille / 1000.0;
    const double loss_w = plant->loss_w_per_k * (plant->temperature_c - plant->ambient_c);
    plant->temperature_c += (heater_w - loss_w) / plant->capacity_j_per_k;
}

static int combined_model(void)
{
    sc_thermal_outer_config_t outer_config;
    sc_thermal_outer_default_config(&outer_config);
    outer_config.initial_target_q16 = q16(39.5);
    sc_thermal_outer_state_t outer;
    CHECK(sc_thermal_outer_init(&outer, &outer_config));

    sc_thermal_control_t inner;
    sc_thermal_control_init(&inner);
    CHECK(sc_thermal_control_pid_defaults(&inner));
    plant_t plant = {39.5, 36.0, 1.2, 0.2, 50.0}; /* tau=250 s, Gtheta=6 K. */
    double delayed_temperature[SENSOR_DELAY_S + 1U];
    for (uint32_t i = 0U; i <= SENSOR_DELAY_S; i++)
    {
        delayed_temperature[i] = plant.temperature_c;
    }
    sc_temperature_t first = {
        .valid = true, .raw = raw128(plant.temperature_c), .sequence = 1U, .sample_ms = 0U};
    CHECK(sc_thermal_control_update(&inner, &first, 0U));
    CHECK(sc_thermal_control_start(&inner, outer_config.initial_target_q16, 583U, 0U));
    uint16_t applied_duty = 583U;
    int32_t previous_target = outer.target_q16;
    bool saw_outer_start = false, saw_target_rise = false, saw_target_fall = false;
    bool saw_error_pause = false, saw_error_recovery = false;
    bool saw_qualification_motion = false, saw_acquisition_motion = false;
    int32_t peak_target = outer.target_q16;

    for (uint32_t second = 1U; second <= 2400U; second++)
    {
        if (second >= 500U)
        {
            /* Slow synthetic ambient drift.  The plant's full-on gain is
             * 6 K/unit-duty while the configured controller estimate is 10. */
            plant.ambient_c = 36.0 + 0.0008 * (double)(second - 500U);
        }
        if (second == 1250U)
        {
            plant.temperature_c += 0.5; /* force the specified tracking pause */
        }
        plant_step(&plant, applied_duty);
        delayed_temperature[second % (SENSOR_DELAY_S + 1U)] = plant.temperature_c;
        const double measured_c = delayed_temperature[(second + 1U) % (SENSOR_DELAY_S + 1U)];
        const uint32_t now_ms = second * 1000U;
        sc_temperature_t sample = {
            .valid = true, .raw = raw128(measured_c), .sequence = second + 1U, .sample_ms = now_ms};
        CHECK(sc_thermal_control_update(&inner, &sample, now_ms));
        const int32_t before = outer.target_q16;
        /* Centering permission is thermal-only; this plant has no GNSS
         * receiver and clock state is not an input. */
        sc_thermal_outer_input_t input = {
            .raw_temperature_q16 = q16(measured_c),
            .filtered_temperature_q16 = inner.filtered_q16,
            .applied_duty_permille = applied_duty,
            .actuator_saturated = inner.saturated,
            .applied_duty_valid = true,
            .sensor_valid = true,
            .centering_permitted = inner.running,
        };
        int32_t target;
        CHECK(sc_thermal_outer_update(&outer, now_ms, &input, &target));
        if (target != inner.target_q16)
        {
            CHECK(sc_thermal_control_target(&inner, target));
        }
        applied_duty = inner.duty_permille;
        CHECK(target >= outer_config.target_min_q16 && target <= outer_config.target_max_q16);
        saw_outer_start |= outer.centering_started;
        saw_target_rise |= target > previous_target;
        saw_target_fall |= target < previous_target;
        saw_error_pause |= outer.error_paused;
        if (saw_error_pause && !outer.error_paused)
        {
            saw_error_recovery = true;
        }
        if (target > peak_target)
        {
            peak_target = target;
        }
        if (second >= 500U && second < 900U && target != before)
        {
            saw_qualification_motion = true;
        }
        if (second >= 900U && second < 1000U && target != before)
        {
            saw_acquisition_motion = true;
        }
        previous_target = target;
    }
    CHECK(saw_outer_start && saw_target_rise && saw_target_fall);
    CHECK(saw_error_pause && saw_error_recovery);
    CHECK(saw_qualification_motion && saw_acquisition_motion);
    CHECK(peak_target <= outer_config.target_max_q16);
    printf("synthetic combined model: outer_started=%u target_rise=%u target_fall=%u "
           "error_pause=%u recovery=%u final_target=%.4f C final_temp=%.4f C\n",
           saw_outer_start ? 1U : 0U, saw_target_rise ? 1U : 0U, saw_target_fall ? 1U : 0U,
           saw_error_pause ? 1U : 0U, saw_error_recovery ? 1U : 0U,
           (double)outer.target_q16 / SC_THERMAL_Q16, plant.temperature_c);

    /* A long local-time gap must not catch up outer motion.  The real inner
     * controller also rejects this stale sample and stops its output. */
    const int32_t held_target = outer.target_q16;
    const uint32_t gap_ms = 3000000U;
    sc_temperature_t stale = {
        .valid = true, .raw = raw128(plant.temperature_c), .sequence = 1002U, .sample_ms = gap_ms};
    CHECK(sc_thermal_control_update(&inner, &stale, gap_ms));
    CHECK(!inner.running); /* accepted fresh sample resets, but does not restart, the PI */
    sc_thermal_outer_input_t gap_input = {
        q16(plant.temperature_c), q16(plant.temperature_c), applied_duty, false, true, true, true};
    CHECK(sc_thermal_outer_update(&outer, gap_ms, &gap_input, NULL));
    CHECK(outer.target_q16 == held_target && !outer.centering_started);
    return 0;
}

static int replay_file(const char *path)
{
    FILE *file = fopen(path, "r");
    CHECK(file != NULL);
    char line[1024];
    CHECK(fgets(line, sizeof(line), file) != NULL); /* header */

    sc_thermal_outer_config_t config;
    sc_thermal_outer_default_config(&config);
    sc_thermal_outer_state_t outer;
    CHECK(sc_thermal_outer_init(&outer, &config));
    sc_hybrid_stats_t stats;
    sc_hybrid_stats_reset(&stats);
    sc_hybrid_t hybrid;
    sc_hybrid_init(&hybrid);
    sc_hybrid_observed(&hybrid, true, 0, 32U, 0U);
    uint32_t samples = 0U, partial_windows = 0U, stats_windows = 0U;
    uint32_t ppp_gst_has = 0U, fresh_samples = 0U, temp_instant_good = 0U;
    uint32_t continuity_breaks = 0U;
    uint8_t quality_history[SC_HYBRID_STATS_CAPACITY] = {0U};
    uint16_t quality_count = 0U, quality_next = 0U;
    uint32_t previous_tow = 0U, previous_now = 0U;
    unsigned previous_week = 0U;
    bool previous_valid = false;
    double base_bias = 0.0, previous_bias = 0.0, previous_drift = 0.0, residual = 0.0;
    double first_elapsed = 0.0;
    double elapsed, raw, filtered, recorded_target, duty, bias, drift, shift, correction_age;
    char host_time[64], gnss[16], time_system[16];
    int has_corrections, thermal_stable;
    unsigned receiver_week, tow_ms, segment, temperature_age_ms, pi_age_ms, satellites, jumps;
    while (fgets(line, sizeof(line), file))
    {
        const int fields = sscanf(
            line,
            "%lf,%63[^,],%lf,%lf,%lf,%lf,%lf,%lf,%lf,%15[^,],%15[^,],%d,%d,%u,%u,%u,%u,%u,%lf,%u,%"
            "u",
            &elapsed, host_time, &raw, &filtered, &recorded_target, &duty, &bias, &drift, &shift,
            gnss, time_system, &has_corrections, &thermal_stable, &receiver_week, &tow_ms, &segment,
            &temperature_age_ms, &pi_age_ms, &correction_age, &satellites, &jumps);
        CHECK(fields == 21);
        const uint32_t now_ms = (uint32_t)(elapsed * 1000.0 + 0.5);
        (void)filtered;
        (void)shift;
        (void)thermal_stable;
        (void)segment;
        (void)jumps;
        (void)satellites; /* Recorded metadata; no minimum-count steering gate. */
        const double temperature_error = raw - recorded_target;
        const bool sample_fresh = temperature_age_ms <= 2500U && pi_age_ms <= 2500U;
        const bool receiver_coarse_valid = strcmp(gnss, "PPP") == 0 &&
                                           strcmp(time_system, "GST") == 0 &&
                                           has_corrections != 0 && correction_age <= 30.0;
        ppp_gst_has += receiver_coarse_valid ? 1U : 0U;
        fresh_samples += sample_fresh ? 1U : 0U;
        if (sample_fresh && fabs(temperature_error) <= 0.075 && duty >= 2.0 && duty <= 98.0)
        {
            temp_instant_good++;
        }
        const uint64_t epoch_ms = (uint64_t)receiver_week * UINT64_C(604800000) + tow_ms;
        bool contiguous = true;
        if (previous_valid)
        {
            const uint32_t receipt_dt = now_ms - previous_now;
            contiguous = (receiver_week == previous_week && tow_ms == previous_tow + 1000U) &&
                         receipt_dt <= 2500U;
            if (!contiguous)
            {
                sc_hybrid_stats_reset(&stats);
                quality_count = 0U;
                quality_next = 0U;
                residual = 0.0;
                base_bias = bias;
                continuity_breaks++;
            }
            else
            {
                const double dt_s = (double)receipt_dt / 1000.0;
                residual += bias - previous_bias - 0.5 * (drift + previous_drift) * dt_s;
            }
        }
        else
        {
            base_bias = bias;
            first_elapsed = elapsed;
        }
        previous_week = receiver_week;
        previous_tow = tow_ms;
        previous_now = now_ms;
        previous_bias = bias;
        previous_drift = drift;
        previous_valid = true;
        CHECK(sc_hybrid_stats_push(&stats, drift, bias - base_bias, residual));

        sc_thermal_outer_input_t input = {
            q16(raw), q16(filtered), (uint16_t)(duty * 10.0 + 0.5), false, true, true, false};
        CHECK(sc_thermal_outer_update(&outer, now_ms, &input, NULL));
        const double mean_duty = (double)outer.mean_duty_permille / 1000.0;
        const bool thermal_good = sample_fresh && fabs(temperature_error) <= 0.075 &&
                                  outer.window_valid && mean_duty >= 0.02 && mean_duty <= 0.98;
        quality_history[quality_next] =
            (uint8_t)((receiver_coarse_valid ? 1U : 0U) | (sample_fresh ? 2U : 0U) |
                      (thermal_good ? 4U : 0U));
        quality_next = (uint16_t)((quality_next + 1U) % SC_HYBRID_STATS_CAPACITY);
        if (quality_count < SC_HYBRID_STATS_CAPACITY)
        {
            quality_count++;
        }
        sc_hybrid_input_t hybrid_input = {
            .epoch_ms = epoch_ms,
            .now_ms = now_ms,
            .received_ms = now_ms,
            .bias_ns = bias,
            .drift_ppb = drift,
            .gnss_valid = false, /* RFStatus/AlertFlag is absent; fail closed. */
            .coordinate_changed = false,
        };
        sc_hybrid_sample(&hybrid, &hybrid_input);
        CHECK(!sc_hybrid_tracking(&hybrid));
        int64_t unexpected_word;
        CHECK(!sc_hybrid_take_command(&hybrid, now_ms, &unexpected_word));

        sc_hybrid_metrics_t metrics;
        if (sc_hybrid_stats_get(&stats, 121U, &metrics))
        {
            stats_windows++;
            const bool measurement_window_good =
                metrics.drift_sd_ppb <= SC_HYBRID_CLOCK_QUALIFIED_DRIFT_STDDEV_LIMIT_PPB &&
                fabs(metrics.residual_slope_ppb) <=
                    SC_HYBRID_CLOCK_QUALIFIED_BIAS_DRIFT_DISAGREEMENT_LIMIT_PPB &&
                metrics.residual_detrended_rms_ns <=
                    SC_HYBRID_CLOCK_QUALIFIED_BIAS_MINUS_INTEGRATED_DRIFT_DETRENDED_RMS_LIMIT_NS;
            const bool dwell_good = elapsed - first_elapsed > 120.0;
            bool partial_quality_window_good = quality_count == SC_HYBRID_STATS_CAPACITY;
            for (uint16_t i = 0U; i < quality_count; i++)
            {
                partial_quality_window_good &= quality_history[i] == 7U;
            }
            if (measurement_window_good && dwell_good && partial_quality_window_good)
            {
                partial_windows++;
            }
        }
        samples++;
    }
    fclose(file);
    CHECK(samples == 900U);
    printf("telemetry replay %s: rows=%" PRIu32 " contiguous_breaks=%" PRIu32
           " PPP/GST/HAS/correction_age=%" PRIu32 " fresh=%" PRIu32 " "
           "instant_thermal_good=%" PRIu32 " full121_windows=%" PRIu32
           " measurable_partial_windows=%" PRIu32 " "
           "RFStatus=missing AlertFlag=missing SBF_authenticity=unavailable "
           "hybrid_actuator_writes=0; open-loop telemetry only\n",
           path, samples, continuity_breaks, ppp_gst_has, fresh_samples, temp_instant_good,
           stats_windows, partial_windows);
    (void)outer; /* Duty statistics only; healthy tracking is not certifiable here. */
    return 0;
}

typedef struct
{
    double max_error_k, max_target_rate_k_per_s;
    uint32_t saturated_s;
} ramp_result_t;

/* Worst-case day-night ambient: 10 K/h down for 1 h, hold 1 h, 10 K/h up for
 * 1 h, hold 1 h, through the real inner PID and outer loop. The outer target
 * must follow ambient within the 6 K heater authority of this plant. */
static int ambient_ramp(double slew_k_per_s, double authority_k, ramp_result_t *result)
{
    sc_thermal_outer_config_t config;
    sc_thermal_outer_default_config(&config);
    config.setpoint_slew_q16_per_s = q16(slew_k_per_s);
    config.initial_target_q16 = q16(39.0);
    sc_thermal_outer_state_t outer;
    CHECK(sc_thermal_outer_init(&outer, &config));
    sc_thermal_control_t inner;
    sc_thermal_control_init(&inner);
    CHECK(sc_thermal_control_pid_defaults(&inner));
    /* Full-on heater authority = power / loss (measured 5.5-6.5 K across units). */
    plant_t plant = {36.0 + 0.5 * authority_k, 36.0, 1.2, 1.2 / authority_k, 50.0};
    double delayed[SENSOR_DELAY_S + 1U];
    for (uint32_t i = 0U; i <= SENSOR_DELAY_S; i++)
    {
        delayed[i] = plant.temperature_c;
    }
    sc_temperature_t first = {
        .valid = true, .raw = raw128(plant.temperature_c), .sequence = 1U, .sample_ms = 0U};
    CHECK(sc_thermal_control_update(&inner, &first, 0U));
    CHECK(sc_thermal_control_start(&inner, config.initial_target_q16, 500U, 0U));
    uint16_t duty = 500U;
    *result = (ramp_result_t){0};
    const uint32_t settle_s = 3000U, phase_s = 3600U;
    int32_t previous_target = outer.target_q16;
    for (uint32_t second = 1U; second <= settle_s + 4U * phase_s; second++)
    {
        const uint32_t t = second > settle_s ? second - settle_s : 0U;
        const double ramp_k_per_s = 10.0 / 3600.0;
        plant.ambient_c =
            36.0 - (t < phase_s        ? ramp_k_per_s * (double)t
                    : t < 2U * phase_s ? 10.0
                    : t < 3U * phase_s ? 10.0 - ramp_k_per_s * (double)(t - 2U * phase_s)
                                       : 0.0);
        plant_step(&plant, duty);
        delayed[second % (SENSOR_DELAY_S + 1U)] = plant.temperature_c;
        const double measured = delayed[(second + 1U) % (SENSOR_DELAY_S + 1U)];
        const uint32_t now_ms = second * 1000U;
        sc_temperature_t sample = {
            .valid = true, .raw = raw128(measured), .sequence = second + 1U, .sample_ms = now_ms};
        CHECK(sc_thermal_control_update(&inner, &sample, now_ms));
        sc_thermal_outer_input_t input = {
            .raw_temperature_q16 = q16(measured),
            .filtered_temperature_q16 = inner.filtered_q16,
            .applied_duty_permille = duty,
            .actuator_saturated = inner.saturated,
            .applied_duty_valid = true,
            .sensor_valid = true,
            .centering_permitted = inner.running,
        };
        int32_t target;
        CHECK(sc_thermal_outer_update(&outer, now_ms, &input, &target));
        if (target != inner.target_q16)
        {
            CHECK(sc_thermal_control_target(&inner, target));
        }
        duty = inner.duty_permille;
        if (second > settle_s)
        {
            const double error = plant.temperature_c - (double)inner.target_q16 / SC_THERMAL_Q16;
            result->max_error_k = fmax(result->max_error_k, fabs(error));
            result->max_target_rate_k_per_s =
                fmax(result->max_target_rate_k_per_s,
                     fabs((double)(target - previous_target)) / SC_THERMAL_Q16);
            result->saturated_s += duty <= 20U || duty >= 980U ? 1U : 0U;
        }
        previous_target = target;
    }
    return 0;
}

static int day_night_ramps(void)
{
    /* 0.04 ppb/C oscillator sensitivity converts target motion to frequency. */
    const double caps[] = {0.001, 0.003, 0.004, 0.005};
    ramp_result_t results[4];
    for (unsigned i = 0U; i < 4U; i++)
    {
        CHECK(ambient_ramp(caps[i], 6.0, &results[i]) == 0);
        printf("10 K/h ambient, outer cap %.3f K/s: max error %.3f K, saturated %" PRIu32
               " s, peak target rate %.4f K/s = %.1e ppb/s\n",
               caps[i], results[i].max_error_k, results[i].saturated_s,
               results[i].max_target_rate_k_per_s, results[i].max_target_rate_k_per_s * 0.04);
    }
    CHECK(results[0].saturated_s > 0U); /* The 3.6 K/h cap cannot follow 10 K/h. */
    sc_thermal_outer_config_t defaults;
    sc_thermal_outer_default_config(&defaults);
    /* The compiled default holds 10 K/h across the measured unit spread. */
    const double authorities[] = {5.5, 6.0, 6.5};
    for (unsigned i = 0U; i < 3U; i++)
    {
        ramp_result_t compiled;
        CHECK(ambient_ramp((double)defaults.setpoint_slew_q16_per_s / SC_THERMAL_Q16,
                           authorities[i], &compiled) == 0);
        printf("default cap, %.1f K authority: max error %.3f K, saturated %" PRIu32 " s\n",
               authorities[i], compiled.max_error_k, compiled.saturated_s);
        CHECK(compiled.saturated_s == 0U && compiled.max_error_k < 0.1);
    }
    return 0;
}

int main(int argc, char **argv)
{
    CHECK(combined_model() == 0);
    CHECK(day_night_ramps() == 0);
    if (argc > 1)
    {
        CHECK(replay_file(argv[1]) == 0);
    }
    return 0;
}
