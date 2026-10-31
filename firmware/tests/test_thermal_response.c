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

typedef struct
{
    double startup_peak_c, maximum_rise_c_per_min, cooling_dip_c, warming_peak_c;
    double final_error_c;
    unsigned cooling_settle_s, warming_settle_s;
} response_t;

static double absolute(double value)
{
    return value < 0 ? -value : value;
}

static int response(double capacity_j_per_k, double loss_w_per_k, unsigned delay_s,
                    sc_thermal_gains_t gains, response_t *result)
{
    /* Independent floating-point energy balance around the real fixed-point
     * controller: C*dT/dt = 1 W*duty - G*(T - heater-off operating point).
     * The latter includes OCXO self-heating; it is NOT measured room ambient.
     * Capacity/loss/delay cover uncertain mounting as well as the approximate
     * M4 fit. This is regression evidence, never a hardware stability claim. */
    sc_thermal_control_t control;
    sc_thermal_control_init(&control);
    CHECK(sc_thermal_control_gains(&control, gains));
    double temperature_c = 40.0, delayed_c[128];
    *result = (response_t){.startup_peak_c = 40.0, .cooling_dip_c = 42.0, .warming_peak_c = 42.0};
    for (unsigned i = 0; i < 128; i++)
    {
        delayed_c[i] = temperature_c;
    }
    sc_temperature_t sample = {.valid = true, .raw = 40 * 128, .sequence = 1};
    CHECK(sc_thermal_control_update(&control, &sample, 0));
    CHECK(sc_thermal_control_start(&control, 42L * SC_THERMAL_Q16, 0, 0));
    for (unsigned seconds = 1; seconds <= 9000; seconds++)
    {
        /* Cool the OFF operating point by 1 K, then restore it. Each interval
         * is 3000 s; no target or gain changes can mask disturbance response. */
        const double off_c = seconds >= 3000 && seconds < 6000 ? 39.0 : 40.0;
        const double increment_c =
            ((double)control.duty_permille / 1000.0 - loss_w_per_k * (temperature_c - off_c)) /
            capacity_j_per_k;
        temperature_c += increment_c;
        if (seconds < 3000)
        {
            if (temperature_c > result->startup_peak_c)
            {
                result->startup_peak_c = temperature_c;
            }
            if (increment_c * 60.0 > result->maximum_rise_c_per_min)
            {
                result->maximum_rise_c_per_min = increment_c * 60.0;
            }
        }
        else if (seconds < 6000)
        {
            if (temperature_c < result->cooling_dip_c)
            {
                result->cooling_dip_c = temperature_c;
            }
            if (absolute(temperature_c - 42.0) > 0.05)
            {
                result->cooling_settle_s = seconds - 3000;
            }
        }
        else
        {
            if (temperature_c > result->warming_peak_c)
            {
                result->warming_peak_c = temperature_c;
            }
            if (absolute(temperature_c - 42.0) > 0.05)
            {
                result->warming_settle_s = seconds - 6000;
            }
        }
        delayed_c[seconds % 128U] = temperature_c;
        sample = (sc_temperature_t){
            .valid = true,
            .raw = (int16_t)(delayed_c[(seconds + 128U - delay_s) % 128U] * 128.0 + 0.5),
            .sequence = seconds + 1U,
            .sample_ms = seconds * 1000U};
        CHECK(sc_thermal_control_update(&control, &sample, seconds * 1000U));
        CHECK(control.running && control.duty_permille <= 1000U);
    }
    result->final_error_c = temperature_c - 42.0;
    return 0;
}

int main(void)
{
    const double plants[][2] = {
        {71.0, 0.148}, {80.2, 0.1903}, {48.6, 0.10}, {97.2, 0.20}, {145.8, 0.25}};
    const unsigned delays_s[] = {7, 30, 60};
    const sc_thermal_gains_t previous = {300000, 500, 1000};
    sc_thermal_control_t defaults;
    sc_thermal_control_init(&defaults);
    for (unsigned plant = 0; plant < sizeof(plants) / sizeof(plants[0]); plant++)
    {
        for (unsigned delay = 0; delay < sizeof(delays_s) / sizeof(delays_s[0]); delay++)
        {
            response_t old, tuned;
            const bool long_delay_stress = delays_s[delay] == 60U;
            CHECK(response(plants[plant][0], plants[plant][1], delays_s[delay], previous, &old) ==
                  0);
            CHECK(response(plants[plant][0], plants[plant][1], delays_s[delay], defaults.gains,
                           &tuned) == 0);
            CHECK(absolute(old.final_error_c) < 0.05);
            CHECK(absolute(tuned.final_error_c) < 0.02);
            CHECK(tuned.startup_peak_c < (long_delay_stress ? 42.50 : 42.25));
            CHECK(tuned.maximum_rise_c_per_min < (long_delay_stress ? 0.70 : 0.50));
            CHECK(tuned.cooling_dip_c > 41.75 && tuned.warming_peak_c < 42.25);
            const double recovery_fraction = long_delay_stress ? 0.80 : 0.35;
            CHECK(tuned.cooling_settle_s < recovery_fraction * old.cooling_settle_s);
            CHECK(tuned.warming_settle_s < recovery_fraction * old.warming_settle_s);
            printf("C=%.1f J/K G=%.4f W/K delay=%u s: startup overshoot %.3f C, "
                   "rise %.3f C/min; +/-1 K recovery old=%u/%u s tuned=%u/%u s\n",
                   plants[plant][0], plants[plant][1], delays_s[delay], tuned.startup_peak_c - 42.0,
                   tuned.maximum_rise_c_per_min, old.cooling_settle_s, old.warming_settle_s,
                   tuned.cooling_settle_s, tuned.warming_settle_s);
        }
    }
    return 0;
}
