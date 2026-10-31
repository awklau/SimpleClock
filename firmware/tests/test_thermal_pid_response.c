/* SPDX-License-Identifier: MIT */
#include "simpleclock/thermal_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Independent 1 W energy-balance model around the production fixed-point
 * controller. These are synthetic disturbances, not measured airflow or a
 * hardware validation. Delay is on observation, including the fitted transport
 * delay; the controller adds its real P/I and derivative filters separately. */
typedef struct
{
    double overshoot, rise, final_error, duty_span, step_error;
    unsigned cold_band_s, hot_band_s, cold_fine_s, hot_fine_s;
} result_t;

static int response(double capacity, double loss, unsigned delay, bool noise, bool pid,
                    sc_thermal_gains_t gains, sc_thermal_derivative_t derivative, result_t *r)
{
    sc_thermal_control_t c;
    sc_thermal_control_init(&c);
    if (pid &&
        (!sc_thermal_control_gains(&c, gains) || !sc_thermal_control_derivative(&c, derivative)))
    {
        return 1;
    }
    double temperature = 40, history[128], minimum_duty = 1, maximum_duty = 0;
    *r = (result_t){0};
    for (unsigned i = 0; i < 128; i++)
    {
        history[i] = temperature;
    }
    sc_temperature_t s = {.valid = true, .raw = 40 * 128, .sequence = 1};
    if (!sc_thermal_control_update(&c, &s, 0) ||
        !sc_thermal_control_start(&c, 41L * SC_THERMAL_Q16, 0, 0))
    {
        return 1;
    }
    for (unsigned t = 1; t <= 10000; t++)
    {
        /* Finite energy impulses (-/+0.5 K), then a sustained -1 K change
         * in the heater-off operating point (includes OCXO self-heating).
         * Physical temperature and delayed sensed temperature stay separate. */
        if (t == 2000)
        {
            temperature -= 0.5;
        }
        if (t == 4000)
        {
            temperature += 0.5;
        }
        const double off = t >= 6000 && t < 8000 ? 39 : 40;
        const double increment =
            ((double)c.duty_permille / 1000.0 - loss * (temperature - off)) / capacity;
        temperature += increment;
        const double error = temperature - 41;
        if (t < 2000)
        {
            if (error > r->overshoot)
            {
                r->overshoot = error;
            }
        }
        if (increment * 60 > r->rise)
        {
            r->rise = increment * 60;
        }
        if (t >= 2000 && t < 4000)
        {
            if (fabs(error) > 0.25)
            {
                r->cold_band_s = t - 2000 + 1;
            }
            if (fabs(error) > 0.05)
            {
                r->cold_fine_s = t - 2000 + 1;
            }
        }
        if (t >= 4000 && t < 6000)
        {
            if (fabs(error) > 0.25)
            {
                r->hot_band_s = t - 4000 + 1;
            }
            if (fabs(error) > 0.05)
            {
                r->hot_fine_s = t - 4000 + 1;
            }
        }
        if (t >= 6000 && fabs(error) > r->step_error)
        {
            r->step_error = fabs(error);
        }
        history[t % 128U] = temperature;
        const double observed = history[(t + 128U - delay) % 128U];
        /* One TMP117 code of alternating noise, in addition to quantization. */
        s = (sc_temperature_t){.valid = true,
                               .raw = (int16_t)(observed * 128 + 0.5),
                               .sequence = t + 1U,
                               .sample_ms = t * 1000U};
        if (noise)
        {
            /* Modeled temperatures stay near 40 C, far inside int16 bounds. */
            s.raw = (int16_t)((int32_t)s.raw + (t % 2U ? 1 : -1));
        }
        if (!sc_thermal_control_update(&c, &s, s.sample_ms) || !c.running ||
            c.duty_permille > 1000U)
        {
            return 1;
        }
        if (t >= 9500)
        {
            const double duty = (double)c.duty_permille / 1000;
            if (duty < minimum_duty)
            {
                minimum_duty = duty;
            }
            if (duty > maximum_duty)
            {
                maximum_duty = duty;
            }
        }
    }
    r->final_error = temperature - 41;
    r->duty_span = maximum_duty - minimum_duty;
    return 0;
}

int main(int argc, char **argv)
{
    sc_thermal_control_t defaults;
    sc_thermal_control_init(&defaults);
    if (!sc_thermal_control_pid_defaults(&defaults))
    {
        return 1;
    }
    /* Optional offline gain search uses this exact controller/model. No board
     * commands are sent; the no-argument CTest run checks the shipped defaults. */
    if (argc == 4)
    {
        defaults.gains.kp_micro = (int32_t)(atof(argv[1]) * 1000000);
        defaults.gains.ki_micro = (int32_t)(atof(argv[2]) * 1000000);
        defaults.derivative.kd_micro = (int32_t)(atof(argv[3]) * 1000000);
    }
    const double plants[][2] = {{71, .148}, {80.2, .1903}, {48.6, .10}, {97.2, .20}};
    const unsigned delays[] = {7, 15, 30};
    int failed = 0;
    for (unsigned p = 0; p < 4; p++)
    {
        for (unsigned d = 0; d < 3; d++)
        {
            for (unsigned noise = 0; noise < 2; noise++)
            {
                result_t r, old;
                if (response(plants[p][0], plants[p][1], delays[d], noise != 0, true,
                             defaults.gains, defaults.derivative, &r) ||
                    response(plants[p][0], plants[p][1], delays[d], noise != 0, false,
                             defaults.gains, defaults.derivative, &old))
                {
                    return 1;
                }
                printf("C=%.1f G=%.4f delay=%u noise=%u peak=%.4f rise=%.4f "
                       "cold_band=%u hot_band=%u cold_fine=%u hot_fine=%u step=%.4f "
                       "final=%.4f duty_span=%.4f PI_band=%u/%u PI_fine=%u/%u\n",
                       plants[p][0], plants[p][1], delays[d], noise, r.overshoot, r.rise,
                       r.cold_band_s, r.hot_band_s, r.cold_fine_s, r.hot_fine_s, r.step_error,
                       r.final_error, r.duty_span, old.cold_band_s, old.hot_band_s, old.cold_fine_s,
                       old.hot_fine_s);
                /* 30 s is a mismatch stress case, not the fitted delay. If
                 * that delay is observed on hardware, retuning is needed. */
                const bool stress = delays[d] == 30U;
                failed |= r.overshoot > (stress ? .30 : .15) || r.rise > 1.25 ||
                          fabs(r.final_error) > (stress ? .05 : .03) ||
                          r.step_error > (stress ? .30 : .15);
                if (!stress)
                {
                    failed |=
                        r.cold_fine_s >= old.cold_fine_s || r.hot_fine_s > old.hot_fine_s + 2U;
                }
            }
        }
    }
    return argc == 4 ? 0 : failed;
}
