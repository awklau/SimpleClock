/* SPDX-License-Identifier: MIT */
#include "simpleclock/thermal_boot.h"
#include "simpleclock/thermal_control.h"
#include <math.h>
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

static int exact_threshold(uint32_t origin)
{
    sc_thermal_boot_t s;
    sc_thermal_boot_start(&s, origin);
    CHECK(!sc_thermal_boot_sample(&s, origin - 1U, 25 * SC_THERMAL_Q16));
    int32_t t = 25 * SC_THERMAL_Q16;
    for (unsigned i = 0; i <= 90U; i++)
    {
        t = 25 * SC_THERMAL_Q16 + (int32_t)i * 1000;
        CHECK(!sc_thermal_boot_sample(&s, origin + i * 1000U, t));
        CHECK(!s.reference_valid || i == 90U);
        CHECK(i >= 60U || s.count == 0U);
    }
    CHECK(s.reference_valid && s.reference_q16_per_s == 1000);
    _Static_assert(SC_THERMAL_BOOT_CAPTURE_SLOPE_Q16_PER_S == 436, "0.4 C/min, floored");
    CHECK(!sc_thermal_boot_sample(&s, origin + 90000U, t));
    CHECK(s.below_count == 0U);
    for (unsigned i = 91U; i <= 130U; i++)
    {
        t += 437; /* One Q16 C/s count above the absolute boundary must fail. */
        CHECK(!sc_thermal_boot_sample(&s, origin + i * 1000U, t));
    }
    CHECK(s.slope_q16_per_s == 437 && s.below_count == 0U);
    bool captured = false;
    for (unsigned i = 131U; i <= 200U; i++)
    {
        t += 436;
        captured = sc_thermal_boot_sample(&s, origin + i * 1000U, t);
        CHECK(!captured || s.below_elapsed_ms >= 20000U);
        CHECK(captured || s.below_elapsed_ms < 20000U);
        if (!captured && s.below_timing)
        {
            const uint32_t elapsed = s.below_elapsed_ms;
            CHECK(!sc_thermal_boot_sample(&s, origin + i * 1000U, t));
            CHECK(s.below_elapsed_ms == elapsed);
        }
        if (captured)
        {
            break;
        }
    }
    CHECK(captured && s.phase == SC_THERMAL_BOOT_CAPTURED);
    CHECK(s.target_q16 == t && s.below_count == 21U && s.below_elapsed_ms == 20000U);
    return 0;
}

static int confirmation_reset(void)
{
    sc_thermal_boot_t s;
    sc_thermal_boot_start(&s, UINT32_MAX - 170000U);
    int32_t t = 25 * SC_THERMAL_Q16;
    unsigned second = 0;
    for (; second <= 90U; second++)
    {
        t += 1000;
        CHECK(!sc_thermal_boot_sample(&s, s.start_ms + second * 1000U, t));
    }
    for (; second < 130U && !s.below_timing; second++)
    {
        t += 200;
        CHECK(!sc_thermal_boot_sample(&s, s.start_ms + second * 1000U, t));
    }
    CHECK(s.below_timing);
    for (unsigned i = 0; i < 5U; i++, second++)
    {
        t += 200;
        CHECK(!sc_thermal_boot_sample(&s, s.start_ms + second * 1000U, t));
    }
    CHECK(s.below_elapsed_ms == 5000U);
    for (unsigned i = 0; i < 40U; i++, second++)
    {
        t += 4000; /* A renewed rise must discard the partial confirmation. */
        CHECK(!sc_thermal_boot_sample(&s, s.start_ms + second * 1000U, t));
    }
    CHECK(!s.below_timing && s.below_elapsed_ms == 0U && s.below_count == 0U);
    bool captured = false;
    for (unsigned i = 0; i < 100U; i++, second++)
    {
        t += 200;
        captured = sc_thermal_boot_sample(&s, s.start_ms + second * 1000U, t);
        CHECK(!captured || s.below_elapsed_ms == 20000U);
        if (captured)
        {
            break;
        }
    }
    CHECK(captured);
    return 0;
}

static int warm_start_fixed_slope(void)
{
    sc_thermal_boot_t state;
    sc_thermal_boot_start(&state, 0U);
    /* A constant positive slope below the fixed limit needs no fractional
     * decay. Old percentage logic would boost indefinitely on this input. */
    for (unsigned second = 0; second <= 110U; ++second)
    {
        const bool captured = sc_thermal_boot_sample(&state, second * 1000U,
                                                     40L * SC_THERMAL_Q16 + (int32_t)second * 200);
        CHECK(captured == (second == 110U));
    }
    CHECK(state.phase == SC_THERMAL_BOOT_CAPTURED && state.reference_q16_per_s == 200);
    CHECK(state.below_elapsed_ms == 20000U);
    return 0;
}

static int fallbacks(void)
{
    sc_thermal_boot_t s;
    for (int kind = 0; kind < 3; kind++)
    {
        sc_thermal_boot_start(&s, 0);
        bool ready = false;
        for (unsigned i = 0; i <= 90U; i++)
        {
            const int32_t perturbation =
                kind == 1 ? -(int32_t)i * 512 : (kind == 2 ? (i % 2U ? 512 : -512) : 0);
            ready = sc_thermal_boot_sample(&s, i * 1000U, 40 * SC_THERMAL_Q16 + perturbation);
            CHECK(ready == (i == 90U));
        }
        CHECK(s.phase == SC_THERMAL_BOOT_NO_RISE && !s.reference_valid);
    }
    sc_thermal_boot_start(&s, 0);
    CHECK(sc_thermal_boot_sample(&s, 0, 61 * SC_THERMAL_Q16));
    CHECK(s.phase == SC_THERMAL_BOOT_CEILING && s.target_q16 == 60 * SC_THERMAL_Q16);
    sc_thermal_boot_start(&s, 0);
    CHECK(!sc_thermal_boot_sample(&s, 0, 25 * SC_THERMAL_Q16));
    CHECK(!sc_thermal_boot_sample(&s, 2500, 25 * SC_THERMAL_Q16));
    CHECK(s.phase == SC_THERMAL_BOOT_BAD_DATA);
    sc_thermal_boot_start(&s, 0);
    for (unsigned i = 0; i <= 30U; i++)
    {
        CHECK(!sc_thermal_boot_sample(&s, 60000U + i * 1100U,
                                      25 * SC_THERMAL_Q16 + (int32_t)i * 512));
    }
    CHECK(s.phase == SC_THERMAL_BOOT_BAD_DATA); /* Missed reference cadence. */
    sc_thermal_boot_start(&s, 0);
    CHECK(!sc_thermal_boot_sample(&s, 0, 151 * SC_THERMAL_Q16));
    CHECK(s.phase == SC_THERMAL_BOOT_BAD_DATA);
    sc_thermal_boot_start(&s, 0);
    for (unsigned i = 0; i <= 90U; i++)
    {
        (void)sc_thermal_boot_sample(&s, i * 1000U, -2 * SC_THERMAL_Q16);
    }
    CHECK(s.phase == SC_THERMAL_BOOT_NO_RISE && s.target_q16 == 0);
    return 0;
}

static int early_transient(void)
{
    /* A fast local warm-up superimposed on a slower block response. These
     * amplitudes/time constants are synthetic stress inputs, not a plant fit.
     * An early 10-20 s reference is sensitive to this first transient; the
     * later reference should select broadly the same point on the slow rise. */
    unsigned first_capture = 0;
    for (unsigned trial = 0; trial < 3U; trial++)
    {
        sc_thermal_boot_t s;
        sc_thermal_boot_start(&s, 0);
        unsigned capture_s = 0;
        for (unsigned second = 0; second <= 450U; second++)
        {
            const double fast = (double)trial * 0.6 * (1.0 - exp(-(double)second / 8.0));
            const double slow =
                second <= 7U ? 0.0 : 6.75 * (1.0 - exp(-(double)(second - 7U) / 450.0));
            const int32_t raw_q16 = (int32_t)lround((25.0 + fast + slow) * 128.0) * 512;
            if (sc_thermal_boot_sample(&s, second * 1000U, raw_q16))
            {
                capture_s = second;
                break;
            }
        }
        CHECK(s.phase == SC_THERMAL_BOOT_CAPTURED && capture_s >= 300U && capture_s < 450U);
        if (trial == 0U)
        {
            first_capture = capture_s;
        }
        CHECK(capture_s + 30U >= first_capture && capture_s <= first_capture + 30U);
        printf("synthetic local transient amplitude=%.1f C: capture=%u s\n", (double)trial * 0.6,
               capture_s);
    }
    return 0;
}

static int plant(double capacitance, double conductance, unsigned delay_s, double initial)
{
    sc_thermal_boot_t boot;
    sc_thermal_boot_start(&boot, 0);
    sc_thermal_control_t pid;
    sc_thermal_control_init(&pid);
    CHECK(sc_thermal_control_pid_defaults(&pid));
    double commands[32] = {0};
    double t = initial, target = 0, peak = -100;
    unsigned handoff = 0;
    for (unsigned second = 0; second <= 1500U; second++)
    {
        const int16_t raw = (int16_t)lround(t * 128.0);
        const uint32_t ms = second * 1000U;
        const sc_temperature_t sample = {
            .raw = raw, .valid = true, .sample_ms = ms, .sequence = second + 1U};
        (void)sc_thermal_control_update(&pid, &sample, ms);
        double duty = 1.0;
        if (!handoff && sc_thermal_boot_sample(&boot, ms, (int32_t)raw * 512))
        {
            handoff = second;
            target = (double)boot.target_q16 / 65536.0;
            CHECK(sc_thermal_control_start(&pid, boot.target_q16, 0, ms));
            CHECK(pid.integral_micro == 0 && pid.duty_permille == 0);
        }
        if (handoff)
        {
            duty = (double)pid.duty_permille / 1000.0;
            if (t > peak)
            {
                peak = t;
            }
        }
        commands[second % 32U] = duty;
        const double delivered = second >= delay_s ? commands[(second - delay_s) % 32U] : 0;
        /* Independent one-node synthetic plant; 1 W at full duty, T_off=30 C.
         * This tests behavior/model limits, not a measured startup duration. */
        t += (delivered - conductance * (t - 30.0)) / capacitance;
    }
    CHECK(handoff >= 110U && handoff < 900U);
    CHECK(isfinite(t) && peak - target < 0.5);
    CHECK(fabs(t - target) < 0.15);
    printf("boot model C=%.1f G=%.4f delay=%u initial=%.1f capture=%us target=%.3f peak_error=%.3f "
           "final_error=%.3f\n",
           capacitance, conductance, delay_s, initial, handoff, target, peak - target, t - target);
    return 0;
}

int main(void)
{
    CHECK(exact_threshold(0) == 0);
    CHECK(exact_threshold(UINT32_MAX - 15000U) == 0);
    CHECK(exact_threshold(UINT32_MAX - 170000U) == 0); /* Wrap during confirmation. */
    CHECK(confirmation_reset() == 0);
    CHECK(warm_start_fixed_slope() == 0);
    CHECK(fallbacks() == 0);
    CHECK(early_transient() == 0);
    CHECK(plant(71, .148, 7, 30) == 0);
    CHECK(plant(71, .148, 10, 30) == 0);
    CHECK(plant(80.2, .1903, 7, 30) == 0);
    CHECK(plant(71, .148, 15, 30) == 0);
    CHECK(plant(71, .148, 7, 33) == 0);
    puts("Full-power startup slope selection, fallbacks and synthetic handoff passed");
    return 0;
}
