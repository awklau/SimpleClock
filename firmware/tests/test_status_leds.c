/* SPDX-License-Identifier: MIT */
#include "simpleclock/status_leds.h"
#include <limits.h>
#include <stdio.h>
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

static sc_thermal_control_t settled(void)
{
    sc_thermal_control_t c = {0};
    c.running = c.filter_valid = c.slope_valid = true;
    c.target_q16 = c.ramp_q16 = 40 * SC_THERMAL_Q16;
    c.filtered_q16 = c.target_q16 - SC_LED_THERMAL_ERROR_Q16;
    return c;
}

int main(void)
{
    sc_led_modes_t modes = sc_led_modes(false, false, false, false, false, false);
    CHECK(modes.system == SC_LED_BLINK && modes.gnss == SC_LED_OFF && modes.thermal == SC_LED_OFF);
    modes = sc_led_modes(false, false, true, false, true, false);
    CHECK(modes.gnss == SC_LED_BLINK && modes.thermal == SC_LED_BLINK);
    modes = sc_led_modes(false, false, true, true, true, true);
    CHECK(modes.system == SC_LED_BLINK && modes.gnss == SC_LED_SOLID &&
          modes.thermal == SC_LED_SOLID);
    modes = sc_led_modes(false, true, true, true, true, true);
    CHECK(modes.system == SC_LED_SOLID);
    modes = sc_led_modes(true, true, true, true, true, true);
    CHECK(modes.system == SC_LED_FAULT && modes.thermal == SC_LED_OFF);
    CHECK(sc_led_modes(false, true, false, true, true, true).system == SC_LED_BLINK);
    CHECK(sc_led_modes(false, true, false, true, true, true).gnss == SC_LED_OFF);
    for (uint32_t ms = 0; ms < 2000; ms++)
    {
        const uint32_t phase = ms % 1000U;
        CHECK(sc_led_on(SC_LED_BLINK, ms) == (phase < 500U));
        CHECK(sc_led_on(SC_LED_FAULT, ms) == (phase < 100U || (phase >= 200U && phase < 300U)));
        CHECK(sc_led_on(SC_LED_SOLID, ms) && !sc_led_on(SC_LED_OFF, ms));
    }
    CHECK(!sc_led_on((sc_led_mode_t)99, 0));
    CHECK(strcmp(sc_led_mode_name(SC_LED_FAULT), "FAULT") == 0);
    CHECK(sc_led_on(SC_LED_BLINK, UINT32_MAX) == (UINT32_MAX % 1000U < 500U));

    sc_thermal_indicator_t indicator = {0};
    sc_thermal_control_t c = settled();
    const uint32_t start = UINT32_MAX - 100000U;
    for (uint32_t second = 0; second <= 300U; second++)
    {
        c.update_ms = start + second * 1000U;
        sc_thermal_indicator_update(&indicator, &c, true, c.update_ms);
        CHECK(indicator.stable);
    }
    CHECK(!c.stable); /* LED qualification never changes the strict PI flag. */
    c.filtered_q16--; /* One Q16 count beyond -0.25 C clears immediately. */
    sc_thermal_indicator_update(&indicator, &c, true, c.update_ms);
    CHECK(!indicator.stable && !indicator.timing);
    c.filtered_q16 = c.target_q16 + SC_LED_THERMAL_ERROR_Q16;
    sc_thermal_indicator_update(&indicator, &c, true, c.update_ms);
    CHECK(indicator.timing);
    c.filtered_q16++;
    sc_thermal_indicator_update(&indicator, &c, true, c.update_ms);
    CHECK(!indicator.timing);
    c = settled();
    c.slope_q16_per_min = -6553;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(indicator.timing);
    sc_thermal_indicator_update(&indicator, &c, true, 2499);
    CHECK(indicator.stable && indicator.sample_ms == 0); /* Still fresh, no recovery dwell. */
    sc_thermal_indicator_update(&indicator, &c, true, 2500);
    CHECK(!indicator.timing);
    c.slope_q16_per_min = 6554;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(!indicator.timing);
    c.slope_q16_per_min = 6553;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(indicator.timing);
    c.slope_q16_per_min = -6554;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(!indicator.timing);
    c = settled();
    c.window_saturated = true;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(indicator.stable); /* Past saturation cannot add a 120 s recovery delay. */
    c.filtered_q16--;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(!indicator.stable);
    c.filtered_q16++;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(indicator.stable); /* Recover immediately even with the history flag set. */
    sc_thermal_indicator_update(&indicator, &c, false, 0);
    CHECK(!indicator.stable); /* Current hardware faults still override the LED. */
    sc_thermal_indicator_update(&indicator, &c, true, SC_TEMPERATURE_STALE_MS);
    CHECK(!indicator.stable); /* History never masks a stale sensor sample. */
    c = settled();
    c.ramp_q16--;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(indicator.stable); /* A slow target ramp within the error band is healthy. */
    c = settled();
    sc_thermal_indicator_update(&indicator, &c, false, 0);
    CHECK(!indicator.timing);
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    c.update_ms = 1000;
    sc_thermal_indicator_update(&indicator, &c, true, 1000);
    c.update_ms = 4000;
    sc_thermal_indicator_update(&indicator, &c, true, 4000);
    CHECK(indicator.since_ms == 4000 && indicator.stable); /* Fresh data recovers immediately. */
    c.running = false;
    sc_thermal_indicator_update(&indicator, &c, true, 4000);
    CHECK(!indicator.timing);
    c = settled();
    c.target_q16 = INT32_MAX;
    c.filtered_q16 = INT32_MIN;
    sc_thermal_indicator_update(&indicator, &c, true, 0);
    CHECK(!indicator.timing); /* Extreme inputs do not overflow signed subtraction. */
    sc_clock_indicator_t clock = {0};
    CHECK(!sc_clock_indicator_update(&clock, false, true, true, 0));
    CHECK(sc_clock_indicator_update(&clock, true, false, true, 1000));
    CHECK(sc_clock_indicator_update(&clock, false, true, true, 2000));
    CHECK(clock.grace && clock.loss_ms == 2000U);
    CHECK(sc_clock_indicator_update(&clock, false, false, true, 121999));
    CHECK(!sc_clock_indicator_update(&clock, false, true, true, 122000));
    CHECK(!sc_clock_indicator_update(&clock, false, true, true, 123000));
    CHECK(sc_clock_indicator_update(&clock, true, false, true, UINT32_MAX - 499U));
    CHECK(sc_clock_indicator_update(&clock, false, true, true, UINT32_MAX - 399U));
    CHECK(sc_clock_indicator_update(&clock, false, true, true, 119599U));
    CHECK(!sc_clock_indicator_update(&clock, false, true, true, 119600U));
    CHECK(sc_clock_indicator_update(&clock, true, false, true, 130000U));
    CHECK(sc_clock_indicator_update(&clock, false, true, true, 131000U));
    CHECK(!sc_clock_indicator_update(&clock, false, true, false, 131001U));
    CHECK(!clock.grace); /* Fault/thermal loss overrides the display grace. */
    CHECK(sc_clock_indicator_update(&clock, true, false, true, 132000U));
    CHECK(!sc_clock_indicator_update(&clock, false, false, true, 133000U));
    CHECK(!clock.grace); /* A phase-only loss of lock gets no GNSS grace. */
    puts("LED freshness, immediate thermal recovery and two-minute holdover grace passed");
    return 0;
}
