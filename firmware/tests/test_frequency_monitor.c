/* SPDX-License-Identifier: MIT */
#include "simpleclock/frequency_monitor.h"
#include "simpleclock/status_leds.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static sc_frequency_monitor_t s;
static uint32_t now;
static uint64_t epoch;

static void sample(double drift, double requested)
{
    now += 1000U;
    epoch += 1000U;
    sc_frequency_monitor_sample(&s, epoch, now, now, drift, requested);
}

int main(void)
{
    now = UINT32_MAX - 60000U;
    epoch = 1000000U;
    for (unsigned i = 0U; i < 130U; i++)
    {
        sample(0.0, 0.0);
        CHECK(!s.settled);
    }
    sample(0.0, 0.0);
    CHECK(s.settled && s.window_valid && s.count == 121U);
    now += 3000U;
    epoch += 3000U;
    sc_frequency_monitor_service(&s, now);
    CHECK(s.settled); /* Short missing-data gap, no fabricated samples. */
    sample(0.0, 0.0);
    CHECK(s.settled && s.count == 121U);
    for (unsigned outage = 0U; outage < 50U; outage++)
    {
        now += 3000U;
        epoch += 3000U;
        for (unsigned i = 0U; i < 30U; i++)
        {
            sample(0.0, 0.0);
            CHECK(s.settled); /* Repeated short losses with fresh data between them. */
        }
    }
    const uint32_t timestamp = s.last_ms;
    sc_frequency_monitor_sample(&s, epoch, now, now + 1000U, 1.0, 0.0);
    CHECK(s.last_ms == timestamp && s.settled); /* Duplicate cannot update metrics. */
    sc_frequency_monitor_service(&s, now + 119999U);
    CHECK(s.settled);
    sc_frequency_monitor_service(&s, now + 120000U);
    CHECK(!s.settled && !s.window_valid);

    /* Independent positive/negative mean-error gates, and zero-mean scatter. */
    for (unsigned scenario = 0U; scenario < 6U; scenario++)
    {
        memset(&s, 0, sizeof(s));
        const double drift = scenario < 2U ? (scenario == 0U ? 0.051 : -0.051) : 0.0;
        const double requested = scenario < 2U    ? drift
                                 : scenario == 2U ? 0.051
                                 : scenario == 3U ? -0.051
                                                  : 0.0;
        for (unsigned i = 0U; i < 150U; i++)
        {
            sample(scenario == 4U ? (i % 2U ? 0.12 : -0.12) : drift, requested);
        }
        CHECK(s.settled == (scenario == 5U));
    }
    /* Repeated sparse returns cannot indefinitely renew an old good window. */
    for (unsigned i = 0U; i < 3U; i++)
    {
        now += 45000U;
        epoch += 45000U;
        sample(0.0, 0.0);
    }
    CHECK(!s.settled && !s.window_valid && s.count == 1U);
    memset(&s, 0, sizeof(s));
    for (unsigned i = 0U; i < 150U; i++)
    {
        now++;
        epoch += 1000U;
        sc_frequency_monitor_sample(&s, epoch, now, now, 0.0, 0.0);
    }
    CHECK(!s.window_valid && !s.settled); /* Buffered data cannot manufacture 120 s. */

    /* LED1 ignores both other channels, including their fault state. */
    for (unsigned flags = 0U; flags < 32U; flags++)
    {
        const sc_led_modes_t modes =
            sc_led_independent_modes(true, (flags & 1U) != 0U, (flags & 2U) != 0U,
                                     (flags & 4U) != 0U, (flags & 8U) != 0U, (flags & 16U) != 0U);
        CHECK(modes.system == SC_LED_SOLID);
        CHECK(modes.gnss ==
              ((flags & 1U) ? ((flags & 2U) ? SC_LED_SOLID : SC_LED_BLINK) : SC_LED_OFF));
        if (flags & 16U)
        {
            CHECK(modes.thermal == SC_LED_FAULT);
        }
    }
    puts("Independent frequency means/scatter, missing data, dwell and LED separation passed");
    return 0;
}
