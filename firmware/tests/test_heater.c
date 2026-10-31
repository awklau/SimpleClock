/* SPDX-License-Identifier: MIT */
#include "simpleclock/heater.h"
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

static sc_temperature_t sample(uint32_t now_ms)
{
    return (sc_temperature_t){.sample_ms = now_ms, .raw = 3200, .valid = true, .sequence = 1};
}

int main(void)
{
    sc_heater_t heater;
    sc_heater_profile_t profile = {false, 250, 6400};
    sc_temperature_t temp = sample(0);
    sc_heater_init(&heater, profile, 0);
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(sc_heater_manual(&heater, 100, 60, 0) == SC_HEATER_LOCKED);
    CHECK(!sc_heater_tick(&heater, 1000));
    profile.manual_authorized = true;
    for (unsigned duty = 0; duty <= 1000U; duty += 100U)
    {
        profile.max_duty_permille = 1000;
        sc_heater_init(&heater, profile, 0);
        temp = sample(0);
        sc_heater_refresh(&heater, &temp, true, 0);
        CHECK(sc_heater_manual(&heater, (uint16_t)duty, 2, 0) == SC_HEATER_ACCEPTED);
        uint32_t on_ms = 0;
        for (uint32_t ms = 0; ms <= 3000U; ms++)
        {
            if (ms % 1000U == 0)
            {
                temp = sample(ms);
            }
            if (ms % 100U == 0)
            {
                sc_heater_refresh(&heater, &temp, true, ms);
            }
            if (sc_heater_tick(&heater, ms))
            {
                on_ms++;
            }
            if (ms < 1000U)
            {
                CHECK(!heater.output_on);
            }
        }
        CHECK(on_ms == 2U * duty && !heater.manual && !heater.output_on && !heater.faults);
    }

    sc_heater_init(&heater, profile, 0);
    temp = sample(0);
    sc_heater_refresh(&heater, &temp, false, 0);
    CHECK(sc_heater_manual(&heater, 100, 60, 0) == SC_HEATER_NOT_READY);
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(sc_heater_manual(&heater, 1001, 60, 0) == SC_HEATER_RANGE);
    CHECK(sc_heater_manual(&heater, 100, 0, 0) == SC_HEATER_RANGE);
    CHECK(sc_heater_manual(&heater, 100, 301, 0) == SC_HEATER_RANGE);
    CHECK(sc_heater_manual(&heater, 1000, 60, 0) == SC_HEATER_ACCEPTED);
    CHECK(sc_heater_manual(&heater, 1000, 60, 1) == SC_HEATER_BUSY);
    for (uint32_t ms = 100; ms <= 1000; ms += 100)
    {
        sc_heater_refresh(&heater, &temp, true, ms);
    }
    CHECK(sc_heater_tick(&heater, 1000));
    CHECK(sc_heater_tick(&heater, 1249));
    CHECK(!sc_heater_tick(&heater, 1250) && (heater.faults & SC_HEATER_FAULT_LEASE));
    temp = sample(1251);
    sc_heater_refresh(&heater, &temp, true, 1251);
    CHECK(!heater.output_on && heater.faults); /* Recovery never erases a trip. */
    CHECK(sc_heater_clear(&heater, 1251) == SC_HEATER_ACCEPTED);
    CHECK(!heater.manual);
    CHECK(sc_heater_manual(&heater, 100, 60, 1251) == SC_HEATER_ACCEPTED);
    sc_heater_refresh(&heater, &temp, true, 1501); /* Expire before lease renewal. */
    CHECK(heater.faults & SC_HEATER_FAULT_LEASE);

    sc_heater_init(&heater, profile, 0);
    temp = sample(0);
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(sc_heater_manual(&heater, 1000, 60, 0) == SC_HEATER_ACCEPTED);
    for (uint32_t ms = 100; ms <= 2400; ms += 100)
    {
        sc_heater_refresh(&heater, &temp, true, ms);
    }
    CHECK(sc_heater_tick(&heater, 2499));
    CHECK(!sc_heater_tick(&heater, 2500) && (heater.faults & SC_HEATER_FAULT_STALE));
    CHECK(sc_heater_clear(&heater, 2500) == SC_HEATER_NOT_READY);

    for (unsigned fault = 0; fault < 4; fault++)
    {
        sc_heater_init(&heater, profile, 0);
        temp = sample(0);
        sc_heater_refresh(&heater, &temp, true, 0);
        CHECK(sc_heater_manual(&heater, 1000, 60, 0) == SC_HEATER_ACCEPTED);
        if (fault == 0)
        {
            temp.valid = false;
        }
        if (fault == 1)
        {
            temp.raw = 6400;
        }
        if (fault == 2)
        {
            temp.raw = 200 * 128;
        }
        sc_heater_refresh(&heater, &temp, fault != 3, 1);
        CHECK(heater.faults && !heater.manual && !heater.output_on);
    }
    sc_heater_init(&heater, profile, 0);
    temp = sample(0);
    sc_heater_refresh(&heater, &temp, true, 0);
    sc_heater_fault(&heater, SC_HEATER_FAULT_CONTROL);
    CHECK(sc_heater_clear(&heater, 0) == SC_HEATER_FAULTED);
    sc_heater_off(&heater);
    CHECK(heater.faults); /* OFF cannot clear a fault. */

    const uint32_t base = UINT32_MAX - 499U;
    sc_heater_init(&heater, profile, base);
    temp = sample(base);
    sc_heater_refresh(&heater, &temp, true, base);
    CHECK(sc_heater_manual(&heater, 250, 1, base) == SC_HEATER_ACCEPTED);
    unsigned on_ms = 0;
    for (uint32_t elapsed = 0; elapsed <= 2000; elapsed++)
    {
        const uint32_t now = base + elapsed;
        if (elapsed % 1000U == 0)
        {
            temp = sample(now);
        }
        if (elapsed % 100U == 0)
        {
            sc_heater_refresh(&heater, &temp, true, now);
        }
        if (sc_heater_tick(&heater, now))
        {
            on_ms++;
        }
    }
    CHECK(on_ms == 250U && !heater.manual && !heater.faults);

    sc_thermal_command_t command;
    const char *valid[] = {"thermal off",
                           "thermal status",
                           "fault clear",
                           "thermal manual 0.100 60",
                           "thermal manual 0.25 300",
                           "thermal manual 1 1",
                           "thermal manual 0 1"};
    for (unsigned i = 0; i < sizeof(valid) / sizeof(valid[0]); i++)
    {
        CHECK(sc_thermal_parse(valid[i], strlen(valid[i]), &command));
    }
    CHECK(sc_thermal_parse("thermal manual 0.25 300", 23, &command));
    CHECK(command.action == SC_THERMAL_MANUAL && command.duty_permille == 250 &&
          command.duration_s == 300);
    const char *invalid[] = {"thermal manual nan 60",
                             "thermal manual -0.1 60",
                             "thermal manual 1.1 60",
                             "thermal manual .1 60",
                             "thermal manual 0.0001 60",
                             "thermal manual 0.1 301",
                             "thermal manual 0.1 0",
                             "thermal manual 0.1 1x",
                             "thermal manual 0.1 1\n",
                             "thermal manual 0.1 4294967296",
                             "thermal manual 0.1 60 thermal off",
                             "thermal auto"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); i++)
    {
        CHECK(!sc_thermal_parse(invalid[i], strlen(invalid[i]), &command));
    }
    CHECK(!sc_thermal_parse("thermal off\0garbage", 19, &command));
    CHECK(sc_thermal_parse("thermal pi 42.125 1800", 22, &command));
    CHECK(command.action == SC_THERMAL_PI && command.target_milli_c == 42125 &&
          command.duration_s == 1800);
    CHECK(sc_thermal_parse("thermal pi 42.125", 17, &command));
    CHECK(command.action == SC_THERMAL_PI && command.target_milli_c == 42125 &&
          command.duration_s == 0);
    CHECK(sc_thermal_parse("thermal pi 0", 12, &command));
    CHECK(command.action == SC_THERMAL_PI && command.target_milli_c == 0 &&
          command.duration_s == 0);
    CHECK(sc_thermal_parse("thermal continue", 16, &command));
    CHECK(command.action == SC_THERMAL_CONTINUE);
    const char *bad_pi[] = {
        "thermal pi nan 60",     "thermal pi 60.001 60", "thermal pi -2 60",
        "thermal pi 42.0001 60", "thermal pi 42 0",      "thermal pi 42 1801",
        "thermal pi 42 000",     "thermal pi 42 ",       "thermal pi 42 4294967296",
        "thermal pi 42 60x",     "thermal pi 42. 60",    "thermal continue extra"};
    for (unsigned i = 0; i < sizeof(bad_pi) / sizeof(bad_pi[0]); i++)
    {
        CHECK(!sc_thermal_parse(bad_pi[i], strlen(bad_pi[i]), &command));
    }
    CHECK(sc_thermal_parse("thermal pi 60 60", 16, &command));
    CHECK(command.target_milli_c == 60000);
    sc_heater_profile_t expanded = profile;
    expanded.trip_raw = 70 * 128;
    sc_heater_init(&heater, expanded, 0);
    temp = sample(0);
    temp.raw = 70 * 128 - 1;
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(heater.faults == 0U);
    temp.raw = 70 * 128;
    sc_heater_refresh(&heater, &temp, true, 1U);
    CHECK(heater.faults & SC_HEATER_FAULT_OVERTEMP);
    expanded.trip_raw++;
    sc_heater_init(&heater, expanded, 0);
    CHECK(!heater.profile.manual_authorized);
    sc_heater_init(&heater, profile, 0);
    temp = sample(0);
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(sc_heater_control_start(&heater, 1801, 0) == SC_HEATER_RANGE);
    CHECK(sc_heater_control_start(&heater, 10, 0) == SC_HEATER_ACCEPTED);
    CHECK(sc_heater_control_update(&heater, 800, 0) == SC_HEATER_ACCEPTED);
    CHECK(sc_heater_control_start(&heater, 10, 0) == SC_HEATER_BUSY);
    CHECK(sc_heater_manual(&heater, 100, 60, 0) == SC_HEATER_BUSY);
    for (uint32_t ms = 100; ms <= 2400; ms += 100)
    {
        temp = sample(ms); /* Keep foreground and sensor healthy, PI update stalls. */
        sc_heater_refresh(&heater, &temp, true, ms);
    }
    (void)sc_heater_tick(&heater, 2500);
    CHECK(!heater.output_on && !heater.controlled &&
          (heater.faults & SC_HEATER_FAULT_CONTROL_UPDATE));
    CHECK(sc_heater_control_update(&heater, 100, 2500) == SC_HEATER_FAULTED);

    /* PI may now hold 100% across frame boundaries, with no one-tick OFF notch.
     * Independent deadlines still expire it at the requested instant. */
    profile.max_duty_permille = 1000;
    sc_heater_init(&heater, profile, 0);
    temp = sample(0);
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(sc_heater_control_start(&heater, 4, 0) == SC_HEATER_ACCEPTED);
    uint32_t full_on_ticks = 0;
    for (uint32_t ms = 0; ms <= 4000U; ms++)
    {
        if (ms % 100U == 0U)
        {
            temp = sample(ms);
            sc_heater_refresh(&heater, &temp, true, ms);
        }
        if (ms % 1000U == 0U && ms < 4000U)
        {
            CHECK(sc_heater_control_update(&heater, 1000, ms) == SC_HEATER_ACCEPTED);
        }
        if (sc_heater_tick(&heater, ms))
        {
            full_on_ticks++;
        }
    }
    CHECK(full_on_ticks == 3000U && !heater.controlled && !heater.output_on);
    CHECK(sc_heater_control_start(&heater, 4, 4000) == SC_HEATER_ACCEPTED);
    CHECK(sc_heater_control_update(&heater, 1001, 4000) == SC_HEATER_RANGE);
    CHECK(!heater.output_on && (heater.faults & SC_HEATER_FAULT_CONTROL));

    /* Continue checks existing safety deadlines before removing the old expiry. */
    sc_heater_init(&heater, profile, 0);
    temp = sample(0);
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(sc_heater_control_continue(&heater, 0) == SC_HEATER_NOT_READY);
    CHECK(sc_heater_control_start(&heater, 1, 0) == SC_HEATER_ACCEPTED);
    for (uint32_t ms = 100; ms < 1000; ms += 100)
    {
        temp = sample(ms);
        sc_heater_refresh(&heater, &temp, true, ms);
        CHECK(sc_heater_control_update(&heater, 400, ms) == SC_HEATER_ACCEPTED);
    }
    CHECK(sc_heater_control_continue(&heater, 1000) == SC_HEATER_EXPIRED);
    CHECK(!heater.controlled && !heater.continuous && heater.duration_ms == 1000U);

    sc_heater_init(&heater, profile, 0);
    temp = sample(0);
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(sc_heater_control_start(&heater, 10, 0) == SC_HEATER_ACCEPTED);
    CHECK(sc_heater_control_continue(&heater, SC_HEATER_LEASE_MS) == SC_HEATER_FAULTED);
    CHECK((heater.faults & SC_HEATER_FAULT_LEASE) && !heater.controlled);

    sc_heater_init(&heater, profile, 0);
    temp = sample(0);
    sc_heater_refresh(&heater, &temp, true, 0);
    CHECK(sc_heater_control_start(&heater, 10, 0) == SC_HEATER_ACCEPTED);
    for (uint32_t ms = 100; ms < SC_TEMPERATURE_STALE_MS; ms += 100)
    {
        sc_heater_refresh(&heater, &temp, true, ms); /* Foreground is live, sensor sample is old. */
        CHECK(sc_heater_control_update(&heater, 400, ms) == SC_HEATER_ACCEPTED);
    }
    CHECK(sc_heater_control_continue(&heater, SC_TEMPERATURE_STALE_MS) == SC_HEATER_FAULTED);
    CHECK((heater.faults & SC_HEATER_FAULT_STALE) && !(heater.faults & SC_HEATER_FAULT_LEASE) &&
          !heater.controlled);

    /* Continuing near counter wrap survives the bounded trial's original expiry.
     * Conversion preserves the PWM latch and controller cadence at call time. */
    const uint32_t continue_boot = UINT32_MAX - 1500U;
    sc_heater_init(&heater, profile, continue_boot);
    temp = sample(continue_boot);
    sc_heater_refresh(&heater, &temp, true, continue_boot);
    CHECK(sc_heater_control_start(&heater, 3, continue_boot) == SC_HEATER_ACCEPTED);
    CHECK(sc_heater_control_update(&heater, 400, continue_boot) == SC_HEATER_ACCEPTED);
    for (uint32_t elapsed = 100U; elapsed <= 2400U; elapsed += 100U)
    {
        const uint32_t now = continue_boot + elapsed;
        temp = sample(now);
        sc_heater_refresh(&heater, &temp, true, now);
        if (elapsed == 2000U)
        {
            CHECK(sc_heater_control_update(&heater, 400, now) == SC_HEATER_ACCEPTED);
        }
        if (elapsed == 2400U)
        {
            CHECK(sc_heater_control_update(&heater, 600, now) == SC_HEATER_ACCEPTED);
        }
    }
    const uint32_t continued_frame_ms = heater.frame_ms;
    const uint32_t continued_control_ms = heater.control_ms;
    const uint16_t continued_active = heater.active_permille;
    const uint16_t continued_requested = heater.requested_permille;
    CHECK(continued_active == 400U && continued_requested == 600U);
    CHECK(sc_heater_control_continue(&heater, continue_boot + 2500U) == SC_HEATER_ACCEPTED);
    CHECK(heater.continuous && heater.duration_ms == 0U && heater.frame_ms == continued_frame_ms &&
          heater.control_ms == continued_control_ms && heater.active_permille == continued_active &&
          heater.requested_permille == continued_requested);
    for (uint32_t elapsed = 2600U; elapsed <= 4000U; elapsed += 100U)
    {
        const uint32_t now = continue_boot + elapsed;
        temp = sample(now);
        sc_heater_refresh(&heater, &temp, true, now);
        if (elapsed == 3000U)
        {
            CHECK(sc_heater_control_update(&heater, 600, now) == SC_HEATER_ACCEPTED);
        }
        (void)sc_heater_tick(&heater, now);
        CHECK(heater.controlled && heater.continuous && heater.faults == 0U);
    }

    /* Continuous PI survives the trial limit and counter wrap, but the same
     * control-result and foreground deadlines still force a latched stop. */
    const uint32_t boot = UINT32_MAX - 5000U;
    sc_heater_init(&heater, profile, boot);
    temp = sample(boot);
    sc_heater_refresh(&heater, &temp, true, boot);
    CHECK(sc_heater_control_start(&heater, 0, boot) == SC_HEATER_RANGE);
    CHECK(sc_heater_control_start_continuous(&heater, boot) == SC_HEATER_ACCEPTED);
    CHECK(heater.continuous);
    for (uint32_t elapsed = 0; elapsed <= 1801000U; elapsed += 100U)
    {
        const uint32_t time = boot + elapsed;
        temp = sample(time);
        sc_heater_refresh(&heater, &temp, true, time);
        CHECK(sc_heater_control_update(&heater, 1000, time) == SC_HEATER_ACCEPTED);
        CHECK(heater.controlled && heater.continuous && heater.faults == 0);
    }
    for (uint32_t elapsed = 1801100U; elapsed < 1803500U; elapsed += 100U)
    {
        temp = sample(boot + elapsed);
        sc_heater_refresh(&heater, &temp, true, boot + elapsed);
    }
    CHECK(!sc_heater_tick(&heater, boot + 1803500U));
    CHECK(heater.faults == SC_HEATER_FAULT_CONTROL_UPDATE && !heater.continuous);
    temp = sample(boot + 1803500U);
    sc_heater_refresh(&heater, &temp, true, boot + 1803500U);
    CHECK(sc_heater_clear(&heater, boot + 1803500U) == SC_HEATER_ACCEPTED);
    CHECK(sc_heater_control_start_continuous(&heater, boot + 1803500U) == SC_HEATER_ACCEPTED);
    CHECK(!sc_heater_tick(&heater, boot + 1803750U));
    CHECK(heater.faults == SC_HEATER_FAULT_LEASE && !heater.controlled);
    return 0;
}
