/* SPDX-License-Identifier: MIT */
#include "simpleclock/status_leds.h"
#include <string.h>

bool sc_clock_indicator_update(sc_clock_indicator_t *state, bool locked, bool gnss_loss,
                               bool healthy, uint32_t now_ms)
{
    if (!healthy)
    {
        memset(state, 0, sizeof(*state));
        return false;
    }
    if (locked)
    {
        state->was_locked = true;
        state->grace = false;
        return true;
    }
    if (state->was_locked && gnss_loss)
    {
        state->loss_ms = now_ms;
        state->grace = true;
    }
    state->was_locked = false;
    if (state->grace && (uint32_t)(now_ms - state->loss_ms) >= SC_LED_HOLDOVER_GRACE_MS)
    {
        state->grace = false;
    }
    return state->grace;
}

void sc_thermal_indicator_update(sc_thermal_indicator_t *state, const sc_thermal_control_t *control,
                                 bool healthy, uint32_t now_ms)
{
    /* Widen before subtraction/negation so even malformed int32 inputs cannot
     * overflow this display-only calculation. Q16 differences have units K. */
    const int64_t delta = (int64_t)control->target_q16 - control->filtered_q16;
    const int64_t error = delta < 0 ? -delta : delta;
    const int64_t slope = control->slope_q16_per_min;
    const bool eligible = healthy && control->running && control->filter_valid &&
                          (uint32_t)(now_ms - control->update_ms) < SC_TEMPERATURE_STALE_MS &&
                          error <= SC_LED_THERMAL_ERROR_Q16 && control->slope_valid &&
                          (slope < 0 ? -slope : slope) <= SC_LED_THERMAL_SLOPE_Q16;
    /* LED3 describes current thermal quality. A previous zero/full-duty sample
     * must not add a hidden recovery dwell to an otherwise healthy temperature.
     * Clock qualification independently checks actuator headroom. */
    if (!eligible)
    {
        memset(state, 0, sizeof(*state));
        return;
    }
    /* update_ms is the accepted-sample timestamp. Cached observations cannot
     * refresh freshness; current eligibility is checked even for cached data. */
    if (state->seen && state->sample_ms == control->update_ms)
    {
        return;
    }
    if (!state->seen ||
        (uint32_t)(control->update_ms - state->sample_ms) >= SC_TEMPERATURE_STALE_MS)
    {
        state->timing = state->stable = false;
    }
    state->seen = true;
    state->sample_ms = control->update_ms;
    if (!state->timing)
    {
        state->timing = true;
        state->since_ms = control->update_ms;
    }
    state->stable = true; /* No recovery dwell. Quality loss above clears immediately. */
}

sc_led_modes_t sc_led_modes(bool fault, bool record_ready, bool gnss_fix, bool ppp_active,
                            bool thermal_available, bool thermal_stable)
{
    sc_led_modes_t modes = {SC_LED_BLINK, SC_LED_OFF, SC_LED_OFF};
    if (gnss_fix)
    {
        modes.gnss = ppp_active ? SC_LED_SOLID : SC_LED_BLINK;
    }
    if (thermal_available && !fault)
    {
        modes.thermal = thermal_stable ? SC_LED_SOLID : SC_LED_BLINK;
    }
    if (fault)
    {
        modes.system = SC_LED_FAULT;
    }
    else if (record_ready && gnss_fix && ppp_active && thermal_available && thermal_stable)
    {
        modes.system = SC_LED_SOLID;
    }
    return modes;
}

sc_led_modes_t sc_led_independent_modes(bool frequency_settled, bool gnss_fix, bool ppp_active,
                                        bool thermal_available, bool thermal_stable,
                                        bool ocxo_thermal_fault)
{
    const sc_led_modes_t modes = {
        frequency_settled ? SC_LED_SOLID : SC_LED_BLINK,
        gnss_fix ? (ppp_active ? SC_LED_SOLID : SC_LED_BLINK) : SC_LED_OFF,
        ocxo_thermal_fault
            ? SC_LED_FAULT
            : (thermal_available ? (thermal_stable ? SC_LED_SOLID : SC_LED_BLINK) : SC_LED_OFF)};
    return modes;
}

bool sc_led_on(sc_led_mode_t mode, uint32_t now_ms)
{
    /* Modulo gives a repeatable phase across all callers; it does not represent
     * elapsed time and therefore remains valid across MCU tick wrap. */
    const uint32_t phase = now_ms % 1000U;
    switch (mode)
    {
    case SC_LED_SOLID:
        return true;
    case SC_LED_BLINK:
        return phase < 500U;
    case SC_LED_FAULT:
        return phase < 100U || (phase >= 200U && phase < 300U);
    default:
        return false;
    }
}

const char *sc_led_mode_name(sc_led_mode_t mode)
{
    switch (mode)
    {
    case SC_LED_OFF:
        return "OFF";
    case SC_LED_BLINK:
        return "BLINK";
    case SC_LED_SOLID:
        return "SOLID";
    case SC_LED_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}
