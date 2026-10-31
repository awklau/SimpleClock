/* SPDX-License-Identifier: MIT */
#include "simpleclock/heater.h"
#include "simpleclock/thermal_control.h"
#include <string.h>

/* frame_ms is the start of the current 1000 ms PWM frame. requested_permille
 * is latched into active_permille only at a frame boundary, so a foreground
 * command cannot alter the pulse width halfway through the current frame. */
void sc_heater_init(sc_heater_t *state, sc_heater_profile_t profile, uint32_t now_ms)
{
    memset(state, 0, sizeof(*state));
    state->profile = profile;
    state->frame_ms = now_ms;
    /* Reject unusable policy even if a future caller accidentally marks it verified. */
    if (!profile.max_duty_permille || profile.max_duty_permille > 1000U ||
        profile.trip_raw <= -55 * 128 || profile.trip_raw > 70 * 128)
    {
        state->profile.manual_authorized = false;
    }
}

void sc_heater_off(sc_heater_t *state)
{
    if (state->manual || state->controlled)
    {
        state->stops++;
    }
    state->manual = false;
    state->controlled = false;
    state->continuous = false;
    state->requested_permille = 0;
    state->active_permille = 0;
    state->output_on = false;
}

void sc_heater_fault(sc_heater_t *state, uint32_t reason)
{
    state->faults |= reason;
    sc_heater_off(state);
}

static uint32_t sensor_faults(const sc_heater_t *state, uint32_t now_ms)
{
    /* sample_ms is the sensor's trigger-origin timestamp, so this age check is
     * intentionally conservative. The signed raw range is TMP117 counts. */
    uint32_t reasons = 0;
    if (!state->sample_valid)
    {
        reasons |= SC_HEATER_FAULT_SENSOR;
    }
    if ((uint32_t)(now_ms - state->sample_ms) >= SC_TEMPERATURE_STALE_MS)
    {
        reasons |= SC_HEATER_FAULT_STALE;
    }
    if (state->sample_valid && state->temperature_raw >= state->profile.trip_raw)
    {
        reasons |= SC_HEATER_FAULT_OVERTEMP;
    }
    return reasons;
}

bool sc_heater_tick(sc_heater_t *state, uint32_t now_ms)
{
    const uint32_t frame_age = now_ms - state->frame_ms;
    if (frame_age >= SC_HEATER_FRAME_MS)
    {
        state->frame_ms = now_ms - frame_age % SC_HEATER_FRAME_MS;
        state->active_permille = state->requested_permille;
    }
    if (state->manual || state->controlled)
    {
        /* Every active mode must renew the lease from foreground. A safety tick
         * therefore treats a missed refresh as a latched fault before output is
         * recomputed below. */
        uint32_t reasons = sensor_faults(state, now_ms);
        if ((uint32_t)(now_ms - state->lease_ms) >= SC_HEATER_LEASE_MS)
        {
            reasons |= SC_HEATER_FAULT_LEASE;
        }
        if (!state->preheat_permitted)
        {
            reasons |= SC_HEATER_FAULT_PREHEAT;
        }
        if (state->controlled &&
            (uint32_t)(now_ms - state->control_ms) >= SC_HEATER_CONTROL_UPDATE_MS)
        {
            reasons |= SC_HEATER_FAULT_CONTROL_UPDATE;
        }
        if (reasons)
        {
            sc_heater_fault(state, reasons);
        }
        else if (!state->continuous && (uint32_t)(now_ms - state->manual_ms) >= state->duration_ms)
        {
            sc_heater_off(state);
        }
    }
    /* With a 1000 ms frame, duty permille equals pulse duration in ms.
     * Zero/off/fault bypass the boundary latch immediately; no delayed off. */
    state->output_on = state->profile.manual_authorized && (state->manual || state->controlled) &&
                       !state->faults &&
                       (uint32_t)(now_ms - state->frame_ms) < state->active_permille;
    return state->output_on;
}

void sc_heater_refresh(sc_heater_t *state, const sc_temperature_t *sample, bool preheat_permitted,
                       uint32_t now_ms)
{
    /* Expire the old lease BEFORE renewal, so resuming after a stall cannot hide it. */
    (void)sc_heater_tick(state, now_ms);
    state->temperature_raw = sample->raw;
    state->sample_ms = sample->sample_ms;
    state->sample_valid = sample->valid && sample->raw >= -55 * 128 && sample->raw <= 150 * 128;
    state->preheat_permitted = preheat_permitted;
    state->lease_ms = now_ms;
    if (state->sample_valid && state->temperature_raw >= state->profile.trip_raw)
    {
        sc_heater_fault(state, SC_HEATER_FAULT_OVERTEMP);
    }
    (void)sc_heater_tick(state, now_ms);
}

sc_heater_result_t sc_heater_manual(sc_heater_t *state, uint16_t duty_permille, uint32_t duration_s,
                                    uint32_t now_ms)
{
    (void)sc_heater_tick(state, now_ms);
    if (duty_permille > state->profile.max_duty_permille || !duration_s ||
        duration_s > SC_HEATER_MAX_DURATION_S)
    {
        return SC_HEATER_RANGE;
    }
    if (!duty_permille)
    {
        sc_heater_off(state);
        return SC_HEATER_ACCEPTED;
    }
    if (!state->profile.manual_authorized)
    {
        return SC_HEATER_LOCKED;
    }
    if (state->faults)
    {
        return SC_HEATER_FAULTED;
    }
    if (state->manual || state->controlled)
    {
        return SC_HEATER_BUSY;
    } /* No command can prolong a pulse trial. */
    if (!state->preheat_permitted || sensor_faults(state, now_ms) ||
        (uint32_t)(now_ms - state->lease_ms) >= SC_HEATER_LEASE_MS)
    {
        return SC_HEATER_NOT_READY;
    }
    state->requested_permille = duty_permille;
    state->manual_ms = now_ms;
    /* Duration measures whole active frames. The bounded wait to the next
     * boundary is additional (1..1000 ms); the lease applies during that wait. */
    /* duration_ms starts at manual_ms and includes the partial-frame wait; the
     * resulting trial contains exactly duration_s complete active frames. */
    state->duration_ms = duration_s * 1000U + SC_HEATER_FRAME_MS - (now_ms - state->frame_ms);
    state->manual = true;
    /* Wait for the next frame boundary, even when requested partway through a frame. */
    return SC_HEATER_ACCEPTED;
}

sc_heater_result_t sc_heater_clear(sc_heater_t *state, uint32_t now_ms)
{
    if (state->manual || state->controlled)
    {
        return SC_HEATER_BUSY;
    }
    if (state->faults & SC_HEATER_FAULT_CONTROL)
    {
        return SC_HEATER_FAULTED;
    }
    if (sensor_faults(state, now_ms))
    {
        return SC_HEATER_NOT_READY;
    }
    state->faults = 0;
    sc_heater_off(state); /* Clearance never resumes an expired or faulted trial. */
    return SC_HEATER_ACCEPTED;
}

static sc_heater_result_t control_start(sc_heater_t *state, uint32_t duration_s, bool continuous,
                                        uint32_t now_ms)
{
    (void)sc_heater_tick(state, now_ms);
    if ((!continuous && (!duration_s || duration_s > SC_HEATER_MAX_CONTROL_DURATION_S)) ||
        state->active_permille > SC_THERMAL_PI_MAX_PERMILLE)
    {
        return SC_HEATER_RANGE;
    }
    if (!state->profile.manual_authorized)
    {
        return SC_HEATER_LOCKED;
    }
    if (state->faults)
    {
        return SC_HEATER_FAULTED;
    }
    if (state->controlled || (continuous && state->manual))
    {
        return SC_HEATER_BUSY;
    }
    if (!state->preheat_permitted || sensor_faults(state, now_ms) ||
        (uint32_t)(now_ms - state->lease_ms) >= SC_HEATER_LEASE_MS)
    {
        return SC_HEATER_NOT_READY;
    }
    state->requested_permille = state->active_permille;
    state->manual = false;
    state->controlled = true;
    state->continuous = continuous;
    state->manual_ms = now_ms;
    state->duration_ms = duration_s * 1000U;
    state->control_ms = now_ms;
    return SC_HEATER_ACCEPTED;
}

sc_heater_result_t sc_heater_control_start(sc_heater_t *state, uint32_t duration_s, uint32_t now_ms)
{
    return control_start(state, duration_s, false, now_ms);
}

sc_heater_result_t sc_heater_control_start_continuous(sc_heater_t *state, uint32_t now_ms)
{
    return control_start(state, 0, true, now_ms);
}

sc_heater_result_t sc_heater_control_continue(sc_heater_t *state, uint32_t now_ms)
{
    const bool was_controlled = state->controlled;
    const bool was_continuous = state->continuous;
    (void)sc_heater_tick(state, now_ms);
    if (state->faults)
    {
        return SC_HEATER_FAULTED;
    }
    if (!state->controlled)
    {
        return was_controlled && !was_continuous ? SC_HEATER_EXPIRED : SC_HEATER_NOT_READY;
    }
    state->continuous = true;
    state->duration_ms = 0;
    return SC_HEATER_ACCEPTED;
}

sc_heater_result_t sc_heater_control_update(sc_heater_t *state, uint16_t duty_permille,
                                            uint32_t now_ms)
{
    (void)sc_heater_tick(state, now_ms); /* Old deadlines apply before a renewal. */
    if (state->faults)
    {
        return SC_HEATER_FAULTED;
    }
    if (!state->controlled)
    {
        return SC_HEATER_NOT_READY;
    }
    if (duty_permille > SC_THERMAL_PI_MAX_PERMILLE ||
        duty_permille > state->profile.max_duty_permille)
    {
        sc_heater_fault(state, SC_HEATER_FAULT_CONTROL);
        return SC_HEATER_RANGE;
    }
    state->requested_permille = duty_permille;
    state->control_ms = now_ms;
    return SC_HEATER_ACCEPTED;
}

static bool parse_pi(const char *line, size_t length, sc_thermal_command_t *out)
{
    /* Celsius 0..60 with up to three decimals. A target alone means continuous
     * PI; an explicit duration is bounded to 1..1800 seconds. */
    size_t index = 11U;
    uint32_t whole = 0, fraction = 0, scale = 100U, duration = 0;
    unsigned digits = 0;
    while (index < length && line[index] >= '0' && line[index] <= '9')
    {
        if (++digits > 2U)
        {
            return false;
        }
        whole = whole * 10U + (uint32_t)(line[index++] - '0');
    }
    if (!digits)
    {
        return false;
    }
    if (index < length && line[index] == '.')
    {
        index++;
        digits = 0;
        while (index < length && line[index] >= '0' && line[index] <= '9')
        {
            if (!scale)
            {
                return false;
            }
            fraction += (uint32_t)(line[index++] - '0') * scale;
            scale /= 10U;
            digits++;
        }
        if (!digits)
        {
            return false;
        }
    }
    const uint32_t target = whole * 1000U + fraction;
    if (target > 60000U)
    {
        return false;
    }
    if (index < length)
    {
        if (line[index++] != ' ' || index == length)
        {
            return false;
        }
        for (; index < length; index++)
        {
            if (line[index] < '0' || line[index] > '9' || duration > 1800U)
            {
                return false;
            }
            duration = duration * 10U + (uint32_t)(line[index] - '0');
        }
        if (!duration || duration > SC_HEATER_MAX_CONTROL_DURATION_S)
        {
            return false;
        }
    }
    *out = (sc_thermal_command_t){SC_THERMAL_PI, 0, duration, (int32_t)target};
    return true;
}

bool sc_thermal_parse(const char *line, size_t length, sc_thermal_command_t *out)
{
    /* The transport supplies an explicit byte count. Reject embedded NUL so a
     * caller cannot accidentally validate only a C-string prefix. */
    if (line == NULL || out == NULL || memchr(line, '\0', length) != NULL)
    {
        return false;
    }
    *out = (sc_thermal_command_t){0};
    if (length >= 12U && memcmp(line, "thermal pi ", 11U) == 0)
    {
        return parse_pi(line, length, out);
    }
    if (length == 16U && memcmp(line, "thermal continue", length) == 0)
    {
        out->action = SC_THERMAL_CONTINUE;
        return true;
    }
    if (length == 14U && memcmp(line, "thermal status", length) == 0)
    {
        out->action = SC_THERMAL_STATUS;
        return true;
    }
    if (length == 11U && memcmp(line, "thermal off", length) == 0)
    {
        out->action = SC_THERMAL_OFF;
        return true;
    }
    if (length == 11U && memcmp(line, "fault clear", length) == 0)
    {
        out->action = SC_THERMAL_CLEAR;
        return true;
    }
    if (length < 18U || memcmp(line, "thermal manual ", 15) != 0)
    {
        return false;
    }
    size_t index = 15U;
    if (line[index] != '0' && line[index] != '1')
    {
        return false;
    }
    unsigned duty = (unsigned)(line[index++] - '0') * 1000U;
    if (index < length && line[index] == '.')
    {
        index++;
        unsigned scale = 100U, digits = 0U;
        while (index < length && line[index] >= '0' && line[index] <= '9')
        {
            if (!scale)
            {
                return false;
            }
            duty += (unsigned)(line[index++] - '0') * scale;
            scale /= 10U;
            digits++;
        }
        if (!digits)
        {
            return false;
        }
    }
    if (duty > 1000U || index >= length || line[index++] != ' ' || index == length)
    {
        return false;
    }
    uint32_t duration = 0;
    for (; index < length; index++)
    {
        if (line[index] < '0' || line[index] > '9' || duration > 300U)
        {
            return false;
        }
        duration = duration * 10U + (uint32_t)(line[index] - '0');
    }
    if (!duration || duration > SC_HEATER_MAX_DURATION_S)
    {
        return false;
    }
    *out = (sc_thermal_command_t){SC_THERMAL_MANUAL, (uint16_t)duty, duration, 0};
    return true;
}
