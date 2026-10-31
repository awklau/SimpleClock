/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_HEATER_H
#define SIMPLECLOCK_HEATER_H
#include "simpleclock/temperature.h"
#include <stddef.h>

#define SC_HEATER_FRAME_MS 1000U               /* PWM frame, milliseconds. */
#define SC_HEATER_LEASE_MS 250U                /* Foreground refresh lease, milliseconds. */
#define SC_HEATER_MAX_DURATION_S 300U          /* Manual trial duration, seconds. */
#define SC_HEATER_MAX_CONTROL_DURATION_S 1800U /* Bounded PI trial duration, seconds. */
#define SC_HEATER_CONTROL_UPDATE_MS 2500U      /* Maximum PI update age, milliseconds. */

enum
{
    SC_HEATER_FAULT_SENSOR = 1U,
    SC_HEATER_FAULT_STALE = 2U,
    SC_HEATER_FAULT_OVERTEMP = 4U,
    SC_HEATER_FAULT_LEASE = 8U,
    SC_HEATER_FAULT_CONTROL = 16U,
    SC_HEATER_FAULT_PREHEAT = 32U,
    SC_HEATER_FAULT_CONTROL_UPDATE = 64U
};

typedef enum
{
    SC_HEATER_ACCEPTED,
    SC_HEATER_LOCKED,
    SC_HEATER_NOT_READY,
    SC_HEATER_FAULTED,
    SC_HEATER_RANGE,
    SC_HEATER_BUSY,
    SC_HEATER_BAD_COMMAND,
    SC_HEATER_EXPIRED
} sc_heater_result_t;

/* manual_authorized is the actuator permission shared by manual trials and PI
 * (including automatic startup); the platform checks each mode's preheat policy. It does
 * not certify the assembly. Duty uses permille (1000 = 100%). */
typedef struct
{
    bool manual_authorized;
    uint16_t max_duty_permille;
    int16_t trip_raw; /* TMP117 counts, exactly 1/128 C */
} sc_heater_profile_t;

/* Caller serializes every access. tick is bounded integer work for a 1 ms safety
 * ISR; refresh runs in foreground only. All times are modulo-2^32 ms, with
 * service intervals below 2^31 ms. No peripheral, PPS or host dependency.
 * preheat_permitted may be qualified preheat or an explicitly logged manual
 * operator exception. The caller must keep that distinction in its telemetry. */
typedef struct
{
    sc_heater_profile_t profile;
    uint32_t frame_ms, manual_ms, duration_ms, lease_ms, sample_ms, faults, stops, control_ms;
    uint16_t requested_permille, active_permille; /* 0..1000, requested/applied frame duty. */
    int16_t temperature_raw;                      /* TMP117 signed counts (1/128 C). */
    bool sample_valid, preheat_permitted, manual, controlled, continuous, output_on;
} sc_heater_t;

/* state is caller-owned and must be initialized before any other operation. All
 * calls share the caller's serialization rule; refresh is the foreground lease
 * renewal, while tick is suitable for bounded 1 ms safety service. The sample
 * pointer is read during the call and is not retained. now_ms is modulo-2^32. */
void sc_heater_init(sc_heater_t *state, sc_heater_profile_t profile, uint32_t now_ms);
void sc_heater_refresh(sc_heater_t *state, const sc_temperature_t *sample, bool preheat_permitted,
                       uint32_t now_ms);
sc_heater_result_t sc_heater_manual(sc_heater_t *state, uint16_t duty_permille, uint32_t duration_s,
                                    uint32_t now_ms);
/* Bounded operator PI trial. Can replace manual output only at its current
 * applied duty <=100%; cannot extend an already-running PI trial. update only
 * follows a fresh controller result; it does not renew the trial's expiry. */
sc_heater_result_t sc_heater_control_start(sc_heater_t *state, uint32_t duration_s,
                                           uint32_t now_ms);
/* Normal automatic PI, with no trial expiry. All sensor/foreground/control
 * deadlines still apply. Only starts from OFF; boot policy belongs to caller. */
sc_heater_result_t sc_heater_control_start_continuous(sc_heater_t *state, uint32_t now_ms);
/* Remove the expiry from a live PI trial after the safety tick. Expiry, faults,
 * stale samples and missed leases stop the heater before conversion; this call
 * does not renew sensor, foreground or controller deadlines. */
sc_heater_result_t sc_heater_control_continue(sc_heater_t *state, uint32_t now_ms);
sc_heater_result_t sc_heater_control_update(sc_heater_t *state, uint16_t duty_permille,
                                            uint32_t now_ms);
void sc_heater_off(sc_heater_t *state);
void sc_heater_fault(sc_heater_t *state, uint32_t reason);
sc_heater_result_t sc_heater_clear(sc_heater_t *state, uint32_t now_ms);
bool sc_heater_tick(sc_heater_t *state, uint32_t now_ms);

typedef enum
{
    SC_THERMAL_STATUS,
    SC_THERMAL_OFF,
    SC_THERMAL_MANUAL,
    SC_THERMAL_CLEAR,
    SC_THERMAL_PI,
    SC_THERMAL_CONTINUE
} sc_thermal_action_t;

typedef struct
{
    sc_thermal_action_t action;
    uint16_t duty_permille;
    uint32_t duration_s;
    int32_t target_milli_c;
} sc_thermal_command_t;

/* Entire command only: duty is decimal 0..1, at most three fractional digits;
 * duration is integer seconds. No floats, exponent/NaN, signs or trailing text.
 * Explicit length rejects embedded NUL and partial/overflowed transport lines. */
bool sc_thermal_parse(const char *line, size_t length, sc_thermal_command_t *out);
#endif
