/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_THERMAL_BOOT_H
#define SIMPLECLOCK_THERMAL_BOOT_H
#include "simpleclock/hybrid_defaults.h"
#include <stdbool.h>
#include <stdint.h>

/* One point at each end plus nominal 1 Hz samples throughout the fit. */
#define SC_THERMAL_BOOT_POINTS (SC_HYBRID_THERMAL_BOOT_REFERENCE_WINDOW_S + 1U)
/* Compile-time units conversion; floor to one Q16 C/s count. The initial fit
 * remains diagnostic and does not scale this absolute handoff threshold. */
#define SC_THERMAL_BOOT_CAPTURE_SLOPE_Q16_PER_S                                                    \
    ((int32_t)(SC_HYBRID_THERMAL_BOOT_CAPTURE_SLOPE_MC_PER_MIN * 65536L / 60000L))

typedef enum
{
    SC_THERMAL_BOOT_DELAY,
    SC_THERMAL_BOOT_REFERENCE,
    SC_THERMAL_BOOT_DECAY,
    SC_THERMAL_BOOT_CAPTURED,
    SC_THERMAL_BOOT_NO_RISE,
    SC_THERMAL_BOOT_CEILING,
    SC_THERMAL_BOOT_BAD_DATA
} sc_thermal_boot_phase_t;

/* Foreground-only, caller-owned slope selector. It never drives the heater.
 * Temperatures/slopes are Q16 C and Q16 C/s. start_ms denotes the first full
 * powered PWM frame; sample_ms denotes sensor trigger time, not USB arrival.
 * The caller supplies each fresh conversion once and independently enforces
 * sensor validity, thermal bounds, lease and control-update deadlines. */
typedef struct
{
    uint32_t time_ms[SC_THERMAL_BOOT_POINTS];
    int32_t temperature_q16[SC_THERMAL_BOOT_POINTS];
    uint32_t start_ms, last_ms;
    uint32_t below_since_ms, below_elapsed_ms;
    int32_t reference_q16_per_s, slope_q16_per_s, target_q16;
    uint8_t count, next, below_count;
    bool have_sample, reference_valid, below_timing;
    sc_thermal_boot_phase_t phase;
} sc_thermal_boot_t;

void sc_thermal_boot_start(sc_thermal_boot_t *state, uint32_t first_power_ms);
/* True when a bounded target is ready, including explicitly flagged no-rise
 * or ceiling fallbacks. BAD_DATA must stop boost; it never selects a target.
 * Duplicate/early samples do not advance a fit or the continuous confirmation.
 * below_elapsed_ms freezes at capture; it is not wall-clock time since handoff. */
bool sc_thermal_boot_sample(sc_thermal_boot_t *state, uint32_t sample_ms, int32_t raw_q16);
#endif
