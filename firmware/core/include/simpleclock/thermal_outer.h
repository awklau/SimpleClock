/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_THERMAL_OUTER_H
#define SIMPLECLOCK_THERMAL_OUTER_H

#include <stdbool.h>
#include <stdint.h>

/* The outer loop is deliberately independent of thermal_control.c.  It only
 * returns a bounded temperature target; the inner PI/PID still owns the
 * heater command. */
#define SC_THERMAL_OUTER_Q16 65536L
#define SC_THERMAL_OUTER_DUTY_MAX_PERMILLE 1000U
#define SC_THERMAL_OUTER_WINDOW_MS 30000U
#define SC_THERMAL_OUTER_MAX_GAP_MS 3000U
/* User-authorized upper raw trip, 2026-09-16. This is a bench policy,
 * not a rating or calibration of the assembled thermal path. */
#define SC_THERMAL_OUTER_MAX_TRIP_Q16 (70L * SC_THERMAL_OUTER_Q16)

typedef struct
{
    uint16_t target_duty_permille;   /* 500 for the specified 50% target. */
    uint16_t duty_deadband_permille; /* 10 for the continuous 49..51% band. */
    uint32_t duty_filter_tau_ms;     /* 100000 ms. */
    uint32_t nominal_recentering_s;  /* 1000 s. */
    int32_t gain_q16_per_unit_duty;  /* Gtheta estimate, 10 K per unit duty. */
    int32_t setpoint_slew_q16_per_s; /* 0.004 K/s (14.4 K/h), represented in Q16. */
    int32_t initial_target_q16;
    int32_t target_min_q16;
    int32_t target_max_q16;
    int32_t overtemperature_trip_q16; /* Independent raw sensor trip. */
    int32_t pause_abs_error_q16;
    uint32_t pause_error_duration_s;
    int32_t resume_abs_error_q16;
    uint32_t resume_good_duration_s;
    uint16_t saturation_low_permille;
    uint16_t saturation_high_permille;
    uint32_t centered_dwell_s; /* Advisory only. */
    uint16_t centered_halfwidth_permille;
} sc_thermal_outer_config_t;

typedef enum
{
    SC_THERMAL_OUTER_PAUSE_NONE = 0,
    SC_THERMAL_OUTER_PAUSE_BOOT,
    SC_THERMAL_OUTER_PAUSE_EXPLICIT,
    SC_THERMAL_OUTER_PAUSE_PERMISSION,
    SC_THERMAL_OUTER_PAUSE_ERROR,
    SC_THERMAL_OUTER_PAUSE_SATURATION, /* Reserved legacy telemetry value. */
    SC_THERMAL_OUTER_PAUSE_SAFETY,
    SC_THERMAL_OUTER_PAUSE_CONFIG,
    SC_THERMAL_OUTER_PAUSE_DUTY_DATA
} sc_thermal_outer_pause_reason_t;

typedef struct
{
    int32_t raw_temperature_q16;
    int32_t filtered_temperature_q16;
    uint16_t applied_duty_permille; /* Duty applied over the preceding interval. */
    bool actuator_saturated;        /* Diagnostic only; saturation does not block recovery. */
    bool applied_duty_valid;
    bool sensor_valid;
    bool centering_permitted; /* Independent of initial clock qualification/acquisition. */
} sc_thermal_outer_input_t;

typedef struct
{
    uint32_t duration_ms;
    uint16_t duty_permille;
} sc_thermal_outer_interval_t;

#define SC_THERMAL_OUTER_INTERVAL_HISTORY 32U

typedef struct
{
    sc_thermal_outer_config_t config;
    sc_thermal_outer_interval_t intervals[SC_THERMAL_OUTER_INTERVAL_HISTORY];
    uint64_t duty_ms_sum;
    uint32_t filtered_duty_q16; /* Permille*Q16; retains sub-permille residual. */
    uint32_t window_ms;
    uint32_t last_update_ms;
    uint32_t bad_error_since_ms;
    uint32_t recovery_since_ms;
    uint32_t centered_since_ms;
    uint8_t interval_count;
    uint8_t interval_next;
    int32_t target_q16;
    int32_t raw_error_q16;
    int32_t filtered_error_q16;
    int32_t target_rate_q16_per_s;
    int64_t target_remainder_num; /* Fractional target motion, projected at bounds. */
    uint16_t filtered_duty_permille;
    uint16_t mean_duty_permille;
    bool enabled;
    bool explicit_paused;
    bool centering_started;
    bool error_paused;     /* Hysteresis: only corrective target motion is allowed. */
    bool direction_paused; /* Requested motion would worsen excessive raw error. */
    bool safety_fault;
    bool config_fault;
    bool window_valid;
    bool window_saturated;
    bool power_target_limited;
    bool duty_centered;
    bool duty_data_valid;
    bool adaptation_was_allowed;
    bool have_update;
    bool bad_error_timing;
    bool recovery_timing;
    bool centered_timing;
    sc_thermal_outer_pause_reason_t pause_reason;
} sc_thermal_outer_state_t;

/* Fill an independent literal representation of default_parameters.json. */
void sc_thermal_outer_default_config(sc_thermal_outer_config_t *config);

/* Returns false for inconsistent bounds/gains.  Boot leaves the outer loop
 * enabled; adaptation needs permission and a fresh 30 s completed-duty window. */
bool sc_thermal_outer_init(sc_thermal_outer_state_t *state,
                           const sc_thermal_outer_config_t *config);

/* Adopt a captured/manual target and discard duty history from the previous
 * operating point (especially 100% boost). Preserves explicit pause/faults.
 * Does not restart or reset the inner PID. */
bool sc_thermal_outer_adopt_target(sc_thermal_outer_state_t *state, int32_t target_q16);

void sc_thermal_outer_pause(sc_thermal_outer_state_t *state);
void sc_thermal_outer_resume(sc_thermal_outer_state_t *state);
/* Revoke integration immediately between completed frames (e.g. loss of usable
 * GNSS). Does not latch manual pause or discard measured duty history. */
void sc_thermal_outer_freeze(sc_thermal_outer_state_t *state);
/* Clears only the latched outer safety indication after the independent
 * heater path has been made safe and the sensor has recovered. */
bool sc_thermal_outer_clear_safety_fault(sc_thermal_outer_state_t *state);

/* Foreground-only, monotonic local-time update.  A valid update always leaves
 * target_q16 bounded; out_target_q16 may be NULL.  The return value says that
 * the interval/sample was accepted.  A timing gap is accepted to refresh
 * diagnostics and the duty filter/window, but never catches up target motion. */
bool sc_thermal_outer_update(sc_thermal_outer_state_t *state, uint32_t now_ms,
                             const sc_thermal_outer_input_t *input, int32_t *out_target_q16);

#endif
