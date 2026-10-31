/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_THERMAL_CONTROL_H
#define SIMPLECLOCK_THERMAL_CONTROL_H
#include "simpleclock/temperature.h"

#define SC_THERMAL_Q16 65536L   /* Q16 scale: 65536 counts per degree Celsius. */
#define SC_THERMAL_HISTORY 120U /* One control point per nominal second. */
/* User-authorized full-power PI ceiling (2026-09-15), still subject to all
 * actuator/sensor deadlines and the physical 0..1000 permille range. */
#define SC_THERMAL_PI_MAX_PERMILLE 1000U
/* Autonomous target bounds authorized 2026-09-16; legacy adapters enforce
 * their own lower ceiling before invoking this reusable controller. */
#define SC_THERMAL_TARGET_MAX_Q16 (60L * SC_THERMAL_Q16)
#define SC_THERMAL_DEFAULT_KP_MICRO 1000000L
#define SC_THERMAL_DEFAULT_KI_MICRO 4000L

/* Legacy diagnostic PI gains; AUTO_HYBRID selects generated PID defaults.
 * Signed Q16 Celsius;
 * gains use millionths of full duty: kp=1000000 is 1.0 duty/K and ki=4000
 * is 0.004 duty/(K*s), giving an integral time Kp/Ki of 250 s.
 * Controller arithmetic is fixed-point and foreground-only. */
typedef struct
{
    int32_t kp_micro, ki_micro; /* Gain units described above, millionths. */
    uint16_t max_permille;      /* 0..1000, PI output ceiling. */
} sc_thermal_gains_t;

/* Parallel PID derivative: -Kd*d(measured temperature)/dt. Kd is in
 * millionths of duty*s/K; filter_ms is a backward-Euler low-pass time constant.
 * limit_micro bounds derivative authority separately from final duty limits. */
typedef struct
{
    int32_t kd_micro, limit_micro;
    uint32_t filter_ms;
} sc_thermal_derivative_t;

typedef struct
{
    uint32_t ms;            /* MCU observation time, modulo-2^32 milliseconds. */
    int32_t error_q16;      /* Filtered temperature minus contemporaneous target, Q16 K. */
    uint16_t duty_permille; /* Quantized actuator command, 0..1000. */
    bool saturated;
} sc_thermal_point_t;

/* Foreground-owned. update consumes at most one NEW, fresh conversion per
 * nominal second, using actual elapsed time; it never catches up in a burst.
 * The heater actuator owns physical output, independent expiry and shutdown.
 * `stable` retains section 7's strict diagnostic test (0.05 K, 0.01 K/min,
 * five-minute dwell). LED3 and automatic clock qualification use their own
 * operator-selected 0.25 K / 0.1 K/min policy; neither depends on this flag. */
typedef struct
{
    sc_thermal_point_t history[SC_THERMAL_HISTORY];
    sc_thermal_gains_t gains;
    sc_thermal_derivative_t derivative;
    uint32_t update_ms, control_ms, sequence, stable_since_ms, exit_since_ms;
    int32_t filtered_q16, ramp_q16, target_q16; /* Signed Q16 Celsius. */
    int32_t integral_micro;                     /* PI output in duty millionths. */
    int32_t measured_q16, derivative_rate_q16_per_s, derivative_micro;
    int32_t slope_q16_per_min;             /* Q16 Celsius per minute. */
    uint32_t updates;                      /* Number of accepted new samples. */
    uint16_t duty_permille, mean_permille; /* Current/window mean, 0..1000. */
    uint8_t count, next;                   /* Valid history count/circular index. */
    bool filter_valid, running, stable, slope_valid, saturated;
    bool entry_timing, exit_timing, window_saturated;
} sc_thermal_control_t;

/* Foreground-only API. state is caller-owned and must be initialized once;
 * update consumes a fresh sample without retaining its pointer. The control
 * state owns PI qualification, while the heater owns actuator expiry and
 * shutdown. All timestamps use modulo-2^32 MCU milliseconds. */
void sc_thermal_control_init(sc_thermal_control_t *state);
void sc_thermal_control_stop(sc_thermal_control_t *state);
/* Fresh filter required. Set the initial target and start its ramp at filtered
 * temperature, matching previous applied duty. Later bounded target changes
 * use sc_thermal_control_target without restarting PID.
 * A previous manual duty above the PI cap is rejected, never silently clipped. */
bool sc_thermal_control_start(sc_thermal_control_t *state, int32_t target_q16,
                              uint16_t previous_permille, uint32_t now_ms);
/* Retunes in one operation with I_new = u_previous - Kp_new*error - D. */
bool sc_thermal_control_gains(sc_thermal_control_t *state, sc_thermal_gains_t gains);
/* Change the D profile only while stopped. Kd=0 preserves legacy PI behavior.
 * Start/stop, stale data and sensor faults clear D history; setpoint changes
 * preserve it and cannot create a derivative kick. */
bool sc_thermal_control_derivative(sc_thermal_control_t *state, sc_thermal_derivative_t config);
/* Select generated AUTO_HYBRID PID defaults before starting control. */
bool sc_thermal_control_pid_defaults(sc_thermal_control_t *state);
/* Slow outer-loop target update, 0..60 C. Preserve integrator, filter and ramp
 * history; the caller owns the tighter 1 mK/s outer limit. No proportional
 * cancellation: changing the setpoint must still request a heater response. */
bool sc_thermal_control_target(sc_thermal_control_t *state, int32_t target_q16);
/* true only on a new control/filter update; invalid data resets qualification
 * and stops PI. Repeated cached samples do not integrate or prolong stability. */
bool sc_thermal_control_update(sc_thermal_control_t *state, const sc_temperature_t *sample,
                               uint32_t now_ms);
#endif
