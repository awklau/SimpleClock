/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_HYBRID_H
#define SIMPLECLOCK_HYBRID_H
#include "simpleclock/hybrid_defaults.h"
#include "simpleclock/hybrid_stats.h"
#include "simpleclock/frequency_monitor.h"
#include <stdbool.h>
#include <stdint.h>

/* Contiguous 1 Hz epochs: an N-second fitted span needs N+1 points. The
 * shortened startup window must not shorten the independent tracking fits. */
#define SC_HYBRID_QUALIFICATION_POINTS                                                             \
    ((unsigned)SC_HYBRID_QUALIFICATION_TEMPERATURE_STATISTICS_WINDOW_S + 1U)
#define SC_HYBRID_TRACKING_POINTS                                                                  \
    ((unsigned)SC_HYBRID_CLOCK_TRACKING_TEMPERATURE_STATISTICS_WINDOW_S + 1U)
#define SC_HYBRID_LOCK_SLOPE_POINTS ((unsigned)SC_HYBRID_CLOCK_LOCK_STATISTICS_WINDOW_S + 1U)

_Static_assert(SC_HYBRID_QUALIFICATION_POINTS >= 2U &&
                   SC_HYBRID_QUALIFICATION_POINTS <= SC_HYBRID_STATS_CAPACITY &&
                   SC_HYBRID_TRACKING_POINTS >= 2U &&
                   SC_HYBRID_TRACKING_POINTS <= SC_HYBRID_STATS_CAPACITY &&
                   SC_HYBRID_LOCK_SLOPE_POINTS >= 2U &&
                   SC_HYBRID_LOCK_SLOPE_POINTS <= SC_HYBRID_STATS_CAPACITY,
               "hybrid fit windows exceed stored history");
_Static_assert(SC_HYBRID_CLOCK_ACQUISITION_AVERAGE_SAMPLES >= 2U &&
                   SC_HYBRID_CLOCK_ACQUISITION_AVERAGE_SAMPLES <= SC_HYBRID_QUALIFICATION_POINTS &&
                   SC_HYBRID_CLOCK_ACQUISITION_VERIFICATION_SAMPLES >= 2U &&
                   SC_HYBRID_CLOCK_ACQUISITION_VERIFICATION_SAMPLES <= SC_HYBRID_STATS_CAPACITY,
               "acquisition averages must fit their available history");
_Static_assert(SC_HYBRID_CLOCK_FREQUENCY_SETTLE_SAMPLES >= 2U &&
                   SC_HYBRID_CLOCK_FREQUENCY_SETTLE_SAMPLES <= SC_HYBRID_STATS_CAPACITY,
               "frequency settling window exceeds stored history");

typedef enum
{
    /* Numbering is telemetry (hybrid_transition state=). Retired entries keep
     * their slots and are never entered: clock control has no thermal gates. */
    SC_HYBRID_BOOT,
    SC_HYBRID_PREHEAT, /* Retired. */
    SC_HYBRID_QUALIFY,
    SC_HYBRID_ACQUIRE,
    SC_HYBRID_VERIFY,
    SC_HYBRID_TRACK,
    SC_HYBRID_PHASE_RECOVERING,
    SC_HYBRID_HOLDOVER,
    SC_HYBRID_MANUAL_HOLD,
    SC_HYBRID_ACTUATOR_UNKNOWN,
    SC_HYBRID_ACQUISITION_FAULT,
    SC_HYBRID_CONFIG_FAULT,
    SC_HYBRID_THERMAL_RECOVERY, /* Retired. */
    SC_HYBRID_FREQUENCY,
    SC_HYBRID_DISABLED
} sc_hybrid_state_t;

typedef enum
{
    SC_HYBRID_LOOP_BOTH,
    SC_HYBRID_LOOP_FREQUENCY,
    SC_HYBRID_LOOP_PHASE,
    SC_HYBRID_LOOP_OFF
} sc_hybrid_loop_mode_t;

typedef enum
{
    SC_HYBRID_PROFILE_PPP,
    SC_HYBRID_PROFILE_SBAS,
    SC_HYBRID_PROFILE_STANDALONE,
    SC_HYBRID_PROFILE_STATIC /* Static PVT fixed-position solution. */
} sc_hybrid_profile_t;

#define SC_HYBRID_RUNTIME_FREQUENCY_MIN_S 10.0
#define SC_HYBRID_RUNTIME_PHASE_MIN_S 10.0
#define SC_HYBRID_RUNTIME_TIME_MAX_S 86400.0
#define SC_HYBRID_BIAS_CLIP_MAX_PS 5000U
#define SC_HYBRID_BIAS_CLIP_DEFAULT_PS                                                             \
    ((uint32_t)(SC_HYBRID_CLOCK_SINGLE_EPOCH_PHASE_INNOVATION_LIMIT_NS * 1000.0))

enum
{
    SC_HYBRID_LOCK_SPAN = 1U,
    SC_HYBRID_LOCK_PHASE = 2U,
    SC_HYBRID_LOCK_SLOPE = 4U,
    SC_HYBRID_LOCK_LIMIT = 8U,
    SC_HYBRID_LOCK_GNSS = 16U,
    SC_HYBRID_LOCK_ACTUATOR = 32U,
    SC_HYBRID_LOCK_EPOCH = 64U,
    SC_HYBRID_LOCK_INNOVATION = 128U
};

typedef struct
{
    uint64_t epoch_ms;
    /* innovation = drift - previous drift - trim delta (the drift guard). */
    double bias_ns, drift_ppb, trim_delta_ppb, innovation_ppb;
    double phase_error_ns, bias_slope_ppb;
    uint32_t sequence, now_ms, flags;
    uint8_t source_mode, time_system;
    bool phase_enabled;
} sc_hybrid_failure_t;

/* Positive times are independent. Slower phase response is an operator
 * recommendation, not a validation or persistence constraint. */

enum
{
    /* Bit values are telemetry; 2, 4 and 1024 (thermal) are retired. */
    SC_HYBRID_Q_GNSS = 1U,
    SC_HYBRID_Q_ACTUATOR = 8U,
    SC_HYBRID_Q_EPOCH = 16U,
    SC_HYBRID_Q_STALE = 32U,
    SC_HYBRID_Q_INNOVATION = 64U,
    SC_HYBRID_Q_WINDOW = 128U,
    SC_HYBRID_Q_LIMIT = 256U,
    SC_HYBRID_Q_COMMAND = 512U
};

/* Input belongs to one assembled receiver epoch. The adapter rejects SBF DNU
 * before converting ms->ns and ppm->ppb. epoch_ms is week*604800000+TOW;
 * received_ms/now_ms use modulo-2^32 MCU time, never USB arrival. A phase
 * coordinate survives missing packets; while tracking, gaps up to the short-gap
 * bridge keep steering authority and the reference. Reset/jump/reference/
 * configuration changes and longer gaps require recovery. Contiguous usable solution/time-system changes
 * retain running control with a fresh local phase anchor and the new profile.
 * Receiver data only: the thermal loop is an independent service. */
typedef struct
{
    uint64_t epoch_ms;
    uint32_t now_ms, received_ms;
    double bias_ns, drift_ppb;
    bool gnss_valid, coordinate_changed;
    /* gnss_valid means qualified PPP; fallback permissions are separate.
     * frequency_valid feeds the observer only, without granting control. A
     * frequency_discontinuity discards observer history as well as phase state. */
    bool standalone_valid, sbas_valid, frequency_valid, frequency_discontinuity;
    /* static_fix marks a standalone_valid Static-PVT fixed-position solution. */
    bool static_fix;
    uint8_t time_system;
} sc_hybrid_input_t;

/* Foreground-only controller and exclusive automatic FCW owner. No heap, I/O,
 * ISR, receiver reset or persistence. All actuator values are total pull ppb;
 * unquantized accumulator is deliberately separate from verified FCW.
 * Initialize with sc_hybrid_init before any other call. State/input/output
 * pointers must be non-NULL; calls retain no caller buffers. Service regularly
 * to enforce timeouts and keep unsigned MCU time differences unambiguous. */
typedef struct
{
    sc_hybrid_stats_t history;
    sc_hybrid_metrics_t metrics;
    sc_frequency_monitor_t frequency;
    /* Same-domain phase span: cold/recovery authority and post-handover lock
     * qualification use this; ordinary handovers can retain active steering. */
    uint32_t phase_good_ms;
    uint16_t phase_good_samples;
    uint8_t source_mode, time_system, loop_mode;
    /* warm survives holds/faults after initial acquisition; phase_enabled grants
     * steering authority independently of the relative-lock indication. */
    bool warm, phase_enabled, ramping;
    sc_hybrid_state_t state;
    /* accumulator_ppb retains sub-FCW corrections; applied_ppb is readback-verified. */
    double boot_ppb, applied_ppb, acquired_ppb, requested_ppb, accumulator_ppb;
    double acquire_target_ppb, lower_ppb, upper_ppb;
    /* Local control anchor/error in ns. The PI integral is total trim in PHASE
     * and requested FLL drift in BOTH, both expressed in ppb. */
    double phase_reference_ns, phase_error_ns, filtered_error_ns, phase_integral_ppb;
    /* phase_base_ns aligns the statistics coordinate across handovers;
     * residual_ns accumulates bias minus trapezoidally integrated drift. */
    double desired_drift_ppb, phase_base_ns, residual_ns;
    double previous_bias_ns, previous_drift_ppb, previous_applied_ppb;
    /* previous_bias_ns is the clipped estimate the loop used; the raw receiver
     * innovation against prediction is kept for telemetry. */
    double phase_innovation_ns;
    /* Operator times are whole seconds. Compact storage preserves target SRAM
     * headroom; getters promote to double before all control arithmetic. */
    uint32_t frequency_time_s, phase_time_s;
    uint32_t sbas_frequency_time_s, sbas_phase_time_s;
    uint32_t standalone_frequency_time_s, standalone_phase_time_s;
    uint32_t static_frequency_time_s, static_phase_time_s;
    int64_t applied_word, requested_word, recovery_word;
    uint64_t previous_epoch_ms, qualify_epoch_ms, phase_reference_epoch_ms, lock_epoch_ms;
    uint32_t last_received_ms, observed_ms, applied_ms, pending_ms, qualify_ms;
    uint32_t generation, writes, transitions, reason, attempts, verify_samples;
    /* ramp_ms starts the warm FLL authority ramp; lock_ms the lock dwell. */
    uint32_t ramp_ms, lock_ms, consecutive_rejects, recovery_count;
    sc_hybrid_failure_t last_failure;
    uint64_t unlock_epoch_ms;
    uint32_t unlock_ms, lock_failures;
    uint32_t bias_clip_ps, clipped_samples;
    /* Restart re-anchors: phase step of the latest (new minus old reference,
     * i.e. the holdover phase error left in place) and their running total. */
    double reanchor_step_ns, reanchor_total_ns;
    uint32_t reanchor_count;
    /* Short receiver gaps bridged while tracking (no requalification or
     * re-anchor) and the receiver epochs they skipped. */
    uint32_t bridged_gaps, bridged_epochs;
    /* phase_valid records an anchor's existence, not steering or lock permission. */
    bool enabled, known, boot_known, pending, request_ready, phase_valid;
    bool previous_valid, qualifying, locking, locked, gnss_good;
    bool limited, slew_limited, phase_limited, acquisition_latched;
    bool unlocking, integral_limited, input_clipped;
} sc_hybrid_t;

/* Discard all learned state and load the generated boot policy/bandwidths. */
void sc_hybrid_init(sc_hybrid_t *s);
/* Poll from foreground even without GNSS/USB. A stale stream clears lock and
 * freezes integrators; while tracking, only after the short-gap bridge
 * expires. The independent heater keeps running. */
void sc_hybrid_service(sc_hybrid_t *s, uint32_t now_ms);
/* Evaluate one receiver epoch. The adapter supplies
 * validity flags; the controller additionally checks freshness/continuity.
 * Proposes FCW work only; the platform must execute the transaction below. */
void sc_hybrid_sample(sc_hybrid_t *s, const sc_hybrid_input_t *in);
/* Complete paced probe, after pending writes. Invalid/mismatched readback means
 * UNKNOWN, never a claim that the previous command survived a partial write.
 * Two subsequent consistent valid probes permit automatic requalification. */
void sc_hybrid_observed(sc_hybrid_t *s, bool valid, int64_t word, uint16_t clip, uint32_t now_ms);
/* True reserves one bounded FCW write and returns its signed raw word. The
 * platform must report write completion, then provide an independent readback.
 * False means no write; ignore *word (it may contain a quantized no-op). */
bool sc_hybrid_take_command(sc_hybrid_t *s, uint32_t now_ms, int64_t *word);
/* Report the final word-write completion time. Success remains pending until
 * sc_hybrid_observed verifies it; a bus ACK alone never establishes applied trim. */
void sc_hybrid_written(sc_hybrid_t *s, bool success, uint32_t now_ms);
void sc_hybrid_hold(sc_hybrid_t *s);
void sc_hybrid_resume(sc_hybrid_t *s);
/* Fine-control state, including frequency-only/recovering; not a lock claim. */
bool sc_hybrid_tracking(const sc_hybrid_t *s);
/* Independent frequency indication; all valid modes contribute. */
bool sc_hybrid_frequency_lit(const sc_hybrid_t *s, uint32_t now_ms);
double sc_hybrid_frequency_time(const sc_hybrid_t *s);
double sc_hybrid_phase_time(const sc_hybrid_t *s);
/* Effective mode is derived from the active pair; zero disables that loop. */
sc_hybrid_loop_mode_t sc_hybrid_pair_mode(double frequency_s, double phase_s);
const char *sc_hybrid_loop_name(sc_hybrid_loop_mode_t mode);
/* Pure validation shared by live commands and persisted configuration. */
bool sc_hybrid_parameters_valid(double frequency_s, double phase_s);
/* Live bandwidth changes retain windows/reference and preload the active PI
 * for continuous output. Reject nonfinite/out-of-range values and pending
 * writes. Each time is independently zero (off), or 10..86400 whole seconds.
 * No ratio constraint or automatic persistence. USB shorthand and explicit profile
 * commands both call this single parameter-update API. */
bool sc_hybrid_profile_parameters(sc_hybrid_t *s, sc_hybrid_profile_t profile, double frequency_s,
                                  double phase_s);
/* Per-sample bias clip: the loop uses prediction + clamp(innovation, +/-clip).
 * Independent of output-clock TDEV; a persistent shift passes at clip ns/s. */
bool sc_hybrid_bias_clip(sc_hybrid_t *s, uint32_t picoseconds);
const char *sc_hybrid_state_name(sc_hybrid_state_t state);
#endif
