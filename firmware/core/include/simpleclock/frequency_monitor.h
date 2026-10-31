/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_FREQUENCY_MONITOR_H
#define SIMPLECLOCK_FREQUENCY_MONITOR_H
#include "simpleclock/hybrid_defaults.h"
#include <stdbool.h>
#include <stdint.h>

#define SC_FREQUENCY_POINTS SC_HYBRID_CLOCK_FREQUENCY_SCATTER_SAMPLES
#define SC_FREQUENCY_MEAN_POINTS SC_HYBRID_CLOCK_FREQUENCY_SETTLE_SAMPLES
#define SC_FREQUENCY_HOLDOVER_MS                                                                   \
    ((uint32_t)(SC_HYBRID_CLOCK_FREQUENCY_MISSING_MEASUREMENT_GRACE_S * 1000.0))
_Static_assert(SC_FREQUENCY_MEAN_POINTS >= 2U && SC_FREQUENCY_MEAN_POINTS <= SC_FREQUENCY_POINTS,
               "frequency means must fit the scatter history");

/* Independent, read-only frequency assessment. A missing measurement never
 * becomes a zero. Short gaps retain measured history, with a bounded total
 * gap allowance; no control state, temperature or PPP-mode gate is involved.
 * Setpoints are bounded to +/-0.1 ppb by the controller. Q18 storage saves RAM;
 * half an LSB is conservatively included in the tracking-error test.
 * Zero-initialize before first use. All calls are foreground-only, require a
 * non-NULL state and retain no caller storage. */
typedef struct
{
    float drift[SC_FREQUENCY_POINTS];
    int16_t setpoint_q18[SC_FREQUENCY_MEAN_POINTS];
    double mean_drift_ppb, mean_error_ppb, drift_sd_ppb;
    uint64_t last_epoch_ms, first_epoch_ms, since_epoch_ms;          /* Receiver week/TOW domain. */
    uint32_t last_ms, first_ms, since_ms, gaps_ms, previous_gaps_ms; /* MCU domain/durations. */
    uint16_t count, next, setpoint_next, block_count;
    bool window_valid, timing, settled;
} sc_frequency_monitor_t;

/* Poll even without measurements; clears history/settled after the grace period.
 * now_ms is modulo-2^32 MCU time, serviced regularly to disambiguate tick wrap. */
void sc_frequency_monitor_service(sc_frequency_monitor_t *s, uint32_t now_ms);
/* Supply a quality-qualified measurement and its contemporaneous requested
 * drift, both in ppb. received_ms is MCU PVT frame-completion time, not host/USB
 * arrival; epoch_ms is receiver week/TOW in milliseconds. Cached epochs cannot
 * refresh the indication. Invalid/stale samples are ignored after expiry is
 * serviced. Backward/subsecond epochs or gaps beyond the allowance restart
 * history. No control is applied. */
void sc_frequency_monitor_sample(sc_frequency_monitor_t *s, uint64_t epoch_ms, uint32_t received_ms,
                                 uint32_t now_ms, double drift_ppb, double setpoint_ppb);
#endif
