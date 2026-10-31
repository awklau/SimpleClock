/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_TIMING_H
#define SIMPLECLOCK_TIMING_H
#include "simpleclock/sbf_fields.h"

#define SC_EPOCH_SLOT_COUNT 4U        /* Maximum incomplete receiver epochs retained. */
#define SC_EPOCH_SOURCE_COUNT 5U      /* PVT, status, time, measurement, EndOfPVT. */
#define SC_PPP_MAX_CORR_AGE_CS 12000U /* Strictly below 120 s; source unit 0.01 s. */

/* Foreground-only observation assembly. Zero-initialize once; push copies the
 * decoded observation, poll publishes the oldest slot after a settling window.
 * A segment is a run without a detected discontinuity, not a calibrated clock.
 * Source-index/bit order is PVT, ReceiverStatus, ReceiverTime, MeasEpoch, EndOfPVT. */

enum
{
    SC_EPOCH_PVT = 1U,
    SC_EPOCH_STATUS = 2U,
    SC_EPOCH_TIME = 4U,
    SC_EPOCH_MEAS = 8U,
    SC_EPOCH_END = 16U,
    SC_EPOCH_ALL = 31U
};

enum
{
    SC_SEG_START = 1U,
    SC_SEG_CONFIG = 2U,
    SC_SEG_RESET = 4U,
    SC_SEG_GAP = 8U,
    SC_SEG_LOSS = 16U,
    SC_SEG_JUMP = 32U,
    SC_SEG_SYSTEM = 64U,
    SC_SEG_REFERENCE = 128U,
    SC_SEG_INCOMPLETE = 256U,
    SC_SEG_DUPLICATE = 512U,
    SC_SEG_QUALITY = 1024U
};

typedef struct
{
    sc_sbf_observation_t pvt;
    /* first_ms/source_ms are MCU frame-completion times (HAL tick domain).
     * tow_ms/week identify the receiver epoch; rx_uptime is in receiver seconds. */
    uint32_t first_ms, source_ms[SC_EPOCH_SOURCE_COUNT], tow_ms, rx_uptime, rx_state, rx_error;
    uint32_t sequence, segment, reasons; /* Local publication sequence/segment/reason bits. */
    uint16_t week;
    uint8_t mask, duplicate, sync_level, common_flags, jumps, ext_error;
    bool jumps_valid, sources_valid;
} sc_epoch_t;

typedef struct
{
    /* Fixed slots bound RAM usage. Published 'last' is immutable to late data. */
    sc_epoch_t slots[SC_EPOCH_SLOT_COUNT], last;
    uint32_t segment, pending_reasons, sequence, late, duplicates, discarded;
    uint32_t receiver_boot, status_uptime;
    bool have_last, have_status;
} sc_epochs_t;

/* Foreground-only assembly API. Zero-initialize state before first use; push
 * copies an accepted observation and retains no caller storage. poll copies a
 * settled slot to out, which is caller-owned. now_ms is modulo-2^32 MCU ms.
 * Observation times are MCU frame completion times, never physical PPS times;
 * no clock-jump unwrapping or absolute PPS association is inferred. */
void sc_epochs_break(sc_epochs_t *state, uint32_t reason);
void sc_epochs_push(sc_epochs_t *state, const sc_sbf_observation_t *observation, uint32_t now_ms);
bool sc_epochs_poll(sc_epochs_t *state, uint32_t now_ms, sc_epoch_t *out);
bool sc_epochs_pps_ready(const sc_epochs_t *state, uint32_t now_ms);
/* LED2 fix indication only: newest valid PVT, fresh within 1.5 s, no integrity
 * alert. Does not require EXT_FREQ/PPS or infer PPP qualification. */
bool sc_epochs_fix_valid(const sc_epochs_t *state, uint32_t now_ms);
/* G5 guide p.25/269: PPP mode 10, GST, orbit/clock-use bit and valid correction
 * age. This observation is separate from convergence or clock qualification. */
bool sc_pvt_has_corrections(const sc_sbf_observation_t *pvt);
/* User-selected LED2 policy: newest fresh valid PVT reports mode 10 (PPP).
 * No MCU dwell, correction-age gate or convergence claim. */
bool sc_epochs_ppp_active(const sc_epochs_t *state, uint32_t now_ms);

typedef enum
{
    SC_PPS_ABSENT,
    SC_PPS_ACQUIRING,
    SC_PPS_PRESENT,
    SC_PPS_LOST
} sc_pps_state_t;

typedef struct
{
    uint32_t tick;           /* Timer capture ticks, nominally 1 MHz, modulo-2^32. */
    uint32_t ms;             /* ISR service time in MCU milliseconds, modulo-2^32. */
    uint32_t sequence;       /* Monotonic edge sequence supplied by the producer. */
    uint32_t interval_us;    /* Unsigned tick delta interpreted as microseconds. */
    unsigned good_intervals; /* Consecutive intervals within the presence band. */
    bool seen, interval_valid;
    sc_pps_state_t state;
} sc_pps_t;

/* Tick is a modulo-2^32 hardware capture at nominal 1 MHz. The millisecond
 * time is ISR service time and bounds wrap ambiguity after long gaps. */
void sc_pps_edge(sc_pps_t *state, uint32_t tick, uint32_t ms, uint32_t sequence, bool loss);
void sc_pps_expire(sc_pps_t *state, uint32_t now_ms);
#endif
