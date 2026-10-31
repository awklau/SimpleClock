/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_PREHEAT_H
#define SIMPLECLOCK_PREHEAT_H
#include <stdbool.h>
#include <stdint.h>

#define SC_PREHEAT_MIN_MS 300000U      /* Minimum elapsed qualification time, milliseconds. */
#define SC_PREHEAT_TIMEOUT_MS 1800000U /* Evidence timeout from init, milliseconds. */
#define SC_PREHEAT_WINDOW 120U         /* One power sample per nominal second. */

/* Section 6's provisional power-plateau rule. Foreground only, one snapshot/s.
 * The bench must establish a meaningful power range and readout coherence before
 * evidence_verified can permit readiness. A flat number alone proves neither.
 * Scratch is bounded on the caller's stack; no floats, allocation or waits. */
typedef struct
{
    uint32_t power[SC_PREHEAT_WINDOW];     /* Device power code, raw units. */
    uint32_t sample_ms[SC_PREHEAT_WINDOW]; /* MCU observation time, milliseconds. */
    uint32_t start_ms, last_ms, first_ms;  /* MCU times, modulo-2^32 milliseconds. */
    uint32_t p05, median, p95;             /* Percentiles of the retained power codes. */
    unsigned count, next;                  /* Valid count and circular write index. */
    bool observed, plateau, complete_inferred, unconfirmed;
} sc_preheat_t;

/* Foreground-only API. state is caller-owned, zeroed by init, and retains no
 * caller buffers. push accepts at most one sample per roughly one-second slot;
 * expire may be called independently to apply stale-sensor/timeout failure. */
void sc_preheat_init(sc_preheat_t *state, uint32_t now_ms);
void sc_preheat_push(sc_preheat_t *state, uint32_t now_ms, uint32_t power_code, bool device_valid,
                     bool sensor_fresh, bool evidence_verified);
void sc_preheat_expire(sc_preheat_t *state, uint32_t now_ms, bool sensor_fresh);
#endif
