/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_TEMPERATURE_H
#define SIMPLECLOCK_TEMPERATURE_H
#include <stdbool.h>
#include <stdint.h>

#define SC_TEMPERATURE_STALE_MS 2500U      /* Sample age limit, milliseconds. */
#define SC_TEMPERATURE_CYCLE_MS 1000U      /* Minimum trigger cadence, milliseconds. */
#define SC_TEMPERATURE_CONVERSION_MS 200U  /* One-shot wait after trigger, milliseconds. */
#define SC_TEMPERATURE_READ_WINDOW_MS 100U /* Completion/read-ready window, milliseconds. */

typedef enum
{
    SC_TEMPERATURE_STARTING,
    SC_TEMPERATURE_OK,
    SC_TEMPERATURE_I2C_ERROR,
    SC_TEMPERATURE_BAD_ID,
    SC_TEMPERATURE_BAD_CONFIG,
    SC_TEMPERATURE_INVALID
} sc_temperature_status_t;

typedef struct
{
    uint16_t value; /* MSB-first register value, used only for writes. */
    uint8_t reg;
    bool write;
} sc_temperature_transfer_t;

/* Foreground-owned TMP117 one-shot sequencer. Zero-initialize once. Only this
 * sequencer may configure/consume the sensor (reads clear Data_Ready).
 * begin/complete describe one bounded bus transaction; no blocking or I/O here.
 * EEPROM lock is checked before any volatile configuration write. Repeated
 * values count as new samples only after a commanded conversion and Data_Ready.
 * sample_ms is the trigger write start: a conservative age origin, not the
 * exact conversion time. No config read occurs during the 200 ms conversion wait. */
typedef struct
{
    uint32_t sample_ms, sequence, errors, cycle_ms, transfer_ms, shot_ms;
    uint32_t wait_ms, ready_observed_ms;
    uint16_t device_id, configuration;
    int16_t raw; /* Signed TMP117 counts (1/128 C). */
    uint8_t phase;
    bool pending, started, id_valid, valid;
    sc_temperature_status_t status;
} sc_temperature_t;

/* Foreground-only API. Initialize state to zero before the first begin call;
 * begin publishes one transfer for the caller's bus adapter, and complete must
 * consume that transfer result exactly once. The transfer pointer is caller-
 * owned and only populated while begin returns true. now_ms is modulo-2^32. */
bool sc_temperature_begin(sc_temperature_t *state, uint32_t now_ms,
                          sc_temperature_transfer_t *transfer);
void sc_temperature_complete(sc_temperature_t *state, uint32_t now_ms, bool success,
                             uint16_t value);
bool sc_temperature_fresh(const sc_temperature_t *state, uint32_t now_ms);
#endif
