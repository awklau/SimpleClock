/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_SBF_H
#define SIMPLECLOCK_SBF_H
#include "simpleclock/crc16.h"
#include <stdbool.h>
#include <stdint.h>
#define SC_SBF_PREFIX 128U
#define SC_SBF_ID_MASK 0x1fffU
#define SC_SBF_REVISION_SHIFT 13U

/* Zero-initialize once. Feed/expire from one execution context. Counters are
 * cumulative modulo 2^32; used/length describe the current partial wire frame.
 * Only prefix[] is retained, but CRC and MeasEpoch bounds cover every byte. */

typedef struct
{
    uint8_t prefix[SC_SBF_PREFIX];
    uint16_t used, length;
    simpleclock_crc16_context_t crc;
    uint32_t last_byte_ms, start_ms;
    uint32_t frames, crc_errors, length_errors, timeouts, structure_errors;
    uint32_t meas_next;
    uint8_t meas_remaining;
    bool meas_check, meas_bad;
} sc_sbf_t;

typedef void (*sc_sbf_frame_fn)(void *context, const uint8_t *prefix, uint16_t length,
                                bool structure_valid);
typedef void (*sc_sbf_ascii_fn)(void *context, uint8_t byte);
/* mosaic-G5 Firmware v1.1.0 Reference Guide, section 4.1: callbacks
 * run synchronously and must copy retained data before the next feed. A CRC-valid
 * but structurally invalid body is delivered for raw diagnostics with false;
 * consumers must NOT promote its fields to a valid observation. */
void sc_sbf_feed(sc_sbf_t *parser, const uint8_t *bytes, unsigned count, uint32_t now_ms,
                 sc_sbf_frame_fn frame, sc_sbf_ascii_fn ascii, void *context);
void sc_sbf_expire(sc_sbf_t *parser, uint32_t now_ms, uint32_t baud);
uint16_t sc_sbf_u16(const uint8_t *p);
uint32_t sc_sbf_u32(const uint8_t *p);
#endif
