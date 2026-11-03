/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_SIT5801_H
#define SIMPLECLOCK_SIT5801_H

#include <stdbool.h>
#include <stdint.h>

/* Register map and FCW format: SiTime SiT58xx/SiT71xx digital control documentation. */
#define SIT5801_ADDRESS_7BIT 0x50U
#define SIT5801_READ_GAP_MS 50U
#define SIT5801_FCW_MAX INT64_C(274877906943)
#define SIT5801_FCW_MIN (-INT64_C(274877906944))

/* q is a signed 39-bit FCW. words are MSB/middle/LSB register values; transport
 * byte order is a separate concern. These codecs never write the oscillator. */
bool sit5801_fcw_encode(int64_t q, uint16_t words[3]);
int64_t sit5801_fcw_decode(const uint16_t words[3]);
/* ppb is fractional frequency offset * 1e9. The documented nominal full scale
 * is +/-800,000 ppb; device clipping can be tighter. No tuning-sign claim is
 * made until a physical FCW step is measured. */
bool sit5801_ppb_to_fcw(double ppb, int64_t *q);
double sit5801_fcw_to_ppb(int64_t q);
double sit5801_clip_ppb(uint16_t clip_register);
uint32_t sit5801_power_code(uint16_t high, uint16_t low);

typedef struct
{
    uint16_t wire[8]; /* byte 0 in bits 15:8, byte 1 in bits 7:0; no assumed order */
    uint32_t completed_ms;
    uint32_t errors;
    uint32_t sequence;
    uint8_t index;
    uint8_t valid_mask;
    bool pending;
    bool started;
} sit5801_probe_t;

/* One read at a time; caller supplies completion time even after a failure.
 * No writes, retries, sleeps, or transport-specific address shifts here. */
bool sit5801_probe_begin(sit5801_probe_t *probe, uint32_t now_ms, uint8_t *reg);
void sit5801_probe_complete(sit5801_probe_t *probe, uint32_t now_ms, bool success,
                            const uint8_t bytes[2]);
/* 0 = unknown/ambiguous, 1 = MSB first, 2 = LSB first. Corroborates ID and
 * expected factory clip=32, and is evidence for this read probe only. */
unsigned sit5801_probe_word_order(const sit5801_probe_t *probe);
uint16_t sit5801_probe_word(const sit5801_probe_t *probe, unsigned index, unsigned order);
#endif
