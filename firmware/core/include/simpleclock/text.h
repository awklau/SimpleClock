/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_TEXT_H
#define SIMPLECLOCK_TEXT_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Allocation-free append-only telemetry builder. capacity includes the NUL.
 * Overflow latches ok=false, keeps the existing NUL-terminated prefix and makes
 * later appends no-ops. Callers must check ok before publishing a record.
 * The caller owns data for the builder's lifetime; string inputs are non-NULL. */

typedef struct
{
    char *data;
    size_t capacity, length;
    bool ok;
} sc_text_t;

void sc_text_init(sc_text_t *text, char *data, size_t capacity);
void sc_text_string(sc_text_t *text, const char *value);
void sc_text_u32(sc_text_t *text, uint32_t value);
void sc_text_i32(sc_text_t *text, int32_t value);
void sc_text_u64(sc_text_t *text, uint64_t value);
void sc_text_i64(sc_text_t *text, int64_t value);
/* Signed Q16 value rendered to nearest 0.001 unit; no printf/float dependency. */
void sc_text_q16(sc_text_t *text, int32_t value);
void sc_text_hex(sc_text_t *text, uint32_t value, unsigned digits);
/* Exact decimal Celsius for TMP117 counts (1/128 C). Caller checks validity. */
void sc_text_temperature(sc_text_t *text, int16_t raw);
#endif
