/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_GNSS_CONTROL_H
#define SIMPLECLOCK_GNSS_CONTROL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SC_GNSS_CONTROL_COMMAND_SIZE 184U
#define SC_GNSS_CONTROL_QUERY_SIZE 64U

/* Stable kind values reported by gnss_control telemetry. */
enum
{
    SC_GNSS_CONTROL_INVALID = 0,
    SC_GNSS_CONTROL_READ = 1,
    SC_GNSS_CONTROL_WRITE = 2,
    SC_GNSS_CONTROL_ACTION = 3
};

/* A bounded expert bridge, not unrestricted UART passthrough. The host catalog
 * supplies field types/units; receiver MIB and readbacks remain authoritative.
 * kind: 1=read, 2=checked write (caller verifies/saves), 3=explicit action. */
unsigned sc_gnss_control_prepare(const char *usb, char *command, size_t capacity, char *query,
                                 size_t query_capacity);
/* Match one native setting line to the requested selector. Requested blank
 * arguments are preserved, not defaults. Hash omitted fields before/after.
 * Numeric formatting zeros and list ordering do not create false changes. */
bool sc_gnss_control_readback(const char *command, const char *line, bool *matches,
                              uint32_t *preserved);
/* Strict complete PVTMode row eligible for clock control. selection uses
 * SC_SETTINGS_GNSS_* values. Static is eligible only without PPP, with an
 * explicit valid RefPos; actual solution/integrity still qualify separately. */
bool sc_gnss_control_pvt_policy(const char *line, uint32_t *selection, bool *fixed);

/* Full signal/satellite lists can exceed the transaction's echo line buffer.
 * Exact sets fit in 256 bits: 30 signals or 230 documented satellite IDs.
 * Expected includes unchanged fields; +/- edits derive it from actual Current. */
typedef struct
{
    uint32_t wanted[2][8], expected[2][8], actual[2][8];
    char word[32];
    int8_t mode[2];
    uint8_t family, field, used;
    bool selected, before, bad;
} sc_gnss_control_lists_t;

/* Foreground-only, one state per active edit. init clears state and returns
 * whether the full-name setter is a list family; bad marks invalid arguments.
 * start resets parsing for each before/after query, retaining the target and
 * any failure. Feed only reply bytes after the matching command echo. */
bool sc_gnss_control_lists_init(sc_gnss_control_lists_t *state, const char *command);
void sc_gnss_control_lists_start(sc_gnss_control_lists_t *state, bool before);
/* True once per matching completed row, updating matches. Caller must require
 * exactly one row and no bad flag, timeout or transport loss before proceeding. */
bool sc_gnss_control_lists_byte(sc_gnss_control_lists_t *state, uint8_t byte, bool *matches);
#endif
