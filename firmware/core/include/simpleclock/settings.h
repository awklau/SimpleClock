/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_SETTINGS_H
#define SIMPLECLOCK_SETTINGS_H
#include <stdbool.h>
#include <stdint.h>

enum
{
    SC_SETTINGS_GNSS_AUTO,
    SC_SETTINGS_GNSS_SBAS,
    SC_SETTINGS_GNSS_HAS,
    SC_SETTINGS_GNSS_STANDALONE
};

enum
{
    SC_SETTINGS_PPS_CONTINUOUS,
    SC_SETTINGS_PPS_QUALIFIED,
    SC_SETTINGS_PPS_OFF,
    SC_SETTINGS_PPS_MANUAL
};

/* Operator configuration only: never persist trim, phase reference, integrals,
 * actuator ownership, heater trials or lock/qualification state. */
typedef struct
{
    uint32_t gnss_selection;
    uint32_t ppp_frequency_s, ppp_phase_s, sbas_frequency_s, sbas_phase_s;
    uint32_t standalone_frequency_s, standalone_phase_s;
    uint32_t receiver_user_profile;
    uint32_t pps_policy;
    uint32_t bias_clip_ps;
    uint32_t static_frequency_s, static_phase_s; /* Static PVT fixed-position pair. */
} sc_settings_t;

/* v5 record; v1-v4 records are 64 bytes and are read in place (the tail of a
 * slot holding one is erased flash). */
#define SC_SETTINGS_RECORD_BYTES 72U

typedef struct
{
    void *context;
    bool (*read)(void *context, unsigned slot, uint8_t record[SC_SETTINGS_RECORD_BYTES]);
    bool (*erase)(void *context, unsigned slot);
    bool (*program)(void *context, unsigned slot, unsigned offset, const uint8_t bytes[8]);
} sc_settings_io_t;

typedef enum
{
    SC_SETTINGS_EMPTY,
    SC_SETTINGS_VALID,
    SC_SETTINGS_INVALID,
    SC_SETTINGS_IO_ERROR
} sc_settings_status_t;

void sc_settings_defaults(sc_settings_t *out);
bool sc_settings_valid(const sc_settings_t *settings);
bool sc_settings_equal(const sc_settings_t *a, const sc_settings_t *b);
/* Stable little-endian schema, CRC-32/ISO-HDLC and a separate final commit.
 * Decode validates the complete record and semantic bounds before returning. */
bool sc_settings_encode(uint8_t record[SC_SETTINGS_RECORD_BYTES], const sc_settings_t *settings,
                        uint32_t generation);
bool sc_settings_decode(const uint8_t record[SC_SETTINGS_RECORD_BYTES], sc_settings_t *settings,
                        uint32_t *generation);
/* Invalid/unwritten storage returns compiled defaults. Newer generation wins,
 * including uint32 wrap. Two slots must be in independently erasable pages. */
sc_settings_status_t sc_settings_load(const sc_settings_io_t *io, sc_settings_t *out,
                                      uint32_t *generation, unsigned *slot);
/* Copy-on-write: erase inactive slot, verify payload, commit last, read back.
 * Identical settings avoid erase/program. Failure preserves the previous slot. */
bool sc_settings_save(const sc_settings_io_t *io, const sc_settings_t *settings);
#endif
