/* SPDX-License-Identifier: MIT */
#include "simpleclock/settings.h"
#include "simpleclock/hybrid.h"
#include <string.h>

#define MAGIC UINT32_C(0x53434346)
#define COMMIT UINT32_C(0x53415645)
#define VERSION 5U
/* Each layout ends with CRC, ~CRC, commit, ~commit (16 bytes). v1-v4 records
 * are 64 bytes; v5 appends the static pair before the CRC (72 bytes). */
#define LEGACY_BYTES 64U
#define CRC_OFFSET(bytes) ((bytes) - 16U)
#define COMMIT_OFFSET(bytes) ((bytes) - 8U)

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8U | (uint32_t)p[2] << 16U | (uint32_t)p[3] << 24U;
}

static void put32(uint8_t *p, uint32_t value)
{
    for (unsigned i = 0U; i < 4U; i++)
    {
        p[i] = (uint8_t)(value >> (8U * i));
    }
}

static uint32_t crc32(const uint8_t *bytes, unsigned length)
{
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0U; i < length; i++)
    {
        crc ^= bytes[i];
        for (unsigned bit = 0U; bit < 8U; bit++)
        {
            crc = (crc >> 1U) ^ ((crc & 1U) ? UINT32_C(0xedb88320) : 0U);
        }
    }
    return ~crc;
}

void sc_settings_defaults(sc_settings_t *out)
{
    *out = (sc_settings_t){SC_SETTINGS_GNSS_AUTO,
                           (uint32_t)SC_HYBRID_CLOCK_FREQUENCY_TIME_CONSTANT_S,
                           (uint32_t)SC_HYBRID_CLOCK_PHASE_NATURAL_TIME_S,
                           (uint32_t)SC_HYBRID_CLOCK_FALLBACK_FREQUENCY_TIME_CONSTANT_S,
                           (uint32_t)SC_HYBRID_CLOCK_FALLBACK_PHASE_NATURAL_TIME_S,
                           (uint32_t)SC_HYBRID_CLOCK_FALLBACK_FREQUENCY_TIME_CONSTANT_S,
                           (uint32_t)SC_HYBRID_CLOCK_FALLBACK_PHASE_NATURAL_TIME_S,
                           0U,
                           SC_SETTINGS_PPS_CONTINUOUS,
                           SC_HYBRID_BIAS_CLIP_DEFAULT_PS,
                           (uint32_t)SC_HYBRID_CLOCK_STATIC_FREQUENCY_TIME_CONSTANT_S,
                           (uint32_t)SC_HYBRID_CLOCK_STATIC_PHASE_NATURAL_TIME_S};
}

bool sc_settings_valid(const sc_settings_t *s)
{
    return s != NULL && s->gnss_selection <= SC_SETTINGS_GNSS_STANDALONE &&
           s->receiver_user_profile <= 1U && s->pps_policy <= SC_SETTINGS_PPS_MANUAL &&
           s->bias_clip_ps >= 1U && s->bias_clip_ps <= SC_HYBRID_BIAS_CLIP_MAX_PS &&
           sc_hybrid_parameters_valid((double)s->ppp_frequency_s, (double)s->ppp_phase_s) &&
           sc_hybrid_parameters_valid((double)s->sbas_frequency_s, (double)s->sbas_phase_s) &&
           sc_hybrid_parameters_valid((double)s->standalone_frequency_s,
                                      (double)s->standalone_phase_s) &&
           sc_hybrid_parameters_valid((double)s->static_frequency_s, (double)s->static_phase_s);
}

bool sc_settings_equal(const sc_settings_t *a, const sc_settings_t *b)
{
    return a->gnss_selection == b->gnss_selection && a->ppp_frequency_s == b->ppp_frequency_s &&
           a->ppp_phase_s == b->ppp_phase_s && a->sbas_frequency_s == b->sbas_frequency_s &&
           a->sbas_phase_s == b->sbas_phase_s &&
           a->standalone_frequency_s == b->standalone_frequency_s &&
           a->standalone_phase_s == b->standalone_phase_s &&
           a->receiver_user_profile == b->receiver_user_profile && a->pps_policy == b->pps_policy &&
           a->bias_clip_ps == b->bias_clip_ps && a->static_frequency_s == b->static_frequency_s &&
           a->static_phase_s == b->static_phase_s;
}

bool sc_settings_encode(uint8_t record[SC_SETTINGS_RECORD_BYTES], const sc_settings_t *s,
                        uint32_t generation)
{
    if (!sc_settings_valid(s))
    {
        return false;
    }
    memset(record, 0, SC_SETTINGS_RECORD_BYTES);
    put32(record, MAGIC);
    put32(record + 4U, VERSION);
    put32(record + 8U, generation);
    put32(record + 12U, SC_SETTINGS_RECORD_BYTES);
    put32(record + 16U, s->gnss_selection);
    put32(record + 20U, s->ppp_frequency_s);
    put32(record + 24U, s->ppp_phase_s);
    put32(record + 28U, s->sbas_frequency_s);
    put32(record + 32U, s->sbas_phase_s);
    put32(record + 36U, s->standalone_frequency_s);
    put32(record + 40U, s->standalone_phase_s);
    /* Word 44 (since v4) packs bounded options into the former profile-marker
     * word. Bits 3..6 are reserved (a removed filter option): written 0, ignored. */
    put32(record + 44U, s->receiver_user_profile | (s->pps_policy << 1U) | (s->bias_clip_ps << 7U));
    put32(record + 48U, s->static_frequency_s);
    put32(record + 52U, s->static_phase_s);
    const uint32_t crc = crc32(record, CRC_OFFSET(SC_SETTINGS_RECORD_BYTES));
    put32(record + CRC_OFFSET(SC_SETTINGS_RECORD_BYTES), crc);
    put32(record + CRC_OFFSET(SC_SETTINGS_RECORD_BYTES) + 4U, ~crc);
    put32(record + COMMIT_OFFSET(SC_SETTINGS_RECORD_BYTES), COMMIT);
    put32(record + COMMIT_OFFSET(SC_SETTINGS_RECORD_BYTES) + 4U, ~COMMIT);
    return true;
}

bool sc_settings_decode(const uint8_t record[SC_SETTINGS_RECORD_BYTES], sc_settings_t *s,
                        uint32_t *generation)
{
    const uint32_t version = get32(record + 4U);
    /* Older layouts never read past their own 64 bytes. */
    const uint32_t bytes = version < VERSION ? LEGACY_BYTES : SC_SETTINGS_RECORD_BYTES;
    const uint32_t crc = get32(record + CRC_OFFSET(bytes));
    if (get32(record) != MAGIC || version < 1U || version > VERSION ||
        get32(record + 12U) != bytes || (version < 3U && get32(record + 44U) != 0U) ||
        get32(record + CRC_OFFSET(bytes) + 4U) != ~crc || crc32(record, CRC_OFFSET(bytes)) != crc ||
        get32(record + COMMIT_OFFSET(bytes)) != COMMIT ||
        get32(record + COMMIT_OFFSET(bytes) + 4U) != ~COMMIT)
    {
        return false;
    }
    sc_settings_t decoded;
    if (version == 1U)
    {
        const uint32_t mode = get32(record + 16U);
        const uint32_t pf = get32(record + 24U), pp = get32(record + 28U);
        const uint32_t ff = get32(record + 32U), fp = get32(record + 36U);
        /* Validate the old schema before migration. Legacy single-loop
         * records still stored positive values for the unused time. */
        if (get32(record + 40U) != 0U || mode > SC_HYBRID_LOOP_PHASE || pf < 10U || pp < 10U ||
            ff < 10U || fp < 10U || pf > 86400U || pp > 86400U || ff > 86400U || fp > 86400U)
        {
            return false;
        }
        decoded = (sc_settings_t){get32(record + 20U),
                                  mode == SC_HYBRID_LOOP_PHASE ? 0U : pf,
                                  mode == SC_HYBRID_LOOP_FREQUENCY ? 0U : pp,
                                  mode == SC_HYBRID_LOOP_PHASE ? 0U : ff,
                                  mode == SC_HYBRID_LOOP_FREQUENCY ? 0U : fp,
                                  mode == SC_HYBRID_LOOP_PHASE ? 0U : ff,
                                  mode == SC_HYBRID_LOOP_FREQUENCY ? 0U : fp,
                                  0U,
                                  SC_SETTINGS_PPS_CONTINUOUS,
                                  SC_HYBRID_BIAS_CLIP_DEFAULT_PS,
                                  mode == SC_HYBRID_LOOP_PHASE ? 0U : ff,
                                  mode == SC_HYBRID_LOOP_FREQUENCY ? 0U : fp};
    }
    else
    {
        /* Before v5, static fixed-position solutions used the standalone pair. */
        decoded = (sc_settings_t){
            get32(record + 16U),
            get32(record + 20U),
            get32(record + 24U),
            get32(record + 28U),
            get32(record + 32U),
            get32(record + 36U),
            get32(record + 40U),
            version >= 3U ? get32(record + 44U) & 1U : 0U,
            version >= 4U ? (get32(record + 44U) >> 1U) & 3U : SC_SETTINGS_PPS_CONTINUOUS,
            version >= 4U ? get32(record + 44U) >> 7U : SC_HYBRID_BIAS_CLIP_DEFAULT_PS,
            version >= 5U ? get32(record + 48U) : get32(record + 36U),
            version >= 5U ? get32(record + 52U) : get32(record + 40U)};
        if (version == 3U && get32(record + 44U) > 1U)
        {
            return false;
        }
    }
    if (!sc_settings_valid(&decoded))
    {
        return false;
    }
    *s = decoded;
    *generation = get32(record + 8U);
    return true;
}

sc_settings_status_t sc_settings_load(const sc_settings_io_t *io, sc_settings_t *out,
                                      uint32_t *generation, unsigned *slot)
{
    sc_settings_defaults(out);
    *generation = 0U;
    *slot = 0U;
    bool found = false, nonempty = false;
    for (unsigned i = 0U; i < 2U; i++)
    {
        uint8_t record[SC_SETTINGS_RECORD_BYTES];
        if (!io->read(io->context, i, record))
        {
            return SC_SETTINGS_IO_ERROR;
        }
        for (unsigned j = 0U; j < SC_SETTINGS_RECORD_BYTES; j++)
        {
            nonempty |= record[j] != UINT8_MAX;
        }
        sc_settings_t decoded;
        uint32_t seq;
        if (sc_settings_decode(record, &decoded, &seq) &&
            (!found ||
             (seq != *generation && (uint32_t)(seq - *generation) < UINT32_C(0x80000000))))
        {
            *out = decoded;
            *generation = seq;
            *slot = i;
            found = true;
        }
    }
    return found ? SC_SETTINGS_VALID : nonempty ? SC_SETTINGS_INVALID : SC_SETTINGS_EMPTY;
}

bool sc_settings_save(const sc_settings_io_t *io, const sc_settings_t *settings)
{
    if (!sc_settings_valid(settings))
    {
        return false;
    }
    sc_settings_t previous;
    uint32_t generation;
    unsigned slot;
    const sc_settings_status_t status = sc_settings_load(io, &previous, &generation, &slot);
    if (status == SC_SETTINGS_IO_ERROR)
    {
        return false;
    }
    if (status == SC_SETTINGS_VALID && sc_settings_equal(&previous, settings))
    {
        return true;
    }
    slot = status == SC_SETTINGS_VALID ? 1U - slot : 0U;
    uint8_t record[SC_SETTINGS_RECORD_BYTES], readback[SC_SETTINGS_RECORD_BYTES];
    if (!sc_settings_encode(record, settings, generation + 1U) || !io->erase(io->context, slot))
    {
        return false;
    }
    if (!io->read(io->context, slot, readback))
    {
        return false;
    }
    for (unsigned i = 0U; i < SC_SETTINGS_RECORD_BYTES; i++)
    {
        if (readback[i] != UINT8_MAX)
        {
            return false;
        }
    }
    const unsigned commit = COMMIT_OFFSET(SC_SETTINGS_RECORD_BYTES);
    for (unsigned offset = 0U; offset < commit; offset += 8U)
    {
        if (!io->program(io->context, slot, offset, record + offset))
        {
            return false;
        }
    }
    if (!io->read(io->context, slot, readback) || memcmp(record, readback, commit) != 0)
    {
        return false;
    }
    for (unsigned i = commit; i < SC_SETTINGS_RECORD_BYTES; i++)
    {
        if (readback[i] != UINT8_MAX)
        {
            return false;
        }
    }
    return io->program(io->context, slot, commit, record + commit) &&
           io->read(io->context, slot, readback) &&
           memcmp(record, readback, SC_SETTINGS_RECORD_BYTES) == 0;
}
