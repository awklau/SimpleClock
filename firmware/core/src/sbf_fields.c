/* SPDX-License-Identifier: MIT */
/* Decode only supported SBF revisions into raw source fields and explicit validity bits.
 * Wire offsets below include the 8-byte SBF header; no packed-struct casts are used. */
#include "simpleclock/sbf_fields.h"
#include "simpleclock/sbf.h"
#include <float.h>
#include <limits.h>
#include <math.h>
#include <string.h>

_Static_assert(CHAR_BIT == 8 && sizeof(float) == 4 && sizeof(double) == 8,
               "SBF requires IEEE binary32/binary64");
_Static_assert(FLT_RADIX == 2 && FLT_MANT_DIG == 24 && DBL_MANT_DIG == 53 && FLT_MAX_EXP == 128 &&
                   DBL_MAX_EXP == 1024,
               "IEEE representation");

static uint64_t read_u64_le(const uint8_t *prefix)
{
    return (uint64_t)sc_sbf_u32(prefix) | ((uint64_t)sc_sbf_u32(prefix + 4) << 32);
}

static int8_t decode_i8(uint8_t x)
{
    return (int8_t)(x < 128U ? (int)x : (int)x - 256);
}

bool sc_sbf_bias_seconds(uint64_t bits, double *seconds)
{
    double value;
    memcpy(&value, &bits, sizeof(value));
    if (!isfinite(value) || value == -2.0e10 || seconds == NULL)
    {
        return false;
    }
    *seconds = value * 1.0e-3;
    return true;
}

bool sc_sbf_drift_fraction(uint32_t bits, double *fraction)
{
    float value;
    memcpy(&value, &bits, sizeof(value));
    if (!isfinite(value) || value == -2.0e10F || fraction == NULL)
    {
        return false;
    }
    *fraction = (double)value * 1.0e-6;
    return true;
}

bool sc_sbf_covariance_value(uint32_t bits, double *value)
{
    float raw;
    if (value == NULL)
    {
        return false;
    }
    memcpy(&raw, &bits, sizeof(raw));
    if (!isfinite(raw) || raw < 0.0F || raw == -2.0e10F)
    {
        return false;
    }
    *value = (double)raw;
    return true;
}

bool sc_sbf_formal_sigma(uint32_t bits, double *sigma)
{
    double value;
    if (sigma == NULL || !sc_sbf_covariance_value(bits, &value))
    {
        return false;
    }
    *sigma = sqrt(value) / 299792458.0;
    return isfinite(*sigma);
}

bool sc_sbf_dop_value(uint16_t hundredths, double *value)
{
    if (hundredths == 0U || value == NULL)
    {
        return false;
    }
    *value = (double)hundredths / 100.0;
    return true;
}

static bool known_time_system(uint8_t system)
{
    return system == 0U || system == 1U || system == 3U || system == 4U || system == 5U ||
           system == 100U;
}

static bool known_fix(uint8_t mode)
{
    const uint8_t fix_mode = mode & SC_SBF_PVT_MODE_MASK;
    return (fix_mode >= 1U && fix_mode <= 8U) || fix_mode == SC_SBF_PVT_MODE_PPP;
}

sc_sbf_decode_result_t sc_sbf_decode(const uint8_t *prefix, size_t retained, uint16_t wire_length,
                                     sc_sbf_observation_t *out)
{
    memset(out, 0, sizeof(*out));
    if (retained < 8U || wire_length < 8U || (wire_length & 3U) != 0U || prefix[0] != '$' ||
        prefix[1] != '@' || sc_sbf_u16(prefix + 6) != wire_length)
    {
        return SC_SBF_DECODE_LENGTH;
    }
    const uint16_t id = sc_sbf_u16(prefix + 4) & SC_SBF_ID_MASK;
    const uint8_t revision = (uint8_t)(sc_sbf_u16(prefix + 4) >> SC_SBF_REVISION_SHIFT);
    unsigned required_length, max_revision;
    switch (id)
    {
    case SC_SBF_PVT:
        required_length = revision >= 2U ? 96U : 88U;
        max_revision = 3U;
        break;
    case SC_SBF_STATUS:
        required_length = 32U;
        max_revision = 1U;
        break;
    case SC_SBF_MEAS:
        required_length = 20U;
        max_revision = 1U;
        break;
    case SC_SBF_TIME:
        required_length = 24U;
        max_revision = 0U;
        break;
    case SC_SBF_END:
        required_length = 16U;
        max_revision = 0U;
        break;
    case SC_SBF_GAL_CNAV:
        required_length = 84U;
        max_revision = 0U;
        break;
    case SC_SBF_RF_STATUS:
        /* RFStatus has N, SBLength, Flags and three reserved bytes after
         * the common time header.  RFBand entries are optional here; the
         * host needs the authenticated-output warning flags only. */
        required_length = 20U;
        max_revision = 0U;
        break;
    case SC_SBF_POS_COV_CARTESIAN:
    case SC_SBF_VEL_COV_CARTESIAN:
        required_length = 56U;
        max_revision = 0U;
        break;
    case SC_SBF_DOP:
        /* mosaic-G5 Firmware v1.1.0 Reference Guide p. 279: common time
         * header, NrSV/reserved, four u2 DOPs, then two f4 protection levels. */
        required_length = 32U;
        max_revision = 0U;
        break;
    default:
        return SC_SBF_DECODE_UNKNOWN;
    }
    if (revision > max_revision)
    {
        return SC_SBF_DECODE_REVISION;
    }
    if (retained < required_length || wire_length < required_length)
    {
        return SC_SBF_DECODE_LENGTH;
    }
    if (id == SC_SBF_STATUS)
    {
        const unsigned body = 32U + (unsigned)prefix[28] * prefix[29];
        if ((prefix[28] != 0U && prefix[29] < 4U) || body > wire_length)
        {
            return SC_SBF_DECODE_LENGTH;
        }
    }
    out->id = id;
    out->revision = revision;
    out->length = wire_length;
    out->tow_ms = sc_sbf_u32(prefix + 8);
    out->week = sc_sbf_u16(prefix + 12);
    if (out->tow_ms < SC_SBF_WEEK_MS && out->week != UINT16_MAX)
    {
        out->valid = SC_SBF_VALID_EPOCH;
    }
    switch (id)
    {
    case SC_SBF_PVT:
    {
        out->data.pvt.mode = prefix[14];
        out->data.pvt.error = prefix[15];
        out->data.pvt.bias_bits = read_u64_le(prefix + 60);
        out->data.pvt.drift_bits = sc_sbf_u32(prefix + 68);
        out->data.pvt.time_system = prefix[72];
        out->data.pvt.nr_sv = prefix[74];
        out->data.pvt.wa_corr_info = prefix[75];
        out->data.pvt.mean_corr_age = sc_sbf_u16(prefix + 78);
        out->data.pvt.signal_info = sc_sbf_u32(prefix + 80);
        out->data.pvt.alert = prefix[84];
        /* Rev 0 ends at AlertFlag; later fields must not read old padding. */
        if (revision >= 1U)
        {
            out->data.pvt.ppp_info = sc_sbf_u16(prefix + 86);
        }
        if (revision >= 2U)
        {
            out->data.pvt.latency = sc_sbf_u16(prefix + 88);
            out->data.pvt.h_accuracy = sc_sbf_u16(prefix + 90);
            out->data.pvt.v_accuracy = sc_sbf_u16(prefix + 92);
        }
        double unused;
        if ((out->valid & SC_SBF_VALID_EPOCH) != 0U && prefix[15] == 0U && known_fix(prefix[14]))
        {
            out->valid |= SC_SBF_VALID_FIX;
            if (known_time_system(prefix[72]))
            {
                out->valid |= SC_SBF_VALID_TIME_SYSTEM;
                if (sc_sbf_bias_seconds(out->data.pvt.bias_bits, &unused))
                {
                    out->valid |= SC_SBF_VALID_BIAS;
                }
                if (sc_sbf_drift_fraction(out->data.pvt.drift_bits, &unused))
                {
                    out->valid |= SC_SBF_VALID_DRIFT;
                }
            }
            if (revision >= 1U)
            {
                out->valid |= SC_SBF_VALID_PPP_INFO;
            }
            if (revision >= 2U && out->data.pvt.latency != UINT16_MAX)
            {
                out->valid |= SC_SBF_VALID_LATENCY;
            }
            if (out->data.pvt.mean_corr_age != UINT16_MAX)
            {
                out->valid |= SC_SBF_VALID_CORR_AGE;
            }
        }
        break;
    }
    case SC_SBF_STATUS:
        out->data.status.cpu_load = prefix[14];
        out->data.status.ext_error = prefix[15];
        out->data.status.uptime_s = sc_sbf_u32(prefix + 16);
        out->data.status.rx_state = sc_sbf_u32(prefix + 20);
        out->data.status.rx_error = sc_sbf_u32(prefix + 24);
        out->data.status.command_count = prefix[30];
        out->data.status.temperature = prefix[31];
        break;
    case SC_SBF_MEAS:
        out->data.meas.n1 = prefix[14];
        out->data.meas.sb1_length = prefix[15];
        out->data.meas.sb2_length = prefix[16];
        out->data.meas.common_flags = prefix[17];
        if (revision >= 1U)
        {
            out->data.meas.cum_jumps = prefix[18];
            out->valid |= SC_SBF_VALID_JUMPS;
        }
        break;
    case SC_SBF_TIME:
        out->data.time.year = decode_i8(prefix[14]);
        out->data.time.month = decode_i8(prefix[15]);
        out->data.time.day = decode_i8(prefix[16]);
        out->data.time.hour = decode_i8(prefix[17]);
        out->data.time.minute = decode_i8(prefix[18]);
        out->data.time.second = decode_i8(prefix[19]);
        out->data.time.delta_ls = decode_i8(prefix[20]);
        out->data.time.sync_level = prefix[21];
        if (prefix[14] <= 99U && prefix[15] >= 1U && prefix[15] <= 12U && prefix[16] >= 1U &&
            prefix[16] <= 31U && prefix[17] <= 23U && prefix[18] <= 59U && prefix[19] <= 59U &&
            prefix[20] != 128U)
        {
            out->valid |= SC_SBF_VALID_UTC;
        }
        break;
    case SC_SBF_GAL_CNAV:
        out->data.cnav.svid = prefix[14];
        out->data.cnav.crc_passed = prefix[15];
        out->data.cnav.viterbi_count = prefix[16];
        out->data.cnav.source = prefix[17];
        out->data.cnav.channel = prefix[19];
        /* SBF CRC and navigation-page CRC are distinct checks. These pages
         * establish a received E6 data link, not applied HAS corrections. */
        if (prefix[15] == 1U && (prefix[17] & 31U) == 19U && prefix[14] >= 71U &&
            prefix[14] <= 106U)
        {
            out->valid |= SC_SBF_VALID_E6_PAGE;
        }
        break;
    case SC_SBF_RF_STATUS:
        out->data.rf_status.n = prefix[14];
        out->data.rf_status.sb_length = prefix[15];
        out->data.rf_status.flags = prefix[16];
        /* G5 1.1 RFStatus/RFBand: eight defined bytes per band, with SBLength
         * allowing future extensions. The framing layer checks the entire
         * wire CRC. Only header Flags are consumed here, so unused band bytes
         * need not fit the retained prefix. Validate their declared wire span
         * and the standard 0..3 bytes of final padding before accepting Flags. */
        if ((out->valid & SC_SBF_VALID_EPOCH) != 0U &&
            (out->data.rf_status.n == 0U || out->data.rf_status.sb_length >= 8U))
        {
            const unsigned body =
                20U + (unsigned)out->data.rf_status.n * (unsigned)out->data.rf_status.sb_length;
            if (body <= wire_length && wire_length - body < 4U)
            {
                out->valid |= SC_SBF_VALID_RF_STATUS;
            }
        }
        break;
    case SC_SBF_POS_COV_CARTESIAN:
        out->data.cov.mode = prefix[14];
        out->data.cov.error = prefix[15];
        out->data.cov.cov_bb_bits = sc_sbf_u32(prefix + 28);
        if ((out->valid & SC_SBF_VALID_EPOCH) != 0U && out->data.cov.error == 0U &&
            known_fix(out->data.cov.mode))
        {
            double unused;
            if (sc_sbf_covariance_value(out->data.cov.cov_bb_bits, &unused))
            {
                out->valid |= SC_SBF_VALID_COV_BB;
            }
        }
        break;
    case SC_SBF_VEL_COV_CARTESIAN:
        out->data.cov.mode = prefix[14];
        out->data.cov.error = prefix[15];
        out->data.cov.cov_dtdt_bits = sc_sbf_u32(prefix + 28);
        if ((out->valid & SC_SBF_VALID_EPOCH) != 0U && out->data.cov.error == 0U &&
            known_fix(out->data.cov.mode))
        {
            double unused;
            if (sc_sbf_covariance_value(out->data.cov.cov_dtdt_bits, &unused))
            {
                out->valid |= SC_SBF_VALID_COV_DTDT;
            }
        }
        break;
    case SC_SBF_DOP:
        out->data.dop.nr_sv = prefix[14];
        out->data.dop.pdop = sc_sbf_u16(prefix + 16);
        out->data.dop.tdop = sc_sbf_u16(prefix + 18);
        out->data.dop.hdop = sc_sbf_u16(prefix + 20);
        out->data.dop.vdop = sc_sbf_u16(prefix + 22);
        if ((out->valid & SC_SBF_VALID_EPOCH) != 0U && out->data.dop.nr_sv != 0U &&
            out->data.dop.tdop != 0U)
        {
            out->valid |= SC_SBF_VALID_TDOP;
        }
        break;
    default:
        break;
    }
    return SC_SBF_DECODE_OK;
}
