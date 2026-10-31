/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_SBF_FIELDS_H
#define SIMPLECLOCK_SBF_FIELDS_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SC_SBF_WEEK_MS 604800000U
#define SC_SBF_RX_STATE_EXT_FREQ 0x04U
#define SC_SBF_TIME_SYNC_READY_MASK 0x07U
#define SC_SBF_MEAS_CLOCK_STEERING 0x08U

/* PVTGeodetic wire values. Mode's high bits carry flags, not a solution type. */
#define SC_SBF_PVT_MODE_MASK 0x0FU
#define SC_SBF_PVT_MODE_STANDALONE 1U
#define SC_SBF_PVT_MODE_FIXED 3U
#define SC_SBF_PVT_STATIC_PENDING 0x40U
#define SC_SBF_PVT_MODE_SBAS 6U
#define SC_SBF_PVT_MODE_PPP 10U
#define SC_SBF_TIME_SYSTEM_GPS 0U
#define SC_SBF_TIME_SYSTEM_GALILEO 1U
#define SC_SBF_PVT_ALERT_RAIM_OK 1U
#define SC_SBF_PVT_WA_ORBIT_CLOCK_USED 0x01U

/* Field layouts follow the supplied mosaic-G5 Firmware v1.1.0 Reference Guide.
 * IDs/revisions and valid bits are also part of the saved diagnostic format;
 * keep tools/sbf_observations.py and wire fixtures in sync with changes. */

enum
{
    SC_SBF_PVT = 4007,
    SC_SBF_STATUS = 4014,
    SC_SBF_MEAS = 4027,
    SC_SBF_TIME = 5914,
    SC_SBF_END = 5921,
    SC_SBF_GAL_CNAV = 4024,
    SC_SBF_RF_STATUS = 4092,
    SC_SBF_POS_COV_CARTESIAN = 5905,
    SC_SBF_VEL_COV_CARTESIAN = 5907,
    SC_SBF_DOP = 4001
};

enum
{
    SC_SBF_VALID_EPOCH = 1U,
    SC_SBF_VALID_FIX = 2U,
    SC_SBF_VALID_BIAS = 4U,
    SC_SBF_VALID_DRIFT = 8U,
    SC_SBF_VALID_TIME_SYSTEM = 16U,
    SC_SBF_VALID_UTC = 32U,
    SC_SBF_VALID_JUMPS = 64U,
    SC_SBF_VALID_PPP_INFO = 128U,
    SC_SBF_VALID_LATENCY = 256U,
    SC_SBF_VALID_CORR_AGE = 512U,
    SC_SBF_VALID_E6_PAGE = 1024U,
    SC_SBF_VALID_COV_BB = 2048U,
    SC_SBF_VALID_COV_DTDT = 4096U,
    SC_SBF_VALID_COVARIANCE = SC_SBF_VALID_COV_BB | SC_SBF_VALID_COV_DTDT,
    SC_SBF_VALID_RF_STATUS = 8192U,
    SC_SBF_VALID_TDOP = 16384U
};

typedef enum
{
    SC_SBF_DECODE_OK,
    SC_SBF_DECODE_UNKNOWN,
    SC_SBF_DECODE_REVISION,
    SC_SBF_DECODE_LENGTH,
    SC_SBF_DECODE_STRUCTURE
} sc_sbf_decode_result_t;

typedef struct
{
    /* Receiver epoch, not USB arrival or hardware PPS capture time. */
    uint32_t tow_ms;
    uint16_t week, id, length, valid;
    uint8_t revision;

    union
    {
        struct
        {
            /* Preserve IEEE bits exactly. Convert only through the helpers
             * below after checking valid; source units are milliseconds/ppm. */
            uint64_t bias_bits;
            uint32_t drift_bits, signal_info;
            uint16_t mean_corr_age, ppp_info, latency, h_accuracy, v_accuracy;
            uint8_t mode, error, time_system, nr_sv, wa_corr_info, alert;
        } pvt;

        struct
        {
            uint32_t uptime_s, rx_state, rx_error;
            uint8_t cpu_load, ext_error, command_count, temperature;
        } status;

        struct
        {
            uint8_t common_flags, cum_jumps, n1, sb1_length, sb2_length;
        } meas;

        struct
        {
            int8_t year, month, day, hour, minute, second, delta_ls;
            uint8_t sync_level;
        } time;

        struct
        {
            uint8_t svid, crc_passed, viterbi_count, source, channel;
        } cnav;

        struct
        {
            uint8_t n, sb_length, flags;
        } rf_status;

        struct
        {
            /* Cov_bb (5905) and Cov_DtDt (5907) share this bounded storage.
             * The active field is selected by observation.id. */
            uint32_t cov_bb_bits, cov_dtdt_bits;
            uint8_t mode, error;
        } cov;

        struct
        {
            /* DOP rev0: raw dimensionless hundredths; zero means unavailable,
             * not a valid zero. NrSV=0 makes all DOP values unavailable. */
            uint16_t pdop, tdop, hdop, vdop;
            uint8_t nr_sv;
        } dop;
    } data;
} sc_sbf_observation_t;

/* Call only after full-frame CRC and variable-body validation by sc_sbf_feed.
 * retained is the actual available prefix length, NOT advertised wire length.
 * Output is cleared on any unsupported/malformed input. No freshness claim. */
sc_sbf_decode_result_t sc_sbf_decode(const uint8_t *prefix, size_t retained, uint16_t wire_length,
                                     sc_sbf_observation_t *out);
/* Raw source units: f8 milliseconds / f4 ppm. False leaves result untouched. */
bool sc_sbf_bias_seconds(uint64_t bits, double *seconds);
bool sc_sbf_drift_fraction(uint32_t bits, double *fraction);
/* Covariance f4 values are retained losslessly. Valid values are finite,
 * nonnegative, and not the guide's Do-Not-Use value (-2e10). */
bool sc_sbf_covariance_value(uint32_t bits, double *value);
/* Formal sigma from a valid covariance: sqrt(value) / c, where c is the
 * speed of light in m/s. For Cov_bb this is seconds; for Cov_DtDt it is a
 * dimensionless fractional frequency sigma. */
bool sc_sbf_formal_sigma(uint32_t bits, double *sigma);
/* Dimensionless DOP hundredths. Native zero is unavailable; false leaves value
 * untouched. Epoch/NrSV/freshness remain separate caller-owned validity gates. */
bool sc_sbf_dop_value(uint16_t hundredths, double *value);
#endif
