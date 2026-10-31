/* SPDX-License-Identifier: MIT */
#include "simpleclock/sbf.h"
#include "simpleclock/sbf_fields.h"
#include "simpleclock/gnss_command.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(e)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(e))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #e);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

static void u16(uint8_t *p, uint16_t n)
{
    p[0] = (uint8_t)n;
    p[1] = (uint8_t)(n >> 8);
}

static void u32(uint8_t *p, uint32_t n)
{
    u16(p, (uint16_t)n);
    u16(p + 2, (uint16_t)(n >> 16));
}

static void seal(uint8_t *p, uint16_t length)
{
    u16(p + 2, simpleclock_crc16_ccitt(p + 4, (size_t)length - 4U));
}

static void header(uint8_t *p, uint16_t id, unsigned rev, uint16_t length)
{
    memset(p, 0, length);
    p[0] = '$';
    p[1] = '@';
    u16(p + 4, (uint16_t)(id | (rev << 13)));
    u16(p + 6, length);
    u32(p + 8, 123456000);
    u16(p + 12, 2436);
}

static unsigned frames;
static unsigned rejected_frames;
static sc_sbf_observation_t observation;

static void frame(void *ctx, const uint8_t *p, uint16_t length, bool structure_valid)
{
    (void)ctx;
    if (!structure_valid)
    {
        rejected_frames++;
        return;
    }
    if (sc_sbf_decode(p, length < SC_SBF_PREFIX ? length : SC_SBF_PREFIX, length, &observation) ==
        SC_SBF_DECODE_OK)
    {
        frames++;
    }
}

static void command_reply(sc_gnss_command_t *s, const char *p)
{
    while (*p)
    {
        sc_gnss_command_byte(s, (uint8_t)*p++);
    }
}

int main(void)
{
    uint8_t p[4120];
    sc_sbf_observation_t o;
    header(p, 4024, 0, 84);
    p[14] = 71;
    p[15] = 1;
    p[17] = 19;
    p[19] = 5;
    CHECK(sc_sbf_decode(p, 84, 84, &o) == SC_SBF_DECODE_OK && o.valid == 1025 &&
          o.data.cnav.channel == 5);
    p[15] = 0;
    CHECK(sc_sbf_decode(p, 84, 84, &o) == SC_SBF_DECODE_OK && o.valid == 1);
    p[15] = 1;
    p[17] = 17;
    CHECK(sc_sbf_decode(p, 84, 84, &o) == SC_SBF_DECODE_OK && o.valid == 1);
    p[17] = 19;
    p[14] = 107;
    CHECK(sc_sbf_decode(p, 84, 84, &o) == SC_SBF_DECODE_OK && o.valid == 1);
    /* RFStatus 4092: N/SBLength/Flags follow the common time header at
     * offsets 14/15/16; the three reserved bytes complete the 20-byte body. */
    header(p, 4092, 0, 20);
    p[14] = 0;
    p[15] = 0;
    p[16] = 0;
    CHECK(sc_sbf_decode(p, 20, 20, &o) == SC_SBF_DECODE_OK && o.data.rf_status.flags == 0 &&
          (o.valid & SC_SBF_VALID_RF_STATUS) != 0);
    p[16] = 1; /* RF authenticity warning: retain raw flags, quality rejects it. */
    CHECK(sc_sbf_decode(p, 20, 20, &o) == SC_SBF_DECODE_OK && o.data.rf_status.flags == 1 &&
          (o.valid & SC_SBF_VALID_RF_STATUS) != 0);
    header(p, 4092, 0, 28);
    p[14] = 1;
    p[15] = 8;
    p[16] = 0;
    CHECK(sc_sbf_decode(p, 28, 28, &o) == SC_SBF_DECODE_OK &&
          (o.valid & SC_SBF_VALID_RF_STATUS) != 0);
    CHECK(sc_sbf_decode(p, 20, 28, &o) == SC_SBF_DECODE_OK &&
          (o.valid & SC_SBF_VALID_RF_STATUS) != 0);
    p[15] = 0; /* N>0 with no sub-block bytes is malformed. */
    CHECK(sc_sbf_decode(p, 28, 28, &o) == SC_SBF_DECODE_OK &&
          (o.valid & SC_SBF_VALID_RF_STATUS) == 0);
    p[15] = 4; /* A band shorter than the documented fields is also malformed. */
    CHECK(sc_sbf_decode(p, 28, 28, &o) == SC_SBF_DECODE_OK &&
          (o.valid & SC_SBF_VALID_RF_STATUS) == 0);
    header(p, 4092, 0, 100);
    p[14] = 10;
    p[15] = 8;
    CHECK(sc_sbf_decode(p, 20, 100, &o) == SC_SBF_DECODE_OK &&
          (o.valid & SC_SBF_VALID_RF_STATUS) != 0);
    p[14] = 11;
    CHECK(sc_sbf_decode(p, 20, 100, &o) == SC_SBF_DECODE_OK &&
          (o.valid & SC_SBF_VALID_RF_STATUS) == 0);
    for (unsigned n = 0; n < 84; n++)
    {
        CHECK(sc_sbf_decode(p, n, 84, &o) == SC_SBF_DECODE_LENGTH);
    }
    header(p, 4024, 1, 84);
    CHECK(sc_sbf_decode(p, 84, 84, &o) == SC_SBF_DECODE_REVISION);
    header(p, 4007, 2, 96);
    p[14] = 10;
    p[74] = 12;
    p[75] = 1;
    /* IEEE golden values: bias +0.5 ms, drift -1 ppm, little-endian on wire. */
    p[66] = 0xe0;
    p[67] = 0x3f;
    p[70] = 0x80;
    p[71] = 0xbf;
    u16(p + 78, 11999);
    u16(p + 86, 0xa003);
    u16(p + 88, 2100);
    u16(p + 90, 123);
    u16(p + 92, 456);
    CHECK(sc_sbf_decode(p, 96, 96, &o) == SC_SBF_DECODE_OK);
    CHECK(o.data.pvt.bias_bits == UINT64_C(0x3fe0000000000000));
    CHECK(o.data.pvt.drift_bits == UINT32_C(0xbf800000));
    CHECK(o.data.pvt.ppp_info == 0xa003 && o.data.pvt.latency == 2100 &&
          o.data.pvt.h_accuracy == 123);
    CHECK(o.valid == 927U); /* epoch/fix/bias/drift/system/PPP-info/latency/correction age */
    double d;
    CHECK(sc_sbf_bias_seconds(o.data.pvt.bias_bits, &d) && fabs(d - 0.0005) < 1e-18);
    CHECK(sc_sbf_drift_fraction(o.data.pvt.drift_bits, &d) && fabs(d + 1e-6) < 1e-20);
    CHECK(!sc_sbf_bias_seconds(UINT64_C(0x7ff0000000000000), &d));
    CHECK(!sc_sbf_bias_seconds(UINT64_C(0x7ff8000000000001), &d));
    CHECK(!sc_sbf_drift_fraction(UINT32_C(0x7f800000), &d));
    CHECK(!sc_sbf_drift_fraction(UINT32_C(0xffc00001), &d));
    CHECK(!sc_sbf_bias_seconds(UINT64_C(0xc212a05f20000000), &d));
    CHECK(!sc_sbf_drift_fraction(UINT32_C(0xd09502f9), &d));
    /* PosCovCartesian/VelCovCartesian rev0: 8-byte header, TOW/WNc, mode/error,
     * then ten f4 covariance terms. Cov_bb and Cov_DtDt are both term 4 at
     * wire offset 28; retain their exact bits and promote only valid values. */
    header(p, 5905, 0, 56);
    p[14] = 10;
    u32(p + 28, UINT32_C(0x40800000)); /* Cov_bb = 4 m2 */
    CHECK(sc_sbf_decode(p, 56, 56, &o) == SC_SBF_DECODE_OK &&
          o.data.cov.cov_bb_bits == UINT32_C(0x40800000) && o.data.cov.mode == 10 &&
          o.data.cov.error == 0 && o.valid == (SC_SBF_VALID_EPOCH | SC_SBF_VALID_COV_BB));
    CHECK(sc_sbf_covariance_value(o.data.cov.cov_bb_bits, &d) && d == 4.0);
    CHECK(sc_sbf_formal_sigma(o.data.cov.cov_bb_bits, &d) && fabs(d - 2.0 / 299792458.0) < 1e-20);
    const uint32_t bad_covariances[] = {UINT32_C(0xbf800000), UINT32_C(0x7f800000),
                                        UINT32_C(0x7fc00001), UINT32_C(0xd09502f9)};
    for (unsigned i = 0; i < sizeof(bad_covariances) / sizeof(bad_covariances[0]); i++)
    {
        const uint32_t bad = bad_covariances[i];
        u32(p + 28, bad);
        CHECK(sc_sbf_decode(p, 56, 56, &o) == SC_SBF_DECODE_OK && o.data.cov.cov_bb_bits == bad &&
              o.valid == SC_SBF_VALID_EPOCH);
        CHECK(!sc_sbf_covariance_value(bad, &d) && !sc_sbf_formal_sigma(bad, &d));
    }
    p[15] = 1;
    u32(p + 28, UINT32_C(0x40800000));
    CHECK(sc_sbf_decode(p, 56, 56, &o) == SC_SBF_DECODE_OK && o.valid == SC_SBF_VALID_EPOCH);
    p[15] = 0;
    p[14] = 9;
    CHECK(sc_sbf_decode(p, 56, 56, &o) == SC_SBF_DECODE_OK && o.valid == SC_SBF_VALID_EPOCH);
    header(p, 5905, 1, 56);
    CHECK(sc_sbf_decode(p, 56, 56, &o) == SC_SBF_DECODE_REVISION);
    header(p, 5905, 0, 52);
    CHECK(sc_sbf_decode(p, 52, 52, &o) == SC_SBF_DECODE_LENGTH);
    header(p, 5907, 0, 56);
    p[14] = 1;
    u32(p + 28, UINT32_C(0x41100000)); /* Cov_DtDt = 9 m2/s2 */
    CHECK(sc_sbf_decode(p, 56, 56, &o) == SC_SBF_DECODE_OK &&
          o.data.cov.cov_dtdt_bits == UINT32_C(0x41100000) &&
          o.valid == (SC_SBF_VALID_EPOCH | SC_SBF_VALID_COV_DTDT));
    CHECK(sc_sbf_formal_sigma(o.data.cov.cov_dtdt_bits, &d) && fabs(d - 3.0 / 299792458.0) < 1e-20);
    u16(p + 12, 65535);
    CHECK(sc_sbf_decode(p, 56, 56, &o) == SC_SBF_DECODE_OK && o.valid == 0);
    /* DOP rev0, guide p.279: raw u2 hundredths at16..22; raw zero is DNU.
     * NrSV=0 is independently unavailable, even for inconsistent nonzero DOP. */
    header(p, 4001, 0, 32);
    p[14] = 12;
    u16(p + 16, 234);
    u16(p + 18, 123);
    u16(p + 20, 145);
    u16(p + 22, 187);
    CHECK(sc_sbf_decode(p, 32, 32, &o) == SC_SBF_DECODE_OK &&
          o.valid == (SC_SBF_VALID_EPOCH | SC_SBF_VALID_TDOP) && o.data.dop.nr_sv == 12 &&
          o.data.dop.pdop == 234 && o.data.dop.tdop == 123 && o.data.dop.hdop == 145 &&
          o.data.dop.vdop == 187);
    CHECK(sc_sbf_dop_value(o.data.dop.tdop, &d) && fabs(d - 1.23) < 1e-12);
    CHECK(sc_sbf_dop_value(1, &d) && d == 0.01);
    CHECK(sc_sbf_dop_value(UINT16_MAX, &d) && d == 655.35);
    d = 12.0;
    CHECK(!sc_sbf_dop_value(0, &d) && d == 12.0 && !sc_sbf_dop_value(1, NULL));
    u16(p + 18, 0);
    CHECK(sc_sbf_decode(p, 32, 32, &o) == SC_SBF_DECODE_OK && o.valid == SC_SBF_VALID_EPOCH &&
          o.data.dop.tdop == 0);
    u16(p + 18, 123);
    p[14] = 0;
    CHECK(sc_sbf_decode(p, 32, 32, &o) == SC_SBF_DECODE_OK && o.valid == SC_SBF_VALID_EPOCH &&
          o.data.dop.tdop == 123);
    p[14] = 12;
    u16(p + 12, UINT16_MAX);
    CHECK(sc_sbf_decode(p, 32, 32, &o) == SC_SBF_DECODE_OK && o.valid == 0);
    header(p, 4001, 0, 32);
    u32(p + 8, SC_SBF_WEEK_MS);
    CHECK(sc_sbf_decode(p, 32, 32, &o) == SC_SBF_DECODE_OK && o.valid == 0);
    header(p, 4001, 1, 32);
    CHECK(sc_sbf_decode(p, 32, 32, &o) == SC_SBF_DECODE_REVISION);
    header(p, 4001, 0, 28);
    CHECK(sc_sbf_decode(p, 28, 28, &o) == SC_SBF_DECODE_LENGTH);
    header(p, 4001, 0, 32);
    CHECK(sc_sbf_decode(p, 31, 32, &o) == SC_SBF_DECODE_LENGTH);
    /* Preserve unsupported or bad observations as raw, never a fresh fix. */
    header(p, 4007, 2, 96);
    p[15] = 1;
    CHECK(sc_sbf_decode(p, 96, 96, &o) == SC_SBF_DECODE_OK && o.valid == SC_SBF_VALID_EPOCH);
    p[15] = 0;
    p[14] = 9;
    CHECK(sc_sbf_decode(p, 96, 96, &o) == SC_SBF_DECODE_OK && o.valid == SC_SBF_VALID_EPOCH);
    p[14] = 1;
    p[72] = 255;
    CHECK(sc_sbf_decode(p, 96, 96, &o) == SC_SBF_DECODE_OK && (o.valid & 28U) == 0U);
    p[72] = 0;
    u32(p + 8, 604800000);
    CHECK(sc_sbf_decode(p, 96, 96, &o) == SC_SBF_DECODE_OK && o.valid == 0U);
    u32(p + 8, 123456000);
    u16(p + 12, 65535);
    CHECK(sc_sbf_decode(p, 96, 96, &o) == SC_SBF_DECODE_OK && o.valid == 0U);
    /* Older revision padding contains junk, not modern fields. */
    header(p, 4007, 0, 88);
    memset(p + 85, 0xff, 3);
    CHECK(sc_sbf_decode(p, 88, 88, &o) == SC_SBF_DECODE_OK && o.data.pvt.ppp_info == 0);
    CHECK((o.valid & (SC_SBF_VALID_PPP_INFO | SC_SBF_VALID_LATENCY)) == 0);
    header(p, 4007, 1, 88);
    u16(p + 86, 0x2002);
    CHECK(sc_sbf_decode(p, 88, 88, &o) == SC_SBF_DECODE_OK && o.data.pvt.ppp_info == 0x2002);
    header(p, 4007, 2, 92);
    CHECK(sc_sbf_decode(p, 92, 92, &o) == SC_SBF_DECODE_LENGTH && o.id == 0);
    header(p, 4007, 4, 96);
    CHECK(sc_sbf_decode(p, 96, 96, &o) == SC_SBF_DECODE_REVISION && o.valid == 0);
    header(p, 4038, 2, 96);
    CHECK(sc_sbf_decode(p, 96, 96, &o) == SC_SBF_DECODE_UNKNOWN); /* ExtEventPVT */
    header(p, 4007, 2, 96);
    for (unsigned n = 0; n < 96; n++)
    {
        CHECK(sc_sbf_decode(p, n, 96, &o) == SC_SBF_DECODE_LENGTH);
    }
    header(p, 5914, 0, 24);
    p[14] = 26;
    p[15] = 9;
    p[16] = 14;
    p[17] = 22;
    p[18] = 10;
    p[19] = 30;
    p[20] = 18;
    p[21] = 7;
    CHECK(sc_sbf_decode(p, 24, 24, &o) == SC_SBF_DECODE_OK && (o.valid & SC_SBF_VALID_UTC) != 0);
    p[20] = 128;
    CHECK(sc_sbf_decode(p, 24, 24, &o) == SC_SBF_DECODE_OK && o.data.time.delta_ls == -128 &&
          (o.valid & SC_SBF_VALID_UTC) == 0);
    header(p, 4014, 1, 36);
    u32(p + 16, 400);
    u32(p + 20, 4);
    u32(p + 24, 64);
    p[28] = 1;
    p[29] = 4;
    p[30] = 12;
    CHECK(sc_sbf_decode(p, 36, 36, &o) == SC_SBF_DECODE_OK && o.data.status.rx_state == 4 &&
          o.data.status.rx_error == 64);
    p[28] = 2;
    CHECK(sc_sbf_decode(p, 36, 36, &o) == SC_SBF_DECODE_LENGTH);
    header(p, 5921, 0, 16);
    CHECK(sc_sbf_decode(p, 16, 16, &o) == SC_SBF_DECODE_OK && o.valid == SC_SBF_VALID_EPOCH);
    /* 205 complete Type1 blocks exceed the UART ring; N2 is beyond prefix. */
    header(p, 4027, 1, 4120);
    p[14] = 205;
    p[15] = 20;
    p[16] = 12;
    p[17] = 8;
    p[18] = 255;
    seal(p, 4120);
    for (unsigned split = 0; split <= 4120; split++)
    {
        sc_sbf_t s = {0};
        frames = 0;
        sc_sbf_feed(&s, p, split, 0, frame, NULL, NULL);
        sc_sbf_feed(&s, p + split, 4120 - split, 1, frame, NULL, NULL);
        CHECK(frames == 1 && s.structure_errors == 0 && observation.data.meas.cum_jumps == 255);
    }
    sc_sbf_t s = {0};
    frames = 0;
    p[4039] = 1;
    seal(p, 4120); /* forged N2 in a later sub-block */
    sc_sbf_feed(&s, p, 4120, 0, frame, NULL, NULL);
    CHECK(frames == 0 && s.structure_errors == 1 && rejected_frames == 1 && s.frames == 1);
    p[4039] = 0;
    seal(p, 4120);
    p[3000] ^= 1;
    sc_sbf_feed(&s, p, 4120, 1, frame, NULL, NULL);
    CHECK(frames == 0 && s.crc_errors == 1);
    header(p, 4027, 0, 20);
    p[18] = 255;
    seal(p, 20);
    sc_sbf_feed(&s, p, 20, 2, frame, NULL, NULL);
    CHECK(frames == 1 && (observation.valid & SC_SBF_VALID_JUMPS) == 0);
    /* Receiver rev1 produced 3 Type1 records, 4 nested Type2 records and 16
     * trailing bytes. R3 4.1.5 puts no three-byte cap on padding. */
    header(p, 4027, 1, 144);
    p[14] = 3;
    p[15] = 20;
    p[16] = 12;
    p[17] = 0x85;
    p[18] = 255;
    p[59] = 3;
    p[115] = 1;
    memset(p + 128, 0xa7, 16);
    seal(p, 144);
    sc_sbf_feed(&s, p, 144, 3, frame, NULL, NULL);
    CHECK(frames == 2 && observation.data.meas.cum_jumps == 255 && observation.valid == 65);
    p[115] = 255;
    seal(p, 144);
    sc_sbf_feed(&s, p, 144, 4, frame, NULL, NULL);
    CHECK(frames == 2 && s.structure_errors == 2 && rejected_frames == 2);
    header(p, 4014, 1, 48);
    memset(p + 32, 0xa7, 16);
    CHECK(sc_sbf_decode(p, 48, 48, &o) == SC_SBF_DECODE_OK); /* reserved padding */
    /* Command prompt/echo, interleaved payload, error and wrap-safe timeout. */
    sc_gnss_command_t cmd = {0};
    CHECK(sc_gnss_command_start(&cmd, "getReceiverTime\r\n", UINT32_MAX - 1000U));
    CHECK(!sc_gnss_command_start(&cmd, "other\r\n", 0));
    command_reply(&cmd, "COM1>$R: wrong\r\nCOM1>");
    CHECK(cmd.state == SC_CMD_WAITING);
    command_reply(&cmd, "$R: getReceiverTime\r\n---->\r\n");
    CHECK(cmd.state == SC_CMD_WAITING);
    command_reply(&cmd, "COM1>");
    CHECK(cmd.state == SC_CMD_OK);
    CHECK(sc_gnss_command_start(&cmd, "getReceiverTime\r\n", 0));
    command_reply(&cmd, "$R? Invalid command\r\nCOM1>");
    CHECK(cmd.state == SC_CMD_REJECTED);
    CHECK(sc_gnss_command_start(&cmd, "getReceiverTime\r\n", UINT32_MAX - 1000U));
    sc_gnss_command_expire(&cmd, 1999);
    CHECK(cmd.state == SC_CMD_WAITING);
    sc_gnss_command_expire(&cmd, 2000);
    CHECK(cmd.state == SC_CMD_TIMEOUT);
    return 0;
}
