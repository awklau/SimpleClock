/* SPDX-License-Identifier: MIT */
/* Exercise readback gates with vendor-shaped replies, including long lists. */
#include "simpleclock/gnss_command.h"
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

static void feed(sc_gnss_command_t *s, const char *text)
{
    while (*text)
    {
        sc_gnss_command_byte(s, (uint8_t)*text++);
    }
}

static sc_cmd_state_t pvt_reply(sc_cmd_expect_t expected, const char *reply)
{
    sc_gnss_command_t s = {0};
    (void)sc_gnss_command_start_checked(&s, "getPVTMode\r\n", expected, 0);
    feed(&s, "COM1>$R: getPVTMode\r\n");
    feed(&s, reply);
    feed(&s, "COM1>");
    return s.state;
}

int main(void)
{
    const char *angles[] = {"-90", "-1", "0", "15.000", "90"};
    const int values[] = {-90, -1, 0, 15, 90};
    for (unsigned engine = 0; engine < 2U; ++engine)
    {
        for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); ++i)
        {
            char reply[128];
            (void)snprintf(reply, sizeof(reply), "ElevationMask, %s, %s\r\n",
                           engine ? "Tracking" : "PVT", angles[i]);
            const sc_cmd_expect_t any =
                engine ? SC_CMD_EXPECT_ELEVATION_TRACKING : SC_CMD_EXPECT_ELEVATION_PVT;
            const unsigned base =
                engine ? SC_CMD_EXPECT_TRACKING_MASK_BASE : SC_CMD_EXPECT_PVT_MASK_BASE;
            CHECK(pvt_reply(any, reply) == SC_CMD_OK);
            CHECK(pvt_reply((sc_cmd_expect_t)((int)base + values[i] + 90), reply) == SC_CMD_OK);
            CHECK(pvt_reply((sc_cmd_expect_t)((int)base + (values[i] == 90 ? 179 : 180)), reply) ==
                  SC_CMD_REJECTED);
            CHECK(pvt_reply(engine ? SC_CMD_EXPECT_ELEVATION_PVT : SC_CMD_EXPECT_ELEVATION_TRACKING,
                            reply) == SC_CMD_REJECTED);
        }
    }
    const char *bad_masks[] = {"",
                               "ElevationMask, PVT\r\n",
                               "ElevationMask, PVT, \r\n",
                               "ElevationMask, PVT, -91\r\n",
                               "ElevationMask, PVT, 100\r\n",
                               "ElevationMask, PVT, 15.1\r\n",
                               "ElevationMask, PVT, 15.\r\n",
                               "ElevationMask, PVT, 15deg\r\n",
                               "ElevationMask, PVT, 15, extra\r\n",
                               "ElevationMask, PVT, 1 5\r\n",
                               "ElevationMask, PVT, 15+15\r\n",
                               "ElevationMask, PVT, 15\r\nElevationMask, PVT, 15\r\n"};
    for (unsigned i = 0; i < sizeof(bad_masks) / sizeof(bad_masks[0]); ++i)
    {
        CHECK(pvt_reply(SC_CMD_EXPECT_ELEVATION_PVT, bad_masks[i]) == SC_CMD_REJECTED);
    }
    CHECK(pvt_reply(SC_CMD_EXPECT_ELEVATION_PVT,
                    "ElevationMask, Tracking, 5\r\nElevationMask, PVT, 15\r\n") == SC_CMD_OK);
    const char *bad_saves[] = {
        "",
        "CopyConfigFile, Boot, Current\r\n",
        "CopyConfigFile, Current, User1\r\n",
        "CopyConfigFile, Current, Boot, extra\r\n",
        "CopyConfigFile, Current+Current, Boot\r\n",
        "CopyConfigFile, Current, Boot\r\nCopyConfigFile, Current, Boot\r\n"};
    CHECK(pvt_reply(SC_CMD_EXPECT_SAVE_BOOT, "CopyConfigFile, Current, Boot\r\n") == SC_CMD_OK);
    for (unsigned i = 0; i < sizeof(bad_saves) / sizeof(bad_saves[0]); ++i)
    {
        CHECK(pvt_reply(SC_CMD_EXPECT_SAVE_BOOT, bad_saves[i]) == SC_CMD_REJECTED);
    }
    CHECK(pvt_reply(SC_CMD_EXPECT_STANDALONE, "  PVTMode, Rover, StandAlone, auto\r\n") ==
          SC_CMD_OK);
    CHECK(pvt_reply(SC_CMD_EXPECT_STANDALONE_PPP, "PVTMode, Rover, StandAlone+PPP, auto\r\n") ==
          SC_CMD_OK);
    CHECK(pvt_reply(SC_CMD_EXPECT_STANDALONE_PPP, "PVTMode, Rover, PPP+StandAlone, auto\r\n") ==
          SC_CMD_OK);
    CHECK(pvt_reply(SC_CMD_EXPECT_PPP_TEST_MODE, "PVTMode, Rover, StandAlone, auto\r\n") ==
          SC_CMD_OK);
    CHECK(pvt_reply(SC_CMD_EXPECT_PPP_TEST_MODE, "PVTMode, Rover, StandAlone+PPP, auto\r\n") ==
          SC_CMD_OK);
    const char *bad[] = {
        "",
        "PVTMode, Static, StandAlone, auto\r\n",
        "PVTMode, Rover, StandAlone, Geodetic1\r\n",
        "PVTMode, Rover, StandAlone+RTK+PPP, auto\r\n",
        "PVTMode, Rover, StandAlone+PPP, auto, extra\r\n",
        "PVTMode, Rover, StandAlone+PPP+PPP, auto\r\n",
        "PVTMode, Rover, StandAlone, auto\r\n",
        "PVTMode, Rover, PPP, auto\r\n",
        "PVTMode, Rover, StandAlone+PPP, auto\r\nPVTMode, Rover, StandAlone+PPP, auto\r\n"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i)
    {
        CHECK(pvt_reply(SC_CMD_EXPECT_STANDALONE_PPP, bad[i]) == SC_CMD_REJECTED);
    }

    const char *bench_modes[] = {"PVTMode, Rover, StandAlone, auto\r\n",
                                 "PVTMode, Rover, StandAlone+PPP, auto\r\n",
                                 "PVTMode, Rover, SBAS+StandAlone, auto\r\n"};
    for (unsigned i = 0; i < sizeof(bench_modes) / sizeof(bench_modes[0]); ++i)
    {
        CHECK(pvt_reply(SC_CMD_EXPECT_BENCH_PVT_MODE, bench_modes[i]) == SC_CMD_OK);
        CHECK(pvt_reply(SC_CMD_EXPECT_STANDALONE_SBAS, bench_modes[i]) ==
              (i == 2U ? SC_CMD_OK : SC_CMD_REJECTED));
    }
    CHECK(pvt_reply(SC_CMD_EXPECT_PPP_TEST_MODE, bench_modes[2]) == SC_CMD_REJECTED);
    const char *bad_bench[] = {
        "",
        "PVTMode, Static, StandAlone+SBAS, auto\r\n",
        "PVTMode, Rover, StandAlone+SBAS, Geodetic1\r\n",
        "PVTMode, Rover, StandAlone+SBAS+RTK, auto\r\n",
        "PVTMode, Rover, StandAlone+SBAS+SBAS, auto\r\n",
        "PVTMode, Rover, SBAS, auto\r\n",
        "PVTMode, Rover, StandAlone+SBAS, auto, extra\r\n",
        "PVTMode, Rover, StandAlone+SBAS, auto\r\nPVTMode, Rover, StandAlone+SBAS, auto\r\n"};
    for (unsigned i = 0; i < sizeof(bad_bench) / sizeof(bad_bench[0]); ++i)
    {
        CHECK(pvt_reply(SC_CMD_EXPECT_BENCH_PVT_MODE, bad_bench[i]) == SC_CMD_REJECTED);
        CHECK(pvt_reply(SC_CMD_EXPECT_STANDALONE_SBAS, bad_bench[i]) == SC_CMD_REJECTED);
    }

    CHECK(pvt_reply(SC_CMD_EXPECT_BENCH_PVT_MODE,
                    "PVTMode, Rover, StandAlone+SBAS+PPP, auto\r\n") == SC_CMD_OK);
    CHECK(pvt_reply(SC_CMD_EXPECT_MULTIMODE, "PVTMode, Rover, PPP+StandAlone+SBAS, auto\r\n") ==
          SC_CMD_OK);
    CHECK(pvt_reply(SC_CMD_EXPECT_MULTIMODE, "PVTMode, Rover, StandAlone+PPP, auto\r\n") ==
          SC_CMD_REJECTED);
    CHECK(pvt_reply(SC_CMD_EXPECT_STANDALONE, "PVTMode, Rover, StandAlone+SBAS+PPP, auto\r\n") ==
          SC_CMD_REJECTED);
    sc_gnss_command_t s = {0};
    const char *sbas_capabilities[] = {
        "ReceiverCapabilities, Main, GPSL1CA, COM1, SBAS+RAIM, 200, 200\r\n",
        "ReceiverCapabilities, Main, SBAS, COM1, RAIM, 200, 200\r\n",
        "ReceiverCapabilities, Main, GPSL1CA, COM1, NOTSBAS+RAIM, 200, 200\r\n"};
    for (unsigned i = 0; i < sizeof(sbas_capabilities) / sizeof(sbas_capabilities[0]); ++i)
    {
        CHECK(sc_gnss_command_start_checked(&s, "getReceiverCapabilities\r\n",
                                            SC_CMD_EXPECT_SBAS_CAPABILITY, 0));
        feed(&s, "$R: getReceiverCapabilities\r\n");
        feed(&s, sbas_capabilities[i]);
        feed(&s, "COM1>");
        CHECK(s.state == (i == 0U ? SC_CMD_OK : SC_CMD_REJECTED));
    }
    CHECK(sc_gnss_command_start_checked(&s, "getReceiverCapabilities\r\n",
                                        SC_CMD_EXPECT_SBAS_CAPABILITY, 0));
    feed(&s, sbas_capabilities[0]);
    feed(&s, "$R: getReceiverCapabilities\r\nCOM1>");
    CHECK(s.state == SC_CMD_REJECTED);
    CHECK(sc_gnss_command_start_checked(&s, "getReceiverCapabilities\r\n",
                                        SC_CMD_EXPECT_HAS_CAPABILITY, 0));
    /* A matching capability before the command's echo cannot authorize it. */
    feed(&s, "ReceiverCapabilities, Main, GPSL1CA, COM1, PPPGalileoHAS-SIS\r\n");
    feed(&s, "$R: getReceiverCapabilities\r\nCOM1>");
    CHECK(s.state == SC_CMD_REJECTED);
    CHECK(sc_gnss_command_start_checked(&s, "getReceiverCapabilities\r\n",
                                        SC_CMD_EXPECT_HAS_CAPABILITY, 0));
    feed(&s, "$R: getReceiverCapabilities\r\nReceiverCapabilities, Main, ");
    for (unsigned i = 0; i < 80; ++i)
    {
        feed(&s, "GPSL1CA+");
    }
    feed(&s, "GALE6BC, COM1+COM2, PPPGalileoHAS-SIS+RAIM, 200, 200\r\nCOM1>");
    CHECK(s.state == SC_CMD_OK); /* Capability list is much longer than 192 bytes. */
    CHECK(sc_gnss_command_start_checked(&s, "getSignalUsage\r\n", SC_CMD_EXPECT_E6_NAVIGATION, 0));
    feed(&s, "$R: getSignalUsage\r\nSignalUsage, GALE6BC+GPSL1CA, GPSL1CA\r\nCOM1>");
    CHECK(s.state == SC_CMD_REJECTED); /* Measurement use is not navigation use. */
    CHECK(sc_gnss_command_start_checked(&s, "getSignalUsage\r\n", SC_CMD_EXPECT_E6_NAVIGATION, 0));
    feed(&s, "$R: getSignalUsage\r\nSignalUsage, GPSL1CA, GPSL1CA+GALE6BC\r\nCOM1>");
    CHECK(s.state == SC_CMD_OK);
    CHECK(sc_gnss_command_start_checked(&s, "getSignalTracking\r\n", SC_CMD_EXPECT_E6_TRACKING, 0));
    feed(&s, "$R: getSignalTracking\r\nSignalTracking, GPSL1CA+NOTGALE6BC\r\nCOM1>");
    CHECK(s.state == SC_CMD_REJECTED);
    CHECK(sc_gnss_command_start_checked(&s, "getSignalTracking\r\n", SC_CMD_EXPECT_E6_TRACKING,
                                        UINT32_MAX - 1000U));
    feed(&s, "$R: getSignalTracking\r\nSignalTracking, GALE6BC\r\n");
    sc_gnss_command_expire(&s, 2000);
    CHECK(s.state == SC_CMD_TIMEOUT); /* Readback alone, without prompt, is not success. */
    CHECK(sc_gnss_command_start(&s, "getPVTMode\r\n", 0U));
    feed(&s, "$R: getPVTMode\r\n$R? ");
    for (unsigned i = 0; i < 300U; ++i)
    {
        feed(&s, "x");
    }
    feed(&s, "\r\nCOM1>");
    CHECK(s.state == SC_CMD_REJECTED); /* Error prefix survives a long error payload. */
    for (unsigned checked = 0U; checked < 2U; checked++)
    {
        CHECK(sc_gnss_command_start_checked(
            &s, "lstCurrentUser\r\n", checked ? SC_CMD_EXPECT_E6_TRACKING : SC_CMD_EXPECT_NONE,
            0U));
        feed(&s, "$R! lstCurrentUser\r\nNot logged in\r\nCOM1>");
        CHECK(s.state == SC_CMD_REJECTED); /* Authentication error needs no positive echo. */
    }
    CHECK(sc_gnss_command_start(&s, "lstCurrentUser\r\n", 0U));
    feed(&s, "$R! ");
    for (unsigned i = 0U; i < 300U; i++)
    {
        feed(&s, "x");
    }
    feed(&s, "\r\nCOM1>");
    CHECK(s.state == SC_CMD_REJECTED);
    return 0;
}
