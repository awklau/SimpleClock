/* SPDX-License-Identifier: MIT */
#include "simpleclock/gnss_control.h"
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
static char command[184], query[64];

static unsigned prepare(const char *s)
{
    return sc_gnss_control_prepare(s, command, sizeof(command), query, sizeof(query));
}

int main(void)
{
    uint32_t selection;
    bool fixed;
    CHECK(sc_gnss_control_pvt_policy("PVTMode, Static, StandAlone, auto", &selection, &fixed) &&
          fixed && selection == 3U);
    CHECK(sc_gnss_control_pvt_policy("PVTMode,Static,SBAS+StandAlone,Geodetic1", &selection,
                                     &fixed) &&
          fixed && selection == 1U);
    CHECK(sc_gnss_control_pvt_policy("PVTMode,Rover,PPP+StandAlone,auto", &selection, &fixed) &&
          !fixed && selection == 2U);
    CHECK(!sc_gnss_control_pvt_policy("PVTMode,Static,StandAlone+PPP,auto", &selection, &fixed));
    CHECK(
        !sc_gnss_control_pvt_policy("PVTMode,Static,StandAlone+SBAS+PPP,auto", &selection, &fixed));
    CHECK(!sc_gnss_control_pvt_policy("PVTMode,Static,StandAlone+StandAlone,auto", &selection,
                                      &fixed));
    CHECK(!sc_gnss_control_pvt_policy("PVTMode,Rover,StandAlone+RTK,auto", &selection, &fixed));
    CHECK(!sc_gnss_control_pvt_policy("PVTMode,Static,StandAlone,auto,junk", &selection, &fixed));
    CHECK(!sc_gnss_control_pvt_policy("PVTMode,Static,StandAlone,Unknown", &selection, &fixed));
    CHECK(prepare("gnss write setReceiverDynamics,Low,Static") == 2);
    CHECK(!strcmp(query, "getReceiverDynamics\r\n"));
    bool match;
    uint32_t preserved, other;
    CHECK(sc_gnss_control_readback(command, " ReceiverDynamics,Low,Static", &match, &preserved) &&
          match);
    CHECK(sc_gnss_control_readback(command, "ReceiverDynamics,Moderate,Static", &match, &other) &&
          !match);
    CHECK(prepare("gnss write setReceiverDynamics,,Static") == 2);
    CHECK(sc_gnss_control_readback(command, "ReceiverDynamics,Low,Static", &match, &preserved) &&
          match);
    CHECK(sc_gnss_control_readback(command, "ReceiverDynamics,High,Static", &match, &other) &&
          match);
    CHECK(preserved != other); /* Unrequested Level changed. */
    CHECK(!prepare("gnss write setReceiverDynamics,Foo,Static"));
    CHECK(prepare("gnss write setCN0Mask,GALE6BC,60") == 2);
    CHECK(!strcmp(query, "getCN0Mask,GALE6BC\r\n"));
    CHECK(!prepare("gnss write setCN0Mask,GPSL2PY,10"));
    CHECK(!prepare("gnss write setCN0Mask,Reserved2,10"));
    CHECK(!prepare("gnss write setCN0Mask,GALE6BC,61"));
    CHECK(!prepare("gnss write setCN0Mask,all,10"));
    CHECK(prepare("gnss write setPVTMode,Static,,Geodetic1") == 2);
    CHECK(!prepare("gnss write setPVTMode,Rover,StandAlone+RTK,auto"));
    CHECK(!prepare("gnss write setPVTMode,Rover,all,auto"));
    CHECK(prepare("gnss write setDiffCorrMaxAge,,,100") == 2);
    CHECK(!prepare("gnss write setDiffCorrMaxAge,100,,100"));
    CHECK(prepare("gnss write setDiffCorrUsage,,100") == 2);
    CHECK(!prepare("gnss write setDiffCorrUsage,,100,manual"));
    CHECK(!prepare("gnss write setGalOSNMAUsage,loose,KEY,loose"));
    CHECK(prepare("gnss write setPPSParameters,sec1,Low2High,0,RxClock,0,5") == 2);
    CHECK(!prepare("gnss write setPPSParameters,MHz10"));
    CHECK(!prepare("gnss write setCOMSettings,COM1,baud9600"));
    CHECK(!prepare("gnss write setTimeSyncSource,EventA"));
    CHECK(prepare("gnss write setSignalTracking,all") == 2);
    CHECK(prepare("gnss write setSignalUsage,,+GALE6BC") == 2);
    CHECK(!prepare("gnss write setSBFOutput,Stream1,COM1,RFStatus,sec5"));
    CHECK(prepare("gnss write setSBFOutput,Stream3,COM1,RFStatus+QualityInd,sec5") == 2);
    CHECK(prepare("gnss read lstConfigFile,Boot") == 1);
    CHECK(prepare("gnss read getFutureSupportedSetting") == 1);
    CHECK(!prepare("gnss read lstInternalFile,License"));
    CHECK(!prepare("gnss read getUserAccessLevel"));
    CHECK(!prepare("gnss read getuseraccesslevel"));
    CHECK(!prepare("gnss read getPVTMode;factoryReset"));
    CHECK(!prepare("gnss read getPVTMode\r\nfactoryReset"));
    CHECK(prepare("gnss exec exeResetNavFilter,PVT") == 3);
    CHECK(!prepare("gnss exec exeResetNavFilter,all"));
    CHECK(!prepare("gnss exec exeResetReceiver,Hard,all"));
    CHECK(!prepare("gnss exec exeCopyConfigFile,RxDefault,Current"));
    CHECK(prepare("gnss write setAntennaOffset,Main,,,1.2,\"A B\"") == 2);
    CHECK(sc_gnss_control_readback(command, "AntennaOffset,Main,0,0,1.2000,\"A B\",X,0", &match,
                                   &preserved) &&
          match);
    CHECK(!sc_gnss_control_readback(command, "AntennaOffset,Aux1,0,0,1.2000,\"A B\",X,0", &match,
                                    &preserved));
    CHECK(prepare("gnss write setSignalTracking,GPSL1CA+GPSL2C") == 2);
    CHECK(sc_gnss_control_readback(command, "SignalTracking,GPSL2C+GPSL1CA", &match, &preserved) &&
          match);
    return 0;
}
