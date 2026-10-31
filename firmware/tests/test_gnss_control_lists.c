/* SPDX-License-Identifier: MIT */
#include "simpleclock/gnss_control.h"
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

/* Independent expanded fixtures from the supplied G5 v1.1.0 pp. 93/119. */
#define PVT                                                                                        \
    "GPSL1CA+GPSL2PY+GPSL2C+GPSL5+GPSL1C+GLOL1CA+GLOL2P+GLOL2CA+GLOL3+"                            \
    "GALE1BC+GALE6BC+GALE5a+GALE5b+GALE5+BDSB1I+BDSB2I+BDSB3I+BDSB1C+"                             \
    "BDSB2a+BDSB2b+QZSL1CA+QZSL2C+QZSL5+QZSL6+QZSL1C+QZSL1S+QZSL1CB+NAVICL5"
#define NAV                                                                                        \
    "GPSL1CA+GPSL2C+GPSL5+GLOL1CA+GLOL2CA+GALE1BC+GALE5a+GALE5b+GALE5+"                            \
    "GEOL1+BDSB1I+BDSB2I+BDSB3I+BDSB1C+BDSB2a+BDSB2b+QZSL1CA+QZSL2C+"                              \
    "QZSL5+QZSL6+QZSL1CB+NAVICL5"
static const char all_signals[] = PVT "+GEOL1+GEOL5";
static sc_gnss_control_lists_t state;
_Static_assert(sizeof(state) <= 232U, "bounded streaming state");
_Static_assert(sizeof(PVT) > 192U, "fixture exceeds the old line buffer");

static unsigned feed(const char *text, bool *matches)
{
    unsigned rows = 0;
    for (; *text; ++text)
    {
        rows += sc_gnss_control_lists_byte(&state, (uint8_t)*text, matches) ? 1U : 0U;
    }
    return rows;
}

int main(void)
{
    bool matches = false;
    CHECK(!sc_gnss_control_lists_init(&state, "getPVTMode"));
    CHECK(feed("Anything, all\r\n", &matches) == 0U && !state.bad);

    CHECK(sc_gnss_control_lists_init(&state, "setSignalUsage,,+GALE6BC\r\n"));
    sc_gnss_control_lists_start(&state, true);
    CHECK(feed("$R: getSignalUsage\r\nReceiverDynamics,Low,Static\r\n", &matches) == 0U);
    CHECK(feed("  SignalUsage, " PVT ", " NAV "\r\n", &matches) == 1U);
    CHECK(!state.bad && !matches);
    sc_gnss_control_lists_start(&state, false);
    CHECK(feed("SignalUsage," PVT ",GALE6BC+" NAV "\r\n", &matches) == 1U);
    CHECK(matches && !state.bad);
    /* An omitted PVT field must be exactly preserved, even with a long reply. */
    sc_gnss_control_lists_start(&state, false);
    CHECK(feed("SignalUsage,GPSL1CA,GALE6BC+" NAV "\n", &matches) == 1U && !matches);

    CHECK(sc_gnss_control_lists_init(&state, "setSignalTracking , all\r\n"));
    sc_gnss_control_lists_start(&state, true);
    CHECK(feed("SignalTracking,GPSL1CA\n", &matches) == 1U && !matches && !state.bad);
    sc_gnss_control_lists_start(&state, false);
    CHECK(feed("SignalTracking,", &matches) == 0U);
    CHECK(feed(all_signals, &matches) == 0U);
    CHECK(feed("\r\n", &matches) == 1U && matches && !state.bad);

    CHECK(sc_gnss_control_lists_init(&state, "setSignalTracking,GALILEO"));
    sc_gnss_control_lists_start(&state, true);
    CHECK(feed("SignalTracking,GALE5+GALE5b+GALE6BC+GALE1BC+GALE5a\r", &matches) == 1U);
    CHECK(matches && !state.bad); /* Ordering and aliases compare as sets. */

    const char *const dependencies[] = {"setSignalTracking,-GPSL1CA", "setSignalTracking,+GLOL2P",
                                        "setSignalTracking,+GLOL3", "setSignalTracking,+QZSL6"};
    for (unsigned i = 0; i < sizeof(dependencies) / sizeof(dependencies[0]); ++i)
    {
        CHECK(sc_gnss_control_lists_init(&state, dependencies[i]));
        sc_gnss_control_lists_start(&state, true);
        CHECK(feed("SignalTracking,GPSL1CA+GPSL2PY\n", &matches) == 1U);
        CHECK(state.bad && !matches); /* Reject before the setter is sent. */
    }
    CHECK(sc_gnss_control_lists_init(&state, "setSignalTracking,-GPSL2PY"));
    sc_gnss_control_lists_start(&state, true);
    CHECK(feed("SignalTracking,GPSL1CA+GPSL2PY\n", &matches) == 1U && !state.bad);
    sc_gnss_control_lists_start(&state, false);
    CHECK(feed("SignalTracking,GPSL1CA\r\n", &matches) == 1U && matches);

    CHECK(sc_gnss_control_lists_init(&state, "setSatelliteTracking,all"));
    sc_gnss_control_lists_start(&state, true);
    CHECK(feed("SatelliteTracking,G01\n", &matches) == 1U && !matches);
    sc_gnss_control_lists_start(&state, false);
    CHECK(feed("SatelliteTracking,", &matches) == 0U);
    unsigned total = 0;
    const char codes[] = "GRESCJI";
    const unsigned first[] = {1, 1, 1, 120, 1, 1, 1}, last[] = {39, 30, 36, 158, 63, 10, 14};
    for (unsigned g = 0; g < 7U; ++g)
    {
        for (unsigned n = first[g]; n <= last[g]; ++n)
        {
            if (g == 0U && n == 37U)
            {
                continue;
            }
            char word[16];
            (void)snprintf(word, sizeof(word), "%s%c%02u", total ? "+" : "", codes[g], n);
            CHECK(feed(word, &matches) == 0U);
            total++;
        }
    }
    CHECK(total == 230U && feed("\r\n", &matches) == 1U && matches && !state.bad);

    CHECK(sc_gnss_control_lists_init(&state, "setSatelliteTracking,+G38+G39+S158+I14"));
    sc_gnss_control_lists_start(&state, true);
    CHECK(feed("SatelliteTracking,G01\n", &matches) == 1U && !state.bad);
    sc_gnss_control_lists_start(&state, false);
    CHECK(feed("SatelliteTracking,I14+S158+G39+G38+G01\n", &matches) == 1U && matches);
    CHECK(sc_gnss_control_lists_init(&state, "setSatelliteUsage,-GALILEO"));
    sc_gnss_control_lists_start(&state, true);
    CHECK(feed("SatelliteUsage,G01+E01+E36\n", &matches) == 1U && !state.bad);
    sc_gnss_control_lists_start(&state, false);
    CHECK(feed("SatelliteUsage,G01\n", &matches) == 1U && matches);

    const char *const invalid[] = {"setSatelliteTracking,G37",  "setSatelliteTracking,S159",
                                   "setSatelliteTracking,I15",  "setSatelliteTracking,C64",
                                   "setSatelliteUsage,G33",     "setSatelliteUsage,S120",
                                   "setSatelliteUsage,I01",     "setSignalUsage,GEOL1",
                                   "setSignalTracking,UNKNOWN", "setSignalTracking,GPSL1CA,extra"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        CHECK(sc_gnss_control_lists_init(&state, invalid[i]) && state.bad);
    }
    const char *const bad_replies[] = {
        "SignalUsage,GPSL1CA,UNKNOWN\n", "SignalUsage,GPSL1CA\n",
        "SignalUsage,GPSL1CA,GPSL1CA,extra\n",
        "SignalUsage,GPSL1CA,THIS_IS_AN_INVALID_TOKEN_LONGER_THAN_32_BYTES\n"};
    for (unsigned i = 0; i < sizeof(bad_replies) / sizeof(bad_replies[0]); ++i)
    {
        CHECK(sc_gnss_control_lists_init(&state, "setSignalUsage,,+GALE6BC"));
        sc_gnss_control_lists_start(&state, true);
        CHECK(feed(bad_replies[i], &matches) == 1U && state.bad && !matches);
    }
    return 0;
}
