/* SPDX-License-Identifier: MIT */
#include "simpleclock/gnss_control.h"
#include "simpleclock/settings.h"
#include <string.h>

typedef struct
{
    const char *p;
    size_t n;
} field_t;

typedef struct
{
    const char *name;
    uint8_t fields, selector;
} setting_t;

/* R and D families in the 2026-10-18 G5-T audit. Transport-owned exceptions
 * SBF/NMEA are restricted below to slow, bounded COM1 diagnostic output. */
static const setting_t settings[] = {{"CalibCommonDelay", 1, 0},
                                     {"CalibSignalDelay", 2, 1},
                                     {"ChannelAllocation", 5, 1},
                                     {"CN0Mask", 2, 1},
                                     {"MultipathMitigation", 2, 0},
                                     {"SatelliteTracking", 1, 0},
                                     {"SignalTracking", 1, 0},
                                     {"SmoothingInterval", 3, 1},
                                     {"AGCMode", 3, 1},
                                     {"BBSamplingMode", 1, 0},
                                     {"ManualNotchFilter", 4, 1},
                                     {"RFInterferenceMitigation", 1, 0},
                                     {"AntennaOffset", 7, 1},
                                     {"DiffCorrMaxAge", 3, 0},
                                     {"DiffCorrUsage", 5, 0},
                                     {"ElevationMask", 2, 1},
                                     {"IonosphereModel", 1, 0},
                                     {"PVTMode", 3, 0},
                                     {"ReceiverDynamics", 2, 0},
                                     {"SatelliteUsage", 1, 0},
                                     {"SBASCorrections", 2, 0},
                                     {"SignalUsage", 2, 0},
                                     {"StaticPosCartesian", 5, 1},
                                     {"StaticPosGeodetic", 5, 1},
                                     {"TroposphereModel", 2, 0},
                                     {"TroposphereParameters", 3, 0},
                                     {"GalOSNMAUsage", 3, 0},
                                     {"SignalAuthentication", 1, 0},
                                     {"GeodeticDatum", 1, 0},
                                     {"ClockSyncThreshold", 2, 0},
                                     {"HoldoverTrigger", 1, 0},
                                     {"PPSParameters", 6, 0},
                                     {"TimingSystem", 1, 0},
                                     {"MarkerParameters", 7, 0},
                                     {"NMEAOutput", 4, 1},
                                     {"SBFOutput", 4, 1}};

static unsigned split(const char *s, field_t *f, unsigned capacity)
{
    unsigned count = 0;
    for (;;)
    {
        if (count == capacity)
        {
            return 0;
        }
        while (*s == ' ')
        {
            ++s;
        }
        const char *start = s;
        bool quoted = false;
        while (*s && *s != '\r' && *s != '\n')
        {
            if (*s == '"')
            {
                quoted = !quoted;
            }
            if (*s == ',' && !quoted)
            {
                break;
            }
            ++s;
        }
        if (quoted)
        {
            return 0;
        }
        const char *end = s;
        while (end > start && end[-1] == ' ')
        {
            --end;
        }
        f[count++] = (field_t){start, (size_t)(end - start)};
        if (*s != ',')
        {
            return count;
        }
        ++s;
    }
}

static bool eq(field_t f, const char *s)
{
    return strlen(s) == f.n && memcmp(f.p, s, f.n) == 0;
}

static bool eq_fold(field_t f, const char *s)
{
    if (strlen(s) != f.n)
    {
        return false;
    }
    for (size_t i = 0; i < f.n; ++i)
    {
        char a = f.p[i], b = s[i];
        if (a >= 'A' && a <= 'Z')
        {
            a = (char)(a + ('a' - 'A'));
        }
        if (b >= 'A' && b <= 'Z')
        {
            b = (char)(b + ('a' - 'A'));
        }
        if (a != b)
        {
            return false;
        }
    }
    return true;
}

static const setting_t *lookup(field_t name)
{
    for (unsigned i = 0; i < sizeof(settings) / sizeof(settings[0]); ++i)
    {
        if (eq(name, settings[i].name))
        {
            return &settings[i];
        }
    }
    return NULL;
}

static bool choice(field_t f, const char *options)
{
    const char *p = options;
    do
    {
        const char *end = strchr(p, '|');
        const size_t n = end ? (size_t)(end - p) : strlen(p);
        if (n == f.n && memcmp(p, f.p, n) == 0)
        {
            return true;
        }
        if (!end)
        {
            break;
        }
        p = end + 1;
    } while (*p);
    return false;
}

static bool list(field_t f, const char *options)
{
    if (!f.n || f.p[0] == '+' || f.p[0] == '-')
    {
        return false;
    }
    size_t start = 0;
    for (size_t i = 0; i <= f.n; ++i)
    {
        if (i == f.n || f.p[i] == '+')
        {
            if (!choice((field_t){f.p + start, i - start}, options))
            {
                return false;
            }
            start = i + 1;
        }
    }
    return true;
}

bool sc_gnss_control_pvt_policy(const char *line, uint32_t *selection, bool *fixed)
{
    field_t f[4];
    if (split(line, f, 4U) != 4U || !eq(f[0], "PVTMode") || !choice(f[1], "Static|Rover") ||
        !choice(f[3], "auto|Geodetic1|Geodetic2|Geodetic3|Geodetic4|Geodetic5|"
                      "Cartesian1|Cartesian2|Cartesian3|Cartesian4|Cartesian5"))
    {
        return false;
    }
    unsigned bits = 0U;
    size_t start = 0U;
    for (size_t i = 0U; i <= f[2].n; ++i)
    {
        if (i == f[2].n || f[2].p[i] == '+')
        {
            const field_t mode = {f[2].p + start, i - start};
            const unsigned bit = eq(mode, "StandAlone") ? 1U
                                 : eq(mode, "SBAS")     ? 2U
                                 : eq(mode, "PPP")      ? 4U
                                                        : 0U;
            if (!bit || (bits & bit))
            {
                return false;
            }
            bits |= bit;
            start = i + 1U;
        }
    }
    const bool is_fixed = eq(f[1], "Static");
    if (!(bits & 1U) || (is_fixed ? (bits & 4U) != 0U : !eq(f[3], "auto")))
    {
        return false;
    }
    *selection = bits == 1U   ? SC_SETTINGS_GNSS_STANDALONE
                 : bits == 3U ? SC_SETTINGS_GNSS_SBAS
                 : bits == 5U ? SC_SETTINGS_GNSS_HAS
                              : SC_SETTINGS_GNSS_AUTO;
    *fixed = is_fixed;
    return true;
}

static bool integer(field_t f, int lo, int hi)
{
    size_t i = 0;
    bool negative = f.n && f.p[0] == '-';
    if (negative)
    {
        ++i;
    }
    if (i == f.n)
    {
        return false;
    }
    int n = 0;
    for (; i < f.n; ++i)
    {
        if (f.p[i] < '0' || f.p[i] > '9' || n > 100000)
        {
            return false;
        }
        n = n * 10 + (f.p[i] - '0');
    }
    if (negative)
    {
        n = -n;
    }
    return n >= lo && n <= hi;
}

static bool block_list(field_t f)
{
    unsigned count = 1U;
    for (size_t i = 0; i < f.n; ++i)
    {
        if (f.p[i] == '+')
        {
            ++count;
        }
    }
    return count <= 4U &&
           list(f, "PVTGeodetic|PVTCartesian|PosCovCartesian|VelCovCartesian|DOP|SatVisibility|"
                   "ChannelStatus|RFStatus|GALAuthStatus|ReceiverStatus|ReceiverTime|ReceiverSetup|"
                   "InputLink|OutputLink|EndOfPVT|QualityInd|xPPSOffset|RxMessage|GALGstGps");
}

static bool allowed_write(const setting_t *s, field_t *f)
{
    /* Host catalog owns full version-specific types. These firmware restrictions
     * cannot be bypassed by a client sending native expert text directly. */
    const char *name = s->name;
    if (!strcmp(name, "PVTMode"))
    {
        return (!f[1].n || choice(f[1], "Static|Rover")) &&
               (!f[2].n || list(f[2], "StandAlone|SBAS|PPP")) &&
               (!f[3].n || choice(f[3], "auto|Geodetic1|Geodetic2|Geodetic3|Geodetic4|Geodetic5|"
                                        "Cartesian1|Cartesian2|Cartesian3|Cartesian4|Cartesian5"));
    }
    if (!strcmp(name, "ReceiverDynamics"))
    {
        return (!f[1].n || choice(f[1], "High|Moderate|Low")) &&
               (!f[2].n || choice(f[2], "Static|Quasistatic|Pedestrian|Automotive|RaceCar|"
                                        "HeavyMachinery|UAV|Unlimited"));
    }
    if (!strcmp(name, "MultipathMitigation"))
    {
        return (!f[1].n || choice(f[1], "on|off")) && (!f[2].n || choice(f[2], "on|off"));
    }
    if (!strcmp(name, "CN0Mask"))
    {
        return !eq(f[1], "GPSL2PY") && !eq(f[1], "Reserved2") && integer(f[2], 0, 60);
    }
    if (!strcmp(name, "ElevationMask"))
    {
        return choice(f[1], "PVT|Tracking") && integer(f[2], -90, 90);
    }
    if (!strcmp(name, "DiffCorrMaxAge"))
    {
        return !f[1].n && !f[2].n && f[3].n;
    }
    if (!strcmp(name, "DiffCorrUsage"))
    {
        return !f[1].n && f[2].n && !f[3].n && !f[4].n && !f[5].n;
    }
    if (!strcmp(name, "GalOSNMAUsage"))
    {
        return !f[2].n;
    }
    if (!strcmp(name, "PPSParameters"))
    {
        return !f[1].n || choice(f[1], "off|sec1");
    }
    if (!strcmp(name, "ClockSyncThreshold"))
    {
        return !f[1].n || choice(f[1], "ClockSteering|usec500");
    }
    if (!strcmp(name, "AntennaOffset"))
    {
        return eq(f[1], "Main");
    }
    if (!strcmp(name, "SBFOutput") || !strcmp(name, "NMEAOutput"))
    {
        return eq(f[1], "Stream3") && eq(f[2], "COM1") &&
               (!strcmp(name, "SBFOutput") ? (eq(f[3], "none") || block_list(f[3]))
                                           : (eq(f[3], "none") || eq(f[3], "GSV"))) &&
               choice(f[4], "sec5|sec10|sec30|sec60|off");
    }
    return true;
}

unsigned sc_gnss_control_prepare(const char *usb, char *command, size_t capacity, char *query,
                                 size_t query_capacity)
{
    const char *p;
    unsigned kind;
    if (!strncmp(usb, "gnss read ", 10))
    {
        p = usb + 10;
        kind = SC_GNSS_CONTROL_READ;
    }
    else if (!strncmp(usb, "gnss write ", 11))
    {
        p = usb + 11;
        kind = SC_GNSS_CONTROL_WRITE;
    }
    else if (!strncmp(usb, "gnss exec ", 10))
    {
        p = usb + 10;
        kind = SC_GNSS_CONTROL_ACTION;
    }
    else
    {
        return 0;
    }
    size_t length = strlen(p);
    if (length + 3 > capacity || !length)
    {
        return 0;
    }
    for (size_t i = 0; i < length; ++i)
    {
        if (p[i] < 32 || p[i] > 126 || p[i] == ';' || p[i] == '\\')
        {
            return 0;
        }
    }
    field_t f[8] = {{0}};
    unsigned n = split(p, f, 8);
    if (!n || f[0].n < 4)
    {
        return 0;
    }
    query[0] = '\0';
    if (kind == SC_GNSS_CONTROL_READ)
    {
        if (strncmp(p, "get", 3))
        {
            if (!(eq(f[0], "lstAntennaInfo") || eq(f[0], "lstCommandHelp") ||
                  eq(f[0], "lstMIBDescription") || eq(f[0], "lstAsciiDisplay") ||
                  eq(f[0], "lstCurrentUser") || eq(f[0], "lstGalOSNMAPublicKeys") ||
                  (eq(f[0], "lstConfigFile") && n == 2 && choice(f[1], "Current|Boot|RxDefault")) ||
                  (eq(f[0], "lstInternalFile") && n == 2 &&
                   choice(f[1], "Identification|Permissions|RxMessages|Error|SisError|"
                                "DiffCorrError|SetupError"))))
            {
                return 0;
            }
        }
        else if (eq_fold(f[0], "getUserAccessLevel"))
        {
            return 0;
        }
    }
    else if (kind == SC_GNSS_CONTROL_ACTION)
    {
        if (!((eq(f[0], "exeResetNavFilter") && n == 2 && eq(f[1], "PVT")) ||
              (eq(f[0], "exeCopyConfigFile") && n == 3 && eq(f[1], "Current") &&
               eq(f[2], "Boot")) ||
              (eq(f[0], "exeSBFOnce") && n == 3 && eq(f[1], "COM1") && block_list(f[2])) ||
              (eq(f[0], "exeNMEAOnce") && n == 3 && eq(f[1], "COM1") && eq(f[2], "GSV"))))
        {
            return 0;
        }
    }
    else
    {
        if (strncmp(p, "set", 3))
        {
            return 0;
        }
        const setting_t *s = lookup((field_t){f[0].p + 3, f[0].n - 3});
        if (!s || n > s->fields + 1U || !allowed_write(s, f))
        {
            return 0;
        }
        bool changed = false;
        for (unsigned i = 1; i < n; ++i)
        {
            changed = changed || f[i].n != 0;
        }
        if (!changed ||
            (s->selector && (!f[1].n || eq(f[1], "all") || memchr(f[1].p, '+', f[1].n))))
        {
            return 0;
        }
        size_t used = f[0].n;
        if (used + 3 > query_capacity)
        {
            return 0;
        }
        memcpy(query, p, used);
        query[0] = 'g';
        if (s->selector)
        {
            if (used + f[1].n + 4 > query_capacity)
            {
                return 0;
            }
            query[used++] = ',';
            memcpy(query + used, f[1].p, f[1].n);
            used += f[1].n;
        }
        memcpy(query + used, "\r\n", 3);
    }
    memcpy(command, p, length);
    memcpy(command + length, "\r\n", 3);
    return kind;
}

static field_t normalized(field_t f)
{
    if (f.n >= 2 && f.p[0] == '"' && f.p[f.n - 1] == '"')
    {
        ++f.p;
        f.n -= 2;
        return f;
    }
    size_t i = (f.n && (f.p[0] == '-' || f.p[0] == '+')) ? 1U : 0U;
    bool numeric = i < f.n, dot = false;
    for (size_t j = i; j < f.n; ++j)
    {
        if (f.p[j] == '.' && !dot)
        {
            dot = true;
        }
        else if (f.p[j] < '0' || f.p[j] > '9')
        {
            numeric = false;
        }
    }
    if (numeric)
    {
        if (f.p[0] == '+')
        {
            ++f.p;
            --f.n;
        }
        if (dot)
        {
            while (f.n && f.p[f.n - 1] == '0')
            {
                --f.n;
            }
            if (f.n && f.p[f.n - 1] == '.')
            {
                --f.n;
            }
        }
        if (!f.n || (f.n == 1 && f.p[0] == '-'))
        {
            return (field_t){"0", 1};
        }
        if (f.n == 2 && !memcmp(f.p, "-0", 2))
        {
            return (field_t){"0", 1};
        }
    }
    return f;
}

static bool equal(field_t a, field_t b)
{
    a = normalized(a);
    b = normalized(b);
    if (a.n == b.n && !memcmp(a.p, b.p, a.n))
    {
        return true;
    }
    /* Scalar-family lists compare as sets. Signal/satellite +/- operations use
     * the separate streaming verifier with their complete Current baseline. */
    if (!memchr(a.p, '+', a.n) && !memchr(b.p, '+', b.n))
    {
        return false;
    }
    for (unsigned side = 0; side < 2; ++side)
    {
        field_t x = side ? b : a, y = side ? a : b;
        size_t start = 0;
        for (size_t i = 0; i <= x.n; ++i)
        {
            if (i == x.n || x.p[i] == '+')
            {
                bool found = false;
                size_t ys = 0;
                for (size_t j = 0; j <= y.n; ++j)
                {
                    if (j == y.n || y.p[j] == '+')
                    {
                        if (i - start == j - ys && !memcmp(x.p + start, y.p + ys, i - start))
                        {
                            found = true;
                        }
                        ys = j + 1;
                    }
                }
                if (!found)
                {
                    return false;
                }
                start = i + 1;
            }
        }
    }
    return true;
}

bool sc_gnss_control_readback(const char *command, const char *line, bool *matches,
                              uint32_t *preserved)
{
    field_t wanted[8] = {{0}}, actual[8] = {{0}};
    unsigned nw = split(command, wanted, 8), na = split(line, actual, 8);
    if (!nw || !na || wanted[0].n < 4)
    {
        return false;
    }
    const setting_t *s = lookup((field_t){wanted[0].p + 3, wanted[0].n - 3});
    if (!s || !eq(actual[0], s->name) || na < nw)
    {
        return false;
    }
    if (!strcmp(s->name, "DiffCorrMaxAge") && na != 4U)
    {
        return false;
    }
    if (s->selector && !equal(wanted[1], actual[1]))
    {
        return false;
    }
    *matches = true;
    *preserved = 2166136261U;
    for (unsigned i = 1; i < na; ++i)
    {
        if (i < nw && wanted[i].n)
        {
            if (!equal(wanted[i], actual[i]))
            {
                *matches = false;
            }
        }
        else
        {
            field_t f = normalized(actual[i]);
            for (size_t j = 0; j < f.n; ++j)
            {
                *preserved = (*preserved ^ (uint8_t)f.p[j]) * 16777619U;
            }
            *preserved = (*preserved ^ 255U) * 16777619U;
        }
    }
    return true;
}
