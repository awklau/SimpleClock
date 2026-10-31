/* SPDX-License-Identifier: MIT */
#include "simpleclock/gnss_control.h"
#include <string.h>

typedef struct
{
    const char *p;
    size_t n;
} word_t;

/* Family order matches names[]; mode zero preserves an omitted argument. */
enum
{
    LIST_SIGNAL_TRACKING = 1,
    LIST_SIGNAL_USAGE = 2,
    LIST_SATELLITE_TRACKING = 3,
    LIST_SATELLITE_USAGE = 4
};

enum
{
    LIST_REMOVE = -1,
    LIST_PRESERVE = 0,
    LIST_ADD = 1,
    LIST_REPLACE = 2
};

static bool eq(word_t w, const char *s)
{
    return w.n == strlen(s) && !memcmp(w.p, s, w.n);
}

static const char *const names[] = {"SignalTracking", "SignalUsage", "SatelliteTracking",
                                    "SatelliteUsage"};
static const char *const signals[] = {
    "GPSL1CA", "GPSL2PY", "GPSL2C",  "GPSL5",  "GPSL1C",  "GLOL1CA", "GLOL2P",  "GLOL2CA",
    "GLOL3",   "GALE1BC", "GALE6BC", "GALE5a", "GALE5b",  "GALE5",   "GEOL1",   "GEOL5",
    "BDSB1I",  "BDSB2I",  "BDSB3I",  "BDSB1C", "BDSB2a",  "BDSB2b",  "QZSL1CA", "QZSL2C",
    "QZSL5",   "QZSL6",   "QZSL1C",  "QZSL1S", "QZSL1CB", "NAVICL5"};
static const char *const groups[] = {"GPS",    "GLONASS", "GALILEO", "SBAS",
                                     "BEIDOU", "QZSS",    "NAVIC"};

static void bit(uint32_t *set, unsigned i)
{
    set[i / 32U] |= UINT32_C(1) << (i % 32U);
}

static bool has(const uint32_t *set, unsigned i)
{
    return (set[i / 32U] & (UINT32_C(1) << (i % 32U))) != 0U;
}

static bool token(uint32_t *set, word_t w, unsigned family, unsigned field)
{
    if (eq(w, "none"))
    {
        return true;
    }
    const bool all = eq(w, "all");
    int group = -1;
    for (unsigned i = 0; i < 7U; ++i)
    {
        if (eq(w, groups[i]))
        {
            group = (int)i;
        }
    }
    if (family <= LIST_SIGNAL_USAGE)
    {
        static const char *const prefixes[] = {"GPS", "GLO", "GAL", "GEO", "BDS", "QZS", "NAV"};
        bool found = false;
        for (unsigned i = 0; i < sizeof(signals) / sizeof(signals[0]); ++i)
        {
            if (all || eq(w, signals[i]) ||
                (group >= 0 && !strncmp(signals[i], prefixes[(unsigned)group], 3)))
            {
                found = true;
                if (family == LIST_SIGNAL_USAGE && field == 0U && !strncmp(signals[i], "GEO", 3))
                {
                    if (!all && group < 0)
                    {
                        return false;
                    }
                }
                else
                {
                    bit(set, i);
                }
            }
        }
        return found;
    }
    /* G5 v1.1.0 pp. 89/116: disjoint ranges within 256 bits. GPS omits G37;
     * SatelliteUsage also excludes G33..G39, SBAS and NavIC. Highest bit = 231. */
    static const char codes[] = "GRESCJI";
    static const unsigned bases[] = {0U, 40U, 70U, 106U, 145U, 208U, 218U};
    static const unsigned first[] = {1U, 1U, 1U, 120U, 1U, 1U, 1U};
    static const unsigned last[] = {39U, 30U, 36U, 158U, 63U, 10U, 14U};
    for (unsigned g = 0; g < 7U; ++g)
    {
        unsigned number = 0U;
        bool explicit_id = w.n >= 2U && w.p[0] == codes[g];
        if (explicit_id)
        {
            for (size_t i = 1; i < w.n; ++i)
            {
                if (w.p[i] < '0' || w.p[i] > '9' || number > 1000U)
                {
                    explicit_id = false;
                    break;
                }
                number = number * 10U + (unsigned)(w.p[i] - '0');
            }
        }
        if (!all && group != (int)g && !explicit_id)
        {
            continue;
        }
        for (unsigned n = first[g]; n <= last[g]; ++n)
        {
            if (g == 0U && n == 37U)
            {
                continue;
            }
            if (family == LIST_SATELLITE_USAGE && ((g == 0U && n > 32U) || g == 3U || g == 6U))
            {
                continue;
            }
            if (!explicit_id || n == number)
            {
                bit(set, bases[g] + n - first[g]);
            }
        }
        if (explicit_id)
        {
            return number >= first[g] && number <= last[g] && !(g == 0U && number == 37U) &&
                   !(family == LIST_SATELLITE_USAGE &&
                     ((g == 0U && number > 32U) || g == 3U || g == 6U));
        }
    }
    return all || group >= 0;
}

static bool value(uint32_t *set, word_t w, unsigned family, unsigned field)
{
    size_t start = 0U;
    for (size_t i = 0; i <= w.n; ++i)
    {
        if (i == w.n || w.p[i] == '+')
        {
            if (!token(set, (word_t){w.p + start, i - start}, family, field))
            {
                return false;
            }
            start = i + 1U;
        }
    }
    return true;
}

bool sc_gnss_control_lists_init(sc_gnss_control_lists_t *s, const char *command)
{
    memset(s, 0, sizeof(*s));
    if (strncmp(command, "set", 3))
    {
        return false;
    }
    const size_t raw_length = strcspn(command + 3, ",\r\n");
    size_t length = raw_length;
    while (length && command[3U + length - 1U] == ' ')
    {
        --length;
    }
    for (unsigned i = 0; i < 4U; ++i)
    {
        if (eq((word_t){command + 3, length}, names[i]))
        {
            s->family = (uint8_t)(i + 1U);
        }
    }
    if (!s->family)
    {
        return false;
    }
    const char *p = command + 3 + raw_length;
    const unsigned count = s->family == LIST_SIGNAL_USAGE ? 2U : 1U;
    for (unsigned i = 0; i < count && *p == ','; ++i)
    {
        ++p;
        while (*p == ' ')
        {
            ++p;
        }
        const char *start = p;
        while (*p && *p != ',' && *p != '\r' && *p != '\n')
        {
            ++p;
        }
        const char *end = p;
        while (end > start && end[-1] == ' ')
        {
            --end;
        }
        if (end == start)
        {
            continue;
        }
        s->mode[i] = LIST_REPLACE;
        if (*start == '+' || *start == '-')
        {
            s->mode[i] = (int8_t)(*start == '+' ? LIST_ADD : LIST_REMOVE);
            ++start;
        }
        if (!value(s->wanted[i], (word_t){start, (size_t)(end - start)}, s->family, i))
        {
            s->bad = true;
        }
    }
    if (*p == ',')
    {
        s->bad = true;
    }
    return true;
}

void sc_gnss_control_lists_start(sc_gnss_control_lists_t *s, bool before)
{
    memset(s->actual, 0, sizeof(s->actual));
    s->field = s->used = 0U;
    s->selected = false;
    s->before = before;
}

static void finish_token(sc_gnss_control_lists_t *s)
{
    if (s->selected && s->field >= 1U && s->field <= 2U && s->used &&
        !token(s->actual[s->field - 1U], (word_t){s->word, s->used}, s->family, s->field - 1U))
    {
        s->bad = true;
    }
    s->used = 0U;
}

bool sc_gnss_control_lists_byte(sc_gnss_control_lists_t *s, uint8_t byte, bool *matches)
{
    if (!s->family)
    {
        return false;
    }
    if (byte == '\r' || byte == '\n')
    {
        const bool selected = s->selected;
        if (selected)
        {
            finish_token(s);
            const unsigned count = s->family == LIST_SIGNAL_USAGE ? 2U : 1U;
            if (s->field != count)
            {
                s->bad = true;
            }
            if (s->before)
            {
                for (unsigned f = 0; f < count; ++f)
                {
                    for (unsigned w = 0; w < 8U; ++w)
                    {
                        const uint32_t a = s->actual[f][w], want = s->wanted[f][w];
                        s->expected[f][w] = s->mode[f] == LIST_REPLACE  ? want
                                            : s->mode[f] == LIST_ADD    ? a | want
                                            : s->mode[f] == LIST_REMOVE ? a & ~want
                                                                        : a;
                    }
                }
                if (s->family == LIST_SIGNAL_TRACKING)
                {
                    const uint32_t *set = s->expected[0];
                    if ((has(set, 1U) && !has(set, 0U)) || (has(set, 6U) && !has(set, 7U)) ||
                        (has(set, 8U) && !has(set, 5U)) ||
                        (has(set, 25U) && !has(set, 22U) && !has(set, 28U)))
                    {
                        s->bad = true;
                    }
                }
            }
            *matches = !s->bad && !memcmp(s->actual, s->expected, sizeof(s->actual));
        }
        memset(s->actual, 0, sizeof(s->actual));
        s->selected = false;
        s->used = s->field = 0U;
        return selected;
    }
    if (byte == ',')
    {
        if (!s->field)
        {
            s->selected = eq((word_t){s->word, s->used}, names[s->family - 1U]);
        }
        else
        {
            finish_token(s);
        }
        s->used = 0U;
        if (s->field < 255U)
        {
            ++s->field;
        }
        return false;
    }
    if (byte == '+' && s->selected)
    {
        finish_token(s);
        return false;
    }
    if (byte == ' ' || byte == '\t')
    {
        return false;
    }
    if (s->used < sizeof(s->word))
    {
        s->word[s->used++] = (char)byte;
    }
    else if (s->selected)
    {
        s->bad = true;
    }
    return false;
}
