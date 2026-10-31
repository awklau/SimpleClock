/* SPDX-License-Identifier: MIT */
/* Recognize one receiver command reply; no UART writes or automatic retries. */
#include "simpleclock/gnss_command.h"
#include <string.h>

static bool elevation_expectation(sc_cmd_expect_t expected)
{
    const unsigned value = (unsigned)expected;
    return expected == SC_CMD_EXPECT_ELEVATION_PVT ||
           expected == SC_CMD_EXPECT_ELEVATION_TRACKING ||
           (value >= SC_CMD_EXPECT_PVT_MASK_BASE && value <= SC_CMD_EXPECT_PVT_MASK_BASE + 180U) ||
           (value >= SC_CMD_EXPECT_TRACKING_MASK_BASE &&
            value <= SC_CMD_EXPECT_TRACKING_MASK_BASE + 180U);
}

static bool elevation_tracking(sc_cmd_expect_t expected)
{
    return expected == SC_CMD_EXPECT_ELEVATION_TRACKING ||
           (unsigned)expected >= SC_CMD_EXPECT_TRACKING_MASK_BASE;
}

/* Replies may render integral angles as 15 or 15.000. Reject fractional angles,
 * overflow, trailing tokens and values outside the supplied G5 1.1.0 p.105 range. */
static bool elevation_number(const char *word, int8_t *degrees)
{
    const bool negative = *word == '-';
    if (negative)
    {
        word++;
    }
    unsigned value = 0U, digits = 0U;
    while (*word >= '0' && *word <= '9')
    {
        if (++digits > 2U)
        {
            return false;
        }
        value = value * 10U + (unsigned)(*word++ - '0');
    }
    if (*word == '.')
    {
        word++;
        if (*word != '0')
        {
            return false;
        }
        while (*word == '0')
        {
            word++;
        }
    }
    if (*word || !digits || value > 90U)
    {
        return false;
    }
    *degrees = (int8_t)(negative ? -(int)value : (int)value);
    return true;
}

static void readback_word(sc_cmd_readback_t *r, sc_cmd_expect_t expected)
{
    if (!r->used)
    {
        return;
    }
    r->word[r->used] = '\0';
    const bool elevation = elevation_expectation(expected);
    const char *prefix =
        expected == SC_CMD_EXPECT_SAVE_BOOT ? "CopyConfigFile"
        : elevation                         ? "ElevationMask"
        : (expected == SC_CMD_EXPECT_HAS_CAPABILITY || expected == SC_CMD_EXPECT_SBAS_CAPABILITY)
            ? "ReceiverCapabilities"
        : expected == SC_CMD_EXPECT_E6_TRACKING   ? "SignalTracking"
        : expected == SC_CMD_EXPECT_E6_NAVIGATION ? "SignalUsage"
                                                  : "PVTMode";
    if (!r->field)
    {
        r->selected = !r->overflow && strcmp(r->word, prefix) == 0;
    }
    else if (r->selected)
    {
        r->bad |= r->overflow;
        if (expected == SC_CMD_EXPECT_SAVE_BOOT)
        {
            const bool valid = (r->field == 1U && strcmp(r->word, "Current") == 0) ||
                               (r->field == 2U && strcmp(r->word, "Boot") == 0);
            const uint8_t bit = r->field == 1U ? 1U : 2U;
            r->bad |= !valid || (r->bits & bit) != 0U;
            r->bits |= bit;
        }
        else if (elevation)
        {
            if (r->field == 1U)
            {
                if (strcmp(r->word, elevation_tracking(expected) ? "Tracking" : "PVT") != 0)
                {
                    r->selected = false; /* Another engine's line cannot satisfy this query. */
                }
                else
                {
                    r->bad |= r->bits != 0U;
                    r->bits = 1U;
                    r->elevation_valid = false;
                }
            }
            else if (r->field == 2U)
            {
                r->bad |= r->bits != 1U || r->elevation_valid ||
                          !elevation_number(r->word, &r->elevation_deg);
                r->elevation_valid = !r->bad;
            }
            else
            {
                r->bad = true;
            }
        }
        else if (expected >= SC_CMD_EXPECT_STANDALONE && expected != SC_CMD_EXPECT_SBAS_CAPABILITY)
        {
            uint8_t bit = 0U;
            if (r->field == 1U && strcmp(r->word, "Rover") == 0)
            {
                bit = 1U;
            }
            if (r->field == 2U && strcmp(r->word, "StandAlone") == 0)
            {
                bit = 2U;
            }
            if (r->field == 2U && strcmp(r->word, "PPP") == 0)
            {
                bit = 4U;
            }
            if (r->field == 2U && strcmp(r->word, "SBAS") == 0)
            {
                bit = 16U;
            }
            if (r->field == 3U && strcmp(r->word, "auto") == 0)
            {
                bit = 8U;
            }
            r->bad |= !bit || (r->bits & bit) != 0U;
            r->bits |= bit;
        }
        else if ((expected == SC_CMD_EXPECT_HAS_CAPABILITY && r->field == 4U &&
                  strcmp(r->word, "PPPGalileoHAS-SIS") == 0) ||
                 (expected == SC_CMD_EXPECT_SBAS_CAPABILITY && r->field == 4U &&
                  strcmp(r->word, "SBAS") == 0) ||
                 (expected == SC_CMD_EXPECT_E6_TRACKING && r->field == 1U &&
                  strcmp(r->word, "GALE6BC") == 0) ||
                 (expected == SC_CMD_EXPECT_E6_NAVIGATION && r->field == 2U &&
                  strcmp(r->word, "GALE6BC") == 0))
        {
            r->bits = 1U;
        }
    }
    r->used = 0U;
    r->overflow = false;
}

static void readback_byte(sc_cmd_readback_t *r, sc_cmd_expect_t expected, uint8_t byte)
{
    if (byte == ',' || byte == '+' || byte == ' ' || byte == '\t' || byte == '\r' || byte == '\n')
    {
        readback_word(r, expected);
        if (byte == ',')
        {
            if (r->field < UINT8_MAX)
            {
                r->field++;
            }
            else
            {
                r->bad = true;
            }
        }
        if (byte == '\r' || byte == '\n')
        {
            if (r->selected)
            {
                bool value = false;
                if (expected == SC_CMD_EXPECT_SAVE_BOOT)
                {
                    value = r->field == 2U && r->bits == 3U;
                }
                else if (elevation_expectation(expected))
                {
                    value = r->field == 2U && r->elevation_valid;
                    if ((unsigned)expected >= SC_CMD_EXPECT_PVT_MASK_BASE)
                    {
                        const unsigned base = elevation_tracking(expected)
                                                  ? SC_CMD_EXPECT_TRACKING_MASK_BASE
                                                  : SC_CMD_EXPECT_PVT_MASK_BASE;
                        value = value && (int)expected - (int)base - 90 == r->elevation_deg;
                    }
                }
                else if (expected >= SC_CMD_EXPECT_STANDALONE &&
                         expected != SC_CMD_EXPECT_SBAS_CAPABILITY)
                {
                    const bool standalone = r->bits == 11U, ppp = r->bits == 15U;
                    const bool sbas = r->bits == 27U, multimode = r->bits == 31U;
                    if (expected == SC_CMD_EXPECT_MULTIMODE)
                    {
                        value = r->field == 3U && multimode;
                    }
                    else if (expected == SC_CMD_EXPECT_STANDALONE_SBAS)
                    {
                        value = r->field == 3U && sbas;
                    }
                    else if (expected == SC_CMD_EXPECT_BENCH_PVT_MODE)
                    {
                        value = r->field == 3U && (standalone || ppp || sbas || multimode);
                    }
                    else
                    {
                        value = r->field == 3U &&
                                ((expected != SC_CMD_EXPECT_STANDALONE_PPP && standalone) ||
                                 (expected != SC_CMD_EXPECT_STANDALONE && ppp));
                    }
                }
                else
                {
                    value =
                        r->bits == 1U && ((expected == SC_CMD_EXPECT_HAS_CAPABILITY ||
                                           expected == SC_CMD_EXPECT_SBAS_CAPABILITY)
                                              ? r->field >= 4U
                                          : expected == SC_CMD_EXPECT_E6_TRACKING ? r->field == 1U
                                                                                  : r->field == 2U);
                }
                /* A second matching response line is ambiguous, not a repair. */
                r->matched = value && !r->bad && !r->seen;
                r->seen = true;
                if (elevation_expectation(expected))
                {
                    r->elevation_valid = r->matched;
                }
            }
            r->field = r->bits = 0U;
            r->selected = r->bad = false;
        }
    }
    else if (r->used + 1U < sizeof(r->word))
    {
        r->word[r->used++] = (char)byte;
    }
    else
    {
        r->overflow = true;
    }
}

bool sc_gnss_command_start(sc_gnss_command_t *transaction, const char *command, uint32_t now_ms)
{
    return sc_gnss_command_start_checked(transaction, command, SC_CMD_EXPECT_NONE, now_ms);
}

bool sc_gnss_command_start_checked(sc_gnss_command_t *transaction, const char *command,
                                   sc_cmd_expect_t expected, uint32_t now_ms)
{
    if (transaction->state == SC_CMD_WAITING || command == NULL ||
        strcspn(command, "\r\n") + 5U >= sizeof(transaction->line) ||
        ((unsigned)expected > SC_CMD_EXPECT_SAVE_BOOT && !elevation_expectation(expected)))
    {
        return false;
    }
    memset(transaction, 0, sizeof(*transaction));
    transaction->command = command;
    transaction->started_ms = now_ms;
    transaction->state = SC_CMD_WAITING;
    transaction->expected = expected;
    return true;
}

void sc_gnss_command_byte(sc_gnss_command_t *transaction, uint8_t byte)
{
    if (transaction->state != SC_CMD_WAITING)
    {
        return;
    }
    if (transaction->expected != SC_CMD_EXPECT_NONE && transaction->echo)
    {
        readback_byte(&transaction->readback, transaction->expected, byte);
    }
    if (byte == '\r' || byte == '\n')
    {
        if (!transaction->overflow && transaction->used >= 3U)
        {
            const size_t command_length = strcspn(transaction->command, "\r\n");
            /* No asynchronous receiver message or mismatched echo can ACK. */
            if ((strncmp(transaction->line, "$R:", 3) == 0 ||
                 strncmp(transaction->line, "$R;", 3) == 0) &&
                transaction->used == command_length + 4U && transaction->line[3] == ' ' &&
                strncmp(transaction->line + 4, transaction->command, command_length) == 0)
            {
                transaction->echo = true;
            }
            /* A receiver error always fails closed, even if it lacks an echo. */
            if (strncmp(transaction->line, "$R?", 3) == 0 ||
                strncmp(transaction->line, "$R!", 3) == 0)
            {
                transaction->rejected = true;
            }
        }
        transaction->used = 0U;
        transaction->overflow = false;
        transaction->line[0] = '\0';
    }
    else if (!transaction->overflow && transaction->used + 1U < sizeof(transaction->line))
    {
        transaction->line[transaction->used++] = (char)byte;
        transaction->line[transaction->used] = '\0';
        if (transaction->used == 3U &&
            (!memcmp(transaction->line, "$R?", 3U) || !memcmp(transaction->line, "$R!", 3U)))
        {
            transaction->rejected = true;
        }
        if (strcmp(transaction->line, "COM1>") == 0)
        {
            if (transaction->rejected)
            {
                transaction->state = SC_CMD_REJECTED;
            }
            else if (transaction->echo)
            {
                transaction->state =
                    transaction->expected == SC_CMD_EXPECT_NONE || transaction->readback.matched
                        ? SC_CMD_OK
                        : SC_CMD_REJECTED;
            }
            /* Prompts are not newline terminated and can precede an echo. */
            transaction->used = 0U;
            transaction->line[0] = '\0';
        }
    }
    else
    {
        transaction->overflow = true;
    }
}

void sc_gnss_command_expire(sc_gnss_command_t *transaction, uint32_t now_ms)
{
    if (transaction->state == SC_CMD_WAITING &&
        (uint32_t)(now_ms - transaction->started_ms) > 3000U)
    {
        transaction->state = SC_CMD_TIMEOUT;
    }
}
