/* SPDX-License-Identifier: MIT */
#include "simpleclock/temperature.h"
#include "simpleclock/text.h"
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

static int transfer(sc_temperature_t *state, uint32_t ms, uint8_t reg, bool write, uint16_t value)
{
    sc_temperature_transfer_t request;
    CHECK(sc_temperature_begin(state, ms, &request));
    CHECK(request.reg == reg && request.write == write);
    CHECK(!write || request.value == value);
    CHECK(!sc_temperature_begin(state, ms, &request));
    sc_temperature_complete(state, ms, true, write ? 0 : value);
    return 0;
}

static int trigger(sc_temperature_t *state, uint32_t ms)
{
    CHECK(transfer(state, ms, 15, false, 0x0117) == 0);
    CHECK(transfer(state, ms + 1U, 4, false, 0) == 0); /* Verify EEPROM locked. */
    CHECK(transfer(state, ms + 2U, 1, true, 0x0620) == 0);
    CHECK(transfer(state, ms + 3U, 1, false, 0x2620) == 0); /* Clear old ready. */
    CHECK(transfer(state, ms + 4U, 1, true, 0x0e20) == 0);
    return 0;
}

static int sample(sc_temperature_t *state, uint32_t ms, uint16_t raw)
{
    sc_temperature_transfer_t request;
    CHECK(trigger(state, ms) == 0);
    CHECK(!sc_temperature_begin(state, ms + 203U, &request));
    CHECK(transfer(state, ms + 204U, 1, false, 0x2e20) == 0);
    CHECK(transfer(state, ms + 205U, 0, false, raw) == 0);
    return 0;
}

int main(void)
{
    sc_temperature_t state = {0};
    sc_temperature_transfer_t request;
    CHECK(!sc_temperature_fresh(&state, 0));
    CHECK(sample(&state, 0, 3200) == 0);
    CHECK(state.sequence == 1 && state.sample_ms == 4 && state.raw == 3200);
    CHECK(sc_temperature_fresh(&state, 2503) && !sc_temperature_fresh(&state, 2504));
    CHECK(!sc_temperature_begin(&state, 999, &request));
    CHECK(sample(&state, 1000, 3200) == 0); /* Same value, new commanded conversion. */
    CHECK(state.sequence == 2 && state.sample_ms == 1004);
    CHECK(trigger(&state, 2000) == 0);
    CHECK(transfer(&state, 2204, 1, false, 0x0e20) == 0); /* ACKed trigger, no ready. */
    CHECK(!state.valid && state.status == SC_TEMPERATURE_INVALID);
    CHECK(state.sequence == 2 && state.sample_ms == 1004); /* Never renew old data. */
    CHECK(!sc_temperature_begin(&state, 3203, &request));
    CHECK(sample(&state, 3204, 3200) == 0 && state.valid); /* Bounded recovery. */

    /* Neither an unlocked nor a busy EEPROM may be followed by any write. */
    const uint16_t lock_failures[] = {0x8000, 0x4000, 0xc000};
    for (unsigned i = 0; i < 3; i++)
    {
        state = (sc_temperature_t){0};
        CHECK(transfer(&state, 0, 15, false, 0x0117) == 0);
        CHECK(transfer(&state, 1, 4, false, lock_failures[i]) == 0);
        CHECK(state.status == SC_TEMPERATURE_BAD_CONFIG && !state.valid);
        CHECK(!sc_temperature_begin(&state, 2, &request));
        CHECK(transfer(&state, 1001, 15, false, 0x0117) == 0);
    }
    const uint16_t config_failures[] = {0x2220, 0x2620, 0x3e20, 0x2e40};
    for (unsigned i = 0; i < 4; i++)
    {
        state = (sc_temperature_t){0};
        CHECK(trigger(&state, 0) == 0);
        CHECK(transfer(&state, 204, 1, false, config_failures[i]) == 0);
        CHECK(state.status == SC_TEMPERATURE_BAD_CONFIG && !state.valid);
    }
    state = (sc_temperature_t){0};
    CHECK(transfer(&state, 0, 15, false, 0xffff) == 0);
    CHECK(state.status == SC_TEMPERATURE_BAD_ID && !state.valid);
    state = (sc_temperature_t){0};
    CHECK(sample(&state, 0, 0x8000) == 0);
    CHECK(state.status == SC_TEMPERATURE_INVALID && !state.valid);

    state = (sc_temperature_t){0};
    CHECK(trigger(&state, 0) == 0);
    CHECK(transfer(&state, 204, 1, false, 0x2e20) == 0);
    CHECK(!sc_temperature_begin(&state, 304, &request)); /* Ready-to-read deadline. */
    CHECK(state.status == SC_TEMPERATURE_INVALID && !state.valid);
    CHECK(trigger(&state, 1304) == 0);
    CHECK(transfer(&state, 3808, 1, false, 0x2e20) == 0); /* Delayed result stays stale. */
    CHECK(state.status == SC_TEMPERATURE_INVALID && !state.valid);

    state = (sc_temperature_t){0};
    CHECK(transfer(&state, 0, 15, false, 0x0117) == 0);
    CHECK(transfer(&state, 1, 4, false, 0) == 0);
    CHECK(sc_temperature_begin(&state, 2, &request) && request.write);
    sc_temperature_complete(&state, 7, false, 0); /* Failed write invalidates immediately. */
    CHECK(state.status == SC_TEMPERATURE_I2C_ERROR && !state.valid);
    CHECK(sc_temperature_begin(&state, 1007, &request));
    sc_temperature_complete(&state, 1107, true, 0x0117); /* Late ACK cannot be used. */
    CHECK(state.status == SC_TEMPERATURE_I2C_ERROR && !state.valid);

    state = (sc_temperature_t){0};
    const uint32_t start = UINT32_MAX - 100U;
    CHECK(sample(&state, start, 0xff80) == 0);
    CHECK(state.raw == -128 && sc_temperature_fresh(&state, start + 205U));
    CHECK(sample(&state, start + 1000U, 3200) == 0 && state.sequence == 2);

    char buffer[32];
    sc_text_t text;
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_temperature(&text, -1);
    CHECK(text.ok && strcmp(buffer, "-0.0078125") == 0);
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_temperature(&text, 6400);
    CHECK(text.ok && strcmp(buffer, "50.0000000") == 0);
    sc_text_init(&text, buffer, 3);
    sc_text_temperature(&text, 3200);
    CHECK(!text.ok && buffer[2] == '\0');
    return 0;
}
