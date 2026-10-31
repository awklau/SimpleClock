/* SPDX-License-Identifier: MIT */
#include "simpleclock/preheat.h"
#include <stdio.h>
#define CHECK(e)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(e))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #e);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

int main(void)
{
    sc_preheat_t state;
    sc_preheat_init(&state, 0);
    for (uint32_t second = 0; second < 300; second++)
    {
        sc_preheat_push(&state, second * 1000, 10000 + second % 3, true, true, true);
        CHECK(!state.complete_inferred);
    }
    sc_preheat_push(&state, 300000, 10000, true, true, true);
    CHECK(state.plateau && state.complete_inferred && state.count == 120);
    sc_preheat_push(&state, 301000, 20000, true, true, true);
    CHECK(state.complete_inferred); /* Deliberate external heating may move oven drive. */
    sc_preheat_expire(&state, 302501, true);
    CHECK(!state.complete_inferred && state.count == 0);

    sc_preheat_init(&state, 0);
    for (uint32_t second = 0; second <= 300; second++)
    {
        sc_preheat_push(&state, second * 1000, 10000, true, true, false);
    }
    CHECK(state.plateau && !state.complete_inferred); /* A flat unknown code proves nothing. */
    sc_preheat_push(&state, 301000, 0, true, true, true);
    CHECK(state.count == 0 && !state.complete_inferred);
    sc_preheat_push(&state, 302000, 262143, true, true, true);
    CHECK(state.count == 0);

    sc_preheat_init(&state, 0);
    for (uint32_t second = 0; second <= 300; second++)
    {
        sc_preheat_push(&state, second * 1000, second % 2 ? 10100 : 9900, true, true, true);
    }
    CHECK(state.plateau && state.median == 10000); /* Exactly 2%, inclusive. */
    for (uint32_t second = 301; second <= 450; second++)
    {
        sc_preheat_push(&state, second * 1000, second % 2 ? 10200 : 9800, true, true, true);
    }
    CHECK(!state.plateau && state.complete_inferred);
    sc_preheat_push(&state, 451000, 10000, false, true, true);
    CHECK(!state.complete_inferred && !state.count);

    sc_preheat_init(&state, 0);
    for (uint32_t i = 0; i < 1000; i++)
    {
        sc_preheat_push(&state, 300000, 10000, true, true, true);
    }
    CHECK(state.count == 1 && !state.complete_inferred);
    sc_preheat_expire(&state, 1800000, true);
    CHECK(state.unconfirmed);
    for (uint32_t second = 1800; second <= 1920; second++)
    {
        sc_preheat_push(&state, second * 1000, 10000, true, true, true);
    }
    CHECK(!state.complete_inferred); /* Timeout requires inspection/restart, no silent promotion. */

    const uint32_t base = UINT32_MAX - 100000U;
    sc_preheat_init(&state, base);
    for (uint32_t second = 0; second <= 300; second++)
    {
        sc_preheat_push(&state, base + second * 1000, 10000, true, true, true);
    }
    CHECK(state.complete_inferred);
    sc_preheat_expire(&state, base + 300001, false);
    CHECK(!state.complete_inferred);
    return 0;
}
