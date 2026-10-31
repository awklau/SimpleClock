/* SPDX-License-Identifier: MIT */
#include "simpleclock/fcw_trial.h"
#include "simpleclock/sit5801.h"
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

static int run_trial(uint32_t start, int64_t original, bool abort_up)
{
    sc_fcw_trial_t s = {0};
    CHECK(sc_fcw_trial_start(&s, original, 32, start, true));
    int64_t last = original, q = 0;
    uint32_t applied = start, n = 0, high_start = 0, return_start = 0;
    bool high_seen = false;
    for (uint32_t dt = 0; dt < 600000U; dt += 50U)
    {
        const uint32_t now = start + dt;
        if (abort_up && dt == 126000U)
        {
            sc_fcw_trial_abort(&s, now);
        }
        if (sc_fcw_trial_next(&s, now, true, &q))
        {
            CHECK((uint32_t)(now - applied) >= 1000U);
            CHECK(q - last <= SC_FCW_STEP_COUNTS && last - q <= SC_FCW_STEP_COUNTS);
            CHECK(q >= original && q <= original + SC_FCW_TEST_COUNTS);
            CHECK(dt >= 120000U);
            applied = now;
            last = q;
            n++;
            sc_fcw_trial_written(&s, now, true);
            CHECK(s.pending);
            CHECK(!sc_fcw_trial_next(&s, now + 1U, true, &q));
            sc_fcw_trial_observed(&s, now + 40U, true, last);
        }
        if (s.phase == SC_FCW_HIGH && !high_seen)
        {
            high_seen = true;
            high_start = dt;
        }
        if (s.phase == SC_FCW_RECOVERY && !return_start)
        {
            return_start = dt;
        }
        if (s.phase == SC_FCW_DONE || s.phase == SC_FCW_ABORTED)
        {
            CHECK(s.current == original && !s.pending);
            CHECK(dt - return_start >= 180000U);
            CHECK(abort_up ? s.phase == SC_FCW_ABORTED : s.phase == SC_FCW_DONE);
            if (!abort_up)
            {
                CHECK(n == 42U && high_seen && return_start - high_start >= 200000U);
            }
            CHECK(!sc_fcw_trial_start(&s, 0, 32, now, true));
            return 0;
        }
    }
    CHECK(false);
}

int main(void)
{
    _Static_assert(SC_FCW_STEP_COUNTS * INT64_C(16000000) <= SIT5801_FCW_MAX, "slew ceiling");
    _Static_assert((SC_FCW_STEP_COUNTS + 1) * INT64_C(16000000) > SIT5801_FCW_MAX,
                   "floor rounding");
    _Static_assert(SC_FCW_TEST_COUNTS == (SIT5801_FCW_MAX + 400000) / 800000, "one ppb rounding");
    CHECK(run_trial(1800000U, 0, false) == 0);
    CHECK(run_trial(UINT32_MAX - 50000U, -1234567, false) == 0);
    CHECK(run_trial(1800000U, 1234567, true) == 0);
    sc_fcw_trial_t s = {0};
    int64_t q;
    CHECK(!sc_fcw_trial_start(&s, 0, 32, 1799999U, true));
    CHECK(!sc_fcw_trial_start(&s, 0, 32, 1800000U, false));
    CHECK(!sc_fcw_trial_start(&s, SIT5801_FCW_MAX, 0, 1800000U, true));
    CHECK(!sc_fcw_trial_start(&s, INT64_MIN, 32, 1800000U, true));
    CHECK(!sc_fcw_trial_start(&s, INT64_MAX, 32, 1800000U, true));
    CHECK(sc_fcw_trial_start(&s, 0, 32, 1800000U, true));
    CHECK(sc_fcw_trial_next(&s, 1920000U, true, &q));
    sc_fcw_trial_written(&s, 1920001U, true);
    sc_fcw_trial_observed(&s, 1920100U, true, q + 1);
    CHECK(s.phase == SC_FCW_FAULT);
    CHECK(!sc_fcw_trial_next(&s, 1950000U, true, &q));
    s = (sc_fcw_trial_t){0};
    CHECK(sc_fcw_trial_start(&s, 0, 32, 1800000U, true));
    CHECK(sc_fcw_trial_next(&s, 1920000U, true, &q));
    sc_fcw_trial_written(&s, 1920001U, false);
    CHECK(s.phase == SC_FCW_FAULT);
    s = (sc_fcw_trial_t){0};
    CHECK(sc_fcw_trial_start(&s, 0, 32, 1800000U, true));
    CHECK(sc_fcw_trial_next(&s, 1920000U, true, &q));
    sc_fcw_trial_written(&s, 1920001U, true);
    CHECK(!sc_fcw_trial_next(&s, 1922502U, true, &q));
    CHECK(s.phase == SC_FCW_FAULT);
    s = (sc_fcw_trial_t){0};
    CHECK(sc_fcw_trial_start(&s, 0, 32, 1800000U, true));
    CHECK(!sc_fcw_trial_next(&s, 1800001U, false, &q));
    CHECK(s.aborted && s.phase == SC_FCW_RECOVERY);
    puts("bounded FCW trial tests passed");
    return 0;
}
