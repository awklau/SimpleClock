/* SPDX-License-Identifier: MIT */
#include "simpleclock/fcw_trial.h"
#include "simpleclock/sit5801.h"

bool sc_fcw_trial_active(const sc_fcw_trial_t *s)
{
    return s->phase >= SC_FCW_BASELINE && s->phase <= SC_FCW_RECOVERY;
}

bool sc_fcw_trial_start(sc_fcw_trial_t *s, int64_t original, uint16_t clip, uint32_t now_ms,
                        bool qualified)
{
    /* Integer clip bound uses the same positive scale as the vendor FCW
     * formula. It is conservative by <1 count on the negative endpoint. */
    const uint16_t fraction = clip & 0x1fffU;
    const int64_t limit = fraction ? SIT5801_FCW_MAX * fraction / 8192 : SIT5801_FCW_MAX;
    if (s->phase != SC_FCW_IDLE || !qualified || now_ms < SC_FCW_SETTLE_AFTER_BOOT_MS ||
        original < -limit || original > limit - SC_FCW_TEST_COUNTS)
    {
        return false;
    }
    *s = (sc_fcw_trial_t){0};
    s->original = s->current = s->requested = original;
    s->phase = SC_FCW_BASELINE;
    s->phase_ms = s->applied_ms = now_ms;
    return true;
}

void sc_fcw_trial_abort(sc_fcw_trial_t *s, uint32_t now_ms)
{
    if (sc_fcw_trial_active(s) && !s->aborted)
    {
        s->aborted = true;
        s->phase = SC_FCW_DOWN;
        s->phase_ms = now_ms;
    }
}

bool sc_fcw_trial_next(sc_fcw_trial_t *s, uint32_t now_ms, bool quality, int64_t *word)
{
    if (!sc_fcw_trial_active(s))
    {
        return false;
    }
    if (!quality)
    {
        sc_fcw_trial_abort(s, now_ms);
    }
    if (s->pending)
    {
        if ((uint32_t)(now_ms - s->applied_ms) > 2500U)
        {
            s->phase = SC_FCW_FAULT;
        }
        return false;
    }
    const uint32_t elapsed = now_ms - s->phase_ms;
    if ((s->phase == SC_FCW_BASELINE && elapsed >= 120000U) ||
        (s->phase == SC_FCW_HIGH && elapsed >= 180000U))
    {
        s->phase = s->phase == SC_FCW_BASELINE ? SC_FCW_UP : SC_FCW_DOWN;
        s->phase_ms = now_ms;
    }
    if (s->phase == SC_FCW_RECOVERY && elapsed >= 180000U)
    {
        s->phase = s->aborted ? SC_FCW_ABORTED : SC_FCW_DONE;
    }
    if (s->phase != SC_FCW_UP && s->phase != SC_FCW_DOWN)
    {
        return false;
    }
    const int64_t goal = s->original + (s->phase == SC_FCW_UP ? SC_FCW_TEST_COUNTS : 0);
    if (s->current == goal)
    {
        s->phase = s->phase == SC_FCW_UP ? SC_FCW_HIGH : SC_FCW_RECOVERY;
        s->phase_ms = now_ms;
        return false;
    }
    if ((uint32_t)(now_ms - s->applied_ms) < 1000U)
    {
        return false;
    }
    const int64_t delta = goal - s->current;
    const int64_t step = delta > SC_FCW_STEP_COUNTS
                             ? SC_FCW_STEP_COUNTS
                             : (delta < -SC_FCW_STEP_COUNTS ? -SC_FCW_STEP_COUNTS : delta);
    *word = s->requested = s->current + step;
    s->pending = true;
    s->applied_ms = now_ms;
    return true;
}

void sc_fcw_trial_written(sc_fcw_trial_t *s, uint32_t now_ms, bool success)
{
    if (!s->pending || !success)
    {
        s->phase = SC_FCW_FAULT;
        return;
    }
    s->applied_ms = now_ms; /* Slew spacing starts at last LSW completion. */
    s->writes++;
}

void sc_fcw_trial_observed(sc_fcw_trial_t *s, uint32_t now_ms, bool valid, int64_t word)
{
    (void)now_ms;
    if (!sc_fcw_trial_active(s))
    {
        return;
    }
    if (!valid || word != (s->pending ? s->requested : s->current))
    {
        s->phase = SC_FCW_FAULT;
        return;
    }
    s->current = word;
    s->pending = false;
}
