/* SPDX-License-Identifier: MIT */
/* Assemble receiver epochs and qualify passive PPS presence.
 * Receiver week/TOW, MCU milliseconds and TIM2 capture ticks remain separate clocks. */
#include "simpleclock/timing.h"
#include <string.h>

#define EPOCH_SETTLE_MS 250U           /* Out-of-order assembly wait, milliseconds. */
#define SOURCE_FRESHNESS_MS 1500U      /* Source/observation age limit, milliseconds. */
#define EPOCH_PERIOD_MS 1000U          /* Expected receiver epoch period, milliseconds. */
#define PPS_MIN_INTERVAL_US 950000U    /* Presence-test lower bound, microseconds. */
#define PPS_MAX_INTERVAL_US 1050000U   /* Presence-test upper bound, microseconds. */
#define PPS_GOOD_INTERVALS_REQUIRED 3U /* Consecutive intervals; requires four edges. */

static uint64_t receiver_epoch_ms(uint16_t week, uint32_t tow)
{
    /* This comparison clock is receiver time only. It is deliberately kept
     * separate from MCU now_ms and from the timer capture tick domain. */
    return (uint64_t)week * (uint64_t)SC_SBF_WEEK_MS + tow;
}

static bool epoch_sources_usable(const sc_epoch_t *epoch);

static bool pvt_integrity_ok(const sc_sbf_observation_t *pvt)
{
    const uint8_t alert = pvt->data.pvt.alert;
    return (alert & 3U) < 2U && !(alert & 0x0cU) && (pvt->revision < 3U || !(alert & 0x60U));
}

void sc_epochs_break(sc_epochs_t *state, uint32_t reason)
{
    for (unsigned i = 0; i < SC_EPOCH_SLOT_COUNT; i++)
    {
        if (state->slots[i].mask)
        {
            state->discarded++;
        }
        memset(&state->slots[i], 0, sizeof(state->slots[i]));
    }
    state->segment++;
    state->pending_reasons |= reason;
    state->have_last = false;
}

bool sc_pvt_has_corrections(const sc_sbf_observation_t *pvt)
{
    const uint16_t required =
        SC_SBF_VALID_EPOCH | SC_SBF_VALID_FIX | SC_SBF_VALID_TIME_SYSTEM | SC_SBF_VALID_CORR_AGE;
    return (pvt->valid & required) == required && pvt->data.pvt.error == 0U &&
           pvt_integrity_ok(pvt) &&
           (pvt->data.pvt.mode & SC_SBF_PVT_MODE_MASK) == SC_SBF_PVT_MODE_PPP &&
           pvt->data.pvt.time_system == SC_SBF_TIME_SYSTEM_GALILEO &&
           (pvt->data.pvt.wa_corr_info & SC_SBF_PVT_WA_ORBIT_CLOCK_USED) != 0U &&
           pvt->data.pvt.mean_corr_age < SC_PPP_MAX_CORR_AGE_CS;
}

void sc_epochs_push(sc_epochs_t *state, const sc_sbf_observation_t *observation, uint32_t now_ms)
{
    unsigned index;
    switch (observation->id)
    {
    case SC_SBF_PVT:
        index = 0;
        break;
    case SC_SBF_STATUS:
        index = 1;
        break;
    case SC_SBF_TIME:
        index = 2;
        break;
    case SC_SBF_MEAS:
        index = 3;
        break;
    case SC_SBF_END:
        index = 4;
        break;
    default:
        return; /* E6 navigation pages have SIS epochs, not receiver epochs. */
    }
    if (!(observation->valid & SC_SBF_VALID_EPOCH))
    {
        sc_epochs_break(state, SC_SEG_QUALITY);
        return;
    }
    if (index == 1U)
    {
        if (state->have_status && observation->data.status.uptime_s < state->status_uptime)
        {
            sc_epochs_break(state, SC_SEG_RESET);
            state->receiver_boot++;
        }
        state->have_status = true;
        state->status_uptime = observation->data.status.uptime_s;
    }
    const uint64_t source = receiver_epoch_ms(observation->week, observation->tow_ms);
    if (state->have_last && source <= receiver_epoch_ms(state->last.week, state->last.tow_ms))
    {
        state->late++;
        return; /* Published epochs are immutable, including late EndOfPVT. */
    }
    sc_epoch_t *slot = NULL, *free_slot = NULL;
    for (unsigned i = 0; i < SC_EPOCH_SLOT_COUNT; i++)
    {
        if (!state->slots[i].mask)
        {
            free_slot = &state->slots[i];
        }
        else if (state->slots[i].tow_ms == observation->tow_ms &&
                 state->slots[i].week == observation->week)
        {
            slot = &state->slots[i];
        }
    }
    if (slot == NULL)
    {
        if (free_slot == NULL)
        {
            sc_epochs_break(state, SC_SEG_LOSS);
            free_slot = &state->slots[0];
        }
        slot = free_slot;
        memset(slot, 0, sizeof(*slot));
        slot->tow_ms = observation->tow_ms;
        slot->week = observation->week;
        slot->first_ms = now_ms;
    }
    const uint8_t bit = (uint8_t)(1U << index);
    if (slot->mask & bit)
    {
        slot->duplicate |= bit;
        state->duplicates++;
        return;
    }
    slot->mask |= bit;
    slot->source_ms[index] = now_ms;
    switch (index)
    {
    case 0:
        slot->pvt = *observation;
        break;
    case 1:
        slot->rx_uptime = observation->data.status.uptime_s;
        slot->rx_state = observation->data.status.rx_state;
        slot->rx_error = observation->data.status.rx_error;
        slot->ext_error = observation->data.status.ext_error;
        break;
    case 2:
        slot->sync_level = observation->data.time.sync_level;
        break;
    case 3:
        slot->common_flags = observation->data.meas.common_flags;
        slot->jumps = observation->data.meas.cum_jumps;
        slot->jumps_valid = (observation->valid & SC_SBF_VALID_JUMPS) != 0U;
        break;
    default:
        break;
    }
}

/* This is a prerequisite for the optional RxClock PPS setter, not proof of
 * UTC calibration, PPP convergence, clock-jump unwrapping or PPS association. */
static bool epoch_sources_usable(const sc_epoch_t *epoch)
{
    const uint16_t required = SC_SBF_VALID_EPOCH | SC_SBF_VALID_FIX | SC_SBF_VALID_BIAS |
                              SC_SBF_VALID_DRIFT | SC_SBF_VALID_TIME_SYSTEM;
    return epoch->mask == SC_EPOCH_ALL && !epoch->duplicate &&
           (epoch->pvt.valid & required) == required && pvt_integrity_ok(&epoch->pvt) &&
           (epoch->sync_level & SC_SBF_TIME_SYNC_READY_MASK) == SC_SBF_TIME_SYNC_READY_MASK &&
           (epoch->rx_state & SC_SBF_RX_STATE_EXT_FREQ) != 0U && !epoch->rx_error &&
           !epoch->ext_error && epoch->jumps_valid &&
           !(epoch->common_flags & SC_SBF_MEAS_CLOCK_STEERING);
}

static const sc_epoch_t *newest_pvt(const sc_epochs_t *state)
{
    const sc_epoch_t *latest =
        state->have_last && (state->last.mask & SC_EPOCH_PVT) ? &state->last : NULL;
    /* A newly received invalid PVT must replace the indication immediately,
     * before the normal 250 ms epoch assembly delay. Late/duplicate frames
     * already retain their original timestamp in the assembly logic. */
    for (unsigned i = 0; i < SC_EPOCH_SLOT_COUNT; i++)
    {
        const sc_epoch_t *slot = &state->slots[i];
        if ((slot->mask & SC_EPOCH_PVT) &&
            (latest == NULL || receiver_epoch_ms(slot->week, slot->tow_ms) >
                                   receiver_epoch_ms(latest->week, latest->tow_ms)))
        {
            latest = slot;
        }
        if ((slot->mask & SC_EPOCH_STATUS) && (slot->rx_error || slot->ext_error))
        {
            return NULL;
        }
    }
    return latest;
}

bool sc_epochs_fix_valid(const sc_epochs_t *state, uint32_t now_ms)
{
    const sc_epoch_t *latest = newest_pvt(state);
    const uint16_t required = SC_SBF_VALID_EPOCH | SC_SBF_VALID_FIX;
    return latest != NULL && !(latest->duplicate & SC_EPOCH_PVT) &&
           (uint32_t)(now_ms - latest->source_ms[0]) <= SOURCE_FRESHNESS_MS &&
           (latest->pvt.valid & required) == required && latest->pvt.data.pvt.error == 0U &&
           pvt_integrity_ok(&latest->pvt) &&
           (!(latest->mask & SC_EPOCH_STATUS) || (!latest->rx_error && !latest->ext_error));
}

bool sc_epochs_ppp_active(const sc_epochs_t *state, uint32_t now_ms)
{
    const sc_epoch_t *latest = newest_pvt(state);
    return latest != NULL && sc_epochs_fix_valid(state, now_ms) &&
           (latest->pvt.data.pvt.mode & SC_SBF_PVT_MODE_MASK) == SC_SBF_PVT_MODE_PPP;
}

bool sc_epochs_poll(sc_epochs_t *state, uint32_t now_ms, sc_epoch_t *out)
{
    sc_epoch_t *oldest = NULL;
    for (unsigned i = 0; i < SC_EPOCH_SLOT_COUNT; i++)
    {
        sc_epoch_t *epoch = &state->slots[i];
        if (epoch->mask && (oldest == NULL || receiver_epoch_ms(epoch->week, epoch->tow_ms) <
                                                  receiver_epoch_ms(oldest->week, oldest->tow_ms)))
        {
            oldest = epoch;
        }
    }
    /* Allow 250 ms for out-of-order members even after EndOfPVT. Publish the
     * oldest slot first; an incomplete snapshot remains explicit in its mask. */
    if (oldest == NULL || (uint32_t)(now_ms - oldest->first_ms) < EPOCH_SETTLE_MS)
    {
        return false;
    }
    *out = *oldest;
    memset(oldest, 0, sizeof(*oldest));
    uint32_t reason = state->pending_reasons;
    state->pending_reasons = 0;
    out->sources_valid =
        epoch_sources_usable(out) && (uint32_t)(now_ms - out->first_ms) <= SOURCE_FRESHNESS_MS;
    if (out->mask != SC_EPOCH_ALL)
    {
        reason |= SC_SEG_INCOMPLETE;
    }
    if (out->duplicate)
    {
        reason |= SC_SEG_DUPLICATE;
    }
    if (state->have_last)
    {
        const sc_epoch_t *previous = &state->last;
        if (receiver_epoch_ms(out->week, out->tow_ms) !=
                receiver_epoch_ms(previous->week, previous->tow_ms) + EPOCH_PERIOD_MS ||
            (uint32_t)(out->first_ms - previous->first_ms) > SOURCE_FRESHNESS_MS)
        {
            reason |= SC_SEG_GAP;
        }
        if ((previous->mask & SC_EPOCH_PVT) && (out->mask & SC_EPOCH_PVT) &&
            previous->pvt.data.pvt.time_system != out->pvt.data.pvt.time_system)
        {
            reason |= SC_SEG_SYSTEM;
        }
        if (previous->jumps_valid && out->jumps_valid && previous->jumps != out->jumps)
        {
            reason |= SC_SEG_JUMP;
        }
        if (((previous->rx_state ^ out->rx_state) & SC_SBF_RX_STATE_EXT_FREQ) != 0U)
        {
            reason |= SC_SEG_REFERENCE;
        }
        if (previous->sources_valid != out->sources_valid)
        {
            reason |= SC_SEG_QUALITY;
        }
        if (reason)
        {
            state->segment++;
        }
    }
    else if (!reason)
    {
        reason = SC_SEG_START;
    }
    out->segment = state->segment;
    out->reasons = reason;
    out->sequence = state->sequence++;
    state->last = *out;
    state->have_last = true;
    return true;
}

bool sc_epochs_pps_ready(const sc_epochs_t *state, uint32_t now_ms)
{
    if (!state->have_last || !state->last.sources_valid ||
        (uint32_t)(now_ms - state->last.first_ms) > SOURCE_FRESHNESS_MS)
    {
        return false;
    }
    /* New invalid fields veto immediately, including while an older slot is
     * being published. Do not let a past valid snapshot mask a new outage. */
    for (unsigned i = 0; i < SC_EPOCH_SLOT_COUNT; i++)
    {
        const sc_epoch_t *epoch = &state->slots[i];
        sc_epoch_t check = state->last;
        if (epoch->duplicate)
        {
            return false;
        }
        if (epoch->mask & SC_EPOCH_PVT)
        {
            check.pvt = epoch->pvt;
        }
        if (epoch->mask & SC_EPOCH_STATUS)
        {
            check.rx_state = epoch->rx_state;
            check.rx_error = epoch->rx_error;
            check.ext_error = epoch->ext_error;
        }
        if (epoch->mask & SC_EPOCH_TIME)
        {
            check.sync_level = epoch->sync_level;
        }
        if (epoch->mask & SC_EPOCH_MEAS)
        {
            check.common_flags = epoch->common_flags;
            check.jumps_valid = epoch->jumps_valid;
        }
        if (!epoch_sources_usable(&check))
        {
            return false;
        }
    }
    return true;
}

void sc_pps_edge(sc_pps_t *state, uint32_t tick, uint32_t ms, uint32_t sequence, bool loss)
{
    /* Unsigned subtraction handles one 32-bit timer wrap. The separate MCU-ms
     * age gate prevents long gaps from masquerading as a valid wrapped period.
     * +/-5% is only a presence test; three good intervals require four edges. */
    /* Unsigned subtraction is correct for one 32-bit capture wrap; sequence and
     * MCU-ms age checks reject a long gap that happens to have a valid period. */
    const uint32_t interval = tick - state->tick;
    state->interval_valid = state->seen && !loss && sequence - state->sequence == 1U &&
                            (uint32_t)(ms - state->ms) <= SOURCE_FRESHNESS_MS &&
                            interval >= PPS_MIN_INTERVAL_US && interval <= PPS_MAX_INTERVAL_US;
    state->good_intervals =
        state->interval_valid
            ? (state->good_intervals < PPS_GOOD_INTERVALS_REQUIRED ? state->good_intervals + 1U
                                                                   : PPS_GOOD_INTERVALS_REQUIRED)
            : 0U;
    state->interval_us = interval;
    state->tick = tick;
    state->ms = ms;
    state->sequence = sequence;
    state->seen = true;
    state->state =
        state->good_intervals >= PPS_GOOD_INTERVALS_REQUIRED ? SC_PPS_PRESENT : SC_PPS_ACQUIRING;
}

void sc_pps_expire(sc_pps_t *state, uint32_t now_ms)
{
    if (state->seen && (uint32_t)(now_ms - state->ms) > SOURCE_FRESHNESS_MS)
    {
        state->state = SC_PPS_LOST;
        state->good_intervals = 0;
        state->interval_valid = false;
    }
}
