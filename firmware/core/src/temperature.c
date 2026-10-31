/* SPDX-License-Identifier: MIT */
#include "simpleclock/temperature.h"
#include "simpleclock/tmp117.h"

/* TI SNOSD82D sections 7.4.2/3, 7.6.3/6. Preserve 8-average selection;
 * CONV=100 is retained but does not affect one-shot duration. EEPROM stays
 * locked: these are volatile writes, never calibration/EEPROM programming. */
#define CONFIG_SHUTDOWN 0x0620U
#define CONFIG_ONE_SHOT 0x0e20U
#define REG_EEPROM_LOCK 0x04U
#define EEPROM_WRITE_BLOCKED_MASK 0xc000U /* EUN (unlocked) or EEPROM_Busy. */
#define CONFIG_READBACK_MASK 0x1fffU      /* Exclude the three read-only alert/ready flags. */

enum
{
    WAIT_CYCLE,      /* Enforce the one-second trigger cadence. */
    READ_ID,         /* Confirm the device before changing configuration. */
    READ_LOCK,       /* Verify EEPROM remains locked before volatile writes. */
    WRITE_SHUTDOWN,  /* Stop any prior conversion without programming EEPROM. */
    CLEAR_READY,     /* Read configuration and validate the shutdown mode. */
    WRITE_TRIGGER,   /* Start one-shot conversion after the ready flag is cleared. */
    WAIT_CONVERSION, /* Hold off bus access for the bounded conversion interval. */
    READ_VALUE       /* Consume Data_Ready and decode the signed temperature. */
};

static void invalidate(sc_temperature_t *state, sc_temperature_status_t reason, uint32_t now_ms)
{
    state->valid = state->id_valid = false;
    state->status = reason;
    state->errors++;
    state->phase = WAIT_CYCLE;
    state->cycle_ms = now_ms;
}

bool sc_temperature_begin(sc_temperature_t *state, uint32_t now_ms,
                          sc_temperature_transfer_t *transfer)
{
    /* pending is the bus adapter's ownership handoff: begin sets it, and only
     * complete may clear it after the adapter reports this exact transaction. */
    if (state->pending)
    {
        return false;
    }
    if (state->phase == READ_VALUE &&
        (uint32_t)(now_ms - state->ready_observed_ms) >= SC_TEMPERATURE_READ_WINDOW_MS)
    {
        invalidate(state, SC_TEMPERATURE_INVALID, now_ms);
        return false;
    }
    if (state->phase == WAIT_CYCLE)
    {
        if (state->started && (uint32_t)(now_ms - state->cycle_ms) < SC_TEMPERATURE_CYCLE_MS)
        {
            return false;
        }
        /* Skip missed cycles instead of issuing a catch-up burst. */
        state->cycle_ms = now_ms;
        state->started = true;
        state->phase = READ_ID;
    }
    if (state->phase == WAIT_CONVERSION &&
        (uint32_t)(now_ms - state->wait_ms) < SC_TEMPERATURE_CONVERSION_MS)
    {
        return false;
    }
    *transfer = (sc_temperature_transfer_t){0, SIMPLECLOCK_TMP117_REG_CONFIGURATION, false};
    switch (state->phase)
    {
    case READ_ID:
        transfer->reg = SIMPLECLOCK_TMP117_REG_DEVICE_ID;
        break;
    case READ_LOCK:
        transfer->reg = REG_EEPROM_LOCK;
        break;
    case WRITE_SHUTDOWN:
        transfer->write = true;
        transfer->value = CONFIG_SHUTDOWN;
        break;
    case WRITE_TRIGGER:
        transfer->write = true;
        transfer->value = CONFIG_ONE_SHOT;
        break;
    case READ_VALUE:
        transfer->reg = SIMPLECLOCK_TMP117_REG_TEMPERATURE;
        break;
    default:
        break;
    }
    state->transfer_ms = now_ms;
    state->pending = true;
    return true;
}

void sc_temperature_complete(sc_temperature_t *state, uint32_t now_ms, bool success, uint16_t value)
{
    /* transfer_ms is when begin published the request. A late completion is
     * rejected even if the bus adapter reports success, keeping age semantics
     * independent of how long the foreground transport was stalled. */
    if (!state->pending)
    {
        return;
    }
    state->pending = false;
    if (!success || (uint32_t)(now_ms - state->transfer_ms) >= SC_TEMPERATURE_READ_WINDOW_MS)
    {
        invalidate(state, SC_TEMPERATURE_I2C_ERROR, now_ms);
        return;
    }
    switch (state->phase)
    {
    case READ_ID:
        state->device_id = value;
        state->id_valid = simpleclock_tmp117_device_id_valid(value);
        if (!state->id_valid)
        {
            invalidate(state, SC_TEMPERATURE_BAD_ID, now_ms);
            return;
        }
        state->phase = READ_LOCK;
        break;
    case READ_LOCK:
        if (value & EEPROM_WRITE_BLOCKED_MASK)
        {
            invalidate(state, SC_TEMPERATURE_BAD_CONFIG, now_ms);
            return;
        }
        state->phase = WRITE_SHUTDOWN;
        break;
    case WRITE_SHUTDOWN:
        state->phase = CLEAR_READY;
        break;
    case CLEAR_READY:
    case WAIT_CONVERSION:
        state->configuration = value;
        /* MOD is the programmed mode, not a conversion-busy flag. The board
         * reads 0x2e20 after completion: OS bits remain 11 even while the ADC
         * is shut down. Require Data_Ready below to prove completion. */
        if ((value & CONFIG_READBACK_MASK) !=
            (state->phase == CLEAR_READY ? CONFIG_SHUTDOWN : CONFIG_ONE_SHOT))
        {
            invalidate(state, SC_TEMPERATURE_BAD_CONFIG, now_ms);
            return;
        }
        if (state->phase == CLEAR_READY)
        {
            /* Abort any prior conversion before clearing its ready flag. This
             * makes even an ACKed-but-ignored trigger unable to reuse old data. */
            state->phase = WRITE_TRIGGER;
        }
        else if (!simpleclock_tmp117_data_ready(value) ||
                 (uint32_t)(now_ms - state->shot_ms) >= SC_TEMPERATURE_STALE_MS)
        {
            invalidate(state, SC_TEMPERATURE_INVALID, now_ms);
        }
        else
        {
            state->ready_observed_ms = state->transfer_ms;
            state->phase = READ_VALUE;
        }
        break;
    case WRITE_TRIGGER:
        state->shot_ms = state->transfer_ms;
        state->wait_ms = now_ms;
        /* Eight conversions take at most 8*17.5=140 ms per TI's conversion
         * timing. Wait 200 ms from ACK before touching clear-on-read flags.
         * This avoids the known Data_Ready race during continuous polling. */
        state->phase = WAIT_CONVERSION;
        break;
    case READ_VALUE:
    {
        const int16_t raw =
            simpleclock_tmp117_decode_temperature_raw((uint8_t)(value >> 8), (uint8_t)value);
        if ((uint32_t)(now_ms - state->ready_observed_ms) >= SC_TEMPERATURE_READ_WINDOW_MS ||
            (uint32_t)(now_ms - state->shot_ms) >= SC_TEMPERATURE_STALE_MS || raw < -55 * 128 ||
            raw > 150 * 128)
        {
            invalidate(state, SC_TEMPERATURE_INVALID, now_ms);
            return;
        }
        state->raw = raw;
        state->sample_ms = state->shot_ms;
        state->sequence++;
        state->valid = true;
        state->status = SC_TEMPERATURE_OK;
        state->phase = WAIT_CYCLE;
        break;
    }
    default:
        invalidate(state, SC_TEMPERATURE_INVALID, now_ms);
        break;
    }
}

bool sc_temperature_fresh(const sc_temperature_t *state, uint32_t now_ms)
{
    return state->valid && (uint32_t)(now_ms - state->sample_ms) < SC_TEMPERATURE_STALE_MS;
}
