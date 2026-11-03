/* SPDX-License-Identifier: MIT */
/* Bounded M6 experiment. No servo, no persistent receiver/oscillator settings.
 * SiTime SiT58xx/SiT71xx digital control documentation: read gap >=50ms; 0C,0D,0E
 * write order, with 0E committing the change. Bus reads stay in clock_probe. */
#include "actuator_probe.h"
#include "main.h"
#include "simpleclock/fcw_trial.h"
#include "simpleclock/text.h"
#include "thermal_probe.h"
#include "timing_probe.h"
#include <string.h>

static sc_fcw_trial_t trial;
/* USB IRQ owns requests; foreground consumes under PRIMASK. Disconnect only
 * cancels an unexecuted start: an active experiment must still return to u0. */
static volatile uint32_t request, requested_ms;
static uint32_t denied, tx_drops, segment, write_ms[4], write_mask;
static uint16_t commanded[3];
static uint32_t covariance_ms[2];
static uint32_t covariance_tow[2];
static uint16_t covariance_week[2];
static bool covariance_valid[2];

/* One immutable snapshot survives ordinary CDC BUSY without heap allocation.
 * A new cycle replacing unsent evidence is a real drop, unlike a retry. */
typedef struct
{
    sc_fcw_trial_t state;
    uint32_t ms, read_ms[3], write_ms[4], write_mask, denied, drops;
    uint32_t week, tow_ms, pvt_ms;
    uint16_t commanded[3], readback[3];
    bool read_valid;
} trial_record_t;

static trial_record_t record;
static bool report_pending;
static uint32_t report_try_ms;
static void report(void);

bool actuator_probe_command(const char *line)
{
    const uint32_t code =
        strcmp(line, "clock trial") == 0 ? 1U : (strcmp(line, "clock abort") == 0 ? 2U : 0U);
    if (!code)
    {
        return false;
    }
    /* A pending abort cannot be replaced by a start. Duplicate starts never
     * restart the trial or extend its plateaus. */
    if (request != 2U)
    {
        requested_ms = HAL_GetTick();
        request = code;
    }
    return true;
}

void actuator_probe_disconnect(void)
{
    if (request == 1U)
    {
        request = 0U;
    }
}

void actuator_probe_observe(const sc_sbf_observation_t *o, uint32_t now_ms)
{
    if (o->id == SC_SBF_POS_COV_CARTESIAN || o->id == SC_SBF_VEL_COV_CARTESIAN)
    {
        const unsigned i = o->id == SC_SBF_POS_COV_CARTESIAN ? 0U : 1U;
        const uint16_t required =
            (uint16_t)(SC_SBF_VALID_EPOCH | (i ? SC_SBF_VALID_COV_DTDT : SC_SBF_VALID_COV_BB));
        covariance_valid[i] = (o->valid & required) == required &&
                              (o->data.cov.mode & 15U) == 10U && o->data.cov.error == 0U;
        covariance_ms[i] = now_ms;
        covariance_tow[i] = o->tow_ms;
        covariance_week[i] = o->week;
    }
}

static bool quality_ok(uint32_t now_ms, bool starting)
{
    const sc_epoch_t *epoch = timing_probe_latest();
    if (!epoch)
    {
        return false;
    }
    for (unsigned i = 0; i < 2U; i++)
    {
        const int64_t delta_ms = ((int64_t)covariance_week[i] - epoch->week) * SC_SBF_WEEK_MS +
                                 (int64_t)covariance_tow[i] - epoch->tow_ms;
        /* A just-arrived covariance can precede publication of its PVT by
         * the 250 ms assembly window. Accept that next epoch, never a replay
         * of an older epoch merely because its UART arrival was recent. */
        if (delta_ms != 0 && delta_ms != 1000)
        {
            return false;
        }
    }
    return epoch && timing_probe_pps_ready(now_ms) && timing_probe_ppp_active(now_ms) &&
           sc_pvt_has_corrections(&epoch->pvt) && (starting || epoch->segment == segment) &&
           thermal_probe_trial_ready(now_ms, starting) && covariance_valid[0] &&
           covariance_valid[1] && (uint32_t)(now_ms - covariance_ms[0]) <= 1500U &&
           (uint32_t)(now_ms - covariance_ms[1]) <= 1500U;
}

void actuator_probe_service(uint32_t now_ms)
{
    if (sc_fcw_trial_active(&trial) && !quality_ok(now_ms, false))
    {
        sc_fcw_trial_abort(&trial, now_ms);
    }
    if (report_pending && report_try_ms != now_ms)
    {
        report_try_ms = now_ms;
        report();
    }
}

static void number(sc_text_t *t, const char *key, uint32_t value)
{
    sc_text_string(t, key);
    sc_text_u32(t, value);
}

static void words(sc_text_t *t, const char *key, const uint16_t value[3])
{
    sc_text_string(t, key);
    for (unsigned i = 0; i < 3U; i++)
    {
        sc_text_hex(t, value[i], 4);
    }
}

static void report(void)
{
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t t;
    sc_text_init(&t, line, sizeof(line));
    sc_text_string(&t, "type=fcw_trial v=1 science_timing=0");
    number(&t, " uptime_ms=", record.ms);
    number(&t, " phase=", (uint32_t)record.state.phase);
    number(&t, " phase_ms=", record.state.phase_ms);
    number(&t, " writes=", record.state.writes);
    number(&t, " pending=", record.state.pending);
    number(&t, " aborted=", record.state.aborted);
    number(&t, " denied=", record.denied);
    number(&t, " tx_drops=", record.drops);
    uint16_t original[3] = {0};
    (void)sit5801_fcw_encode(record.state.original, original);
    words(&t, " original=", original);
    words(&t, " commanded=", record.commanded);
    words(&t, " readback=", record.readback);
    number(&t, " read_valid=", record.read_valid);
    number(&t, " write_ok_mask=", record.write_mask);
    number(&t, " write_start_ms=", record.write_ms[0]);
    number(&t, " write_0c_ms=", record.write_ms[1]);
    number(&t, " write_0d_ms=", record.write_ms[2]);
    number(&t, " write_0e_ms=", record.write_ms[3]);
    number(&t, " read_0c_ms=", record.read_ms[0]);
    number(&t, " read_0d_ms=", record.read_ms[1]);
    number(&t, " read_0e_ms=", record.read_ms[2]);
    number(&t, " nearby_week=", record.week);
    number(&t, " nearby_tow_ms=", record.tow_ms);
    number(&t, " nearby_pvt_ms=", record.pvt_ms);
    sc_text_string(&t, "\r\n");
    if (t.ok && simpleclock_usb_cdc_try_send_queued((const uint8_t *)line, (uint16_t)t.length))
    {
        report_pending = false;
    }
}

void actuator_probe_cycle(const sit5801_probe_t *probe, const uint32_t read_ms[3], uint32_t now_ms)
{
    if (report_pending)
    {
        tx_drops++;
        sc_fcw_trial_abort(&trial, now_ms);
    }
    const bool valid = sit5801_probe_word_order(probe) == 1U && probe->wire[2] == 0U &&
                       (probe->wire[5] & 0x01ffU) == 0U;
    const int64_t current = sit5801_fcw_decode(probe->wire + 3);
    sc_fcw_trial_observed(&trial, now_ms, valid, current);
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t command = request, age = now_ms - requested_ms;
    request = 0;
    __set_PRIMASK(mask);
    if (command == 2U)
    {
        sc_fcw_trial_abort(&trial, now_ms);
    }
    else if (command == 1U)
    {
        if (age > 1500U || !sc_fcw_trial_start(&trial, current, probe->wire[1], now_ms,
                                               valid && quality_ok(now_ms, true)))
        {
            denied++;
        }
        else
        {
            segment = timing_probe_latest()->segment;
        }
    }
    int64_t next;
    if (sc_fcw_trial_next(&trial, now_ms, quality_ok(now_ms, false), &next))
    {
        (void)sit5801_fcw_encode(next, commanded);
        write_ms[0] = HAL_GetTick();
        write_mask = 0;
        write_ms[1] = write_ms[2] = write_ms[3] = 0;
        for (unsigned i = 0; i < 3U; i++)
        {
            /* Existing board reads establish MSB-first register words. The
             * first small write is checked independently before another step.
             * No auto-increment or undocumented reserved-bit write is used. */
            uint8_t bytes[2] = {(uint8_t)(commanded[i] >> 8), (uint8_t)commanded[i]};
            const HAL_StatusTypeDef result =
                (I2C2->ISR & I2C_ISR_BUSY)
                    ? HAL_ERROR
                    : HAL_I2C_Mem_Write(&hi2c2, SIT5801_ADDRESS_7BIT << 1U, (uint16_t)(0x0cU + i),
                                        I2C_MEMADD_SIZE_8BIT, bytes, 2, 5);
            write_ms[i + 1U] = HAL_GetTick();
            if (result != HAL_OK)
            {
                break;
            }
            write_mask |= 1U << i;
        }
        sc_fcw_trial_written(&trial, HAL_GetTick(), write_mask == 7U);
    }
    record.state = trial;
    record.ms = HAL_GetTick();
    record.denied = denied;
    record.drops = tx_drops;
    record.write_mask = write_mask;
    record.read_valid = valid;
    memcpy(record.commanded, commanded, sizeof(commanded));
    memcpy(record.readback, probe->wire + 3, sizeof(record.readback));
    memcpy(record.read_ms, read_ms, sizeof(record.read_ms));
    memcpy(record.write_ms, write_ms, sizeof(write_ms));
    const sc_epoch_t *epoch = timing_probe_latest();
    record.week = epoch ? epoch->week : UINT16_MAX;
    record.tow_ms = epoch ? epoch->tow_ms : UINT32_MAX;
    record.pvt_ms = epoch ? epoch->source_ms[0] : UINT32_MAX;
    report_pending = true;
}

uint32_t actuator_probe_writes(void)
{
    return trial.writes;
}
