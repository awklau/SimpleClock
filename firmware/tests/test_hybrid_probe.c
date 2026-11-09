/* SPDX-License-Identifier: MIT */
/* Host exercise of the AUTO_HYBRID_V1 platform adapter with a fake HAL. */
#include "main.h"
#include "telemetry_check.h"
#include "hybrid_probe.h"
#include "gnss_probe.h"
#include "simpleclock/sbf_fields.h"
#include "simpleclock/sit5801.h"
#include "simpleclock/timing.h"
#include "simpleclock/thermal_outer.h"
#include "simpleclock/version.h"
#include "thermal_probe.h"
#include "settings_flash.h"
#include <math.h>
#include <stdint.h>
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

I2C_HandleTypeDef hi2c2;
I2C_TypeDef test_i2c2;
TIM_TypeDef test_tim2;

static uint32_t now_ms;
static bool usb_busy;
static bool usb_window_mode;
static uint32_t usb_window_ms;
static bool usb_window_used;
static uint32_t report_type_mask;
static bool report_bad;
static bool bus_busy;
static bool auto_ready = true;
static bool receiver_config_fault;
static unsigned recheck_requests;
static bool standalone_mode;
static bool static_pvt;
static bool thermal_safe = true;
static bool thermal_sensor_valid = true;
static uint32_t thermal_faults;
static uint32_t thermal_target_q16;
static uint16_t thermal_duty_permille = 500U;
static int16_t thermal_raw_offset;
static double boot_target_C = 39.399;
static sc_epoch_t latest_epoch;
static bool epoch_available;
static uint32_t epoch_tow_ms;
static uint32_t epoch_sequence;
static uint16_t oscillator_words[3];
static unsigned write_count;
static unsigned report_count;
static unsigned thermal_fault_count;
static uint32_t skipped_frames;
static bool snapshot_interrupt;
static bool reported_duty_window_valid;
static bool command_seen, command_accepted, loop_reported, profile_reported, phase_wait_reported;
static unsigned command_reports;
static bool command_busy_reported;
static bool reset_called, reboot_requested, reboot_sent, reboot_failed, gnss_idle = true;
static bool reboot_failure_reported;
static bool gnss_managed;
static uint32_t gnss_selected = SC_SETTINGS_GNSS_SBAS;
static uint32_t pps_policy = SC_SETTINGS_PPS_CONTINUOUS, pps_quality_updates;
static bool pps_quality;
static int fail_register;

typedef struct
{
    uint16_t reg;
    uint16_t value;
    uint32_t ms;
} write_event_t;

static write_event_t write_events[4096];

uint32_t HAL_GetTick(void)
{
    return now_ms;
}

bool simpleclock_usb_cdc_tx_idle(void)
{
    return !usb_busy;
}

void simpleclock_system_reset(void)
{
    reset_called = true;
}

bool gnss_probe_commands_idle(void)
{
    return gnss_idle;
}

bool gnss_probe_static_pvt(void)
{
    return auto_ready && static_pvt;
}

bool gnss_probe_user_profile(void)
{
    return gnss_managed;
}

bool gnss_probe_defaults_request(void)
{
    if (gnss_idle)
    {
        gnss_selected = SC_SETTINGS_GNSS_AUTO;
    }
    return gnss_idle;
}

bool gnss_probe_reboot_request(void)
{
    reboot_requested = gnss_idle;
    if (gnss_idle)
    {
        auto_ready = false;
    }
    return gnss_idle;
}

bool gnss_probe_reboot_transmitted(void)
{
    return reboot_sent;
}

bool gnss_probe_reboot_failed(void)
{
    return reboot_failed;
}

uint32_t __get_PRIMASK(void)
{
    return 0U;
}

void __disable_irq(void) {}

void __set_PRIMASK(uint32_t mask)
{
    (void)mask;
}

void __DMB(void) {}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address, uint16_t reg,
                                    uint16_t address_size, uint8_t *data, uint16_t size,
                                    uint32_t timeout)
{
    CHECK(bus == &hi2c2 && address == (SIT5801_ADDRESS_7BIT << 1U) &&
          address_size == I2C_MEMADD_SIZE_8BIT && size == 2U && timeout == 5U && reg >= 0x0cU &&
          reg <= 0x0eU);
    CHECK(write_count < sizeof(write_events) / sizeof(write_events[0]));
    write_events[write_count++] =
        (write_event_t){reg, (uint16_t)(((uint16_t)data[0] << 8U) | data[1]), now_ms};
    if (bus_busy)
    {
        return HAL_ERROR;
    }
    if (fail_register == (int)reg)
    {
        return HAL_ERROR;
    }
    oscillator_words[reg - 0x0cU] = write_events[write_count - 1U].value;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address, uint16_t reg,
                                   uint16_t address_size, uint8_t *data, uint16_t size,
                                   uint32_t timeout)
{
    (void)bus;
    (void)address;
    (void)reg;
    (void)address_size;
    (void)data;
    (void)size;
    (void)timeout;
    return HAL_ERROR;
}

static bool fake_send(const uint8_t *data, uint16_t length)
{
    telemetry_check(data, length);
    char captured[1025];
    if (usb_busy)
    {
        return false;
    }
    if (usb_window_mode)
    {
        if ((uint32_t)(now_ms - usb_window_ms) >= 10U)
        {
            usb_window_ms = now_ms;
            usb_window_used = false;
        }
        if (usb_window_used)
        {
            return false;
        }
        usb_window_used = true;
    }
    if (length > 1024U || length < 2U || data[length - 2U] != '\r' || data[length - 1U] != '\n')
    {
        report_bad = true;
    }
    if (length < sizeof(captured))
    {
        memcpy(captured, data, length);
        captured[length] = '\0';
        if (strstr(captured, "type=hybrid_command") != NULL)
        {
            command_reports++;
            command_busy_reported |= strstr(captured, "status=busy") != NULL;
            command_seen = true;
            command_accepted = strstr(captured, " accepted=1") != NULL;
        }
        if (strstr(captured, "type=system_reboot") && strstr(captured, " stage=failed"))
        {
            reboot_failure_reported = true;
        }
        if (strstr(captured, "type=clock_auto") != NULL &&
            strstr(captured, " loop_mode=PHASE") != NULL)
        {
            loop_reported = true;
            phase_wait_reported |= strstr(captured, " phase_mode=PHASE_WAIT") != NULL;
        }
        if (strstr(captured, "type=hybrid_config") != NULL &&
            strstr(captured, " sbas_fll_time_s=1200") != NULL &&
            strstr(captured, " sbas_phase_time_s=12000") != NULL)
        {
            profile_reported = true;
        }
        if (strstr(captured, "type=clock_auto") != NULL)
        {
            report_type_mask |= 1U;
        }
        if (strstr(captured, "type=clock_actuator") != NULL)
        {
            report_type_mask |= 2U;
        }
        if (strstr(captured, "type=thermal_outer") != NULL)
        {
            report_type_mask |= 4U;
            reported_duty_window_valid = strstr(captured, " window_valid=1") != NULL;
        }
        if (strstr(captured, "type=hybrid_quality") != NULL)
        {
            report_type_mask |= 8U;
        }
        if (strstr(captured, "type=hybrid_config") != NULL &&
            strstr(captured, " version=" SC_FIRMWARE_VERSION " ") != NULL)
        {
            report_type_mask |= 16U; /* Counted only with the release version. */
        }
    }
    report_count++;
    return true;
}

bool simpleclock_usb_cdc_try_send(const uint8_t *data, uint16_t length)
{
    return fake_send(data, length);
}

bool simpleclock_usb_cdc_try_send_queued(const uint8_t *data, uint16_t length)
{
    return fake_send(data, length);
}

void thermal_probe_fault(void)
{
    thermal_fault_count++;
    thermal_safe = false;
}

void thermal_probe_snapshot(uint32_t at_ms, thermal_probe_snapshot_t *out)
{
    if (snapshot_interrupt)
    {
        /* SysTick can complete a frame after the caller captures its time,
         * before the foreground takes the interrupt-owned duty snapshot. */
        now_ms++;
        snapshot_interrupt = false;
    }
    const int32_t target = (int32_t)thermal_target_q16;
    out->sample_ms = at_ms;
    out->frame_sequence = now_ms / 1000U + skipped_frames;
    out->frame_ms = now_ms - now_ms % 1000U;
    out->faults = thermal_faults;
    out->filtered_q16 = target;
    out->target_q16 = target;
    out->proportional_micro = 0;
    out->integral_micro = 0;
    out->raw = (int16_t)((target + 256L) / 512L + thermal_raw_offset);
    out->completed_duty_permille = thermal_duty_permille;
    out->safe = thermal_safe;
    out->sensor_valid = thermal_sensor_valid;
    out->warmed = thermal_sensor_valid;
    out->running = true;
    out->completed_frame_valid = true;
    out->regulation_stable = true;
}

bool thermal_probe_set_target(int32_t target)
{
    thermal_target_q16 = (uint32_t)target;
    return true;
}

bool gnss_probe_auto_ready(void)
{
    return auto_ready;
}

uint32_t gnss_probe_selection(void)
{
    return gnss_selected;
}

bool gnss_probe_selection_ready(void)
{
    return auto_ready;
}

bool gnss_probe_auto_fault(void)
{
    return receiver_config_fault;
}

bool gnss_probe_auto_recheck(void)
{
    if (!gnss_managed || !gnss_idle)
    {
        return false;
    }
    recheck_requests++;
    return true;
}

bool gnss_probe_pps_snapshot(gnss_probe_pps_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return false;
    }
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->policy = pps_policy;
    return false;
}

bool gnss_probe_pps_policy_set(uint32_t policy)
{
    if (policy > SC_SETTINGS_PPS_MANUAL || !gnss_idle)
    {
        return false;
    }
    pps_policy = policy;
    return true;
}

uint32_t gnss_probe_pps_policy(void)
{
    return pps_policy;
}

void gnss_probe_pps_quality(bool qualified)
{
    pps_quality = qualified;
    pps_quality_updates++;
}

const sc_epoch_t *timing_probe_latest(void)
{
    return epoch_available ? &latest_epoch : NULL;
}

static uint64_t raw_bias_bits(double bias_ns)
{
    const double raw_ms = bias_ns / 1.0e6;
    uint64_t bits = 0U;
    memcpy(&bits, &raw_ms, sizeof(bits));
    return bits;
}

static uint32_t raw_drift_bits(double drift_ppb)
{
    const float raw_ppm = (float)(drift_ppb / 1000.0);
    uint32_t bits = 0U;
    memcpy(&bits, &raw_ppm, sizeof(bits));
    return bits;
}

static void quality_epoch(uint32_t source_ms, uint32_t tow_ms, double bias_ns, double drift_ppb)
{
    const uint8_t previous_system = latest_epoch.pvt.data.pvt.time_system;
    memset(&latest_epoch, 0, sizeof(latest_epoch));
    latest_epoch.mask = SC_EPOCH_ALL;
    latest_epoch.sources_valid = true;
    latest_epoch.week = 23U;
    latest_epoch.tow_ms = tow_ms;
    latest_epoch.sequence = epoch_sequence;
    for (unsigned i = 0U; i < SC_EPOCH_SOURCE_COUNT; ++i)
    {
        latest_epoch.source_ms[i] = source_ms;
    }
    latest_epoch.pvt.id = SC_SBF_PVT;
    latest_epoch.pvt.valid = SC_SBF_VALID_EPOCH | SC_SBF_VALID_FIX | SC_SBF_VALID_BIAS |
                             SC_SBF_VALID_DRIFT | SC_SBF_VALID_TIME_SYSTEM | SC_SBF_VALID_CORR_AGE;
    latest_epoch.pvt.week = latest_epoch.week;
    latest_epoch.pvt.tow_ms = tow_ms;
    latest_epoch.pvt.data.pvt.bias_bits = raw_bias_bits(bias_ns);
    latest_epoch.pvt.data.pvt.drift_bits = raw_drift_bits(drift_ppb);
    latest_epoch.pvt.data.pvt.mode = 10U;
    latest_epoch.pvt.data.pvt.error = 0U;
    latest_epoch.pvt.data.pvt.time_system = 1U;
    /* Exercise full autonomous acquisition/tracking with valid five-SV PPP;
     * the removed minimum-count gate must not survive in the board adapter. */
    latest_epoch.pvt.data.pvt.nr_sv = 5U;
    latest_epoch.pvt.data.pvt.wa_corr_info = 1U;
    latest_epoch.pvt.data.pvt.alert = 1U;
    latest_epoch.pvt.data.pvt.mean_corr_age = 100U;
    if (standalone_mode)
    {
        latest_epoch.pvt.data.pvt.mode =
            static_pvt ? SC_SBF_PVT_MODE_FIXED : SC_SBF_PVT_MODE_STANDALONE;
        latest_epoch.pvt.data.pvt.time_system = 0U;
        latest_epoch.pvt.data.pvt.mean_corr_age = UINT16_MAX;
        latest_epoch.pvt.data.pvt.wa_corr_info = 0U;
        latest_epoch.pvt.valid &= (uint16_t)~SC_SBF_VALID_CORR_AGE;
    }
    if (previous_system != latest_epoch.pvt.data.pvt.time_system)
    {
        latest_epoch.reasons = SC_SEG_SYSTEM;
    }
    latest_epoch.rx_state = SC_SBF_RX_STATE_EXT_FREQ;
    latest_epoch.sync_level = SC_SBF_TIME_SYNC_READY_MASK;
    latest_epoch.jumps_valid = true;
}

static void rf_status(uint32_t source_ms, uint32_t tow_ms)
{
    sc_sbf_observation_t rf = {0};
    rf.id = SC_SBF_RF_STATUS;
    rf.week = 23U;
    rf.tow_ms = tow_ms;
    rf.valid = SC_SBF_VALID_EPOCH | SC_SBF_VALID_RF_STATUS;
    rf.data.rf_status.flags = 0U;
    hybrid_probe_observe(&rf, source_ms);
}

static void make_probe(sit5801_probe_t *probe, bool valid)
{
    memset(probe, 0, sizeof(*probe));
    probe->wire[0] = 0xd012U;
    probe->wire[1] = 32U;
    probe->wire[2] = 0U;
    probe->wire[3] = oscillator_words[0];
    probe->wire[4] = oscillator_words[1];
    probe->wire[5] = oscillator_words[2];
    probe->valid_mask = valid ? 0xffU : 0xfeU;
    probe->index = 0U;
}

static void cycle(double physical_ppb, double *bias_ns, bool valid_probe)
{
    now_ms += 1000U;
    epoch_tow_ms += 1000U;
    epoch_sequence++;
    const int64_t current = sit5801_fcw_decode(oscillator_words);
    const double drift = physical_ppb + sit5801_fcw_to_ppb(current);
    *bias_ns += drift;
    quality_epoch(now_ms, epoch_tow_ms, *bias_ns, drift);
    rf_status(now_ms, epoch_tow_ms);
    sit5801_probe_t probe;
    make_probe(&probe, valid_probe);
    const uint32_t reads[3] = {now_ms - 150U, now_ms - 100U, now_ms - 50U};
    hybrid_probe_cycle(&probe, reads, now_ms);
    test_tim2.CNT += 10U;
    hybrid_probe_service(now_ms);
}

static void reset_fixture(double initial_ppb)
{
    reset_called = reboot_requested = reboot_sent = reboot_failed = false;
    reboot_failure_reported = false;
    gnss_idle = true;
    now_ms = 100000U;
    usb_busy = false;
    usb_window_ms = now_ms;
    usb_window_used = false;
    bus_busy = false;
    fail_register = -1;
    auto_ready = true;
    receiver_config_fault = false;
    recheck_requests = 0U;
    standalone_mode = false;
    static_pvt = false;
    thermal_safe = true;
    thermal_sensor_valid = true;
    thermal_faults = 0U;
    thermal_duty_permille = 500U;
    thermal_raw_offset = 0;
    epoch_available = true;
    thermal_fault_count = 0U;
    skipped_frames = 0U;
    report_count = 0U;
    report_type_mask = 0U;
    report_bad = false;
    write_count = 0U;
    epoch_tow_ms = 1000000U;
    epoch_sequence = 0U;
    memset(&latest_epoch, 0, sizeof(latest_epoch));
    memset(write_events, 0, sizeof(write_events));
    memset(&test_i2c2, 0, sizeof(test_i2c2));
    memset(&test_tim2, 0, sizeof(test_tim2));
    int64_t q = 0;
    (void)sit5801_ppb_to_fcw(initial_ppb, &q);
    uint16_t encoded[3];
    (void)sit5801_fcw_encode(q, encoded);
    memcpy(oscillator_words, encoded, sizeof(oscillator_words));
    thermal_target_q16 = (uint32_t)(boot_target_C * 65536.0);
    /* Boot like a unit with operator-saved BOTH pairs; these scenarios test
     * FLL acquisition, not the compiled PPP PHASE default. */
    sc_settings_t saved;
    uint32_t generation;
    if (settings_flash_load(&saved, &generation) != SC_SETTINGS_VALID)
    {
        sc_settings_defaults(&saved);
        saved.gnss_selection = gnss_selected;
        saved.ppp_frequency_s = 60U;
        saved.ppp_phase_s = 600U;
        saved.sbas_frequency_s = saved.standalone_frequency_s = saved.static_frequency_s = 120U;
        saved.sbas_phase_s = saved.standalone_phase_s = saved.static_phase_s = 3000U;
        (void)settings_flash_save(&saved);
    }
    hybrid_probe_init();
    hybrid_probe_service(now_ms);
}

static int check_ordered_writes(void)
{
    CHECK(write_count == hybrid_probe_writes() * 3U);
    for (unsigned i = 0U; i < write_count; ++i)
    {
        CHECK(write_events[i].reg == (uint16_t)(0x0cU + i % 3U));
    }
    for (unsigned base = 0U; base < write_count; base += 3U)
    {
        uint16_t triple[3] = {write_events[base].value, write_events[base + 1U].value,
                              write_events[base + 2U].value};
        int64_t decoded = sit5801_fcw_decode(triple);
        uint16_t roundtrip[3];
        CHECK(sit5801_fcw_encode(decoded, roundtrip));
        CHECK(memcmp(triple, roundtrip, sizeof(triple)) == 0);
    }
    if (write_count != 0U)
    {
        uint16_t triple[3] = {write_events[write_count - 3U].value,
                              write_events[write_count - 2U].value,
                              write_events[write_count - 1U].value};
        CHECK(sit5801_fcw_decode(triple) == sit5801_fcw_decode(oscillator_words));
    }
    return 0;
}

static int scenario_normal(void)
{
    reset_fixture(2.0);
    usb_busy = true; /* No host/DTR and every diagnostic send is unavailable. */
    report_count = 0U;
    double bias = 0.0;
    for (unsigned i = 0U; i < 600U && !hybrid_probe_locked(); ++i)
    {
        cycle(1.25, &bias, true);
    }
    const sc_hybrid_t *state = hybrid_probe_state();
    CHECK(hybrid_probe_locked() && state->known && state->boot_ppb > 1.9 && state->boot_ppb < 2.1 &&
          hybrid_probe_writes() > 0U);
    CHECK(hybrid_probe_frequency_settled() && state->frequency.settled);
    CHECK(check_ordered_writes() == 0);
    CHECK(report_count == 0U); /* USB busy never gates the controller. */
    return 0;
}

static int scenario_standalone(bool fixed)
{
    reset_fixture(1.25);
    /* Standalone is the selected (primary) mode here, so it steers phase. */
    gnss_selected = SC_SETTINGS_GNSS_STANDALONE;
    double bias = 0.0;
    const double physical = -sit5801_fcw_to_ppb(sit5801_fcw_decode(oscillator_words));
    for (unsigned i = 0U; i < 400U; i++)
    {
        cycle(physical, &bias, true);
    }
    CHECK(hybrid_probe_locked() && hybrid_probe_frequency_settled());
    const int64_t base = sit5801_fcw_decode(oscillator_words);
    standalone_mode = true;
    static_pvt = fixed;
    bias += 1000000.0;
    for (unsigned i = 0U; i < 250U; i++)
    {
        cycle(physical, &bias, true);
        CHECK(sit5801_fcw_decode(oscillator_words) == base);
        if (i < 120U)
        {
            CHECK(sc_hybrid_frequency_lit(hybrid_probe_state(), now_ms));
        }
    }
    /* A Static-PVT fixed-position solution is its own source (static pair). */
    CHECK(hybrid_probe_state()->source_mode == (fixed ? 3U : 1U) &&
          hybrid_probe_state()->phase_enabled);
    cycle(physical + 0.08, &bias, true);
    const double step =
        sit5801_fcw_to_ppb(sit5801_fcw_decode(oscillator_words)) - sit5801_fcw_to_ppb(base);
    CHECK(fabs(step + 0.08 / 120.0) < 0.000002);
    standalone_mode = false;
    static_pvt = false;
    bias = 0.0;
    cycle(physical, &bias, true);
    CHECK(hybrid_probe_state()->state == SC_HYBRID_PHASE_RECOVERING &&
          hybrid_probe_state()->source_mode == 10U && hybrid_probe_state()->phase_enabled &&
          hybrid_probe_state()->phase_valid && !hybrid_probe_state()->locked);
    CHECK(hybrid_probe_frequency_settled());
    return 0;
}

static int scenario_integrity_loss(void)
{
    reset_fixture(2.0);
    thermal_duty_permille = 400U;
    double bias = 0.0;
    for (unsigned i = 0U; i < 600U && !hybrid_probe_locked(); ++i)
    {
        cycle(1.25, &bias, true);
    }
    CHECK(hybrid_probe_locked());
    /* Let the outer loop see a fully healthy locked interval before loss. */
    for (unsigned i = 0U; i < 360U; ++i)
    {
        cycle(1.25, &bias, true);
    }
    const uint32_t nominal_target = (uint32_t)(39.399 * 65536.0);
    CHECK(thermal_target_q16 != nominal_target);
    auto_ready = false; /* Models loss of verified PPP/HAS/RF readiness. */
    for (unsigned i = 0U; i < 12U; ++i)
    {
        cycle(1.25, &bias, true);
    }
    /* Clock holdover only: thermal centering keeps its own permission. */
    CHECK(!hybrid_probe_locked() && hybrid_probe_state()->state == SC_HYBRID_HOLDOVER);
    CHECK(hybrid_probe_frequency_settled()); /* Bounded missing-measurement indication. */
    for (unsigned i = 0U; i < 120U; ++i)
    {
        cycle(1.25, &bias, true);
    }
    CHECK(!hybrid_probe_frequency_settled() && !hybrid_probe_state()->frequency.settled);
    return 0;
}

static int scenario_null_latest_freeze(void)
{
    reset_fixture(2.0);
    thermal_duty_permille = 400U;
    double bias = 0.0;
    for (unsigned i = 0U; i < 600U && !hybrid_probe_locked(); ++i)
    {
        cycle(1.25, &bias, true);
    }
    CHECK(hybrid_probe_locked());
    for (unsigned i = 0U; i < 360U; ++i)
    {
        cycle(1.25, &bias, true);
    }
    const uint32_t nominal_target = (uint32_t)(39.399 * 65536.0);
    CHECK(thermal_target_q16 != nominal_target);
    const unsigned writes_before = write_count;

    epoch_available = false; /* Timing assembler invalidated latest on input loss. */
    now_ms += 1U;
    hybrid_probe_service(now_ms);
    /* Immediately: trim held and no lock indication, but a bridged short gap. */
    CHECK(sc_hybrid_tracking(hybrid_probe_state()) && !hybrid_probe_state()->gnss_good &&
          !hybrid_probe_locked());

    sit5801_probe_t probe;
    make_probe(&probe, true);
    for (unsigned i = 0U; i < 12U; ++i) /* No writes, then holdover past the bridge. */
    {
        now_ms += 1000U;
        hybrid_probe_service(now_ms);
        const uint32_t reads[3] = {now_ms - 150U, now_ms - 100U, now_ms - 50U};
        hybrid_probe_cycle(&probe, reads, now_ms);
        CHECK(write_count == writes_before);
    }
    CHECK(hybrid_probe_state()->state == SC_HYBRID_HOLDOVER);
    return 0;
}

static int scenario_usb_parts(void)
{
    usb_window_mode = true;
    reset_fixture(2.0);
    for (unsigned i = 0U; i < 20U; ++i)
    {
        hybrid_probe_service(now_ms);
        now_ms += 10U;
    }
    CHECK(!report_bad && (report_type_mask & 31U) == 31U);
    return 0;
}

static int scenario_partial(void)
{
    reset_fixture(2.0);
    double bias = 0.0;
    bool failed = false;
    unsigned before = 0U;
    for (unsigned i = 0U; i < 220U && !failed; ++i)
    {
        const sc_hybrid_t *state = hybrid_probe_state();
        if (state->state == SC_HYBRID_ACQUIRE)
        {
            /* 0x0C is committed only after all three exact writes succeed. */
            fail_register = 0x0d;
            before = write_count;
            failed = true;
        }
        cycle(1.25, &bias, true);
    }
    CHECK(failed && write_count == before + 2U && write_events[before].reg == 0x0cU &&
          write_events[before + 1U].reg == 0x0dU);
    CHECK(!hybrid_probe_state()->known && hybrid_probe_faulted());
    const unsigned after_fault = write_count;
    bus_busy = false;
    cycle(1.25, &bias, true);
    CHECK(write_count == after_fault && !hybrid_probe_state()->known);
    return 0;
}

static int scenario_busy(void)
{
    reset_fixture(2.0);
    double bias = 0.0;
    bool reached = false;
    unsigned before = 0U;
    for (unsigned i = 0U; i < 220U && !reached; ++i)
    {
        reached = hybrid_probe_state()->state == SC_HYBRID_ACQUIRE;
        if (reached)
        {
            before = write_count;
            test_i2c2.ISR = I2C_ISR_BUSY;
        }
        cycle(1.25, &bias, true);
    }
    CHECK(reached && write_count == before && !hybrid_probe_state()->known &&
          hybrid_probe_faulted());
    test_i2c2.ISR = 0U;
    cycle(1.25, &bias, true);
    CHECK(write_count == before && !hybrid_probe_state()->known);
    return 0;
}

static int scenario_bad_readback(void)
{
    reset_fixture(2.0);
    double bias = 0.0;
    for (unsigned i = 0U; i < 220U; ++i)
    {
        cycle(1.25, &bias, true);
        if (hybrid_probe_writes() != 0U)
        {
            break;
        }
    }
    CHECK(hybrid_probe_writes() != 0U);
    const unsigned before = write_count;
    cycle(1.25, &bias, false);
    CHECK(write_count == before && !hybrid_probe_state()->known && hybrid_probe_faulted());
    return 0;
}

static int scenario_sticky_hold(void)
{
    reset_fixture(2.0);
    double bias = 0.0;
    cycle(1.25, &bias, true);
    hybrid_probe_manual_request();
    hybrid_probe_disconnect();
    hybrid_probe_service(now_ms);
    CHECK(!hybrid_probe_state()->enabled && hybrid_probe_state()->state == SC_HYBRID_MANUAL_HOLD);
    const unsigned before = write_count;
    for (unsigned i = 0U; i < 4U; ++i)
    {
        cycle(1.25, &bias, true);
    }
    CHECK(write_count == before && !hybrid_probe_state()->enabled);
    CHECK(hybrid_probe_command("hybrid resume"));
    now_ms += 2001U;
    hybrid_probe_service(now_ms);
    CHECK(!hybrid_probe_state()->enabled && hybrid_probe_state()->state == SC_HYBRID_MANUAL_HOLD);
    return 0;
}

static int scenario_sensor_fault(void)
{
    reset_fixture(2.0);
    double bias = 0.0;
    cycle(1.25, &bias, true);
    thermal_sensor_valid = false;
    thermal_safe = false;
    thermal_faults = 1U;
    now_ms += 1000U;
    hybrid_probe_service(now_ms);
    /* The heater faults (LED3), but GNSS qualification of the clock continues. */
    CHECK(hybrid_probe_faulted() && hybrid_probe_state()->state != SC_HYBRID_HOLDOVER);
    const unsigned accepted = hybrid_probe_state()->history.accepted;
    for (unsigned i = 0U; i < 5U; ++i)
    {
        cycle(1.25, &bias, true);
    }
    CHECK(hybrid_probe_state()->qualifying &&
          hybrid_probe_state()->history.accepted == accepted + 5U);
    CHECK(hybrid_probe_thermal_warning() & HYBRID_PROBE_THERMAL_SENSOR);
    return 0;
}

static int scenario_duty_gap(void)
{
    reset_fixture(2.0);
    double bias = 0.0;
    for (unsigned i = 0U; i < 60U; ++i)
    {
        cycle(1.25, &bias, true);
    }
    CHECK(hybrid_probe_state()->qualifying);
    /* One completed heater frame was overwritten before foreground sampled
     * it. Receiver epochs remain continuous, isolating missing duty evidence. */
    skipped_frames++;
    cycle(1.25, &bias, true);
    cycle(1.25, &bias, true);
    /* Missing duty evidence is thermal-side: the heater is not faulted and the
     * clock's 61 s qualification completes on schedule (it used to restart). */
    CHECK(hybrid_probe_state()->state == SC_HYBRID_ACQUIRE && thermal_fault_count == 0U);
    /* Missing evidence is not a headroom warning: DUTY needs a valid window. */
    CHECK(!(hybrid_probe_thermal_warning() & HYBRID_PROBE_THERMAL_DUTY));
    for (unsigned i = 0U; i < 30U; ++i)
    {
        cycle(1.25, &bias, true);
    }
    CHECK(thermal_fault_count == 0U && write_count > 0U);
    return 0;
}

static int scenario_startup_recovery(void)
{
    boot_target_C = 43.25;
    reset_fixture(0.0);
    const uint32_t captured = thermal_target_q16;
    thermal_raw_offset = 38; /* Hot board, zero external heater demand. */
    thermal_duty_permille = 0;
    auto_ready = false;
    double bias = 0.0;
    for (unsigned i = 0; i < 100U; ++i)
    {
        cycle(0.1, &bias, true);
    }
    CHECK(thermal_target_q16 > captured && write_count == 0U);
    CHECK(!hybrid_probe_state()->gnss_good);

    auto_ready = true;
    thermal_raw_offset = 0;
    thermal_duty_permille = 200; /* Usable headroom; 50% is not a clock prerequisite. */
    bool moved_in_qualify = false, moved_in_acquire = false;
    unsigned elapsed = 0;
    while (elapsed++ < 300U && write_count == 0U)
    {
        const uint32_t before = thermal_target_q16;
        cycle(0.1, &bias, true);
        if (hybrid_probe_state()->qualifying)
        {
            moved_in_qualify |= thermal_target_q16 > before;
        }
        if (hybrid_probe_state()->state == SC_HYBRID_ACQUIRE)
        {
            moved_in_acquire |= thermal_target_q16 > before;
        }
    }
    CHECK(moved_in_qualify && moved_in_acquire && write_count > 0U);
    CHECK(!hybrid_probe_state()->phase_valid); /* Latch must already protect acquisition. */
    auto_ready = false;
    const uint32_t held = thermal_target_q16;
    const unsigned held_writes = write_count;
    for (unsigned i = 0; i < 15U; ++i)
    {
        cycle(0.1, &bias, true);
        CHECK(write_count == held_writes); /* Clock holdover: no trim writes. */
    }
    CHECK(hybrid_probe_state()->state == SC_HYBRID_HOLDOVER);
    /* GNSS loss no longer freezes outer centering during clock holdover. */
    CHECK(thermal_target_q16 > held);
    CHECK(hybrid_probe_command("hybrid hold"));
    cycle(0.1, &bias, true);
    CHECK(hybrid_probe_command("hybrid resume"));
    cycle(0.1, &bias, true);
    auto_ready = true;
    for (unsigned i = 0; i < 10U; ++i)
    {
        cycle(0.1, &bias, true);
    }
    CHECK(thermal_target_q16 > held && thermal_fault_count == 0U);
    return 0;
}

static int scenario_frame_snapshot_race(void)
{
    reset_fixture(0.0);
    for (unsigned i = 0; i < 45U; ++i)
    {
        now_ms = 101000U + i * 1000U - 1U;
        snapshot_interrupt = true;
        hybrid_probe_service(now_ms);
        for (unsigned part = 0; part < 6U; ++part)
        {
            now_ms += 10U;
            hybrid_probe_service(now_ms);
        }
        if (i >= 32U)
        {
            CHECK(reported_duty_window_valid);
        }
    }
    CHECK(thermal_fault_count == 0U);
    return 0;
}

static int run_command(const char *line, bool expected)
{
    command_seen = command_accepted = false;
    CHECK(hybrid_probe_command(line));
    now_ms += 1U;
    hybrid_probe_service(now_ms);
    CHECK(command_seen && command_accepted == expected);
    return 0;
}

static int scenario_usb_commands(void)
{
    reset_fixture(1.25);
    command_seen = command_accepted = command_busy_reported = false;
    command_reports = 0U;
    usb_busy = true;
    CHECK(hybrid_probe_command("hybrid hold"));
    hybrid_probe_service(++now_ms);
    CHECK(!hybrid_probe_state()->enabled && !command_seen);
    now_ms += 100U;
    usb_busy = false;
    hybrid_probe_service(now_ms);
    CHECK(command_seen && command_accepted && command_reports == 1U);
    hybrid_probe_service(++now_ms);
    CHECK(command_reports == 1U); /* Reply retry never executes/reports twice. */
    usb_busy = true;
    CHECK(hybrid_probe_command("hybrid status"));
    hybrid_probe_service(++now_ms);
    CHECK(hybrid_probe_command("hybrid resume")); /* Queued behind an unsent reply. */
    now_ms += 3100U;
    hybrid_probe_service(now_ms); /* Expire the old reply, leaving control held. */
    usb_busy = false;
    hybrid_probe_service(++now_ms);
    CHECK(command_seen && !command_accepted && !hybrid_probe_state()->enabled);
    CHECK(hybrid_probe_command("hybrid status"));
    CHECK(hybrid_probe_command("hybrid resume")); /* Mailbox full: reject/count, never replace. */
    hybrid_probe_service(++now_ms);
    hybrid_probe_service(++now_ms);
    CHECK(command_busy_reported && !hybrid_probe_state()->enabled);
    return 0;
}

static int scenario_reboot_usb_timeout(void)
{
    for (unsigned sent = 0U; sent < 2U; sent++)
    {
        reset_fixture(1.25);
        usb_busy = sent == 0U;
        CHECK(hybrid_probe_command("system reboot mcu"));
        hybrid_probe_service(++now_ms);
        usb_busy = true;
        now_ms += 6000U;
        hybrid_probe_service(now_ms); /* Fail before reset if acceptance/drain cannot finish. */
        now_ms += 3001U;
        hybrid_probe_service(now_ms); /* Bound the undeliverable failure report too. */
        CHECK(!reset_called && !hybrid_probe_state()->enabled);
        usb_busy = false;
        CHECK(run_command("hybrid resume", true) == 0);
        now_ms += 1000U;
        hybrid_probe_service(now_ms);
        CHECK(!reset_called); /* Reconnecting cannot execute an old reboot. */
    }
    return 0;
}

/* Settings edits while steering: an FCW write is pending for most of each
 * second, so they wait for the call after the paced readback, not rejected. */
static int scenario_steering_commands(void)
{
    reset_fixture(1.25);
    double bias = 0.0;
    const double physical = -sit5801_fcw_to_ppb(sit5801_fcw_decode(oscillator_words));
    for (unsigned i = 0U; i < 900U && !hybrid_probe_locked(); ++i)
    {
        cycle(physical, &bias, true);
    }
    CHECK(hybrid_probe_locked());
    const char *const lines[] = {"hybrid params sbas 10 100", "hybrid clip 1500"};
    for (unsigned k = 0U; k < 2U; ++k)
    {
        for (unsigned i = 0U; i < 200U && !hybrid_probe_state()->pending; ++i)
        {
            cycle(physical + (i % 2U ? 0.002 : -0.002), &bias, true);
        }
        CHECK(hybrid_probe_state()->pending);
        command_seen = command_accepted = false;
        CHECK(hybrid_probe_command(lines[k]));
        hybrid_probe_service(++now_ms);
        CHECK(!command_seen); /* Waiting in the mailbox, not rejected. */
        cycle(physical, &bias, true);
        CHECK(command_seen && command_accepted);
    }
    CHECK(hybrid_probe_state()->sbas_phase_time_s == 100U &&
          hybrid_probe_state()->bias_clip_ps == 1500U && hybrid_probe_state()->phase_enabled);
    /* Saving those changed values keeps steering: no requalification or re-anchor. */
    const uint32_t generation = hybrid_probe_state()->generation;
    const uint32_t reanchors = hybrid_probe_state()->reanchor_count;
    for (unsigned i = 0U; i < 200U && !hybrid_probe_state()->pending; ++i)
    {
        cycle(physical + (i % 2U ? 0.002 : -0.002), &bias, true);
    }
    command_seen = command_accepted = false;
    CHECK(hybrid_probe_command("hybrid save"));
    hybrid_probe_service(++now_ms);
    CHECK(!command_seen);
    cycle(physical, &bias, true);
    CHECK(command_seen && command_accepted);
    sc_settings_t saved;
    uint32_t saved_generation;
    CHECK(settings_flash_load(&saved, &saved_generation) == SC_SETTINGS_VALID &&
          saved.sbas_phase_s == 100U && saved.bias_clip_ps == 1500U);
    for (unsigned i = 0U; i < 30U; ++i)
    {
        cycle(physical, &bias, true);
        CHECK(sc_hybrid_tracking(hybrid_probe_state()) && hybrid_probe_state()->phase_enabled);
    }
    CHECK(hybrid_probe_state()->generation == generation &&
          hybrid_probe_state()->reanchor_count == reanchors && hybrid_probe_locked());
    /* A receiver-profile save with nothing to write succeeds with an FCW in
     * flight (`gnss auto` on an already matching receiver while steering);
     * a real change is still refused until the actuator has settled. */
    for (unsigned i = 0U; i < 200U && !hybrid_probe_state()->pending; ++i)
    {
        cycle(physical + (i % 2U ? 0.002 : -0.002), &bias, true);
    }
    CHECK(hybrid_probe_state()->pending);
    CHECK(hybrid_probe_receiver_persist(saved.receiver_user_profile != 0U));
    CHECK(!hybrid_probe_receiver_persist(saved.receiver_user_profile == 0U));
    CHECK(sc_hybrid_tracking(hybrid_probe_state()) && hybrid_probe_state()->phase_enabled);
    /* Without a readback inside 1.5 s it still expires and is rejected. */
    for (unsigned i = 0U; i < 200U && !hybrid_probe_state()->pending; ++i)
    {
        cycle(physical + (i % 2U ? 0.002 : -0.002), &bias, true);
    }
    CHECK(hybrid_probe_state()->pending);
    command_seen = command_accepted = false;
    CHECK(hybrid_probe_command("hybrid params sbas 10 300"));
    for (unsigned i = 0U; i < 20U && !command_seen; ++i)
    {
        now_ms += 100U;
        hybrid_probe_service(now_ms);
    }
    CHECK(command_seen && !command_accepted && hybrid_probe_state()->sbas_phase_time_s == 100U);
    return 0;
}

static int scenario_runtime_loops(void)
{
    reset_fixture(1.25);
    /* Standalone is the selected (primary) mode here, so it steers phase. */
    gnss_selected = SC_SETTINGS_GNSS_STANDALONE;
    CHECK(run_command("hybrid params ppp 0 600", true) == 0);
    CHECK(hybrid_probe_state()->loop_mode == SC_HYBRID_LOOP_PHASE && hybrid_probe_state()->enabled);
    CHECK(run_command("hybrid params ppp 0 6000", true) == 0);
    CHECK(run_command("hybrid params sbas 1200 12000", true) == 0);
    CHECK(run_command("hybrid params standalone 1200 12000", true) == 0);
    CHECK(run_command("hybrid params static 10 40", true) == 0);
    CHECK(hybrid_probe_state()->static_frequency_time_s == 10U &&
          hybrid_probe_state()->static_phase_time_s == 40U &&
          hybrid_probe_state()->standalone_phase_time_s == 12000U);
    CHECK(run_command("hybrid params static 10 9", false) == 0);
    CHECK(hybrid_probe_state()->frequency_time_s == 0.0 &&
          hybrid_probe_state()->phase_time_s == 6000.0 &&
          hybrid_probe_state()->sbas_frequency_time_s == 1200.0 &&
          hybrid_probe_state()->sbas_phase_time_s == 12000.0);
    CHECK(run_command("hybrid mode invalid", false) == 0);
    CHECK(run_command("hybrid mode phase extra", false) == 0);
    CHECK(run_command("hybrid params ppp 1 600", false) == 0);
    CHECK(run_command("hybrid params sbas 120 86401", false) == 0);
    CHECK(run_command("hybrid params ppp 600", false) == 0);
    CHECK(run_command("hybrid params sbas 10 10 x", false) == 0);
    CHECK(hybrid_probe_state()->loop_mode == SC_HYBRID_LOOP_PHASE &&
          hybrid_probe_state()->frequency_time_s == 0.0);
    CHECK(run_command("hybrid params 0 600", true) == 0); /* Legacy shorthand. */
    double bias = 0.0;
    const double physical = -sit5801_fcw_to_ppb(sit5801_fcw_decode(oscillator_words));
    loop_reported = profile_reported = phase_wait_reported = false;
    for (unsigned i = 0U; i < 600U && !hybrid_probe_state()->phase_enabled; i++)
    {
        cycle(physical, &bias, true);
    }
    /* PHASE also verifies frequency first; a zero offset needs no new word. */
    CHECK(hybrid_probe_state()->phase_enabled && hybrid_probe_state()->attempts == 1U);
    CHECK(run_command("hybrid config", true) == 0);
    /* Foreground normally runs between epochs; let all five bounded report
     * parts drain instead of polling only once per simulated second. */
    for (unsigned i = 0U; i < 6U; i++)
    {
        now_ms += 10U;
        hybrid_probe_service(now_ms);
    }
    CHECK(hybrid_probe_writes() == 0U);
    CHECK(loop_reported);
    CHECK(phase_wait_reported);
    CHECK(profile_reported);
    CHECK(!report_bad);
    CHECK(run_command("hybrid params ppp 60 0", true) == 0);
    for (unsigned i = 0U; i < 80U; i++)
    {
        cycle(physical, &bias, true);
    }
    CHECK(!hybrid_probe_state()->phase_enabled && hybrid_probe_state()->desired_drift_ppb == 0.0);
    CHECK(run_command("hybrid params ppp 60 600", true) == 0);
    for (unsigned i = 0U; i < 180U; i++)
    {
        cycle(physical, &bias, true);
    }
    CHECK(hybrid_probe_state()->phase_enabled);
    /* Real USB command mailbox: tune both profiles while running after a
     * fallback and again after PPP recovery. No hold/resume or requalification. */
    standalone_mode = true;
    bias += 100.0;
    cycle(physical, &bias, true);
    CHECK(hybrid_probe_state()->phase_enabled && hybrid_probe_state()->source_mode == 1U);
    bias += 0.01; /* Nonzero phase error, small enough to avoid an FCW transaction. */
    cycle(physical, &bias, true);
    const sc_hybrid_t *running = hybrid_probe_state();
    CHECK(!running->pending && fabs(running->filtered_error_ns) > 0.0001);
    const double reference = running->phase_reference_ns, accumulator = running->accumulator_ppb;
    const double integral = running->phase_integral_ppb, v = running->desired_drift_ppb;
    const unsigned count = running->history.count, reference_generation = running->generation;
    const int64_t held_word = sit5801_fcw_decode(oscillator_words);
    CHECK(run_command("hybrid params ppp 100 1000", true) == 0); /* Inactive profile. */
    CHECK(running->phase_integral_ppb == integral);
    CHECK(run_command("hybrid params standalone 500 5000", true) == 0); /* Active PI preload. */
    CHECK(running->enabled && running->phase_enabled && running->previous_valid &&
          running->phase_reference_ns == reference && running->generation == reference_generation &&
          running->accumulator_ppb == accumulator && running->history.count == count &&
          running->standalone_frequency_time_s == 500.0 &&
          running->standalone_phase_time_s == 5000.0);
    CHECK(fabs(running->phase_integral_ppb - 2.0 / 5000.0 * running->filtered_error_ns - v) <
          1e-14);
    CHECK(sit5801_fcw_decode(oscillator_words) == held_word);
    standalone_mode = false;
    bias -= 100.0;
    cycle(physical, &bias, true);
    CHECK(running->enabled && running->phase_enabled && running->source_mode == 10U);
    const double recovered_v = running->desired_drift_ppb;
    CHECK(run_command("hybrid params ppp 200 2000", true) == 0);
    CHECK(running->enabled && running->phase_enabled && running->frequency_time_s == 200.0 &&
          running->phase_time_s == 2000.0);
    CHECK(fabs(running->phase_integral_ppb - 2.0 / 2000.0 * running->filtered_error_ns -
               recovered_v) < 1e-14);
    const double recovered_i = running->phase_integral_ppb;
    CHECK(run_command("hybrid params sbas 1000 10000", true) == 0);
    CHECK(running->phase_integral_ppb == recovered_i && running->enabled);
    CHECK(run_command("hybrid hold", true) == 0);
    CHECK(run_command("hybrid params ppp 86400 10", true) == 0);
    CHECK(run_command("hybrid params sbas 10 10", true) == 0);
    CHECK(run_command("hybrid params standalone 86400 86400", true) == 0);
    CHECK(run_command("hybrid save", true) == 0);
    sc_settings_t independent_saved;
    uint32_t independent_generation;
    CHECK(settings_flash_load(&independent_saved, &independent_generation) == SC_SETTINGS_VALID);
    CHECK(independent_saved.ppp_frequency_s == 86400U && independent_saved.ppp_phase_s == 10U &&
          independent_saved.sbas_frequency_s == 10U && independent_saved.sbas_phase_s == 10U &&
          independent_saved.standalone_frequency_s == 86400U &&
          independent_saved.standalone_phase_s == 86400U);
    CHECK(run_command("hybrid params ppp 0 10", true) == 0);
    CHECK(run_command("hybrid params sbas 86400 0", true) == 0);
    CHECK(run_command("hybrid params standalone 0 0", true) == 0);
    CHECK(run_command("hybrid params ppp 9 10", false) == 0);
    CHECK(run_command("hybrid params sbas 10 9", false) == 0);
    CHECK(run_command("hybrid mode both", false) == 0);
    CHECK(run_command("hybrid mode frequency", false) == 0);
    CHECK(run_command("hybrid params fallback 120 300", false) == 0);
    CHECK(run_command("hybrid save", true) == 0);
    sc_settings_t saved;
    uint32_t generation;
    CHECK(settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID);
    CHECK(saved.gnss_selection == SC_SETTINGS_GNSS_STANDALONE && saved.ppp_frequency_s == 0U &&
          saved.ppp_phase_s == 10U && saved.sbas_frequency_s == 86400U &&
          saved.sbas_phase_s == 0U && saved.standalone_frequency_s == 0U &&
          saved.standalone_phase_s == 0U);
    CHECK(!hybrid_probe_state()->enabled);
    CHECK(run_command("hybrid params sbas 120 300", true) == 0);
    hybrid_probe_init();
    CHECK(hybrid_probe_state()->enabled &&
          hybrid_probe_state()->loop_mode == SC_HYBRID_LOOP_PHASE &&
          hybrid_probe_state()->frequency_time_s == 0.0 &&
          hybrid_probe_state()->phase_time_s == 10.0 &&
          hybrid_probe_state()->sbas_frequency_time_s == 86400.0 &&
          hybrid_probe_state()->sbas_phase_time_s == 0.0 &&
          hybrid_probe_state()->standalone_frequency_time_s == 0.0 &&
          hybrid_probe_state()->standalone_phase_time_s == 0.0 &&
          !hybrid_probe_state()->phase_enabled);
    return 0;
}

static int scenario_reboot(void)
{
    reset_fixture(1.25);
    CHECK(run_command("system reboot invalid", false) == 0);
    gnss_idle = false;
    CHECK(run_command("system reboot mcu", false) == 0);
    gnss_idle = true;
    /* Brief USB backpressure retains the acknowledgement without executing early.
     * Expiry of a vanished host is exercised by reboot_usb_timeout. */
    usb_busy = true;
    CHECK(hybrid_probe_command("system reboot mcu"));
    now_ms++;
    hybrid_probe_service(now_ms);
    now_ms += 2000U;
    hybrid_probe_service(now_ms);
    CHECK(!reset_called && !hybrid_probe_state()->enabled);
    usb_busy = false;
    hybrid_probe_service(now_ms);
    now_ms += 499U;
    hybrid_probe_service(now_ms);
    CHECK(!reset_called);
    now_ms++;
    hybrid_probe_service(now_ms);
    CHECK(reset_called && !reboot_requested);
    /* GNSS restart waits for fresh configuration, then restores prior enable. */
    reset_fixture(1.25);
    CHECK(run_command("system reboot gnss", true) == 0);
    now_ms += 501U;
    hybrid_probe_service(now_ms);
    CHECK(reboot_requested && !reset_called && !hybrid_probe_state()->enabled);
    CHECK(run_command("hybrid resume", false) == 0);
    reboot_sent = true;
    now_ms += 1000U;
    hybrid_probe_service(now_ms);
    CHECK(!hybrid_probe_state()->enabled);
    auto_ready = true;
    hybrid_probe_service(now_ms);
    CHECK(hybrid_probe_state()->enabled && !reset_called);
    /* A later explicit HOLD wins over the remembered automatic-resume policy. */
    reset_fixture(1.25);
    CHECK(run_command("system reboot gnss", true) == 0);
    now_ms += 501U;
    hybrid_probe_service(now_ms);
    CHECK(run_command("hybrid hold", true) == 0);
    reboot_sent = auto_ready = true;
    hybrid_probe_service(now_ms);
    CHECK(!hybrid_probe_state()->enabled);
    /* Combined restart sends GNSS first and drains its final event before MCU. */
    reset_fixture(1.25);
    CHECK(run_command("system reboot all", true) == 0);
    now_ms += 501U;
    hybrid_probe_service(now_ms);
    CHECK(reboot_requested && !reset_called);
    reboot_sent = auto_ready = true;
    hybrid_probe_service(now_ms);
    CHECK(!reset_called);
    now_ms += 501U;
    hybrid_probe_service(now_ms);
    CHECK(reset_called);
    /* Rejection and no-response timeouts never trigger a second reset. */
    for (unsigned failure = 0U; failure < 2U; failure++)
    {
        reset_fixture(1.25);
        CHECK(run_command("system reboot all", true) == 0);
        now_ms += 501U;
        hybrid_probe_service(now_ms);
        reboot_failed = failure == 0U;
        now_ms += 30001U;
        hybrid_probe_service(now_ms);
        CHECK(!reset_called && !hybrid_probe_state()->enabled);
    }
    sc_settings_t saved;
    uint32_t generation;
    /* Only the fixture's boot record exists: reboot never saves settings. */
    CHECK(settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID && generation == 1U);
    return 0;
}

static int scenario_reboot_pending(unsigned failure)
{
    static const char *const commands[] = {"system reboot mcu", "system reboot gnss",
                                           "system reboot all"};
    for (unsigned target = 0U; target < 3U; target++)
    {
        reset_fixture(2.0);
        double bias = 0.0;
        for (unsigned i = 0U; i < 220U && !hybrid_probe_state()->pending; i++)
        {
            cycle(1.25, &bias, true);
        }
        CHECK(hybrid_probe_state()->pending && hybrid_probe_state()->known);
        const unsigned writes = write_count;
        CHECK(run_command(commands[target], true) == 0);
        CHECK(hybrid_probe_state()->pending && !hybrid_probe_state()->enabled);
        CHECK(!hybrid_probe_receiver_change_ready());
        now_ms += 501U;
        hybrid_probe_service(now_ms);
        CHECK(!reset_called && !reboot_requested);
        if (failure == 2U)
        {
            now_ms += 5001U;
            hybrid_probe_service(now_ms);
        }
        else
        {
            cycle(1.25, &bias, failure == 0U);
        }
        CHECK(write_count == writes); /* Holding never emits another FCW word. */
        if (failure)
        {
            CHECK(reboot_failure_reported && !reset_called && !reboot_requested);
            cycle(1.25, &bias, true); /* A later good probe cannot revive a failed reboot. */
            CHECK(!reset_called && !reboot_requested && !hybrid_probe_state()->enabled);
        }
        else if (target == 0U)
        {
            CHECK(reset_called && !reboot_requested);
        }
        else
        {
            CHECK(reboot_requested && !reset_called);
            reboot_sent = auto_ready = true;
            hybrid_probe_service(now_ms);
            now_ms += 501U;
            hybrid_probe_service(now_ms);
            CHECK(reset_called == (target == 2U));
            CHECK(hybrid_probe_state()->enabled == (target == 1U));
        }
    }
    sc_settings_t saved;
    uint32_t generation;
    /* Only the fixture's boot record exists: reboot never saves settings. */
    CHECK(settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID && generation == 1U);
    return 0;
}

static int scenario_defaults(void)
{
    reset_fixture(0.0);
    CHECK(run_command("hybrid hold", true) == 0);
    CHECK(run_command("hybrid params ppp 0 0", true) == 0);
    CHECK(run_command("hybrid params sbas 30 300", true) == 0);
    CHECK(run_command("hybrid params standalone 50 500", true) == 0);
    CHECK(run_command("hybrid save", true) == 0);
    gnss_idle = false;
    CHECK(run_command("system defaults", false) == 0);
    CHECK(hybrid_probe_state()->frequency_time_s == 0U);
    gnss_idle = true;
    CHECK(run_command("system defaults", true) == 0);
    CHECK(!hybrid_probe_state()->enabled);
    CHECK(hybrid_probe_state()->frequency_time_s == 0U &&
          hybrid_probe_state()->sbas_frequency_time_s == 30U);
    hybrid_probe_defaults_finish(false); /* Failed preflight preserves all six times. */
    CHECK(hybrid_probe_state()->frequency_time_s == 0U &&
          hybrid_probe_state()->sbas_frequency_time_s == 30U &&
          hybrid_probe_state()->standalone_phase_time_s == 500U);
    hybrid_probe_defaults_finish(true); /* A stale completion cannot reset them later. */
    CHECK(hybrid_probe_state()->frequency_time_s == 0U);
    CHECK(run_command("system defaults", true) == 0);
    hybrid_probe_defaults_finish(true);
    CHECK(hybrid_probe_receiver_persist(true)); /* Simulated receiver Boot completion. */
    gnss_managed = true;
    hybrid_probe_init();
    sc_settings_t saved, defaults;
    uint32_t generation;
    sc_settings_defaults(&defaults);
    defaults.receiver_user_profile = 1U;
    CHECK(settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID &&
          sc_settings_equal(&saved, &defaults));
    /* Compiled operating defaults: PPP/HAS PHASE Tphi 10 s; SBAS and
     * standalone Tf 10 s / Tphi 100 s. Every start acquires frequency first. */
    CHECK(hybrid_probe_state()->frequency_time_s == 0U &&
          hybrid_probe_state()->phase_time_s == 10U &&
          hybrid_probe_state()->sbas_frequency_time_s == 10U &&
          hybrid_probe_state()->sbas_phase_time_s == 100U &&
          hybrid_probe_state()->standalone_frequency_time_s == 10U &&
          hybrid_probe_state()->standalone_phase_time_s == 100U);
    return 0;
}

static int scenario_resume_recheck(void)
{
    reset_fixture(1.25);
    double bias = 0.0;
    cycle(1.25, &bias, true);
    hybrid_probe_manual_request();
    hybrid_probe_service(now_ms);
    auto_ready = false;
    receiver_config_fault = true;
    gnss_managed = true;
    const unsigned writes = write_count;
    CHECK(run_command("hybrid resume", true) == 0);
    CHECK(recheck_requests == 1U && hybrid_probe_state()->enabled);
    cycle(1.25, &bias, true);
    CHECK(write_count == writes && !hybrid_probe_state()->gnss_good);
    auto_ready = true;
    receiver_config_fault = false;
    cycle(1.25, &bias, true);
    CHECK(hybrid_probe_state()->gnss_good);
    /* Successful configuration validation does not bypass a real receiver
     * warning: the bench's SOFTWARE latch (bit 3) still inhibits steering. */
    latest_epoch.sequence++;
    latest_epoch.rx_error = 8U;
    latest_epoch.sources_valid = false;
    now_ms++;
    hybrid_probe_service(now_ms);
    CHECK(!hybrid_probe_state()->gnss_good && hybrid_probe_state()->state == SC_HYBRID_HOLDOVER &&
          write_count == writes);
    hybrid_probe_manual_request();
    hybrid_probe_service(now_ms);
    auto_ready = false;
    receiver_config_fault = true;
    gnss_idle = false;
    CHECK(run_command("hybrid resume", false) == 0);
    CHECK(recheck_requests == 1U && !hybrid_probe_state()->enabled);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "scenario required: normal integrity_loss null_latest_freeze usb_parts "
                        "partial busy bad_readback sticky_hold sensor_fault duty_gap "
                        "startup_recovery frame_snapshot_race runtime_loops\n");
        return 2;
    }
    if (strcmp(argv[1], "normal") == 0)
    {
        return scenario_normal();
    }
    if (strcmp(argv[1], "standalone") == 0)
    {
        return scenario_standalone(false);
    }
    if (strcmp(argv[1], "static_pvt") == 0)
    {
        return scenario_standalone(true);
    }
    if (strcmp(argv[1], "usb_commands") == 0)
    {
        return scenario_usb_commands();
    }
    if (strcmp(argv[1], "reboot_usb_timeout") == 0)
    {
        return scenario_reboot_usb_timeout();
    }
    if (strcmp(argv[1], "integrity_loss") == 0)
    {
        return scenario_integrity_loss();
    }
    if (strcmp(argv[1], "null_latest_freeze") == 0)
    {
        return scenario_null_latest_freeze();
    }
    if (strcmp(argv[1], "usb_parts") == 0)
    {
        return scenario_usb_parts();
    }
    if (strcmp(argv[1], "partial") == 0)
    {
        return scenario_partial();
    }
    if (strcmp(argv[1], "busy") == 0)
    {
        return scenario_busy();
    }
    if (strcmp(argv[1], "bad_readback") == 0)
    {
        return scenario_bad_readback();
    }
    if (strcmp(argv[1], "sticky_hold") == 0)
    {
        return scenario_sticky_hold();
    }
    if (strcmp(argv[1], "sensor_fault") == 0)
    {
        return scenario_sensor_fault();
    }
    if (strcmp(argv[1], "duty_gap") == 0)
    {
        return scenario_duty_gap();
    }
    if (strcmp(argv[1], "startup_recovery") == 0)
    {
        return scenario_startup_recovery();
    }
    if (strcmp(argv[1], "frame_snapshot_race") == 0)
    {
        return scenario_frame_snapshot_race();
    }
    if (strcmp(argv[1], "runtime_loops") == 0)
    {
        return scenario_runtime_loops();
    }
    if (strcmp(argv[1], "steering_commands") == 0)
    {
        return scenario_steering_commands();
    }
    if (strcmp(argv[1], "reboot") == 0)
    {
        return scenario_reboot();
    }
    if (strcmp(argv[1], "reboot_pending") == 0)
    {
        return scenario_reboot_pending(0U);
    }
    if (strcmp(argv[1], "reboot_pending_bad") == 0)
    {
        return scenario_reboot_pending(1U);
    }
    if (strcmp(argv[1], "reboot_pending_timeout") == 0)
    {
        return scenario_reboot_pending(2U);
    }
    if (strcmp(argv[1], "defaults") == 0)
    {
        return scenario_defaults();
    }
    if (strcmp(argv[1], "resume_recheck") == 0)
    {
        return scenario_resume_recheck();
    }
    fprintf(stderr, "unknown scenario: %s\n", argv[1]);
    return 2;
}
