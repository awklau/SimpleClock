/* SPDX-License-Identifier: MIT */
/* Host exercise of the real bounded FCW actuator adapter. */
#include "main.h"
#include "actuator_probe.h"
#include "simpleclock/fcw_trial.h"
#include "simpleclock/sit5801.h"
#include "simpleclock/sbf_fields.h"
#include "simpleclock/timing.h"
#include "thermal_probe.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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

static uint32_t now_ms;
static bool pps_ok, ppp_ok, thermal_ok, latest_ok;
static bool publish_covariances;
static bool stale_covariances;
static sc_epoch_t epoch;
static uint16_t oscillator_words[3];
static unsigned write_count;
static unsigned report_count;
static int fail_register;
static bool usb_blocked;
static char last_report[1024];

typedef struct
{
    uint16_t reg;
    uint16_t value;
    uint32_t ms;
} write_event_t;

static write_event_t writes[128];

uint32_t HAL_GetTick(void)
{
    return now_ms;
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
    if (bus != &hi2c2 || address != (SIT5801_ADDRESS_7BIT << 1U) ||
        address_size != I2C_MEMADD_SIZE_8BIT || size != 2U || timeout != 5U || reg < 0x0cU ||
        reg > 0x0eU)
    {
        return HAL_ERROR;
    }
    if (write_count >= sizeof(writes) / sizeof(writes[0]))
    {
        return HAL_ERROR;
    }
    writes[write_count].reg = reg;
    writes[write_count].value = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    writes[write_count].ms = now_ms;
    write_count++;
    if (fail_register == (int)reg)
    {
        return HAL_ERROR;
    }
    oscillator_words[reg - 0x0cU] = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
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

static bool capture_report(const uint8_t *data, uint16_t length)
{
    if (usb_blocked)
    {
        return false;
    }
    if (length >= sizeof(last_report))
    {
        return false;
    }
    memcpy(last_report, data, length);
    last_report[length] = '\0';
    report_count++;
    return true;
}

bool simpleclock_usb_cdc_try_send(const uint8_t *data, uint16_t length)
{
    return capture_report(data, length);
}

bool simpleclock_usb_cdc_try_send_queued(const uint8_t *data, uint16_t length)
{
    return capture_report(data, length);
}

const sc_epoch_t *timing_probe_latest(void)
{
    return latest_ok ? &epoch : NULL;
}

bool timing_probe_pps_ready(uint32_t now)
{
    (void)now;
    return pps_ok;
}

bool timing_probe_ppp_active(uint32_t now)
{
    (void)now;
    return ppp_ok;
}

bool thermal_probe_trial_ready(uint32_t now, bool require_stable)
{
    (void)now;
    (void)require_stable;
    return thermal_ok;
}

static void quality_setup(void)
{
    memset(&epoch, 0, sizeof(epoch));
    epoch.segment = 11U;
    epoch.week = 23U;
    epoch.tow_ms = 456000U;
    epoch.pvt.valid =
        SC_SBF_VALID_EPOCH | SC_SBF_VALID_FIX | SC_SBF_VALID_TIME_SYSTEM | SC_SBF_VALID_CORR_AGE;
    epoch.pvt.data.pvt.mode = 10U;
    epoch.pvt.data.pvt.error = 0U;
    epoch.pvt.data.pvt.time_system = 1U;
    epoch.pvt.data.pvt.wa_corr_info = 1U;
    epoch.pvt.data.pvt.mean_corr_age = 1U;
    epoch.pvt.data.pvt.alert = 0U;
    pps_ok = true;
    ppp_ok = true;
    thermal_ok = true;
    latest_ok = true;
    publish_covariances = true;
    stale_covariances = false;
}

static void covariance_setup(bool publish)
{
    if (!publish)
    {
        return;
    }
    sc_sbf_observation_t observation = {0};
    observation.week = stale_covariances ? (uint16_t)(epoch.week - 1U) : epoch.week;
    observation.tow_ms = epoch.tow_ms;
    observation.id = SC_SBF_POS_COV_CARTESIAN;
    observation.valid = SC_SBF_VALID_EPOCH | SC_SBF_VALID_COV_BB;
    observation.data.cov.mode = 10U;
    actuator_probe_observe(&observation, now_ms);
    observation.id = SC_SBF_VEL_COV_CARTESIAN;
    observation.valid = SC_SBF_VALID_EPOCH | SC_SBF_VALID_COV_DTDT;
    observation.data.cov.mode = 10U;
    actuator_probe_observe(&observation, now_ms);
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
    probe->wire[6] = 0U;
    probe->wire[7] = 0U;
    probe->valid_mask = valid ? 0xffU : 0xfeU;
    probe->index = 0U;
}

static void cycle(bool valid)
{
    sit5801_probe_t probe;
    uint32_t read_ms[3] = {now_ms - 150U, now_ms - 100U, now_ms - 50U};
    covariance_setup(publish_covariances);
    make_probe(&probe, valid);
    actuator_probe_cycle(&probe, read_ms, now_ms);
    actuator_probe_service(now_ms);
}

static int check_group(unsigned group, int64_t expected, uint32_t expected_ms)
{
    uint16_t words[3];
    CHECK(sit5801_fcw_encode(expected, words));
    const unsigned base = group * 3U;
    CHECK(base + 2U < write_count);
    CHECK(writes[base].reg == 0x0cU && writes[base + 1U].reg == 0x0dU &&
          writes[base + 2U].reg == 0x0eU);
    for (unsigned i = 0; i < 3U; i++)
    {
        CHECK(writes[base + i].value == words[i] && writes[base + i].ms == expected_ms);
    }
    return 0;
}

static int scenario_denied(const char *name)
{
    quality_setup();
    now_ms = SC_FCW_SETTLE_AFTER_BOOT_MS;
    if (strcmp(name, "before") == 0)
    {
        now_ms--;
        covariance_setup(true);
    }
    else if (strcmp(name, "missing") != 0 && strcmp(name, "stale_cov") != 0)
    {
        covariance_setup(true);
        pps_ok = false;
    }
    else if (strcmp(name, "missing") == 0)
    {
        publish_covariances = false;
    }
    else
    {
        stale_covariances = true;
    }
    CHECK(actuator_probe_command("clock trial"));
    cycle(true);
    CHECK(write_count == 0U && actuator_probe_writes() == 0U);
    CHECK(strstr(last_report, "denied=1") != NULL);
    return 0;
}

static int scenario_normal(void)
{
    quality_setup();
    now_ms = SC_FCW_SETTLE_AFTER_BOOT_MS;
    cycle(true); /* No command means no automatic mutation. */
    CHECK(write_count == 0U && actuator_probe_writes() == 0U);
    CHECK(strstr(last_report, "writes=0") != NULL);

    /* The adapter suppresses a second report retry at the same MCU tick. */
    now_ms++;
    CHECK(actuator_probe_command("clock trial"));
    cycle(true);
    CHECK(write_count == 0U); /* Accepted start enters the 120 s baseline. */
    const uint32_t start_ms = now_ms;
    for (unsigned second = 1U; second <= 522U; second++)
    {
        now_ms = start_ms + second * 1000U;
        cycle(true);
        if (second == 121U)
        {
            uint16_t first_step[3];
            char expected_readback[32];
            CHECK(sit5801_fcw_encode(SC_FCW_STEP_COUNTS, first_step));
            const int written =
                snprintf(expected_readback, sizeof(expected_readback), "readback=%04X%04X%04X",
                         first_step[0], first_step[1], first_step[2]);
            CHECK(written > 0 && (size_t)written < sizeof(expected_readback));
            CHECK(strstr(last_report, expected_readback) != NULL);
        }
    }
    CHECK(write_count == 126U && actuator_probe_writes() == 42U);
    CHECK(oscillator_words[0] == 0U && oscillator_words[1] == 0U && oscillator_words[2] == 0U);
    CHECK(strstr(last_report, "phase=6") != NULL && strstr(last_report, "pending=0") != NULL);
    CHECK(strstr(last_report, "readback=000000000000") != NULL);
    CHECK(strstr(last_report, "read_0c_ms=") != NULL && report_count != 0U);

    for (unsigned group = 0U; group < 21U; group++)
    {
        const int64_t q = (int64_t)(group + 1U) * SC_FCW_STEP_COUNTS;
        CHECK(check_group(group, q > SC_FCW_TEST_COUNTS ? SC_FCW_TEST_COUNTS : q,
                          start_ms + 120000U + group * 1000U) == 0);
    }
    for (unsigned group = 0U; group < 21U; group++)
    {
        const int64_t q = SC_FCW_TEST_COUNTS - (int64_t)(group + 1U) * SC_FCW_STEP_COUNTS;
        CHECK(check_group(21U + group, q > 0 ? q : 0, start_ms + 321000U + group * 1000U) == 0);
    }
    CHECK(sit5801_fcw_decode(oscillator_words) == 0);
    return 0;
}

static int scenario_abort(void)
{
    quality_setup();
    now_ms = SC_FCW_SETTLE_AFTER_BOOT_MS;
    CHECK(actuator_probe_command("clock trial"));
    cycle(true);
    for (unsigned second = 1U; second <= 125U; second++)
    {
        now_ms += 1000U;
        cycle(true);
    }
    CHECK(actuator_probe_writes() == 6U);
    actuator_probe_disconnect(); /* A live experiment survives USB loss. */
    CHECK(actuator_probe_command("clock abort"));
    now_ms++;
    cycle(true);
    for (unsigned second = 1U; second <= 187U; second++)
    {
        now_ms += 1000U;
        cycle(true);
    }
    CHECK(oscillator_words[0] == 0U && oscillator_words[1] == 0U && oscillator_words[2] == 0U);
    CHECK(strstr(last_report, "phase=7") != NULL && strstr(last_report, "aborted=1") != NULL);
    return 0;
}

static int scenario_disconnect(void)
{
    quality_setup();
    now_ms = SC_FCW_SETTLE_AFTER_BOOT_MS;
    CHECK(actuator_probe_command("clock trial"));
    actuator_probe_disconnect();
    cycle(true);
    CHECK(write_count == 0U && actuator_probe_writes() == 0U);
    CHECK(strstr(last_report, "denied=0") != NULL);
    return 0;
}

static int scenario_partial(void)
{
    quality_setup();
    now_ms = SC_FCW_SETTLE_AFTER_BOOT_MS;
    CHECK(actuator_probe_command("clock trial"));
    cycle(true);
    fail_register = 0x0d;
    now_ms += 120000U;
    cycle(true);
    CHECK(write_count == 2U && writes[0].reg == 0x0cU && writes[1].reg == 0x0dU);
    CHECK(strstr(last_report, "phase=8") != NULL && strstr(last_report, "write_ok_mask=1") != NULL);
    const unsigned before = write_count;
    now_ms += 10000U;
    cycle(true);
    CHECK(write_count == before && actuator_probe_writes() == 0U);
    return 0;
}

static int scenario_readback(void)
{
    quality_setup();
    now_ms = SC_FCW_SETTLE_AFTER_BOOT_MS;
    CHECK(actuator_probe_command("clock trial"));
    cycle(true);
    now_ms += 120000U;
    cycle(true);
    CHECK(write_count == 3U);
    now_ms += 1000U;
    cycle(false); /* The first post-write read is incomplete and cannot advance. */
    CHECK(write_count == 3U && strstr(last_report, "phase=8") != NULL);
    return 0;
}

static int scenario_usb(void)
{
    quality_setup();
    now_ms = SC_FCW_SETTLE_AFTER_BOOT_MS;
    CHECK(actuator_probe_command("clock trial"));
    usb_blocked = true;
    cycle(true); /* The first report remains queued while CDC is unavailable. */
    now_ms += 1000U;
    cycle(true); /* Replacing that evidence aborts the active experiment. */
    usb_blocked = false;
    actuator_probe_service(now_ms + 1U);
    CHECK(write_count == 0U && actuator_probe_writes() == 0U);
    CHECK(strstr(last_report, "tx_drops=1") != NULL && strstr(last_report, "aborted=1") != NULL);
    return 0;
}

int main(int argc, char **argv)
{
#ifdef _MSC_VER
    /* A failed fixture abort()s; report through the exit code, not a modal
     * Debug CRT/Windows Error Reporting dialog that blocks unattended ctest. */
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
    if (argc != 2)
    {
        fprintf(stderr, "scenario required: normal before missing stale_cov unqualified abort "
                        "disconnect partial readback usb\n");
        return 2;
    }
    const char *scenario = argv[1];
    if (strcmp(scenario, "normal") == 0)
    {
        return scenario_normal();
    }
    if (strcmp(scenario, "before") == 0 || strcmp(scenario, "missing") == 0 ||
        strcmp(scenario, "stale_cov") == 0 || strcmp(scenario, "unqualified") == 0)
    {
        return scenario_denied(scenario);
    }
    if (strcmp(scenario, "abort") == 0)
    {
        return scenario_abort();
    }
    if (strcmp(scenario, "disconnect") == 0)
    {
        return scenario_disconnect();
    }
    if (strcmp(scenario, "partial") == 0)
    {
        return scenario_partial();
    }
    if (strcmp(scenario, "readback") == 0)
    {
        return scenario_readback();
    }
    if (strcmp(scenario, "usb") == 0)
    {
        return scenario_usb();
    }
    fprintf(stderr, "unknown scenario: %s\n", scenario);
    return 2;
}
