/* SPDX-License-Identifier: MIT */
/* Exercise the actual M4 adapter with virtual time, fake bus and blocked USB. */
#include "main.h"
#include "telemetry_check.h"
#include "thermal_probe.h"
#include "gnss_probe.h"
#ifdef SIMPLECLOCK_AUTO_HYBRID
#include "simpleclock/hybrid_defaults.h"
#endif
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
GPIO_TypeDef test_gpio_b;
I2C_TypeDef test_i2c2;
static uint32_t now_ms, irq_mask, gnss_profile, conversions, bus_calls;
static bool ready, usb_blocked, bus_failed, converting, suppress_conversion;
static uint32_t trigger_ms;
static uint16_t sensor_config = 0x0220;
static uint16_t sensor_raw = 3200;
static bool ocxo_missing;
static char last_reply[256], last_status[SIMPLECLOCK_TMP117_USB_TX_MAX];
static char last_pi_status[SIMPLECLOCK_TMP117_USB_TX_MAX];
#ifdef SIMPLECLOCK_AUTO_HYBRID
static char last_boot_status[384];
#endif
static unsigned replies, protocol_failures;
static bool pin_on;
static uint32_t high_ms;
static uint32_t pin_binding = 0x00300010U;
#ifdef SIMPLECLOCK_ACTUATOR_TEST
bool actuator_probe_command(const char *line)
{
    (void)line;
    return false;
}

void actuator_probe_disconnect(void) {}
#endif
#ifdef SIMPLECLOCK_AUTO_HYBRID
bool hybrid_probe_command(const char *line)
{
    (void)line;
    return false;
}

void hybrid_probe_disconnect(void) {}

static unsigned manual_hold_requests;

void hybrid_probe_manual_request(void)
{
    manual_hold_requests++;
}
#endif

uint32_t HAL_SYSCFG_GetPinBinding(uint32_t source)
{
    if (source != 0x30U)
    {
        protocol_failures++;
    }
    return pin_binding;
}

uint32_t HAL_GetTick(void)
{
    return now_ms;
}

uint32_t __get_PRIMASK(void)
{
    return irq_mask;
}

void __disable_irq(void)
{
    irq_mask = 1;
}

void __set_PRIMASK(uint32_t mask)
{
    irq_mask = mask;
}

void __DMB(void) {}

bool gnss_probe_control(const char *line)
{
    (void)line;
    return false;
}

void gnss_probe_request(uint32_t profile)
{
    gnss_profile = profile;
}

HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address, uint16_t reg,
                                   uint16_t address_size, uint8_t *data, uint16_t size,
                                   uint32_t timeout)
{
    bus_calls++;
    if (bus != &hi2c2 || address != 0x90 || address_size != 1 || size != 2 || timeout != 5 ||
        irq_mask)
    {
        protocol_failures++;
        return HAL_ERROR;
    }
    if (bus_failed)
    {
        return HAL_ERROR;
    }
    uint16_t value = 0;
    if (reg == 15)
    {
        value = 0x0117;
    }
    else if (reg == 1)
    {
        if (converting)
        {
            protocol_failures++; /* No clear-on-read flag polling during conversion. */
        }
        value = (uint16_t)(sensor_config | (ready ? 0x2000U : 0U));
        ready = false;
    }
    else if (reg == 4)
    {
        value = 0; /* EEPROM locked, not busy. */
    }
    else if (reg == 0)
    {
        value = sensor_raw;
        ready = false;
        conversions++;
    }
    else
    {
        protocol_failures++;
        return HAL_ERROR;
    }
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address, uint16_t reg,
                                    uint16_t address_size, uint8_t *data, uint16_t size,
                                    uint32_t timeout)
{
    bus_calls++;
    if (bus != &hi2c2 || address != 0x90 || address_size != 1 || size != 2 || timeout != 5 ||
        irq_mask || reg != 1)
    {
        protocol_failures++;
        return HAL_ERROR;
    }
    if (bus_failed)
    {
        return HAL_ERROR;
    }
    const uint16_t value = (uint16_t)((uint16_t)data[0] << 8 | data[1]);
    if (value != 0x0620 && value != 0x0e20)
    {
        protocol_failures++;
        return HAL_ERROR;
    }
    sensor_config = value;
    converting = value == 0x0e20;
    trigger_ms = now_ms;
    return HAL_OK;
}

static bool capture(const uint8_t *data, uint16_t length)
{
    telemetry_check(data, length);
    if (irq_mask)
    {
        protocol_failures++;
    }
    if (usb_blocked)
    {
        return false;
    }
    if (length >= 21 && memcmp(data, "type=thermal_command ", 21) == 0)
    {
        if (length >= sizeof(last_reply))
        {
            protocol_failures++;
            return false;
        }
        memcpy(last_reply, data, length);
        last_reply[length] = 0;
        replies++;
    }
    else if (length >= 16 && memcmp(data, "type=thermal_pi ", 16) == 0)
    {
        if (length >= sizeof(last_pi_status))
        {
            return false;
        }
        memcpy(last_pi_status, data, length);
        last_pi_status[length] = 0;
    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
    else if (length >= 18U && memcmp(data, "type=thermal_boot ", 18U) == 0)
    {
        if (length >= sizeof(last_boot_status))
        {
            return false;
        }
        memcpy(last_boot_status, data, length);
        last_boot_status[length] = 0;
    }
#endif
    else
    {
        if (length >= sizeof(last_status))
        {
            protocol_failures++;
            return false;
        }
        memcpy(last_status, data, length);
        last_status[length] = 0;
    }
    return true;
}

bool simpleclock_usb_cdc_try_send(const uint8_t *data, uint16_t length)
{
    return capture(data, length);
}

bool simpleclock_usb_cdc_try_send_queued(const uint8_t *data, uint16_t length)
{
    return capture(data, length);
}

static int advance(uint32_t milliseconds)
{
    while (milliseconds--)
    {
        now_ms++;
        if (test_gpio_b.BRR & 8U)
        {
            pin_on = false;
        }
        test_gpio_b.ODR = test_gpio_b.IDR = pin_on ? 8U : 0U;
        test_gpio_b.BRR = test_gpio_b.BSRR = 0;
        if (converting && (uint32_t)(now_ms - trigger_ms) >= 140U)
        {
            converting = false;
            /* Completion powers down the ADC; it does not rewrite MOD. */
            ready = !suppress_conversion;
        }
        thermal_probe_tick(now_ms);
        if (!ocxo_missing && now_ms % 1000U == 0)
        {
            const sit5801_probe_t probe = {.valid_mask = 255,
                                           .wire = {0xd012, 32, 0, 0, 0, 0, 2500, 0}};
            thermal_probe_ocxo(&probe, now_ms);
        }
        const uint32_t before = bus_calls;
        thermal_probe_service(now_ms);
        CHECK(bus_calls - before <= 1U); /* Bounded service, never a burst of transfers. */
        CHECK((test_gpio_b.BSRR & ~(8U | (8U << 16))) == 0 && irq_mask == 0);
        if (test_gpio_b.BSRR & 8U)
        {
            pin_on = true;
        }
        if ((test_gpio_b.BSRR & (8U << 16)) || (test_gpio_b.BRR & 8U))
        {
            pin_on = false;
        }
        if (pin_on)
        {
            high_ms++;
        }
    }
    return 0;
}

static void send(const char *line)
{
    thermal_probe_receive((const uint8_t *)line, (uint32_t)strlen(line));
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
static int32_t q16_to_milli(int32_t value_q16)
{
    const int64_t scaled = (int64_t)value_q16 * 1000;
    return scaled >= 0 ? (int32_t)((scaled + 32768) / 65536)
                       : -(int32_t)((-scaled + 32768) / 65536);
}

static bool line_has_q16_field(const char *line, const char *key, int32_t value_q16)
{
    const int32_t milli = q16_to_milli(value_q16);
    int32_t whole = milli / 1000;
    int32_t fraction = milli % 1000;
    if (fraction < 0)
    {
        fraction = -fraction;
    }
    char expected[80];
    (void)snprintf(expected, sizeof(expected), "%s%d.%03d", key, (int)whole, (int)fraction);
    return strstr(line, expected) != NULL;
}

static int hybrid_startup_test(const char *scenario)
{
    const bool low_clamp = strcmp(scenario, "low_clamp") == 0;
    const bool high_clamp = strcmp(scenario, "high_clamp") == 0;
    const bool nohost = strcmp(scenario, "nohost") == 0;
    const bool baseline = strcmp(scenario, "baseline") == 0;
    const bool trip70 = strcmp(scenario, "trip70") == 0;
    const bool decay = strcmp(scenario, "decay") == 0;
    const bool cancel = strcmp(scenario, "cancel") == 0;
    const bool continuous = strcmp(scenario, "continuous") == 0;
    const bool storage_inhibit = strcmp(scenario, "storage_inhibit") == 0;
    if (!low_clamp && !high_clamp && !nohost && !baseline && !trip70 && !decay && !cancel &&
        !continuous && !storage_inhibit)
    {
        return -1;
    }
    now_ms = 0U;
    if (trip70)
    {
        sensor_raw = (uint16_t)(70 * 128);
        thermal_probe_init(now_ms);
        CHECK(advance(3000U) == 0 && high_ms == 0U);
        CHECK(strstr(last_status, "mode=FAULT") && strstr(last_status, "auto_state=FAULT"));
        return 0;
    }
    const int32_t baseline_c = low_clamp ? -2 : high_clamp ? 62 : 37;
    sensor_raw = (uint16_t)(int16_t)(baseline_c * 128);
    thermal_probe_init(now_ms);
    CHECK(advance(3000U) == 0);
    if (high_clamp)
    {
        CHECK(high_ms == 0U);
    }
    else
    {
        CHECK(high_ms > 0U && strstr(last_status, "auto_state=BOOST"));
    }
    if (nohost)
    {
        thermal_probe_disconnect();
        usb_blocked = true;
    }
    if (cancel)
    {
        send("thermal off\n");
        CHECK(advance(40000U) == 0 && !pin_on);
        CHECK(strstr(last_status, "auto_state=CANCELED"));
        return 0;
    }
    if (storage_inhibit)
    {
        const uint32_t before = high_ms;
        thermal_probe_storage_inhibit(true);
        CHECK(test_gpio_b.BRR == 8U);
        CHECK(advance(50U) == 0 && high_ms == before && !pin_on);
        thermal_probe_storage_inhibit(false);
        CHECK(advance(10U) == 0 && high_ms > before); /* Boost resumes normally. */
    }
    if (decay)
    {
        /* Full heater power with an imposed 2-code/s initial fit, then 1 code
         * every two seconds (0.234 C/min, below the fixed 0.4 C/min limit).
         * This exercises the real adapter and frame transition. */
        while (now_ms < 93000U)
        {
            sensor_raw += 2U;
            CHECK(advance(1000U) == 0);
        }
        CHECK(strstr(last_status, "auto_state=BOOST"));
        while (now_ms < 150000U)
        {
            sensor_raw += (now_ms / 1000U) % 2U;
            CHECK(advance(1000U) == 0);
        }
        CHECK(strstr(last_boot_status, "phase=CAPTURED"));
        CHECK(strstr(last_status, "auto_state=ACTIVE"));
        CHECK(!strstr(last_pi_status, "integral_micro=1000000"));
        return 0;
    }
    CHECK(advance(94000U) == 0);
    usb_blocked = false;
    CHECK(advance(5000U) == 0);
    CHECK(strstr(last_status, "mode=PI") && strstr(last_status, "continuous=1"));
    CHECK(strstr(last_status, "auto_state=ACTIVE") && strstr(last_status, "faults=0"));
    CHECK(strstr(last_pi_status, "kp_micro=2600000 ki_micro=16000"));
    CHECK(strstr(last_pi_status, "controller=PID kd_micro=16900000 d_filter_ms=3000"));
    CHECK(strstr(last_pi_status, "auto_start_policy=FULL_POWER_SLOPE_CAPTURE"));
    CHECK(strstr(last_boot_status, high_clamp ? "phase=CEILING" : "phase=NO_RISE"));
    const int32_t target_c = low_clamp ? 0 : high_clamp ? 60 : baseline_c;
    CHECK(line_has_q16_field(last_pi_status, "target_c=", target_c * 65536L));
    CHECK(line_has_q16_field(last_status, "trip_c=", 70L * 65536L));
    if (storage_inhibit)
    {
        CHECK(thermal_probe_set_target((baseline_c + 1) * 65536L));
        const uint32_t before_target = high_ms;
        CHECK(advance(2000U) == 0 && high_ms > before_target);
        const uint32_t before = high_ms;
        thermal_probe_storage_inhibit(true);
        CHECK(test_gpio_b.BRR == 8U);
        CHECK(advance(50U) == 0 && high_ms == before && !pin_on);
        thermal_probe_snapshot_t snapshot;
        thermal_probe_snapshot(now_ms, &snapshot);
        CHECK(snapshot.running && snapshot.safe && !snapshot.completed_frame_valid);
        thermal_probe_storage_inhibit(false);
        CHECK(advance(2000U) == 0 && high_ms > before);
        send("thermal off\n");
        thermal_probe_storage_inhibit(true);
        thermal_probe_storage_inhibit(false);
        CHECK(advance(1U) == 0 && !pin_on); /* OFF still dominates release. */
    }
    if (continuous)
    {
        send("thermal off\n");
        CHECK(advance(1000U) == 0);
        send("thermal pi 37.000 5\n");
        CHECK(advance(2000U) == 0 && strstr(last_status, "continuous=0"));
        const unsigned holds_before_continue = manual_hold_requests;
        send("thermal continue\n");
        CHECK(advance(15000U) == 0 && strstr(last_status, "continuous=1"));
        CHECK(manual_hold_requests == holds_before_continue);
        CHECK(strstr(last_status, "mode=PI"));
        send("thermal off\n");
        CHECK(advance(1000U) == 0);
        send("thermal pi 37.000\n");
        CHECK(advance(1802000U) == 0 && strstr(last_status, "continuous=1"));
        CHECK(strstr(last_status, "mode=PI") && strstr(last_status, "faults=0"));
        bool available, stable, fault;
        thermal_probe_led_status(now_ms, &available, &stable, &fault);
        CHECK(available && stable && !fault); /* Zero duty is not an LED3 veto. */
        send("thermal off\n");
        CHECK(advance(1U) == 0 && !pin_on);
    }
    CHECK(protocol_failures == 0);
    return 0;
}
#endif

static int automatic_test(const char *scenario)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    const int hybrid_result = hybrid_startup_test(scenario);
    if (hybrid_result >= 0)
    {
        return hybrid_result;
    }
#endif
#ifdef SIMPLECLOCK_ACTUATOR_TEST
    if (strcmp(scenario, "fixed") == 0)
    {
        now_ms = 0;
        sensor_raw = 37 * 128;
        thermal_probe_init(now_ms);
        CHECK(advance(299999) == 0 && high_ms == 0);
        CHECK(!thermal_probe_trial_ready(now_ms, true));
        CHECK(advance(3001) == 0);
        CHECK(strstr(last_status, "mode=PI") && strstr(last_status, "continuous=1"));
        CHECK(strstr(last_pi_status, "target_c=39.399"));
        CHECK(strstr(last_pi_status, "kp_micro=1000000 ki_micro=4000"));
        CHECK(thermal_probe_trial_ready(now_ms, false));
        sensor_raw = 38 * 128;
        CHECK(advance(20000) == 0);
        CHECK(strstr(last_pi_status, "target_c=39.399"));
        send("thermal off\n");
        CHECK(advance(1) == 0 && !thermal_probe_trial_ready(now_ms, false));
        CHECK(protocol_failures == 0);
        return 0;
    }
#endif
    const bool wrap = strcmp(scenario, "wrap") == 0;
    const bool range = strcmp(scenario, "range") == 0;
    const bool missing = strcmp(scenario, "missing") == 0;
    const bool manual = strcmp(scenario, "manual") == 0;
    const bool canceled = strcmp(scenario, "off") == 0;
    const bool stop = strcmp(scenario, "stop") == 0;
    const bool boot_fault = strcmp(scenario, "fault") == 0;
    const bool ceiling = strcmp(scenario, "ceiling") == 0;
    /* Separate CTest processes provide genuine power-on zero-initialization. */
    now_ms = wrap ? UINT32_MAX - 310000U : 0U;
    sensor_raw = boot_fault ? 50 * 128 : (range ? 44 * 128 : (ceiling ? 43 * 128 : 25 * 128));
    ocxo_missing = missing;
    thermal_probe_init(now_ms);
    if (boot_fault)
    {
        CHECK(advance(3000) == 0 && high_ms == 0);
        CHECK(strstr(last_status, "auto_state=FAULT"));
        sensor_raw = 25 * 128;
        CHECK(advance(3000) == 0);
        send("fault clear\n");
        CHECK(advance(1) == 0 && strstr(last_reply, "result=OK"));
        CHECK(advance(293998) == 0);
    }
    else if (manual)
    {
        CHECK(advance(121000) == 0);
        send("thermal manual 0.1 1\n");
        CHECK(advance(1) == 0 && strstr(last_reply, "result=OK"));
        CHECK(advance(178998) == 0);
    }
    else
    {
        CHECK(advance(299999) == 0 && high_ms == 0);
        if (strcmp(scenario, "normal") == 0)
        {
            /* Core parser now accepts 60 C, but legacy M5/M6 profiles remain
             * capped at 45 C; reject before taking actuator ownership. */
            send("thermal pi 60 60\n");
            CHECK(advance(1) == 0 && strstr(last_reply, "result=RANGE") && high_ms == 0U);
        }
    }
    CHECK(strstr(last_status, "mode=OFF"));
    if (canceled)
    {
        usb_blocked = true;
        send("thermal status\n");
        thermal_probe_service(now_ms); /* Block the reply before receiving OFF. */
        send("thermal off\n");
    }
    thermal_probe_disconnect(); /* Enumeration/host connection never gates boot PI. */
    CHECK(advance(2001) == 0);
    usb_blocked = false;
    CHECK(advance(2000) == 0);
    if (range || canceled || manual || boot_fault)
    {
        CHECK(strstr(last_status, "mode=OFF") && !pin_on);
        CHECK(strstr(last_status, range
                                      ? "auto_state=TARGET_RANGE"
                                      : (boot_fault ? "auto_state=FAULT" : "auto_state=CANCELED")));
        sensor_raw = 25 * 128;
        send("fault clear\n");
        const uint32_t before = high_ms;
        CHECK(advance(20000) == 0 && high_ms == before);
        CHECK(strstr(last_status, "mode=OFF"));
        CHECK(protocol_failures == 0);
        return 0;
    }
    if (missing)
    {
        CHECK(strstr(last_status, "mode=OFF") && strstr(last_status, "auto_state=WAITING"));
        CHECK(high_ms == 0);
        /* OCXO recovery alone cannot start with an unhealthy temperature bus. */
        bus_failed = true;
        CHECK(advance(3000) == 0);
        ocxo_missing = false;
        CHECK(advance(3000) == 0 && high_ms == 0);
        bus_failed = false;
        CHECK(advance(5000) == 0);
    }
    CHECK(strstr(last_status, "mode=PI") && strstr(last_status, "continuous=1"));
    CHECK(strstr(last_status, "auto_state=ACTIVE") && strstr(last_status, "faults=0"));
    if (!wrap && !missing)
    {
        CHECK(strstr(last_pi_status, "auto_start_ms=300000 "));
    }
    if (ceiling)
    {
        CHECK(strstr(last_pi_status, "auto_baseline_c=43.000") &&
              strstr(last_pi_status, "target_c=45.000"));
        CHECK(protocol_failures == 0);
        return 0;
    }
    CHECK(strstr(last_pi_status, "auto_baseline_c=25.000") &&
          strstr(last_pi_status, "target_c=27.000"));
    CHECK(strstr(last_pi_status, "max_permille=1000") && strstr(last_pi_status, "tx_drops=0"));
    sensor_raw = 26 * 128;
    CHECK(advance(20000) == 0 && high_ms > 0);
    CHECK(strstr(last_pi_status, "target_c=27.000")); /* Never chase the warming baseline. */
    CHECK(strstr(last_status, "faults=0")); /* Also after the wrap scenario crosses zero. */
    if (stop)
    {
        usb_blocked = true;
        send("thermal off\n");
        CHECK(test_gpio_b.BRR == 8U); /* IRQ OFF cannot wait for an acknowledgment. */
        thermal_probe_disconnect();
        const uint32_t before = high_ms;
        CHECK(advance(3000) == 0 && high_ms == before);
        usb_blocked = false;
        send("fault clear\n");
        CHECK(advance(3000) == 0 && high_ms == before);
        CHECK(strstr(last_status, "auto_state=CANCELED") && strstr(last_status, "mode=OFF"));
        CHECK(protocol_failures == 0);
        return 0;
    }
    usb_blocked = true;
    thermal_probe_disconnect();
    CHECK(advance(1801000) == 0); /* No 30-minute trial expiry in normal operation. */
    usb_blocked = false;
    CHECK(advance(2000) == 0 && strstr(last_status, "mode=PI"));
    CHECK(strstr(last_pi_status, "target_c=27.000") && strstr(last_status, "faults=0"));
    bus_failed = true;
    CHECK(advance(2000) == 0 && !pin_on && strstr(last_status, "auto_state=FAULT"));
    bus_failed = false;
    CHECK(advance(5000) == 0);
    send("fault clear\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=OK"));
    const uint32_t before = high_ms;
    CHECK(advance(5000) == 0 && high_ms == before && strstr(last_status, "mode=OFF"));
    CHECK(strstr(last_status, "auto_state=FAULT")); /* Clearance never restarts heat. */
    CHECK(protocol_failures == 0);
    return 0;
}

int main(int argc, char **argv)
{
    test_gpio_b.MODER = 1U << 6; /* Stand-in for clock_probe_early_gpio's PB3 output. */
    if (argc == 2)
    {
        return automatic_test(argv[1]);
    }
    thermal_probe_tick(0); /* Safe before initialization/USB enumeration. */
    thermal_probe_disconnect();
    thermal_probe_init(0);
    bool led_available, led_stable, led_fault;
    thermal_probe_led_status(0, &led_available, &led_stable, &led_fault);
    CHECK(!led_available && !led_stable && !led_fault);
    CHECK(advance(4000) == 0);
    CHECK(conversions >= 3 && strstr(last_status, "fresh=1") &&
          strstr(last_status, "temp_c=25.0000000"));
    CHECK(strstr(last_status, "commissioned=0") && strstr(last_status, "pin_on=0"));
    const unsigned before_reply = replies;
    send("thermal manual 0.");
    CHECK(advance(1) == 0 && replies == before_reply);
    send("1 60\r\n");
    CHECK(advance(1) == 0 && replies == before_reply + 1);
    CHECK(strstr(last_reply, "result=NOT_READY"));

    send("thermal manual 0.");
    thermal_probe_disconnect();
    send("1 60\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=BAD_COMMAND"));
    static const uint8_t embedded[] = "thermal off\0ignored\n";
    thermal_probe_receive(embedded, sizeof(embedded) - 1U);
    CHECK(advance(1) == 0 && strstr(last_reply, "result=BAD_COMMAND"));
    send("thermal manual 0.100000000000000000000000000000000000000000000 60\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=BAD_COMMAND"));
    send("gnss e6survey\n");
    CHECK(gnss_profile == 7U); /* Existing GNSS command dispatch still works. */
    const unsigned before_gnss = replies;
    send("gnss pppsur");
    CHECK(gnss_profile == GNSS_PROFILE_E6_SURVEY);
    send("vey\r\n");
    CHECK(gnss_profile == GNSS_PROFILE_PPP_SURVEY);
    send("gnss ppp\r\n");
    CHECK(gnss_profile == GNSS_PROFILE_PPP_ENABLE);
    send("gnss ppprestore\r\n");
    CHECK(gnss_profile == GNSS_PROFILE_PPP_RESTORE);
    CHECK(advance(1) == 0 && replies == before_gnss); /* Not thermal BAD_COMMAND. */
    send("gnss ppprestore extra\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=BAD_COMMAND"));
    CHECK(gnss_profile == GNSS_PROFILE_PPP_RESTORE);

    usb_blocked = true;
    send("thermal status\n");
    CHECK(advance(1) == 0);
    send("thermal manual 0.1 60\nthermal manual 0.25 300\n");
    send("thermal off\n"); /* OFF cancels a queued manual command despite full replies. */
    const uint32_t samples_before = conversions;
    CHECK(advance(4000) == 0 && conversions > samples_before);
    usb_blocked = false;
    CHECK(advance(1200) == 0 && strstr(last_reply, "result=OK"));
    CHECK(strstr(last_reply, "command_hex=746865726D616C206F6666"));
    CHECK(strstr(last_status, "command_drops=1") && strstr(last_status, "fresh=1"));

    bus_failed = true;
    CHECK(advance(2000) == 0 && strstr(last_status, "fresh=0") && strstr(last_status, "temp_c=NA"));
    bus_failed = false;
    CHECK(advance(3000) == 0 && strstr(last_status, "fresh=1"));
    test_i2c2.ISR = I2C_ISR_BUSY | I2C_ISR_BERR;
    const uint32_t calls_before_busy = bus_calls;
    CHECK(advance(2000) == 0 && strstr(last_status, "fresh=0"));
    CHECK(bus_calls == calls_before_busy && test_i2c2.ICR == I2C_ICR_BERRCF);
    test_i2c2.ISR = 0;
    CHECK(advance(3000) == 0 && strstr(last_status, "fresh=1"));
    CHECK(high_ms == 0); /* Nothing heats at boot or before the minimum warm-up. */
    CHECK(advance(119998U - now_ms) == 0);
    send("thermal manual 0.1 60\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=NOT_READY"));
    CHECK(advance(1) == 0); /* Permission opens at 120 s, never at 119.999 s. */
    CHECK(strstr(last_status, "preheat_policy=OPERATOR_MANUAL_AFTER_120S"));
    CHECK(strstr(last_status, "preheat_ready=0")); /* Permission is not qualification. */
    send("thermal manual 0.1 60\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=OK"));
    CHECK(advance(999) == 0 && pin_on);
    send("thermal off\n");
    CHECK(test_gpio_b.BRR == 8U); /* Immediate IRQ off, before the next foreground/tick. */
    CHECK(advance(1) == 0 && !pin_on);
    const uint32_t on_before_trial = high_ms;
    send("thermal manual 0.1 60\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=OK"));
    usb_blocked = true;
    CHECK(advance(61000) == 0 && !pin_on);
    CHECK(high_ms - on_before_trial == 6000U); /* 60 complete 100 ms pulses. */
    usb_blocked = false;
    CHECK(advance(1000) == 0 && strstr(last_status, "mode=OFF"));
    CHECK(advance(299998U - now_ms) == 0);
    send("thermal pi 27 60\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=NOT_READY"));
    CHECK(advance(1) == 0);
    send("thermal pi 27 60\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=OK"));
    thermal_probe_led_status(now_ms, &led_available, &led_stable, &led_fault);
    CHECK(led_available && !led_stable && !led_fault);
    CHECK(advance(10000) == 0 && strstr(last_status, "mode=PI"));
    CHECK(strstr(last_pi_status, "running=1") && strstr(last_pi_status, "target_c=27.000"));
    CHECK(strstr(last_pi_status, "max_permille=1000") && strstr(last_pi_status, "stable=0"));
    CHECK(strstr(last_pi_status, "led_band_c=0.250") && strstr(last_pi_status, "led_stable=0"));
    const uint32_t before_pi_blocked = high_ms;
    usb_blocked = true;
    CHECK(advance(10000) == 0 && high_ms > before_pi_blocked);
    send("thermal off\n");
    CHECK(test_gpio_b.BRR == 8U);
    CHECK(advance(1) == 0 && !pin_on);
    usb_blocked = false;
    CHECK(advance(1000) == 0 && strstr(last_status, "mode=OFF"));
    send("thermal pi 27 10\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=OK"));
    send("thermal pi 28 1800\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=BUSY"));
    CHECK(advance(11000) == 0 && !pin_on && strstr(last_status, "mode=OFF"));
    CHECK(strstr(last_status, "faults=0") && strstr(last_pi_status, "running=0"));
    const uint32_t before_long_pi = conversions;
    send("thermal pi 27 900\n");
    CHECK(advance(901000) == 0 && !pin_on && strstr(last_status, "mode=OFF"));
    CHECK(strstr(last_status, "faults=0") && conversions - before_long_pi == 901U);
    send("thermal pi 27 60\n");
    CHECK(advance(2000) == 0);
    suppress_conversion = true;
    CHECK(advance(2000) == 0 && !pin_on && strstr(last_status, "mode=FAULT"));
    thermal_probe_led_status(now_ms, &led_available, &led_stable, &led_fault);
    CHECK(!led_available && !led_stable && led_fault);
    suppress_conversion = false;
    CHECK(advance(3000) == 0);
    send("fault clear\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=OK") && !pin_on);
    send("thermal pi 27 60\n");
    CHECK(advance(2000) == 0);
    bus_failed = true;
    CHECK(advance(2000) == 0 && !pin_on && strstr(last_status, "mode=FAULT"));
    bus_failed = false;
    CHECK(advance(3000) == 0);
    send("fault clear\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=OK") && !pin_on);
    usb_blocked = true;
    send("thermal status\n");
    CHECK(advance(1) == 0);
    send("thermal manual 0.1 60\n");
    CHECK(advance(1500) == 0);
    usb_blocked = false;
    CHECK(advance(2) == 0 && strstr(last_reply, "result=EXPIRED"));
    CHECK(!pin_on); /* A delayed command never unexpectedly starts a trial. */
    send("thermal manual 1 60\n");
    const uint32_t before_full_duty = high_ms;
    CHECK(advance(1) == 0 && strstr(last_reply, "result=OK"));
    CHECK(advance(2000) == 0 && pin_on);
    CHECK(strstr(last_status, "pb3_mode=1") && strstr(last_status, "pb3_odr=1") &&
          strstr(last_status, "pb3_idr=1") && strstr(last_status, "max_duty_permille=1000"));
    CHECK(advance(59000) == 0 && !pin_on);
    CHECK(high_ms - before_full_duty == 60000U); /* Full duty has no 1 ms OFF gap per frame. */
    CHECK(advance(1000) == 0 && strstr(last_status, "mode=OFF"));
    send("thermal manual 1 60\n");
    CHECK(advance(2000) == 0 && pin_on && strstr(last_status, "pinmux2=1"));
    pin_binding = 0x00300000U; /* Reset binding PB6 makes PB3 MODER insufficient. */
    CHECK(advance(1) == 0 && !pin_on && test_gpio_b.BRR == 8U);
    pin_binding = 0x00300010U;
    send("fault clear\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=FAULT_LATCHED"));
    thermal_probe_fault();
    send("fault clear\n");
    CHECK(advance(1) == 0 && strstr(last_reply, "result=FAULT_LATCHED"));
    CHECK(protocol_failures == 0);
    return 0;
}
