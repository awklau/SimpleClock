/* SPDX-License-Identifier: MIT */
/* Foreground scheduler for the assembled-board diagnostic.
 * GPIO safety precedes acquisition; USB backpressure never gates watchdog/heartbeat. */
#include "clock_probe.h"
#include "main.h"
#include "simpleclock/sit5801.h"
#include "simpleclock/text.h"
#include "gnss_probe.h"
#include "timing_probe.h"
#ifdef SIMPLECLOCK_ACTUATOR_TEST
#include "actuator_probe.h"
#endif
#ifdef SIMPLECLOCK_AUTO_HYBRID
#include "hybrid_probe.h"
#include "simpleclock/status_leds.h"
#endif
#if defined(SIMPLECLOCK_ACTUATOR_TEST) || defined(SIMPLECLOCK_AUTO_HYBRID)
static uint32_t fcw_read_ms[3];
#endif
#ifdef SIMPLECLOCK_THERMAL_PROBE
#include "thermal_probe.h"
#include "simpleclock/status_leds.h"
static sc_led_modes_t led_modes;
#endif
#include <stddef.h>

extern uint32_t __StackLimit[], __StackTop[], __static_end[];
#define STACK_PATTERN UINT32_C(0xa5c39e71)
static sit5801_probe_t probe;
static uint32_t last_cycle_ms, last_resource_ms, last_service_ms, max_gap_ms, last_gnss_ms;
static uint32_t reset_flags;
static bool faulted;

#ifdef SIMPLECLOCK_THERMAL_PROBE
static void update_leds(uint32_t now_ms)
{
    bool thermal_available, thermal_stable, thermal_fault;
    thermal_probe_led_status(now_ms, &thermal_available, &thermal_stable, &thermal_fault);
    /* LED2 follows the G5-T's fresh valid PPP mode directly, per user policy.
     * Full READY_TO_RECORD remains unqualified: a mode bit is not calibration. */
#ifdef SIMPLECLOCK_AUTO_HYBRID
    led_modes = sc_led_independent_modes(
        sc_hybrid_frequency_lit(hybrid_probe_state(), now_ms), timing_probe_fix_valid(now_ms),
        timing_probe_ppp_active(now_ms), thermal_available, thermal_stable,
        faulted || thermal_fault || hybrid_probe_faulted());
#else
    led_modes = sc_led_modes(faulted || thermal_fault, false, timing_probe_fix_valid(now_ms),
                             timing_probe_ppp_active(now_ms), thermal_available, thermal_stable);
#endif
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8,
                      sc_led_on(led_modes.system, now_ms) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,
                      sc_led_on(led_modes.gnss, now_ms) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,
                      sc_led_on(led_modes.thermal, now_ms) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
#endif

void clock_probe_early_gpio(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /* TSSOP20 pad 20 is multibonded. Program unused pads as analog; only PB3
     * becomes an output, with its latch already LOW. No PWM/heat command. */
    GPIOB->BRR = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &gpio);
#ifdef SIMPLECLOCK_THERMAL_PROBE
    /* RM0490 9.1.3, PINMUX2: reset selects PB6, even if PB3 MODER says output.
     * SECURE_MUXING_EN=1 forces unselected pads passive. Select PB3 while all
     * candidates are analog and latched low; preserve other muxes/option bytes. */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    HAL_SYSCFG_SetPinBinding(HAL_BIND_TSSOP20_PIN20_PB3);
#endif
    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    /* Enable receiver reference and PPS distribution before any acquisition. */
    GPIOA->BSRR = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOA, &gpio);
    GPIOC->BRR = GPIO_PIN_14 | GPIO_PIN_15;
    gpio.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOC, &gpio);
    /* PA4 receiver reset remains high impedance. Never reset a running G5. */
}

void clock_probe_fault(void)
{
#ifdef SIMPLECLOCK_THERMAL_PROBE
    thermal_probe_fault();
#endif
    GPIOB->BRR = GPIO_PIN_3;
    faulted = true;
}

static bool guard_ok(void)
{
    const volatile uint32_t *guard = __StackLimit;
    return guard[0] == STACK_PATTERN && guard[1] == STACK_PATTERN && guard[2] == STACK_PATTERN &&
           guard[3] == STACK_PATTERN;
}

void clock_probe_init(void)
{
    /* Called before other application work. Leave active frames + 32 bytes
     * untouched. This conservative high-water includes that initial margin. */
    const uintptr_t fill_end = (uintptr_t)__get_MSP() - 32U;
    for (uintptr_t addr = (uintptr_t)__StackLimit; addr < fill_end; addr += 4U)
    {
        *(volatile uint32_t *)addr = STACK_PATTERN;
    }
    reset_flags = RCC->CSR2;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    last_service_ms = HAL_GetTick();
    /* Independent watchdog: nominal 2.008 s at 32 kHz LSI (not measured).
     * No option-byte changes; reset leaves Q1 off via its external pulldown. */
    IWDG->KR = 0xccccU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 6U;
    IWDG->RLR = 250U;
    const uint32_t start = HAL_GetTick();
    while (IWDG->SR != 0U)
    {
        if ((uint32_t)(HAL_GetTick() - start) > 100U)
        {
            clock_probe_fault();
            return;
        }
    }
    IWDG->KR = 0xaaaaU;
    last_service_ms = HAL_GetTick();
    gnss_probe_init();
    timing_probe_init();
#ifdef SIMPLECLOCK_AUTO_HYBRID
    hybrid_probe_init();
#endif
}

static void send_line(sc_text_t *text)
{
    sc_text_string(text, "\r\n");
    if (text->ok)
    {
        (void)simpleclock_usb_cdc_try_send((const uint8_t *)text->data, (uint16_t)text->length);
    }
}

static void append_u32_field(sc_text_t *text, const char *key, uint32_t value)
{
    sc_text_string(text, key);
    sc_text_u32(text, value);
}

static void report_probe(uint32_t now_ms)
{
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
    sc_text_string(&text, "type=ocxo_probe v=1");
    append_u32_field(&text, " uptime_ms=", now_ms);
    append_u32_field(&text, " seq=", probe.sequence);
    append_u32_field(&text, " valid_mask=", probe.valid_mask);
    append_u32_field(&text, " wire_order=", sit5801_probe_word_order(&probe));
    append_u32_field(&text, " errors=", probe.errors);
    static const char *const keys[8] = {
        " id=", " clip=", " oe=", " fcw_m=", " fcw_n=", " fcw_l=", " power_h=", " power_l="};
    for (unsigned i = 0; i < 8U; i++)
    {
        sc_text_string(&text, keys[i]);
        sc_text_hex(&text, probe.wire[i], 4);
    }
    sc_text_string(&text, " power_atomic=0");
#ifdef SIMPLECLOCK_ACTUATOR_TEST
    append_u32_field(&text, " writes=", actuator_probe_writes());
#elif defined(SIMPLECLOCK_AUTO_HYBRID)
    append_u32_field(&text, " writes=", hybrid_probe_writes());
#else
    sc_text_string(&text, " writes=0");
#endif
    sc_text_string(&text, " stable=unknown");
    send_line(&text);
}

static void report_resources(uint32_t now_ms)
{
    uintptr_t first_used = (uintptr_t)__StackLimit;
    while (first_used < (uintptr_t)__StackTop && *(volatile uint32_t *)first_used == STACK_PATTERN)
    {
        first_used += 4U;
    }
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
#ifdef SIMPLECLOCK_AUTO_HYBRID
    sc_text_string(&text, "type=resources v=1 profile=AUTO_HYBRID_V1");
#elif defined(SIMPLECLOCK_ACTUATOR_TEST)
    sc_text_string(&text, "type=resources v=1 profile=M6_FCW_TRIAL");
#elif defined(SIMPLECLOCK_THERMAL_PROBE)
    sc_text_string(&text, "type=resources v=1 profile=M5_THERMAL_PI");
#else
    sc_text_string(&text, "type=resources v=1 profile=M3_PPS_E6_PROBE");
#endif
    append_u32_field(&text, " uptime_ms=", now_ms);
    append_u32_field(&text, " core_hz=", SystemCoreClock);
    append_u32_field(&text, " flash_kib=", *(const uint16_t *)FLASHSIZE_BASE);
    append_u32_field(&text, " static_ram=", (uint32_t)((uintptr_t)__static_end - SRAM_BASE));
    sc_text_string(&text, " stack_reserved=4096");
    append_u32_field(&text, " stack_high_water=", (uint32_t)((uintptr_t)__StackTop - first_used));
    append_u32_field(&text, " guard=", guard_ok() ? 1U : 0U);
    append_u32_field(&text, " loop_gap_ms=", max_gap_ms);
    sc_text_string(&text, " reset_flags=");
    sc_text_hex(&text, reset_flags, 8);
#ifdef SIMPLECLOCK_THERMAL_PROBE
    sc_text_string(&text,
                   " heater=AUTO_PI_OR_TRIALS buffers=11 gnss=BOUNDED_COMMANDS pps=TIM2_CAPTURE "
#ifdef SIMPLECLOCK_AUTO_HYBRID
                   "discipline=AUTO_HYBRID calibrated=0");
#else
                   "discipline=OFF calibrated=0");
#endif
    sc_text_string(&text, " led1=");
    sc_text_string(&text, sc_led_mode_name(led_modes.system));
    sc_text_string(&text, " led2=");
    sc_text_string(&text, sc_led_mode_name(led_modes.gnss));
    sc_text_string(&text, " led3=");
    sc_text_string(&text, sc_led_mode_name(led_modes.thermal));
    /* LED3 reports regulation, not the historical strict preheat diagnostic.
     * Keep ready_to_record=0 as the legacy commissioning marker; it neither
     * disables passive logging nor describes the autonomous frequency servo. */
    sc_text_string(&text, " led3_basis=TEMPERATURE_REGULATION ready_to_record=0");
#ifdef SIMPLECLOCK_AUTO_HYBRID
    sc_text_string(&text, " led1_basis=FREQUENCY_SETTLED");
    append_u32_field(&text, " led1_holdover_grace=", 0U);
    append_u32_field(&text, " led1_holdover_ms=", 0U);
#endif
#else
    sc_text_string(&text, " heater=DISARMED buffers=11 gnss=BOUNDED_COMMANDS pps=TIM2_CAPTURE "
                          "discipline=OFF calibrated=0");
#endif
    send_line(&text);
}

void clock_probe_service(uint32_t now_ms)
{
    const uint32_t gap = now_ms - last_service_ms;
    last_service_ms = now_ms;
    if (gap > max_gap_ms)
    {
        max_gap_ms = gap;
    }
    if (gap > 500U || !guard_ok())
    {
        clock_probe_fault();
    }
    const uint32_t phase = now_ms % 1000U;
#ifndef SIMPLECLOCK_THERMAL_PROBE
    const bool led = faulted ? (phase < 100U || (phase >= 200U && phase < 300U)) : phase < 500U;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, led ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
    if (faulted)
    {
#ifdef SIMPLECLOCK_THERMAL_PROBE
        update_leds(now_ms);
#endif
        return; /* Do not feed from an ISR or after a health failure. */
    }
    gnss_probe_service(now_ms);
    timing_probe_service(HAL_GetTick());
#ifdef SIMPLECLOCK_ACTUATOR_TEST
    actuator_probe_service(HAL_GetTick());
#endif
#ifdef SIMPLECLOCK_AUTO_HYBRID
    hybrid_probe_service(HAL_GetTick());
#endif

    /* One bounded transaction per call. Pace from completion, including errors.
     * A full eight-read snapshot is emitted once/s; no auto increment assumed. */
    if (probe.index != 0U || (uint32_t)(now_ms - last_cycle_ms) >= 1000U)
    {
        uint8_t reg;
        if (sit5801_probe_begin(&probe, now_ms, &reg))
        {
            if (probe.index == 0U)
            {
                last_cycle_ms = now_ms;
            }
            uint8_t bytes[2] = {0};
            const HAL_StatusTypeDef status =
#ifdef SIMPLECLOCK_THERMAL_PROBE
                (I2C2->ISR & I2C_ISR_BUSY) ? HAL_ERROR :
#endif
                                           HAL_I2C_Mem_Read(&hi2c2, SIT5801_ADDRESS_7BIT << 1U, reg,
                                                            I2C_MEMADD_SIZE_8BIT, bytes, 2U, 5U);
            sit5801_probe_complete(&probe, HAL_GetTick(), status == HAL_OK, bytes);
#if defined(SIMPLECLOCK_ACTUATOR_TEST) || defined(SIMPLECLOCK_AUTO_HYBRID)
            if (reg >= 0x0cU && reg <= 0x0eU)
            {
                fcw_read_ms[reg - 0x0cU] = HAL_GetTick();
            }
#endif
            if (probe.index == 0U)
            {
#ifdef SIMPLECLOCK_THERMAL_PROBE
                thermal_probe_ocxo(&probe, HAL_GetTick());
#endif
                report_probe(HAL_GetTick());
#ifdef SIMPLECLOCK_ACTUATOR_TEST
                actuator_probe_cycle(&probe, fcw_read_ms, HAL_GetTick());
#endif
#ifdef SIMPLECLOCK_AUTO_HYBRID
                hybrid_probe_cycle(&probe, fcw_read_ms, HAL_GetTick());
#endif
            }
#ifdef SIMPLECLOCK_THERMAL_PROBE
            gnss_probe_service(HAL_GetTick());
#endif
        }
    }
#ifdef SIMPLECLOCK_THERMAL_PROBE
    update_leds(HAL_GetTick());
#endif
    if ((uint32_t)(now_ms - last_resource_ms) >= 1000U && phase >= 700U)
    {
        last_resource_ms = now_ms;
        report_resources(now_ms);
    }
    if ((uint32_t)(now_ms - last_gnss_ms) >= 1000U && phase >= 960U)
    {
        last_gnss_ms = now_ms;
        gnss_probe_report(now_ms);
    }
    IWDG->KR = 0xaaaaU;
}
