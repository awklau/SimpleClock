/* SPDX-License-Identifier: MIT */
/* Shared CDC entry point for sensor-only, M3 receiver/clock and M5 thermal builds.
 * M5 owns TMP117 acquisition and automatic PI; other builds sample directly.
 * Sensor reads are bounded; normal USB telemetry may drop under backpressure. */

#include "main.h"

#include "simpleclock/tmp117.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_rcc_ex.h"
#include "usb_device.h"
#ifdef SIMPLECLOCK_THERMAL_PROBE
#include "thermal_probe.h"
#endif
#ifdef SIMPLECLOCK_CLOCK_PROBE
#include "clock_probe.h"
#endif

#define LED1_PIN GPIO_PIN_8
#define TMP117_I2C_ADDRESS (SIMPLECLOCK_TMP117_I2C_ADDRESS << 1U)
#define TMP117_TIMING_100KHZ_48MHZ (0x20303F5BU)

I2C_HandleTypeDef hi2c2;

typedef struct
{
    uint32_t nack;
    uint32_t bus;
    uint32_t error;
    uint32_t timeout;
} i2c_error_counters_t;

#ifndef SIMPLECLOCK_THERMAL_PROBE
static i2c_error_counters_t g_i2c_errors;
static uint32_t g_sample_sequence;
#endif
#ifndef SIMPLECLOCK_CLOCK_PROBE
static bool g_led_on;
#endif

/* The project links the vendor startup with -nostartfiles.  Newlib still
 * calls __libc_init_array(), which expects this empty init hook; no heap or
 * C++ static initialization is used by the diagnostic image. */
void _init(void) {}

static void led1_set(bool on)
{
    HAL_GPIO_WritePin(GPIOA, LED1_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void configure_gpio(void)
{
    GPIO_InitTypeDef init = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    init.Pin = LED1_PIN;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &init);
    led1_set(false);
}

static void configure_i2c2(void)
{
    hi2c2.Instance = I2C2;
    /* 48 MHz I2CCLK/APB1, Standard-mode 100 kHz, analog filter enabled. */
    hi2c2.Init.Timing = TMP117_TIMING_100KHZ_48MHZ;
    hi2c2.Init.OwnAddress1 = 0U;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0U;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
}

#ifndef SIMPLECLOCK_THERMAL_PROBE
static void record_i2c_failure(HAL_StatusTypeDef status)
{
    const uint32_t error_code = hi2c2.ErrorCode;
    const uint32_t peripheral_status = I2C2->ISR;

    /* ES0618 I2C transfer-fault handling: clear and count BERR explicitly. */
    if ((peripheral_status & I2C_ISR_BERR) != 0U)
    {
        I2C2->ICR = I2C_ICR_BERRCF;
        g_i2c_errors.bus++;
    }
    else if ((error_code & HAL_I2C_ERROR_AF) != 0U)
    {
        g_i2c_errors.nack++;
    }
    else if ((error_code & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_OVR)) != 0U)
    {
        g_i2c_errors.bus++;
    }
    else if ((status == HAL_TIMEOUT) || ((error_code & HAL_I2C_ERROR_TIMEOUT) != 0U))
    {
        g_i2c_errors.timeout++;
    }
    else
    {
        g_i2c_errors.error++;
    }
}

static bool tmp117_read_u16(uint8_t reg, uint16_t *value)
{
    uint8_t bytes[2] = {0U, 0U};
    const HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(&hi2c2, TMP117_I2C_ADDRESS, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, bytes,
                         (uint16_t)sizeof(bytes), SIMPLECLOCK_TMP117_I2C_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        record_i2c_failure(status);
        return false;
    }

    *value = simpleclock_tmp117_decode_u16_be(bytes[0], bytes[1]);
    return true;
}

static size_t append_char(char *text, size_t text_size, size_t used, char value, bool *truncated)
{
    if (text_size == 0U)
    {
        *truncated = true;
        return 0U;
    }
    if (used >= (text_size - 1U))
    {
        *truncated = true;
        text[text_size - 1U] = '\0';
        return text_size - 1U;
    }
    text[used] = value;
    used++;
    text[used] = '\0';
    return used;
}

static size_t append_literal(char *text, size_t text_size, size_t used, const char *value,
                             bool *truncated)
{
    while ((*value != '\0') && !*truncated)
    {
        used = append_char(text, text_size, used, *value, truncated);
        value++;
    }
    return used;
}

static size_t append_decimal(char *text, size_t text_size, size_t used, uint32_t value,
                             bool *truncated)
{
    char digits[10];
    size_t count = 0U;

    do
    {
        digits[count] = (char)('0' + (char)(value % 10U));
        value /= 10U;
        count++;
    } while (value != 0U);

    while (count > 0U)
    {
        count--;
        used = append_char(text, text_size, used, digits[count], truncated);
        if (*truncated)
        {
            break;
        }
    }
    return used;
}

static size_t append_hex16(char *text, size_t text_size, size_t used, uint16_t value,
                           bool *truncated)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t shift = 12U;

    while (true)
    {
        used = append_char(text, text_size, used, hex[(value >> shift) & 0x0FU], truncated);
        if (*truncated)
        {
            break;
        }
        if (shift == 0U)
        {
            break;
        }
        shift -= 4U;
    }
    return used;
}

static size_t append_temperature(char *text, size_t text_size, size_t used, int16_t raw,
                                 bool *truncated)
{
    const uint32_t magnitude = (raw < 0) ? (uint32_t)(-(int32_t)raw) : (uint32_t)raw;
    /* 1/128 C = 78125 * 1e-7 C, so this decimal is exact for every raw code. */
    const uint32_t scaled = magnitude * 78125U;
    const uint32_t whole = scaled / 10000000U;
    uint32_t fraction = scaled % 10000000U;
    uint32_t divisor = 1000000U;

    if (raw < 0)
    {
        used = append_char(text, text_size, used, '-', truncated);
    }
    if (*truncated)
    {
        return used;
    }
    used = append_decimal(text, text_size, used, whole, truncated);
    if (*truncated)
    {
        return used;
    }
    used = append_char(text, text_size, used, '.', truncated);
    while ((divisor > 0U) && !*truncated)
    {
        used =
            append_char(text, text_size, used, (char)('0' + (char)(fraction / divisor)), truncated);
        fraction %= divisor;
        divisor /= 10U;
    }
    return used;
}

static const char *sample_status(bool id_read_ok, bool device_id_valid, bool config_ok,
                                 bool temp_ok, bool data_ready, bool temp_valid)
{
    if (!id_read_ok || !config_ok || !temp_ok)
    {
        return "I2C_ERROR";
    }
    if (!device_id_valid)
    {
        return "BAD_ID";
    }
    if (!temp_valid)
    {
        return "NO_SAMPLE";
    }
    if (!data_ready)
    {
        return "NOT_READY";
    }
    return "OK";
}

static void sample_and_report(uint32_t uptime_ms)
{
    uint16_t device_id = 0U;
    uint16_t configuration = 0U;
    uint16_t temperature_encoded = 0U;
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    const bool id_ok = tmp117_read_u16(SIMPLECLOCK_TMP117_REG_DEVICE_ID, &device_id);
    const bool config_ok = tmp117_read_u16(SIMPLECLOCK_TMP117_REG_CONFIGURATION, &configuration);
    const bool temp_ok = tmp117_read_u16(SIMPLECLOCK_TMP117_REG_TEMPERATURE, &temperature_encoded);
    const int16_t temperature_raw = simpleclock_tmp117_decode_temperature_raw(
        (uint8_t)(temperature_encoded >> 8U), (uint8_t)temperature_encoded);
    const bool temp_valid = temp_ok && simpleclock_tmp117_temperature_raw_valid(temperature_raw);
    const bool id_valid = id_ok && simpleclock_tmp117_device_id_valid(device_id);
    const bool data_ready = config_ok && simpleclock_tmp117_data_ready(configuration);
    uint32_t usb_disconnected = 0U;
    uint32_t usb_not_configured = 0U;
    uint32_t usb_busy_drops = 0U;
    uint32_t usb_errors = 0U;
    const uint8_t revision = simpleclock_tmp117_device_revision(device_id);

    simpleclock_usb_get_counters(&usb_disconnected, &usb_not_configured, &usb_busy_drops,
                                 &usb_errors);

    bool format_truncated = false;
    size_t line_length = 0U;
    line_length =
        append_literal(line, sizeof(line), line_length, "format_truncated=", &format_truncated);
    const size_t format_flag_index = line_length;
    line_length = append_char(line, sizeof(line), line_length, '0', &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, " uptime_ms=", &format_truncated);
    line_length = append_decimal(line, sizeof(line), line_length, uptime_ms, &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, " sample=", &format_truncated);
    line_length =
        append_decimal(line, sizeof(line), line_length, g_sample_sequence, &format_truncated);
    line_length =
        append_literal(line, sizeof(line), line_length, " raw_temp=0x", &format_truncated);
    line_length =
        append_hex16(line, sizeof(line), line_length, temperature_encoded, &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, " temp_c=", &format_truncated);
    if (temp_valid)
    {
        line_length =
            append_temperature(line, sizeof(line), line_length, temperature_raw, &format_truncated);
    }
    else
    {
        line_length = append_literal(line, sizeof(line), line_length, "NA", &format_truncated);
    }
    line_length = append_literal(line, sizeof(line), line_length, " status=", &format_truncated);
    line_length =
        append_literal(line, sizeof(line), line_length,
                       sample_status(id_ok, id_valid, config_ok, temp_ok, data_ready, temp_valid),
                       &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, " config=0x", &format_truncated);
    line_length = append_hex16(line, sizeof(line), line_length, configuration, &format_truncated);
    line_length =
        append_literal(line, sizeof(line), line_length, " device_id=0x", &format_truncated);
    line_length = append_hex16(line, sizeof(line), line_length, device_id, &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, " rev=", &format_truncated);
    line_length = append_decimal(line, sizeof(line), line_length, revision, &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, " i2c_nack=", &format_truncated);
    line_length =
        append_decimal(line, sizeof(line), line_length, g_i2c_errors.nack, &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, " i2c_bus=", &format_truncated);
    line_length =
        append_decimal(line, sizeof(line), line_length, g_i2c_errors.bus, &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, " i2c_error=", &format_truncated);
    line_length =
        append_decimal(line, sizeof(line), line_length, g_i2c_errors.error, &format_truncated);
    line_length =
        append_literal(line, sizeof(line), line_length, " i2c_timeout=", &format_truncated);
    line_length =
        append_decimal(line, sizeof(line), line_length, g_i2c_errors.timeout, &format_truncated);
    line_length =
        append_literal(line, sizeof(line), line_length, " usb_disconnected=", &format_truncated);
    line_length =
        append_decimal(line, sizeof(line), line_length, usb_disconnected, &format_truncated);
    line_length =
        append_literal(line, sizeof(line), line_length, " usb_not_configured=", &format_truncated);
    line_length =
        append_decimal(line, sizeof(line), line_length, usb_not_configured, &format_truncated);
    line_length =
        append_literal(line, sizeof(line), line_length, " usb_busy_drops=", &format_truncated);
    line_length =
        append_decimal(line, sizeof(line), line_length, usb_busy_drops, &format_truncated);
    line_length =
        append_literal(line, sizeof(line), line_length, " usb_errors=", &format_truncated);
    line_length = append_decimal(line, sizeof(line), line_length, usb_errors, &format_truncated);
    line_length = append_literal(line, sizeof(line), line_length, "\r\n", &format_truncated);

    if (format_truncated && (format_flag_index < (sizeof(line) - 1U)))
    {
        line[format_flag_index] = '1';
    }
    const size_t bounded_line_length =
        (line_length < (sizeof(line) - 1U)) ? line_length : (sizeof(line) - 1U);

    (void)simpleclock_usb_cdc_try_send((const uint8_t *)line, (uint16_t)bounded_line_length);
#ifndef SIMPLECLOCK_CLOCK_PROBE
    g_led_on = !g_led_on;
    led1_set(g_led_on && id_valid && temp_valid);
#endif
    g_sample_sequence++;
}

#endif /* legacy sensor/report path; M4 has a single fresh-sample owner */

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef init = {0};

    if (hi2c->Instance == I2C2)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_I2C2_CLK_ENABLE();

        init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        init.Mode = GPIO_MODE_AF_OD;
        init.Pull = GPIO_NOPULL;
        init.Speed = GPIO_SPEED_FREQ_HIGH;
        init.Alternate = GPIO_AF6_I2C2;
        HAL_GPIO_Init(GPIOA, &init);
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C2)
    {
        __HAL_RCC_I2C2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6 | GPIO_PIN_7);
    }
}

void Error_Handler(void)
{
#ifdef SIMPLECLOCK_CLOCK_PROBE
    clock_probe_fault();
#endif
    configure_gpio();
    led1_set(true);
    for (;;)
    {
        __WFI();
    }
}

static void system_clock_config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clocks = {0};
    RCC_PeriphCLKInitTypeDef peripheral_clocks = {0};
    RCC_CRSInitTypeDef crs = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
    osc.HSIState = RCC_HSI_ON;
    osc.HSIDiv = RCC_HSI_DIV1;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.HSI48State = RCC_HSI48_ON;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    clocks.SYSCLKDivider = RCC_SYSCLK_DIV1;
    clocks.AHBCLKDivider = RCC_HCLK_DIV1;
    clocks.APB1CLKDivider = RCC_APB1_DIV1;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }

    peripheral_clocks.PeriphClockSelection = RCC_PERIPHCLK_USB;
    peripheral_clocks.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral_clocks) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_RCC_CRS_CLK_ENABLE();
    crs.Prescaler = RCC_CRS_SYNC_DIV1;
    crs.Source = RCC_CRS_SYNC_SOURCE_USB;
    crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000U, 1000U);
    crs.ErrorLimitValue = 34U;
    crs.HSI48CalibrationValue = 32U;
    HAL_RCCEx_CRSConfig(&crs);
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
void simpleclock_system_reset(void)
{
    /* Foreground caller has held discipline and finished any FCW transaction.
     * The physical heater is off before reset returns GPIO to its boot state. */
    thermal_probe_storage_inhibit(true);
    NVIC_SystemReset();
}
#endif

int main(void)
{
#ifndef SIMPLECLOCK_THERMAL_PROBE
    uint32_t last_sample_ms = 0U;
#endif

    HAL_Init();
#ifdef SIMPLECLOCK_CLOCK_PROBE
    clock_probe_early_gpio();
#endif
    system_clock_config();
    configure_gpio();
    configure_i2c2();
    MX_USB_Device_Init();
#ifdef SIMPLECLOCK_CLOCK_PROBE
    clock_probe_init();
#endif
#ifdef SIMPLECLOCK_THERMAL_PROBE
    thermal_probe_init(HAL_GetTick());
#endif

    for (;;)
    {
#ifdef SIMPLECLOCK_THERMAL_PROBE
        /* One bounded sensor transaction/control step, then drain receiver work.
         * SysTick retains the independent actuator lease even if main stalls. */
        thermal_probe_service(HAL_GetTick());
#else
        const uint32_t now_ms = HAL_GetTick();

        /* SysTick remains the monotonic scheduler; USB completion never gates it. */
        if ((uint32_t)(now_ms - last_sample_ms) >= SIMPLECLOCK_TMP117_SAMPLE_PERIOD_MS)
        {
            last_sample_ms = now_ms;
            sample_and_report(now_ms);
        }
#endif
#ifdef SIMPLECLOCK_CLOCK_PROBE
        clock_probe_service(HAL_GetTick());
#endif
        __WFI();
    }
}
