/* SPDX-License-Identifier: MIT */
/* Quiet maintenance application: safe GPIO, clocks, watchdog and the binary bridge.
 * This alternative main never starts the normal text diagnostics or GNSS probe. */
#include "main.h"
#include "bridge_service.h"
#include "usb_device.h"
#include "stm32c0xx_hal_rcc_ex.h"

extern uint32_t __StackLimit[], __StackTop[];
#define STACK_PATTERN UINT32_C(0xa5c39e71)

void _init(void) {}

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

static void safe_gpio(void)
{
    GPIO_InitTypeDef pin = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIOB->BRR = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    pin.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
    pin.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &pin);
    pin.Pin = GPIO_PIN_3;
    pin.Mode = GPIO_MODE_OUTPUT_PP;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &pin);
    /* Preserve OCXO reference; mute PPS distribution during maintenance. */
    GPIOA->BSRR = GPIO_PIN_1;
    GPIOA->BRR = GPIO_PIN_0 | GPIO_PIN_8;
    pin.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &pin);
    GPIOC->BRR = GPIO_PIN_14 | GPIO_PIN_15;
    pin.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOC, &pin);
    /* PA4 reset stays at reset-state high impedance. No I2C writes. */
}

bool bridge_guard_ok(void)
{
    const volatile uint32_t *p = __StackLimit;
    return p[0] == STACK_PATTERN && p[1] == STACK_PATTERN && p[2] == STACK_PATTERN &&
           p[3] == STACK_PATTERN;
}

uint32_t bridge_stack_peak(void)
{
    uintptr_t p = (uintptr_t)__StackLimit;
    while (p < (uintptr_t)__StackTop && *(volatile uint32_t *)p == STACK_PATTERN)
    {
        p += 4U;
    }
    return (uint32_t)((uintptr_t)__StackTop - p);
}

void Error_Handler(void)
{
    GPIOB->BRR = GPIO_PIN_3;
    for (;;)
    {
        __WFI();
    }
}

/* Clock setup is identical to the proven TMP117/M3 USB diagnostic. */
static void system_clock_config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clocks = {0};
    RCC_PeriphCLKInitTypeDef peripheral = {0};
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
    peripheral.PeriphClockSelection = RCC_PERIPHCLK_USB;
    peripheral.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&peripheral) != HAL_OK)
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

int main(void)
{
    HAL_Init();
    safe_gpio();
    system_clock_config();
    uintptr_t end = (uintptr_t)__get_MSP() - 32U;
    for (uintptr_t p = (uintptr_t)__StackLimit; p < end; p += 4U)
    {
        *(volatile uint32_t *)p = STACK_PATTERN;
    }
    IWDG->KR = 0xccccU;
    IWDG->KR = 0x5555U;
    IWDG->PR = 6U;
    IWDG->RLR = 250U; /* Nominal 2.008 s at 32 kHz LSI. */
    uint32_t start = HAL_GetTick();
    while (IWDG->SR != 0U)
    {
        if (HAL_GetTick() - start > 100U)
        {
            Error_Handler();
        }
    }
    IWDG->KR = 0xaaaaU;
    bridge_init();
    MX_USB_Device_Init();
    for (;;)
    {
        bridge_service();
        uint32_t phase = HAL_GetTick() % 1000U;
        bool led = bridge_faults ? (phase < 100U || (phase >= 200U && phase < 300U)) : phase < 500U;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, led ? GPIO_PIN_SET : GPIO_PIN_RESET);
        /* Transport faults stop TX, but must not reset a receiver being flashed. */
        if (bridge_guard_ok() && !(bridge_faults & SC_BRIDGE_FAULT_HEALTH))
        {
            IWDG->KR = 0xaaaaU;
        }
        __WFI();
    }
}
