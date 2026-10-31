/*
 * STM32C071FBP6 LED1 diagnostic.
 *
 * This image intentionally exercises only the MCU, PA8, and the Cortex-M0+
 * SysTick interrupt. It does not initialize any board peripheral or claim
 * timing precision.
 */

#include <stdbool.h>

#include "stm32c0xx.h"

#define LED1_PIN (1UL << 8U)
#define LED1_MODE_MASK (GPIO_MODER_MODE8_Msk)
#define LED1_MODE_OUTPUT (GPIO_MODER_MODE8_0)
#define MILLISECONDS_PER_TOGGLE (500U)

static volatile uint32_t g_millis;

/* The official ST startup calls __libc_init_array; no CRT startup is linked. */
void _init(void) {}

void SysTick_Handler(void)
{
    g_millis += 1U;
}

static void led1_set(bool on)
{
    if (on)
    {
        GPIOA->BSRR = LED1_PIN;
    }
    else
    {
        GPIOA->BSRR = LED1_PIN << 16U;
    }
}

static __NO_RETURN void diagnostic_safe_state(void)
{
    led1_set(true);
    for (;;)
    {
        __WFI();
    }
}

int main(void)
{
    uint32_t last_toggle_ms = 0U;
    bool led_on = false;

    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    /* Set the output latch low before changing only PA8's mode to output. */
    led1_set(false);
    GPIOA->MODER = (GPIOA->MODER & ~LED1_MODE_MASK) | LED1_MODE_OUTPUT;

    SystemCoreClockUpdate();
    if (SysTick_Config(SystemCoreClock / 1000U) != 0U)
    {
        diagnostic_safe_state();
    }

    for (;;)
    {
        const uint32_t now_ms = g_millis;

        /* Unsigned subtraction keeps this interval check valid across wrap. */
        if ((uint32_t)(now_ms - last_toggle_ms) >= MILLISECONDS_PER_TOGGLE)
        {
            last_toggle_ms = now_ms;
            led_on = !led_on;
            led1_set(led_on);
        }
        __WFI();
    }
}
