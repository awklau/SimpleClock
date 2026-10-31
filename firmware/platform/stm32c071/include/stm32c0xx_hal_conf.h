/* SPDX-License-Identifier: MIT */

#ifndef SIMPLECLOCK_STM32C0XX_HAL_CONF_H
#define SIMPLECLOCK_STM32C0XX_HAL_CONF_H

/* The target uses only the clock, GPIO, I2C2, USB PCD, and supporting HAL. */
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_PCD_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED

#define HSE_VALUE (48000000U)
#define HSE_STARTUP_TIMEOUT (100U)
#define HSI_VALUE (48000000U)
#define HSI48_VALUE (48000000U)
#define LSI_VALUE (32000U)
#define LSE_VALUE (32768U)
#define LSE_STARTUP_TIMEOUT (5000U)
#define EXTERNAL_I2S1_CLOCK_VALUE (48000U)
#define VDD_VALUE (3300U)
#define TICK_INT_PRIORITY (3U)
#define USE_RTOS 0U
#define PREFETCH_ENABLE 1U
#define INSTRUCTION_CACHE_ENABLE 0U
#define DATA_CACHE_ENABLE 0U
#define USE_HAL_ADC_REGISTER_CALLBACKS 0U
#define USE_HAL_CORTEX_REGISTER_CALLBACKS 0U
#define USE_HAL_FLASH_REGISTER_CALLBACKS 0U
#define USE_HAL_I2C_REGISTER_CALLBACKS 0U
#define USE_HAL_PCD_REGISTER_CALLBACKS 0U
#define USE_HAL_PWR_REGISTER_CALLBACKS 0U
#define USE_HAL_RCC_REGISTER_CALLBACKS 0U

/* Single PMA buffers keep the ISR bounded and make the errata audit direct. */
#define USE_USB_DOUBLE_BUFFER 0U

/* HAL common definitions must precede the module declarations below.  The
 * ST configuration template includes these headers from this file; keeping
 * that ordering is required because stm32c0xx_hal.h includes the application
 * configuration before it declares its own API. */
#include "stm32c0xx_hal_def.h"

/* Selected module declarations, matching the ST HAL configuration template. */
#include "stm32c0xx_hal_rcc.h"
#include "stm32c0xx_hal_rcc_ex.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_cortex.h"
#include "stm32c0xx_hal_flash.h"
#include "stm32c0xx_hal_flash_ex.h"
#include "stm32c0xx_hal_i2c.h"
#include "stm32c0xx_hal_i2c_ex.h"
#include "stm32c0xx_hal_pcd.h"
#include "stm32c0xx_hal_pcd_ex.h"
#include "stm32c0xx_hal_pwr.h"

#ifndef USE_FULL_ASSERT
#define assert_param(expr) ((void)0U)
#endif

#endif /* SIMPLECLOCK_STM32C0XX_HAL_CONF_H */
