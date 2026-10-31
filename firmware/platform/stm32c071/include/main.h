/* SPDX-License-Identifier: MIT */

#ifndef SIMPLECLOCK_STM32C071_TMP117_CDC_MAIN_H
#define SIMPLECLOCK_STM32C071_TMP117_CDC_MAIN_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32c0xx_hal.h"

#define SIMPLECLOCK_TMP117_I2C_TIMEOUT_MS (25U)
#define SIMPLECLOCK_TMP117_SAMPLE_PERIOD_MS (1000U)
#ifdef SIMPLECLOCK_THERMAL_PROBE
/* Thermal policy and GPIO evidence must fit even as 32-bit counters grow. */
#define SIMPLECLOCK_TMP117_USB_TX_MAX (1024U)
#elif defined(SIMPLECLOCK_CLOCK_PROBE)
#define SIMPLECLOCK_TMP117_USB_TX_MAX (768U)
#else
#define SIMPLECLOCK_TMP117_USB_TX_MAX (384U)
#endif

extern I2C_HandleTypeDef hi2c2;

void Error_Handler(void);

/* Non-blocking, bounded CDC submission. */
bool simpleclock_usb_cdc_try_send(const uint8_t *data, uint16_t length);
/* Caller retains queued bytes on BUSY; do not count a retry as a dropped record. */
bool simpleclock_usb_cdc_try_send_queued(const uint8_t *data, uint16_t length);
bool simpleclock_usb_cdc_tx_idle(void);
void simpleclock_system_reset(void);
void simpleclock_usb_get_counters(uint32_t *disconnected, uint32_t *not_configured,
                                  uint32_t *busy_drops, uint32_t *errors);

#endif /* SIMPLECLOCK_STM32C071_TMP117_CDC_MAIN_H */
