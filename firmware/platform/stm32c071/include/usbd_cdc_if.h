/*
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * Header adapted from the STM32C0 CDC_Standalone application (CoreMW apps
 * v1.1.0, commit 334655ed5eb719db18fa5990a5044332eeca59dc).
 * The exact Applications license is retained at
 * vendor/st/stm32c0-classic-coremw-apps/LICENSE.md.
 *
 * SPDX-License-Identifier: LicenseRef-ST-SLA0044
 */

#ifndef SIMPLECLOCK_USBD_CDC_IF_H
#define SIMPLECLOCK_USBD_CDC_IF_H

#include <stdbool.h>
#include <stdint.h>

#include "usbd_cdc.h"

#define SIMPLECLOCK_CDC_RX_PACKET_SIZE (64U)

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

uint8_t CDC_Transmit_FS(const uint8_t *buffer, uint16_t length);

#endif /* SIMPLECLOCK_USBD_CDC_IF_H */
