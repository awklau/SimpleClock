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

#ifndef SIMPLECLOCK_USB_DEVICE_H
#define SIMPLECLOCK_USB_DEVICE_H

#include "usbd_core.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_Device_Init(void);

#endif /* SIMPLECLOCK_USB_DEVICE_H */
