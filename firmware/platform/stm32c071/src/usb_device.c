/*
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * CDC device initialization adapted from the STM32C0 CDC_Standalone
 * reference (CoreMW apps v1.1.0, commit
 * 334655ed5eb719db18fa5990a5044332eeca59dc).
 * The exact Applications license is retained at
 * vendor/st/stm32c0-classic-coremw-apps/LICENSE.md.
 *
 * SPDX-License-Identifier: LicenseRef-ST-SLA0044
 */

#include "usb_device.h"
#include "main.h"

#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_desc.h"

/* One CDC class/interface per image. CMake selects the matching fops adapter;
 * initialization failures are terminal and never fall back to a different profile. */
USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_Device_Init(void)
{
    if (USBD_Init(&hUsbDeviceFS, &CDC_Desc, 0U) != USBD_OK)
    {
        Error_Handler();
    }
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
    {
        Error_Handler();
    }
    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
    {
        Error_Handler();
    }
    if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
    {
        Error_Handler();
    }
}
