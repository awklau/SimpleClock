/*
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * CDC descriptors adapted from the STM32C0 CDC_Standalone application
 * (CoreMW apps v1.1.0, commit 334655ed5eb719db18fa5990a5044332eeca59dc).
 * The exact Applications license is retained at
 * vendor/st/stm32c0-classic-coremw-apps/LICENSE.md.
 *
 * SPDX-License-Identifier: LicenseRef-ST-SLA0044
 */

#include "usbd_desc.h"

#include "stm32c0xx.h"
#include "usbd_core.h"

#if !defined(SIMPLECLOCK_PRIVATE_USB_TEST_IDENTITY_ACK) ||                                         \
    (SIMPLECLOCK_PRIVATE_USB_TEST_IDENTITY_ACK != 1)
#error "USB CDC target requires SIMPLECLOCK_PRIVATE_USB_TEST_IDENTITY_ACK=1"
#endif

#define SIMPLECLOCK_USB_VID (0x1209U)
#define SIMPLECLOCK_USB_PID (0x0001U)
#define SIMPLECLOCK_USB_LANGID (1033U)
#define SIMPLECLOCK_USB_SERIAL_LENGTH (26U)

static uint8_t g_string_descriptor[USBD_MAX_STR_DESC_SIZ];
static uint8_t g_serial_descriptor[SIMPLECLOCK_USB_SERIAL_LENGTH] = {SIMPLECLOCK_USB_SERIAL_LENGTH,
                                                                     USB_DESC_TYPE_STRING};

static uint8_t g_device_descriptor[USB_LEN_DEV_DESC] = {USB_LEN_DEV_DESC,
                                                        USB_DESC_TYPE_DEVICE,
                                                        0x00U,
                                                        0x02U,
                                                        0x02U,
                                                        0x02U,
                                                        0x00U,
                                                        USB_MAX_EP0_SIZE,
                                                        LOBYTE(SIMPLECLOCK_USB_VID),
                                                        HIBYTE(SIMPLECLOCK_USB_VID),
                                                        LOBYTE(SIMPLECLOCK_USB_PID),
                                                        HIBYTE(SIMPLECLOCK_USB_PID),
                                                        0x00U,
                                                        0x01U,
                                                        USBD_IDX_MFC_STR,
                                                        USBD_IDX_PRODUCT_STR,
                                                        USBD_IDX_SERIAL_STR,
                                                        USBD_MAX_NUM_CONFIGURATION};

static uint8_t *device_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = (uint16_t)sizeof(g_device_descriptor);
    return g_device_descriptor;
}

static uint8_t *language_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    static uint8_t const language[] = {USB_LEN_LANGID_STR_DESC, USB_DESC_TYPE_STRING,
                                       LOBYTE(SIMPLECLOCK_USB_LANGID),
                                       HIBYTE(SIMPLECLOCK_USB_LANGID)};
    (void)speed;
    *length = (uint16_t)sizeof(language);
    return (uint8_t *)language;
}

static uint8_t *manufacturer_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"SimpleClock", g_string_descriptor, length);
    return g_string_descriptor;
}

static uint8_t *product_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"SimpleClock PRIVATE DIAGNOSTIC", g_string_descriptor, length);
    return g_string_descriptor;
}

static uint8_t *serial_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    const uint32_t uid0 = *(volatile const uint32_t *)(UID_BASE + 0x00U);
    const uint32_t uid1 = *(volatile const uint32_t *)(UID_BASE + 0x04U);
    const uint32_t uid2 = *(volatile const uint32_t *)(UID_BASE + 0x08U);
    uint32_t value = uid0 + uid2;

    (void)speed;
    for (uint8_t index = 0U; index < 8U; index++)
    {
        const uint32_t nibble = (value >> 28U) & 0x0FU;
        g_serial_descriptor[2U + (2U * index)] =
            (uint8_t)((nibble < 10U) ? ('0' + nibble) : ('A' + nibble - 10U));
        g_serial_descriptor[3U + (2U * index)] = 0U;
        value <<= 4U;
    }
    value = uid1;
    for (uint8_t index = 0U; index < 4U; index++)
    {
        const uint32_t nibble = (value >> 28U) & 0x0FU;
        g_serial_descriptor[18U + (2U * index)] =
            (uint8_t)((nibble < 10U) ? ('0' + nibble) : ('A' + nibble - 10U));
        g_serial_descriptor[19U + (2U * index)] = 0U;
        value <<= 4U;
    }
    *length = (uint16_t)sizeof(g_serial_descriptor);
    return g_serial_descriptor;
}

static uint8_t *configuration_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"CDC private bench configuration", g_string_descriptor, length);
    return g_string_descriptor;
}

static uint8_t *interface_descriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)"TMP117 PRIVATE TELEMETRY", g_string_descriptor, length);
    return g_string_descriptor;
}

USBD_DescriptorsTypeDef CDC_Desc = {
    device_descriptor, language_descriptor,      manufacturer_descriptor, product_descriptor,
    serial_descriptor, configuration_descriptor, interface_descriptor};
