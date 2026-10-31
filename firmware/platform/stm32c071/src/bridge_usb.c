/*
 * Copyright (c) 2024 STMicroelectronics. All rights reserved.
 * Adapted from this project's usbd_cdc_if.c and the STM32C0 CDC application.
 * License retained at vendor/st/stm32c0-classic-coremw-apps/LICENSE.md.
 * SPDX-License-Identifier: LicenseRef-ST-SLA0044
 */
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "bridge_service.h"
#include "simpleclock/bridge.h"
#include <string.h>

/* SCB1 binary adapter; mutually exclusive with usbd_cdc_if.c's text adapter.
 * RX is copied into the service ring before main re-arms OUT. TX storage lives
 * until asynchronous USB IN completes; CDC_Transmit_FS copies only when idle. */

static uint8_t rx_buffer[64], tx_buffer[SC_BRIDGE_FRAME_MAX];

static int8_t cdc_init(void)
{
    (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_buffer);
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    bridge_usb_state(true);
    return (int8_t)USBD_OK;
}

static int8_t cdc_deinit(void)
{
    bridge_usb_state(false);
    return (int8_t)USBD_OK;
}

static int8_t cdc_control(uint8_t command, uint8_t *buffer, uint16_t length)
{
    static const uint8_t coding[7] = {0x00, 0xc2, 0x01, 0x00, 0, 0, 8};
    if (command == CDC_GET_LINE_CODING)
    {
        if (length > sizeof(coding))
        {
            length = (uint16_t)sizeof(coding);
        }
        memcpy(buffer, coding, length);
    }
    if (command == CDC_SET_LINE_CODING &&
        (length != sizeof(coding) || memcmp(buffer, coding, sizeof(coding)) != 0))
    {
        return (int8_t)USBD_FAIL;
    }
    /* Fixed UART 115200 8N1. DTR/RTS and SEND_BREAK never affect G5 reset. */
    return (int8_t)USBD_OK;
}

static int8_t cdc_receive(uint8_t *buffer, uint32_t *length)
{
    bridge_usb_received(buffer, *length);
    return (int8_t)USBD_OK; /* Main loop re-arms OUT only when there is room. */
}

static int8_t cdc_complete(uint8_t *buffer, uint32_t *length, uint8_t epnum)
{
    (void)buffer;
    (void)length;
    (void)epnum;
    return (int8_t)USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {cdc_init, cdc_deinit, cdc_control, cdc_receive,
                                              cdc_complete};

void bridge_usb_rearm(void)
{
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}

uint8_t CDC_Transmit_FS(const uint8_t *buffer, uint16_t length)
{
    USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
    if (pdev->dev_state != USBD_STATE_CONFIGURED || pdev->classId >= USBD_MAX_SUPPORTED_CLASS ||
        pdev->pClassDataCmsit[pdev->classId] == NULL)
    {
        return USBD_FAIL;
    }
    USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
    if (cdc->TxState != 0U)
    {
        return USBD_BUSY;
    }
    if (buffer == NULL || length > sizeof(tx_buffer))
    {
        return USBD_FAIL;
    }
    memcpy(tx_buffer, buffer, length);
    (void)USBD_CDC_SetTxBuffer(pdev, tx_buffer, length);
    return USBD_CDC_TransmitPacket(pdev);
}
