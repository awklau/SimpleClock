/*
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * CDC interface adapted from the STM32C0 CDC_Standalone application
 * (CoreMW apps v1.1.0, commit 334655ed5eb719db18fa5990a5044332eeca59dc).
 * The exact Applications license is retained at
 * vendor/st/stm32c0-classic-coremw-apps/LICENSE.md.
 *
 * SPDX-License-Identifier: LicenseRef-ST-SLA0044
 */

#include "usbd_cdc_if.h"

#include <string.h>

#include "main.h"
#include "usb_device.h"
#ifdef SIMPLECLOCK_THERMAL_PROBE
#include "thermal_probe.h"
#endif
#ifdef SIMPLECLOCK_CLOCK_PROBE
#include "gnss_probe.h"
#endif

static uint8_t g_rx_buffer[SIMPLECLOCK_CDC_RX_PACKET_SIZE];
static uint8_t g_tx_buffer[SIMPLECLOCK_TMP117_USB_TX_MAX];
static uint8_t g_tx_line_coding[7];
static USBD_CDC_LineCodingTypeDef g_line_coding = {115200U, 0U, 0U, 8U};

static int8_t cdc_init(void)
{
#ifdef SIMPLECLOCK_THERMAL_PROBE
    thermal_probe_disconnect();
#endif
    (void)USBD_CDC_SetRxBuffer(&hUsbDeviceFS, g_rx_buffer);
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (int8_t)USBD_OK;
}

static int8_t cdc_deinit(void)
{
#ifdef SIMPLECLOCK_THERMAL_PROBE
    thermal_probe_disconnect();
#endif
    return (int8_t)USBD_OK;
}

static int8_t cdc_control(uint8_t command, uint8_t *buffer, uint16_t length)
{
    const uint16_t line_coding_size = (uint16_t)sizeof(g_tx_line_coding);

    switch (command)
    {
    case CDC_SET_LINE_CODING:
        if (length >= line_coding_size)
        {
            g_line_coding.bitrate = ((uint32_t)buffer[0]) | ((uint32_t)buffer[1] << 8U) |
                                    ((uint32_t)buffer[2] << 16U) | ((uint32_t)buffer[3] << 24U);
            g_line_coding.format = buffer[4];
            g_line_coding.paritytype = buffer[5];
            g_line_coding.datatype = buffer[6];
        }
        break;
    case CDC_GET_LINE_CODING:
        g_tx_line_coding[0] = (uint8_t)g_line_coding.bitrate;
        g_tx_line_coding[1] = (uint8_t)(g_line_coding.bitrate >> 8U);
        g_tx_line_coding[2] = (uint8_t)(g_line_coding.bitrate >> 16U);
        g_tx_line_coding[3] = (uint8_t)(g_line_coding.bitrate >> 24U);
        g_tx_line_coding[4] = g_line_coding.format;
        g_tx_line_coding[5] = g_line_coding.paritytype;
        g_tx_line_coding[6] = g_line_coding.datatype;
        {
            const uint16_t copy_length = (length < line_coding_size) ? length : line_coding_size;
            for (uint16_t index = 0U; index < copy_length; index++)
            {
                buffer[index] = g_tx_line_coding[index];
            }
        }
        break;
    case CDC_SET_CONTROL_LINE_STATE:
#ifdef SIMPLECLOCK_THERMAL_PROBE
        /* ST passes its setup request for zero-length class controls. DTR close
         * cancels only unexecuted input; it never stops a healthy thermal loop. */
        if (length == 0U && buffer != NULL &&
            (((USBD_SetupReqTypedef *)(void *)buffer)->wValue & 1U) == 0U)
        {
            thermal_probe_disconnect();
        }
        break;
#endif
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
    case CDC_SEND_BREAK:
    default:
        /* Diagnostic transport has no modem or command side effects. */
        break;
    }
    return (int8_t)USBD_OK;
}

static int8_t cdc_receive(uint8_t *buffer, uint32_t *length)
{
#ifdef SIMPLECLOCK_THERMAL_PROBE
    thermal_probe_receive(buffer, *length);
#elif defined(SIMPLECLOCK_CLOCK_PROBE)
    static char command[32];
    static uint32_t used;
    static bool overflow;
    for (uint32_t i = 0U; i < *length; i++)
    {
        if (buffer[i] == '\r' || buffer[i] == '\n')
        {
            command[used] = '\0';
            const uint32_t profile =
                overflow ? GNSS_PROFILE_NONE : gnss_probe_command_profile(command);
            if (profile != GNSS_PROFILE_NONE)
            {
                gnss_probe_request(profile); /* Mailbox only; no UART I/O in ISR. */
            }
            used = 0U;
            overflow = false;
        }
        else if (used + 1U < sizeof(command))
        {
            command[used++] = (char)buffer[i];
        }
        else
        {
            overflow = true;
        }
    }
#else
    /* RX is intentionally discarded; the bounded callback still re-arms OUT. */
    (void)buffer;
    (void)length;
#endif
    (void)USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    return (int8_t)USBD_OK;
}

static int8_t cdc_transmit_complete(uint8_t *buffer, uint32_t *length, uint8_t epnum)
{
    (void)buffer;
    (void)length;
    (void)epnum;
    return (int8_t)USBD_OK;
}

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {cdc_init, cdc_deinit, cdc_control, cdc_receive,
                                              cdc_transmit_complete};

uint8_t CDC_Transmit_FS(const uint8_t *buffer, uint16_t length)
{
    USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
    USBD_CDC_HandleTypeDef *cdc;

    if ((pdev->dev_state != USBD_STATE_CONFIGURED) || (pdev->classId >= USBD_MAX_SUPPORTED_CLASS) ||
        (pdev->pClassDataCmsit[pdev->classId] == NULL))
    {
        return USBD_FAIL;
    }
    cdc = (USBD_CDC_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
    if (cdc->TxState != 0U)
    {
        return USBD_BUSY;
    }
    if ((buffer == NULL) || (length > (uint16_t)sizeof(g_tx_buffer)))
    {
        return USBD_FAIL;
    }

    /* The class is idle and configured above; only now copy into the buffer
   * whose lifetime covers the asynchronous multi-packet transfer. */
    (void)memcpy(g_tx_buffer, buffer, length);
    (void)USBD_CDC_SetTxBuffer(pdev, g_tx_buffer, length);
    return USBD_CDC_TransmitPacket(pdev);
}

bool simpleclock_usb_cdc_tx_idle(void)
{
    const USBD_HandleTypeDef *pdev = &hUsbDeviceFS;
    if (pdev->dev_state != USBD_STATE_CONFIGURED || pdev->classId >= USBD_MAX_SUPPORTED_CLASS ||
        pdev->pClassDataCmsit[pdev->classId] == NULL)
    {
        return true; /* A disconnected host cannot retain an active transfer. */
    }
    const USBD_CDC_HandleTypeDef *cdc = pdev->pClassDataCmsit[pdev->classId];
    return cdc->TxState == 0U;
}
