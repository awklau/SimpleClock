/*
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * Adapted from the STM32C0 CDC_Standalone low-level device configuration
 * (CoreMW apps v1.1.0, commit 334655ed5eb719db18fa5990a5044332eeca59dc).
 * The application source and this adapter are covered by ST SLA0044; the
 * exact Applications license is retained at
 * vendor/st/stm32c0-classic-coremw-apps/LICENSE.md.
 *
 * SPDX-License-Identifier: LicenseRef-ST-SLA0044
 */

#include "stm32c0xx_hal.h"
#include "stm32c0xx_hal_pcd_ex.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
#include "usbd_conf.h"

#include "main.h"

/* Common HAL <-> USB-device adapter for text and maintenance profiles.
 * HAL callbacks execute in the USB IRQ. Keep parsing/formatting out of them;
 * interface-specific buffering lives in usbd_cdc_if.c or bridge_usb.c. */

PCD_HandleTypeDef hpcd_USB_DRD_FS;

static volatile uint32_t g_usb_disconnected;
static volatile uint32_t g_usb_not_configured;
static volatile uint32_t g_usb_busy_drops;
static volatile uint32_t g_usb_errors;

static USBD_StatusTypeDef usb_status_from_hal(HAL_StatusTypeDef status)
{
    switch (status)
    {
    case HAL_OK:
        return USBD_OK;
    case HAL_BUSY:
        return USBD_BUSY;
    case HAL_ERROR:
    case HAL_TIMEOUT:
    default:
        return USBD_FAIL;
    }
}

void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB_DRD_FS)
    {
        __HAL_RCC_USB_CLK_ENABLE();
        HAL_NVIC_SetPriority(USB_DRD_FS_IRQn, 3U, 0U);
        HAL_NVIC_EnableIRQ(USB_DRD_FS_IRQn);
    }
}

void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Instance == USB_DRD_FS)
    {
        __HAL_RCC_USB_CLK_DISABLE();
        HAL_NVIC_DisableIRQ(USB_DRD_FS_IRQn);
    }
}

void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
{
    (void)USBD_LL_SetupStage((USBD_HandleTypeDef *)hpcd->pData, (uint8_t *)hpcd->Setup);
}

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    (void)USBD_LL_DataOutStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                               hpcd->OUT_ep[epnum].xfer_buff);
}

void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
{
    (void)USBD_LL_DataInStage((USBD_HandleTypeDef *)hpcd->pData, epnum,
                              hpcd->IN_ep[epnum].xfer_buff);
}

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
{
    (void)USBD_LL_SOF((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
{
    if (hpcd->Init.speed != PCD_SPEED_FULL)
    {
        g_usb_errors++;
        return;
    }
    (void)USBD_LL_SetSpeed((USBD_HandleTypeDef *)hpcd->pData, USBD_SPEED_FULL);
    (void)USBD_LL_Reset((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
{
    (void)USBD_LL_Suspend((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
{
    (void)USBD_LL_Resume((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
{
    (void)USBD_LL_DevConnected((USBD_HandleTypeDef *)hpcd->pData);
}

void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
{
    g_usb_disconnected++;
    (void)USBD_LL_DevDisconnected((USBD_HandleTypeDef *)hpcd->pData);
}

USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev)
{
    hpcd_USB_DRD_FS.pData = pdev;
    pdev->pData = &hpcd_USB_DRD_FS;
    hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
    hpcd_USB_DRD_FS.Init.dev_endpoints = 8U;
    hpcd_USB_DRD_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;

    if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK)
    {
        return USBD_FAIL;
    }

    (void)HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x00U, PCD_SNG_BUF, 0x14U);
    (void)HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, 0x80U, PCD_SNG_BUF, 0x54U);
    (void)HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_IN_EP, PCD_SNG_BUF, 0x94U);
    (void)HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_OUT_EP, PCD_SNG_BUF, 0xD4U);
    (void)HAL_PCDEx_PMAConfig(&hpcd_USB_DRD_FS, CDC_CMD_EP, PCD_SNG_BUF, 0x114U);

    return USBD_OK;
}

USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev)
{
    return usb_status_from_hal(HAL_PCD_DeInit(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev)
{
    return usb_status_from_hal(HAL_PCD_Start(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev)
{
    return usb_status_from_hal(HAL_PCD_Stop(pdev->pData));
}

USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type,
                                  uint16_t ep_mps)
{
    return usb_status_from_hal(HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type));
}

USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return usb_status_from_hal(HAL_PCD_EP_Close(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return usb_status_from_hal(HAL_PCD_EP_Flush(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return usb_status_from_hal(HAL_PCD_EP_SetStall(pdev->pData, ep_addr));
}

USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return usb_status_from_hal(HAL_PCD_EP_ClrStall(pdev->pData, ep_addr));
}

uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    const PCD_HandleTypeDef *hpcd = (const PCD_HandleTypeDef *)pdev->pData;
    if ((ep_addr & 0x80U) != 0U)
    {
        return hpcd->IN_ep[ep_addr & 0x7FU].is_stall;
    }
    return hpcd->OUT_ep[ep_addr & 0x7FU].is_stall;
}

USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr)
{
    return usb_status_from_hal(HAL_PCD_SetAddress(pdev->pData, dev_addr));
}

USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf,
                                    uint32_t size)
{
    return usb_status_from_hal(HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size));
}

USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf,
                                          uint32_t size)
{
    return usb_status_from_hal(HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size));
}

uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

void USBD_LL_Delay(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}

void *USBD_static_malloc(uint32_t size)
{
    static uint32_t memory[(sizeof(USBD_CDC_HandleTypeDef) + 3U) / 4U];

    /* The USB stack owns this single block and reuses it after every deinit. */
    if (size > sizeof(memory))
    {
        return NULL;
    }
    return memory;
}

void USBD_static_free(void *ptr)
{
    (void)ptr;
}

static bool usb_try_send(const uint8_t *data, uint16_t length, bool retained)
{
    USBD_HandleTypeDef *pdev = (USBD_HandleTypeDef *)hpcd_USB_DRD_FS.pData;

    if ((hpcd_USB_DRD_FS.pData == NULL) || (pdev->dev_state != USBD_STATE_CONFIGURED))
    {
        if (!retained)
        {
            g_usb_not_configured++;
        }
        return false;
    }

    const uint8_t result = CDC_Transmit_FS((uint8_t *)data, length);
    if (result == USBD_OK)
    {
        return true;
    }
    if (result == USBD_BUSY)
    {
        if (!retained)
        {
            g_usb_busy_drops++;
        }
    }
    else
    {
        g_usb_errors++;
    }
    return false;
}

bool simpleclock_usb_cdc_try_send(const uint8_t *data, uint16_t length)
{
    return usb_try_send(data, length, false);
}

bool simpleclock_usb_cdc_try_send_queued(const uint8_t *data, uint16_t length)
{
    return usb_try_send(data, length, true);
}

void simpleclock_usb_get_counters(uint32_t *disconnected, uint32_t *not_configured,
                                  uint32_t *busy_drops, uint32_t *errors)
{
    *disconnected = g_usb_disconnected;
    *not_configured = g_usb_not_configured;
    *busy_drops = g_usb_busy_drops;
    *errors = g_usb_errors;
}
