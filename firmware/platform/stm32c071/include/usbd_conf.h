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

#ifndef SIMPLECLOCK_USBD_CONF_H
#define SIMPLECLOCK_USBD_CONF_H

#include <string.h>

#include "stm32c0xx_hal.h"

#define USBD_MAX_NUM_INTERFACES (2U)
#define USBD_MAX_NUM_CONFIGURATION (1U)
#define USBD_MAX_STR_DESC_SIZ (128U)
#define USBD_DEBUG_LEVEL (0U)
#define USBD_LPM_ENABLED (0U)
#define USBD_CLASS_BOS_ENABLED (0U)
#define USBD_USER_REGISTER_CALLBACK (0U)
#define USBD_SELF_POWERED (1U)
#define USBD_SUPPORT_USER_STRING (0U)

#define USBD_malloc (void *)USBD_static_malloc
#define USBD_free USBD_static_free
#define USBD_memset memset
#define USBD_memcpy memcpy
#define USBD_Delay HAL_Delay

#define USBD_UsrLog(...)
#define USBD_ErrLog(...)
#define USBD_DbgLog(...)

void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *ptr);

#endif /* SIMPLECLOCK_USBD_CONF_H */
