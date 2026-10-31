/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_BRIDGE_SERVICE_H
#define SIMPLECLOCK_BRIDGE_SERVICE_H
#include <stdbool.h>
#include <stdint.h>
#include "simpleclock/bridge.h"

/* Maintenance profile only. init/service and stack queries run in foreground;
 * usb_received/state are USB callbacks. No function here resets the receiver.
 * All protocol faults are sticky for this MCU boot and inhibit future UART TX.
 * Closing a COM handle alone does not reset the session or replay any chunk. */
void bridge_init(void);
void bridge_service(void);
void bridge_usb_received(const uint8_t *data, uint32_t length);
void bridge_usb_state(bool configured);
void bridge_usb_rearm(void);
uint32_t bridge_stack_peak(void);
bool bridge_guard_ok(void);
extern volatile uint32_t bridge_faults;
#endif
