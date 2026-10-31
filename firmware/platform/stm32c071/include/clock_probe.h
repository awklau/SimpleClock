/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_CLOCK_PROBE_H
#define SIMPLECLOCK_CLOCK_PROBE_H
#include <stdint.h>
/* Call early_gpio after HAL_Init, before clocks/USB or device access. init paints
 * unused stack and starts IWDG; service must run continuously from foreground.
 * fault forces heater off and withholds watchdog service on subsequent calls. */
void clock_probe_early_gpio(void);
void clock_probe_init(void);
void clock_probe_service(uint32_t now_ms);
void clock_probe_fault(void);
#endif
