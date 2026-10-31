/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_SETTINGS_FLASH_H
#define SIMPLECLOCK_SETTINGS_FLASH_H
#include "simpleclock/settings.h"

/* Final two independently erasable 2 KiB pages of the 128 KiB C071.
 * The linker excludes these pages from every application's loadable image. */
sc_settings_status_t settings_flash_load(sc_settings_t *out, uint32_t *generation);
/* Foreground only. Caller holds the clock and excludes pending FCW/manual work.
 * Physically inhibit heater output during single-bank stalls, restore elapsed
 * HAL time from TIM2, invalidate duty evidence, then allow normal regulation. */
bool settings_flash_save(const sc_settings_t *settings);
#endif
