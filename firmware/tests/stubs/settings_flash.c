/* SPDX-License-Identifier: MIT */
/* Byte-accurate flash-record fixture; no target flash/timing claim. */
#include "settings_flash.h"
#include <string.h>

static uint8_t records[2][SC_SETTINGS_RECORD_BYTES];
static bool initialized;
void (*settings_flash_test_pause)(void);

static void initialize(void)
{
    if (!initialized)
    {
        memset(records, UINT8_MAX, sizeof(records));
        initialized = true;
    }
}

static bool read_slot(void *context, unsigned slot, uint8_t *record)
{
    (void)context;
    memcpy(record, records[slot], SC_SETTINGS_RECORD_BYTES);
    return true;
}

static bool erase_slot(void *context, unsigned slot)
{
    (void)context;
    if (settings_flash_test_pause)
    {
        settings_flash_test_pause();
    }
    memset(records[slot], UINT8_MAX, SC_SETTINGS_RECORD_BYTES);
    return true;
}

static bool program_slot(void *context, unsigned slot, unsigned offset, const uint8_t *record)
{
    (void)context;
    for (unsigned i = 0U; i < 8U; i++)
    {
        records[slot][offset + i] &= record[i];
    }
    return true;
}

static const sc_settings_io_t io = {NULL, read_slot, erase_slot, program_slot};

sc_settings_status_t settings_flash_load(sc_settings_t *out, uint32_t *generation)
{
    initialize();
    unsigned slot;
    return sc_settings_load(&io, out, generation, &slot);
}

bool settings_flash_save(const sc_settings_t *settings)
{
    initialize();
    return sc_settings_save(&io, settings);
}
