/* SPDX-License-Identifier: MIT */
#include "settings_flash.h"
#include "main.h"
#include "thermal_probe.h"
#include <string.h>

/* STM32C071 RM0490 flash program/erase procedure; DS14693 table 44 specifies
 * <=40 ms/page erase and <=125 us/double-word program. No option bytes or mass
 * erase. Code may stall in this single bank: PB3 is already physically low. */
#define SETTINGS_BASE UINT32_C(0x0801f000)
#define PAGE_BYTES 2048U
#define BUSY_FLAGS (FLASH_SR_BSY1 | FLASH_SR_CFGBSY)
#define ERROR_FLAGS                                                                                \
    (FLASH_SR_OPERR | FLASH_SR_PROGERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | FLASH_SR_SIZERR |     \
     FLASH_SR_PGSERR | FLASH_SR_MISERR | FLASH_SR_FASTERR | FLASH_SR_RDERR | FLASH_SR_OPTVERR)

static bool geometry_valid(void)
{
    return *(const uint16_t *)FLASHSIZE_BASE == 128U;
}

static bool read_slot(void *context, unsigned slot, uint8_t record[SC_SETTINGS_RECORD_BYTES])
{
    (void)context;
    if (!geometry_valid() || slot >= 2U)
    {
        return false;
    }
    memcpy(record, (const void *)(uintptr_t)(SETTINGS_BASE + slot * PAGE_BYTES),
           SC_SETTINGS_RECORD_BYTES);
    return true;
}

static bool wait_idle(void)
{
    const uint32_t start = TIM2->CNT;
    while (FLASH->SR & BUSY_FLAGS)
    {
        /* TIM2 continues during flash stalls; SysTick may not. A stuck BSY
         * can also stall instruction fetch indefinitely; heater stays OFF. */
        if ((uint32_t)(TIM2->CNT - start) > 100000U)
        {
            return false;
        }
    }
    return true;
}

static bool completed(void)
{
    if (!wait_idle())
    {
        return false;
    }
    const uint32_t status = FLASH->SR;
    FLASH->SR = ERROR_FLAGS | FLASH_SR_EOP;
    /* RM0490 table 25: EOP/OPERR depend on interrupt-enable bits. Polling
     * must not require EOP. Always-set error flags plus readback decide. */
    return (status & ERROR_FLAGS) == 0U;
}

static bool erase_slot(void *context, unsigned slot)
{
    (void)context;
    if (slot >= 2U || !wait_idle())
    {
        return false;
    }
    FLASH->SR = ERROR_FLAGS | FLASH_SR_EOP;
    FLASH->CR = (FLASH->CR & ~FLASH_CR_PNB_Msk) | FLASH_CR_PER | ((62U + slot) << FLASH_CR_PNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    const bool ok = completed();
    if (!(FLASH->SR & BUSY_FLAGS))
    {
        FLASH->CR &= ~(FLASH_CR_PER | FLASH_CR_PNB_Msk);
    }
    return ok;
}

static bool program_slot(void *context, unsigned slot, unsigned offset, const uint8_t bytes[8])
{
    (void)context;
    if (slot >= 2U || offset > SC_SETTINGS_RECORD_BYTES - 8U || offset % 8U || !wait_idle())
    {
        return false;
    }
    uint32_t words[2];
    memcpy(words, bytes, sizeof(words));
    volatile uint32_t *address =
        (volatile uint32_t *)(uintptr_t)(SETTINGS_BASE + slot * PAGE_BYTES + offset);
    FLASH->SR = ERROR_FLAGS | FLASH_SR_EOP;
    FLASH->CR |= FLASH_CR_PG;
    address[0] = words[0];
    __ISB();
    address[1] = words[1];
    const bool ok = completed();
    if (!(FLASH->SR & BUSY_FLAGS))
    {
        FLASH->CR &= ~FLASH_CR_PG;
    }
    return ok && address[0] == words[0] && address[1] == words[1];
}

static const sc_settings_io_t io = {NULL, read_slot, erase_slot, program_slot};

sc_settings_status_t settings_flash_load(sc_settings_t *out, uint32_t *generation)
{
    unsigned slot;
    return sc_settings_load(&io, out, generation, &slot);
}

bool settings_flash_save(const sc_settings_t *settings)
{
    if (!geometry_valid() || !sc_settings_valid(settings) || !wait_idle())
    {
        return false;
    }
    /* Avoid even a transient control interruption on an unchanged save. */
    sc_settings_t previous;
    uint32_t generation;
    if (settings_flash_load(&previous, &generation) == SC_SETTINGS_VALID &&
        sc_settings_equal(settings, &previous))
    {
        return true;
    }
    thermal_probe_storage_inhibit(true);
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const uint32_t start = TIM2->CNT;
    if (FLASH->CR & FLASH_CR_LOCK)
    {
        FLASH->KEYR = UINT32_C(0x45670123);
        FLASH->KEYR = UINT32_C(0xcdef89ab);
    }
    const bool ok = !(FLASH->CR & FLASH_CR_LOCK) && sc_settings_save(&io, settings);
    const bool idle = (FLASH->SR & BUSY_FLAGS) == 0U;
    if (idle)
    {
        FLASH->CR |= FLASH_CR_LOCK;
    }
    const uint32_t elapsed_ms = (uint32_t)(TIM2->CNT - start) / 1000U;
    for (uint32_t i = 0U; i < elapsed_ms; i++)
    {
        HAL_IncTick();
    }
    /* One pending SysTick may add a further millisecond. This conservatively
     * advances deadlines; the counter never moves backwards or loses 40 ms. */
    if (!idle)
    {
        thermal_probe_fault();
    }
    thermal_probe_storage_inhibit(false);
    __set_PRIMASK(mask);
    return ok && idle;
}
