/* SPDX-License-Identifier: MIT */
/* HAL tick and USB IRQ entry points shared by the diagnostic and maintenance builds. */

#include "stm32c0xx_hal.h"
#ifdef SIMPLECLOCK_THERMAL_PROBE
#include "thermal_probe.h"
#include <stddef.h>

/* A fault may make the foreground state/stack unusable. These vectors command
 * PB3 low without a C prologue, HAL, tick or shared-state access, then let the
 * already-running IWDG reset the MCU. Address/offset come from stm32c071xx.h. */
_Static_assert(GPIOB_BASE + offsetof(GPIO_TypeDef, BRR) == 0x50000428UL,
               "fault-off assembly must match this target's GPIOB BRR");

__attribute__((naked, noreturn)) void HardFault_Handler(void)
{
    __asm volatile("cpsid i\n"
                   "ldr r0, =0x50000428\n"
                   "movs r1, #8\n"
                   "str r1, [r0]\n"
                   "dsb\n"
                   "1: b 1b\n");
}

void NMI_Handler(void) __attribute__((alias("HardFault_Handler"), naked, noreturn));
#endif

extern PCD_HandleTypeDef hpcd_USB_DRD_FS;

void SysTick_Handler(void)
{
    HAL_IncTick();
#ifdef SIMPLECLOCK_THERMAL_PROBE
    thermal_probe_tick(HAL_GetTick());
#endif
}

void USB_DRD_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_DRD_FS);
}
