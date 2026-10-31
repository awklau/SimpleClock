# STM32C071 ES0618 PMA read workaround

This directory contains the STMicroelectronics STM32C0xx HAL PCD driver from
commit `038bed9f00e0c07328627470328fe8abc184d071` (v1.4.1). The checked-in
`Src/stm32c0xx_hal_pcd.c` is the compiled source for the USB CDC target and
contains the `SIMPLECLOCK_USB_PMA_READ_DELAY` local patch.

ES0618 revision Z, erratum 2.11.1 requires a delay of at least 800 ns before a
USB PMA-to-SRAM read at the maximum core clock. The patch executes a volatile
48-iteration `__NOP()` delay immediately before every `USB_ReadPMA()` call in
the PCD endpoint ISR, including SETUP, control OUT, single-buffer OUT, and
double-buffer OUT paths. The target uses a 48 MHz HSI system clock, so the NOP
count alone is 1 us of core cycles; loop overhead adds margin.

The USB target builds this local source directly. The unmodified upstream
copy is retained only in the ignored `tmp/bringup` reference tree for
provenance comparison; no dead workaround copy is used by the target.
