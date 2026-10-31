/* Reviewed for STM32C071FBP6: 128 KiB flash and 24 KiB SRAM. */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 124K
    SETTINGS (r) : ORIGIN = 0x0801F000, LENGTH = 4K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 24K
}

/* The Cortex-M0+ stack grows down from the end of SRAM. */
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x1000; /* Plan v2: 4 KiB, separate from static RAM. */
_estack = ORIGIN(RAM) + LENGTH(RAM);
__StackTop = _estack;
__StackLimit = __StackTop - _Min_Stack_Size;

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text.Reset_Handler)
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } >FLASH

    .preinit_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .init_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
        . = ALIGN(4);
    } >FLASH

    .fini_array :
    {
        . = ALIGN(4);
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
        . = ALIGN(4);
    } >FLASH

    _sidata = LOADADDR(.data);
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >RAM AT>FLASH

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
    } >RAM

    .heap (NOLOAD) :
    {
        __HeapBase = .;
        . = . + _Min_Heap_Size;
        __HeapLimit = .;
    } >RAM

    .noinit (NOLOAD) :
    {
        . = ALIGN(8);
        *(.noinit*)
        . = ALIGN(8);
        __static_end = .;
    } >RAM

    /* Place the reserved stack at the top of RAM after all initialized data. */
    .stack __StackLimit (NOLOAD) :
    {
        . = ALIGN(8);
        __StackLimit = .;
        . = . + _Min_Stack_Size;
        . = ALIGN(8);
        __StackTop = .;
    } >RAM
}

ASSERT(_estack == ORIGIN(RAM) + LENGTH(RAM), "stack top is not at RAM end")
ASSERT(__StackTop == _estack, "stack reservation does not reach RAM end")
ASSERT(_ebss <= __StackLimit, "RAM overflow: data and bss overlap stack")
ASSERT(SIZEOF(.heap) == 0, "heap allocation is not permitted")
/* Legacy diagnostics keep their original gates. The integrated controller has
 * a bounded 3 KiB statistics ring and explicit additional live state. Its
 * profile still reserves 4 KiB stack plus at least 1 KiB unused SRAM. */
ASSERT(__static_end - ORIGIN(RAM) <= (DEFINED(__hybrid_release) ? 19K : 14K),
       "static RAM exceeds 14 KiB diagnostic / 19 KiB hybrid budget")
ASSERT(__StackLimit - __static_end >= (DEFINED(__hybrid_release) ? 1K : 6K),
       "insufficient uncommitted SRAM for selected profile")
/* Persistent mask support grows the actuator diagnostic from 48,724 to about
 * 50,652 B. Reviewed 52 KiB flash goal; its RAM/stack budgets remain unchanged. */
ASSERT(LOADADDR(.data) + SIZEOF(.data) - ORIGIN(FLASH) <=
       (DEFINED(__hybrid_release) ? 112K : (DEFINED(__actuator_diagnostic) ? 52K : 48K)),
       "loadable image exceeds reviewed profile flash goal")
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(FLASH) + LENGTH(FLASH),
       "FLASH overflow: data load image exceeds flash")
