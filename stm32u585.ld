/* =====================================================================
 * STM32U585AI linker script
 * =====================================================================
 *
 * Memory layout
 * -------------
 *   Flash  : 1 MB @ 0x08000000   (only first 256 KB used by default)
 *   SRAM   : 256 KB @ 0x20000000
 *
 * Sections
 * --------
 *   .isr_vector   : vector table at start of flash
 *   .text/.rodata : program + read-only data
 *   .data         : initialised data (copied from flash to SRAM at boot)
 *   .bss          : zero-initialised data
 *   ._user_heap_stack : 8 KB heap used by pvPortMalloc() (heap_4)
 * ===================================================================== */

ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)   : ORIGIN = 0x08000000, LENGTH = 256K
    RAM   (rwx)  : ORIGIN = 0x20000000, LENGTH = 256K
}

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
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } >FLASH

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } >FLASH

    .ARM :
    {
        __exidx_start = .;
        *(.ARM.exidx*)
        __exidx_end = .;
    } >FLASH

    __etext = .;          /* end of .text -> source for .data copy */

    .data : AT(__etext)
    {
        . = ALIGN(4);
        __data_start__ = .;
        *(vtable)
        *(.data)
        *(.data*)

        . = ALIGN(4);
        __data_end__ = .;
    } >RAM AT> FLASH

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        __bss_start__ = .;
        *(.bss)
        *(.bss*)
        *(COMMON)

        . = ALIGN(4);
        __bss_end__ = .;
    } >RAM

    ._user_heap_stack (NOLOAD) :
    {
        . = ALIGN(8);
        PROVIDE ( end = . );
        PROVIDE ( _end = . );
        . = . + 8K;        /* FreeRTOS heap_4 pool */
        . = ALIGN(8);
    } >RAM
}
