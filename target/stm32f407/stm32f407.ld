/* Linker script for the STM32F407VG.
 *
 * MEMORY LAYOUT
 *
 *   FLASH   1024 KB at 0x08000000   code, constants, and the initial values
 *                                   of every initialised variable
 *   SRAM     128 KB at 0x20000000   data, bss, and the stack
 *   CCM       64 KB at 0x10000000   deliberately left out, see below
 *
 * The core coupled memory is faster than SRAM and is not on the bus matrix,
 * so DMA cannot reach it. Nothing in this firmware uses DMA yet, but a part
 * of the address map that will silently fail the first time something does is
 * not somewhere to put data by default. It is declared so the size report
 * accounts for it, and nothing is placed there.
 *
 * WHY THE STACK IS AT THE TOP AND GROWS DOWN INTO BSS
 *
 * The Cortex-M stack grows downward from the end of SRAM. There is no
 * hardware guard between the bottom of the stack and the top of .bss, so a
 * deep call chain corrupts static data silently rather than faulting. The
 * _stack_guard symbol below marks a 1 KB band immediately above .bss that
 * nothing should ever write; filling it at startup and checking it later is
 * the cheap way to find out whether the stack ever got that far.
 */

ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  SRAM  (rw)  : ORIGIN = 0x20000000, LENGTH = 128K
  CCM   (rw)  : ORIGIN = 0x10000000, LENGTH = 64K
}

/* Minimum headroom the build must leave for the stack. If data plus bss grow
 * so large that less than this remains, the link FAILS rather than producing
 * an image that runs until it does not. */
_min_stack_size = 8K;

SECTIONS
{
  /* The vector table must be first, because the core fetches the initial
   * stack pointer from address 0 of the image and the reset vector from
   * address 4. */
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } > FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.rodata)
    *(.rodata*)
    /* The C library's own init and fini stubs. Leaving these unplaced does
     * not drop them: the linker puts them at the next free flash address,
     * which is exactly where .data's load image was told to start, and the
     * link then fails on an overlap. */
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
  } > FLASH

  /* Static constructors. __libc_init_array walks these, so the start and end
   * symbols have to exist even when the arrays are empty. */
  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
    . = ALIGN(4);
    /* Only now is flash finished, so only now does .data's load address
     * follow. */
    _etext = .;
  } > FLASH

  /* ARM exception unwinding tables. Discarded, because the build uses
   * -fno-exceptions: keeping them would add kilobytes of flash describing how
   * to unwind through exceptions that can never be thrown. */
  /DISCARD/ :
  {
    *(.ARM.exidx)
    *(.ARM.exidx*)
    *(.ARM.extab*)
  }

  /* Initialised variables live in SRAM at runtime but their values are
   * stored in flash. The reset handler copies them across. */
  .data : AT(_etext)
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > SRAM

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > SRAM

  /* The guard band, and the hard check that the stack has somewhere to live. */
  ._stack_guard :
  {
    . = ALIGN(8);
    _stack_guard_start = .;
    . = . + 1K;
    _stack_guard_end = .;
  } > SRAM

  ._stack_check :
  {
    . = ALIGN(8);
    . = . + _min_stack_size;
  } > SRAM

  _estack = ORIGIN(SRAM) + LENGTH(SRAM);

  /* There is no heap. No .heap section is defined, _sbrk always fails, and
   * any code that calls malloc fails to link rather than failing at runtime
   * in a way that depends on how much memory happened to be free. */
}
