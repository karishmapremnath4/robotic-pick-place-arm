/* Reset handler, vector table and C library stubs for the STM32F407.
 *
 * WHAT HAPPENS BEFORE main()
 *
 * The core fetches the initial stack pointer from the first word of the image
 * and the reset vector from the second, then starts executing. At that moment
 * no global is initialised, .bss is whatever was in SRAM at power on, and the
 * C runtime does not exist. Everything main() takes for granted is set up
 * here, in this order:
 *
 *   1  copy .data from flash into SRAM, so initialised globals have values
 *   2  zero .bss, so uninitialised globals are actually zero
 *   3  fill the stack guard band with a known pattern
 *   4  run the static constructors
 *   5  call main
 *
 * Step 2 is the one people assume the hardware does. It does not.
 */

#include <stdint.h>

extern uint32_t _etext, _sdata, _edata, _sbss, _ebss, _estack;
extern uint32_t _stack_guard_start, _stack_guard_end;

extern int main(void);
extern void __libc_init_array(void);
void SystemInit(void);

/* The pattern written into the guard band. Checked later by
 * stack_guard_untouched(); if any of it has changed, the stack reached data
 * it had no business reaching. */
#define GUARD_WORD 0xC0FFEE00u

void Reset_Handler(void)
{
    uint32_t *src = &_etext;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) *dst++ = *src++;

    dst = &_sbss;
    while (dst < &_ebss) *dst++ = 0;

    for (dst = &_stack_guard_start; dst < &_stack_guard_end; ++dst)
        *dst = GUARD_WORD;

    SystemInit();
    __libc_init_array();
    main();

    /* main() must not return on a microcontroller: there is nothing to
     * return to. If it does, stop here rather than executing whatever
     * follows in flash. */
    for (;;) { }
}

/* How much of the guard band is still intact, in bytes. A firmware that
 * reports less than the full band at any point has had a stack excursion,
 * and the number says how close it came. */
uint32_t stack_guard_intact_bytes(void)
{
    uint32_t n = 0;
    for (const uint32_t *p = &_stack_guard_start; p < &_stack_guard_end; ++p) {
        if (*p != GUARD_WORD) break;
        n += 4;
    }
    return n;
}

static void Default_Handler(void)
{
    /* An interrupt fired that nothing was written to handle. Looping here is
     * deliberate: it is a bug, and a silent return would hide it. */
    for (;;) { }
}

/* A hard fault captures the stacked return address, which is the single most
 * useful thing to know afterwards. Kept in a global so a debugger, or the
 * next boot, can read it. */
volatile uint32_t g_fault_pc;
volatile uint32_t g_fault_lr;
volatile uint32_t g_fault_psr;

void HardFault_Handler(void) __attribute__((naked));
void HardFault_Handler(void)
{
    __asm volatile(
        "tst lr, #4            \n"  /* which stack was in use */
        "ite eq                \n"
        "mrseq r0, msp         \n"
        "mrsne r0, psp         \n"
        "ldr r1, [r0, #24]     \n"  /* stacked PC  */
        "ldr r2, [r0, #20]     \n"  /* stacked LR  */
        "ldr r3, [r0, #28]     \n"  /* stacked PSR */
        "ldr r4, =g_fault_pc   \n"
        "str r1, [r4]          \n"
        "ldr r4, =g_fault_lr   \n"
        "str r2, [r4]          \n"
        "ldr r4, =g_fault_psr  \n"
        "str r3, [r4]          \n"
        "b .                   \n");
}

void NMI_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void)  __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void)        __attribute__((weak, alias("Default_Handler")));
void DebugMon_Handler(void)   __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void)     __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void)    __attribute__((weak, alias("Default_Handler")));

/* Only the core exceptions are listed. The F407 has 82 peripheral interrupts
 * after these, all pointing at Default_Handler, and listing every one by name
 * would be 82 lines saying nothing. They are filled in as they are used. */
/* The first entry is the initial stack pointer, not a function, which is
 * what the hardware expects and what ISO C objects to. The diagnostic is
 * silenced here rather than globally so it still applies everywhere else. */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
__attribute__((section(".isr_vector"), used))
void (* const g_vectors[])(void) = {
    (void (*)(void))&_estack,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
};
#pragma GCC diagnostic pop

/* ------------------------------------------------------------------------
 * C library stubs.
 *
 * _sbrk ALWAYS FAILS, and that is the point. There is no heap section in the
 * linker script, so any call to malloc, new, std::vector or std::string ends
 * up here and gets a failure. That turns "this firmware does not allocate"
 * from a claim into something the build enforces.
 * ---------------------------------------------------------------------- */
void *_sbrk(int incr) { (void)incr; return (void *)-1; }
int _write(int fd, char *buf, int len) { (void)fd; (void)buf; return len; }
int _read(int fd, char *buf, int len) { (void)fd; (void)buf; (void)len; return 0; }
int _close(int fd) { (void)fd; return -1; }
int _lseek(int fd, int pos, int whence) { (void)fd; (void)pos; (void)whence; return 0; }
int _fstat(int fd, void *st) { (void)fd; (void)st; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
void _exit(int code) { (void)code; for (;;) { } }
