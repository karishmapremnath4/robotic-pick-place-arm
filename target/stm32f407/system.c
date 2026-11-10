/* Clock setup and the microsecond time base.
 *
 * Out of reset the F407 runs from its internal 16 MHz oscillator. Getting to
 * 168 MHz means running the external crystal through the PLL, and the order
 * matters: the flash wait states and the bus prescalers have to be right
 * BEFORE the core speeds up, or the first instruction fetched at the new
 * speed fails.
 *
 *   8 MHz crystal / M=8 = 1 MHz -> PLL N=336 -> 336 MHz / P=2 = 168 MHz
 *
 * At 168 MHz and 3.3 V the flash needs 5 wait states, AHB runs at 168, APB1
 * is limited to 42 so it gets /4, and APB2 to 84 so it gets /2.
 */

#include <stdint.h>

#define RCC_BASE     0x40023800u
#define RCC_CR       (*(volatile uint32_t *)(RCC_BASE + 0x00))
#define RCC_PLLCFGR  (*(volatile uint32_t *)(RCC_BASE + 0x04))
#define RCC_CFGR     (*(volatile uint32_t *)(RCC_BASE + 0x08))
#define FLASH_ACR    (*(volatile uint32_t *)0x40023C00u)

#define PWR_BASE     0x40007000u
#define PWR_CR       (*(volatile uint32_t *)(PWR_BASE + 0x00))

#define DWT_CTRL     (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT   (*(volatile uint32_t *)0xE0001004u)
#define DEMCR        (*(volatile uint32_t *)0xE000EDF0u)
#define SCB_CPACR    (*(volatile uint32_t *)0xE000ED88u)

#define CPU_HZ 168000000u

/* Bounded wait on a status bit.
 *
 * Nothing in reset code may spin forever. A crystal that never starts is an
 * ordinary hardware failure: a cracked part, a bad load capacitor, a board
 * assembled without it. A firmware that waits for it has not failed safe, it
 * has simply never started, and from outside that is indistinguishable from
 * a dead board.
 *
 * The count is iterations rather than time, because this runs before any
 * clock is trustworthy enough to measure with. 100000 at the 16 MHz internal
 * oscillator is several milliseconds, far longer than the few hundred
 * microseconds a crystal needs.
 */
static int wait_bit(volatile uint32_t *r, uint32_t mask, int want)
{
    for (int i = 0; i < 100000; ++i)
        if (((*r & mask) != 0) == (want != 0)) return 1;
    return 0;
}

/* Non zero once the PLL is actually running. If the crystal never arrived,
 * the part keeps running from its internal oscillator at 16 MHz and this
 * reads zero, so everything downstream can scale its idea of time instead of
 * silently being ten times out. */
static int g_pll_locked;

int clock_is_external(void) { return g_pll_locked; }

void SystemInit(void)
{
    /* The FPU is off at reset. Enabling it before anything else matters,
     * because this firmware is full of float arithmetic and without it every
     * one of those instructions is an undefined instruction fault. */
    SCB_CPACR |= (0xFu << 20);

    RCC_CR |= (1u << 16);                       /* HSE on */
    if (!wait_bit(&RCC_CR, 1u << 17, 1)) {
        /* No crystal. Stay on the internal oscillator rather than hang, and
         * leave g_pll_locked at zero so cpu_hz() reports the truth. */
        return;
    }

    RCC_CR |= (1u << 28);                       /* power interface clock */
    PWR_CR |= (3u << 14);                       /* scale 1, needed >144  */

    /* Wait states and prescalers first, while still slow. */
    FLASH_ACR = (1u << 10) | (1u << 9) | (1u << 8) | 5u;  /* caches + 5 WS */
    RCC_CFGR = (0u << 4) | (5u << 10) | (4u << 13);       /* AHB/1 APB1/4 APB2/2 */

    /* M=8, N=336, P=2 (encoded 0), Q=7, source HSE */
    RCC_PLLCFGR = 8u | (336u << 6) | (0u << 16) | (1u << 22) | (7u << 24);

    RCC_CR |= (1u << 24);                       /* PLL on */
    if (!wait_bit(&RCC_CR, 1u << 25, 1)) return;

    RCC_CFGR |= 2u;                             /* switch to PLL */
    for (int i = 0; i < 100000; ++i)
        if (((RCC_CFGR >> 2) & 3u) == 2u) { g_pll_locked = 1; break; }
    if (!g_pll_locked) return;

    /* The cycle counter is the only honest time source on this part: it
     * counts core clocks with no prescaler and no interrupt, so it measures
     * what a task actually cost rather than what a timer rounded it to.
     * This is what replaces the host clock the scheduler uses in simulation. */
    DEMCR |= (1u << 24);
    DWT_CYCCNT = 0;
    DWT_CTRL |= 1u;
}

/* The real core frequency, not the intended one. Running at 16 MHz while
 * every timing calculation assumes 168 would put the control loop out by a
 * factor of ten and the scheduler would report nonsense. */
uint32_t cpu_hz(void) { return g_pll_locked ? CPU_HZ : 16000000u; }

/* The raw counter, for the scheduler's execution timing. */
uint32_t dwt_cycles(void) { return DWT_CYCCNT; }

uint64_t micros_since_boot(void)
{
    /* The 32 bit cycle counter wraps every 25.6 seconds at 168 MHz, which is
     * far too short for a monotonic clock, so the wraps are counted. Only
     * safe while this is called more often than once per wrap, which the
     * 1 kHz control loop guarantees many times over. */
    static uint32_t last;
    static uint64_t high;
    const uint32_t now = DWT_CYCCNT;
    if (now < last) high += 0x100000000ull;
    last = now;
    return (high + now) / (CPU_HZ / 1000000u);
}
