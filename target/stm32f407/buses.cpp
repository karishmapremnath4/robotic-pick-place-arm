// The four bus interfaces, against STM32F407 peripheral registers.
//
// This is the file that makes the abstraction real. Everything above it, the
// drivers, the filter, the control loops, the sequence, is unchanged from the
// simulation build. Only what sits beneath these four interfaces differs.
//
// PIN ASSIGNMENT
//
//   I2C1    PB6 SCL, PB7 SDA            AF4, open drain
//   SPI1    PA5 SCK, PA6 MISO, PA7 MOSI AF5, plus PA4 and PC4 as chip selects
//   CAN1    PB8 RX, PB9 TX              AF9
//   USART2  PA2 TX, PA3 RX              AF7
//
// EVERY WAIT HAS A TIMEOUT
//
// Not one loop here spins on a status bit without a bound. A peripheral that
// never raises the flag it is supposed to raise is an ordinary hardware
// failure: a pulled SDA line, an unpowered device, a missing terminator. A
// firmware that waits forever for it does not report a fault, it simply stops,
// and a 1 kHz safety loop that has stopped is the worst outcome available.
//
// That lesson was learned in simulation, where a four line angle wrapping
// function hung the whole firmware on an infinite input. The same discipline
// applies harder here, where the inputs come from outside the chip.
#include <cstdint>

#include "hal/bus.hpp"

extern "C" uint64_t micros_since_boot(void);

namespace ppa {
namespace {

inline volatile uint32_t& reg(uint32_t addr) {
    return *reinterpret_cast<volatile uint32_t*>(addr);
}

constexpr uint32_t kRccBase = 0x40023800u;
constexpr uint32_t kRccAhb1Enr = kRccBase + 0x30;
constexpr uint32_t kRccApb1Enr = kRccBase + 0x40;
constexpr uint32_t kRccApb2Enr = kRccBase + 0x44;

constexpr uint32_t kGpioA = 0x40020000u;
constexpr uint32_t kGpioB = 0x40020400u;
constexpr uint32_t kGpioC = 0x40020800u;

constexpr uint32_t kI2c1 = 0x40005400u;
constexpr uint32_t kSpi1 = 0x40013000u;
constexpr uint32_t kCan1 = 0x40006400u;
constexpr uint32_t kUsart2 = 0x40004400u;

// Bus clocks after SystemInit: AHB 168 MHz, APB1 42 MHz, APB2 84 MHz.
constexpr uint32_t kApb1Hz = 42000000u;
constexpr uint32_t kApb2Hz = 84000000u;

// How long any single peripheral wait may take before it is called a fault.
// 2 ms is far longer than any transfer here needs and far shorter than one
// control period, so a timeout costs a cycle rather than a deadline.
constexpr uint64_t kTimeoutUs = 2000;

// Configure one pin as an alternate function.
void pin_af(uint32_t port, int pin, int af, bool open_drain) {
    reg(port + 0x00) = (reg(port + 0x00) & ~(3u << (pin * 2))) | (2u << (pin * 2));
    reg(port + 0x04) = open_drain ? (reg(port + 0x04) | (1u << pin))
                                  : (reg(port + 0x04) & ~(1u << pin));
    reg(port + 0x08) |= (3u << (pin * 2));                 // very high speed
    reg(port + 0x0C) &= ~(3u << (pin * 2));                // no pull
    const uint32_t afr = port + (pin < 8 ? 0x20 : 0x24);
    const int shift = (pin & 7) * 4;
    reg(afr) = (reg(afr) & ~(0xFu << shift)) | (static_cast<uint32_t>(af) << shift);
}

void pin_output(uint32_t port, int pin) {
    reg(port + 0x00) = (reg(port + 0x00) & ~(3u << (pin * 2))) | (1u << (pin * 2));
    reg(port + 0x08) |= (3u << (pin * 2));
}

inline void pin_set(uint32_t port, int pin, bool high) {
    reg(port + 0x18) = high ? (1u << pin) : (1u << (pin + 16));
}

// Spin on a condition with a bound. Returns false on timeout.
template <typename Fn>
bool wait_for(Fn ready) {
    const uint64_t start = micros_since_boot();
    while (!ready()) {
        if (micros_since_boot() - start > kTimeoutUs) return false;
    }
    return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// I2C1
// ---------------------------------------------------------------------------
class Stm32I2c final : public I2cBus {
public:
    void init() {
        reg(kRccAhb1Enr) |= (1u << 1);          // GPIOB
        reg(kRccApb1Enr) |= (1u << 21);         // I2C1
        pin_af(kGpioB, 6, 4, true);
        pin_af(kGpioB, 7, 4, true);

        reg(kI2c1 + 0x00) = 0;                  // CR1, peripheral off to configure
        reg(kI2c1 + 0x04) = kApb1Hz / 1000000u; // CR2, APB clock in MHz
        // 400 kHz fast mode, duty 2:1. CCR = PCLK / (3 * 400k).
        reg(kI2c1 + 0x1C) = (1u << 15) | (kApb1Hz / (3u * 400000u));
        reg(kI2c1 + 0x20) = (kApb1Hz / 1000000u) * 300u / 1000u + 1u;
        reg(kI2c1 + 0x00) = 1u;                 // enable
    }

    Status write_reg(u8 addr, u8 reg_addr, u8 value) override {
        if (!start()) return Status::kTimeout;
        if (!send_addr(addr, false)) return Status::kNack;
        if (!send_byte(reg_addr)) return Status::kNack;
        if (!send_byte(value)) return Status::kNack;
        stop();
        return Status::kOk;
    }

    Status read_regs(u8 addr, u8 reg_addr, u8* dst, u8 len) override {
        if (len == 0) return Status::kRangeError;

        // Write the register pointer, then a repeated start to read. The
        // high bit of the register address is the auto increment flag on
        // many parts; the driver above sets it where the part needs it, so
        // nothing is added here.
        if (!start()) return Status::kTimeout;
        if (!send_addr(addr, false)) return Status::kNack;
        if (!send_byte(reg_addr)) return Status::kNack;
        if (!start()) return Status::kTimeout;
        if (!send_addr(addr, true)) return Status::kNack;

        for (u8 i = 0; i < len; ++i) {
            const bool last = (i + 1 == len);
            if (last) {
                reg(kI2c1 + 0x00) &= ~(1u << 10);   // NACK the final byte
                reg(kI2c1 + 0x00) |= (1u << 9);     // and schedule the stop
            } else {
                reg(kI2c1 + 0x00) |= (1u << 10);    // ACK
            }
            if (!wait_for([] { return (reg(kI2c1 + 0x14) & (1u << 6)) != 0; }))
                return Status::kTimeout;
            dst[i] = static_cast<u8>(reg(kI2c1 + 0x10));
        }
        return Status::kOk;
    }

private:
    bool start() {
        reg(kI2c1 + 0x00) |= (1u << 8);
        return wait_for([] { return (reg(kI2c1 + 0x14) & 1u) != 0; });
    }
    void stop() { reg(kI2c1 + 0x00) |= (1u << 9); }

    bool send_addr(u8 addr, bool reading) {
        reg(kI2c1 + 0x10) = static_cast<uint32_t>(addr << 1) | (reading ? 1u : 0u);
        const bool ok = wait_for([] {
            const uint32_t sr1 = reg(kI2c1 + 0x14);
            return (sr1 & (1u << 1)) || (sr1 & (1u << 10));  // ADDR or AF
        });
        if (!ok) return false;
        if (reg(kI2c1 + 0x14) & (1u << 10)) {   // the device did not acknowledge
            reg(kI2c1 + 0x14) &= ~(1u << 10);
            stop();
            return false;
        }
        // Reading SR2 is what CLEARS the ADDR flag, so the read has to
        // actually happen. Writing (void)reg(...) does not read a volatile
        // at all, the compiler says so, and the peripheral would then hang
        // on every single transfer waiting for a flag nothing cleared.
        const uint32_t sr2 = reg(kI2c1 + 0x18);
        (void)sr2;
        return true;
    }

    bool send_byte(u8 b) {
        if (!wait_for([] { return (reg(kI2c1 + 0x14) & (1u << 7)) != 0; }))
            return false;
        reg(kI2c1 + 0x10) = b;
        return wait_for([] { return (reg(kI2c1 + 0x14) & (1u << 2)) != 0; });
    }
};

// ---------------------------------------------------------------------------
// SPI1, one instance per chip select
// ---------------------------------------------------------------------------
class Stm32Spi final : public SpiBus {
public:
    Stm32Spi(uint32_t cs_port, int cs_pin) : cs_port_(cs_port), cs_pin_(cs_pin) {}

    static void init_peripheral() {
        reg(kRccAhb1Enr) |= (1u << 0) | (1u << 2);   // GPIOA, GPIOC
        reg(kRccApb2Enr) |= (1u << 12);              // SPI1
        pin_af(kGpioA, 5, 5, false);
        pin_af(kGpioA, 6, 5, false);
        pin_af(kGpioA, 7, 5, false);

        // Mode 1 (CPOL 0, CPHA 1), which is what the AS5047P expects.
        // 84 MHz / 16 = 5.25 MHz, inside the part's 10 MHz limit with margin.
        reg(kSpi1 + 0x00) = (1u << 9) | (1u << 8) |  // software slave select
                            (1u << 2) |              // master
                            (3u << 3) |              // /16
                            (1u << 0);               // CPHA
        reg(kSpi1 + 0x00) |= (1u << 6);              // enable
    }

    void init_cs() {
        pin_output(cs_port_, cs_pin_);
        pin_set(cs_port_, cs_pin_, true);
    }

    Status transfer(const u8* tx, u8* rx, u8 len) override {
        pin_set(cs_port_, cs_pin_, false);
        Status st = Status::kOk;
        for (u8 i = 0; i < len; ++i) {
            if (!wait_for([] { return (reg(kSpi1 + 0x08) & (1u << 1)) != 0; })) {
                st = Status::kTimeout;
                break;
            }
            *reinterpret_cast<volatile uint8_t*>(kSpi1 + 0x0C) = tx[i];
            if (!wait_for([] { return (reg(kSpi1 + 0x08) & 1u) != 0; })) {
                st = Status::kTimeout;
                break;
            }
            rx[i] = *reinterpret_cast<volatile uint8_t*>(kSpi1 + 0x0C);
        }
        // The chip select is released on every path, including the timeout.
        // A select left asserted after a failed transfer locks the bus for
        // every other device on it.
        pin_set(cs_port_, cs_pin_, true);
        return st;
    }

private:
    uint32_t cs_port_;
    int cs_pin_;
};

// ---------------------------------------------------------------------------
// CAN1
// ---------------------------------------------------------------------------
class Stm32Can final : public CanBus {
public:
    void init() {
        reg(kRccAhb1Enr) |= (1u << 1);
        reg(kRccApb1Enr) |= (1u << 25);
        pin_af(kGpioB, 8, 9, false);
        pin_af(kGpioB, 9, 9, false);

        reg(kCan1 + 0x00) &= ~(1u << 1);            // leave sleep
        if (!wait_for([] { return (reg(kCan1 + 0x04) & (1u << 1)) == 0; })) return;
        reg(kCan1 + 0x00) |= 1u;                    // request init
        if (!wait_for([] { return (reg(kCan1 + 0x04) & 1u) != 0; })) return;

        // 1 Mbit/s from 42 MHz APB1: prescaler 3, so 14 MHz, 14 time quanta
        // per bit as 1 + 9 + 4. Sample point at 71 percent, which is what
        // CiA recommends for 1 Mbit.
        reg(kCan1 + 0x1C) = (3u - 1u) | ((9u - 1u) << 16) | ((4u - 1u) << 20);

        // One filter, accepting everything. Filtering in software costs a few
        // microseconds per frame and means a bus that gains a node does not
        // need the firmware reflashed to see it.
        reg(kCan1 + 0x200) |= 1u;                   // filter init
        reg(kCan1 + 0x20C) = 0;                     // filter 0 id
        reg(kCan1 + 0x210) = 0;                     // mask: care about nothing
        reg(kCan1 + 0x200) &= ~1u;
        reg(kCan1 + 0x21C) |= 1u;                   // activate filter 0

        reg(kCan1 + 0x00) &= ~1u;                   // leave init
        (void)wait_for([] { return (reg(kCan1 + 0x04) & 1u) == 0; });
    }

    Status send(const CanFrame& frame) override {
        // Bus off is a real state the controller enters after too many
        // errors, and it is reported rather than retried: the layer above
        // counts transmit failures and the supervisor acts on them.
        if (reg(kCan1 + 0x18) & (1u << 2)) return Status::kTimeout;

        int mb = -1;
        for (int i = 0; i < 3; ++i)
            if (reg(kCan1 + 0x08) & (1u << (26 + i))) { mb = i; break; }
        if (mb < 0) return Status::kTimeout;

        const uint32_t base = kCan1 + 0x180 + static_cast<uint32_t>(mb) * 0x10;
        reg(base + 0x04) = frame.dlc & 0xFu;
        uint32_t lo = 0, hi = 0;
        for (int i = 0; i < 4 && i < frame.dlc; ++i)
            lo |= static_cast<uint32_t>(frame.data[i]) << (8 * i);
        for (int i = 4; i < 8 && i < frame.dlc; ++i)
            hi |= static_cast<uint32_t>(frame.data[i]) << (8 * (i - 4));
        reg(base + 0x08) = lo;
        reg(base + 0x0C) = hi;
        reg(base + 0x00) = (static_cast<uint32_t>(frame.id) << 21) | 1u;  // TXRQ
        return Status::kOk;
    }

    bool receive(CanFrame* out) override {
        if ((reg(kCan1 + 0x0C) & 3u) == 0) return false;   // FIFO 0 empty

        const uint32_t rir = reg(kCan1 + 0x1B0);
        const uint32_t rdtr = reg(kCan1 + 0x1B4);
        const uint32_t lo = reg(kCan1 + 0x1B8);
        const uint32_t hi = reg(kCan1 + 0x1BC);

        out->id = static_cast<u16>((rir >> 21) & 0x7FFu);
        out->dlc = static_cast<u8>(rdtr & 0xFu);
        for (int i = 0; i < 4; ++i) out->data[i] = static_cast<u8>(lo >> (8 * i));
        for (int i = 0; i < 4; ++i) out->data[4 + i] = static_cast<u8>(hi >> (8 * i));

        reg(kCan1 + 0x0C) |= (1u << 5);                    // release the mailbox
        return true;
    }
};

// ---------------------------------------------------------------------------
// USART2
// ---------------------------------------------------------------------------
class Stm32Uart final : public UartBus {
public:
    void init() {
        reg(kRccAhb1Enr) |= (1u << 0);
        reg(kRccApb1Enr) |= (1u << 17);
        pin_af(kGpioA, 2, 7, false);
        pin_af(kGpioA, 3, 7, false);

        reg(kUsart2 + 0x08) = kApb1Hz / 115200u;           // BRR
        reg(kUsart2 + 0x0C) = (1u << 13) | (1u << 3) | (1u << 2);  // UE, TE, RE
    }

    // Non blocking by construction: it reads what has arrived and returns.
    // A UART driver that waits for a byte is a UART driver that stops the
    // control loop whenever the cable is unplugged.
    u8 read(u8* dst, u8 len) override {
        u8 n = 0;
        while (n < len && (reg(kUsart2 + 0x00) & (1u << 5)))
            dst[n++] = static_cast<u8>(reg(kUsart2 + 0x04));
        return n;
    }

    Status write(const u8* src, u8 len) override {
        for (u8 i = 0; i < len; ++i) {
            if (!wait_for([] { return (reg(kUsart2 + 0x00) & (1u << 7)) != 0; }))
                return Status::kTimeout;
            reg(kUsart2 + 0x04) = src[i];
        }
        return Status::kOk;
    }
};

// ---------------------------------------------------------------------------
// The cycle counter, as the firmware's clock.
// ---------------------------------------------------------------------------
class Stm32Clock final : public Clock {
public:
    TimeUs now_us() const override { return micros_since_boot(); }
};

// Static storage. Nothing here is allocated, because nothing can be: there is
// no heap and _sbrk always fails.
Stm32I2c g_i2c;
Stm32Spi g_spi_enc1(kGpioA, 4);
Stm32Spi g_spi_enc2(kGpioC, 4);
Stm32Can g_can;
Stm32Uart g_uart;
Stm32Clock g_clock;

void init_buses() {
    g_i2c.init();
    Stm32Spi::init_peripheral();
    g_spi_enc1.init_cs();
    g_spi_enc2.init_cs();
    g_can.init();
    g_uart.init();
}

I2cBus& imu_bus() { return g_i2c; }
SpiBus& encoder1_bus() { return g_spi_enc1; }
SpiBus& encoder2_bus() { return g_spi_enc2; }
CanBus& can_bus() { return g_can; }
UartBus& laser_bus() { return g_uart; }
Clock& system_clock() { return g_clock; }

}  // namespace ppa
