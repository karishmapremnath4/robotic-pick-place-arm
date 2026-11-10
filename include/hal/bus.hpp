// Hardware abstraction for the three buses the firmware talks over.
//
// The firmware only ever sees these interfaces. On a real board they are
// backed by the microcontroller peripherals. In this project they are backed
// by a simulated sensor board, which is also how faults get injected: the
// simulated bus can be told to start NACKing, or to freeze its data.
//
// Nothing here allocates.
#pragma once

#include "types.hpp"

namespace ppa {

class I2cBus {
public:
    virtual ~I2cBus() = default;
    // Write one register on a 7 bit device address.
    virtual Status write_reg(u8 addr, u8 reg, u8 value) = 0;
    // Burst read starting at reg. Auto increment is assumed.
    virtual Status read_regs(u8 addr, u8 reg, u8* dst, u8 len) = 0;
};

class SpiBus {
public:
    virtual ~SpiBus() = default;
    // Full duplex transfer. The device is selected for the whole transfer.
    virtual Status transfer(const u8* tx, u8* rx, u8 len) = 0;
};

struct CanFrame {
    u16 id;
    u8 dlc;
    u8 data[8];
};

class CanBus {
public:
    virtual ~CanBus() = default;
    virtual Status send(const CanFrame& frame) = 0;
    // Returns false when the receive queue is empty. Never blocks.
    virtual bool receive(CanFrame* out) = 0;
};

// Asynchronous serial port.
//
// Deliberately different in shape from the other three. I2C, SPI and CAN all
// deal in whole transactions or whole frames, so their interfaces can return
// a complete thing. A UART is a byte pipe with no framing at all: it hands
// over however many bytes happen to have arrived, which may be none, may be
// half a message, and may be two and a half messages.
//
// So read() returns a count rather than a Status, and finding the message
// boundaries is the driver's problem. That is the whole difficulty of a UART
// driver and the reason it is worth writing one.
class UartBus {
public:
    virtual ~UartBus() = default;
    // Copies up to len bytes out of the receive buffer. Returns how many were
    // actually available. Never blocks, and zero is a normal answer.
    virtual u8 read(u8* dst, u8 len) = 0;
    virtual Status write(const u8* src, u8 len) = 0;
};

// Monotonic microsecond clock, used for scheduling and for timing the tasks.
class Clock {
public:
    virtual ~Clock() = default;
    virtual TimeUs now_us() const = 0;
};

}  // namespace ppa
