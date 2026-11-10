// Shared plain types. No dynamic allocation anywhere in this project, so
// everything here is fixed size and trivially copyable.
#pragma once

#include <cstdint>

namespace ppa {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using i16 = std::int16_t;
using i32 = std::int32_t;
using u64 = std::uint64_t;

// Monotonic time in microseconds since boot.
using TimeUs = u64;

enum class Status : u8 { kOk = 0, kTimeout, kBadId, kNack, kRangeError };

}  // namespace ppa
