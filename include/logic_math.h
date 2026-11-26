#pragma once

// Small inline math helpers shared by the pure logic files (C++11, no Arduino headers).

#include <stdint.h>

namespace RobotLogic {

/** |v| in 32 bits; safe for any int16_t input (never overflows). */
inline int32_t abs32(int32_t v) {
    return (v < 0L) ? -v : v;
}

/** max(|a|, |b|): peak magnitude of a wheel pair. */
inline int32_t peakAbs(int32_t a, int32_t b) {
    const int32_t absA = abs32(a);
    const int32_t absB = abs32(b);
    return (absA > absB) ? absA : absB;
}

/** cur + add, saturated at cap (all in 16 bits, so a uint8_t counter never wraps). */
inline uint8_t satAddCap(uint8_t cur, uint16_t add, uint8_t cap) {
    const uint16_t next = static_cast<uint16_t>(cur + add);
    return (next > cap) ? cap : static_cast<uint8_t>(next);
}

/** cur - sub, floored at 0. */
inline uint8_t satSub(uint8_t cur, uint8_t sub) {
    return (cur > sub) ? static_cast<uint8_t>(cur - sub) : 0U;
}

/** Battery volts -> millivolts, input clamped to 0..25.5 V, rounded to nearest. */
inline uint16_t voltsToMv(float volts) {
    const float bounded = (volts < 0.0f) ? 0.0f : ((volts > 25.5f) ? 25.5f : volts);
    return static_cast<uint16_t>(bounded * 1000.0f + 0.5f);
}

} // namespace RobotLogic
