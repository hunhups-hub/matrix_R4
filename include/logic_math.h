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

/** Battery volts -> millivolts, input clamped to 0..25.5 V, rounded to nearest. */
inline uint16_t voltsToMv(float volts) {
    const float bounded = (volts < 0.0f) ? 0.0f : ((volts > 25.5f) ? 25.5f : volts);
    return static_cast<uint16_t>(bounded * 1000.0f + 0.5f);
}

} // namespace RobotLogic
