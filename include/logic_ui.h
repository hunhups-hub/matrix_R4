#pragma once

// OLED / LED text and colour helpers with no hardware dependencies (pure C++11).

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

/** 32-bit FNV-1a over a NUL-terminated string, continuing from `seed` (start with 2166136261). */
uint32_t fnv1aHash(const char* str, uint32_t seed);

/** Packs 8-bit R,G,B into 0xRRGGBB. */
uint32_t packRgb(uint8_t r, uint8_t g, uint8_t b);

/** OLED header label for the current system/link/control state, in priority order. */
const char* headerTag(const SystemSnapshot& snap);

/** ' ' Normal, 'L' Low, 'C' Critical, 'E' Emergency. */
char batteryTierChar(BatteryTier tier);

/** '+' state 0, '~' state 1, '!' otherwise. */
char stabFlagChar(uint8_t state);

} // namespace RobotLogic
