#pragma once

// EEPROM PersistentConfig CRC and pack/unpack. (pure C++11, host-testable).

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

/** Computes CRC-16-CCITT (poly 0x1021, init 0xFFFF) over a byte buffer. */
uint16_t computeCrc16(const uint8_t* data, uint8_t len);

/** Packs PersistentConfig into an 8-byte EEPROM frame with magic header + CRC-16. */
void packPersistentConfig(const PersistentConfig& cfg, uint8_t outBytes[8]);

/** Validates magic, version, and CRC-16 of an 8-byte EEPROM frame and unpacks it. */
bool unpackPersistentConfig(const uint8_t inBytes[8], PersistentConfig& outCfg);

} // namespace RobotLogic
