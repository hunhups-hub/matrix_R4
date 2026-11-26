#include "robot_logic.h"

// EEPROM PersistentConfig CRC-16-CCITT pack/unpack (pure C++11).

namespace RobotLogic {

namespace {

// EEPROM flag byte layout (byte 3). Bit 2 (precisionMode) is stored but ignored on load.
constexpr uint8_t kFlagLeftReversed  = 0x01U;
constexpr uint8_t kFlagRightReversed = 0x02U;
constexpr uint8_t kFlagPrecisionMode = 0x04U;
constexpr uint8_t kFlagRc2Smooth     = 0x08U;

} // namespace

uint16_t computeCrc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFFU;
    if (data == nullptr) {
        return crc;
    }
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

void packPersistentConfig(const PersistentConfig& cfg, uint8_t outBytes[8]) {
    if (outBytes == nullptr) {
        return;
    }
    uint8_t flags = 0U;
    if (cfg.leftReversed)     flags |= kFlagLeftReversed;
    if (cfg.rightReversed)    flags |= kFlagRightReversed;
    if (cfg.precisionMode)    flags |= kFlagPrecisionMode;
    if (cfg.rc2SmoothEnabled) flags |= kFlagRc2Smooth;

    const int8_t clampedTrim = stepMotorTrim(cfg.motorTrimPct, 0);

    outBytes[0] = Config::kEepromMagic0;
    outBytes[1] = Config::kEepromMagic1;
    outBytes[2] = Config::kEepromVersion;
    outBytes[3] = flags;
    outBytes[4] = static_cast<uint8_t>(clampedTrim);
    outBytes[5] = 0U;

    const uint16_t crc = computeCrc16(outBytes, 6U);
    outBytes[6] = static_cast<uint8_t>(crc & 0xFFU);
    outBytes[7] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
}

bool unpackPersistentConfig(const uint8_t inBytes[8], PersistentConfig& outCfg) {
    if (inBytes == nullptr) {
        return false;
    }
    if (inBytes[0] != Config::kEepromMagic0 ||
        inBytes[1] != Config::kEepromMagic1 ||
        inBytes[2] != Config::kEepromVersion ||
        inBytes[5] != 0U) {
        return false;
    }
    const uint16_t expectedCrc = computeCrc16(inBytes, 6U);
    const uint16_t storedCrc   = static_cast<uint16_t>(inBytes[6]) |
                                 (static_cast<uint16_t>(inBytes[7]) << 8U);
    if (expectedCrc != storedCrc) {
        return false;
    }

    const uint8_t flags = inBytes[3];
    if ((flags & 0xF0U) != 0U) {
        return false;
    }

    const int8_t rawTrim = static_cast<int8_t>(inBytes[4]);
    if (rawTrim < -Config::kMaxMotorTrimPct || rawTrim > Config::kMaxMotorTrimPct) {
        return false;
    }

    outCfg.leftReversed     = (flags & kFlagLeftReversed) != 0U;
    outCfg.rightReversed    = (flags & kFlagRightReversed) != 0U;
    outCfg.precisionMode    = (flags & kFlagPrecisionMode) != 0U;
    outCfg.rc2SmoothEnabled = (flags & kFlagRc2Smooth) != 0U;
    outCfg.motorTrimPct     = rawTrim;
    return true;
}

} // namespace RobotLogic
