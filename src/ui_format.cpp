#include "robot_logic.h"
#include "logic_ui.h"

namespace RobotLogic {

uint32_t fnv1aHash(const char* str, uint32_t seed) {
    uint32_t h = seed;
    while (*str != '\0') {
        h ^= static_cast<uint8_t>(*str++);
        h *= 16777619UL;
    }
    return h;
}

uint32_t packRgb(uint8_t r, uint8_t g, uint8_t b) {
    return (static_cast<uint32_t>(r) << 16) |
           (static_cast<uint32_t>(g) << 8) |
           static_cast<uint32_t>(b);
}

const char* headerTag(const SystemSnapshot& snap) {
    if (snap.comboHolding) {
        return "HOLD+DN";
    }
    if (snap.sysState == SystemState::Estop) {
        return "ESTOP!";
    }
    if (snap.activeSource == ControlSource::Auto) {
        return "AUTO RUN";
    }
    if (snap.ps2Link == LinkState::Operational) {
        return "PS2 OK";
    }
    if (snap.ps2Link == LinkState::Stabilizing) {
        return "PS2 SYNC";
    }
    if (snap.ps2Link == LinkState::Failsafe) {
        return "FAILSAFE";
    }
    if (snap.activeSource == ControlSource::SerialCmd) {
        return "SER OK";
    }
    return "WAIT PS2";
}

char batteryTierChar(BatteryTier tier) {
    switch (tier) {
        case BatteryTier::Low:       return 'L';
        case BatteryTier::Critical:  return 'C';
        case BatteryTier::Emergency: return 'E';
        case BatteryTier::Normal:
        default:                     return ' ';
    }
}

char stabFlagChar(uint8_t state) {
    switch (state) {
        case 0U: return '+';
        case 1U: return '~';
        default: return '!';
    }
}

} // namespace RobotLogic
