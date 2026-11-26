#include "robot_logic.h"

// Control-policy decisions shared by main.cpp (pure C++11).

namespace RobotLogic {

DriveSource selectDriveSource(bool estop,
                              bool ps2Failsafe,
                              bool ps2Operational,
                              bool frameValid,
                              bool crossHeld,
                              bool serialLatched) {
    if (estop) {
        return DriveSource::Estop;
    }
    if (ps2Failsafe) {
        return DriveSource::Failsafe;
    }
    if (ps2Operational && frameValid && crossHeld) {
        return DriveSource::CrossBrake;
    }
    if (serialLatched) {
        return DriveSource::Serial;
    }
    if (ps2Operational) {
        return DriveSource::Ps2;
    }
    return DriveSource::Coast;
}

int16_t computeMaxDriveSpeed(bool precisionMode, int16_t batteryMaxPowerCap) {
    const int16_t modeCap = precisionMode ? Config::kPrecisionMaxPower : Config::kNormalMaxPower;
    return (modeCap < batteryMaxPowerCap) ? modeCap : batteryMaxPowerCap;
}

uint8_t classifyStabPenalty(uint8_t penalty) {
    if (penalty == 0U) {
        return 0U;
    }
    return (penalty <= 10U) ? 1U : 2U;
}

bool advanceControlTick(uint32_t nowMs, uint32_t& lastTickMs, uint32_t& outDtMs) {
    outDtMs = static_cast<uint32_t>(nowMs - lastTickMs);
    if (outDtMs < Config::kControlPeriodMs) {
        return false;
    }
    if (outDtMs >= (Config::kControlPeriodMs * 2UL)) {
        lastTickMs = nowMs;
    } else {
        lastTickMs += Config::kControlPeriodMs;
    }
    return true;
}

} // namespace RobotLogic
