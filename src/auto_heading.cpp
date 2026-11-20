#include "robot_logic.h"

// Wheel stall detection and Auto drive polarity (pure C++11).

namespace RobotLogic {

bool hasEncoderProgressed(int32_t prevCount, int32_t curCount, int32_t minDelta) {
    const int32_t threshold = (minDelta <= 0L) ? 1L : minDelta;
    const int64_t diff      = static_cast<int64_t>(curCount) - static_cast<int64_t>(prevCount);
    const int64_t absDiff   = (diff < 0) ? -diff : diff;
    return absDiff >= static_cast<int64_t>(threshold);
}

void ManualStallDetector::reset() {
    tripped_    = false;
    hasBase_    = false;
    prevLeft_   = 0;
    prevRight_  = 0;
    lastMoveMs_ = 0UL;
}

bool ManualStallDetector::update(int16_t cmdLeft,
                                 int16_t cmdRight,
                                 bool sampleFresh,
                                 bool sampleValid,
                                 int32_t encLeft,
                                 int32_t encRight,
                                 uint32_t nowMs) {
    const int32_t absL = (cmdLeft < 0) ? -static_cast<int32_t>(cmdLeft) : cmdLeft;
    const int32_t absR = (cmdRight < 0) ? -static_cast<int32_t>(cmdRight) : cmdRight;
    const int32_t mag  = (absL > absR) ? absL : absR;

    if (mag <= Config::kManualStallRearmCmdPct) {
        reset();
        lastMoveMs_ = nowMs;
        return false;
    }
    if (tripped_) {
        return true;
    }
    if (mag < Config::kManualStallMinCmdPct) {
        hasBase_    = false;
        lastMoveMs_ = nowMs;
        return false;
    }
    if (sampleFresh) {
        if (!sampleValid) {
            hasBase_    = false;
            lastMoveMs_ = nowMs;
            return false;
        }
        if (!hasBase_) {
            prevLeft_   = encLeft;
            prevRight_  = encRight;
            hasBase_    = true;
            lastMoveMs_ = nowMs;
        } else if (hasEncoderProgressed(prevLeft_, encLeft) ||
                   hasEncoderProgressed(prevRight_, encRight)) {
            prevLeft_   = encLeft;
            prevRight_  = encRight;
            lastMoveMs_ = nowMs;
        }
    }
    if (hasBase_ && static_cast<uint32_t>(nowMs - lastMoveMs_) >= Config::kManualStallTimeoutMs) {
        tripped_ = true;
    }
    return tripped_;
}

void resolveAutoDrivePolarity(uint8_t leftMotorID,
                              uint8_t rightMotorID,
                              bool savedLeftReversed,
                              bool savedRightReversed,
                              bool& ioLeftReverse,
                              bool& ioRightReverse) {
    if (leftMotorID != Config::kMotorLeftId || rightMotorID != Config::kMotorRightId) {
        return;
    }
    const bool leftDelta  = (savedLeftReversed != Config::kMotorLeftReverseInit);
    const bool rightDelta = (savedRightReversed != Config::kMotorRightReverseInit);
    ioLeftReverse         = (ioLeftReverse != leftDelta);
    ioRightReverse        = (ioRightReverse != rightDelta);
}

} // namespace RobotLogic
