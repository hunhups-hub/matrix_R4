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
    const int32_t mag = peakAbs(cmdLeft, cmdRight);

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

void AutoStallTracker::reset(uint32_t nowMs) {
    cmdStartMs_      = nowMs;
    lastCheckMs_     = nowMs;
    lastWheelMoveMs_ = nowMs;
    prevEncL_        = 0;
    prevEncR_        = 0;
    prevDriveDeg_    = 0;
    hasEncBaseline_  = false;
    hasDegBaseline_  = false;
}

void AutoStallTracker::clearBaselines() {
    hasEncBaseline_ = false;
    hasDegBaseline_ = false;
}

bool AutoStallTracker::checkDue(uint32_t nowMs) {
    if (static_cast<uint32_t>(nowMs - cmdStartMs_) < Config::kAutoStallGraceMs ||
        static_cast<uint32_t>(nowMs - lastCheckMs_) < Config::kAutoStallPollMs) {
        return false;
    }
    lastCheckMs_ = nowMs;
    return true;
}

void AutoStallTracker::markIdle(uint32_t nowMs) {
    clearBaselines();
    lastWheelMoveMs_ = nowMs;
}

bool AutoStallTracker::timedOut(uint32_t nowMs) const {
    return static_cast<uint32_t>(nowMs - lastWheelMoveMs_) >= Config::kAutoStallTimeoutMs;
}

bool AutoStallTracker::update(uint32_t nowMs,
                              bool encOk,
                              int32_t encLeft,
                              int32_t encRight,
                              bool degOk,
                              int32_t driveDeg) {
    if (!encOk && !degOk) {
        return timedOut(nowMs);
    }

    const bool hadAnyBaseline = hasEncBaseline_ || hasDegBaseline_;
    bool       moved          = false;

    if (encOk) {
        if (!hasEncBaseline_) {
            prevEncL_       = encLeft;
            prevEncR_       = encRight;
            hasEncBaseline_ = true;
        } else if (hasEncoderProgressed(prevEncL_, encLeft, Config::kAutoStallMinDeltaDeg) ||
                   hasEncoderProgressed(prevEncR_, encRight, Config::kAutoStallMinDeltaDeg)) {
            moved = true;
        }
    }

    if (degOk) {
        if (!hasDegBaseline_) {
            prevDriveDeg_   = driveDeg;
            hasDegBaseline_ = true;
        } else if (hasEncoderProgressed(prevDriveDeg_, driveDeg, Config::kAutoStallMinDeltaDeg)) {
            moved = true;
        }
    }

    if (moved) {
        if (encOk) {
            prevEncL_ = encLeft;
            prevEncR_ = encRight;
        }
        if (degOk) {
            prevDriveDeg_ = driveDeg;
        }
        lastWheelMoveMs_ = nowMs;
        return false;
    }
    if (!hadAnyBaseline) {
        // First baseline sample captured: start the stall timeout window from now.
        lastWheelMoveMs_ = nowMs;
        return false;
    }
    return timedOut(nowMs);
}

} // namespace RobotLogic
