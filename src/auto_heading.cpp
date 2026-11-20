#include "robot_logic.h"

// Auto/teleop heading math, wheel stall detection, heading hold and Auto drive polarity (pure C++11).

namespace RobotLogic {

bool hasEncoderProgressed(int32_t prevCount, int32_t curCount, int32_t minDelta) {
    const int32_t threshold = (minDelta <= 0L) ? 1L : minDelta;
    const int64_t diff      = static_cast<int64_t>(curCount) - static_cast<int64_t>(prevCount);
    const int64_t absDiff   = (diff < 0) ? -diff : diff;
    return absDiff >= static_cast<int64_t>(threshold);
}

int16_t wrapYawDeg(int32_t yawDeg) {
    int32_t r = yawDeg % 360L;
    if (r > 180L) {
        r -= 360L;
    } else if (r < -180L) {
        r += 360L;
    }
    return static_cast<int16_t>(r);
}

int16_t yawErrorDeg(int16_t targetYawDeg, int16_t actualYawDeg) {
    return wrapYawDeg(static_cast<int32_t>(targetYawDeg) - static_cast<int32_t>(actualYawDeg));
}

int16_t snapYawDeg(int16_t yawDeg, int16_t stepDeg) {
    if (stepDeg <= 0) {
        return yawDeg;
    }
    const int32_t step = stepDeg;
    const int32_t v    = yawDeg;
    const int32_t half = step / 2;
    const int32_t q    = (v >= 0) ? ((v + half) / step) : -(((-v) + half) / step);
    return static_cast<int16_t>(q * step);
}

int16_t estimatePivotYawDeg(int16_t leftPower, int16_t rightPower, uint16_t wheelDegs) {
    if (leftPower == 0 || wheelDegs == 0U || leftPower != static_cast<int16_t>(-rightPower)) {
        return 0;
    }
    const int32_t perYawX100 = (Config::kAutoPivotWheelDegPerYawDegX100 > 0)
                                   ? Config::kAutoPivotWheelDegPerYawDegX100
                                   : 1;
    const int32_t mag        = (static_cast<int32_t>(wheelDegs) * 100L + perYawX100 / 2) / perYawX100;
    const int32_t dir        = (rightPower > 0) ? 1L : -1L;
    return wrapYawDeg(mag * dir * Config::kAutoPivotYawSign);
}

AutoMoveKind classifyAutoMove(int16_t leftPower, int16_t rightPower, uint16_t wheelDegs) {
    if (wheelDegs == 0U || leftPower == 0 || rightPower == 0) {
        return AutoMoveKind::Other;
    }
    if (leftPower == rightPower) {
        return AutoMoveKind::Straight;
    }
    if (leftPower == static_cast<int16_t>(-rightPower)) {
        return AutoMoveKind::Pivot;
    }
    return AutoMoveKind::Other;
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

int16_t computeHeadingHoldCorrection(int16_t targetYawDeg, int16_t actualYawDeg) {
    const int32_t err = yawErrorDeg(targetYawDeg, actualYawDeg);
    const int32_t mag = (err < 0) ? -err : err;
    if (mag <= Config::kTeleopHoldDeadbandDeg) {
        return 0;
    }
    int32_t corr = (err * Config::kTeleopHoldKpX10) / 10L;
    if (corr > Config::kTeleopHoldMaxCorrPct) {
        corr = Config::kTeleopHoldMaxCorrPct;
    } else if (corr < -Config::kTeleopHoldMaxCorrPct) {
        corr = -Config::kTeleopHoldMaxCorrPct;
    }
    return static_cast<int16_t>(corr);
}

void applyTurnCorrection(int16_t& left, int16_t& right, int16_t corrPct) {
    if (corrPct == 0 || (left == 0 && right == 0)) {
        return;
    }
    const int32_t absL  = (left < 0) ? -static_cast<int32_t>(left) : left;
    const int32_t absR  = (right < 0) ? -static_cast<int32_t>(right) : right;
    const int32_t bound = (absL > absR) ? absL : absR;

    int32_t l = static_cast<int32_t>(left) - corrPct;
    int32_t r = static_cast<int32_t>(right) + corrPct;
    if (l > bound) {
        r -= (l - bound);
        l = bound;
    } else if (l < -bound) {
        r += (-bound - l);
        l = -bound;
    }
    if (r > bound) {
        l -= (r - bound);
        r = bound;
    } else if (r < -bound) {
        l += (-bound - r);
        r = -bound;
    }
    left  = clamp16(static_cast<int16_t>(clampLong(l, -bound, bound)),
                    Config::kMotorMinPower, Config::kMotorMaxPower);
    right = clamp16(static_cast<int16_t>(clampLong(r, -bound, bound)),
                    Config::kMotorMinPower, Config::kMotorMaxPower);
}

void TeleopHeadingHold::reset() {
    straight_      = false;
    latched_       = false;
    targetYawDeg_  = 0;
    corrPct_       = 0;
    straightSince_ = 0UL;
}

int16_t TeleopHeadingHold::update(bool straightCmd,
                                  bool sampleFresh,
                                  bool sampleValid,
                                  int16_t yawDeg,
                                  uint32_t nowMs) {
    if (!straightCmd) {
        reset();
        return 0;
    }
    if (!straight_) {
        straight_      = true;
        straightSince_ = nowMs;
    }
    if (!sampleFresh) {
        return corrPct_;
    }
    if (!sampleValid) {
        // IMU/STM32 read failed: drop the latch and fail open (no correction).
        latched_ = false;
        corrPct_ = 0;
        return 0;
    }
    if (!latched_) {
        if (static_cast<uint32_t>(nowMs - straightSince_) >= Config::kTeleopHoldLatchDelayMs) {
            targetYawDeg_ = yawDeg;
            latched_      = true;
        }
        corrPct_ = 0;
        return 0;
    }
    corrPct_ = computeHeadingHoldCorrection(targetYawDeg_, yawDeg);
    return corrPct_;
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
