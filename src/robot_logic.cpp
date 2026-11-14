#include "robot_logic.h"

namespace RobotLogic {

namespace {

int16_t abs16(int16_t v) {
    return (v < 0) ? static_cast<int16_t>(-v) : v;
}

void desaturateAndClampDrive(int16_t& left, int16_t& right, int16_t maxSpeed) {
    const int16_t limit = clamp16(maxSpeed, 0, Config::kMotorMaxPower);
    if (limit == 0) {
        left  = 0;
        right = 0;
        return;
    }

    const int16_t peak = (abs16(left) > abs16(right)) ? abs16(left) : abs16(right);
    if (peak > limit) {
        left  = static_cast<int16_t>((static_cast<int32_t>(left) * limit) / peak);
        right = static_cast<int16_t>((static_cast<int32_t>(right) * limit) / peak);
    }

    left  = clamp16(left, static_cast<int16_t>(-limit), limit);
    right = clamp16(right, static_cast<int16_t>(-limit), limit);
}

} // namespace

int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

long clampLong(long v, long lo, long hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int16_t computeAxisWithDeadzone(uint8_t raw, uint8_t deadzoneRaw, bool invert) {
    if (deadzoneRaw >= 127U) {
        return 0;
    }

    const int16_t lowEdge  = static_cast<int16_t>(127U - deadzoneRaw);
    const int16_t highEdge = static_cast<int16_t>(128U + deadzoneRaw);
    const int16_t span     = lowEdge;

    int16_t normalized = 0;
    if (static_cast<int16_t>(raw) <= lowEdge) {
        const int32_t dist = static_cast<int32_t>(lowEdge - static_cast<int16_t>(raw));
        normalized         = static_cast<int16_t>(-(dist * Config::kMotorMaxPower) / span);
    } else if (static_cast<int16_t>(raw) >= highEdge) {
        const int32_t dist = static_cast<int32_t>(static_cast<int16_t>(raw) - highEdge);
        normalized         = static_cast<int16_t>((dist * Config::kMotorMaxPower) / span);
    } else {
        return 0;
    }

    if (invert) {
        normalized = static_cast<int16_t>(-normalized);
    }
    return clamp16(normalized, Config::kMotorMinPower, Config::kMotorMaxPower);
}

int16_t applyAsymmetricSlewStep(int16_t current,
                                int16_t target,
                                int16_t accelStep,
                                int16_t decelStep) {
    if (current == target) {
        return target;
    }

    const bool accelerating =
        (current >= 0 && target > current) || (current <= 0 && target < current);
    const int16_t step = accelerating ? accelStep : decelStep;
    if (step <= 0) {
        return target;
    }

    // Pass cleanly through 0 on direction reversal to limit current spikes
    if ((current > 0 && target < 0 && current <= step) ||
        (current < 0 && target > 0 && -current <= step)) {
        return 0;
    }

    if (target > current) {
        const int16_t stepped = static_cast<int16_t>(current + step);
        return (stepped > target) ? target : stepped;
    }

    const int16_t stepped = static_cast<int16_t>(current - step);
    return (stepped < target) ? target : stepped;
}

DriveOutput applyDriveTargetsWithSlew(int16_t targetLeft,
                                        int16_t targetRight,
                                        int16_t currentLeft,
                                        int16_t currentRight,
                                        int16_t maxSpeed,
                                        int16_t accelSlewStep,
                                        int16_t decelSlewStep,
                                        uint8_t compPercent) {
    const int16_t cappedComp = clamp16(static_cast<int16_t>(compPercent),
                                       100,
                                       static_cast<int16_t>(Config::kBattCompMaxPct));
    int16_t scaledL = targetLeft;
    int16_t scaledR = targetRight;
    if (cappedComp > 100) {
        scaledL = static_cast<int16_t>((static_cast<int32_t>(targetLeft) * cappedComp) / 100);
        scaledR = static_cast<int16_t>((static_cast<int32_t>(targetRight) * cappedComp) / 100);
    }
    desaturateAndClampDrive(scaledL, scaledR, maxSpeed);

    DriveOutput out;
    out.left  = applyAsymmetricSlewStep(currentLeft, scaledL, accelSlewStep, decelSlewStep);
    out.right = applyAsymmetricSlewStep(currentRight, scaledR, accelSlewStep, decelSlewStep);
    return out;
}

DriveOutput computeDifferentialDrive(uint8_t lyRaw,
                                     uint8_t rxRaw,
                                     int16_t currentLeft,
                                     int16_t currentRight,
                                     int16_t maxSpeed,
                                     int16_t accelSlewStep,
                                     int16_t decelSlewStep,
                                     uint8_t compPercent) {
    // PS2 LY=0 is stick pushed UP (forward) -> invert=true
    const int16_t throttle = computeAxisWithDeadzone(lyRaw, Config::kStickDeadzoneRaw, true);
    const int16_t turn     = computeAxisWithDeadzone(rxRaw, Config::kStickDeadzoneRaw, false);

    const int16_t targetLeft  = static_cast<int16_t>(throttle + turn);
    const int16_t targetRight = static_cast<int16_t>(throttle - turn);
    return applyDriveTargetsWithSlew(targetLeft,
                                     targetRight,
                                     currentLeft,
                                     currentRight,
                                     maxSpeed,
                                     accelSlewStep,
                                     decelSlewStep,
                                     compPercent);
}

int16_t computeArmStep(int16_t currentDeg, bool upPressed, bool downPressed) {
    if (upPressed == downPressed) {
        return clamp16(currentDeg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    }
    const int16_t dirStep = Config::kRc2InvertDir
                                ? static_cast<int16_t>(-Config::kRc2StepDeg)
                                : Config::kRc2StepDeg;
    const int16_t delta   = upPressed ? dirStep : static_cast<int16_t>(-dirStep);
    return clamp16(static_cast<int16_t>(currentDeg + delta),
                   Config::kRc2MinDeg,
                   Config::kRc2MaxDeg);
}

ServoTargets clampServoTargets(const ServoTargets& targets) {
    ServoTargets clamped;
    clamped.rc1Deg = clamp16(targets.rc1Deg, Config::kServoMinDeg, Config::kServoMaxDeg);
    clamped.rc2Deg = clamp16(targets.rc2Deg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    clamped.rc3Deg = clamp16(targets.rc3Deg, Config::kServoMinDeg, Config::kServoMaxDeg);
    clamped.rc4Deg = clamp16(targets.rc4Deg, Config::kServoMinDeg, Config::kServoMaxDeg);
    return clamped;
}

void updateServoTargetsFromPs2(const Ps2Frame& frame,
                               ServoTargets& targets,
                               bool allowArmStep) {
    if (allowArmStep) {
        targets.rc2Deg = computeArmStep(targets.rc2Deg, frame.up, frame.down);
    }

    if (frame.l1 && !frame.r1) {
        targets.rc3Deg = Config::kRc3L1Deg;
        targets.rc4Deg = Config::kRc4L1Deg;
    } else if (frame.r1 && !frame.l1) {
        targets.rc3Deg = Config::kRc3R1Deg;
        targets.rc4Deg = Config::kRc4R1Deg;
    }

    if (frame.l2 && !frame.r2) {
        targets.rc1Deg = Config::kRc1L2Deg;
    } else if (frame.r2 && !frame.l2) {
        targets.rc1Deg = Config::kRc1R2Deg;
    }

    targets = clampServoTargets(targets);
}

void decodePs2Payload(const uint8_t raw[9], Ps2Frame& outFrame) {
    outFrame.modeId   = raw[1];
    outFrame.syncByte = raw[2];
    outFrame.buttons1 = raw[3];
    outFrame.buttons2 = raw[4];
    outFrame.rx       = raw[5];
    outFrame.ry       = raw[6];
    outFrame.lx       = raw[7];
    outFrame.ly       = raw[8];

    // Sony PS2 button bytes are active-low (0 = pressed)
    const uint8_t b1 = static_cast<uint8_t>(~raw[3]);
    const uint8_t b2 = static_cast<uint8_t>(~raw[4]);

    outFrame.select   = (b1 & 0x01U) != 0U;
    outFrame.l3       = (b1 & 0x02U) != 0U;
    outFrame.r3       = (b1 & 0x04U) != 0U;
    outFrame.start    = (b1 & 0x08U) != 0U;
    outFrame.up       = (b1 & 0x10U) != 0U;
    outFrame.right    = (b1 & 0x20U) != 0U;
    outFrame.down     = (b1 & 0x40U) != 0U;
    outFrame.left     = (b1 & 0x80U) != 0U;

    outFrame.l2       = (b2 & 0x01U) != 0U;
    outFrame.r2       = (b2 & 0x02U) != 0U;
    outFrame.l1       = (b2 & 0x04U) != 0U;
    outFrame.r1       = (b2 & 0x08U) != 0U;
    outFrame.triangle = (b2 & 0x10U) != 0U;
    outFrame.circle   = (b2 & 0x20U) != 0U;
    outFrame.cross    = (b2 & 0x40U) != 0U;
    outFrame.square   = (b2 & 0x80U) != 0U;

    outFrame.linkOk = (outFrame.modeId == Config::kPs2ExpectedMode) &&
                      (outFrame.syncByte == Config::kPs2ExpectedSync);
}

bool isPs2UnconfiguredMode(const Ps2Frame& frame) {
    return (frame.syncByte == Config::kPs2ExpectedSync) &&
           (frame.modeId != Config::kPs2ExpectedMode);
}

bool isPs2FrameValid(const Ps2Frame& frame) {
    // Stage 1: Exact Sony analog red-mode (0x73) + valid sync (0x5A)
    if (!frame.linkOk ||
        frame.modeId != Config::kPs2ExpectedMode ||
        frame.syncByte != Config::kPs2ExpectedSync) {
        return false;
    }

    // Stage 2: Physically impossible opposing D-pad states
    if ((frame.up && frame.down) || (frame.left && frame.right)) {
        return false;
    }

    // Stage 3: Impossible all-4 shape buttons pressed simultaneously (bus short)
    if (frame.triangle && frame.circle && frame.cross && frame.square) {
        return false;
    }

    // Stage 4: Stuck floating-high SPI line (every byte == 0xFF)
    if (frame.buttons1 == 0xFFU && frame.buttons2 == 0xFFU &&
        frame.rx == 0xFFU && frame.ry == 0xFFU &&
        frame.lx == 0xFFU && frame.ly == 0xFFU) {
        return false;
    }

    // Stage 5: Short-to-ground SPI line (every byte == 0x00)
    // Note: pushing both sticks to (0,0) or (255,255) with normal buttons passes cleanly.
    if (frame.buttons1 == 0x00U && frame.buttons2 == 0x00U &&
        frame.rx == 0x00U && frame.ry == 0x00U &&
        frame.lx == 0x00U && frame.ly == 0x00U) {
        return false;
    }

    return true;
}

void FailsafeStateMachine::begin(bool initialReady, uint32_t nowMs) {
    goodFrames_          = 0U;
    badFrames_           = 0U;
    modeResetCnt_        = 0U;
    lastValidMs_         = nowMs;
    lastReinitDoneMs_    = nowMs;
    reinitCooldownArmed_ = initialReady;
    state_               = initialReady ? LinkState::Stabilizing : LinkState::Disconnected;
}

void FailsafeStateMachine::onFrame(bool frameValid, uint32_t nowMs, bool modeReset) {
    if (!frameValid) {
        goodFrames_ = 0U;
        if (badFrames_ < 0xFFFFU) {
            badFrames_++;
        }

        if (modeReset) {
            if (modeResetCnt_ < 255U) {
                modeResetCnt_++;
            }
            if (modeResetCnt_ >= Config::kPs2ModeResetFrames) {
                state_               = LinkState::Disconnected;
                reinitCooldownArmed_ = false; // Allow immediate re-config without waiting 500 ms
                return;
            }
        } else {
            modeResetCnt_ = 0U;
        }

        if (state_ == LinkState::Disconnected) {
            return;
        }

        if (badFrames_ >= Config::kPs2PersistentLossFrames ||
            static_cast<uint32_t>(nowMs - lastValidMs_) >= Config::kPs2DisconnectMs) {
            state_               = LinkState::Disconnected;
            reinitCooldownArmed_ = false;
            return;
        }

        if (state_ == LinkState::Stabilizing || badFrames_ >= Config::kPs2FailsafeBadFrames) {
            state_ = LinkState::Failsafe;
        }
        return;
    }

    lastValidMs_  = nowMs;
    badFrames_    = 0U;
    modeResetCnt_ = 0U;
    if (goodFrames_ < 255U) {
        goodFrames_++;
    }

    state_ = (goodFrames_ >= Config::kPs2StableFrames)
                 ? LinkState::Operational
                 : LinkState::Stabilizing;
}

bool FailsafeStateMachine::shouldAttemptReinit(uint32_t nowMs) const {
    if (state_ != LinkState::Disconnected) {
        return false;
    }
    if (!reinitCooldownArmed_) {
        return true;
    }
    return static_cast<uint32_t>(nowMs - lastReinitDoneMs_) >= Config::kPs2ReconnectPeriodMs;
}

void FailsafeStateMachine::onReinitResult(bool ready, uint32_t nowMs) {
    lastReinitDoneMs_    = nowMs;
    reinitCooldownArmed_ = true;
    goodFrames_          = 0U;
    badFrames_           = 0U;
    modeResetCnt_        = 0U;
    if (ready) {
        lastValidMs_ = nowMs;
        state_       = LinkState::Stabilizing;
    } else {
        state_ = LinkState::Disconnected;
    }
}

void BatteryStabilizer::reset() {
    tier_                = BatteryTier::Normal;
    filteredMv_          = 0U;
    baselineMv_          = 0U;
    initBaselineMv_      = 0U;
    hasFilter_           = false;
    hasBaseline_         = false;
    extPower_            = false;
    offStreak_           = 0U;
    badStreak_           = 0U;
    recoveryActive_      = false;
    recoveryStartMs_     = 0UL;
    baselineRiseActive_  = false;
    baselineRiseStartMs_ = 0UL;
}

void BatteryStabilizer::computeThresholds(uint16_t& lowEnter,
                                          uint16_t& critEnter,
                                          uint16_t& emergEnter) const {
    const uint16_t sagLow   = hasBaseline_ ? static_cast<uint16_t>((static_cast<uint32_t>(baselineMv_) * Config::kBattLowSagPct) / 100U) : 0U;
    const uint16_t sagCrit  = hasBaseline_ ? static_cast<uint16_t>((static_cast<uint32_t>(baselineMv_) * Config::kBattCritSagPct) / 100U) : 0U;
    const uint16_t sagEmerg = hasBaseline_ ? static_cast<uint16_t>((static_cast<uint32_t>(baselineMv_) * Config::kBattEmergSagPct) / 100U) : 0U;

    lowEnter   = (sagLow > Config::kLowVoltWarnMv) ? sagLow : Config::kLowVoltWarnMv;
    critEnter  = (sagCrit > Config::kBattCritVoltMv) ? sagCrit : Config::kBattCritVoltMv;
    emergEnter = (sagEmerg > Config::kBattEmergVoltMv) ? sagEmerg : Config::kBattEmergVoltMv;

    if (critEnter >= lowEnter) {
        critEnter = (lowEnter > 200U) ? static_cast<uint16_t>(lowEnter - 200U) : 0U;
    }
    if (emergEnter >= critEnter) {
        emergEnter = (critEnter > 200U) ? static_cast<uint16_t>(critEnter - 200U) : 0U;
    }
}

BatteryTier BatteryStabilizer::update(uint16_t rawMv, bool valid, uint32_t nowMs) {
    if (!valid) {
        if (badStreak_ < 255U) {
            badStreak_++;
        }
        if (badStreak_ >= Config::kBattInvalidFallbackCnt) {
            const uint8_t savedBad = badStreak_;
            reset();
            badStreak_ = savedBad;
        }
        return tier_;
    }

    badStreak_ = 0U;

    if (rawMv < Config::kExtPowerPresentMv) {
        if (offStreak_ < 255U) {
            offStreak_++;
        }
        if (!extPower_ || offStreak_ >= Config::kBattOffDebounceTicks) {
            reset();
        }
        return tier_;
    }

    offStreak_ = 0U;
    extPower_  = true;

    const uint16_t boundedRaw = (rawMv > 25500U) ? 25500U : rawMv;
    if (!hasBaseline_) {
        initBaselineMv_      = boundedRaw;
        baselineMv_          = boundedRaw;
        hasBaseline_         = true;
        filteredMv_          = boundedRaw;
        hasFilter_           = true;
        baselineRiseActive_  = false;
        baselineRiseStartMs_ = 0UL;
    } else {
        const uint32_t pctMargin    = (static_cast<uint32_t>(initBaselineMv_) * 3U) / 100U;
        const uint16_t riseMargin   = static_cast<uint16_t>((pctMargin < 350U) ? pctMargin : 350U);
        const uint32_t maxAllowed32 = static_cast<uint32_t>(initBaselineMv_) + riseMargin;
        const uint16_t maxAllowedMv = static_cast<uint16_t>((maxAllowed32 < 25500UL) ? maxAllowed32 : 25500UL);
        const uint16_t emaInput     = (boundedRaw > maxAllowedMv) ? baselineMv_ : boundedRaw;

        if (!hasFilter_) {
            filteredMv_ = emaInput;
            hasFilter_  = true;
        } else if (emaInput < filteredMv_) {
            uint32_t step = (static_cast<uint32_t>(filteredMv_ - emaInput) * Config::kBattEmaDownAlphaPct + 50U) / 100U;
            if (step == 0U) {
                step = 1U;
            }
            filteredMv_ = (step >= filteredMv_) ? 0U : static_cast<uint16_t>(filteredMv_ - step);
        } else if (emaInput > filteredMv_) {
            uint32_t step = (static_cast<uint32_t>(emaInput - filteredMv_) * Config::kBattEmaUpAlphaPct + 50U) / 100U;
            if (step == 0U) {
                step = 1U;
            }
            filteredMv_ = static_cast<uint16_t>(filteredMv_ + step);
        }

        // Guard against transient/sustained regen voltage spikes ratcheting baselineMv_ above the true pack.
        if (tier_ == BatteryTier::Normal &&
            filteredMv_ > baselineMv_ &&
            boundedRaw > baselineMv_ &&
            boundedRaw <= maxAllowedMv) {
            if (!baselineRiseActive_) {
                baselineRiseActive_  = true;
                baselineRiseStartMs_ = nowMs;
            } else if (static_cast<uint32_t>(nowMs - baselineRiseStartMs_) >= Config::kBattRecoveryHoldMs) {
                baselineMv_         = (filteredMv_ < maxAllowedMv) ? filteredMv_ : maxAllowedMv;
                baselineRiseActive_ = false;
            }
        } else {
            baselineRiseActive_ = false;
        }
    }

    uint16_t lowEnter   = Config::kLowVoltWarnMv;
    uint16_t critEnter  = Config::kBattCritVoltMv;
    uint16_t emergEnter = Config::kBattEmergVoltMv;
    computeThresholds(lowEnter, critEnter, emergEnter);

    // Instant escalation to protect brownout-sensitive motors/STM32/PS2 receiver
    BatteryTier downTier = BatteryTier::Normal;
    if (filteredMv_ <= emergEnter) {
        downTier = BatteryTier::Emergency;
    } else if (filteredMv_ <= critEnter) {
        downTier = BatteryTier::Critical;
    } else if (filteredMv_ <= lowEnter) {
        downTier = BatteryTier::Low;
    }

    if (static_cast<uint8_t>(downTier) > static_cast<uint8_t>(tier_)) {
        tier_           = downTier;
        recoveryActive_ = false;
        return tier_;
    }

    if (tier_ == BatteryTier::Normal) {
        recoveryActive_ = false;
        return tier_;
    }

    uint16_t currentEnter = lowEnter;
    if (tier_ == BatteryTier::Emergency) {
        currentEnter = emergEnter;
    } else if (tier_ == BatteryTier::Critical) {
        currentEnter = critEnter;
    }
    const uint16_t exitThreshold = static_cast<uint16_t>(currentEnter + Config::kBattHysteresisMv);

    if (filteredMv_ >= exitThreshold) {
        if (!recoveryActive_) {
            recoveryActive_  = true;
            recoveryStartMs_ = nowMs;
        } else if (static_cast<uint32_t>(nowMs - recoveryStartMs_) >= Config::kBattRecoveryHoldMs) {
            // Step up one tier at a time so brief voltage rebound never triggers a full spike
            if (tier_ == BatteryTier::Emergency) {
                tier_ = BatteryTier::Critical;
            } else if (tier_ == BatteryTier::Critical) {
                tier_ = BatteryTier::Low;
            } else {
                tier_ = BatteryTier::Normal;
            }
            recoveryActive_ = false;
        }
    } else {
        recoveryActive_ = false;
    }

    return tier_;
}

uint8_t BatteryStabilizer::batteryPercent() const {
    if (!extPower_ || !hasFilter_ || filteredMv_ < Config::kExtPowerPresentMv) {
        return 0U;
    }

    uint16_t emptyMv = Config::kBattCritVoltMv;
    uint16_t fullMv  = baselineMv_;
    if (baselineMv_ <= 8700U) {
        // 2S LiPo (6.6V..8.4V)
        emptyMv = 6600U;
        fullMv  = 8400U;
    } else if (baselineMv_ <= 10400U) {
        // 9V adapter / NiMH pack
        emptyMv = 7000U;
        fullMv  = baselineMv_;
    } else if (baselineMv_ <= 13200U) {
        // 3S LiPo / 12V pack (9.8V..12.6V)
        emptyMv = 9800U;
        fullMv  = 12600U;
    } else if (baselineMv_ <= 17200U) {
        // 4S LiPo (13.2V..16.8V)
        emptyMv = 13200U;
        fullMv  = 16800U;
    } else {
        // 18V adapter / high-voltage supply
        emptyMv = static_cast<uint16_t>((static_cast<uint32_t>(baselineMv_) * 78U) / 100U);
        fullMv  = baselineMv_;
    }

    if (fullMv <= emptyMv) {
        return 100U;
    }
    if (filteredMv_ <= emptyMv) {
        return 0U;
    }
    if (filteredMv_ >= fullMv) {
        return 100U;
    }

    const uint32_t span = static_cast<uint32_t>(fullMv - emptyMv);
    const uint32_t pct  = (static_cast<uint32_t>(filteredMv_ - emptyMv) * 100UL + (span / 2U)) / span;
    return (pct > 100UL) ? 100U : static_cast<uint8_t>(pct);
}

MotorPowerProfile BatteryStabilizer::motorProfile() const {
    MotorPowerProfile profile;

    auto computeComp = [this](uint8_t maxCompPct) -> uint8_t {
        if (!extPower_ || !hasBaseline_ || !hasFilter_ || filteredMv_ == 0U || filteredMv_ >= baselineMv_) {
            return 100U;
        }
        const uint32_t ratio = (static_cast<uint32_t>(baselineMv_) * 100UL + (filteredMv_ / 2U)) / filteredMv_;
        if (ratio <= 100UL) {
            return 100U;
        }
        return (ratio > maxCompPct) ? maxCompPct : static_cast<uint8_t>(ratio);
    };

    if (!Config::kEnableBatterySagThrottle || tier_ == BatteryTier::Normal) {
        profile.maxPowerCap   = Config::kMotorMaxPower;
        profile.accelSlewStep = Config::kMotorAccelSlewStep;
        profile.decelSlewStep = Config::kMotorDecelSlewStep;
        profile.compPercent   = computeComp(Config::kBattCompMaxPct);
        profile.slewLimited   = false;
        return profile;
    }

    switch (tier_) {
    case BatteryTier::Low:
        profile.maxPowerCap   = Config::kBattLowMaxPower;
        profile.accelSlewStep = Config::kBattLowAccelSlew;
        profile.decelSlewStep = Config::kBattLowDecelSlew;
        profile.compPercent   = computeComp(110U);
        profile.slewLimited   = true;
        break;
    case BatteryTier::Critical:
        profile.maxPowerCap   = Config::kBattCritMaxPower;
        profile.accelSlewStep = Config::kBattCritAccelSlew;
        profile.decelSlewStep = Config::kBattCritDecelSlew;
        profile.compPercent   = 100U;
        profile.slewLimited   = true;
        break;
    case BatteryTier::Emergency:
    default:
        profile.maxPowerCap   = Config::kBattEmergMaxPower;
        profile.accelSlewStep = Config::kBattEmergAccelSlew;
        profile.decelSlewStep = Config::kBattEmergDecelSlew;
        profile.compPercent   = 100U;
        profile.slewLimited   = true;
        break;
    }
    return profile;
}

namespace {

uint8_t computePowerStabilityPenalty(bool extPower,
                                     uint8_t battPct,
                                     BatteryTier batTier,
                                     uint8_t compPct) {
    if (!extPower) {
        // Logic runs on USB, but motor/servo battery rail is absent.
        return Config::kStabilityUsbNoBattPen;
    }

    const uint8_t boundedPct = (battPct > 100U) ? 100U : battPct;

    // 1. State-of-charge headroom penalty (continuous with battery %).
    uint16_t socPen = 0U;
    if (boundedPct < 35U) {
        socPen = 12U + (static_cast<uint16_t>(35U - boundedPct) * 18U + 17U) / 35U;
    } else if (boundedPct < 75U) {
        socPen = (static_cast<uint16_t>(75U - boundedPct) * 12U + 20U) / 40U;
    }

    // 2. Dynamic voltage sag & brownout tier penalty.
    uint16_t sagPen = (compPct > 103U)
                          ? static_cast<uint16_t>((compPct - 103U + 1U) / 2U)
                          : 0U;
    if (sagPen > 8U) {
        sagPen = 8U;
    }

    switch (batTier) {
        case BatteryTier::Low:
            sagPen = static_cast<uint16_t>(Config::kStabilityPowerLowPen) + (sagPen / 2U);
            break;
        case BatteryTier::Critical:
            sagPen = Config::kStabilityPowerCritPen;
            break;
        case BatteryTier::Emergency:
            sagPen = Config::kStabilityPowerEmergPen;
            break;
        case BatteryTier::Normal:
        default:
            break;
    }

    const uint16_t combined = socPen + sagPen;
    return (combined > Config::kStabilityPowerMaxPen)
               ? Config::kStabilityPowerMaxPen
               : static_cast<uint8_t>(combined);
}

} // namespace

void StabilityMonitor::reset() {
    everConnected_ = false;
    scorePct_      = 100U;
    jitterMs_      = 0U;
    linkPen_       = 0U;
    commBusPen_    = 0U;
    commPen_       = 0U;
    powerPen_      = 0U;
    loopPen_       = 0U;
}

uint8_t StabilityMonitor::update(const StabilityInput& in) {
    if (in.ps2Link != LinkState::Disconnected) {
        everConnected_ = true;
    }

    // 1. PS2 / control link penalty
    switch (in.ps2Link) {
        case LinkState::Failsafe:
            linkPen_ = Config::kStabilityLinkMaxPen;
            break;
        case LinkState::Disconnected:
            if (everConnected_) {
                linkPen_ = Config::kStabilityLinkDropPen;
            } else if (in.activeSource == ControlSource::SerialCmd) {
                linkPen_ = Config::kStabilitySerialLinkPen;
            } else {
                linkPen_ = Config::kStabilityIdleWaitPen;
            }
            break;
        case LinkState::Stabilizing:
            linkPen_ = Config::kStabilitySyncWaitPen;
            break;
        case LinkState::Operational:
        default:
            if (!in.frameValid) {
                const uint16_t next =
                    static_cast<uint16_t>(linkPen_) + Config::kStabilityGlitchStep;
                linkPen_ = (next > Config::kStabilityLinkMaxPen)
                               ? Config::kStabilityLinkMaxPen
                               : static_cast<uint8_t>(next);
            } else {
                linkPen_ = (linkPen_ > Config::kStabilityRecoverStep)
                               ? static_cast<uint8_t>(linkPen_ - Config::kStabilityRecoverStep)
                               : 0U;
            }
            break;
    }

    // 2. STM32 lower-MCU bus health + hardware fault penalty
    if (!in.commHealthy) {
        const uint16_t next = static_cast<uint16_t>(commBusPen_) + Config::kStabilityCommStepPen;
        commBusPen_ = (next > Config::kStabilityCommBusMaxPen)
                          ? Config::kStabilityCommBusMaxPen
                          : static_cast<uint8_t>(next);
    } else {
        commBusPen_ = (commBusPen_ > Config::kStabilityRecoverStep)
                          ? static_cast<uint8_t>(commBusPen_ - Config::kStabilityRecoverStep)
                          : 0U;
    }
    const uint16_t rawCommPen = static_cast<uint16_t>(commBusPen_) +
                                (in.hwFault ? Config::kStabilityHwFaultPen : 0U);
    commPen_ = (rawCommPen > Config::kStabilityCommMaxPen)
                   ? Config::kStabilityCommMaxPen
                   : static_cast<uint8_t>(rawCommPen);

    // 3. Battery % reserve + voltage sag penalty
    powerPen_ = computePowerStabilityPenalty(in.extPower, in.battPct, in.batTier, in.compPct);

    // 4. Control loop timing jitter penalty
    const uint32_t targetMaxMs = Config::kControlPeriodMs + Config::kStabilityJitterSlackMs;
    const uint16_t overshoot   = (in.loopDtMs > targetMaxMs)
                                     ? static_cast<uint16_t>(
                                           (in.loopDtMs - Config::kControlPeriodMs > 1000UL)
                                               ? 1000UL
                                               : (in.loopDtMs - Config::kControlPeriodMs))
                                     : 0U;
    if (overshoot >= jitterMs_) {
        jitterMs_ = overshoot;
    } else {
        uint16_t decay = static_cast<uint16_t>(jitterMs_ / 8U);
        if (decay == 0U) {
            decay = 1U;
        }
        jitterMs_ = (jitterMs_ > decay) ? static_cast<uint16_t>(jitterMs_ - decay) : 0U;
    }

    const uint16_t rawLoopPen = static_cast<uint16_t>(jitterMs_ / 5U);
    loopPen_ = (rawLoopPen > Config::kStabilityLoopMaxPen)
                   ? Config::kStabilityLoopMaxPen
                   : static_cast<uint8_t>(rawLoopPen);

    const uint16_t totalPen = static_cast<uint16_t>(linkPen_) + commPen_ + powerPen_ + loopPen_;
    const uint8_t  rawScore = (totalPen >= 100U) ? 0U : static_cast<uint8_t>(100U - totalPen);

    if (rawScore <= scorePct_) {
        scorePct_ = rawScore; // Reflect degradation immediately
    } else {
        const uint16_t climbed = static_cast<uint16_t>(scorePct_) + Config::kStabilityRecoverStep;
        scorePct_ = (climbed > rawScore) ? rawScore : static_cast<uint8_t>(climbed);
    }

    return scorePct_;
}

} // namespace RobotLogic
