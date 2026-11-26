#include "robot_logic.h"

// Battery sag stabilizer and live stability index (pure C++11).

namespace RobotLogic {

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

    uint16_t lowEnter   = 0U;
    uint16_t critEnter  = 0U;
    uint16_t emergEnter = 0U;
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

    uint16_t emptyMv = 0U;
    uint16_t fullMv  = 0U;
    classifyBatteryPack(baselineMv_, emptyMv, fullMv);

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
        // Sag throttle off or tier Normal: full torque compensation. With throttle enabled,
        // Low scales comp to 110% and Critical/Emergency pin it to 100% (see the switch below).
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

void classifyBatteryPack(uint16_t baselineMv, uint16_t& emptyMv, uint16_t& fullMv) {
    if (baselineMv <= 8700U) {
        // 2S Li-ion/LiPo (6.6V..8.4V)
        emptyMv = 6600U;
        fullMv  = 8400U;
    } else if (baselineMv < 9600U) {
        // 9V adapter / NiMH pack (a 3S this low is below 3.2 V/cell, already dead)
        emptyMv = 7000U;
        fullMv  = baselineMv;
    } else if (baselineMv <= 12900U) {
        // 3S LiPo (9.8V..12.6V). Starts at 9.6 V so a drained 3S reads ~0%, not adapter 100%.
        emptyMv = 9800U;
        fullMv  = 12600U;
    } else if (baselineMv <= 17200U) {
        // 4S LiPo (13.2V..16.8V). Starts above 12.9 V so a drained 4S is not read as a full 3S.
        emptyMv = 13200U;
        fullMv  = 16800U;
    } else {
        // 18V adapter / high-voltage supply
        emptyMv = static_cast<uint16_t>((static_cast<uint32_t>(baselineMv) * 78U) / 100U);
        fullMv  = baselineMv;
    }
}

uint8_t computeAutoPowerCompPct(uint16_t battMv, uint16_t refMv, uint8_t minPct, uint8_t maxPct) {
    if (battMv < Config::kExtPowerPresentMv || refMv == 0U) {
        return 100U;
    }
    // The clamp window always contains 100 so a bad setting can never invert the compensation.
    const uint32_t lo  = (minPct > 100U) ? 100UL : static_cast<uint32_t>(minPct);
    const uint32_t hi  = (maxPct < 100U) ? 100UL : static_cast<uint32_t>(maxPct);
    const uint32_t pct = (static_cast<uint32_t>(refMv) * 100UL + (battMv / 2U)) / battMv;
    if (pct < lo) {
        return static_cast<uint8_t>(lo);
    }
    if (pct > hi) {
        return static_cast<uint8_t>(hi);
    }
    return static_cast<uint8_t>(pct);
}

int16_t applyAutoPowerComp(int16_t power, uint8_t compPct) {
    if (power == 0 || compPct == 0U || compPct == 100U) {
        return power;
    }
    const bool    negative = (power < 0);
    const int32_t mag      = abs32(power);
    int32_t       scaled   = (mag * static_cast<int32_t>(compPct) + 50L) / 100L;
    if (scaled < 1L) {
        scaled = 1L;
    }
    if (scaled > static_cast<int32_t>(Config::kMotorMaxPower)) {
        scaled = static_cast<int32_t>(Config::kMotorMaxPower);
    }
    return static_cast<int16_t>(negative ? -scaled : scaled);
}

uint8_t computeDriveStabilityPenalty(uint8_t comScalePct,
                                     int16_t driveLeft,
                                     int16_t driveRight,
                                     bool    wheelStall) {
    // Arm-raise factor 0..100: 0 at CoM scale 100 (arm low), 100 at the minimum CoM scale (arm high).
    const int32_t minScale = static_cast<int32_t>(Config::kCoMHighMinScalePct);
    int32_t armFactor = 0L;
    if (minScale < 100L) {
        const int32_t scale = clampLong(static_cast<int32_t>(comScalePct), minScale, 100L);
        armFactor = ((100L - scale) * 100L + ((100L - minScale) / 2L)) / (100L - minScale);
    }

    // Motion intensity 0..100: peak commanded wheel power.
    const int32_t motion = clampLong(peakAbs(driveLeft, driveRight), 0L, 100L);

    const int32_t tipPen   = (armFactor * motion * static_cast<int32_t>(Config::kStabilityTipMaxPen) + 5000L) / 10000L;
    const int32_t stallPen = wheelStall ? static_cast<int32_t>(Config::kStabilityStallPen) : 0L;
    return static_cast<uint8_t>(clampLong(tipPen + stallPen, 0L, static_cast<int32_t>(Config::kStabilityDriveMaxPen)));
}

void StabilityMonitor::reset() {
    everConnected_ = false;
    scorePct_      = 100U;
    jitterMs_      = 0U;
    linkPen_       = 0U;
    commBusPen_    = 0U;
    commPen_       = 0U;
    powerPen_      = 0U;
    loopPen_       = 0U;
    drivePen_      = 0U;
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
            if (in.activeSource == ControlSource::Auto) {
                linkPen_ = 0U;
            } else if (everConnected_) {
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

    // 5. Drive risk: arm raised while driving hard (tip) + wheel stall
    drivePen_ = computeDriveStabilityPenalty(in.comScalePct, in.driveLeft, in.driveRight, in.wheelStall);

    const uint16_t totalPen =
        static_cast<uint16_t>(linkPen_) + commPen_ + powerPen_ + loopPen_ + drivePen_;
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
