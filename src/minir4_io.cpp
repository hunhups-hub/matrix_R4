#include "minir4_io.h"

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "Modules/MMLower.h"

#include "robot_logic.h"

namespace {

// Matches MatrixMiniR4.h: MiniR4BTN<2> BTN_UP, MiniR4BTN<1> BTN_DOWN.
constexpr uint8_t kBtnUpId   = 2U;
constexpr uint8_t kBtnDownId = 1U;

} // namespace

bool MiniR4IO::begin() {
    const bool ok = MiniR4.begin();

    leftReverse_  = Config::kMotorLeftReverseInit;
    rightReverse_ = Config::kMotorRightReverseInit;

    const bool cfgOk = applyLowerMcuConfig(true);

    // Zero M1/M2 immediately at boot for safety.
    const bool m1Ok = MiniR4.M1.setPower(0);
    const bool m2Ok = MiniR4.M2.setPower(0);
    lastLeftPower_    = 0;
    lastRightPower_   = 0;
    lastBrakeEngaged_ = false;
    motorsSynced_     = m1Ok && m2Ok;
    if (!motorsSynced_ || !cfgOk) {
        commHealthy_   = false;
        commNeedsHeal_ = true;
    }

    staggerBootServos();

    // Reconfigure PS2 pins after MiniR4.begin() using deterministic non-blocking SPI driver.
    ps2Ready_ = ps2_.begin(Config::kPs2ClkPin,
                           Config::kPs2CmdPin,
                           Config::kPs2AttPin,
                           Config::kPs2DatPin,
                           Config::kPs2BootTimeoutMs);

    // Seed initial button state so stuck buttons at boot cannot arm the upload shortcut.
    bool upInit   = false;
    bool downInit = false;
    if (mmL.GetButtonState(kBtnUpId, upInit) != MMLower::RESULT::OK) {
        upInit         = false;
        commHealthy_   = false;
        commNeedsHeal_ = true;
    }
    if (mmL.GetButtonState(kBtnDownId, downInit) != MMLower::RESULT::OK) {
        downInit       = false;
        commHealthy_   = false;
        commNeedsHeal_ = true;
    }
    btnUp_.rawState           = upInit;
    btnUp_.stableState        = btnUp_.rawState;
    btnDown_.rawState         = downInit;
    btnDown_.stableState      = btnDown_.rawState;
    suppressUploadUntilClear_ = btnUp_.stableState || btnDown_.stableState;

    const uint32_t nowMs = millis();
    lastLowerHealthMs_   = nowMs;
    lastConfigResyncMs_  = nowMs;
    lastStopRefreshMs_   = nowMs;
    lastButtonPollMs_    = nowMs;

    return ok;
}

bool MiniR4IO::applyLowerMcuConfig(bool zeroAuxMotors) {
    // Allow 9V-18V DC adapters as well as 2S-4S packs without false STM32 cutoff.
    const bool pwrOk =
        (mmL.SetPowerParam(Config::kPowerFullVolt,
                           Config::kPowerCutoffVolt,
                           Config::kPowerAlarmVolt) == MMLower::RESULT::OK);

    const bool m1RevOk = MiniR4.M1.setReverse(leftReverse_);
    const bool m2RevOk = MiniR4.M2.setReverse(rightReverse_);
    const bool rngOk   = configureServoRanges();

    bool auxOk = true;
    if (zeroAuxMotors || !auxMotorsZeroed_) {
        const bool m3Ok = MiniR4.M3.setPower(0);
        const bool m4Ok = MiniR4.M4.setPower(0);
        auxOk            = m3Ok && m4Ok;
        auxMotorsZeroed_ = auxOk;
    }

    return pwrOk && m1RevOk && m2RevOk && rngOk && auxOk;
}

void MiniR4IO::setMotorPolarity(bool leftReverse, bool rightReverse) {
    leftReverse_  = leftReverse;
    rightReverse_ = rightReverse;
    const bool ok1 = MiniR4.M1.setReverse(leftReverse_);
    const bool ok2 = MiniR4.M2.setReverse(rightReverse_);
    motorsSynced_  = false;
    if (!ok1 || !ok2) {
        commHealthy_   = false;
        commNeedsHeal_ = true;
    }
}

bool MiniR4IO::configureServoRanges() {
    servoRangeErr_ = 0U;
    bool allOk     = true;
    for (uint8_t ch = 1U; ch <= 4U; ++ch) {
        const MMLower::RESULT angleRes = mmL.SetServoAngleRange(
            ch,
            static_cast<uint16_t>(Config::kServoMinDeg),
            static_cast<uint16_t>(Config::kServoMaxDeg));
        if (angleRes != MMLower::RESULT::OK) {
            allOk = false;
            if (servoRangeErr_ == 0U) {
                servoRangeErr_ = static_cast<uint8_t>(angleRes);
            }
        }
    }
    return allOk;
}

void MiniR4IO::staggerBootServos() {
    const ServoTargets safeInit = RobotLogic::clampServoTargets(ServoTargets{});
    const bool ok1 = MiniR4.RC1.setAngle(static_cast<uint16_t>(safeInit.rc1Deg));
    delay(Config::kServoBootStaggerMs);
    const bool ok2 = MiniR4.RC2.setAngle(static_cast<uint16_t>(safeInit.rc2Deg));
    delay(Config::kServoBootStaggerMs);
    const bool ok3 = MiniR4.RC3.setAngle(static_cast<uint16_t>(safeInit.rc3Deg));
    delay(Config::kServoBootStaggerMs);
    const bool ok4 = MiniR4.RC4.setAngle(static_cast<uint16_t>(safeInit.rc4Deg));

    lastServos_   = safeInit;
    servosSynced_ = ok1 && ok2 && ok3 && ok4;
    if (!servosSynced_) {
        commHealthy_   = false;
        commNeedsHeal_ = true;
    }
}

bool MiniR4IO::tryReconnectPs2() {
    // Non-blocking 1-cycle probe (~0.8 ms unplugged, ~7 ms if responding); no 12 ms delay() inside loop().
    return ps2_.tryConnect(1U);
}

void MiniR4IO::pollPs2(Ps2Frame& outFrame) {
    ps2_.poll(outFrame);
}

bool MiniR4IO::updateButtonDebounce(DebouncedBtn& btn,
                                    bool sample,
                                    uint32_t nowMs,
                                    bool& outShortRelease) {
    outShortRelease = false;
    if (sample != btn.rawState) {
        btn.rawState     = sample;
        btn.lastChangeMs = nowMs;
    }

    if (static_cast<uint32_t>(nowMs - btn.lastChangeMs) >= Config::kButtonDebounceMs &&
        btn.stableState != btn.rawState) {
        btn.stableState = btn.rawState;
        if (btn.stableState) {
            btn.pressLatched = true;
        } else if (btn.pressLatched) {
            btn.pressLatched = false;
            outShortRelease  = true;
        }
        return true;
    }
    return false;
}

void MiniR4IO::pollButtons(uint32_t nowMs, ButtonEvents& outEvents) {
    outEvents = ButtonEvents{};

    const uint32_t pollInterval = commHealthy_
                                      ? Config::kButtonPollPeriodMs
                                      : Config::kButtonUnhealthyPollPeriodMs;
    bool pollRan   = false;
    bool shortUp   = false;
    bool shortDown = false;

    // When comm is unhealthy, throttle button polling to prevent continuous 100 ms blocking UART timeouts.
    if (static_cast<uint32_t>(nowMs - lastButtonPollMs_) >= pollInterval) {
        lastButtonPollMs_ = nowMs;
        pollRan           = true;

        // Alternate button reads over SoftwareSerial to halve UART round-trips per tick.
        bool sample = false;
        if (pollUpTurn_) {
            if (mmL.GetButtonState(kBtnUpId, sample) == MMLower::RESULT::OK) {
                updateButtonDebounce(btnUp_, sample, nowMs, shortUp);
            } else {
                commHealthy_   = false;
                commNeedsHeal_ = true;
            }
        } else {
            if (mmL.GetButtonState(kBtnDownId, sample) == MMLower::RESULT::OK) {
                updateButtonDebounce(btnDown_, sample, nowMs, shortDown);
            } else {
                commHealthy_   = false;
                commNeedsHeal_ = true;
            }
        }
        pollUpTurn_ = !pollUpTurn_;
    }

    outEvents.upPressed   = btnUp_.stableState;
    outEvents.downPressed = btnDown_.stableState;

    if (suppressUploadUntilClear_) {
        if (!btnUp_.stableState && !btnDown_.stableState) {
            suppressUploadUntilClear_ = false;
            btnUp_.pressLatched       = false;
            btnDown_.pressLatched     = false;
        }
        return;
    }

    const bool bothHeld = btnUp_.stableState && btnDown_.stableState;
    if (bothHeld) {
        // Prevent single-button release actions after releasing an upload combo.
        btnUp_.pressLatched   = false;
        btnDown_.pressLatched = false;

        if (!comboPrevActive_) {
            comboPrevActive_ = true;
            comboFired_      = false;
            comboStartMs_    = nowMs;
        }

        const uint32_t elapsed = static_cast<uint32_t>(nowMs - comboStartMs_);
        outEvents.comboHolding = true;
        float ratio            = static_cast<float>(elapsed) / static_cast<float>(Config::kUploadComboHoldMs);
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }
        outEvents.uploadHoldRatio = ratio;

        if (!comboFired_ && elapsed >= Config::kUploadComboHoldMs) {
            comboFired_               = true;
            outEvents.uploadRequested = true;
        }
        return;
    }

    comboPrevActive_ = false;
    comboFired_      = false;

    if (pollRan) {
        outEvents.shortUp   = shortUp;
        outEvents.shortDown = shortDown;
    }
}

bool MiniR4IO::verifyAndHealLowerMcu(uint32_t nowMs,
                                     bool isMoving,
                                     bool& outVoltValid,
                                     uint16_t& outBattMv,
                                     bool force) {
    outVoltValid = false;
    outBattMv    = 0U;

    const uint32_t intervalMs = (isMoving || !commHealthy_ || commNeedsHeal_)
                                    ? Config::kLowerFastHealthMs
                                    : Config::kLowerHealthIntervalMs;
    if (!force && static_cast<uint32_t>(nowMs - lastLowerHealthMs_) < intervalMs) {
        return false;
    }
    lastLowerHealthMs_ = nowMs;

    float v    = 0.0f;
    float perc = 0.0f;
    if (mmL.GetPowerInfo(v, perc) != MMLower::RESULT::OK) {
        commHealthy_   = false;
        commNeedsHeal_ = true;
        return true;
    }

    const float boundedV = (v < 0.0f) ? 0.0f : ((v > 25.5f) ? 25.5f : v);
    outBattMv    = static_cast<uint16_t>(boundedV * 1000.0f + 0.5f);
    outVoltValid = true;

    const bool brownoutRecovered =
        (prevRawMv_ < Config::kExtPowerPresentMv && outBattMv >= Config::kExtPowerPresentMv);
    prevRawMv_ = outBattMv;

    const bool periodicIdleResync =
        !isMoving &&
        static_cast<uint32_t>(nowMs - lastConfigResyncMs_) >= Config::kLowerResyncConfigMs;

    if (!commHealthy_ || commNeedsHeal_ || brownoutRecovered || periodicIdleResync) {
        if (!commHealthy_ || commNeedsHeal_ || brownoutRecovered) {
            invalidateActuatorCaches();
        }
        const bool cfgOk    = applyLowerMcuConfig(brownoutRecovered || !commHealthy_);
        commHealthy_        = cfgOk;
        commNeedsHeal_      = !cfgOk;
        lastConfigResyncMs_ = nowMs;
    }

    return true;
}

void MiniR4IO::invalidateActuatorCaches() {
    motorsSynced_ = false;
    servosSynced_ = false;
}

bool MiniR4IO::shouldRefreshStoppedMotors(uint32_t nowMs) const {
    if (!motorsSynced_) {
        const uint32_t retryInterval = commHealthy_
                                           ? Config::kCommRetryIntervalMs
                                           : Config::kCommUnhealthyRetryIntervalMs;
        return static_cast<uint32_t>(nowMs - lastMotorAttemptMs_) >= retryInterval;
    }
    return static_cast<uint32_t>(nowMs - lastStopRefreshMs_) >= Config::kFailsafeRefreshMs;
}

void MiniR4IO::applyMotors(int16_t leftPower, int16_t rightPower, bool forceBrake, uint32_t nowMs) {
    const int16_t clampedL = RobotLogic::clamp16(leftPower, Config::kMotorMinPower, Config::kMotorMaxPower);
    const int16_t clampedR = RobotLogic::clamp16(rightPower, Config::kMotorMinPower, Config::kMotorMaxPower);

    const bool wantBrake    = (clampedL == 0 && clampedR == 0 && forceBrake);
    const bool brakeChanged = (wantBrake != lastBrakeEngaged_);
    const bool keepaliveDue =
        static_cast<uint32_t>(nowMs - lastMotorSendMs_) >= Config::kMotorKeepaliveMs;

    const bool updateL = !motorsSynced_ || (clampedL != lastLeftPower_) ||
                         (clampedL == 0 && brakeChanged) || keepaliveDue;
    const bool updateR = !motorsSynced_ || (clampedR != lastRightPower_) ||
                         (clampedR == 0 && brakeChanged) || keepaliveDue;

    if (!updateL && !updateR) {
        return;
    }

    const uint32_t retryInterval = commHealthy_
                                       ? Config::kCommRetryIntervalMs
                                       : Config::kCommUnhealthyRetryIntervalMs;
    if (!motorsSynced_ &&
        static_cast<uint32_t>(nowMs - lastMotorAttemptMs_) < retryInterval) {
        return;
    }
    lastMotorAttemptMs_ = nowMs;

    bool okL = true;
    bool okR = true;

    if (updateL) {
        if (clampedL == 0 && forceBrake) {
            okL = MiniR4.M1.setBrake(true);
        } else {
            okL = MiniR4.M1.setPower(clampedL);
        }
        if (okL) {
            lastLeftPower_ = clampedL;
        }
    }

    if (updateR) {
        if (clampedR == 0 && forceBrake) {
            okR = MiniR4.M2.setBrake(true);
        } else {
            okR = MiniR4.M2.setPower(clampedR);
        }
        if (okR) {
            lastRightPower_ = clampedR;
        }
    }

    if (okL && okR) {
        lastBrakeEngaged_ = wantBrake;
        lastMotorSendMs_  = nowMs;
        motorsSynced_     = true;
    } else {
        motorsSynced_  = false;
        commHealthy_   = false;
        commNeedsHeal_ = true;
    }
}

void MiniR4IO::forceMotorsStop(bool engageBrake, uint32_t nowMs) {
    const uint32_t retryInterval = commHealthy_
                                       ? Config::kCommRetryIntervalMs
                                       : Config::kCommUnhealthyRetryIntervalMs;
    if (!motorsSynced_ &&
        static_cast<uint32_t>(nowMs - lastMotorAttemptMs_) < retryInterval) {
        return;
    }
    lastMotorAttemptMs_ = nowMs;

    bool ok1 = false;
    bool ok2 = false;
    if (engageBrake) {
        ok1 = MiniR4.M1.setBrake(true);
        ok2 = MiniR4.M2.setBrake(true);
    } else {
        ok1 = MiniR4.M1.setPower(0);
        ok2 = MiniR4.M2.setPower(0);
    }

    bool auxOk = true;
    if (!auxMotorsZeroed_) {
        const bool ok3   = MiniR4.M3.setPower(0);
        const bool ok4   = MiniR4.M4.setPower(0);
        auxOk            = ok3 && ok4;
        auxMotorsZeroed_ = auxOk;
    }

    if (ok1) {
        lastLeftPower_ = 0;
    }
    if (ok2) {
        lastRightPower_ = 0;
    }

    if (ok1 && ok2 && auxOk) {
        lastBrakeEngaged_  = engageBrake;
        lastMotorSendMs_   = nowMs;
        lastStopRefreshMs_ = nowMs;
        motorsSynced_      = true;
    } else {
        motorsSynced_  = false;
        commHealthy_   = false;
        commNeedsHeal_ = true;
    }
}

void MiniR4IO::applyServos(const ServoTargets& targets, uint32_t nowMs) {
    const ServoTargets safe = RobotLogic::clampServoTargets(targets);

    const bool keepaliveDue =
        static_cast<uint32_t>(nowMs - lastServoSendMs_) >= Config::kServoKeepaliveMs;

    const bool send1 = !servosSynced_ || (safe.rc1Deg != lastServos_.rc1Deg) || keepaliveDue;
    const bool send2 = !servosSynced_ || (safe.rc2Deg != lastServos_.rc2Deg) || keepaliveDue;
    const bool send3 = !servosSynced_ || (safe.rc3Deg != lastServos_.rc3Deg) || keepaliveDue;
    const bool send4 = !servosSynced_ || (safe.rc4Deg != lastServos_.rc4Deg) || keepaliveDue;

    if (!send1 && !send2 && !send3 && !send4) {
        return;
    }

    const uint32_t retryInterval = commHealthy_
                                       ? Config::kCommRetryIntervalMs
                                       : Config::kCommUnhealthyRetryIntervalMs;
    if (!servosSynced_ &&
        static_cast<uint32_t>(nowMs - lastServoAttemptMs_) < retryInterval) {
        return;
    }
    lastServoAttemptMs_ = nowMs;

    bool allOk = true;
    if (send1) {
        if (MiniR4.RC1.setAngle(static_cast<uint16_t>(safe.rc1Deg))) {
            lastServos_.rc1Deg = safe.rc1Deg;
        } else {
            allOk = false;
        }
    }
    if (send2) {
        if (MiniR4.RC2.setAngle(static_cast<uint16_t>(safe.rc2Deg))) {
            lastServos_.rc2Deg = safe.rc2Deg;
        } else {
            allOk = false;
        }
    }
    if (send3) {
        if (MiniR4.RC3.setAngle(static_cast<uint16_t>(safe.rc3Deg))) {
            lastServos_.rc3Deg = safe.rc3Deg;
        } else {
            allOk = false;
        }
    }
    if (send4) {
        if (MiniR4.RC4.setAngle(static_cast<uint16_t>(safe.rc4Deg))) {
            lastServos_.rc4Deg = safe.rc4Deg;
        } else {
            allOk = false;
        }
    }

    if (allOk) {
        lastServoSendMs_ = nowMs;
        servosSynced_    = true;
    } else {
        servosSynced_  = false;
        commHealthy_   = false;
        commNeedsHeal_ = true;
    }
}
