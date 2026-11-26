#include "minir4_io.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <MatrixMiniR4.h>
#include <Wire.h>
#include "Modules/MMLower.h"

#include "robot_logic.h"

namespace {

// Matches MatrixMiniR4.h: MiniR4BTN<2> BTN_UP, MiniR4BTN<1> BTN_DOWN.
constexpr uint8_t kBtnUpId   = 2U;
constexpr uint8_t kBtnDownId = 1U;

template <typename MotorT>
bool beginMotorBounded(MotorT& motor) {
    for (uint8_t attempt = 0U; attempt < 3U; ++attempt) {
        if (motor.begin()) {
            return true;
        }
        delay(10);
    }
    return false;
}

bool setMotorPowerSingle(uint8_t motorId, int16_t power) {
    return mmL.SetDCMotorPower(motorId, power) == MMLower::RESULT::OK;
}

bool isServoParamError(MMLower::RESULT res) {
    return res == MMLower::RESULT::ERROR_SERVO_MIN_PULSE ||
           res == MMLower::RESULT::ERROR_SERVO_MAX_PULSE ||
           res == MMLower::RESULT::ERROR_SERVO_MIN_ANGLE ||
           res == MMLower::RESULT::ERROR_SERVO_MAX_ANGLE;
}

} // namespace

bool MiniR4IO::begin() {
    // Initialize STM32 UART with bounded EchoTest retries (avoids MiniR4.begin() infinite while loops).
    const bool lowerOk = (mmL.Init() == MMLower::RESULT::OK);

    // Always initialize UNO R4 onboard peripherals regardless of STM32 link state.
    MiniR4.LED.begin(7);
    MiniR4.LED.setColor(1, 0, 0, 0);
    MiniR4.LED.setColor(2, 0, 0, 0);
    MiniR4.Buzzer.begin(6);

    // Arm Wire1 timeout before OLED init so a shorted/hung I2C1 bus cannot stall boot.
    Wire1.begin();
    Wire1.setWireTimeout(static_cast<unsigned int>(Config::kWireTimeoutUs), true);
    MiniR4.OLED = Adafruit_SSD1306(128, 32, &Wire1, -1);
    (void)MiniR4.OLED.begin(SSD1306_SWITCHCAPVCC, MATRIXMINIR4_OLED_ADDRESS, false, false);
    MiniR4.OLED.setTextColor(SSD1306_WHITE);
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.display();

    leftReverse_  = Config::kMotorLeftReverseInit;
    rightReverse_ = Config::kMotorRightReverseInit;

    if (lowerOk) {
        (void)MiniR4.Motion.begin();

        const bool m1InitOk = beginMotorBounded(MiniR4.M1);
        const bool m2InitOk = m1InitOk && beginMotorBounded(MiniR4.M2);
        const bool m3InitOk = m2InitOk && beginMotorBounded(MiniR4.M3);
        const bool m4InitOk = m3InitOk && beginMotorBounded(MiniR4.M4);
        const bool mInitOk  = m1InitOk && m2InitOk && m3InitOk && m4InitOk;

        const bool cfgOk = mInitOk && applyLowerMcuConfig(true);

        // Zero M1/M2 immediately at boot for safety (engage hold-brake if enabled).
        const bool m1Ok = cfgOk && (Config::kEnableSmartHoldBrake
                                        ? MiniR4.M1.setBrake(true)
                                        : setMotorPowerSingle(Config::kMotorLeftId, 0));
        const bool m2Ok = m1Ok && (Config::kEnableSmartHoldBrake
                                       ? MiniR4.M2.setBrake(true)
                                       : setMotorPowerSingle(Config::kMotorRightId, 0));
        lastLeftPower_    = 0;
        lastRightPower_   = 0;
        lastBrakeEngaged_ = Config::kEnableSmartHoldBrake && m1Ok && m2Ok;
        motorsSynced_     = m1Ok && m2Ok;
        if (motorsSynced_) {
            lastMotorSendMs_ = millis();
        }

        if (motorsSynced_) {
            staggerBootServos();
        } else {
            lastServos_   = RobotLogic::clampServoTargets(ServoTargets{});
            servosSynced_ = false;
        }

        // Seed initial button state so stuck buttons at boot cannot arm the upload shortcut.
        bool       upInit    = false;
        bool       downInit  = false;
        const bool btnUpOk   = motorsSynced_ && servosSynced_ &&
                               (mmL.GetButtonState(kBtnUpId, upInit) == MMLower::RESULT::OK);
        const bool btnDownOk = btnUpOk &&
                               (mmL.GetButtonState(kBtnDownId, downInit) == MMLower::RESULT::OK);
        if (!btnUpOk) {
            upInit = false;
        }
        if (!btnDownOk) {
            downInit = false;
        }
        btnUp_.rawState           = upInit;
        btnUp_.stableState        = btnUp_.rawState;
        btnDown_.rawState         = downInit;
        btnDown_.stableState      = btnDown_.rawState;
        suppressUploadUntilClear_ = btnUp_.stableState || btnDown_.stableState;

        if (!mInitOk || !cfgOk || !motorsSynced_ || !servosSynced_ || !btnUpOk || !btnDownOk) {
            (void)noteCommFailure();
        }
    } else {
        // Skip blocking STM32 calls during boot; verifyAndHealLowerMcu() recovers when online.
        lastLeftPower_            = 0;
        lastRightPower_           = 0;
        lastBrakeEngaged_         = false;
        motorsSynced_             = false;
        lastServos_               = RobotLogic::clampServoTargets(ServoTargets{});
        servosSynced_             = false;
        btnUp_                    = DebouncedBtn{};
        btnDown_                  = DebouncedBtn{};
        suppressUploadUntilClear_ = false;
        (void)noteCommFailure();
    }

    // Reconfigure PS2 pins using deterministic non-blocking SPI driver.
    ps2Ready_ = ps2_.begin(Config::kPs2ClkPin,
                           Config::kPs2CmdPin,
                           Config::kPs2AttPin,
                           Config::kPs2DatPin,
                           Config::kPs2BootTimeoutMs);

    const uint32_t nowMs = millis();
    if (!commHealthy_) {
        lastCommFailMs_ = nowMs;
    }
    lastLowerHealthMs_  = nowMs;
    lastConfigResyncMs_ = nowMs;
    lastStopRefreshMs_  = nowMs;
    lastButtonPollMs_   = nowMs;

    return lowerOk && commHealthy_;
}

bool MiniR4IO::applyLowerMcuConfig(bool zeroAuxMotors) {
    // Allow 9V-18V DC adapters as well as 2S-4S packs without false STM32 cutoff.
    if (mmL.SetPowerParam(Config::kPowerFullVolt,
                          Config::kPowerCutoffVolt,
                          Config::kPowerAlarmVolt) != MMLower::RESULT::OK) {
        return false;
    }

    if (!MiniR4.M1.setReverse(leftReverse_) || !MiniR4.M2.setReverse(rightReverse_)) {
        return false;
    }

    if (!configureServoRanges()) {
        return false;
    }

    if (zeroAuxMotors || !auxMotorsZeroed_) {
        if (mmL.SetDCMotorSpeedRange(Config::kMotorLeftId, 0U, 100U) != MMLower::RESULT::OK ||
            mmL.SetDCMotorSpeedRange(Config::kMotorRightId, 0U, 100U) != MMLower::RESULT::OK) {
            return false;
        }

        const bool m3Ok  = setMotorPowerSingle(3U, 0);
        const bool m4Ok  = m3Ok && setMotorPowerSingle(4U, 0);
        auxMotorsZeroed_ = m3Ok && m4Ok;
        if (!auxMotorsZeroed_) {
            return false;
        }
    }

    return true;
}

uint32_t MiniR4IO::noteCommFailure() {
    const uint32_t failMs = millis();
    commHealthy_          = false;
    commNeedsHeal_        = true;
    lastCommFailMs_       = failMs;
    return failMs;
}

bool MiniR4IO::isCommBackoffActive(uint32_t nowMs, uint32_t backoffMs) const {
    return !commHealthy_ &&
           static_cast<int32_t>(nowMs - lastCommFailMs_) < static_cast<int32_t>(backoffMs);
}

bool MiniR4IO::readDriveEncoders(uint32_t nowMs, int32_t& outLeft, int32_t& outRight) {
    if (!commHealthy_ || isCommBackoffActive(nowMs, Config::kCommUnhealthyRetryIntervalMs)) {
        return false;
    }
    int32_t enc[4] = {0, 0, 0, 0};
    if (mmL.GetAllEncoderCounter(enc) != MMLower::RESULT::OK) {
        (void)noteCommFailure();
        return false;
    }
    outLeft  = enc[Config::kMotorLeftId - 1U];
    outRight = enc[Config::kMotorRightId - 1U];
    return true;
}

bool MiniR4IO::isElapsed(uint32_t nowMs, uint32_t sinceMs, uint32_t periodMs) {
    return static_cast<int32_t>(nowMs - sinceMs) >= static_cast<int32_t>(periodMs);
}

void MiniR4IO::setMotorPolarity(bool leftReverse, bool rightReverse) {
    if (leftReverse == leftReverse_ && rightReverse == rightReverse_ && commHealthy_) {
        return;
    }
    leftReverse_  = leftReverse;
    rightReverse_ = rightReverse;
    if (!commHealthy_) {
        motorsSynced_  = false;
        commNeedsHeal_ = true;
        return;
    }
    const bool ok1 = MiniR4.M1.setReverse(leftReverse_);
    const bool ok2 = ok1 && MiniR4.M2.setReverse(rightReverse_);
    motorsSynced_  = false;
    if (!ok1 || !ok2) {
        (void)noteCommFailure();
    }
}

void MiniR4IO::syncServoCache(const ServoTargets& current) {
    lastServos_ = RobotLogic::clampServoTargets(current);
}

bool MiniR4IO::loadConfigFromEeprom(PersistentConfig& outCfg) {
    uint8_t raw[8];
    for (uint8_t i = 0U; i < 8U; ++i) {
        raw[i] = EEPROM.read(static_cast<int>(Config::kEepromConfigAddr + i));
    }
    return RobotLogic::unpackPersistentConfig(raw, outCfg);
}

void MiniR4IO::saveConfigToEeprom(const PersistentConfig& cfg) {
    uint8_t raw[8];
    RobotLogic::packPersistentConfig(cfg, raw);
    for (uint8_t i = 0U; i < 8U; ++i) {
        const int addr = static_cast<int>(Config::kEepromConfigAddr + i);
        if (EEPROM.read(addr) != raw[i]) {
            EEPROM.write(addr, raw[i]);
        }
    }
}

bool MiniR4IO::configureServoRanges() {
    servoRangeErr_ = 0U;
    for (uint8_t ch = 1U; ch <= 4U; ++ch) {
        const MMLower::RESULT angleRes = mmL.SetServoAngleRange(
            ch,
            static_cast<uint16_t>(Config::kServoMinDeg),
            static_cast<uint16_t>(Config::kServoMaxDeg));
        if (angleRes != MMLower::RESULT::OK) {
            if (isServoParamError(angleRes) && servoRangeErr_ == 0U) {
                servoRangeErr_ = static_cast<uint8_t>(angleRes);
            }
            return false;
        }
    }
    return true;
}

void MiniR4IO::staggerBootServos() {
    const ServoTargets safeInit = RobotLogic::clampServoTargets(ServoTargets{});
    const int16_t      hwRc3    = RobotLogic::applyServoDirection(safeInit.rc3Deg, Config::kRc3InvertDir);
    const bool ok1 = MiniR4.RC1.setAngle(static_cast<uint16_t>(safeInit.rc1Deg));
    if (ok1) {
        delay(Config::kServoBootStaggerMs);
    }
    const bool ok2 = ok1 && MiniR4.RC2.setAngle(static_cast<uint16_t>(safeInit.rc2Deg));
    if (ok2) {
        delay(Config::kServoBootStaggerMs);
    }

    const bool ok3 = ok2 && MiniR4.RC3.setAngle(static_cast<uint16_t>(hwRc3));
    if (ok3) {
        delay(Config::kServoBootStaggerMs);
    }

    lastServos_   = safeInit;
    servosSynced_ = ok1 && ok2 && ok3;
    if (servosSynced_) {
        lastServoSendMs_ = millis();
    } else {
        (void)noteCommFailure();
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
                                    bool& outShortRelease,
                                    bool& outPressEdge) {
    outShortRelease = false;
    outPressEdge    = false;
    if (sample != btn.rawState) {
        btn.rawState     = sample;
        btn.lastChangeMs = nowMs;
    }

    if (static_cast<uint32_t>(nowMs - btn.lastChangeMs) >= Config::kButtonDebounceMs &&
        btn.stableState != btn.rawState) {
        btn.stableState = btn.rawState;
        if (btn.stableState) {
            btn.pressLatched = true;
            outPressEdge     = true;
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
    bool downEdge  = false;

    // When comm is unhealthy, throttle button polling to prevent continuous 100 ms blocking UART timeouts.
    if (isElapsed(nowMs, lastButtonPollMs_, pollInterval) &&
        !isCommBackoffActive(nowMs, Config::kButtonUnhealthyPollPeriodMs)) {
        lastButtonPollMs_ = nowMs;
        pollRan           = true;

        // Alternate button reads over SoftwareSerial to halve UART round-trips per tick.
        bool sample = false;
        if (pollUpTurn_) {
            if (mmL.GetButtonState(kBtnUpId, sample) == MMLower::RESULT::OK) {
                bool upEdge = false;
                updateButtonDebounce(btnUp_, sample, nowMs, shortUp, upEdge);
            } else {
                // Drop held state on comm failure so stale buttons never phantom-trigger Bootloader/Auto.
                btnUp_            = DebouncedBtn{};
                comboPrevActive_  = false;
                comboFired_       = false;
                upHoldPrevActive_ = false;
                upHoldFired_      = false;
                lastButtonPollMs_ = noteCommFailure();
            }
        } else {
            if (mmL.GetButtonState(kBtnDownId, sample) == MMLower::RESULT::OK) {
                updateButtonDebounce(btnDown_, sample, nowMs, shortDown, downEdge);
            } else {
                btnDown_          = DebouncedBtn{};
                comboPrevActive_  = false;
                comboFired_       = false;
                lastButtonPollMs_ = noteCommFailure();
            }
        }
        pollUpTurn_ = !pollUpTurn_;
    }

    outEvents.upPressed = btnUp_.stableState;

    // E-stop must fire the instant BTN_DOWN is pressed (never suppressed, never waits for release).
    // The press is consumed here so releasing the button does not emit a second shortDown event.
    if (downEdge) {
        outEvents.downPressEdge = true;
        btnDown_.pressLatched   = false;
    }

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
        upHoldPrevActive_     = false;
        upHoldFired_          = false;

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

    // Single BTN_UP hold (1.2s) triggers Auto mode without requiring PS2 controller
    if (btnUp_.stableState && !btnDown_.stableState) {
        if (!upHoldPrevActive_) {
            upHoldPrevActive_ = true;
            upHoldFired_      = false;
            upHoldStartMs_    = nowMs;
        }
        const uint32_t elapsed = static_cast<uint32_t>(nowMs - upHoldStartMs_);
        if (elapsed >= Config::kAutoHoldBarDelayMs) {
            outEvents.autoHolding = true;
            float ratio           = static_cast<float>(elapsed) / static_cast<float>(Config::kAutoStartBtnHoldMs);
            if (ratio > 1.0f) {
                ratio = 1.0f;
            }
            outEvents.autoHoldRatio = ratio;
        }
        if (!upHoldFired_ && elapsed >= Config::kAutoStartBtnHoldMs) {
            upHoldFired_                 = true;
            btnUp_.pressLatched          = false; // Suppress shortUp release action after hold fires
            outEvents.autoStartRequested = true;
        }
    } else {
        upHoldPrevActive_ = false;
        upHoldFired_      = false;
    }

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
    if (!force &&
        (!isElapsed(nowMs, lastLowerHealthMs_, intervalMs) ||
         isCommBackoffActive(nowMs, Config::kLowerFastHealthMs))) {
        return false;
    }
    lastLowerHealthMs_ = nowMs;

    float v    = 0.0f;
    float perc = 0.0f;
    if (mmL.GetPowerInfo(v, perc) != MMLower::RESULT::OK) {
        lastLowerHealthMs_ = noteCommFailure();
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
        isElapsed(nowMs, lastConfigResyncMs_, Config::kLowerResyncConfigMs);

    if (!commHealthy_ || commNeedsHeal_ || brownoutRecovered || periodicIdleResync) {
        if (!commHealthy_ || commNeedsHeal_ || brownoutRecovered) {
            invalidateActuatorCaches();
        }
        const bool cfgOk    = applyLowerMcuConfig(brownoutRecovered || !commHealthy_ || commNeedsHeal_);
        lastConfigResyncMs_ = millis();
        if (cfgOk) {
            commHealthy_   = true;
            commNeedsHeal_ = false;
        } else {
            lastLowerHealthMs_ = noteCommFailure();
        }
    }

    return true;
}

void MiniR4IO::invalidateActuatorCaches() {
    motorsSynced_ = false;
    servosSynced_ = false;
}

bool MiniR4IO::shouldRefreshStoppedMotors(uint32_t nowMs) const {
    if (!motorsSynced_ || lastLeftPower_ != 0 || lastRightPower_ != 0) {
        const uint32_t retryInterval = commHealthy_
                                           ? Config::kCommRetryIntervalMs
                                           : Config::kCommUnhealthyRetryIntervalMs;
        if (isCommBackoffActive(nowMs, Config::kCommUnhealthyRetryIntervalMs)) {
            return false;
        }
        return isElapsed(nowMs, lastMotorAttemptMs_, retryInterval);
    }
    return isElapsed(nowMs, lastStopRefreshMs_, Config::kFailsafeRefreshMs);
}

void MiniR4IO::applyMotors(int16_t leftPower, int16_t rightPower, bool forceBrake, uint32_t nowMs) {
    const int16_t clampedL = RobotLogic::clamp16(leftPower, Config::kMotorMinPower, Config::kMotorMaxPower);
    const int16_t clampedR = RobotLogic::clamp16(rightPower, Config::kMotorMinPower, Config::kMotorMaxPower);

    const bool wantBrake    = (clampedL == 0 && clampedR == 0 && forceBrake);
    const bool brakeChanged = (wantBrake != lastBrakeEngaged_);
    const bool keepaliveDue = isElapsed(nowMs, lastMotorSendMs_, Config::kMotorKeepaliveMs);

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
    if ((!motorsSynced_ && !isElapsed(nowMs, lastMotorAttemptMs_, retryInterval)) ||
        isCommBackoffActive(nowMs, Config::kCommUnhealthyRetryIntervalMs)) {
        return;
    }
    lastMotorAttemptMs_ = nowMs;

    bool okL = true;
    bool okR = true;

    if (updateL) {
        if (wantBrake) {
            okL = MiniR4.M1.setBrake(true);
        } else {
            okL = setMotorPowerSingle(Config::kMotorLeftId, clampedL);
        }
        if (okL) {
            lastLeftPower_ = clampedL;
        }
    }

    if (updateR && okL) {
        if (wantBrake) {
            okR = MiniR4.M2.setBrake(true);
        } else {
            okR = setMotorPowerSingle(Config::kMotorRightId, clampedR);
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
        motorsSynced_       = false;
        lastMotorAttemptMs_ = noteCommFailure();
    }
}

void MiniR4IO::forceMotorsStop(bool engageBrake, uint32_t nowMs, bool forceImmediate) {
    if (lastLeftPower_ != 0 || lastRightPower_ != 0 || lastBrakeEngaged_ != engageBrake) {
        motorsSynced_ = false;
    }

    // Never drop an emergency stop while comm is healthy (or when explicitly forced for reboot).
    if (!forceImmediate && !commHealthy_ &&
        (isCommBackoffActive(nowMs, Config::kCommUnhealthyRetryIntervalMs) ||
         !isElapsed(nowMs, lastMotorAttemptMs_, Config::kCommUnhealthyRetryIntervalMs))) {
        return;
    }
    lastMotorAttemptMs_ = nowMs;

    bool ok1 = false;
    bool ok2 = false;
    if (engageBrake) {
        ok1 = MiniR4.M1.setBrake(true);
        ok2 = ok1 && MiniR4.M2.setBrake(true);
    } else {
        ok1 = setMotorPowerSingle(Config::kMotorLeftId, 0);
        ok2 = ok1 && setMotorPowerSingle(Config::kMotorRightId, 0);
    }

    bool auxOk = ok1 && ok2;
    if (auxOk && !auxMotorsZeroed_) {
        const bool ok3   = setMotorPowerSingle(3U, 0);
        const bool ok4   = ok3 && setMotorPowerSingle(4U, 0);
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
        motorsSynced_       = false;
        lastMotorAttemptMs_ = noteCommFailure();
    }
}

void MiniR4IO::restoreAfterAuto() {
    // Restore deterministic PS2 bit-bang SPI pins & DAT pull-up in case D2/D3 or MiniR4.PS2 ran.
    ps2_.restorePins();

    auxMotorsZeroed_ = false;
    invalidateActuatorCaches();
    lastLeftPower_    = 0;
    lastRightPower_   = 0;
    lastBrakeEngaged_ = false;

    // Stop any DriveDC task started by Auto block code; short-circuit if STM32 UART is unresponsive.
    const MMLower::Drive_RESULT driveBrakeRes = mmL.Set_Drive_Brake(true, 1U);
    const bool uartAlive = (driveBrakeRes == MMLower::Drive_RESULT::OK ||
                            driveBrakeRes == MMLower::Drive_RESULT::ERROR_Drive_Define);
    const bool cfgOk = uartAlive && applyLowerMcuConfig(true);
    const bool ok1   = cfgOk && MiniR4.M1.setBrake(true);
    const bool ok2   = ok1 && MiniR4.M2.setBrake(true);

    const uint32_t doneMs = millis();
    lastMotorAttemptMs_   = doneMs;
    lastConfigResyncMs_   = doneMs;
    lastBrakeEngaged_     = ok1 && ok2;
    motorsSynced_         = cfgOk && ok1 && ok2;
    if (motorsSynced_) {
        commHealthy_       = true;
        commNeedsHeal_     = false;
        lastMotorSendMs_   = doneMs;
        lastStopRefreshMs_ = doneMs;
    } else {
        lastMotorAttemptMs_ = noteCommFailure();
    }
}

void MiniR4IO::applyServos(const ServoTargets& targets, uint32_t nowMs) {
    const ServoTargets safe = RobotLogic::clampServoTargets(targets);

    // Smooth trajectory slew is applied ONLY to arm RC2; RC1, RC3 actuate immediately.
    const int16_t cmdRc2 = rc2SmoothEnabled_
                               ? RobotLogic::slewServoAngle(lastServos_.rc2Deg,
                                                            safe.rc2Deg,
                                                            Config::kRc2SmoothStepDeg)
                               : safe.rc2Deg;

    const bool keepaliveDue = isElapsed(nowMs, lastServoSendMs_, Config::kServoKeepaliveMs);

    const bool send1 = !servosSynced_ || (safe.rc1Deg != lastServos_.rc1Deg) || keepaliveDue;
    const bool send2 = !servosSynced_ || (cmdRc2 != lastServos_.rc2Deg) || keepaliveDue;
    const bool send3 = !servosSynced_ || (safe.rc3Deg != lastServos_.rc3Deg) || keepaliveDue;

    if (!send1 && !send2 && !send3) {
        return;
    }

    const uint32_t retryInterval = commHealthy_
                                       ? Config::kCommRetryIntervalMs
                                       : Config::kCommUnhealthyRetryIntervalMs;
    if ((!servosSynced_ && !isElapsed(nowMs, lastServoAttemptMs_, retryInterval)) ||
        isCommBackoffActive(nowMs, Config::kCommUnhealthyRetryIntervalMs)) {
        return;
    }
    lastServoAttemptMs_ = nowMs;

    // One packet carries all four STM32 servo channels; RC4 is unused (fixed neutral), so RC1..RC3
    // start moving on the same STM32 frame. A single failure costs at most one UART timeout per cycle.
    const int16_t hwRc3 = RobotLogic::applyServoDirection(safe.rc3Deg, Config::kRc3InvertDir);
    constexpr uint16_t kUnusedRc4Deg = 90U; // RC4 removed; packet still needs a 4th angle
    const bool    allOk =
        mmL.SetAllServoAngle(static_cast<uint16_t>(safe.rc1Deg), static_cast<uint16_t>(cmdRc2),
                             static_cast<uint16_t>(hwRc3), kUnusedRc4Deg) ==
        MMLower::RESULT::OK;
    if (allOk) {
        lastServos_.rc1Deg = safe.rc1Deg;
        lastServos_.rc2Deg = cmdRc2;
        lastServos_.rc3Deg = safe.rc3Deg;
        lastServoSendMs_   = nowMs;
        servosSynced_    = true;
    } else {
        servosSynced_       = false;
        lastServoAttemptMs_ = noteCommFailure();
    }
}
