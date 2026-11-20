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

bool MiniR4IO::readImuYawDeg(int16_t& outYawDeg) {
    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
    if (mmL.GetIMUEuler(roll, pitch, yaw) != MMLower::RESULT::OK) {
        return false;
    }
    // Rejects NaN/garbage before the float->int conversion.
    if (!(yaw > -1.0e6 && yaw < 1.0e6)) {
        return false;
    }
    const int32_t rounded = static_cast<int32_t>(yaw >= 0.0 ? yaw + 0.5 : yaw - 0.5);
    outYawDeg             = RobotLogic::wrapYawDeg(rounded);
    return true;
}

bool MiniR4IO::readYawDeg(uint32_t nowMs, int16_t& outYawDeg) {
    if (!commHealthy_ || isCommBackoffActive(nowMs, Config::kCommUnhealthyRetryIntervalMs)) {
        return false;
    }
    if (!readImuYawDeg(outYawDeg)) {
        (void)noteCommFailure();
        return false;
    }
    return true;
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

bool MiniR4IO::configureRc34PulseRange(uint8_t ch) {
    struct PulseCandidate {
        uint16_t minUs;
        uint16_t maxUs;
    };
    const PulseCandidate candidates[] = {
        {Config::kRc34PulseMinUs, Config::kRc34PulseMaxUs},
        {544U, 2400U},
        {600U, 2400U},
    };

    MMLower::RESULT lastRes = MMLower::RESULT::OK;
    for (const PulseCandidate& c : candidates) {
        lastRes = mmL.SetServoPulseRange(ch, c.minUs, c.maxUs);
        if (lastRes == MMLower::RESULT::OK) {
            return true;
        }
        if (lastRes != MMLower::RESULT::ERROR_SERVO_MIN_PULSE &&
            lastRes != MMLower::RESULT::ERROR_SERVO_MAX_PULSE) {
            if (isServoParamError(lastRes) && rc34PulseErr_ == 0U) {
                rc34PulseErr_ = static_cast<uint8_t>(lastRes);
            }
            return false;
        }
    }

    // Parameter-range rejection by STM32: expose via rngErr telemetry without flagging UART dead.
    if (rc34PulseErr_ == 0U) {
        rc34PulseErr_ = static_cast<uint8_t>(lastRes);
    }
    return true;
}

bool MiniR4IO::configureServoRanges() {
    servoRangeErr_ = 0U;
    if (Config::kEnableRc34CustomPulse && !rc34PulseSynced_) {
        rc34PulseErr_ = 0U;
    }
    for (uint8_t ch = 1U; ch <= 4U; ++ch) {
        if (Config::kEnableRc34CustomPulse && !rc34PulseSynced_ && (ch == 3U || ch == 4U)) {
            if (!configureRc34PulseRange(ch)) {
                servoRangeErr_ = rc34PulseErr_;
                return false;
            }
        }
        const MMLower::RESULT angleRes = mmL.SetServoAngleRange(
            ch,
            static_cast<uint16_t>(Config::kServoMinDeg),
            static_cast<uint16_t>(Config::kServoMaxDeg));
        if (angleRes != MMLower::RESULT::OK) {
            if (isServoParamError(angleRes) && servoRangeErr_ == 0U) {
                servoRangeErr_ = static_cast<uint8_t>(angleRes);
            }
            if (servoRangeErr_ == 0U) {
                servoRangeErr_ = rc34PulseErr_;
            }
            return false;
        }
    }
    if (Config::kEnableRc34CustomPulse && !rc34PulseSynced_) {
        rc34PulseSynced_ = true;
    }
    if (servoRangeErr_ == 0U) {
        servoRangeErr_ = rc34PulseErr_;
    }
    return true;
}

void MiniR4IO::staggerBootServos() {
    const ServoTargets safeInit = RobotLogic::clampServoTargets(ServoTargets{});
    const int16_t      hwRc3    = RobotLogic::applyServoDirection(safeInit.rc3Deg, Config::kRc3InvertDir);
    const int16_t      hwRc4    = RobotLogic::applyServoDirection(safeInit.rc4Deg, Config::kRc4InvertDir);
    const bool ok1 = MiniR4.RC1.setAngle(static_cast<uint16_t>(safeInit.rc1Deg));
    if (ok1) {
        delay(Config::kServoBootStaggerMs);
    }
    const bool ok2 = ok1 && MiniR4.RC2.setAngle(static_cast<uint16_t>(safeInit.rc2Deg));
    if (ok2) {
        delay(Config::kServoBootStaggerMs);
    }

    // Engage paired RC3+RC4 together in a single UART frame to avoid one-sided twist at boot.
    const bool ok34 = ok2 &&
                      (mmL.SetAllServoAngle(static_cast<uint16_t>(safeInit.rc1Deg),
                                            static_cast<uint16_t>(safeInit.rc2Deg),
                                            static_cast<uint16_t>(hwRc3),
                                            static_cast<uint16_t>(hwRc4)) == MMLower::RESULT::OK);

    lastServos_   = safeInit;
    servosSynced_ = ok1 && ok2 && ok34;
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

    outEvents.upPressed   = btnUp_.stableState;
    outEvents.downPressed = btnDown_.stableState;

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
            rc34PulseSynced_ = false;
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

void MiniR4IO::restoreAfterAuto(uint32_t nowMs) {
    (void)nowMs;
    // Restore deterministic PS2 bit-bang SPI pins & DAT pull-up in case D2/D3 or MiniR4.PS2 ran.
    ps2_.restorePins();

    rc34PulseSynced_ = false;
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

    // Smooth trajectory slew is applied ONLY to arm RC2; RC1, RC3, RC4 actuate immediately.
    const int16_t cmdRc2 = rc2SmoothEnabled_
                               ? RobotLogic::slewServoAngle(lastServos_.rc2Deg,
                                                            safe.rc2Deg,
                                                            Config::kRc2SmoothStepDeg)
                               : safe.rc2Deg;

    const bool keepaliveDue = isElapsed(nowMs, lastServoSendMs_, Config::kServoKeepaliveMs);

    const bool send1 = !servosSynced_ || (safe.rc1Deg != lastServos_.rc1Deg) || keepaliveDue;
    const bool send2 = !servosSynced_ || (cmdRc2 != lastServos_.rc2Deg) || keepaliveDue;
    const bool send3 = !servosSynced_ || (safe.rc3Deg != lastServos_.rc3Deg) || keepaliveDue;
    const bool send4 = !servosSynced_ || (safe.rc4Deg != lastServos_.rc4Deg) || keepaliveDue;

    if (!send1 && !send2 && !send3 && !send4) {
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

    bool allOk = false;

    const uint8_t sendCount = static_cast<uint8_t>(send1) +
                              static_cast<uint8_t>(send2) +
                              static_cast<uint8_t>(send3) +
                              static_cast<uint8_t>(send4);

    // Send all 4 channels in a single atomic UART frame whenever paired RC3/RC4 move
    // or 2+ channels change, preventing left/right arm desync and multi-timeout cascades.
    if (send3 || send4 || sendCount >= 2U) {
        const uint16_t hwRc1 = static_cast<uint16_t>(safe.rc1Deg);
        const uint16_t hwRc2 = static_cast<uint16_t>(cmdRc2);
        const uint16_t hwRc3 = static_cast<uint16_t>(
            RobotLogic::applyServoDirection(safe.rc3Deg, Config::kRc3InvertDir));
        const uint16_t hwRc4 = static_cast<uint16_t>(
            RobotLogic::applyServoDirection(safe.rc4Deg, Config::kRc4InvertDir));

        if (mmL.SetAllServoAngle(hwRc1, hwRc2, hwRc3, hwRc4) == MMLower::RESULT::OK) {
            lastServos_.rc1Deg = safe.rc1Deg;
            lastServos_.rc2Deg = cmdRc2;
            lastServos_.rc3Deg = safe.rc3Deg;
            lastServos_.rc4Deg = safe.rc4Deg;
            allOk              = true;
        }
    } else if (send1) {
        if (MiniR4.RC1.setAngle(static_cast<uint16_t>(safe.rc1Deg))) {
            lastServos_.rc1Deg = safe.rc1Deg;
            allOk              = true;
        }
    } else if (send2) {
        if (MiniR4.RC2.setAngle(static_cast<uint16_t>(cmdRc2))) {
            lastServos_.rc2Deg = cmdRc2;
            allOk              = true;
        }
    }

    if (allOk) {
        lastServoSendMs_ = nowMs;
        servosSynced_    = true;
    } else {
        servosSynced_       = false;
        lastServoAttemptMs_ = noteCommFailure();
    }
}
