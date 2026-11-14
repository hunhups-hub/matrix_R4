#include <Arduino.h>
#include <WDT.h>
#include <boot.h>

#include "config.h"
#include "minir4_io.h"
#include "oled_ui.h"
#include "robot_logic.h"
#include "serial_control.h"
#include "types.h"

namespace {

MiniR4IO                         g_io;
SerialControl                    g_serial;
OledUI                           g_oled;
RobotLogic::FailsafeStateMachine g_linkSm;
RobotLogic::BatteryStabilizer    g_battStab;
RobotLogic::StabilityMonitor     g_stabMon;

SystemSnapshot g_snap{};
Ps2Frame       g_lastPs2Frame{};
bool           g_serialDriveLatched = false;
int16_t        g_serialTargetL      = 0;
int16_t        g_serialTargetR      = 0;

uint32_t g_lastControlMs   = 0;
uint32_t g_lastArmStepMs   = 0;
uint32_t g_lastTelemetryMs = 0;

[[noreturn]] void enterUploadBootloader(const char* source) {
    WDT.refresh();
    g_io.forceMotorsStop(true, millis());
    g_oled.showBootloaderScreen();
    g_serial.printEvent("BOOT", source ? source : "Entering upload bootloader...");
    Serial.flush();
    delay(120);
    WDT.refresh();

#ifdef NO_USB
    // On UNO R4 WiFi (-DNO_USB), USB clock may be off; trigger SAM-BA double-tap via VBTBKR[0].
    R_SYSTEM->PRCR       = static_cast<uint16_t>(BSP_PRV_PRCR_PRC1_UNLOCK);
    BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;
    R_SYSTEM->PRCR       = static_cast<uint16_t>(BSP_PRV_PRCR_LOCK);
    __DSB();
    NVIC_SystemReset();
#else
    goBootloader();
#endif
    for (;;) {
    }
}

void triggerEstop(const char* reason, uint32_t nowMs) {
    g_serialDriveLatched = false;
    g_serialTargetL      = 0;
    g_serialTargetR      = 0;
    g_serial.stopMotion();
    if (g_snap.sysState != SystemState::Estop) {
        g_snap.sysState      = SystemState::Estop;
        g_snap.drive         = {0, 0};
        g_io.invalidateActuatorCaches();
        g_io.forceMotorsStop(true, nowMs);
        g_oled.beepWarning();
        g_serial.printEvent("ESTOP", reason);
    }
}

void clearEstop(const char* reason) {
    if (g_snap.sysState == SystemState::Estop) {
        g_snap.sysState = SystemState::Ready;
        g_io.invalidateActuatorCaches();
        g_oled.beepConfirm();
        g_serial.printEvent("STATE", reason);
    }
}

void logLinkTransition(LinkState prev, LinkState next) {
    if (prev == next) {
        return;
    }
    switch (next) {
        case LinkState::Disconnected:
            g_serial.printEvent("PS2", "DISCONNECTED (check D2+D3 & controller power)");
            break;
        case LinkState::Failsafe:
            g_serial.printEvent("PS2", "FAILSAFE -> Motors stopped");
            g_oled.beepWarning();
            break;
        case LinkState::Stabilizing:
            g_serial.printEvent("PS2", "STABILIZING...");
            break;
        case LinkState::Operational:
            g_serial.printEvent("PS2", "OPERATIONAL (PS2 Manual Ready)");
            g_oled.beepConfirm();
            break;
    }
}

void applyBatteryTelemetry(bool voltValid, uint16_t battMv, uint32_t nowMs) {
    const BatteryTier prevTier = g_battStab.tier();
    const BatteryTier nextTier = g_battStab.update(battMv, voltValid, nowMs);

    if (voltValid) {
        if (g_battStab.isExternalPower() && g_battStab.filteredMv() > 0U) {
            g_snap.battVolt = static_cast<float>(g_battStab.filteredMv()) / 1000.0f;
        } else {
            g_snap.battVolt = static_cast<float>(battMv) / 1000.0f;
        }
    }

    const MotorPowerProfile profile = g_battStab.motorProfile();
    g_snap.extPower    = g_battStab.isExternalPower();
    g_snap.battPct     = g_battStab.batteryPercent();
    g_snap.batTier     = nextTier;
    g_snap.powerCapPct = profile.maxPowerCap;
    g_snap.lowVoltWarn = (nextTier != BatteryTier::Normal) ||
                         (g_snap.extPower &&
                          g_battStab.filteredMv() <= Config::kLowVoltWarnMv);
    g_snap.commHealthy = g_io.commHealthy();

    if (prevTier != nextTier && nextTier != BatteryTier::Normal) {
        if (Config::kEnableBatterySagThrottle) {
            g_serial.printEvent("PWR",
                                nextTier == BatteryTier::Emergency ? "EMERGENCY sag cap (35%)"
                                : (nextTier == BatteryTier::Critical ? "CRITICAL sag cap (50%)"
                                                                     : "LOW sag cap (72%)"));
        } else {
            g_serial.printEvent("PWR",
                                nextTier == BatteryTier::Emergency ? "EMERGENCY low batt (100% full power)"
                                : (nextTier == BatteryTier::Critical ? "CRITICAL low batt (100% full power)"
                                                                     : "LOW batt warning (100% full power)"));
        }
    }
}

void applySerialImmediateActions(const SerialActions& act, uint32_t nowMs) {
    if (act.i2cScanRequested) {
        g_oled.printI2cDiag();
    }
    if (act.uploadRequested) {
        enterUploadBootloader("Serial command 'u'");
    }
    if (act.estopRequested) {
        triggerEstop("Serial '!'", nowMs);
    } else if (act.resumeRequested) {
        clearEstop("Serial 'r'");
    }

    if (act.toggleLeftReverse) {
        g_snap.leftReversed = !g_snap.leftReversed;
        g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
        g_serial.printEvent("CFG", g_snap.leftReversed ? "M1 reverse=ON" : "M1 reverse=OFF");
    }
    if (act.toggleRightReverse) {
        g_snap.rightReversed = !g_snap.rightReversed;
        g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
        g_serial.printEvent("CFG", g_snap.rightReversed ? "M2 reverse=ON" : "M2 reverse=OFF");
    }

    if (act.speedPctChanged) {
        g_snap.serialSpeedPct = act.newSpeedPct;
        Serial.print(F("[SER] Speed = "));
        Serial.print(g_snap.serialSpeedPct);
        Serial.println('%');
    }

    if (act.armUpStep) {
        g_snap.servos.rc2Deg = RobotLogic::computeArmStep(g_snap.servos.rc2Deg, true, false);
    } else if (act.armDownStep) {
        g_snap.servos.rc2Deg = RobotLogic::computeArmStep(g_snap.servos.rc2Deg, false, true);
    }

    if (act.gripOpen) {
        g_snap.servos.rc3Deg = Config::kRc3L1Deg;
        g_snap.servos.rc4Deg = Config::kRc4L1Deg;
    } else if (act.gripClose) {
        g_snap.servos.rc3Deg = Config::kRc3R1Deg;
        g_snap.servos.rc4Deg = Config::kRc4R1Deg;
    }

    if (act.auxExtend) {
        g_snap.servos.rc1Deg = Config::kRc1L2Deg;
    } else if (act.auxRetract) {
        g_snap.servos.rc1Deg = Config::kRc1R2Deg;
    }

    if (act.hasDirectServo) {
        switch (act.directServoCh) {
            case 1: g_snap.servos.rc1Deg = act.directServoDeg; break;
            case 2: g_snap.servos.rc2Deg = act.directServoDeg; break;
            case 3: g_snap.servos.rc3Deg = act.directServoDeg; break;
            case 4: g_snap.servos.rc4Deg = act.directServoDeg; break;
            default: break;
        }
    }

    g_snap.servos = RobotLogic::clampServoTargets(g_snap.servos);

    if (g_snap.sysState == SystemState::Estop) {
        g_serialDriveLatched = false;
        g_serialTargetL      = 0;
        g_serialTargetR      = 0;
        g_serial.stopMotion();
    } else {
        g_serialDriveLatched = act.driveCmd.active;
        g_serialTargetL      = act.driveCmd.targetLeft;
        g_serialTargetR      = act.driveCmd.targetRight;
    }
}

uint8_t classifyStabPenalty(uint8_t pen) {
    if (pen == 0U) {
        return 0U;
    }
    return (pen <= 10U) ? 1U : 2U;
}

void updateStabilityMetrics(bool frameValid, uint8_t compPercent, uint32_t loopDtMs) {
    RobotLogic::StabilityInput in{};
    in.ps2Link      = g_snap.ps2Link;
    in.activeSource = g_snap.activeSource;
    in.frameValid   = frameValid;
    in.commHealthy  = g_snap.commHealthy;
    in.hwFault      = (g_snap.servoRangeErr != 0U);
    in.extPower     = g_snap.extPower;
    in.battPct      = g_snap.battPct;
    in.batTier      = g_snap.batTier;
    in.compPct      = compPercent;
    in.loopDtMs     = loopDtMs;

    g_snap.stabilityPct   = g_stabMon.update(in);
    g_snap.loopJitterMs   = g_stabMon.jitterMs();
    g_snap.linkStabState  = classifyStabPenalty(g_stabMon.linkPenalty());
    g_snap.powerStabState = classifyStabPenalty(g_stabMon.powerPenalty());
    g_snap.commStabState  = classifyStabPenalty(g_stabMon.commPenalty());
    g_snap.loopStabState  = classifyStabPenalty(g_stabMon.loopPenalty());
}

void runControlTick(uint32_t nowMs, uint32_t loopDtMs) {
    ButtonEvents btn{};
    g_io.pollButtons(nowMs, btn);
    g_snap.comboHolding  = btn.comboHolding;
    g_snap.uploadHoldPct = btn.uploadHoldRatio;

    if (btn.uploadRequested) {
        enterUploadBootloader("BTN_UP + BTN_DOWN hold 2.5s");
    }
    if (btn.shortDown) {
        triggerEstop("BTN_DOWN press", nowMs);
    } else if (btn.shortUp) {
        if (g_snap.sysState == SystemState::Estop) {
            clearEstop("BTN_UP press");
        } else {
            g_snap.precisionMode = !g_snap.precisionMode;
            g_oled.beepConfirm();
            g_serial.printEvent("CFG", g_snap.precisionMode ? "Precision 55% ON" : "Precision OFF (100%)");
        }
    }

    const LinkState prevLink = g_linkSm.state();
    bool            valid    = false;

    if (!g_linkSm.isConnected()) {
        g_lastPs2Frame = Ps2Frame{};
        if (g_linkSm.shouldAttemptReinit(nowMs)) {
            const bool ready = g_io.tryReconnectPs2();
            g_linkSm.onReinitResult(ready, nowMs);
        }
    }

    if (g_linkSm.isConnected()) {
        g_io.pollPs2(g_lastPs2Frame);
        valid                = RobotLogic::isPs2FrameValid(g_lastPs2Frame);
        const bool modeReset = !valid && RobotLogic::isPs2UnconfiguredMode(g_lastPs2Frame);
        g_linkSm.onFrame(valid, nowMs, modeReset);
    }

    g_snap.ps2Link      = g_linkSm.state();
    g_snap.ps2Glitching = g_linkSm.isLinkGlitching();
    g_snap.ps2Mode      = g_lastPs2Frame.modeId;
    g_snap.ps2Sync      = g_lastPs2Frame.syncByte;
    g_snap.ps2Ly        = g_lastPs2Frame.ly;
    g_snap.ps2Rx        = g_lastPs2Frame.rx;
    logLinkTransition(prevLink, g_snap.ps2Link);

    const bool isMoving = (g_snap.drive.left != 0 || g_snap.drive.right != 0);
    bool       voltValid = false;
    uint16_t   battMv    = 0U;
    if (g_io.verifyAndHealLowerMcu(nowMs, isMoving, voltValid, battMv)) {
        applyBatteryTelemetry(voltValid, battMv, nowMs);
        g_snap.servoRangeErr = g_io.servoRangeErr();
    } else {
        g_snap.commHealthy = g_io.commHealthy();
    }

    const MotorPowerProfile pwrProfile = g_battStab.motorProfile();
    updateStabilityMetrics(valid, pwrProfile.compPercent, loopDtMs);

    if (g_snap.sysState == SystemState::Estop) {
        g_snap.activeSource = ControlSource::None;
        g_snap.drive        = {0, 0};
        if (g_io.shouldRefreshStoppedMotors(nowMs)) {
            g_io.forceMotorsStop(true, nowMs);
        }
    } else if (g_linkSm.isFailsafe()) {
        // Active PS2 link dropped abruptly: cut power & hold brake cleanly without UART flooding.
        const bool enteredFailsafe = (prevLink != LinkState::Failsafe);
        g_snap.activeSource = ControlSource::None;
        g_snap.drive        = {0, 0};
        if (enteredFailsafe) {
            g_io.invalidateActuatorCaches();
        }
        if (enteredFailsafe || g_io.shouldRefreshStoppedMotors(nowMs)) {
            g_io.forceMotorsStop(true, nowMs);
        }
    } else if (g_linkSm.isOperational() && valid && g_lastPs2Frame.cross) {
        // PS2 Cross acts as immediate brake override even during Serial driving.
        const bool wasMoving = (g_snap.drive.left != 0 || g_snap.drive.right != 0);
        g_serialDriveLatched = false;
        g_serialTargetL      = 0;
        g_serialTargetR      = 0;
        g_serial.stopMotion();
        g_snap.activeSource = ControlSource::Ps2;
        g_snap.drive        = {0, 0};
        if (wasMoving || g_io.shouldRefreshStoppedMotors(nowMs)) {
            g_io.forceMotorsStop(true, nowMs);
        }
    } else if (g_serialDriveLatched) {
        g_snap.activeSource = ControlSource::SerialCmd;
        const int16_t modeCap = g_snap.precisionMode ? Config::kPrecisionMaxPower : Config::kMotorMaxPower;
        const int16_t maxSpd  = (modeCap < pwrProfile.maxPowerCap) ? modeCap : pwrProfile.maxPowerCap;
        g_snap.drive          = RobotLogic::applyDriveTargetsWithSlew(
            g_serialTargetL,
            g_serialTargetR,
            g_snap.drive.left,
            g_snap.drive.right,
            maxSpd,
            pwrProfile.accelSlewStep,
            pwrProfile.decelSlewStep,
            pwrProfile.compPercent);
        g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, false, nowMs);
    } else if (g_linkSm.isOperational()) {
        g_snap.activeSource = ControlSource::Ps2;
        if (prevLink != LinkState::Operational) {
            g_lastArmStepMs = nowMs;
            g_io.invalidateActuatorCaches();
        }

        if (valid) {
            const bool    slow    = g_snap.precisionMode || g_lastPs2Frame.square;
            const int16_t modeCap = slow ? Config::kPrecisionMaxPower : Config::kMotorMaxPower;
            const int16_t maxSpd  = (modeCap < pwrProfile.maxPowerCap) ? modeCap : pwrProfile.maxPowerCap;
            g_snap.drive          = RobotLogic::computeDifferentialDrive(
                g_lastPs2Frame.ly,
                g_lastPs2Frame.rx,
                g_snap.drive.left,
                g_snap.drive.right,
                maxSpd,
                pwrProfile.accelSlewStep,
                pwrProfile.decelSlewStep,
                pwrProfile.compPercent);
            g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, false, nowMs);

            const bool allowArm =
                static_cast<uint32_t>(nowMs - g_lastArmStepMs) >= Config::kArmStepIntervalMs;
            const int16_t prevDpad = g_snap.servos.rc2Deg;
            RobotLogic::updateServoTargetsFromPs2(g_lastPs2Frame, g_snap.servos, allowArm);
            if (allowArm && g_snap.servos.rc2Deg != prevDpad) {
                g_lastArmStepMs = nowMs;
            }
        } else {
            // Brief wireless glitch (<100ms): preserve last safe drive & servo state without overwrite.
            g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, false, nowMs);
        }
    } else {
        g_snap.activeSource = ControlSource::None;
        g_snap.drive.left   = RobotLogic::applyAsymmetricSlewStep(
            g_snap.drive.left, 0, pwrProfile.accelSlewStep, pwrProfile.decelSlewStep);
        g_snap.drive.right  = RobotLogic::applyAsymmetricSlewStep(
            g_snap.drive.right, 0, pwrProfile.accelSlewStep, pwrProfile.decelSlewStep);
        g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, false, nowMs);
    }

    g_io.applyServos(g_snap.servos, nowMs);
    g_oled.render(g_snap, nowMs, false);
    g_snap.oledErr3d = g_oled.err3d();
    g_snap.oledErr3c = g_oled.err3c();
    g_snap.oledOk    = (g_oled.lastI2cErr() == 0U);

    if (g_serial.streamTelemetry() &&
        static_cast<uint32_t>(nowMs - g_lastTelemetryMs) >= Config::kTelemetryHeartbeatMs) {
        g_lastTelemetryMs = nowMs;
        g_serial.printSnapshot(g_snap);
    }
}

} // namespace

void setup() {
    g_snap.wdtResetBoot = (R_SYSTEM->RSTSR1_b.WDTRF != 0);

    g_serial.begin(Config::kSerialBaud);

    // Clear any mid-byte stuck SSD1306 slave on Wire1 before MiniR4.begin() starts OLED.
    uint8_t sdaBefore  = 1U;
    uint8_t sdaAfter   = 1U;
    uint8_t recoverClk = 0U;
    OledUI::recoverBus(sdaBefore, sdaAfter, recoverClk);

    const bool miniOk    = g_io.begin();
    g_snap.servoRangeErr = g_io.servoRangeErr();
    g_snap.oledOk        = g_oled.begin();
    g_snap.oledAddr      = g_oled.i2cAddr();
    g_snap.oledErr3d     = g_oled.err3d();
    g_snap.oledErr3c     = g_oled.err3c();
    const bool oledOk    = g_snap.oledOk;

    const uint32_t nowMs = millis();

    // Seed initial power/health sample cleanly
    bool     voltValid = false;
    uint16_t battMv    = 0U;
    g_io.verifyAndHealLowerMcu(nowMs, false, voltValid, battMv, true);
    applyBatteryTelemetry(voltValid, battMv, nowMs);

    const bool ps2Ready = g_io.isPs2ReadyAtBoot();
    g_linkSm.begin(ps2Ready, nowMs);
    if (ps2Ready) {
        g_io.pollPs2(g_lastPs2Frame);
        g_linkSm.onFrame(RobotLogic::isPs2FrameValid(g_lastPs2Frame), nowMs, false);
    }
    g_snap.ps2Link = g_linkSm.state();
    g_snap.ps2Mode = g_lastPs2Frame.modeId;
    g_snap.ps2Sync = g_lastPs2Frame.syncByte;
    updateStabilityMetrics(ps2Ready && RobotLogic::isPs2FrameValid(g_lastPs2Frame),
                           g_battStab.motorProfile().compPercent,
                           Config::kControlPeriodMs);

    g_lastControlMs   = nowMs;
    g_lastArmStepMs   = nowMs;
    g_lastTelemetryMs = nowMs;

    g_oled.render(g_snap, nowMs, true);
    g_oled.beepBoot();

    g_serial.printBanner();
    Serial.print(F("[OLED] SSD1306 addr=0x"));
    Serial.print(g_oled.i2cAddr(), HEX);
    Serial.print(oledOk ? F(" OK (Wire1 ") : F(" ERR="));
    if (!oledOk) {
        Serial.print(g_oled.lastI2cErr());
        Serial.print(F(", Wire1 "));
    }
    Serial.print(Config::kWireTimeoutUs / 1000UL);
    Serial.print(F("ms preSda="));
    Serial.print(sdaBefore);
    Serial.print(F("->"));
    Serial.print(sdaAfter);
    Serial.print(F(" clk="));
    Serial.print(recoverClk);
    Serial.print(F(" rec="));
    Serial.print(g_oled.bootSdaBefore());
    Serial.print(F("->"));
    Serial.print(g_oled.bootSdaAfter());
    Serial.print('/');
    Serial.println(g_oled.bootRecoverClocks());
    if (!miniOk) {
        g_serial.printEvent("WARN", "MiniR4 init returned non-OK");
    }
    if (g_io.servoRangeErr() != 0U) {
        Serial.print(F("[WARN] Servo range err="));
        Serial.println(g_io.servoRangeErr());
    } else {
        Serial.print(F("[CFG] RC1..RC4 range 0..180deg OK | L1(RC3,RC4)=("));
        Serial.print(Config::kRc3L1Deg);
        Serial.print(',');
        Serial.print(Config::kRc4L1Deg);
        Serial.println(')');
    }
    if (g_snap.wdtResetBoot) {
        g_serial.printEvent("WARN", "Previous reset caused by Watchdog");
    }
    g_serial.printSnapshot(g_snap);

    // Enable WDT only after MiniR4 STM32 + staggered servo + PS2 boot init completes.
    if (!WDT.begin(Config::kWdtTimeoutMs)) {
        g_serial.printEvent("WARN", "WDT init failed");
    }
}

void loop() {
    WDT.refresh();

    const uint32_t nowMs = millis();

    SerialActions serialAct{};
    g_serial.poll(nowMs, g_snap.serialSpeedPct, serialAct);
    applySerialImmediateActions(serialAct, nowMs);

    const uint32_t dtMs = static_cast<uint32_t>(nowMs - g_lastControlMs);
    if (dtMs >= Config::kControlPeriodMs) {
        if (dtMs >= (Config::kControlPeriodMs * 2UL)) {
            // Resync cleanly after stalls/UART timeouts instead of burst catch-up ticks.
            g_lastControlMs = nowMs;
        } else {
            g_lastControlMs += Config::kControlPeriodMs;
        }
        runControlTick(nowMs, dtMs);
    }
}
