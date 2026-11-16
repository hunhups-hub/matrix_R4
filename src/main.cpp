#include <Arduino.h>
#include <WDT.h>
#include <boot.h>

#include "auto_mode.h"
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

uint32_t           g_lastControlMs        = 0;
uint32_t           g_lastArmStepMs        = 0;
uint32_t           g_lastTelemetryMs      = 0;
bool               g_prevTrianglePressed  = false;
bool               g_prevSelectPressed    = false;
bool               g_prevL3Pressed        = false;
bool               g_prevR3Pressed        = false;
bool               g_prevDpadLeftPressed  = false;
bool               g_prevDpadRightPressed = false;
bool               g_serialAutoRequested  = false;
FrontArmPushState  g_armPushState{};
ArmPosePresetState g_posePresetState{};

void savePersistentConfig() {
    PersistentConfig cfg{};
    cfg.leftReversed     = g_snap.leftReversed;
    cfg.rightReversed    = g_snap.rightReversed;
    cfg.precisionMode    = false; // Session-only: always boot in 100% normal mode
    cfg.rc2SmoothEnabled = g_snap.rc2SmoothEnabled;
    cfg.motorTrimPct     = g_snap.motorTrimPct;
    g_io.saveConfigToEeprom(cfg);
}

void printTrimChanged(const char* src, int8_t trimPct) {
    Serial.print(F("[CFG] "));
    Serial.print(src);
    Serial.print(F(" Motor Trim = "));
    if (trimPct > 0) {
        Serial.print('+');
    }
    Serial.print(static_cast<int>(trimPct));
    Serial.println(F("% (saved to EEPROM)"));
}

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
        g_serialAutoRequested = false;
        triggerEstop("Serial '!'", nowMs);
    } else if (act.resumeRequested) {
        clearEstop("Serial 'r'");
    }

    if (act.autoStartRequested && g_snap.sysState == SystemState::Ready) {
        g_serialAutoRequested = true;
    }
    if (act.togglePrecision) {
        g_snap.precisionMode = !g_snap.precisionMode;
        g_oled.beepConfirm();
        g_serial.printEvent("CFG", g_snap.precisionMode ? "Precision 55% ON (session)"
                                                        : "Precision OFF 100%");
    }
    if (act.toggleReverseDrive) {
        g_snap.reverseDrive = !g_snap.reverseDrive;
        g_oled.beepConfirm();
        g_serial.printEvent("CFG", g_snap.reverseDrive ? "Reverse-Front ON" : "Reverse-Front OFF");
    }
    if (act.toggleRc2Smooth) {
        g_snap.rc2SmoothEnabled = !g_snap.rc2SmoothEnabled;
        g_io.setRc2SmoothEnabled(g_snap.rc2SmoothEnabled);
        savePersistentConfig();
        g_oled.beepConfirm();
        g_serial.printEvent("CFG", g_snap.rc2SmoothEnabled ? "RC2 Smooth Slew ON (saved)"
                                                           : "RC2 Smooth Slew OFF (saved)");
    }

    if (act.trimLeftStep) {
        g_snap.motorTrimPct = RobotLogic::stepMotorTrim(g_snap.motorTrimPct, -1);
        savePersistentConfig();
        g_oled.beepConfirm();
        printTrimChanged("Serial", g_snap.motorTrimPct);
    } else if (act.trimRightStep) {
        g_snap.motorTrimPct = RobotLogic::stepMotorTrim(g_snap.motorTrimPct, 1);
        savePersistentConfig();
        g_oled.beepConfirm();
        printTrimChanged("Serial", g_snap.motorTrimPct);
    } else if (act.trimReset) {
        g_snap.motorTrimPct = 0;
        savePersistentConfig();
        g_oled.beepConfirm();
        printTrimChanged("Serial", g_snap.motorTrimPct);
    }

    if (act.toggleLeftReverse) {
        g_snap.leftReversed = !g_snap.leftReversed;
        g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
        savePersistentConfig();
        g_serial.printEvent("CFG", g_snap.leftReversed ? "M1 reverse=ON (saved)"
                                                       : "M1 reverse=OFF (saved)");
    }
    if (act.toggleRightReverse) {
        g_snap.rightReversed = !g_snap.rightReversed;
        g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
        savePersistentConfig();
        g_serial.printEvent("CFG", g_snap.rightReversed ? "M2 reverse=ON (saved)"
                                                        : "M2 reverse=OFF (saved)");
    }

    if (act.speedPctChanged) {
        g_snap.serialSpeedPct = act.newSpeedPct;
        Serial.print(F("[SER] Speed = "));
        Serial.print(g_snap.serialSpeedPct);
        Serial.println('%');
    }

    if (act.armUpStep) {
        g_snap.servos.rc2Deg          = RobotLogic::computeArmStep(g_snap.servos.rc2Deg, true, false);
        g_posePresetState.dropActive  = false;
    } else if (act.armDownStep) {
        g_snap.servos.rc2Deg          = RobotLogic::computeArmStep(g_snap.servos.rc2Deg, false, true);
        g_posePresetState.dropActive  = false;
    }

    if (act.gripOpen) {
        g_snap.servos.rc3Deg  = Config::kRc3L1Deg;
        g_snap.servos.rc4Deg  = Config::kRc4L1Deg;
        g_armPushState.active = false;
    } else if (act.gripClose) {
        g_snap.servos.rc3Deg  = Config::kRc3R1Deg;
        g_snap.servos.rc4Deg  = Config::kRc4R1Deg;
        g_armPushState.active = false;
    }

    if (act.auxExtend) {
        g_snap.servos.rc1Deg         = Config::kRc1L2Deg;
        g_posePresetState.dropActive = false;
    } else if (act.auxRetract) {
        g_snap.servos.rc1Deg         = Config::kRc1R2Deg;
        g_posePresetState.dropActive = false;
    } else if (act.togglePosePreset) {
        if (!g_posePresetState.dropActive) {
            g_snap.servos.rc1Deg         = Config::kRc1PresetDropDeg;
            g_snap.servos.rc2Deg         = Config::kRc2PresetDropDeg;
            g_posePresetState.dropActive = true;
            g_serial.printEvent("POSE", "Drop/Place preset (RC1=145, RC2=140)");
        } else {
            g_snap.servos.rc1Deg         = Config::kRc1PresetHoldDeg;
            g_snap.servos.rc2Deg         = Config::kRc2PresetHoldDeg;
            g_posePresetState.dropActive = false;
            g_serial.printEvent("POSE", "Hold/Carry preset (RC1=75, RC2=70)");
        }
        g_oled.beepConfirm();
    }

    if (act.hasDirectServo) {
        switch (act.directServoCh) {
            case 1:
                g_snap.servos.rc1Deg         = act.directServoDeg;
                g_posePresetState.dropActive = false;
                break;
            case 2:
                g_snap.servos.rc2Deg         = act.directServoDeg;
                g_posePresetState.dropActive = false;
                break;
            case 3:
                g_snap.servos.rc3Deg  = act.directServoDeg;
                g_armPushState.active = false;
                break;
            case 4:
                g_snap.servos.rc4Deg  = act.directServoDeg;
                g_armPushState.active = false;
                break;
            default: break;
        }
    }

    g_snap.servos      = RobotLogic::clampServoTargets(g_snap.servos);
    g_snap.comScalePct = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);

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

void runAutoSession(uint32_t nowMs) {
    g_serialAutoRequested = false;
    g_serialDriveLatched  = false;
    g_serialTargetL       = 0;
    g_serialTargetR       = 0;
    g_serial.stopMotion();
    g_io.forceMotorsStop(true, nowMs);

    g_snap.activeSource  = ControlSource::Auto;
    g_snap.drive         = {0, 0};
    g_snap.autoHolding   = false;
    g_snap.autoHoldPct   = 0.0f;
    g_snap.autoElapsedMs = 0UL;
    g_oled.render(g_snap, nowMs, true);
    g_oled.beepConfirm();
    g_serial.printEvent("AUTO", "Started -> Press X / BTN_UP / 'x' to exit");

    const int16_t preAutoRc1 = g_snap.servos.rc1Deg;
    const int16_t preAutoRc2 = g_snap.servos.rc2Deg;
    const int16_t preAutoRc3 = g_snap.servos.rc3Deg;
    const int16_t preAutoRc4 = g_snap.servos.rc4Deg;
    const AutoExitReason reason =
        AutoMode::execute(g_io, g_linkSm, g_snap, g_lastPs2Frame, &g_oled);
    const uint32_t exitMs = millis();

    if (g_snap.servos.rc1Deg != preAutoRc1 || g_snap.servos.rc2Deg != preAutoRc2) {
        g_posePresetState.dropActive = false;
    }
    if (g_snap.servos.rc3Deg != preAutoRc3 || g_snap.servos.rc4Deg != preAutoRc4) {
        g_armPushState.active = false;
    }

    g_io.restoreAfterAuto(exitMs);
    g_snap.drive           = {0, 0};
    g_snap.holdBrakeActive = g_io.isBrakeEngaged();
    g_snap.comboHolding    = false;
    g_snap.uploadHoldPct   = 0.0f;
    g_snap.autoHolding     = false;
    g_snap.autoHoldPct     = 0.0f;
    g_snap.comScalePct     = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);

    if (reason == AutoExitReason::UploadCombo || reason == AutoExitReason::SerialUpload) {
        enterUploadBootloader(AutoMode::exitReasonStr(reason));
    }

    bool validAfter = false;
    if (g_linkSm.isConnected()) {
        g_io.pollPs2(g_lastPs2Frame);
        validAfter = RobotLogic::isPs2FrameValid(g_lastPs2Frame);
        if (!g_linkSm.isOperational() && validAfter) {
            g_linkSm.onFrame(true, exitMs, false);
        }
        g_snap.ps2Link      = g_linkSm.state();
        g_snap.ps2Glitching = g_linkSm.isLinkGlitching();
        g_snap.ps2Mode      = g_lastPs2Frame.modeId;
        g_snap.ps2Sync      = g_lastPs2Frame.syncByte;
        g_snap.ps2Ly        = g_lastPs2Frame.ly;
        g_snap.ps2Rx        = g_lastPs2Frame.rx;
    } else {
        g_lastPs2Frame = Ps2Frame{};
    }
    g_prevTrianglePressed        = validAfter && g_lastPs2Frame.triangle;
    g_prevSelectPressed          = validAfter && g_lastPs2Frame.select;
    g_prevL3Pressed              = validAfter && g_lastPs2Frame.l3;
    g_prevR3Pressed              = validAfter && g_lastPs2Frame.r3;
    g_prevDpadLeftPressed        = validAfter && g_lastPs2Frame.left;
    g_prevDpadRightPressed       = validAfter && g_lastPs2Frame.right;
    g_armPushState.prevSquare    = validAfter && g_lastPs2Frame.square;
    g_posePresetState.prevCircle = validAfter && g_lastPs2Frame.circle;

    g_snap.oledOk    = g_oled.begin();
    g_snap.oledAddr  = g_oled.i2cAddr();
    g_snap.oledErr3d = g_oled.err3d();
    g_snap.oledErr3c = g_oled.err3c();

    if (reason == AutoExitReason::BtnEstop || reason == AutoExitReason::SerialEstop) {
        triggerEstop(AutoMode::exitReasonStr(reason), exitMs);
        g_snap.activeSource = ControlSource::None;
    } else {
        g_snap.activeSource =
            g_linkSm.isOperational() ? ControlSource::Ps2 : ControlSource::None;
        if (reason == AutoExitReason::StallTimeout) {
            g_oled.beepWarning();
        } else {
            g_oled.beepConfirm();
        }
        g_serial.printEvent("AUTO", AutoMode::exitReasonStr(reason));
    }

    bool     voltValid = false;
    uint16_t battMv    = 0U;
    g_io.verifyAndHealLowerMcu(exitMs, false, voltValid, battMv, true);
    applyBatteryTelemetry(voltValid, battMv, exitMs);
    g_snap.servoRangeErr = g_io.servoRangeErr();
    g_io.syncServoCache(g_snap.servos);
    g_io.applyServos(g_snap.servos, exitMs);
    g_oled.render(g_snap, exitMs, true);

    g_lastControlMs = exitMs;
    g_lastArmStepMs = exitMs;
}

void runControlTick(uint32_t nowMs, uint32_t loopDtMs) {
    ButtonEvents btn{};
    g_io.pollButtons(nowMs, btn);
    g_snap.comboHolding  = btn.comboHolding;
    g_snap.uploadHoldPct = btn.uploadHoldRatio;
    g_snap.autoHolding   = btn.autoHolding;
    g_snap.autoHoldPct   = btn.autoHoldRatio;

    if (btn.uploadRequested) {
        enterUploadBootloader("BTN_UP + BTN_DOWN hold 2.5s");
    }
    if (btn.shortDown) {
        g_serialAutoRequested = false;
        triggerEstop("BTN_DOWN press", nowMs);
    } else if (btn.shortUp) {
        if (g_snap.sysState == SystemState::Estop) {
            clearEstop("BTN_UP press");
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

    const bool sysReady = (g_snap.sysState == SystemState::Ready);

    // PS2 Select -> Precision Mode toggle, PS2 R3 -> Reverse-Front Orientation toggle,
    // PS2 L3 -> RC2 Smooth Trajectory toggle (or Start+L3 -> Reset Motor Trim),
    // PS2 Start + D-Pad Left/Right -> Adjust straight-line Motor Balance Trim (-/+1%).
    if (sysReady && g_linkSm.isOperational() && valid) {
        if (g_lastPs2Frame.select && !g_prevSelectPressed) {
            g_snap.precisionMode = !g_snap.precisionMode;
            g_oled.beepConfirm();
            g_serial.printEvent("CFG", g_snap.precisionMode ? "PS2 Select: Precision 55% ON (session)"
                                                            : "PS2 Select: Precision OFF 100%");
        }
        if (g_lastPs2Frame.r3 && !g_prevR3Pressed) {
            g_snap.reverseDrive = !g_snap.reverseDrive;
            g_oled.beepConfirm();
            g_serial.printEvent("CFG", g_snap.reverseDrive ? "PS2 R3: Reverse-Front ON"
                                                           : "PS2 R3: Reverse-Front OFF");
        }
        if (g_lastPs2Frame.l3 && !g_prevL3Pressed) {
            if (g_lastPs2Frame.start) {
                g_snap.motorTrimPct = 0;
                savePersistentConfig();
                g_oled.beepConfirm();
                printTrimChanged("PS2 Start+L3", g_snap.motorTrimPct);
            } else {
                g_snap.rc2SmoothEnabled = !g_snap.rc2SmoothEnabled;
                g_io.setRc2SmoothEnabled(g_snap.rc2SmoothEnabled);
                savePersistentConfig();
                g_oled.beepConfirm();
                g_serial.printEvent("CFG", g_snap.rc2SmoothEnabled
                                               ? "PS2 L3: RC2 Smooth Slew ON (saved)"
                                               : "PS2 L3: RC2 Smooth Slew OFF (saved)");
            }
        }
        if (g_lastPs2Frame.start) {
            if (g_lastPs2Frame.left && !g_prevDpadLeftPressed) {
                g_snap.motorTrimPct = RobotLogic::stepMotorTrim(g_snap.motorTrimPct, -1);
                savePersistentConfig();
                g_oled.beepConfirm();
                printTrimChanged("PS2 Start+Left", g_snap.motorTrimPct);
            } else if (g_lastPs2Frame.right && !g_prevDpadRightPressed) {
                g_snap.motorTrimPct = RobotLogic::stepMotorTrim(g_snap.motorTrimPct, 1);
                savePersistentConfig();
                g_oled.beepConfirm();
                printTrimChanged("PS2 Start+Right", g_snap.motorTrimPct);
            }
        }
    }

    const bool ps2StartAuto = RobotLogic::shouldStartAutoMode(sysReady,
                                                              g_linkSm.isOperational(),
                                                              valid,
                                                              g_lastPs2Frame.triangle,
                                                              g_prevTrianglePressed,
                                                              g_lastPs2Frame.cross);
    const bool startAuto = ps2StartAuto ||
                           (sysReady && (btn.autoStartRequested || g_serialAutoRequested));
    g_serialAutoRequested = false;

    if (!g_linkSm.isConnected() || g_linkSm.isFailsafe()) {
        g_prevTrianglePressed  = false;
        g_prevSelectPressed    = false;
        g_prevL3Pressed        = false;
        g_prevR3Pressed        = false;
        g_prevDpadLeftPressed  = false;
        g_prevDpadRightPressed = false;
    } else if (valid) {
        g_prevTrianglePressed  = g_lastPs2Frame.triangle;
        g_prevSelectPressed    = g_lastPs2Frame.select;
        g_prevL3Pressed        = g_lastPs2Frame.l3;
        g_prevR3Pressed        = g_lastPs2Frame.r3;
        g_prevDpadLeftPressed  = g_lastPs2Frame.left;
        g_prevDpadRightPressed = g_lastPs2Frame.right;
    }

    if (startAuto) {
        runAutoSession(nowMs);
        return;
    }

    bool ps2ServosUpdated = false;
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
        const int16_t modeCap =
            g_snap.precisionMode ? Config::kPrecisionMaxPower : Config::kMotorMaxPower;
        const int16_t maxSpd  = (modeCap < pwrProfile.maxPowerCap) ? modeCap : pwrProfile.maxPowerCap;
        int16_t effTargetL    = g_snap.reverseDrive ? static_cast<int16_t>(-g_serialTargetR) : g_serialTargetL;
        int16_t effTargetR    = g_snap.reverseDrive ? static_cast<int16_t>(-g_serialTargetL) : g_serialTargetR;
        RobotLogic::applyMotorBalanceTrim(effTargetL, effTargetR, g_snap.motorTrimPct);
        g_snap.drive          = RobotLogic::applyDriveTargetsWithSlew(
            effTargetL,
            effTargetR,
            g_snap.drive.left,
            g_snap.drive.right,
            maxSpd,
            pwrProfile.accelSlewStep,
            pwrProfile.decelSlewStep,
            pwrProfile.compPercent);
        g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, Config::kEnableSmartHoldBrake, nowMs);
    } else if (g_linkSm.isOperational()) {
        g_snap.activeSource = ControlSource::Ps2;
        if (prevLink != LinkState::Operational) {
            g_lastArmStepMs = nowMs;
            g_io.invalidateActuatorCaches();
        }

        if (valid) {
            const bool allowArm =
                static_cast<uint32_t>(nowMs - g_lastArmStepMs) >= Config::kArmStepIntervalMs;
            const int16_t prevDpad = g_snap.servos.rc2Deg;
            RobotLogic::updateServoTargetsFromPs2(
                g_lastPs2Frame, g_snap.servos, allowArm, &g_armPushState, &g_posePresetState);
            ps2ServosUpdated = true;
            if (allowArm && g_snap.servos.rc2Deg != prevDpad) {
                g_lastArmStepMs = nowMs;
            }
            g_snap.comScalePct = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);

            const int16_t modeCap =
                g_snap.precisionMode ? Config::kPrecisionMaxPower : Config::kMotorMaxPower;
            const int16_t maxSpd  = (modeCap < pwrProfile.maxPowerCap) ? modeCap : pwrProfile.maxPowerCap;
            const bool microLeft  = !g_lastPs2Frame.start && g_lastPs2Frame.left;
            const bool microRight = !g_lastPs2Frame.start && g_lastPs2Frame.right;
            g_snap.drive          = RobotLogic::computeKinodynamicDrive(
                g_lastPs2Frame.ly,
                g_lastPs2Frame.rx,
                g_snap.drive.left,
                g_snap.drive.right,
                maxSpd,
                pwrProfile.accelSlewStep,
                pwrProfile.decelSlewStep,
                pwrProfile.compPercent,
                g_snap.servos.rc2Deg,
                g_snap.reverseDrive,
                microLeft,
                microRight,
                g_snap.motorTrimPct,
                g_snap.precisionMode);
            g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, Config::kEnableSmartHoldBrake, nowMs);
        } else {
            // Brief wireless glitch (<100ms): preserve last safe drive & servo state without overwrite.
            g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, Config::kEnableSmartHoldBrake, nowMs);
        }
    } else {
        g_snap.activeSource = ControlSource::None;
        g_snap.drive        = RobotLogic::apply2DofKinodynamicSlew(
            0,
            0,
            g_snap.drive.left,
            g_snap.drive.right,
            pwrProfile.accelSlewStep,
            pwrProfile.decelSlewStep,
            Config::kAngularAccelSlewStep,
            Config::kAngularDecelSlewStep);
        g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, Config::kEnableSmartHoldBrake, nowMs);
    }
    g_snap.holdBrakeActive = g_io.isBrakeEngaged();

    if (!g_linkSm.isConnected() || g_linkSm.isFailsafe()) {
        g_armPushState.prevSquare    = false;
        g_posePresetState.prevCircle = false;
    } else if (!ps2ServosUpdated && valid) {
        g_armPushState.prevSquare    = g_lastPs2Frame.square;
        g_posePresetState.prevCircle = g_lastPs2Frame.circle;
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

    PersistentConfig savedCfg{};
    const bool eepromLoaded = g_io.loadConfigFromEeprom(savedCfg);
    if (eepromLoaded) {
        g_snap.leftReversed     = savedCfg.leftReversed;
        g_snap.rightReversed    = savedCfg.rightReversed;
        g_snap.precisionMode    = false;
        g_snap.rc2SmoothEnabled = savedCfg.rc2SmoothEnabled;
        g_snap.motorTrimPct     = savedCfg.motorTrimPct;
        g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
        g_io.setRc2SmoothEnabled(g_snap.rc2SmoothEnabled);
    } else {
        g_io.setRc2SmoothEnabled(g_snap.rc2SmoothEnabled);
    }
    g_snap.holdBrakeActive = g_io.isBrakeEngaged();
    g_snap.servoRangeErr   = g_io.servoRangeErr();
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
        const bool bootValid = RobotLogic::isPs2FrameValid(g_lastPs2Frame);
        g_linkSm.onFrame(bootValid, nowMs, false);
        g_prevTrianglePressed        = bootValid && g_lastPs2Frame.triangle;
        g_prevSelectPressed          = bootValid && g_lastPs2Frame.select;
        g_prevL3Pressed              = bootValid && g_lastPs2Frame.l3;
        g_prevR3Pressed              = bootValid && g_lastPs2Frame.r3;
        g_prevDpadLeftPressed        = bootValid && g_lastPs2Frame.left;
        g_prevDpadRightPressed       = bootValid && g_lastPs2Frame.right;
        g_armPushState.prevSquare    = bootValid && g_lastPs2Frame.square;
        g_posePresetState.prevCircle = bootValid && g_lastPs2Frame.circle;
    }
    g_snap.ps2Link     = g_linkSm.state();
    g_snap.ps2Mode     = g_lastPs2Frame.modeId;
    g_snap.ps2Sync     = g_lastPs2Frame.syncByte;
    g_snap.comScalePct = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);
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
        Serial.print(F(") | Sq=("));
        Serial.print(Config::kRc3SquareDeg);
        Serial.print(',');
        Serial.print(Config::kRc4SquareDeg);
        Serial.print(F(") | inv3/4=("));
        Serial.print(Config::kRc3InvertDir ? 1 : 0);
        Serial.print(',');
        Serial.print(Config::kRc4InvertDir ? 1 : 0);
        Serial.println(')');
    }
    if (g_snap.wdtResetBoot) {
        g_serial.printEvent("WARN", "Previous reset caused by Watchdog");
    }
    if (eepromLoaded) {
        g_serial.printEvent("EEPROM", "Loaded saved config (rev/trim/rc2Smooth); Precision=100%");
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
