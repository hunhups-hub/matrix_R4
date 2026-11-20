#include <Arduino.h>
#include <WDT.h>
#include <boot.h>

#include "auto_mode.h"
#include "config.h"
#include "controls.h"
#include "minir4_io.h"
#include "oled_ui.h"
#include "robot_logic.h"
#include "serial_control.h"
#include "types.h"

namespace {

// ============================================================================
// Global state
// ============================================================================

/** Previous-tick state of PS2 buttons used for rising-edge (press) detection. */
struct Ps2MenuEdges {
    bool autoStart  = false; // Controls::kAutoStart
    bool precision  = false; // Controls::kPrecisionToggle
    bool smooth     = false; // Controls::kSmoothToggle
    bool reverse    = false; // Controls::kReverseToggle
    bool trimLeft   = false; // Controls::kTrimLeft
    bool trimRight  = false; // Controls::kTrimRight
};

/** Latched drive request from the Serial command interface. */
struct SerialDriveState {
    bool    latched = false;
    int16_t targetL = 0;
    int16_t targetR = 0;
};

MiniR4IO                         g_io;
SerialControl                    g_serial;
OledUI                           g_oled;
RobotLogic::FailsafeStateMachine g_linkSm;
RobotLogic::BatteryStabilizer    g_battStab;
RobotLogic::StabilityMonitor     g_stabMon;
RobotLogic::ManualStallDetector  g_stallDet;
RobotLogic::TeleopHeadingHold    g_headingHold;
uint32_t                         g_lastStallPollMs = 0;
uint32_t                         g_lastHoldPollMs  = 0;

SystemSnapshot   g_snap{};
Ps2Frame         g_lastPs2Frame{};
SerialDriveState g_serialDrive{};
bool             g_serialAutoRequested = false;

uint32_t g_lastControlMs   = 0;
uint32_t g_lastArmStepMs   = 0;
uint32_t g_lastTelemetryMs = 0;

Ps2MenuEdges       g_prevPs2{};
FrontArmPushState  g_armPushState{};
ArmPosePresetState g_posePresetState{};

// ============================================================================
// Small helpers
// ============================================================================

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

bool isDriving() {
    return g_snap.drive.left != 0 || g_snap.drive.right != 0;
}

/**
 * Opt-in (Config::kEnableManualStallCut): cuts both wheels when they are commanded hard but the
 * encoders show no motion. Fails open on any encoder read failure. Latched until the stick is
 * released near neutral (see RobotLogic::ManualStallDetector).
 */
void maybeCutStalledDrive(uint32_t nowMs) {
    if (!Config::kEnableManualStallCut) {
        return;
    }
    const int32_t absL = (g_snap.drive.left < 0) ? -static_cast<int32_t>(g_snap.drive.left)
                                                 : g_snap.drive.left;
    const int32_t absR = (g_snap.drive.right < 0) ? -static_cast<int32_t>(g_snap.drive.right)
                                                  : g_snap.drive.right;
    const bool demanding = ((absL > absR) ? absL : absR) >= Config::kManualStallMinCmdPct;

    bool    fresh = false;
    bool    valid = false;
    int32_t encL  = 0;
    int32_t encR  = 0;
    if (demanding &&
        static_cast<uint32_t>(nowMs - g_lastStallPollMs) >= Config::kManualStallPollMs) {
        g_lastStallPollMs = nowMs;
        fresh             = true;
        valid             = g_io.readDriveEncoders(nowMs, encL, encR);
    }

    const bool wasTripped = g_stallDet.tripped();
    if (g_stallDet.update(g_snap.drive.left, g_snap.drive.right, fresh, valid, encL, encR, nowMs)) {
        g_snap.drive = {0, 0};
        if (!wasTripped) {
            g_oled.beepWarning();
            g_serial.printEvent("STALL", "Wheels stalled -> motors cut (release stick to re-arm)");
        }
    }
}

/** Sends the current drive output; turnCorrPct (heading hold) is added on top without touching g_snap.drive. */
void applyDriveToMotors(uint32_t nowMs, int16_t turnCorrPct = 0) {
    maybeCutStalledDrive(nowMs);
    int16_t left  = g_snap.drive.left;
    int16_t right = g_snap.drive.right;
    RobotLogic::applyTurnCorrection(
        left, right, static_cast<int16_t>(turnCorrPct * Config::kAutoPivotYawSign));
    g_io.applyMotors(left, right, Config::kEnableSmartHoldBrake, nowMs);
}

/**
 * Opt-in (Config::kEnableTeleopHeadingHold): when the driver pushes throttle with no turn input,
 * returns a gyro correction (percent turn) that keeps the latched heading. 0 when not holding.
 */
int16_t updateHeadingHold(uint32_t nowMs, const Ps2Frame& f, bool microLeft, bool microRight) {
    const bool throttle = RobotLogic::computeAxisWithDeadzone(f.ly) != 0;
    const bool turning  = RobotLogic::computeAxisWithDeadzone(f.rx) != 0 || microLeft || microRight;
    const bool straight = throttle && !turning;

    bool    fresh = false;
    bool    valid = false;
    int16_t yaw   = 0;
    if (straight && static_cast<uint32_t>(nowMs - g_lastHoldPollMs) >= Config::kTeleopHoldPollMs) {
        g_lastHoldPollMs = nowMs;
        fresh            = true;
        valid            = g_io.readYawDeg(nowMs, yaw);
    }
    return g_headingHold.update(straight, fresh, valid, yaw, nowMs);
}

/** Effective max drive power = min(mode cap [precision/normal], battery power cap). */
int16_t computeMaxDriveSpeed(const MotorPowerProfile& profile) {
    const int16_t modeCap =
        g_snap.precisionMode ? Config::kPrecisionMaxPower : Config::kMotorMaxPower;
    return (modeCap < profile.maxPowerCap) ? modeCap : profile.maxPowerCap;
}

/** Drops any latched Serial drive request and stops the Serial motion timer. */
void cancelSerialDrive() {
    g_serialDrive = SerialDriveState{};
    g_serial.stopMotion();
}

// --- PS2 edge latches ---------------------------------------------------------

void latchMenuEdges(bool valid) {
    using Controls::held;
    g_prevPs2.autoStart = valid && held(g_lastPs2Frame, Controls::kAutoStart);
    g_prevPs2.precision = valid && held(g_lastPs2Frame, Controls::kPrecisionToggle);
    g_prevPs2.smooth    = valid && held(g_lastPs2Frame, Controls::kSmoothToggle);
    g_prevPs2.reverse   = valid && held(g_lastPs2Frame, Controls::kReverseToggle);
    g_prevPs2.trimLeft  = valid && held(g_lastPs2Frame, Controls::kTrimLeft);
    g_prevPs2.trimRight = valid && held(g_lastPs2Frame, Controls::kTrimRight);
}

void latchServoEdges(bool valid) {
    g_armPushState.prevSquare    = valid && Controls::held(g_lastPs2Frame, Controls::kPushGround);
    g_posePresetState.prevCircle = valid && Controls::held(g_lastPs2Frame, Controls::kPosePreset);
}

/** Seeds every edge latch from the current frame so held buttons don't fire on (re)entry. */
void latchAllPs2Edges(bool valid) {
    latchMenuEdges(valid);
    latchServoEdges(valid);
}

void syncPs2Snapshot() {
    g_snap.ps2Link      = g_linkSm.state();
    g_snap.ps2Glitching = g_linkSm.isLinkGlitching();
    g_snap.ps2Mode      = g_lastPs2Frame.modeId;
    g_snap.ps2Sync      = g_lastPs2Frame.syncByte;
    g_snap.ps2Ly        = g_lastPs2Frame.ly;
    g_snap.ps2Rx        = g_lastPs2Frame.rx;
}

void initOled() {
    g_snap.oledOk    = g_oled.begin();
    g_snap.oledAddr  = g_oled.i2cAddr();
    g_snap.oledErr3d = g_oled.err3d();
    g_snap.oledErr3c = g_oled.err3c();
}

// --- Shared configuration actions (Serial + PS2) -------------------------------

void togglePrecisionMode(const char* onMsg, const char* offMsg) {
    g_snap.precisionMode = !g_snap.precisionMode;
    g_oled.beepConfirm();
    g_serial.printEvent("CFG", g_snap.precisionMode ? onMsg : offMsg);
}

void toggleReverseDrive(const char* onMsg, const char* offMsg) {
    g_snap.reverseDrive = !g_snap.reverseDrive;
    g_oled.beepConfirm();
    g_serial.printEvent("CFG", g_snap.reverseDrive ? onMsg : offMsg);
}

void toggleRc2Smooth(const char* onMsg, const char* offMsg) {
    g_snap.rc2SmoothEnabled = !g_snap.rc2SmoothEnabled;
    g_io.setRc2SmoothEnabled(g_snap.rc2SmoothEnabled);
    savePersistentConfig();
    g_oled.beepConfirm();
    g_serial.printEvent("CFG", g_snap.rc2SmoothEnabled ? onMsg : offMsg);
}

void setMotorTrim(int8_t trimPct, const char* src) {
    g_snap.motorTrimPct = trimPct;
    savePersistentConfig();
    g_oled.beepConfirm();
    printTrimChanged(src, g_snap.motorTrimPct);
}

void toggleMotorReverse(bool& reversed, const char* onMsg, const char* offMsg) {
    reversed = !reversed;
    g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
    savePersistentConfig();
    g_oled.beepConfirm();
    g_serial.printEvent("CFG", reversed ? onMsg : offMsg);
}

// --- Servo target setters ------------------------------------------------------

/** Manual RC1/RC2 moves cancel the Circle drop/hold preset toggle. */
void setPoseServo(int16_t& targetDeg, int16_t deg) {
    targetDeg                    = deg;
    g_posePresetState.dropActive = false;
}

/** Manual RC3/RC4 moves cancel the Square push-ground hold. */
void setGripServo(int16_t& targetDeg, int16_t deg) {
    targetDeg             = deg;
    g_armPushState.active = false;
}

void togglePosePreset() {
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

// ============================================================================
// System state transitions
// ============================================================================

[[noreturn]] void enterUploadBootloader(const char* source) {
    WDT.refresh();
    g_io.forceMotorsStop(true, millis(), true);
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
    cancelSerialDrive();
    if (g_snap.sysState != SystemState::Estop) {
        g_snap.sysState = SystemState::Estop;
        g_snap.drive    = {0, 0};
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

// ============================================================================
// Power & health telemetry
// ============================================================================

const char* batteryTierEventMsg(BatteryTier tier) {
    if (Config::kEnableBatterySagThrottle) {
        switch (tier) {
            case BatteryTier::Emergency: return "EMERGENCY sag cap (35%)";
            case BatteryTier::Critical:  return "CRITICAL sag cap (50%)";
            default:                     return "LOW sag cap (72%)";
        }
    }
    switch (tier) {
        case BatteryTier::Emergency: return "EMERGENCY low batt (100% full power)";
        case BatteryTier::Critical:  return "CRITICAL low batt (100% full power)";
        default:                     return "LOW batt warning (100% full power)";
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
        g_serial.printEvent("PWR", batteryTierEventMsg(nextTier));
    }
}

/**
 * Runs the (rate-limited unless forced) STM32 health check and feeds the battery
 * stabilizer. Returns true when a sample was actually taken this call.
 */
bool sampleLowerMcu(uint32_t nowMs, bool isMoving, bool force) {
    bool     voltValid = false;
    uint16_t battMv    = 0U;
    if (!g_io.verifyAndHealLowerMcu(nowMs, isMoving, voltValid, battMv, force)) {
        return false;
    }
    applyBatteryTelemetry(voltValid, battMv, nowMs);
    return true;
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

// ============================================================================
// Serial command handling
// ============================================================================

void applySerialConfigActions(const SerialActions& act) {
    if (act.togglePrecision) {
        togglePrecisionMode("Precision 55% ON (session)", "Precision OFF 100%");
    }
    if (act.toggleReverseDrive) {
        toggleReverseDrive("Reverse-Front ON", "Reverse-Front OFF");
    }
    if (act.toggleRc2Smooth) {
        toggleRc2Smooth("RC2 Smooth Slew ON (saved)", "RC2 Smooth Slew OFF (saved)");
    }

    if (act.trimLeftStep) {
        setMotorTrim(RobotLogic::stepMotorTrim(g_snap.motorTrimPct, -1), "Serial");
    } else if (act.trimRightStep) {
        setMotorTrim(RobotLogic::stepMotorTrim(g_snap.motorTrimPct, 1), "Serial");
    } else if (act.trimReset) {
        setMotorTrim(0, "Serial");
    }

    if (act.toggleLeftReverse) {
        toggleMotorReverse(g_snap.leftReversed, "M1 reverse=ON (saved)", "M1 reverse=OFF (saved)");
    }
    if (act.toggleRightReverse) {
        toggleMotorReverse(g_snap.rightReversed, "M2 reverse=ON (saved)", "M2 reverse=OFF (saved)");
    }

    if (act.speedPctChanged) {
        g_snap.serialSpeedPct = act.newSpeedPct;
        Serial.print(F("[SER] Speed = "));
        Serial.print(g_snap.serialSpeedPct);
        Serial.println('%');
    }
}

void applySerialServoActions(const SerialActions& act) {
    ServoTargets& servos = g_snap.servos;

    if (act.armUpStep) {
        setPoseServo(servos.rc2Deg, RobotLogic::computeArmStep(servos.rc2Deg, true, false));
    } else if (act.armDownStep) {
        setPoseServo(servos.rc2Deg, RobotLogic::computeArmStep(servos.rc2Deg, false, true));
    }

    if (act.gripOpen) {
        setGripServo(servos.rc3Deg, Config::kRc3L1Deg);
        setGripServo(servos.rc4Deg, Config::kRc4L1Deg);
    } else if (act.gripClose) {
        setGripServo(servos.rc3Deg, Config::kRc3R1Deg);
        setGripServo(servos.rc4Deg, Config::kRc4R1Deg);
    }

    if (act.auxExtend) {
        setPoseServo(servos.rc1Deg, Config::kRc1L2Deg);
    } else if (act.auxRetract) {
        setPoseServo(servos.rc1Deg, Config::kRc1R2Deg);
    } else if (act.togglePosePreset) {
        togglePosePreset();
    }

    if (act.hasDirectServo) {
        switch (act.directServoCh) {
            case 1: setPoseServo(servos.rc1Deg, act.directServoDeg); break;
            case 2: setPoseServo(servos.rc2Deg, act.directServoDeg); break;
            case 3: setGripServo(servos.rc3Deg, act.directServoDeg); break;
            case 4: setGripServo(servos.rc4Deg, act.directServoDeg); break;
            default: break;
        }
    }

    g_snap.servos      = RobotLogic::clampServoTargets(g_snap.servos);
    g_snap.comScalePct = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);
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

    applySerialConfigActions(act);

    if (g_snap.sysState == SystemState::Estop) {
        cancelSerialDrive();
    } else {
        applySerialServoActions(act);
        g_serialDrive.latched = act.driveCmd.active;
        g_serialDrive.targetL = act.driveCmd.targetLeft;
        g_serialDrive.targetR = act.driveCmd.targetRight;
    }
}

// ============================================================================
// Autonomous session
// ============================================================================

void enterAutoSession(uint32_t nowMs) {
    g_serialAutoRequested = false;
    cancelSerialDrive();
    g_io.forceMotorsStop(true, nowMs);

    g_snap.activeSource  = ControlSource::Auto;
    g_snap.drive         = {0, 0};
    g_snap.autoHolding   = false;
    g_snap.autoHoldPct   = 0.0f;
    g_snap.autoElapsedMs = 0UL;

    const bool frameValid  = g_linkSm.isConnected() && RobotLogic::isPs2FrameValid(g_lastPs2Frame);
    const uint8_t compPct  = g_battStab.motorProfile().compPercent;
    const uint8_t syncPass = g_linkSm.isConnected() ? 1U : 3U;
    for (uint8_t i = 0U; i < syncPass; ++i) {
        updateStabilityMetrics(frameValid, compPct, Config::kControlPeriodMs);
    }
    g_oled.render(g_snap, nowMs, true);
    g_oled.beepConfirm();
    g_serial.printEvent("AUTO", "Started -> Press X / BTN_UP / 'x' to exit");
}

/** Re-reads the PS2 pad after Auto and seeds edge latches so held buttons don't re-fire. */
void resyncPs2AfterAuto(uint32_t exitMs) {
    bool validAfter = false;
    if (g_linkSm.isConnected()) {
        g_io.pollPs2(g_lastPs2Frame);
        validAfter = RobotLogic::isPs2FrameValid(g_lastPs2Frame);
        if (!g_linkSm.isOperational() && validAfter) {
            g_linkSm.onFrame(true, exitMs, false);
        }
        syncPs2Snapshot();
    } else {
        g_lastPs2Frame = Ps2Frame{};
    }
    latchAllPs2Edges(validAfter);
}

void reportAutoExit(AutoExitReason reason, uint32_t exitMs) {
    if (reason == AutoExitReason::BtnEstop || reason == AutoExitReason::SerialEstop) {
        triggerEstop(AutoMode::exitReasonStr(reason), exitMs);
        g_snap.activeSource = ControlSource::None;
        return;
    }
    g_snap.activeSource = g_linkSm.isOperational() ? ControlSource::Ps2 : ControlSource::None;
    if (reason == AutoExitReason::StallTimeout || reason == AutoExitReason::DriveInitFail) {
        g_oled.beepWarning();
    } else {
        g_oled.beepConfirm();
    }
    g_serial.printEvent("AUTO", AutoMode::exitReasonStr(reason));
}

void runAutoSession(uint32_t nowMs) {
    enterAutoSession(nowMs);

    const ServoTargets   preAuto = g_snap.servos;
    const AutoExitReason reason =
        AutoMode::execute(g_io, g_linkSm, g_snap, g_lastPs2Frame, &g_oled);
    const uint32_t exitMs = millis();

    // Auto moved servos directly: drop manual pose/push states that no longer match.
    if (g_snap.servos.rc1Deg != preAuto.rc1Deg || g_snap.servos.rc2Deg != preAuto.rc2Deg) {
        g_posePresetState.dropActive = false;
    }
    if (g_snap.servos.rc3Deg != preAuto.rc3Deg || g_snap.servos.rc4Deg != preAuto.rc4Deg) {
        g_armPushState.active = false;
    }

    g_io.restoreAfterAuto(exitMs);
    g_stallDet.reset();
    g_headingHold.reset();
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

    resyncPs2AfterAuto(exitMs);
    initOled(); // Auto code may have re-initialized Wire1 / the display.
    reportAutoExit(reason, exitMs);

    if (!sampleLowerMcu(exitMs, false, g_io.commHealthy())) {
        g_snap.commHealthy = g_io.commHealthy();
    }
    g_snap.servoRangeErr = g_io.servoRangeErr();
    updateStabilityMetrics(g_linkSm.isConnected() && RobotLogic::isPs2FrameValid(g_lastPs2Frame),
                           g_battStab.motorProfile().compPercent,
                           Config::kControlPeriodMs);
    g_io.syncServoCache(g_snap.servos);
    g_io.applyServos(g_snap.servos, exitMs);
    g_snap.commHealthy = g_io.commHealthy();
    g_oled.render(g_snap, exitMs, true);

    const uint32_t postExitMs = millis();
    g_lastControlMs           = postExitMs;
    g_lastArmStepMs           = postExitMs;
}

// ============================================================================
// Control tick stages
// ============================================================================

ButtonEvents handleOnboardButtons(uint32_t nowMs) {
    ButtonEvents btn{};
    g_io.pollButtons(nowMs, btn);
    g_snap.comboHolding  = btn.comboHolding;
    g_snap.uploadHoldPct = btn.uploadHoldRatio;
    g_snap.autoHolding   = btn.autoHolding;
    g_snap.autoHoldPct   = btn.autoHoldRatio;

    if (btn.uploadRequested) {
        enterUploadBootloader("BTN_UP + BTN_DOWN hold 2.5s");
    }
    if (btn.downPressEdge || btn.shortDown) {
        g_serialAutoRequested = false;
        triggerEstop("BTN_DOWN press", nowMs);
    } else if (btn.shortUp) {
        clearEstop("BTN_UP press"); // no-op unless in E-stop
    }
    return btn;
}

/** Reconnects/polls the PS2 pad and advances the link state machine. Returns frame validity. */
bool updatePs2Link(uint32_t nowMs) {
    bool valid = false;

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

    syncPs2Snapshot();
    return valid;
}

/**
 * PS2 configuration combos (rising edge only):
 *   Select            -> Precision Mode toggle
 *   R3                -> Reverse-Front orientation toggle
 *   L3                -> RC2 Smooth Trajectory toggle
 *   Start + L3        -> Reset Motor Trim
 *   Start + D-Pad L/R -> Adjust straight-line Motor Balance Trim (-/+1%)
 */
void handlePs2ConfigCombos() {
    using Controls::held;
    const Ps2Frame& f        = g_lastPs2Frame;
    const bool      modifier = held(f, Controls::kTrimModifier);

    if (held(f, Controls::kPrecisionToggle) && !g_prevPs2.precision) {
        togglePrecisionMode("PS2: Precision 55% ON (session)",
                            "PS2: Precision OFF 100%");
    }
    if (held(f, Controls::kReverseToggle) && !g_prevPs2.reverse) {
        toggleReverseDrive("PS2: Reverse-Front ON", "PS2: Reverse-Front OFF");
    }
    if (held(f, Controls::kSmoothToggle) && !g_prevPs2.smooth) {
        if (modifier) {
            setMotorTrim(0, "PS2 trim reset");
        } else {
            toggleRc2Smooth("PS2: RC2 Smooth Slew ON (saved)",
                            "PS2: RC2 Smooth Slew OFF (saved)");
        }
    }
    if (modifier) {
        if (held(f, Controls::kTrimLeft) && !g_prevPs2.trimLeft) {
            setMotorTrim(RobotLogic::stepMotorTrim(g_snap.motorTrimPct, -1), "PS2 trim -");
        } else if (held(f, Controls::kTrimRight) && !g_prevPs2.trimRight) {
            setMotorTrim(RobotLogic::stepMotorTrim(g_snap.motorTrimPct, 1), "PS2 trim +");
        }
    }
}

// --- Drive arbitration branches -------------------------------------------------

void driveEstop(uint32_t nowMs) {
    g_snap.activeSource = ControlSource::None;
    g_snap.drive        = {0, 0};
    if (g_io.shouldRefreshStoppedMotors(nowMs)) {
        g_io.forceMotorsStop(true, nowMs);
    }
}

/** Active PS2 link dropped abruptly: cut power & hold brake cleanly without UART flooding. */
void driveFailsafe(uint32_t nowMs, bool enteredFailsafe) {
    g_snap.activeSource = ControlSource::None;
    g_snap.drive        = {0, 0};
    if (enteredFailsafe) {
        g_io.invalidateActuatorCaches();
    }
    if (enteredFailsafe || g_io.shouldRefreshStoppedMotors(nowMs)) {
        g_io.forceMotorsStop(true, nowMs);
    }
}

/** PS2 Cross acts as immediate brake override even during Serial driving. */
void driveCrossBrake(uint32_t nowMs) {
    const bool wasMoving = isDriving();
    cancelSerialDrive();
    g_snap.activeSource = ControlSource::Ps2;
    g_snap.drive        = {0, 0};
    if (wasMoving || g_io.shouldRefreshStoppedMotors(nowMs)) {
        g_io.forceMotorsStop(true, nowMs);
    }
}

void driveFromSerial(uint32_t nowMs, const MotorPowerProfile& pwr) {
    g_snap.activeSource = ControlSource::SerialCmd;
    int16_t effTargetL  = g_serialDrive.targetL;
    int16_t effTargetR  = g_serialDrive.targetR;
    if (g_snap.reverseDrive) {
        effTargetL = static_cast<int16_t>(-g_serialDrive.targetR);
        effTargetR = static_cast<int16_t>(-g_serialDrive.targetL);
    }
    RobotLogic::applyMotorBalanceTrim(effTargetL, effTargetR, g_snap.motorTrimPct);
    g_snap.drive = RobotLogic::applyDriveTargetsWithSlew(effTargetL,
                                                         effTargetR,
                                                         g_snap.drive.left,
                                                         g_snap.drive.right,
                                                         computeMaxDriveSpeed(pwr),
                                                         pwr.accelSlewStep,
                                                         pwr.decelSlewStep,
                                                         pwr.compPercent);
    applyDriveToMotors(nowMs);
}

/** Returns true when servo targets (and their edge latches) were updated from the frame. */
bool driveFromPs2(uint32_t nowMs, LinkState prevLink, bool valid, const MotorPowerProfile& pwr) {
    g_snap.activeSource = ControlSource::Ps2;
    if (prevLink != LinkState::Operational) {
        g_lastArmStepMs = nowMs;
        g_io.invalidateActuatorCaches();
    }

    if (!valid) {
        // Brief wireless glitch (<100ms): preserve last safe drive & servo state without overwrite.
        g_headingHold.reset();
        applyDriveToMotors(nowMs);
        return false;
    }

    const Ps2Frame& f = g_lastPs2Frame;
    const bool allowArm =
        static_cast<uint32_t>(nowMs - g_lastArmStepMs) >= Config::kArmStepIntervalMs;
    const int16_t prevRc2 = g_snap.servos.rc2Deg;
    RobotLogic::updateServoTargetsFromPs2(
        f, g_snap.servos, allowArm, &g_armPushState, &g_posePresetState);
    if (allowArm && g_snap.servos.rc2Deg != prevRc2) {
        g_lastArmStepMs = nowMs;
    }
    g_snap.comScalePct = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);

    // D-Pad Left/Right without Start = micro-turn alignment (Start+D-Pad is trim).
    const bool trimMod    = Controls::held(f, Controls::kTrimModifier);
    const bool microLeft  = !trimMod && Controls::held(f, Controls::kMicroTurnLeft);
    const bool microRight = !trimMod && Controls::held(f, Controls::kMicroTurnRight);
    g_snap.drive          = RobotLogic::computeKinodynamicDrive(f.ly,
                                                                f.rx,
                                                                g_snap.drive.left,
                                                                g_snap.drive.right,
                                                                computeMaxDriveSpeed(pwr),
                                                                pwr.accelSlewStep,
                                                                pwr.decelSlewStep,
                                                                pwr.compPercent,
                                                                g_snap.servos.rc2Deg,
                                                                g_snap.reverseDrive,
                                                                microLeft,
                                                                microRight,
                                                                g_snap.motorTrimPct,
                                                                g_snap.precisionMode);
    const int16_t holdCorrPct =
        Config::kEnableTeleopHeadingHold ? updateHeadingHold(nowMs, f, microLeft, microRight) : 0;
    applyDriveToMotors(nowMs, holdCorrPct);
    return true;
}

/** No active source: ramp both wheels down to zero through the coupled slew limiter. */
void driveCoastToStop(uint32_t nowMs, const MotorPowerProfile& pwr) {
    g_snap.activeSource = ControlSource::None;
    g_snap.drive        = RobotLogic::apply2DofKinodynamicSlew(0,
                                                               0,
                                                               g_snap.drive.left,
                                                               g_snap.drive.right,
                                                               pwr.accelSlewStep,
                                                               pwr.decelSlewStep,
                                                               Config::kAngularAccelSlewStep,
                                                               Config::kAngularDecelSlewStep);
    applyDriveToMotors(nowMs);
}

/**
 * Selects the drive source by priority:
 *   E-stop > PS2 failsafe > PS2 Cross brake > Serial drive > PS2 manual > coast.
 * Returns true when PS2 servo targets were updated this tick.
 */
bool arbitrateDrive(uint32_t nowMs, LinkState prevLink, bool valid, const MotorPowerProfile& pwr) {
    bool ps2ServosUpdated = false;
    bool ps2Drive         = false;
    if (g_snap.sysState == SystemState::Estop) {
        driveEstop(nowMs);
    } else if (g_linkSm.isFailsafe()) {
        driveFailsafe(nowMs, prevLink != LinkState::Failsafe);
    } else if (g_linkSm.isOperational() && valid && Controls::held(g_lastPs2Frame, Controls::kBrakeOrExit)) {
        driveCrossBrake(nowMs);
    } else if (g_serialDrive.latched) {
        driveFromSerial(nowMs, pwr);
    } else if (g_linkSm.isOperational()) {
        ps2Drive         = true;
        ps2ServosUpdated = driveFromPs2(nowMs, prevLink, valid, pwr);
    } else {
        driveCoastToStop(nowMs, pwr);
    }
    if (!ps2Drive) {
        g_headingHold.reset(); // Any non-PS2 source ends the hold; it re-latches on the next straight run.
    }
    g_snap.holdBrakeActive = g_io.isBrakeEngaged();
    return ps2ServosUpdated;
}

void publishOutputs(uint32_t nowMs) {
    g_io.applyServos(g_snap.servos, nowMs);
    g_snap.commHealthy = g_io.commHealthy();
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

void runControlTick(uint32_t nowMs, uint32_t loopDtMs) {
    const ButtonEvents btn = handleOnboardButtons(nowMs);

    const LinkState prevLink = g_linkSm.state();
    const bool      valid    = updatePs2Link(nowMs);
    logLinkTransition(prevLink, g_snap.ps2Link);

    if (sampleLowerMcu(nowMs, isDriving(), false)) {
        g_snap.servoRangeErr = g_io.servoRangeErr();
    } else {
        g_snap.commHealthy = g_io.commHealthy();
    }

    const MotorPowerProfile pwrProfile = g_battStab.motorProfile();
    updateStabilityMetrics(valid, pwrProfile.compPercent, loopDtMs);

    const bool sysReady    = (g_snap.sysState == SystemState::Ready);
    const bool ps2Usable   = g_linkSm.isOperational() && valid;
    const bool ps2LinkLost = !g_linkSm.isConnected() || g_linkSm.isFailsafe();

    if (sysReady && ps2Usable) {
        handlePs2ConfigCombos();
    }

    const bool ps2StartAuto = RobotLogic::shouldStartAutoMode(sysReady,
                                                              g_linkSm.isOperational(),
                                                              valid,
                                                              Controls::held(g_lastPs2Frame, Controls::kAutoStart),
                                                              g_prevPs2.autoStart,
                                                              Controls::held(g_lastPs2Frame, Controls::kBrakeOrExit));
    const bool startAuto = ps2StartAuto ||
                           (sysReady && (btn.autoStartRequested || g_serialAutoRequested));
    g_serialAutoRequested = false;

    if (ps2LinkLost) {
        latchMenuEdges(false);
    } else if (valid) {
        latchMenuEdges(true);
    }

    if (startAuto) {
        runAutoSession(nowMs);
        return;
    }

    const bool ps2ServosUpdated = arbitrateDrive(nowMs, prevLink, valid, pwrProfile);

    // driveFromPs2() already latched Square/Circle when it consumed the frame.
    if (ps2LinkLost) {
        latchServoEdges(false);
    } else if (!ps2ServosUpdated && valid) {
        latchServoEdges(true);
    }

    publishOutputs(nowMs);
}

// ============================================================================
// Boot
// ============================================================================

/** Loads EEPROM config into the snapshot and applies it to the IO layer. */
bool loadPersistentConfig() {
    PersistentConfig savedCfg{};
    const bool loaded = g_io.loadConfigFromEeprom(savedCfg);
    if (loaded) {
        g_snap.leftReversed     = savedCfg.leftReversed;
        g_snap.rightReversed    = savedCfg.rightReversed;
        g_snap.precisionMode    = false; // Session-only
        g_snap.rc2SmoothEnabled = savedCfg.rc2SmoothEnabled;
        g_snap.motorTrimPct     = savedCfg.motorTrimPct;
        g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
    }
    g_io.setRc2SmoothEnabled(g_snap.rc2SmoothEnabled);
    return loaded;
}

/** Initializes the PS2 link state machine and seeds edge latches from the first frames. */
bool initPs2Link(uint32_t nowMs) {
    const bool ps2Ready = g_io.isPs2ReadyAtBoot();
    g_linkSm.begin(ps2Ready, nowMs);
    if (ps2Ready) {
        bool bootValid = false;
        for (uint8_t i = 0U; i < Config::kPs2StableFrames; ++i) {
            g_io.pollPs2(g_lastPs2Frame);
            bootValid            = RobotLogic::isPs2FrameValid(g_lastPs2Frame);
            const bool modeReset = !bootValid && RobotLogic::isPs2UnconfiguredMode(g_lastPs2Frame);
            g_linkSm.onFrame(bootValid, nowMs, modeReset);
            if (!bootValid) {
                break;
            }
            if (i + 1U < Config::kPs2StableFrames) {
                delay(4);
            }
        }
        latchAllPs2Edges(bootValid);
    }
    if (g_linkSm.isOperational()) {
        g_snap.activeSource = ControlSource::Ps2;
    }
    syncPs2Snapshot();
    return ps2Ready;
}

void printOledBootLine(uint8_t sdaBefore, uint8_t sdaAfter, uint8_t recoverClk) {
    const bool oledOk = g_snap.oledOk;
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
}

void printServoBootLine() {
    if (g_io.servoRangeErr() != 0U) {
        Serial.print(F("[WARN] Servo range err="));
        Serial.println(g_io.servoRangeErr());
        return;
    }
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

void printBootReport(bool miniOk,
                     bool eepromLoaded,
                     uint8_t sdaBefore,
                     uint8_t sdaAfter,
                     uint8_t recoverClk) {
    g_serial.printBanner();
    printOledBootLine(sdaBefore, sdaAfter, recoverClk);
    if (!miniOk) {
        g_serial.printEvent("WARN", "MiniR4 init returned non-OK");
    }
    printServoBootLine();
    if (g_snap.wdtResetBoot) {
        g_serial.printEvent("WARN", "Previous reset caused by Watchdog");
    }
    if (eepromLoaded) {
        g_serial.printEvent("EEPROM", "Loaded saved config (rev/trim/rc2Smooth); Precision=100%");
    }
    g_serial.printSnapshot(g_snap);
}

} // namespace

void setup() {
    g_snap.wdtResetBoot = (R_SYSTEM->RSTSR1_b.WDTRF != 0);
    if (g_snap.wdtResetBoot) {
        // Clear sticky RA4M1 WDT reset flag after reading so subsequent clean resets do not re-warn.
        R_SYSTEM->RSTSR1_b.WDTRF = 0U;
    }

    g_serial.begin(Config::kSerialBaud);

    // Clear any mid-byte stuck SSD1306 slave on Wire1 before MiniR4.begin() starts OLED.
    uint8_t sdaBefore  = 1U;
    uint8_t sdaAfter   = 1U;
    uint8_t recoverClk = 0U;
    OledUI::recoverBus(sdaBefore, sdaAfter, recoverClk);

    const bool miniOk       = g_io.begin();
    const bool eepromLoaded = loadPersistentConfig();
    g_snap.holdBrakeActive  = g_io.isBrakeEngaged();
    g_snap.servoRangeErr    = g_io.servoRangeErr();
    initOled();

    const uint32_t nowMs = millis();

    // Seed initial power/health sample cleanly (skip redundant forced timeout if STM32 already failed init).
    if (!sampleLowerMcu(nowMs, false, miniOk)) {
        g_snap.commHealthy = g_io.commHealthy();
    }

    const bool ps2Ready = initPs2Link(nowMs);
    g_snap.comScalePct  = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);
    updateStabilityMetrics(ps2Ready && RobotLogic::isPs2FrameValid(g_lastPs2Frame),
                           g_battStab.motorProfile().compPercent,
                           Config::kControlPeriodMs);

    g_oled.render(g_snap, nowMs, true);
    g_oled.beepBoot();

    printBootReport(miniOk, eepromLoaded, sdaBefore, sdaAfter, recoverClk);

    // Enable WDT only after MiniR4 STM32 + staggered servo + PS2 boot init completes.
    if (!WDT.begin(Config::kWdtTimeoutMs)) {
        g_serial.printEvent("WARN", "WDT init failed");
    }

    const uint32_t bootDoneMs = millis();
    g_lastControlMs           = bootDoneMs;
    g_lastArmStepMs           = bootDoneMs;
    g_lastTelemetryMs         = bootDoneMs;
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
