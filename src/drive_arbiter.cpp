// Drive source arbitration, stall cut and E-stop transitions.

#include "app_state.h"
#include <WDT.h>

namespace App {

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

/** Sends the current drive output to the motors. */
void applyDriveToMotors(uint32_t nowMs) {
    maybeCutStalledDrive(nowMs);
    g_io.applyMotors(g_snap.drive.left, g_snap.drive.right, Config::kEnableSmartHoldBrake, nowMs);
}

int16_t computeMaxDriveSpeed(const MotorPowerProfile& profile) {
    return RobotLogic::computeMaxDriveSpeed(g_snap.precisionMode, profile.maxPowerCap);
}

/** Drops any latched Serial drive request and stops the Serial motion timer. */
void cancelSerialDrive() {
    g_serialDrive = SerialDriveState{};
    g_serial.stopMotion();
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
        applyDriveToMotors(nowMs);
        return false;
    }

    const Ps2Frame& f = g_lastPs2Frame;
    const bool allowArm =
        static_cast<uint32_t>(nowMs - g_lastArmStepMs) >= Config::kArmStepIntervalMs;
    const int16_t prevRc2 = g_snap.servos.rc2Deg;
    RobotLogic::updateServoTargetsFromPs2(
        f, g_snap.servos, allowArm, &g_armPushState, &g_posePresetState, nowMs);
    if (allowArm && g_snap.servos.rc2Deg != prevRc2) {
        g_lastArmStepMs = nowMs;
    }
    g_snap.comScalePct = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);

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
                                                                false, // dpadLeft micro-turn: unbound in firmware
                                                                false, // dpadRight micro-turn: unbound in firmware
                                                                g_snap.motorTrimPct,
                                                                g_snap.precisionMode);
    applyDriveToMotors(nowMs);
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
    const RobotLogic::DriveSource source = RobotLogic::selectDriveSource(
        g_snap.sysState == SystemState::Estop,
        g_linkSm.isFailsafe(),
        g_linkSm.isOperational(),
        valid,
        Controls::held(g_lastPs2Frame, Controls::kBrakeOrExit),
        g_serialDrive.latched);
    switch (source) {
        case RobotLogic::DriveSource::Estop:
            driveEstop(nowMs);
            break;
        case RobotLogic::DriveSource::Failsafe:
            driveFailsafe(nowMs, prevLink != LinkState::Failsafe);
            break;
        case RobotLogic::DriveSource::CrossBrake:
            driveCrossBrake(nowMs);
            break;
        case RobotLogic::DriveSource::Serial:
            driveFromSerial(nowMs, pwr);
            break;
        case RobotLogic::DriveSource::Ps2:
            ps2ServosUpdated = driveFromPs2(nowMs, prevLink, valid, pwr);
            break;
        case RobotLogic::DriveSource::Coast:
        default:
            driveCoastToStop(nowMs, pwr);
            break;
    }
    g_snap.holdBrakeActive = g_io.isBrakeEngaged();
    return ps2ServosUpdated;
}

} // namespace App
