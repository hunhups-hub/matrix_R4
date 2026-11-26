#pragma once

// Shared application state and cross-module API for the control firmware (see main.cpp for loop()).

#include <Arduino.h>

#include "auto_mode.h"
#include "config.h"
#include "controls.h"
#include "minir4_io.h"
#include "oled_ui.h"
#include "robot_logic.h"
#include "serial_control.h"
#include "types.h"

namespace App {

/** Previous-tick state of PS2 buttons used for rising-edge (press) detection. */
struct Ps2MenuEdges {
    bool autoStart  = false; // Controls::kAutoStart
    bool precision  = false; // Controls::kPrecisionToggle
    bool smooth     = false; // Controls::kSmoothToggle
    bool reverse    = false; // Controls::kReverseToggle
};

/** Latched drive request from the Serial command interface. */
struct SerialDriveState {
    bool    latched = false;
    int16_t targetL = 0;
    int16_t targetR = 0;
};


extern MiniR4IO                         g_io;
extern SerialControl                    g_serial;
extern OledUI                           g_oled;
extern RobotLogic::FailsafeStateMachine g_linkSm;
extern RobotLogic::BatteryStabilizer    g_battStab;
extern RobotLogic::StabilityMonitor     g_stabMon;
extern RobotLogic::ManualStallDetector  g_stallDet;
extern uint32_t                         g_lastStallPollMs;

extern SystemSnapshot   g_snap;
extern Ps2Frame         g_lastPs2Frame;
extern SerialDriveState g_serialDrive;
extern bool             g_serialAutoRequested;

extern uint32_t g_lastControlMs;
extern uint32_t g_lastArmStepMs;
extern uint32_t g_lastTelemetryMs;

extern Ps2MenuEdges       g_prevPs2;
extern FrontArmPushState  g_armPushState;
extern ArmPosePresetState g_posePresetState;

// --- Functions, grouped by module -------------------------------------------

// main.cpp
void latchMenuEdges(bool valid);
void latchServoEdges(bool valid);
void latchAllPs2Edges(bool valid);
void syncPs2Snapshot();
void initOled();
ButtonEvents handleOnboardButtons(uint32_t nowMs);
bool updatePs2Link(uint32_t nowMs);
void handlePs2ConfigCombos();
void publishOutputs(uint32_t nowMs);
void runControlTick(uint32_t nowMs, uint32_t loopDtMs);

// drive_arbiter.cpp
bool isDriving();
void maybeCutStalledDrive(uint32_t nowMs);
void applyDriveToMotors(uint32_t nowMs);
int16_t computeMaxDriveSpeed(const MotorPowerProfile& profile);
void cancelSerialDrive();
void triggerEstop(const char* reason, uint32_t nowMs);
void clearEstop(const char* reason);
void driveEstop(uint32_t nowMs);
void driveFailsafe(uint32_t nowMs, bool enteredFailsafe);
void driveCrossBrake(uint32_t nowMs);
void driveFromSerial(uint32_t nowMs, const MotorPowerProfile& pwr);
bool driveFromPs2(uint32_t nowMs, LinkState prevLink, bool valid, const MotorPowerProfile& pwr);
void driveCoastToStop(uint32_t nowMs, const MotorPowerProfile& pwr);
bool arbitrateDrive(uint32_t nowMs, LinkState prevLink, bool valid, const MotorPowerProfile& pwr);

// pose_control.cpp
void setPoseServo(int16_t& targetDeg, int16_t deg);
void setGripServo(int16_t& targetDeg, int16_t deg);
void togglePosePreset();

// health_telemetry.cpp
void logLinkTransition(LinkState prev, LinkState next);
const char* batteryTierEventMsg(BatteryTier tier);
void applyBatteryTelemetry(bool voltValid, uint16_t battMv, uint32_t nowMs);
bool sampleLowerMcu(uint32_t nowMs, bool isMoving, bool force);
void updateStabilityMetrics(bool frameValid, uint8_t compPercent, uint32_t loopDtMs);

// serial_dispatch.cpp
void savePersistentConfig();
void printTrimChanged(const char* src, int8_t trimPct);
void togglePrecisionMode(const char* prefix);
void toggleReverseDrive(const char* onMsg, const char* offMsg);
void toggleRc2Smooth(const char* onMsg, const char* offMsg);
void setMotorTrim(int8_t trimPct, const char* src);
void toggleMotorReverse(bool& reversed, const char* onMsg, const char* offMsg);
void applySerialConfigActions(const SerialActions& act);
void applySerialServoActions(const SerialActions& act);
void applySerialImmediateActions(const SerialActions& act, uint32_t nowMs);

// auto_session.cpp
void enterAutoSession(uint32_t nowMs);
void resyncPs2AfterAuto(uint32_t exitMs);
void reportAutoExit(AutoExitReason reason, uint32_t exitMs);
void runAutoSession(uint32_t nowMs);

// boot_report.cpp
[[noreturn]] void enterUploadBootloader(const char* source);
bool loadPersistentConfig();
bool initPs2Link(uint32_t nowMs);
void printOledBootLine(uint8_t sdaBefore, uint8_t sdaAfter, uint8_t recoverClk);
void printServoBootLine();
void printBootReport(bool miniOk, bool eepromLoaded, uint8_t sdaBefore, uint8_t sdaAfter, uint8_t recoverClk);

} // namespace App
