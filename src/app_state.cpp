#include "app_state.h"

namespace App {

MiniR4IO                         g_io;
SerialControl                    g_serial;
OledUI                           g_oled;
RobotLogic::FailsafeStateMachine g_linkSm;
RobotLogic::BatteryStabilizer    g_battStab;
RobotLogic::StabilityMonitor     g_stabMon;
RobotLogic::ManualStallDetector  g_stallDet;
uint32_t                         g_lastStallPollMs = 0;

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

} // namespace App
