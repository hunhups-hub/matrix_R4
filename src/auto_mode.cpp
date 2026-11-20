#include "auto_mode.h"

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "Modules/MMLower.h"
#include <WDT.h>
#include <setjmp.h>

#include "config.h"
#include "controls.h"
#include "oled_ui.h"

namespace {

MiniR4IO*                         s_io         = nullptr;
RobotLogic::FailsafeStateMachine* s_linkSm     = nullptr;
SystemSnapshot*                   s_snap       = nullptr;
Ps2Frame*                         s_lastFrame  = nullptr;
OledUI*                           s_oled       = nullptr;

jmp_buf                 s_abortEnv;
volatile bool           s_running              = false;
volatile bool           s_inService            = false;
volatile bool           s_yieldCalled          = false;
volatile bool           s_pollPs2Called        = false;
volatile bool           s_inYieldContext       = false;
bool                    s_entryTrianglePending   = false;
bool                    s_exitAfterLoopIteration = false;
bool                    s_sawUpReleased          = false;
volatile AutoExitReason s_exitReason             = AutoExitReason::None;
uint32_t                s_lastPollMs             = 0UL;
uint32_t                s_autoStartMs            = 0UL;
uint32_t                s_lastOledAutoMs         = 0UL;
uint32_t                s_cmdStartMs             = 0UL;
uint32_t                s_lastStallCheckMs       = 0UL;
uint32_t                s_lastWheelMoveMs        = 0UL;
int32_t                 s_prevEncL               = 0;
int32_t                 s_prevEncR               = 0;
int32_t                 s_prevDriveDeg           = 0;
bool                    s_hasEncBaseline         = false;
bool                    s_hasDegBaseline         = false;
uint8_t                 s_driveEncIdxL           = Config::kMotorLeftId - 1U;
uint8_t                 s_driveEncIdxR           = Config::kMotorRightId - 1U;
bool                    s_driveTaskActive        = false;
bool                    s_expectWheelMotion      = false;
bool                    s_gyroActive             = false;
int16_t                 s_headingTargetDeg       = 0;
uint16_t                s_moveStep               = 0U;

constexpr uint8_t       kSerialPassCap           = 32U;
uint8_t                 s_serialBuf[kSerialPassCap]{};
uint8_t                 s_serialHead             = 0U;
uint8_t                 s_serialTail             = 0U;
uint8_t                 s_serialCount            = 0U;

void clearSerialPassthrough() {
    s_serialHead  = 0U;
    s_serialTail  = 0U;
    s_serialCount = 0U;
}

void pushSerialPassthrough(uint8_t b) {
    if (s_serialCount >= kSerialPassCap) {
        s_serialHead = static_cast<uint8_t>((s_serialHead + 1U) % kSerialPassCap);
        s_serialCount--;
    }
    s_serialBuf[s_serialTail] = b;
    s_serialTail              = static_cast<uint8_t>((s_serialTail + 1U) % kSerialPassCap);
    s_serialCount++;
}

__attribute__((noinline)) void emptyLoopStub() {
    asm volatile("" ::: "memory");
}

// Detects whether user's AutoBlockUser::loop() is empty `{}` or only contains MATRIXblock's
// boilerplate `MiniR4.PS2.read_gamepad(false, 0);` so finished linear Auto sequences return
// straight to Manual control, while active loop() routines keep repeating until X or autoExit().
bool probeAndRunLoopIteration() {
    constexpr uint8_t  kBurstCalls   = 16U;
    constexpr uint32_t kFirstMaxUs   = 25UL;
    constexpr uint32_t kEmptySlackUs = 2UL;

    const uint32_t b0 = micros();
    for (uint8_t i = 0U; i < kBurstCalls; ++i) {
        emptyLoopStub();
    }
    const uint32_t baseUs = static_cast<uint32_t>(micros() - b0);

    s_yieldCalled         = false;
    s_pollPs2Called       = false;
    const uint32_t t0     = micros();
    AutoBlockUser::loop();
    const uint32_t firstUs = static_cast<uint32_t>(micros() - t0);

    if (!s_running || s_exitAfterLoopIteration) {
        return false;
    }
    if (s_pollPs2Called && !s_yieldCalled) {
        return false;
    }
    if (s_yieldCalled || firstUs >= kFirstMaxUs) {
        return true;
    }

    for (uint8_t round = 0U; round < 2U && s_running; ++round) {
        s_yieldCalled     = false;
        s_pollPs2Called   = false;
        const uint32_t r0 = micros();
        for (uint8_t i = 0U; i < kBurstCalls; ++i) {
            AutoBlockUser::loop();
        }
        const uint32_t rUs = static_cast<uint32_t>(micros() - r0);
        if (s_yieldCalled) {
            return true;
        }
        if ((s_pollPs2Called && !s_yieldCalled) || rUs <= (baseUs + kEmptySlackUs)) {
            return false;
        }
    }
    return true;
}

[[noreturn]] void abortAuto(AutoExitReason reason) {
    s_exitReason        = reason;
    s_running           = false;
    s_inService         = false;
    s_inYieldContext    = false;
    s_driveTaskActive   = false;
    s_expectWheelMotion = false;
    longjmp(s_abortEnv, 1);
}

void resetStallTracker(uint32_t nowMs) {
    s_cmdStartMs       = nowMs;
    s_lastStallCheckMs = nowMs;
    s_lastWheelMoveMs  = nowMs;
    s_prevEncL         = 0;
    s_prevEncR         = 0;
    s_prevDriveDeg     = 0;
    s_hasEncBaseline   = false;
    s_hasDegBaseline   = false;
}

void checkDriveStall(uint32_t nowMs) {
    if (!s_inYieldContext ||
        static_cast<uint32_t>(nowMs - s_cmdStartMs) < Config::kAutoStallGraceMs ||
        static_cast<uint32_t>(nowMs - s_lastStallCheckMs) < Config::kAutoStallPollMs) {
        return;
    }
    s_lastStallCheckMs = nowMs;

    bool taskStats[2] = {false, false};
    const bool taskDoneOk =
        (mmL.Get_Drive_isTaskDone(1U, taskStats) == MMLower::Drive_RESULT::OK);
    if ((taskDoneOk && !taskStats[0]) || (!taskDoneOk && !s_driveTaskActive)) {
        // No blocking DriveDC task is active (e.g. inside regular delay()).
        s_hasEncBaseline  = false;
        s_hasDegBaseline  = false;
        s_lastWheelMoveMs = nowMs;
        return;
    }

    if (!taskDoneOk) {
        // STM32 UART timed out during a blocking DriveDC wait: abort if unrecovered.
        if (static_cast<uint32_t>(nowMs - s_lastWheelMoveMs) >= Config::kAutoStallTimeoutMs) {
            abortAuto(AutoExitReason::StallTimeout);
        }
        return;
    }

    if (!s_expectWheelMotion) {
        s_hasEncBaseline  = false;
        s_hasDegBaseline  = false;
        s_lastWheelMoveMs = nowMs;
        return;
    }

    int32_t enc[4]      = {0, 0, 0, 0};
    int32_t driveDeg    = 0;
    const bool encOk    = (mmL.GetAllEncoderCounter(enc) == MMLower::RESULT::OK);
    const bool degOk    = (mmL.Get_Drive_Degrees(1U, driveDeg) == MMLower::Drive_RESULT::OK);
    if (!encOk && !degOk) {
        if (static_cast<uint32_t>(nowMs - s_lastWheelMoveMs) >= Config::kAutoStallTimeoutMs) {
            abortAuto(AutoExitReason::StallTimeout);
        }
        return;
    }

    const int32_t curEncL        = enc[s_driveEncIdxL];
    const int32_t curEncR        = enc[s_driveEncIdxR];
    const bool    hadAnyBaseline = s_hasEncBaseline || s_hasDegBaseline;
    bool          moved          = false;

    if (encOk) {
        if (!s_hasEncBaseline) {
            s_prevEncL       = curEncL;
            s_prevEncR       = curEncR;
            s_hasEncBaseline = true;
        } else if (RobotLogic::hasEncoderProgressed(s_prevEncL, curEncL, Config::kAutoStallMinDeltaDeg) ||
                   RobotLogic::hasEncoderProgressed(s_prevEncR, curEncR, Config::kAutoStallMinDeltaDeg)) {
            moved = true;
        }
    }

    if (degOk) {
        if (!s_hasDegBaseline) {
            s_prevDriveDeg   = driveDeg;
            s_hasDegBaseline = true;
        } else if (RobotLogic::hasEncoderProgressed(s_prevDriveDeg, driveDeg, Config::kAutoStallMinDeltaDeg)) {
            moved = true;
        }
    }

    if (moved) {
        if (encOk) {
            s_prevEncL = curEncL;
            s_prevEncR = curEncR;
        }
        if (degOk) {
            s_prevDriveDeg = driveDeg;
        }
        s_lastWheelMoveMs = nowMs;
    } else if (!hadAnyBaseline) {
        // First baseline sample captured: start the stall timeout window from now.
        s_lastWheelMoveMs = nowMs;
    } else if (static_cast<uint32_t>(nowMs - s_lastWheelMoveMs) >= Config::kAutoStallTimeoutMs) {
        abortAuto(AutoExitReason::StallTimeout);
    }
}

void checkSerialAbort() {
    while (Serial.available() > 0) {
        const int ch = Serial.read();
        if (ch < 0) {
            break;
        }
        if (ch == '!') {
            abortAuto(AutoExitReason::SerialEstop);
        }
        if (ch == 'u' || ch == 'U') {
            abortAuto(AutoExitReason::SerialUpload);
        }
        if (ch == 'x' || ch == 'X' || ch == ' ') {
            abortAuto(AutoExitReason::SerialStop);
        }
        // Buffer non-abort bytes so stray CR/LF or keys never block emergency '!'/'x'/'u'.
        pushSerialPassthrough(static_cast<uint8_t>(ch));
    }
}

void pollExitControls(uint32_t nowMs) {
    ButtonEvents btn{};
    s_io->pollButtons(nowMs, btn);
    if (!btn.upPressed) {
        s_sawUpReleased = true;
    }
    if (btn.uploadRequested) {
        abortAuto(AutoExitReason::UploadCombo);
    }
    if (btn.downPressEdge || btn.shortDown) {
        abortAuto(AutoExitReason::BtnEstop);
    }
    if (s_sawUpReleased && (btn.shortUp || btn.autoStartRequested)) {
        abortAuto(AutoExitReason::BtnStop);
    }

    bool valid = false;
    if (!s_linkSm->isConnected()) {
        *s_lastFrame = Ps2Frame{};
        if (s_linkSm->shouldAttemptReinit(nowMs)) {
            const bool ready = s_io->tryReconnectPs2();
            s_linkSm->onReinitResult(ready, nowMs);
        }
    }

    if (s_linkSm->isConnected()) {
        s_io->pollPs2(*s_lastFrame);
        valid                = RobotLogic::isPs2FrameValid(*s_lastFrame);
        const bool modeReset = !valid && RobotLogic::isPs2UnconfiguredMode(*s_lastFrame);
        s_linkSm->onFrame(valid, nowMs, modeReset);
    }

    s_snap->ps2Link      = s_linkSm->state();
    s_snap->ps2Glitching = s_linkSm->isLinkGlitching();
    s_snap->ps2Mode      = s_lastFrame->modeId;
    s_snap->ps2Sync      = s_lastFrame->syncByte;
    s_snap->ps2Ly        = s_lastFrame->ly;
    s_snap->ps2Rx        = s_lastFrame->rx;
    s_snap->commHealthy  = s_io->commHealthy();

    if (RobotLogic::shouldAbortAutoMode(valid, Controls::held(*s_lastFrame, Controls::kBrakeOrExit))) {
        abortAuto(AutoExitReason::Ps2Cross);
    }

    if (s_oled != nullptr &&
        static_cast<uint32_t>(nowMs - s_lastOledAutoMs) >= Config::kAutoOledRefreshMs) {
        s_lastOledAutoMs = nowMs;
        s_oled->render(*s_snap, nowMs, false);
    }
}

} // namespace

namespace AutoMode {

AutoExitReason execute(MiniR4IO& io,
                       RobotLogic::FailsafeStateMachine& linkSm,
                       SystemSnapshot& snap,
                       Ps2Frame& lastFrame,
                       OledUI* oled) {
    s_io         = &io;
    s_linkSm     = &linkSm;
    s_snap       = &snap;
    s_lastFrame  = &lastFrame;
    s_oled       = oled;
    s_exitReason = AutoExitReason::None;
    s_inService  = false;
    const uint32_t startMs   = millis();
    s_lastPollMs             = startMs;
    s_autoStartMs            = startMs;
    s_lastOledAutoMs         = startMs;
    s_inYieldContext         = false;
    s_driveTaskActive        = false;
    s_expectWheelMotion      = false;
    s_driveEncIdxL           = Config::kMotorLeftId - 1U;
    s_driveEncIdxR           = Config::kMotorRightId - 1U;
    clearSerialPassthrough();
    resetStallTracker(startMs);
    // Gyro assist is opt-in; on any IMU reset failure it stays off so DriveDC keeps legacy behaviour.
    s_gyroActive       = false;
    s_headingTargetDeg = 0;
    s_moveStep         = 0U;
    if (Config::kAutoGyroMode != Config::kAutoGyroOff) {
        s_gyroActive = (mmL.SetIMUToZero() == MMLower::RESULT::OK);
    }
    s_entryTrianglePending   = true;
    s_exitAfterLoopIteration = false;
    s_sawUpReleased          = false;
    snap.autoElapsedMs       = 0UL;

    if (setjmp(s_abortEnv) == 0) {
        s_running = true;
        AutoBlockUser::setup();
        service();
        if (s_running && probeAndRunLoopIteration()) {
            while (s_running && !s_exitAfterLoopIteration) {
                service();
                if (!s_running) {
                    break;
                }
                AutoBlockUser::loop();
            }
        }
        if (s_running) {
            s_exitReason = AutoExitReason::Completed;
            s_running    = false;
        }
    }

    snap.autoElapsedMs       = static_cast<uint32_t>(millis() - s_autoStartMs);
    s_running                = false;
    s_inService              = false;
    s_inYieldContext         = false;
    s_driveTaskActive        = false;
    s_expectWheelMotion      = false;
    s_entryTrianglePending   = false;
    s_exitAfterLoopIteration = false;
    s_gyroActive             = false;
    s_oled                   = nullptr;
    WDT.refresh();

    return (s_exitReason != AutoExitReason::None) ? s_exitReason
                                                  : AutoExitReason::Completed;
}

void service() {
    if (!s_running) {
        return;
    }

    WDT.refresh();

    if (s_inService) {
        return;
    }

    checkSerialAbort();

    const uint32_t nowMs     = millis();
    const uint32_t elapsedMs = static_cast<uint32_t>(nowMs - s_autoStartMs);
    if (s_snap != nullptr) {
        s_snap->autoElapsedMs = elapsedMs;
    }
    if (Config::kAutoMaxDurationMs > 0UL && elapsedMs >= Config::kAutoMaxDurationMs) {
        abortAuto(AutoExitReason::Timeout);
    }

    if (!s_inYieldContext && !s_driveTaskActive) {
        resetStallTracker(nowMs);
    }

    if (static_cast<uint32_t>(nowMs - s_lastPollMs) < Config::kControlPeriodMs) {
        return;
    }
    s_lastPollMs = nowMs;

    s_inService = true;
    pollExitControls(nowMs);
    checkDriveStall(nowMs);
    s_inService = false;
}

bool isRunning() {
    return s_running;
}

void recordServoAngle(uint8_t channel, uint16_t angleDeg) {
    s_yieldCalled = true;
    if (s_snap == nullptr) {
        return;
    }
    const int16_t bounded = (angleDeg > static_cast<uint16_t>(Config::kServoMaxDeg))
                                ? Config::kServoMaxDeg
                                : static_cast<int16_t>(angleDeg);
    switch (channel) {
        case 1U: s_snap->servos.rc1Deg = bounded; break;
        case 2U: s_snap->servos.rc2Deg = bounded; break;
        case 3U:
            // Auto block code writes raw hardware angles; map back to logical Manual angle.
            s_snap->servos.rc3Deg =
                RobotLogic::applyServoDirection(bounded, Config::kRc3InvertDir);
            break;
        case 4U:
            s_snap->servos.rc4Deg =
                RobotLogic::applyServoDirection(bounded, Config::kRc4InvertDir);
            break;
        default: break;
    }
    s_snap->servos = RobotLogic::clampServoTargets(s_snap->servos);
}

const char* exitReasonStr(AutoExitReason reason) {
    switch (reason) {
        case AutoExitReason::Completed:    return "Finished -> Manual ready";
        case AutoExitReason::Ps2Cross:     return "PS2 X pressed -> Manual ready";
        case AutoExitReason::SerialStop:   return "Serial 'x' stop -> Manual ready";
        case AutoExitReason::BtnStop:      return "BTN_UP stop -> Manual ready";
        case AutoExitReason::BtnEstop:     return "BTN_DOWN ESTOP";
        case AutoExitReason::SerialEstop:  return "Serial '!' ESTOP";
        case AutoExitReason::UploadCombo:  return "BTN_UP + BTN_DOWN upload";
        case AutoExitReason::SerialUpload: return "Serial 'u' upload";
        case AutoExitReason::Timeout:      return "Auto timeout -> Manual ready";
        case AutoExitReason::StallTimeout: return "Wheel stall -> Manual ready";
        case AutoExitReason::UserExit:     return "autoExit() -> Manual ready";
        case AutoExitReason::DriveInitFail: return "DriveDC init failed (STM32 comm) -> Manual ready";
        case AutoExitReason::None:
        default:                           return "Finished -> Manual ready";
    }
}

bool pollPs2ForAuto(Ps2Frame& outFrame, bool& outEntryTrianglePulse) {
    s_pollPs2Called = true;
    service();

    outEntryTrianglePulse = false;
    if (!s_running || s_lastFrame == nullptr || s_linkSm == nullptr) {
        outFrame = Ps2Frame{};
        return false;
    }

    outFrame = *s_lastFrame;
    if (s_entryTrianglePending) {
        s_entryTrianglePending = false;
        outEntryTrianglePulse  = true;
    }

    return s_linkSm->isConnected() && RobotLogic::isPs2FrameValid(outFrame);
}

bool hasEntryTrianglePending() {
    return s_running && s_entryTrianglePending;
}

void notifyPs2TriangleMatched() {
    s_exitAfterLoopIteration = true;
}

void resolveDrivePolarity(uint8_t leftMotorID,
                          uint8_t rightMotorID,
                          bool& ioLeftReverse,
                          bool& ioRightReverse) {
    if (leftMotorID >= 1U && leftMotorID <= 4U) {
        s_driveEncIdxL = static_cast<uint8_t>(leftMotorID - 1U);
    }
    if (rightMotorID >= 1U && rightMotorID <= 4U) {
        s_driveEncIdxR = static_cast<uint8_t>(rightMotorID - 1U);
    }
    if (s_snap == nullptr) {
        return;
    }
    RobotLogic::resolveAutoDrivePolarity(leftMotorID,
                                         rightMotorID,
                                         s_snap->leftReversed,
                                         s_snap->rightReversed,
                                         ioLeftReverse,
                                         ioRightReverse);
}

void beginBlockingDriveTask(bool expectWheelMotion) {
    s_yieldCalled       = true;
    s_driveTaskActive   = true;
    s_expectWheelMotion = expectWheelMotion;
    resetStallTracker(millis());
}

void endBlockingDriveTask() {
    s_driveTaskActive   = false;
    s_expectWheelMotion = false;
    s_hasEncBaseline    = false;
    s_hasDegBaseline    = false;
}

void noteUserLoopActivity() {
    s_yieldCalled = true;
}

bool gyroAssistActive() {
    return s_running && s_gyroActive;
}

int16_t headingTargetDeg() {
    return s_headingTargetDeg;
}

void setHeadingTargetDeg(int16_t deg) {
    s_headingTargetDeg = RobotLogic::wrapYawDeg(deg);
}

bool readYawDeg(int16_t& outDeg) {
    return MiniR4IO::readImuYawDeg(outDeg);
}

bool latchHeadingFromImu() {
    int16_t yaw = 0;
    if (!readYawDeg(yaw)) {
        return false;
    }
    s_headingTargetDeg = yaw;
    return true;
}

void logMove(const char* kind, int16_t a, int16_t b, uint16_t degrees, uint8_t result) {
    ++s_moveStep;
    if (!Config::kAutoMoveTelemetry || !s_gyroActive) {
        return;
    }
    int16_t    yaw = 0;
    const bool ok  = readYawDeg(yaw);
    Serial.print(F("[AUTO] #"));
    Serial.print(s_moveStep);
    Serial.print(' ');
    Serial.print(kind != nullptr ? kind : "?");
    Serial.print(F(" a="));
    Serial.print(a);
    Serial.print(F(" b="));
    Serial.print(b);
    Serial.print(F(" d="));
    Serial.print(degrees);
    Serial.print(F(" hdgT="));
    Serial.print(s_headingTargetDeg);
    if (ok) {
        Serial.print(F(" yaw="));
        Serial.print(yaw);
        Serial.print(F(" err="));
        Serial.print(RobotLogic::yawErrorDeg(s_headingTargetDeg, yaw));
    } else {
        Serial.print(F(" yaw=n/a"));
    }
    Serial.print(F(" res="));
    Serial.print(result);
    Serial.print(F(" t="));
    Serial.print(static_cast<uint32_t>(millis() - s_autoStartMs));
    Serial.println(F("ms"));
}

int serialAvailable() {
    service();
    return static_cast<int>(s_serialCount);
}

int serialRead() {
    service();
    if (s_serialCount == 0U) {
        return -1;
    }
    const uint8_t b = s_serialBuf[s_serialHead];
    s_serialHead    = static_cast<uint8_t>((s_serialHead + 1U) % kSerialPassCap);
    s_serialCount--;
    return static_cast<int>(b);
}

int serialPeek() {
    service();
    if (s_serialCount == 0U) {
        return -1;
    }
    return static_cast<int>(s_serialBuf[s_serialHead]);
}

} // namespace AutoMode

bool autoShouldExit() {
    AutoMode::service();
    return !AutoMode::isRunning();
}

void autoExit() {
    if (AutoMode::isRunning()) {
        abortAuto(AutoExitReason::UserExit);
    }
    for (;;) {
    }
}

void autoAbortDriveInit() {
    if (AutoMode::isRunning()) {
        abortAuto(AutoExitReason::DriveInitFail);
    }
    for (;;) {
    }
}

// Overrides Arduino UNO R4's weak yield() called inside delay() and DriveDC waits,
// ensuring PS2 X (Cross) abort, encoder stall detection, and WDT refresh work during Auto.
void yield() {
    if (AutoMode::isRunning()) {
        s_yieldCalled    = true;
        s_inYieldContext = true;
        AutoMode::service();
        s_inYieldContext = false;
    }
}
