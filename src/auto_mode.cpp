#include "auto_mode.h"

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include "Modules/MMLower.h"
#include <WDT.h>
#include <setjmp.h>

#include "config.h"
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
bool                    s_hasStallBaseline       = false;

__attribute__((noinline)) void emptyLoopStub() {
    asm volatile("" ::: "memory");
}

// Detects whether user's AutoBlockUser::loop() is empty `{}` (e.g., all MATRIXblock steps
// live in setup()) so finished linear Auto sequences return straight to Manual control,
// while non-empty loop() routines keep repeating until X or autoExit().
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
    const uint32_t t0     = micros();
    AutoBlockUser::loop();
    const uint32_t firstUs = static_cast<uint32_t>(micros() - t0);

    if (!s_running || s_exitAfterLoopIteration) {
        return false;
    }
    if (s_yieldCalled || firstUs >= kFirstMaxUs) {
        return true;
    }

    for (uint8_t round = 0U; round < 2U && s_running; ++round) {
        s_yieldCalled     = false;
        const uint32_t r0 = micros();
        for (uint8_t i = 0U; i < kBurstCalls; ++i) {
            AutoBlockUser::loop();
        }
        const uint32_t rUs = static_cast<uint32_t>(micros() - r0);
        if (s_yieldCalled) {
            return true;
        }
        if (rUs <= (baseUs + kEmptySlackUs)) {
            return false;
        }
    }
    return true;
}

[[noreturn]] void abortAuto(AutoExitReason reason) {
    s_exitReason     = reason;
    s_running        = false;
    s_inService      = false;
    s_inYieldContext = false;
    longjmp(s_abortEnv, 1);
}

void resetStallTracker(uint32_t nowMs) {
    s_cmdStartMs       = nowMs;
    s_lastStallCheckMs = nowMs;
    s_lastWheelMoveMs  = nowMs;
    s_prevEncL         = 0;
    s_prevEncR         = 0;
    s_prevDriveDeg     = 0;
    s_hasStallBaseline = false;
}

void checkDriveStall(uint32_t nowMs) {
    if (!s_inYieldContext ||
        static_cast<uint32_t>(nowMs - s_cmdStartMs) < Config::kAutoStallGraceMs ||
        static_cast<uint32_t>(nowMs - s_lastStallCheckMs) < Config::kAutoStallPollMs) {
        return;
    }
    s_lastStallCheckMs = nowMs;

    bool taskStats[2] = {false, false};
    if (mmL.Get_Drive_isTaskDone(1U, taskStats) != MMLower::Drive_RESULT::OK || !taskStats[0]) {
        // No blocking DriveDC task is active (e.g. inside regular delay()).
        s_hasStallBaseline = false;
        s_lastWheelMoveMs  = nowMs;
        return;
    }

    int32_t enc[4]      = {0, 0, 0, 0};
    int32_t driveDeg    = 0;
    const bool encOk    = (mmL.GetAllEncoderCounter(enc) == MMLower::RESULT::OK);
    const bool degOk    = (mmL.Get_Drive_Degrees(1U, driveDeg) == MMLower::Drive_RESULT::OK);
    if (!encOk && !degOk) {
        return;
    }

    if (!s_hasStallBaseline) {
        s_prevEncL         = enc[0];
        s_prevEncR         = enc[1];
        s_prevDriveDeg     = driveDeg;
        s_lastWheelMoveMs  = nowMs;
        s_hasStallBaseline = true;
        return;
    }

    const bool movedL = encOk && RobotLogic::hasEncoderProgressed(
                                     s_prevEncL, enc[0], Config::kAutoStallMinDeltaDeg);
    const bool movedR = encOk && RobotLogic::hasEncoderProgressed(
                                     s_prevEncR, enc[1], Config::kAutoStallMinDeltaDeg);
    const bool movedD = degOk && RobotLogic::hasEncoderProgressed(
                                     s_prevDriveDeg, driveDeg, Config::kAutoStallMinDeltaDeg);

    if (movedL || movedR || movedD) {
        if (encOk) {
            s_prevEncL = enc[0];
            s_prevEncR = enc[1];
        }
        if (degOk) {
            s_prevDriveDeg = driveDeg;
        }
        s_lastWheelMoveMs = nowMs;
    } else if (static_cast<uint32_t>(nowMs - s_lastWheelMoveMs) >= Config::kAutoStallTimeoutMs) {
        abortAuto(AutoExitReason::StallTimeout);
    }
}

void checkSerialAbort() {
    while (Serial.available() > 0) {
        const int ch = Serial.peek();
        if (ch == '!') {
            (void)Serial.read();
            abortAuto(AutoExitReason::SerialEstop);
        }
        if (ch == 'u' || ch == 'U') {
            (void)Serial.read();
            abortAuto(AutoExitReason::SerialUpload);
        }
        if (ch == 'x' || ch == 'X' || ch == ' ') {
            (void)Serial.read();
            abortAuto(AutoExitReason::SerialStop);
        }
        // Leave other Serial bytes intact for Auto block code that reads Serial.
        break;
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
    if (btn.shortDown) {
        abortAuto(AutoExitReason::BtnEstop);
    }
    if (s_sawUpReleased && btn.shortUp) {
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

    if (RobotLogic::shouldAbortAutoMode(valid, s_lastFrame->cross)) {
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
    resetStallTracker(startMs);
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
    s_entryTrianglePending   = false;
    s_exitAfterLoopIteration = false;
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

    if (!s_inYieldContext) {
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
        case AutoExitReason::None:
        default:                           return "Finished -> Manual ready";
    }
}

bool pollPs2ForAuto(Ps2Frame& outFrame, bool& outEntryTrianglePulse) {
    s_yieldCalled = true;
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
