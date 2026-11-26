#include "app_state.h"
#include <WDT.h>

namespace App {

void latchMenuEdges(bool valid) {
    using Controls::held;
    g_prevPs2.autoStart = valid && held(g_lastPs2Frame, Controls::kAutoStart);
    g_prevPs2.precision = valid && held(g_lastPs2Frame, Controls::kPrecisionToggle);
    g_prevPs2.smooth    = valid && held(g_lastPs2Frame, Controls::kSmoothToggle);
    g_prevPs2.reverse   = valid && held(g_lastPs2Frame, Controls::kReverseToggle);
}

void latchServoEdges(bool valid) {
    g_armPushState.prevSquare    = valid && Controls::held(g_lastPs2Frame, Controls::kPushGround);
    g_posePresetState.prevCircle = valid && Controls::held(g_lastPs2Frame, Controls::kPosePreset);
    g_posePresetState.prevTriangle = valid && Controls::held(g_lastPs2Frame, Controls::kTrianglePose);
}

/** Seeds every edge latch from the current frame so held buttons don't fire on (re)entry. */
void latchAllPs2Edges(bool valid) {
    latchMenuEdges(valid);
    latchServoEdges(valid);
}

void syncPs2Snapshot() {
    g_snap.ps2Link      = g_linkSm.state();
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
 */
void handlePs2ConfigCombos() {
    using Controls::held;
    const Ps2Frame& f        = g_lastPs2Frame;

    if (held(f, Controls::kPrecisionToggle) && !g_prevPs2.precision) {
        togglePrecisionMode("PS2: ");
    }
    if (held(f, Controls::kReverseToggle) && !g_prevPs2.reverse) {
        toggleReverseDrive("PS2: Reverse-Front ON", "PS2: Reverse-Front OFF");
    }
    if (held(f, Controls::kSmoothToggle) && !g_prevPs2.smooth) {
        toggleRc2Smooth("PS2: RC2 Smooth Slew ON (saved)",
                        "PS2: RC2 Smooth Slew OFF (saved)");
    }
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

} // namespace App

using namespace App;

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

    // Resyncs cleanly after stalls/UART timeouts instead of burst catch-up ticks.
    uint32_t dtMs = 0UL;
    if (RobotLogic::advanceControlTick(nowMs, g_lastControlMs, dtMs)) {
        runControlTick(nowMs, dtMs);
    }
}
