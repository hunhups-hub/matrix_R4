// Auto mode entry/exit around AutoMode::execute().

#include "app_state.h"
#include <WDT.h>

namespace App {

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

    // Fresh rested reading (motors just stopped) so the first mission move is already compensated.
    if (Config::kEnableAutoVoltComp) {
        const uint8_t  pct = AutoMode::samplePowerComp(true);
        const uint16_t mv  = AutoMode::lastPowerCompMv();
        char           buf[48];
        if (mv == 0U) {
            snprintf(buf, sizeof(buf), "volt comp %u%% (no battery reading)",
                     static_cast<unsigned>(pct));
        } else {
            snprintf(buf, sizeof(buf), "volt comp %u%% @%u.%02uV (ref %u.%02uV)",
                     static_cast<unsigned>(pct),
                     static_cast<unsigned>(mv / 1000U),
                     static_cast<unsigned>((mv % 1000U) / 10U),
                     static_cast<unsigned>(Config::kAutoVoltRefMv / 1000U),
                     static_cast<unsigned>((Config::kAutoVoltRefMv % 1000U) / 10U));
        }
        g_serial.printEvent("AUTO", buf);
    }
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
        g_posePresetState.dropActive     = false;
        g_posePresetState.releasePending = false;
    }
    if (g_snap.servos.rc3Deg != preAuto.rc3Deg || g_snap.servos.rc4Deg != preAuto.rc4Deg) {
        g_armPushState.active = false;
    }

    g_io.restoreAfterAuto();
    g_stallDet.reset();
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

} // namespace App
