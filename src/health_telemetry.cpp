// Battery, STM32 health, stability metrics and link-transition logging.

#include "app_state.h"
#include <WDT.h>

namespace App {

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

/** Formats into a static buffer; the result is valid until the next call. */
const char* batteryTierEventMsg(BatteryTier tier) {
    static char buf[48];
    const char* name = "LOW";
    int         cap  = Config::kBattLowMaxPower;
    if (tier == BatteryTier::Emergency) {
        name = "EMERGENCY";
        cap  = Config::kBattEmergMaxPower;
    } else if (tier == BatteryTier::Critical) {
        name = "CRITICAL";
        cap  = Config::kBattCritMaxPower;
    }
    if (Config::kEnableBatterySagThrottle) {
        snprintf(buf, sizeof(buf), "%s sag cap (%d%%)", name, cap);
    } else {
        snprintf(buf, sizeof(buf), "%s low batt (%d%% full power)", name,
                 static_cast<int>(Config::kMotorMaxPower));
    }
    return buf;
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
    in.comScalePct  = g_snap.comScalePct;
    in.driveLeft    = g_snap.drive.left;
    in.driveRight   = g_snap.drive.right;
    in.wheelStall   = Config::kEnableManualStallCut && g_stallDet.tripped();

    g_snap.stabilityPct   = g_stabMon.update(in);
    g_snap.loopJitterMs   = g_stabMon.jitterMs();
    g_snap.linkStabState  = RobotLogic::classifyStabPenalty(g_stabMon.linkPenalty());
    g_snap.powerStabState = RobotLogic::classifyStabPenalty(g_stabMon.powerPenalty());
    g_snap.commStabState  = RobotLogic::classifyStabPenalty(g_stabMon.commPenalty());
    g_snap.loopStabState  = RobotLogic::classifyStabPenalty(g_stabMon.loopPenalty());
    g_snap.driveStabState = RobotLogic::classifyStabPenalty(g_stabMon.drivePenalty());
}

} // namespace App
