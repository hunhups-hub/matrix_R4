#pragma once

// Battery stabilizer, stability monitor, pack classification, Auto power comp. (pure C++11, host-testable).

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

/**
 * Adaptive battery/supply stabilizer for UNO R4 (USB, 9V-18V DC adapter, 2S-4S pack).
 * Uses asymmetric EMA sag filtering, idle baseline tracking, hysteresis, recovery hold,
 * and adaptive motor power/slew limits to prevent brownouts on weak batteries.
 */
class BatteryStabilizer {
public:
    void reset();
    BatteryTier update(uint16_t rawMv, bool valid, uint32_t nowMs);

    BatteryTier       tier() const { return tier_; }
    uint16_t          filteredMv() const { return filteredMv_; }
    uint16_t          baselineMv() const { return baselineMv_; }
    bool              isExternalPower() const { return extPower_; }
    uint8_t           batteryPercent() const;
    MotorPowerProfile motorProfile() const;

private:
    void computeThresholds(uint16_t& lowEnter, uint16_t& critEnter, uint16_t& emergEnter) const;
    void ingestSample(uint16_t boundedRaw, uint32_t nowMs);
    void tryRecoverTier(uint16_t currentEnter, uint32_t nowMs);

    BatteryTier tier_                = BatteryTier::Normal;
    uint16_t    filteredMv_          = 0U;
    uint16_t    baselineMv_          = 0U;
    uint16_t    initBaselineMv_      = 0U;
    bool        hasFilter_           = false;
    bool        hasBaseline_         = false;
    bool        extPower_            = false;
    uint8_t     offStreak_           = 0U;
    uint8_t     badStreak_           = 0U;
    bool        recoveryActive_      = false;
    uint32_t    recoveryStartMs_     = 0UL;
    bool        baselineRiseActive_  = false;
    uint32_t    baselineRiseStartMs_ = 0UL;
};

struct StabilityInput {
    LinkState     ps2Link      = LinkState::Disconnected;
    ControlSource activeSource = ControlSource::None;
    bool          frameValid   = false;
    bool          commHealthy  = true;
    bool          hwFault      = false;
    bool          extPower     = false;
    uint8_t       battPct      = 0U;
    BatteryTier   batTier      = BatteryTier::Normal;
    uint8_t       compPct      = 100U;
    uint32_t      loopDtMs     = Config::kControlPeriodMs;
    uint8_t       comScalePct  = 100U;  // CoM scale from RC2 angle (100 = arm low)
    int16_t       driveLeft    = 0;     // Commanded wheel power this tick (%)
    int16_t       driveRight   = 0;
    bool          wheelStall   = false; // ManualStallDetector tripped (wheels commanded, no motion)
};

/**
 * Heuristic stability / health score (0..100%, not a probability). Starts at 100 and subtracts
 * penalties: PS2/serial link quality, STM32 bus health, battery % + sag tier, loop jitter, and
 * drive risk (arm raised while driving hard = tip risk, wheel stall). Weights are hand-set.
 */
class StabilityMonitor {
public:
    void reset();
    uint8_t update(const StabilityInput& in);

    uint8_t  scorePct() const { return scorePct_; }
    uint16_t jitterMs() const { return jitterMs_; }
    uint8_t  linkPenalty() const { return linkPen_; }
    uint8_t  powerPenalty() const { return powerPen_; }
    uint8_t  commPenalty() const { return commPen_; }
    uint8_t  loopPenalty() const { return loopPen_; }
    uint8_t  drivePenalty() const { return drivePen_; }

private:
    void updateLinkPenalty(const StabilityInput& in);
    void updateCommPenalty(const StabilityInput& in);
    void updateLoopPenalty(const StabilityInput& in);

    bool     everConnected_ = false;
    uint8_t  scorePct_      = 100U;
    uint16_t jitterMs_      = 0U;
    uint8_t  linkPen_       = 0U;
    uint8_t  commBusPen_    = 0U;
    uint8_t  commPen_       = 0U;
    uint8_t  powerPen_      = 0U;
    uint8_t  loopPen_       = 0U;
    uint8_t  drivePen_      = 0U;
};

/**
 * Drive-risk penalty: tip risk grows with how high RC2 lifts the arm (CoM scale below 100)
 * times how hard the wheels are commanded, plus a fixed penalty while wheels are stalled.
 */
uint8_t computeDriveStabilityPenalty(uint8_t comScalePct,
                                     int16_t driveLeft,
                                     int16_t driveRight,
                                     bool    wheelStall);

/**
 * Battery pack range from the rested baseline voltage (mV), auto-detected per cell count:
 * 2S (<= 8.7 V), 9 V adapter/NiMH (8.7..9.6 V), 3S (9.6..12.9 V), 4S (12.9..17.2 V), HV adapter.
 * A drained 3S/4S lands in its own range (low %), never in a range that reads 100%.
 */
void classifyBatteryPack(uint16_t baselineMv, uint16_t& emptyMv, uint16_t& fullMv);

/**
 * Auto voltage compensation factor (%) = refMv / battMv, clamped to [minPct, maxPct].
 * PWM duty sets wheel speed only in proportion to pack voltage, so scaling the commanded
 * power by this ratio keeps the real wheel speed (and the coast after a braked MoveDegs)
 * the same as when the mission was tuned at refMv. Returns 100 (no change) when battMv is
 * below Config::kExtPowerPresentMv (USB only / bad read) or refMv is 0.
 */
uint8_t computeAutoPowerCompPct(uint16_t battMv, uint16_t refMv, uint8_t minPct, uint8_t maxPct);

/**
 * Scales a signed drive power by compPct (%), rounding half up. Keeps the sign, keeps 0 at 0,
 * never turns a non-zero power into 0, and clamps to +/-Config::kMotorMaxPower.
 * compPct 0 or 100 returns the power unchanged.
 */
int16_t applyAutoPowerComp(int16_t power, uint8_t compPct);

} // namespace RobotLogic
