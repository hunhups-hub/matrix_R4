#pragma once

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

int16_t clamp16(int16_t v, int16_t lo, int16_t hi);
long    clampLong(long v, long lo, long hi);

/** Rescales a raw 0..255 PS2 stick axis into -100..+100 with center deadband. */
int16_t computeAxisWithDeadzone(uint8_t raw,
                                uint8_t deadzoneRaw = Config::kStickDeadzoneRaw,
                                bool invert = false);

/** Applies separate accel/decel slew limits and always stops through 0 on sign flip. */
int16_t applyAsymmetricSlewStep(int16_t current,
                                int16_t target,
                                int16_t accelStep = Config::kMotorAccelSlewStep,
                                int16_t decelStep = Config::kMotorDecelSlewStep);

/** Computes compensated + desaturated + slewed tank targets from arbitrary L/R targets. */
DriveOutput applyDriveTargetsWithSlew(int16_t targetLeft,
                                        int16_t targetRight,
                                        int16_t currentLeft,
                                        int16_t currentRight,
                                        int16_t maxSpeed = Config::kMotorMaxPower,
                                        int16_t accelSlewStep = Config::kMotorAccelSlewStep,
                                        int16_t decelSlewStep = Config::kMotorDecelSlewStep,
                                        uint8_t compPercent = 100U);

/** Computes differential tank-steer targets with proportional desaturation, sag comp, and slew. */
DriveOutput computeDifferentialDrive(uint8_t lyRaw,
                                     uint8_t rxRaw,
                                     int16_t currentLeft,
                                     int16_t currentRight,
                                     int16_t maxSpeed = Config::kMotorMaxPower,
                                     int16_t accelSlewStep = Config::kMotorAccelSlewStep,
                                     int16_t decelSlewStep = Config::kMotorDecelSlewStep,
                                     uint8_t compPercent = 100U);

/** Steps the D-pad (RC2) servo angle within safe mechanical bounds. */
int16_t computeArmStep(int16_t currentDeg, bool upPressed, bool downPressed);

/** Clamps all four servo targets into [kServoMinDeg, kServoMaxDeg]. */
ServoTargets clampServoTargets(const ServoTargets& targets);

/** Updates servo targets from PS2 D-pad/shoulder buttons without overwriting idle presets. */
void updateServoTargetsFromPs2(const Ps2Frame& frame,
                               ServoTargets& targets,
                               bool allowArmStep);

/** Decodes a raw 9-byte Sony PS2 SPI response packet into a Ps2Frame. */
void decodePs2Payload(const uint8_t raw[9], Ps2Frame& outFrame);

/** Returns true if the pad replied with valid sync (0x5A) in non-analog reset mode (e.g. 0x41). */
bool isPs2UnconfiguredMode(const Ps2Frame& frame);

/** Validates PS2 frame mode/sync bytes and rejects impossible button/bus-short patterns. */
bool isPs2FrameValid(const Ps2Frame& frame);

class FailsafeStateMachine {
public:
    void begin(bool initialReady, uint32_t nowMs);
    void onFrame(bool frameValid, uint32_t nowMs, bool modeReset = false);

    /** Returns true if Disconnected and bounded reconnect cooldown has elapsed. */
    bool shouldAttemptReinit(uint32_t nowMs) const;

    /** Records outcome & completion timestamp of a bounded reconnect attempt. */
    void onReinitResult(bool ready, uint32_t nowMs);

    LinkState state() const { return state_; }
    bool      isOperational() const { return state_ == LinkState::Operational; }
    bool      isFailsafe() const { return state_ == LinkState::Failsafe; }
    bool      isConnected() const { return state_ != LinkState::Disconnected; }
    bool      isLinkGlitching() const {
        return state_ == LinkState::Operational && badFrames_ > 0U;
    }
    uint8_t   goodFrames() const { return goodFrames_; }
    uint16_t  badFrames() const { return badFrames_; }

private:
    LinkState state_                = LinkState::Disconnected;
    uint8_t   goodFrames_           = 0;
    uint16_t  badFrames_            = 0;
    uint8_t   modeResetCnt_         = 0;
    uint32_t  lastValidMs_          = 0;
    uint32_t  lastReinitDoneMs_     = 0;
    bool      reinitCooldownArmed_  = false;
};

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
};

/**
 * Computes real-time robot stability probability (0..100%) from PS2/serial link
 * quality, battery % + voltage sag risk, STM32 bus/config health, and loop jitter.
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

private:
    bool     everConnected_ = false;
    uint8_t  scorePct_      = 100U;
    uint16_t jitterMs_      = 0U;
    uint8_t  linkPen_       = 0U;
    uint8_t  commBusPen_    = 0U;
    uint8_t  commPen_       = 0U;
    uint8_t  powerPen_      = 0U;
    uint8_t  loopPen_       = 0U;
};

} // namespace RobotLogic
