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

/**
 * Physics Model 1: Cubic polynomial input shaping u = (1 - a)*x + a*(x^3 / 100^2)
 * Reduces jerk (da/dt) near stick center while preserving full +/-100% authority.
 */
int16_t applyCubicExpo(int16_t normVal, uint8_t expoPct = Config::kThrottleExpoPct);

/**
 * Physics Model 2: Unicycle centripetal traction circle limiter (|v * w| / 100 <= maxLatAccel).
 * Prevents lateral wheel slip during high-speed cornering while keeping 100% pivot turn authority.
 */
int16_t limitCentripetalTurn(int16_t linearVel,
                             int16_t angularVel,
                             int16_t maxLatAccel = Config::kMaxCentripetalLatAccel);

/**
 * Physics Model 3: Center-of-Mass (CoM) dynamic pitch/anti-tip scale (65..100%)
 * as lifting arm RC2 elevates from 140..180 deg (low CoM) toward 0 deg (high CoM).
 */
uint8_t computeCoMDynamicScale(int16_t rc2Deg);

/**
 * Physics Model 4: Coupled 2-DOF rigid-body slew limiter in translational (v) and
 * rotational (w) state space so path curvature (w/v) is preserved during accel/braking.
 */
DriveOutput apply2DofKinodynamicSlew(int16_t targetLeft,
                                     int16_t targetRight,
                                     int16_t currentLeft,
                                     int16_t currentRight,
                                     int16_t linAccelStep = Config::kMotorAccelSlewStep,
                                     int16_t linDecelStep = Config::kMotorDecelSlewStep,
                                     int16_t angAccelStep = Config::kAngularAccelSlewStep,
                                     int16_t angDecelStep = Config::kAngularDecelSlewStep);

/**
 * Physics Model 5: Brushed DC gearmotor Coulomb static friction (stiction) feedforward.
 * Maps non-zero commands into [stictionMin, maxPower] to eliminate gearbox deadband lag.
 */
int16_t applyCoulombStiction(int16_t cmdPower,
                             int16_t stictionMin = Config::kMotorStictionMinPower,
                             int16_t maxPower = Config::kMotorMaxPower);

/**
 * Stiction feedforward for a wheel pair that keeps the left/right ratio (path curvature):
 * the larger wheel is lifted through applyCoulombStiction() and the smaller wheel is scaled by
 * the same factor. A wheel commanded to 0 stays 0 (pivot about one wheel).
 */
void applyCoulombStictionPair(int16_t& left,
                              int16_t& right,
                              int16_t stictionMin = Config::kMotorStictionMinPower,
                              int16_t maxPower = Config::kMotorMaxPower);

/**
 * Full physics-based kinodynamic differential drive pipeline combining all 5 models,
 * plus reverse-front orientation toggle (R3), D-Pad Left/Right micro-turn alignment,
 * straight-line left/right motor balance trim, and full-stick Precision Mode scaling.
 */
DriveOutput computeKinodynamicDrive(uint8_t lyRaw,
                                    uint8_t rxRaw,
                                    int16_t currentLeft,
                                    int16_t currentRight,
                                    int16_t maxSpeed = Config::kMotorMaxPower,
                                    int16_t accelSlewStep = Config::kMotorAccelSlewStep,
                                    int16_t decelSlewStep = Config::kMotorDecelSlewStep,
                                    uint8_t compPercent = 100U,
                                    int16_t rc2Deg = Config::kRc2InitDeg,
                                    bool reverseDrive = false,
                                    bool dpadLeft = false,
                                    bool dpadRight = false,
                                    int8_t motorTrimPct = 0,
                                    bool precisionMode = false);

/** Steps straight-line motor balance trim within [-kMaxMotorTrimPct, +kMaxMotorTrimPct]. */
int8_t stepMotorTrim(int8_t currentTrim, int8_t delta);

/**
 * Applies straight-line left/right motor balance trim ([-15..+15]%).
 * Positive trim attenuates Right wheel by (100 - trimPct)% to steer slightly Right;
 * Negative trim attenuates Left wheel by (100 - |trimPct|)% to steer slightly Left.
 */
void applyMotorBalanceTrim(int16_t& left, int16_t& right, int8_t trimPct);

/**
 * Smoothly slews RC2 arm angle toward targetDeg by at most maxStepDeg per tick
 * within [kRc2MinDeg, kRc2MaxDeg]. Used exclusively for arm RC2.
 */
int16_t slewServoAngle(int16_t currentDeg,
                       int16_t targetDeg,
                       int16_t maxStepDeg = Config::kRc2SmoothStepDeg);

/** Computes CRC-16-CCITT (poly 0x1021, init 0xFFFF) over a byte buffer. */
uint16_t computeCrc16(const uint8_t* data, uint8_t len);

/** Packs PersistentConfig into an 8-byte EEPROM frame with magic header + CRC-16. */
void packPersistentConfig(const PersistentConfig& cfg, uint8_t outBytes[8]);

/** Validates magic, version, and CRC-16 of an 8-byte EEPROM frame and unpacks it. */
bool unpackPersistentConfig(const uint8_t inBytes[8], PersistentConfig& outCfg);

/** Returns true when absolute encoder movement |curCount - prevCount| >= minDelta. */
bool hasEncoderProgressed(int32_t prevCount,
                          int32_t curCount,
                          int32_t minDelta = Config::kAutoStallMinDeltaDeg);

/**
 * Applies saved EEPROM motor polarity flips (relative to Config::kMotorLeft/RightReverseInit)
 * onto the polarity flags passed to DriveDC.begin(leftMotorID, rightMotorID, ...).
 */
void resolveAutoDrivePolarity(uint8_t leftMotorID,
                              uint8_t rightMotorID,
                              bool savedLeftReversed,
                              bool savedRightReversed,
                              bool& ioLeftReverse,
                              bool& ioRightReverse);

/** Wraps any yaw angle (deg) into [-180, 180]. */
int16_t wrapYawDeg(int32_t yawDeg);

/** Shortest signed error (deg, [-180, 180]) from actualYawDeg to targetYawDeg. */
int16_t yawErrorDeg(int16_t targetYawDeg, int16_t actualYawDeg);

/** Rounds yawDeg to the nearest multiple of stepDeg (ties away from 0); stepDeg <= 0 returns yawDeg. */
int16_t snapYawDeg(int16_t yawDeg, int16_t stepDeg);

/**
 * Estimates yaw change (deg) of a pivot MoveDegs(left, right, wheelDegs): applies when
 * left == -right != 0. Returns 0 for any other power pair or zero degrees.
 * Positive result = Config::kAutoPivotYawSign direction (right wheel forward).
 */
int16_t estimatePivotYawDeg(int16_t leftPower, int16_t rightPower, uint16_t wheelDegs);

/** Classification of a pasted MoveDegs(left, right, ...) for gyro assist. */
enum class AutoMoveKind : uint8_t { Other = 0, Straight, Pivot };
AutoMoveKind classifyAutoMove(int16_t leftPower, int16_t rightPower, uint16_t wheelDegs);

/**
 * Manual-drive wheel stall detector (opt-in via Config::kEnableManualStallCut).
 * Trips when |wheel cmd| >= kManualStallMinCmdPct and no encoder progress for kManualStallTimeoutMs.
 * Fails open: an invalid encoder sample never trips. Stays tripped until |cmd| <= kManualStallRearmCmdPct.
 */
class ManualStallDetector {
public:
    void reset();
    /**
     * Call every control tick. `sampleFresh` marks ticks that carry a new encoder reading
     * (`sampleValid` = the read succeeded). Returns true while the cut is latched.
     */
    bool update(int16_t cmdLeft,
                int16_t cmdRight,
                bool sampleFresh,
                bool sampleValid,
                int32_t encLeft,
                int32_t encRight,
                uint32_t nowMs);
    bool tripped() const { return tripped_; }

private:
    bool     tripped_    = false;
    bool     hasBase_    = false;
    int32_t  prevLeft_   = 0;
    int32_t  prevRight_  = 0;
    uint32_t lastMoveMs_ = 0UL;
};

/**
 * Proportional heading-hold correction (percent turn power): kTeleopHoldKpX10/10 % per degree of
 * yaw error (target - actual), zero inside kTeleopHoldDeadbandDeg, clamped to +/-kTeleopHoldMaxCorrPct.
 * Positive result = rotate toward increasing yaw.
 */
int16_t computeHeadingHoldCorrection(int16_t targetYawDeg, int16_t actualYawDeg);

/**
 * Teleop heading hold (opt-in via Config::kEnableTeleopHeadingHold). Latches the IMU yaw after
 * kTeleopHoldLatchDelayMs of straight-only input, then returns a turn correction until the
 * driver turns, stops, or an IMU read fails (fail open: correction 0).
 */
class TeleopHeadingHold {
public:
    void reset();
    int16_t update(bool straightCmd,
                   bool sampleFresh,
                   bool sampleValid,
                   int16_t yawDeg,
                   uint32_t nowMs);
    bool latched() const { return latched_; }

private:
    bool     straight_      = false;
    bool     latched_       = false;
    int16_t  targetYawDeg_  = 0;
    int16_t  corrPct_       = 0;
    uint32_t straightSince_ = 0UL;
};

/**
 * Adds a turn correction to a wheel pair without exceeding the current command envelope
 * (max(|left|, |right|)). corrPct > 0 drives the right wheel forward and the left wheel back.
 * Overflow on one wheel is moved onto the other so the L/R difference (the turn) is preserved.
 * Unchanged when both wheels are 0 or corrPct is 0.
 */
void applyTurnCorrection(int16_t& left, int16_t& right, int16_t corrPct);

/** Steps the D-pad (RC2) servo angle within safe mechanical bounds. */
int16_t computeArmStep(int16_t currentDeg, bool upPressed, bool downPressed);

/** Clamps all four servo targets into [kServoMinDeg, kServoMaxDeg]. */
ServoTargets clampServoTargets(const ServoTargets& targets);

/** Clamps a logical servo angle to [kServoMinDeg, kServoMaxDeg] and mirrors it when invertDir is true. */
int16_t applyServoDirection(int16_t angleDeg, bool invertDir);

/** Updates servo targets from PS2 D-pad, shoulder, Square push-ground, and Circle preset buttons. */
void updateServoTargetsFromPs2(const Ps2Frame& frame,
                               ServoTargets& targets,
                               bool allowArmStep,
                               FrontArmPushState* pushState = nullptr,
                               ArmPosePresetState* poseState = nullptr);

/** Decodes a raw 9-byte Sony PS2 SPI response packet into a Ps2Frame. */
void decodePs2Payload(const uint8_t raw[9], Ps2Frame& outFrame);

/** Returns true if the pad replied with valid sync (0x5A) in non-analog reset mode (e.g. 0x41). */
bool isPs2UnconfiguredMode(const Ps2Frame& frame);

/** Validates PS2 frame mode/sync bytes and rejects impossible button/bus-short patterns. */
bool isPs2FrameValid(const Ps2Frame& frame);

/** Returns true on a rising Triangle press while Ready, PS2 Operational, and X not held. */
bool shouldStartAutoMode(bool sysReady,
                         bool ps2Operational,
                         bool frameValid,
                         bool trianglePressed,
                         bool prevTrianglePressed,
                         bool crossPressed);

/** Returns true when a valid PS2 frame reports Cross (X) pressed to exit Auto mode. */
bool shouldAbortAutoMode(bool frameValid, bool crossPressed);

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
