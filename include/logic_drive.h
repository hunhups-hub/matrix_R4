#pragma once

// Drive pipeline: stick shaping, slew, kinodynamic models, motor trim. (pure C++11, host-testable).

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

/**
 * Same stepping as slewServoAngle but clamps to the full servo range
 * [kServoMinDeg, kServoMaxDeg]. Used for front arms RC3/RC4 (they reach 180).
 * maxStepDeg <= 0 jumps straight to the target.
 */
int16_t slewFrontArmAngle(int16_t currentDeg, int16_t targetDeg, int16_t maxStepDeg);

} // namespace RobotLogic
