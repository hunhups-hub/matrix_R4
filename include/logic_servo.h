#pragma once

// Servo arm stepping, clamping and PS2 servo target updates. (pure C++11, host-testable).

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

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
                               ArmPosePresetState* poseState = nullptr,
                               uint32_t nowMs = 0);

} // namespace RobotLogic
