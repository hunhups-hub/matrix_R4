#include "robot_logic.h"

#include "controls.h"

// Servo targets: arm step, PS2 presets, push-ground toggle, direction inversion (pure C++11).

namespace RobotLogic {

int16_t computeArmStep(int16_t currentDeg, bool upPressed, bool downPressed) {
    if (upPressed == downPressed) {
        return clamp16(currentDeg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    }
    const int16_t dirStep = Config::kRc2InvertDir
                                ? static_cast<int16_t>(-Config::kRc2StepDeg)
                                : Config::kRc2StepDeg;
    const int16_t delta   = upPressed ? dirStep : static_cast<int16_t>(-dirStep);
    return clamp16(static_cast<int16_t>(currentDeg + delta),
                   Config::kRc2MinDeg,
                   Config::kRc2MaxDeg);
}

ServoTargets clampServoTargets(const ServoTargets& targets) {
    ServoTargets clamped;
    clamped.rc1Deg = clamp16(targets.rc1Deg, Config::kServoMinDeg, Config::kServoMaxDeg);
    clamped.rc2Deg = clamp16(targets.rc2Deg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    clamped.rc3Deg = clamp16(targets.rc3Deg, Config::kServoMinDeg, Config::kServoMaxDeg);
    clamped.rc4Deg = clamp16(targets.rc4Deg, Config::kServoMinDeg, Config::kServoMaxDeg);
    return clamped;
}

int16_t applyServoDirection(int16_t angleDeg, bool invertDir) {
    const int16_t clamped = clamp16(angleDeg, Config::kServoMinDeg, Config::kServoMaxDeg);
    if (!invertDir) {
        return clamped;
    }
    return static_cast<int16_t>(Config::kServoMinDeg + (Config::kServoMaxDeg - clamped));
}

namespace {

// Front arms (RC3+RC4): L1/R1 jump to preset angles; Square toggles the push-ground pose.
void updateFrontArms(const Ps2Frame& frame, ServoTargets& targets, FrontArmPushState* pushState) {
    using Controls::held;
    const bool armsDown = held(frame, Controls::kFrontArmsDown);
    const bool armsUp   = held(frame, Controls::kFrontArmsUp);
    const bool push     = held(frame, Controls::kPushGround);

    const bool squareRising = (pushState != nullptr) &&
                              push &&
                              !pushState->prevSquare &&
                              !armsDown &&
                              !armsUp;

    if (armsDown && !armsUp) {
        targets.rc3Deg = Config::kRc3L1Deg;
        targets.rc4Deg = Config::kRc4L1Deg;
        if (pushState != nullptr) {
            pushState->active = false;
        }
    } else if (armsUp && !armsDown) {
        targets.rc3Deg = Config::kRc3R1Deg;
        targets.rc4Deg = Config::kRc4R1Deg;
        if (pushState != nullptr) {
            pushState->active = false;
        }
    } else if (squareRising) {
        if (!pushState->active) {
            pushState->savedRc3Deg = clamp16(targets.rc3Deg, Config::kServoMinDeg, Config::kServoMaxDeg);
            pushState->savedRc4Deg = clamp16(targets.rc4Deg, Config::kServoMinDeg, Config::kServoMaxDeg);
            targets.rc3Deg         = Config::kRc3SquareDeg;
            targets.rc4Deg         = Config::kRc4SquareDeg;
            pushState->active      = true;
        } else {
            targets.rc3Deg    = pushState->savedRc3Deg;
            targets.rc4Deg    = pushState->savedRc4Deg;
            pushState->active = false;
        }
    }

    if (pushState != nullptr) {
        pushState->prevSquare = push;
    }
}

// Gripper (RC1): open/close/zero buttons, Circle drop pose with delayed RC1 release.
void updateGripperAndDropPose(const Ps2Frame& frame,
                              ServoTargets& targets,
                              bool armStepHeld,
                              ArmPosePresetState* poseState,
                              uint32_t nowMs) {
    using Controls::held;
    const bool gripOpen   = held(frame, Controls::kGripperOpen);
    const bool gripClose  = held(frame, Controls::kGripperClose);
    const bool gripZero   = held(frame, Controls::kGripperZero);
    const bool posePreset = held(frame, Controls::kPosePreset);

    const bool circleRising = (poseState != nullptr) &&
                              posePreset &&
                              !poseState->prevCircle &&
                              !gripOpen &&
                              !gripClose &&
                              !armStepHeld;

    if (gripOpen && !gripClose) {
        targets.rc1Deg = Config::kRc1L2Deg;
        if (poseState != nullptr) {
            poseState->dropActive = false;
        }
    } else if (gripClose && !gripOpen) {
        targets.rc1Deg = Config::kRc1R2Deg;
        if (poseState != nullptr) {
            poseState->dropActive = false;
        }
    } else if (gripZero) {
        targets.rc1Deg = Config::kRc1ZeroDeg;
        if (poseState != nullptr) {
            poseState->dropActive     = false;
            poseState->releasePending = false;
        }
    } else if (circleRising) {
        // Circle = วางอย่างเดียว (ไม่สลับ): ขยับแขน RC2 ก่อน แล้วค่อยปล่อย RC1 หลัง kPoseDropReleaseDelayMs
        // ท่าถือใช้ Serial 'q' เท่านั้น
        targets.rc2Deg        = Config::kRc2PresetDropDeg;
        poseState->dropActive = true;
        if (Config::kPoseDropReleaseDelayMs == 0U) {
            targets.rc1Deg            = Config::kRc1PresetDropDeg;
            poseState->releasePending = false;
        } else {
            poseState->releasePending = true;
            poseState->releaseAtMs    = nowMs + Config::kPoseDropReleaseDelayMs;
        }
    }

    // มีเพียงปุ่ม Cross (kBrakeOrExit) เท่านั้นที่ยกเลิกการปล่อย RC1 ที่รออยู่
    if (poseState != nullptr && held(frame, Controls::kBrakeOrExit)) {
        poseState->releasePending = false;
    }
    if (poseState != nullptr && poseState->releasePending &&
        static_cast<int32_t>(nowMs - poseState->releaseAtMs) >= 0) {
        targets.rc1Deg            = Config::kRc1PresetDropDeg;
        poseState->releasePending = false;
    }

    if (poseState != nullptr) {
        poseState->prevCircle = posePreset;
    }
}

// Triangle: one-shot jump to the combined RC1/RC2 pose.
void updateTrianglePose(const Ps2Frame& frame, ServoTargets& targets, ArmPosePresetState* poseState) {
    const bool triangleHeld   = Controls::held(frame, Controls::kTrianglePose);
    const bool triangleRising = (poseState != nullptr) && triangleHeld && !poseState->prevTriangle;
    if (triangleRising) {
        targets.rc1Deg        = Config::kRcTrianglePoseRc1Deg;
        targets.rc2Deg        = Config::kRcTrianglePoseRc2Deg;
        poseState->dropActive = false;
    }
    if (poseState != nullptr) {
        poseState->prevTriangle = triangleHeld;
    }
}

} // namespace

void updateServoTargetsFromPs2(const Ps2Frame& frame,
                               ServoTargets& targets,
                               bool allowArmStep,
                               FrontArmPushState* pushState,
                               ArmPosePresetState* poseState,
                               uint32_t nowMs) {
    const bool armUp   = Controls::held(frame, Controls::kArmUp);
    const bool armDown = Controls::held(frame, Controls::kArmDown);

    if (allowArmStep) {
        targets.rc2Deg = computeArmStep(targets.rc2Deg, armUp, armDown);
    }
    if ((armUp != armDown) && poseState != nullptr) {
        poseState->dropActive = false;
    }

    updateFrontArms(frame, targets, pushState);
    updateGripperAndDropPose(frame, targets, armUp != armDown, poseState, nowMs);
    updateTrianglePose(frame, targets, poseState);

    targets = clampServoTargets(targets);
}

} // namespace RobotLogic
