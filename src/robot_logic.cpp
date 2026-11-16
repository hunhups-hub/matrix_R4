#include "robot_logic.h"

namespace RobotLogic {

namespace {

int16_t abs16(int16_t v) {
    return (v < 0) ? static_cast<int16_t>(-v) : v;
}

void desaturateAndClampDrive(int16_t& left, int16_t& right, int16_t maxSpeed) {
    const int16_t limit = clamp16(maxSpeed, 0, Config::kMotorMaxPower);
    if (limit == 0) {
        left  = 0;
        right = 0;
        return;
    }

    const int16_t peak = (abs16(left) > abs16(right)) ? abs16(left) : abs16(right);
    if (peak > limit) {
        left  = static_cast<int16_t>((static_cast<int32_t>(left) * limit) / peak);
        right = static_cast<int16_t>((static_cast<int32_t>(right) * limit) / peak);
    }

    left  = clamp16(left, static_cast<int16_t>(-limit), limit);
    right = clamp16(right, static_cast<int16_t>(-limit), limit);
}

int16_t scaleStickToLimit(int16_t val, int16_t limit) {
    if (val == 0 || limit <= 0) {
        return 0;
    }
    const int32_t absVal = abs16(val);
    int32_t       scaled = (absVal * static_cast<int32_t>(limit) + 50L) / 100L;
    if (scaled < 1L) {
        scaled = 1L;
    }
    if (scaled > static_cast<int32_t>(limit)) {
        scaled = static_cast<int32_t>(limit);
    }
    return (val < 0) ? static_cast<int16_t>(-scaled) : static_cast<int16_t>(scaled);
}

} // namespace

int16_t clamp16(int16_t v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

long clampLong(long v, long lo, long hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int16_t computeAxisWithDeadzone(uint8_t raw, uint8_t deadzoneRaw, bool invert) {
    if (deadzoneRaw >= 127U) {
        return 0;
    }

    const int16_t lowEdge  = static_cast<int16_t>(127U - deadzoneRaw);
    const int16_t highEdge = static_cast<int16_t>(128U + deadzoneRaw);
    const int16_t span     = lowEdge;

    int16_t normalized = 0;
    if (static_cast<int16_t>(raw) <= lowEdge) {
        const int32_t dist = static_cast<int32_t>(lowEdge - static_cast<int16_t>(raw));
        normalized         = static_cast<int16_t>(-(dist * Config::kMotorMaxPower) / span);
    } else if (static_cast<int16_t>(raw) >= highEdge) {
        const int32_t dist = static_cast<int32_t>(static_cast<int16_t>(raw) - highEdge);
        normalized         = static_cast<int16_t>((dist * Config::kMotorMaxPower) / span);
    } else {
        return 0;
    }

    if (invert) {
        normalized = static_cast<int16_t>(-normalized);
    }
    return clamp16(normalized, Config::kMotorMinPower, Config::kMotorMaxPower);
}

int16_t applyAsymmetricSlewStep(int16_t current,
                                int16_t target,
                                int16_t accelStep,
                                int16_t decelStep) {
    if (current == target) {
        return target;
    }

    const bool accelerating =
        (current >= 0 && target > current) || (current <= 0 && target < current);
    const int16_t step = accelerating ? accelStep : decelStep;
    if (step <= 0) {
        return target;
    }

    // Pass cleanly through 0 on direction reversal to limit current spikes
    if ((current > 0 && target < 0 && current <= step) ||
        (current < 0 && target > 0 && -current <= step)) {
        return 0;
    }

    if (target > current) {
        const int16_t stepped = static_cast<int16_t>(current + step);
        return (stepped > target) ? target : stepped;
    }

    const int16_t stepped = static_cast<int16_t>(current - step);
    return (stepped < target) ? target : stepped;
}

DriveOutput applyDriveTargetsWithSlew(int16_t targetLeft,
                                        int16_t targetRight,
                                        int16_t currentLeft,
                                        int16_t currentRight,
                                        int16_t maxSpeed,
                                        int16_t accelSlewStep,
                                        int16_t decelSlewStep,
                                        uint8_t compPercent) {
    const int16_t cappedComp = clamp16(static_cast<int16_t>(compPercent),
                                       100,
                                       static_cast<int16_t>(Config::kBattCompMaxPct));
    int16_t scaledL = targetLeft;
    int16_t scaledR = targetRight;
    if (cappedComp > 100) {
        scaledL = static_cast<int16_t>((static_cast<int32_t>(targetLeft) * cappedComp) / 100);
        scaledR = static_cast<int16_t>((static_cast<int32_t>(targetRight) * cappedComp) / 100);
    }
    desaturateAndClampDrive(scaledL, scaledR, maxSpeed);

    DriveOutput out;
    out.left  = applyAsymmetricSlewStep(currentLeft, scaledL, accelSlewStep, decelSlewStep);
    out.right = applyAsymmetricSlewStep(currentRight, scaledR, accelSlewStep, decelSlewStep);
    return out;
}

DriveOutput computeDifferentialDrive(uint8_t lyRaw,
                                     uint8_t rxRaw,
                                     int16_t currentLeft,
                                     int16_t currentRight,
                                     int16_t maxSpeed,
                                     int16_t accelSlewStep,
                                     int16_t decelSlewStep,
                                     uint8_t compPercent) {
    // PS2 LY=0 is stick pushed UP (forward) -> invert=true
    const int16_t throttle = computeAxisWithDeadzone(lyRaw, Config::kStickDeadzoneRaw, true);
    const int16_t turn     = computeAxisWithDeadzone(rxRaw, Config::kStickDeadzoneRaw, false);

    const int16_t targetLeft  = static_cast<int16_t>(throttle + turn);
    const int16_t targetRight = static_cast<int16_t>(throttle - turn);
    return applyDriveTargetsWithSlew(targetLeft,
                                     targetRight,
                                     currentLeft,
                                     currentRight,
                                     maxSpeed,
                                     accelSlewStep,
                                     decelSlewStep,
                                     compPercent);
}

int16_t applyCubicExpo(int16_t normVal, uint8_t expoPct) {
    const int32_t x    = clamp16(normVal, Config::kMotorMinPower, Config::kMotorMaxPower);
    const int32_t e    = (expoPct > 100U) ? 100L : static_cast<int32_t>(expoPct);
    const int32_t absX = (x < 0L) ? -x : x;
    const int32_t num  = (100L - e) * absX * 10000L + e * absX * absX * absX;
    const int32_t mag  = (num + 500000L) / 1000000L;
    const int16_t res  = static_cast<int16_t>((x < 0L) ? -mag : mag);
    return clamp16(res, Config::kMotorMinPower, Config::kMotorMaxPower);
}

int16_t limitCentripetalTurn(int16_t linearVel, int16_t angularVel, int16_t maxLatAccel) {
    if (maxLatAccel <= 0) {
        return 0;
    }
    const int16_t clampedW = clamp16(angularVel, Config::kMotorMinPower, Config::kMotorMaxPower);
    const int32_t absV     = abs16(clamp16(linearVel, Config::kMotorMinPower, Config::kMotorMaxPower));
    const int32_t absW     = abs16(clampedW);
    if (absV == 0L || absW == 0L) {
        return clampedW;
    }

    const int32_t latAccel = (absV * absW) / 100L;
    if (latAccel <= static_cast<int32_t>(maxLatAccel)) {
        return clampedW;
    }

    const int32_t safeAbsW = (static_cast<int32_t>(maxLatAccel) * 100L) / absV;
    const int16_t boundedW = static_cast<int16_t>(
        clampLong(safeAbsW, 0L, static_cast<long>(Config::kMotorMaxPower)));
    return (clampedW < 0) ? static_cast<int16_t>(-boundedW) : boundedW;
}

uint8_t computeCoMDynamicScale(int16_t rc2Deg) {
    const int16_t clamped = clamp16(rc2Deg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    const int16_t kLowCoMDeg = 140;
    if (clamped >= kLowCoMDeg) {
        return 100U;
    }
    const int32_t span     = static_cast<int32_t>(kLowCoMDeg - Config::kRc2MinDeg);
    const int32_t minScale = static_cast<int32_t>(Config::kCoMHighMinScalePct);
    const int32_t scaled   =
        minScale + ((static_cast<int32_t>(clamped - Config::kRc2MinDeg) * (100L - minScale) + (span / 2L)) / span);
    return static_cast<uint8_t>(clampLong(scaled, minScale, 100L));
}

DriveOutput apply2DofKinodynamicSlew(int16_t targetLeft,
                                     int16_t targetRight,
                                     int16_t currentLeft,
                                     int16_t currentRight,
                                     int16_t linAccelStep,
                                     int16_t linDecelStep,
                                     int16_t angAccelStep,
                                     int16_t angDecelStep) {
    // Operate in doubled state coordinates (V2 = L + R = 2*v, W2 = L - R = 2*w) for zero integer rounding loss
    const int16_t curV2 = static_cast<int16_t>(currentLeft + currentRight);
    const int16_t curW2 = static_cast<int16_t>(currentLeft - currentRight);
    const int16_t tgtV2 = static_cast<int16_t>(targetLeft + targetRight);
    const int16_t tgtW2 = static_cast<int16_t>(targetLeft - targetRight);

    const int16_t nextV2 = applyAsymmetricSlewStep(curV2,
                                                   tgtV2,
                                                   static_cast<int16_t>(linAccelStep * 2),
                                                   static_cast<int16_t>(linDecelStep * 2));
    const int16_t nextW2 = applyAsymmetricSlewStep(curW2,
                                                   tgtW2,
                                                   static_cast<int16_t>(angAccelStep * 2),
                                                   static_cast<int16_t>(angDecelStep * 2));

    DriveOutput out;
    out.left  = clamp16(static_cast<int16_t>((nextV2 + nextW2) / 2),
                        Config::kMotorMinPower,
                        Config::kMotorMaxPower);
    out.right = clamp16(static_cast<int16_t>((nextV2 - nextW2) / 2),
                        Config::kMotorMinPower,
                        Config::kMotorMaxPower);
    return out;
}

int16_t applyCoulombStiction(int16_t cmdPower, int16_t stictionMin, int16_t maxPower) {
    const int16_t limit = clamp16(maxPower, 0, Config::kMotorMaxPower);
    if (cmdPower == 0 || limit == 0) {
        return 0;
    }
    if (stictionMin <= 1 || stictionMin >= limit) {
        return clamp16(cmdPower, static_cast<int16_t>(-limit), limit);
    }

    const int32_t absCmd = abs16(clamp16(cmdPower, static_cast<int16_t>(-limit), limit));
    const int32_t spanIn = static_cast<int32_t>(limit - 1);
    const int32_t spanOut = static_cast<int32_t>(limit - stictionMin);
    const int32_t mapped =
        static_cast<int32_t>(stictionMin) + (((absCmd - 1L) * spanOut + (spanIn / 2L)) / spanIn);
    const int16_t mag = static_cast<int16_t>(clampLong(mapped, static_cast<long>(stictionMin), static_cast<long>(limit)));
    return (cmdPower < 0) ? static_cast<int16_t>(-mag) : mag;
}

int8_t stepMotorTrim(int8_t currentTrim, int8_t delta) {
    const int16_t sum     = static_cast<int16_t>(currentTrim) + static_cast<int16_t>(delta);
    const int16_t clamped = clamp16(sum,
                                    static_cast<int16_t>(-Config::kMaxMotorTrimPct),
                                    static_cast<int16_t>(Config::kMaxMotorTrimPct));
    return static_cast<int8_t>(clamped);
}

void applyMotorBalanceTrim(int16_t& left, int16_t& right, int8_t trimPct) {
    const int16_t clampedTrim = clamp16(static_cast<int16_t>(trimPct),
                                        static_cast<int16_t>(-Config::kMaxMotorTrimPct),
                                        static_cast<int16_t>(Config::kMaxMotorTrimPct));
    if (clampedTrim == 0) {
        return;
    }
    if (clampedTrim > 0) {
        // Positive trim -> attenuate Right wheel by (100 - trim)% so robot steers slightly Right
        const int32_t scale = 100L - static_cast<int32_t>(clampedTrim);
        const int32_t signR = (right < 0) ? -1L : 1L;
        const int32_t magR  = (static_cast<int32_t>(abs16(right)) * scale + 50L) / 100L;
        right               = static_cast<int16_t>(signR * magR);
    } else {
        // Negative trim -> attenuate Left wheel by (100 - |trim|)% so robot steers slightly Left
        const int32_t scale = 100L + static_cast<int32_t>(clampedTrim);
        const int32_t signL = (left < 0) ? -1L : 1L;
        const int32_t magL  = (static_cast<int32_t>(abs16(left)) * scale + 50L) / 100L;
        left                = static_cast<int16_t>(signL * magL);
    }
}

int16_t slewServoAngle(int16_t currentDeg, int16_t targetDeg, int16_t maxStepDeg) {
    const int16_t cur = clamp16(currentDeg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    const int16_t tgt = clamp16(targetDeg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    if (maxStepDeg <= 0 || cur == tgt) {
        return tgt;
    }
    if (tgt > cur) {
        const int16_t next = static_cast<int16_t>(cur + maxStepDeg);
        return (next > tgt) ? tgt : next;
    }
    const int16_t next = static_cast<int16_t>(cur - maxStepDeg);
    return (next < tgt) ? tgt : next;
}

uint16_t computeCrc16(const uint8_t* data, uint8_t len) {
    uint16_t crc = 0xFFFFU;
    if (data == nullptr) {
        return crc;
    }
    for (uint8_t i = 0U; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = static_cast<uint16_t>((crc << 1U) ^ 0x1021U);
            } else {
                crc = static_cast<uint16_t>(crc << 1U);
            }
        }
    }
    return crc;
}

void packPersistentConfig(const PersistentConfig& cfg, uint8_t outBytes[8]) {
    if (outBytes == nullptr) {
        return;
    }
    uint8_t flags = 0U;
    if (cfg.leftReversed)     flags |= 0x01U;
    if (cfg.rightReversed)    flags |= 0x02U;
    if (cfg.precisionMode)    flags |= 0x04U;
    if (cfg.rc2SmoothEnabled) flags |= 0x08U;

    const int8_t clampedTrim = stepMotorTrim(cfg.motorTrimPct, 0);

    outBytes[0] = Config::kEepromMagic0;
    outBytes[1] = Config::kEepromMagic1;
    outBytes[2] = Config::kEepromVersion;
    outBytes[3] = flags;
    outBytes[4] = static_cast<uint8_t>(clampedTrim);
    outBytes[5] = 0U;

    const uint16_t crc = computeCrc16(outBytes, 6U);
    outBytes[6] = static_cast<uint8_t>(crc & 0xFFU);
    outBytes[7] = static_cast<uint8_t>((crc >> 8U) & 0xFFU);
}

bool unpackPersistentConfig(const uint8_t inBytes[8], PersistentConfig& outCfg) {
    if (inBytes == nullptr) {
        return false;
    }
    if (inBytes[0] != Config::kEepromMagic0 ||
        inBytes[1] != Config::kEepromMagic1 ||
        inBytes[2] != Config::kEepromVersion ||
        inBytes[5] != 0U) {
        return false;
    }
    const uint16_t expectedCrc = computeCrc16(inBytes, 6U);
    const uint16_t storedCrc   = static_cast<uint16_t>(inBytes[6]) |
                                 (static_cast<uint16_t>(inBytes[7]) << 8U);
    if (expectedCrc != storedCrc) {
        return false;
    }

    const uint8_t flags = inBytes[3];
    if ((flags & 0xF0U) != 0U) {
        return false;
    }

    const int8_t rawTrim = static_cast<int8_t>(inBytes[4]);
    if (rawTrim < -Config::kMaxMotorTrimPct || rawTrim > Config::kMaxMotorTrimPct) {
        return false;
    }

    outCfg.leftReversed     = (flags & 0x01U) != 0U;
    outCfg.rightReversed    = (flags & 0x02U) != 0U;
    outCfg.precisionMode    = (flags & 0x04U) != 0U;
    outCfg.rc2SmoothEnabled = (flags & 0x08U) != 0U;
    outCfg.motorTrimPct     = rawTrim;
    return true;
}

bool hasEncoderProgressed(int32_t prevCount, int32_t curCount, int32_t minDelta) {
    const int32_t threshold = (minDelta <= 0L) ? 1L : minDelta;
    const int64_t diff      = static_cast<int64_t>(curCount) - static_cast<int64_t>(prevCount);
    const int64_t absDiff   = (diff < 0) ? -diff : diff;
    return absDiff >= static_cast<int64_t>(threshold);
}

DriveOutput computeKinodynamicDrive(uint8_t lyRaw,
                                    uint8_t rxRaw,
                                    int16_t currentLeft,
                                    int16_t currentRight,
                                    int16_t maxSpeed,
                                    int16_t accelSlewStep,
                                    int16_t decelSlewStep,
                                    uint8_t compPercent,
                                    int16_t rc2Deg,
                                    bool reverseDrive,
                                    bool dpadLeft,
                                    bool dpadRight,
                                    int8_t motorTrimPct,
                                    bool precisionMode) {
    const int16_t requestedLimit = clamp16(maxSpeed, 0, Config::kMotorMaxPower);
    const int16_t limit =
        precisionMode
            ? ((requestedLimit < Config::kPrecisionMaxPower) ? requestedLimit : Config::kPrecisionMaxPower)
            : requestedLimit;
    const uint8_t comScale = computeCoMDynamicScale(rc2Deg);

    int16_t linAccel = static_cast<int16_t>((static_cast<int32_t>(accelSlewStep) * comScale + 50L) / 100L);
    int16_t linDecel = static_cast<int16_t>((static_cast<int32_t>(decelSlewStep) * comScale + 50L) / 100L);
    if (accelSlewStep > 0 && linAccel < 1) linAccel = 1;
    if (decelSlewStep > 0 && linDecel < 1) linDecel = 1;

    if (limit == 0) {
        return apply2DofKinodynamicSlew(0,
                                        0,
                                        currentLeft,
                                        currentRight,
                                        linAccel,
                                        linDecel,
                                        Config::kAngularAccelSlewStep,
                                        Config::kAngularDecelSlewStep);
    }

    // 1. Deadzone & reverse-front orientation + cubic jerk-reducing input shaping
    const int16_t rawThrottle  = computeAxisWithDeadzone(lyRaw, Config::kStickDeadzoneRaw, !reverseDrive);
    const int16_t rawTurn      = computeAxisWithDeadzone(rxRaw, Config::kStickDeadzoneRaw, false);
    const uint8_t throttleExpo = precisionMode ? Config::kPrecisionThrottleExpoPct : Config::kThrottleExpoPct;
    const uint8_t turnExpo     = precisionMode ? Config::kPrecisionTurnExpoPct : Config::kTurnExpoPct;
    int16_t       throttle     = applyCubicExpo(rawThrottle, throttleExpo);
    int16_t       turn         = applyCubicExpo(rawTurn, turnExpo);

    const bool isMicroTurn = (throttle == 0 && turn == 0 && (dpadLeft != dpadRight));
    if (isMicroTurn) {
        const int16_t microPower =
            precisionMode ? Config::kPrecisionMicroTurnPower : Config::kMicroTurnPower;
        turn = dpadRight ? microPower : static_cast<int16_t>(-microPower);
    } else {
        if (precisionMode) {
            throttle = scaleStickToLimit(throttle, limit);
            turn     = scaleStickToLimit(turn, limit);
        }
        if (throttle != 0) {
            turn = static_cast<int16_t>((static_cast<int32_t>(turn) * Config::kTurnMaxRatioPct) / 100L);
        }
    }

    // 2. CoM-scaled centripetal traction circle limiter (|v * w| / 100 <= maxLatAccel)
    const int16_t maxLatAccel = static_cast<int16_t>(
        (static_cast<int32_t>(Config::kMaxCentripetalLatAccel) * comScale + 50L) / 100L);
    const int16_t safeTurn = limitCentripetalTurn(throttle, turn, maxLatAccel);

    // 3. Unicycle inverse kinematics + Back-EMF / internal resistance voltage sag compensation
    const int16_t cappedComp = clamp16(static_cast<int16_t>(compPercent),
                                       100,
                                       static_cast<int16_t>(Config::kBattCompMaxPct));
    int16_t scaledL = static_cast<int16_t>(throttle + safeTurn);
    int16_t scaledR = static_cast<int16_t>(throttle - safeTurn);
    if (cappedComp > 100) {
        scaledL = static_cast<int16_t>((static_cast<int32_t>(scaledL) * cappedComp) / 100L);
        scaledR = static_cast<int16_t>((static_cast<int32_t>(scaledR) * cappedComp) / 100L);
    }
    desaturateAndClampDrive(scaledL, scaledR, limit);
    applyMotorBalanceTrim(scaledL, scaledR, motorTrimPct);

    // 4. Coulomb static friction feedforward on proportional stick commands (exact power for Micro-Turn)
    if (!isMicroTurn) {
        scaledL = applyCoulombStiction(scaledL, Config::kMotorStictionMinPower, limit);
        scaledR = applyCoulombStiction(scaledR, Config::kMotorStictionMinPower, limit);
    }

    // 5. Coupled 2-DOF translational (v) & rotational (w) slew limiting
    return apply2DofKinodynamicSlew(scaledL,
                                    scaledR,
                                    currentLeft,
                                    currentRight,
                                    linAccel,
                                    linDecel,
                                    Config::kAngularAccelSlewStep,
                                    Config::kAngularDecelSlewStep);
}

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

void updateServoTargetsFromPs2(const Ps2Frame& frame,
                               ServoTargets& targets,
                               bool allowArmStep,
                               FrontArmPushState* pushState,
                               ArmPosePresetState* poseState) {
    if (allowArmStep) {
        targets.rc2Deg = computeArmStep(targets.rc2Deg, frame.up, frame.down);
    }
    if ((frame.up != frame.down) && poseState != nullptr) {
        poseState->dropActive = false;
    }

    const bool squareRising = (pushState != nullptr) &&
                              frame.square &&
                              !pushState->prevSquare &&
                              !frame.l1 &&
                              !frame.r1;

    if (frame.l1 && !frame.r1) {
        targets.rc3Deg = Config::kRc3L1Deg;
        targets.rc4Deg = Config::kRc4L1Deg;
        if (pushState != nullptr) {
            pushState->active = false;
        }
    } else if (frame.r1 && !frame.l1) {
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
        pushState->prevSquare = frame.square;
    }

    const bool circleRising = (poseState != nullptr) &&
                              frame.circle &&
                              !poseState->prevCircle &&
                              !frame.l2 &&
                              !frame.r2 &&
                              (frame.up == frame.down);

    if (frame.l2 && !frame.r2) {
        targets.rc1Deg = Config::kRc1L2Deg;
        if (poseState != nullptr) {
            poseState->dropActive = false;
        }
    } else if (frame.r2 && !frame.l2) {
        targets.rc1Deg = Config::kRc1R2Deg;
        if (poseState != nullptr) {
            poseState->dropActive = false;
        }
    } else if (circleRising) {
        if (!poseState->dropActive) {
            targets.rc1Deg        = Config::kRc1PresetDropDeg;
            targets.rc2Deg        = Config::kRc2PresetDropDeg;
            poseState->dropActive = true;
        } else {
            targets.rc1Deg        = Config::kRc1PresetHoldDeg;
            targets.rc2Deg        = Config::kRc2PresetHoldDeg;
            poseState->dropActive = false;
        }
    }

    if (poseState != nullptr) {
        poseState->prevCircle = frame.circle;
    }

    targets = clampServoTargets(targets);
}

void decodePs2Payload(const uint8_t raw[9], Ps2Frame& outFrame) {
    outFrame.modeId   = raw[1];
    outFrame.syncByte = raw[2];
    outFrame.buttons1 = raw[3];
    outFrame.buttons2 = raw[4];
    outFrame.rx       = raw[5];
    outFrame.ry       = raw[6];
    outFrame.lx       = raw[7];
    outFrame.ly       = raw[8];

    // Sony PS2 button bytes are active-low (0 = pressed)
    const uint8_t b1 = static_cast<uint8_t>(~raw[3]);
    const uint8_t b2 = static_cast<uint8_t>(~raw[4]);

    outFrame.select   = (b1 & 0x01U) != 0U;
    outFrame.l3       = (b1 & 0x02U) != 0U;
    outFrame.r3       = (b1 & 0x04U) != 0U;
    outFrame.start    = (b1 & 0x08U) != 0U;
    outFrame.up       = (b1 & 0x10U) != 0U;
    outFrame.right    = (b1 & 0x20U) != 0U;
    outFrame.down     = (b1 & 0x40U) != 0U;
    outFrame.left     = (b1 & 0x80U) != 0U;

    outFrame.l2       = (b2 & 0x01U) != 0U;
    outFrame.r2       = (b2 & 0x02U) != 0U;
    outFrame.l1       = (b2 & 0x04U) != 0U;
    outFrame.r1       = (b2 & 0x08U) != 0U;
    outFrame.triangle = (b2 & 0x10U) != 0U;
    outFrame.circle   = (b2 & 0x20U) != 0U;
    outFrame.cross    = (b2 & 0x40U) != 0U;
    outFrame.square   = (b2 & 0x80U) != 0U;

    outFrame.linkOk = (outFrame.modeId == Config::kPs2ExpectedMode) &&
                      (outFrame.syncByte == Config::kPs2ExpectedSync);
}

bool isPs2UnconfiguredMode(const Ps2Frame& frame) {
    return (frame.syncByte == Config::kPs2ExpectedSync) &&
           (frame.modeId != Config::kPs2ExpectedMode);
}

bool isPs2FrameValid(const Ps2Frame& frame) {
    // Stage 1: Exact Sony analog red-mode (0x73) + valid sync (0x5A)
    if (!frame.linkOk ||
        frame.modeId != Config::kPs2ExpectedMode ||
        frame.syncByte != Config::kPs2ExpectedSync) {
        return false;
    }

    // Stage 2: Physically impossible opposing D-pad states
    if ((frame.up && frame.down) || (frame.left && frame.right)) {
        return false;
    }

    // Stage 3: Impossible all-4 shape buttons pressed simultaneously (bus short)
    if (frame.triangle && frame.circle && frame.cross && frame.square) {
        return false;
    }

    // Stage 4: Stuck floating-high SPI line (every byte == 0xFF)
    if (frame.buttons1 == 0xFFU && frame.buttons2 == 0xFFU &&
        frame.rx == 0xFFU && frame.ry == 0xFFU &&
        frame.lx == 0xFFU && frame.ly == 0xFFU) {
        return false;
    }

    // Stage 5: Short-to-ground SPI line (every byte == 0x00)
    // Note: pushing both sticks to (0,0) or (255,255) with normal buttons passes cleanly.
    if (frame.buttons1 == 0x00U && frame.buttons2 == 0x00U &&
        frame.rx == 0x00U && frame.ry == 0x00U &&
        frame.lx == 0x00U && frame.ly == 0x00U) {
        return false;
    }

    return true;
}

bool shouldStartAutoMode(bool sysReady,
                         bool ps2Operational,
                         bool frameValid,
                         bool trianglePressed,
                         bool prevTrianglePressed,
                         bool crossPressed) {
    return sysReady && ps2Operational && frameValid &&
           trianglePressed && !prevTrianglePressed && !crossPressed;
}

bool shouldAbortAutoMode(bool frameValid, bool crossPressed) {
    return frameValid && crossPressed;
}

void FailsafeStateMachine::begin(bool initialReady, uint32_t nowMs) {
    goodFrames_          = 0U;
    badFrames_           = 0U;
    modeResetCnt_        = 0U;
    lastValidMs_         = nowMs;
    lastReinitDoneMs_    = nowMs;
    reinitCooldownArmed_ = initialReady;
    state_               = initialReady ? LinkState::Stabilizing : LinkState::Disconnected;
}

void FailsafeStateMachine::onFrame(bool frameValid, uint32_t nowMs, bool modeReset) {
    if (!frameValid) {
        goodFrames_ = 0U;
        if (badFrames_ < 0xFFFFU) {
            badFrames_++;
        }

        if (modeReset) {
            if (modeResetCnt_ < 255U) {
                modeResetCnt_++;
            }
            if (modeResetCnt_ >= Config::kPs2ModeResetFrames) {
                state_               = LinkState::Disconnected;
                reinitCooldownArmed_ = false; // Allow immediate re-config without waiting 500 ms
                return;
            }
        } else {
            modeResetCnt_ = 0U;
        }

        if (state_ == LinkState::Disconnected) {
            return;
        }

        if (badFrames_ >= Config::kPs2PersistentLossFrames ||
            static_cast<uint32_t>(nowMs - lastValidMs_) >= Config::kPs2DisconnectMs) {
            state_               = LinkState::Disconnected;
            reinitCooldownArmed_ = false;
            return;
        }

        if (state_ == LinkState::Stabilizing || badFrames_ >= Config::kPs2FailsafeBadFrames) {
            state_ = LinkState::Failsafe;
        }
        return;
    }

    lastValidMs_  = nowMs;
    badFrames_    = 0U;
    modeResetCnt_ = 0U;
    if (goodFrames_ < 255U) {
        goodFrames_++;
    }

    state_ = (goodFrames_ >= Config::kPs2StableFrames)
                 ? LinkState::Operational
                 : LinkState::Stabilizing;
}

bool FailsafeStateMachine::shouldAttemptReinit(uint32_t nowMs) const {
    if (state_ != LinkState::Disconnected) {
        return false;
    }
    if (!reinitCooldownArmed_) {
        return true;
    }
    return static_cast<uint32_t>(nowMs - lastReinitDoneMs_) >= Config::kPs2ReconnectPeriodMs;
}

void FailsafeStateMachine::onReinitResult(bool ready, uint32_t nowMs) {
    lastReinitDoneMs_    = nowMs;
    reinitCooldownArmed_ = true;
    goodFrames_          = 0U;
    badFrames_           = 0U;
    modeResetCnt_        = 0U;
    if (ready) {
        lastValidMs_ = nowMs;
        state_       = LinkState::Stabilizing;
    } else {
        state_ = LinkState::Disconnected;
    }
}

void BatteryStabilizer::reset() {
    tier_                = BatteryTier::Normal;
    filteredMv_          = 0U;
    baselineMv_          = 0U;
    initBaselineMv_      = 0U;
    hasFilter_           = false;
    hasBaseline_         = false;
    extPower_            = false;
    offStreak_           = 0U;
    badStreak_           = 0U;
    recoveryActive_      = false;
    recoveryStartMs_     = 0UL;
    baselineRiseActive_  = false;
    baselineRiseStartMs_ = 0UL;
}

void BatteryStabilizer::computeThresholds(uint16_t& lowEnter,
                                          uint16_t& critEnter,
                                          uint16_t& emergEnter) const {
    const uint16_t sagLow   = hasBaseline_ ? static_cast<uint16_t>((static_cast<uint32_t>(baselineMv_) * Config::kBattLowSagPct) / 100U) : 0U;
    const uint16_t sagCrit  = hasBaseline_ ? static_cast<uint16_t>((static_cast<uint32_t>(baselineMv_) * Config::kBattCritSagPct) / 100U) : 0U;
    const uint16_t sagEmerg = hasBaseline_ ? static_cast<uint16_t>((static_cast<uint32_t>(baselineMv_) * Config::kBattEmergSagPct) / 100U) : 0U;

    lowEnter   = (sagLow > Config::kLowVoltWarnMv) ? sagLow : Config::kLowVoltWarnMv;
    critEnter  = (sagCrit > Config::kBattCritVoltMv) ? sagCrit : Config::kBattCritVoltMv;
    emergEnter = (sagEmerg > Config::kBattEmergVoltMv) ? sagEmerg : Config::kBattEmergVoltMv;

    if (critEnter >= lowEnter) {
        critEnter = (lowEnter > 200U) ? static_cast<uint16_t>(lowEnter - 200U) : 0U;
    }
    if (emergEnter >= critEnter) {
        emergEnter = (critEnter > 200U) ? static_cast<uint16_t>(critEnter - 200U) : 0U;
    }
}

BatteryTier BatteryStabilizer::update(uint16_t rawMv, bool valid, uint32_t nowMs) {
    if (!valid) {
        if (badStreak_ < 255U) {
            badStreak_++;
        }
        if (badStreak_ >= Config::kBattInvalidFallbackCnt) {
            const uint8_t savedBad = badStreak_;
            reset();
            badStreak_ = savedBad;
        }
        return tier_;
    }

    badStreak_ = 0U;

    if (rawMv < Config::kExtPowerPresentMv) {
        if (offStreak_ < 255U) {
            offStreak_++;
        }
        if (!extPower_ || offStreak_ >= Config::kBattOffDebounceTicks) {
            reset();
        }
        return tier_;
    }

    offStreak_ = 0U;
    extPower_  = true;

    const uint16_t boundedRaw = (rawMv > 25500U) ? 25500U : rawMv;
    if (!hasBaseline_) {
        initBaselineMv_      = boundedRaw;
        baselineMv_          = boundedRaw;
        hasBaseline_         = true;
        filteredMv_          = boundedRaw;
        hasFilter_           = true;
        baselineRiseActive_  = false;
        baselineRiseStartMs_ = 0UL;
    } else {
        const uint32_t pctMargin    = (static_cast<uint32_t>(initBaselineMv_) * 3U) / 100U;
        const uint16_t riseMargin   = static_cast<uint16_t>((pctMargin < 350U) ? pctMargin : 350U);
        const uint32_t maxAllowed32 = static_cast<uint32_t>(initBaselineMv_) + riseMargin;
        const uint16_t maxAllowedMv = static_cast<uint16_t>((maxAllowed32 < 25500UL) ? maxAllowed32 : 25500UL);
        const uint16_t emaInput     = (boundedRaw > maxAllowedMv) ? baselineMv_ : boundedRaw;

        if (!hasFilter_) {
            filteredMv_ = emaInput;
            hasFilter_  = true;
        } else if (emaInput < filteredMv_) {
            uint32_t step = (static_cast<uint32_t>(filteredMv_ - emaInput) * Config::kBattEmaDownAlphaPct + 50U) / 100U;
            if (step == 0U) {
                step = 1U;
            }
            filteredMv_ = (step >= filteredMv_) ? 0U : static_cast<uint16_t>(filteredMv_ - step);
        } else if (emaInput > filteredMv_) {
            uint32_t step = (static_cast<uint32_t>(emaInput - filteredMv_) * Config::kBattEmaUpAlphaPct + 50U) / 100U;
            if (step == 0U) {
                step = 1U;
            }
            filteredMv_ = static_cast<uint16_t>(filteredMv_ + step);
        }

        // Guard against transient/sustained regen voltage spikes ratcheting baselineMv_ above the true pack.
        if (tier_ == BatteryTier::Normal &&
            filteredMv_ > baselineMv_ &&
            boundedRaw > baselineMv_ &&
            boundedRaw <= maxAllowedMv) {
            if (!baselineRiseActive_) {
                baselineRiseActive_  = true;
                baselineRiseStartMs_ = nowMs;
            } else if (static_cast<uint32_t>(nowMs - baselineRiseStartMs_) >= Config::kBattRecoveryHoldMs) {
                baselineMv_         = (filteredMv_ < maxAllowedMv) ? filteredMv_ : maxAllowedMv;
                baselineRiseActive_ = false;
            }
        } else {
            baselineRiseActive_ = false;
        }
    }

    uint16_t lowEnter   = Config::kLowVoltWarnMv;
    uint16_t critEnter  = Config::kBattCritVoltMv;
    uint16_t emergEnter = Config::kBattEmergVoltMv;
    computeThresholds(lowEnter, critEnter, emergEnter);

    // Instant escalation to protect brownout-sensitive motors/STM32/PS2 receiver
    BatteryTier downTier = BatteryTier::Normal;
    if (filteredMv_ <= emergEnter) {
        downTier = BatteryTier::Emergency;
    } else if (filteredMv_ <= critEnter) {
        downTier = BatteryTier::Critical;
    } else if (filteredMv_ <= lowEnter) {
        downTier = BatteryTier::Low;
    }

    if (static_cast<uint8_t>(downTier) > static_cast<uint8_t>(tier_)) {
        tier_           = downTier;
        recoveryActive_ = false;
        return tier_;
    }

    if (tier_ == BatteryTier::Normal) {
        recoveryActive_ = false;
        return tier_;
    }

    uint16_t currentEnter = lowEnter;
    if (tier_ == BatteryTier::Emergency) {
        currentEnter = emergEnter;
    } else if (tier_ == BatteryTier::Critical) {
        currentEnter = critEnter;
    }
    const uint16_t exitThreshold = static_cast<uint16_t>(currentEnter + Config::kBattHysteresisMv);

    if (filteredMv_ >= exitThreshold) {
        if (!recoveryActive_) {
            recoveryActive_  = true;
            recoveryStartMs_ = nowMs;
        } else if (static_cast<uint32_t>(nowMs - recoveryStartMs_) >= Config::kBattRecoveryHoldMs) {
            // Step up one tier at a time so brief voltage rebound never triggers a full spike
            if (tier_ == BatteryTier::Emergency) {
                tier_ = BatteryTier::Critical;
            } else if (tier_ == BatteryTier::Critical) {
                tier_ = BatteryTier::Low;
            } else {
                tier_ = BatteryTier::Normal;
            }
            recoveryActive_ = false;
        }
    } else {
        recoveryActive_ = false;
    }

    return tier_;
}

uint8_t BatteryStabilizer::batteryPercent() const {
    if (!extPower_ || !hasFilter_ || filteredMv_ < Config::kExtPowerPresentMv) {
        return 0U;
    }

    uint16_t emptyMv = Config::kBattCritVoltMv;
    uint16_t fullMv  = baselineMv_;
    if (baselineMv_ <= 8700U) {
        // 2S LiPo (6.6V..8.4V)
        emptyMv = 6600U;
        fullMv  = 8400U;
    } else if (baselineMv_ <= 10400U) {
        // 9V adapter / NiMH pack
        emptyMv = 7000U;
        fullMv  = baselineMv_;
    } else if (baselineMv_ <= 13200U) {
        // 3S LiPo / 12V pack (9.8V..12.6V)
        emptyMv = 9800U;
        fullMv  = 12600U;
    } else if (baselineMv_ <= 17200U) {
        // 4S LiPo (13.2V..16.8V)
        emptyMv = 13200U;
        fullMv  = 16800U;
    } else {
        // 18V adapter / high-voltage supply
        emptyMv = static_cast<uint16_t>((static_cast<uint32_t>(baselineMv_) * 78U) / 100U);
        fullMv  = baselineMv_;
    }

    if (fullMv <= emptyMv) {
        return 100U;
    }
    if (filteredMv_ <= emptyMv) {
        return 0U;
    }
    if (filteredMv_ >= fullMv) {
        return 100U;
    }

    const uint32_t span = static_cast<uint32_t>(fullMv - emptyMv);
    const uint32_t pct  = (static_cast<uint32_t>(filteredMv_ - emptyMv) * 100UL + (span / 2U)) / span;
    return (pct > 100UL) ? 100U : static_cast<uint8_t>(pct);
}

MotorPowerProfile BatteryStabilizer::motorProfile() const {
    MotorPowerProfile profile;

    auto computeComp = [this](uint8_t maxCompPct) -> uint8_t {
        if (!extPower_ || !hasBaseline_ || !hasFilter_ || filteredMv_ == 0U || filteredMv_ >= baselineMv_) {
            return 100U;
        }
        const uint32_t ratio = (static_cast<uint32_t>(baselineMv_) * 100UL + (filteredMv_ / 2U)) / filteredMv_;
        if (ratio <= 100UL) {
            return 100U;
        }
        return (ratio > maxCompPct) ? maxCompPct : static_cast<uint8_t>(ratio);
    };

    if (!Config::kEnableBatterySagThrottle || tier_ == BatteryTier::Normal) {
        profile.maxPowerCap   = Config::kMotorMaxPower;
        profile.accelSlewStep = Config::kMotorAccelSlewStep;
        profile.decelSlewStep = Config::kMotorDecelSlewStep;
        profile.compPercent   = computeComp(Config::kBattCompMaxPct);
        profile.slewLimited   = false;
        return profile;
    }

    switch (tier_) {
    case BatteryTier::Low:
        profile.maxPowerCap   = Config::kBattLowMaxPower;
        profile.accelSlewStep = Config::kBattLowAccelSlew;
        profile.decelSlewStep = Config::kBattLowDecelSlew;
        profile.compPercent   = computeComp(110U);
        profile.slewLimited   = true;
        break;
    case BatteryTier::Critical:
        profile.maxPowerCap   = Config::kBattCritMaxPower;
        profile.accelSlewStep = Config::kBattCritAccelSlew;
        profile.decelSlewStep = Config::kBattCritDecelSlew;
        profile.compPercent   = 100U;
        profile.slewLimited   = true;
        break;
    case BatteryTier::Emergency:
    default:
        profile.maxPowerCap   = Config::kBattEmergMaxPower;
        profile.accelSlewStep = Config::kBattEmergAccelSlew;
        profile.decelSlewStep = Config::kBattEmergDecelSlew;
        profile.compPercent   = 100U;
        profile.slewLimited   = true;
        break;
    }
    return profile;
}

namespace {

uint8_t computePowerStabilityPenalty(bool extPower,
                                     uint8_t battPct,
                                     BatteryTier batTier,
                                     uint8_t compPct) {
    if (!extPower) {
        // Logic runs on USB, but motor/servo battery rail is absent.
        return Config::kStabilityUsbNoBattPen;
    }

    const uint8_t boundedPct = (battPct > 100U) ? 100U : battPct;

    // 1. State-of-charge headroom penalty (continuous with battery %).
    uint16_t socPen = 0U;
    if (boundedPct < 35U) {
        socPen = 12U + (static_cast<uint16_t>(35U - boundedPct) * 18U + 17U) / 35U;
    } else if (boundedPct < 75U) {
        socPen = (static_cast<uint16_t>(75U - boundedPct) * 12U + 20U) / 40U;
    }

    // 2. Dynamic voltage sag & brownout tier penalty.
    uint16_t sagPen = (compPct > 103U)
                          ? static_cast<uint16_t>((compPct - 103U + 1U) / 2U)
                          : 0U;
    if (sagPen > 8U) {
        sagPen = 8U;
    }

    switch (batTier) {
        case BatteryTier::Low:
            sagPen = static_cast<uint16_t>(Config::kStabilityPowerLowPen) + (sagPen / 2U);
            break;
        case BatteryTier::Critical:
            sagPen = Config::kStabilityPowerCritPen;
            break;
        case BatteryTier::Emergency:
            sagPen = Config::kStabilityPowerEmergPen;
            break;
        case BatteryTier::Normal:
        default:
            break;
    }

    const uint16_t combined = socPen + sagPen;
    return (combined > Config::kStabilityPowerMaxPen)
               ? Config::kStabilityPowerMaxPen
               : static_cast<uint8_t>(combined);
}

} // namespace

void StabilityMonitor::reset() {
    everConnected_ = false;
    scorePct_      = 100U;
    jitterMs_      = 0U;
    linkPen_       = 0U;
    commBusPen_    = 0U;
    commPen_       = 0U;
    powerPen_      = 0U;
    loopPen_       = 0U;
}

uint8_t StabilityMonitor::update(const StabilityInput& in) {
    if (in.ps2Link != LinkState::Disconnected) {
        everConnected_ = true;
    }

    // 1. PS2 / control link penalty
    switch (in.ps2Link) {
        case LinkState::Failsafe:
            linkPen_ = Config::kStabilityLinkMaxPen;
            break;
        case LinkState::Disconnected:
            if (everConnected_) {
                linkPen_ = Config::kStabilityLinkDropPen;
            } else if (in.activeSource == ControlSource::SerialCmd) {
                linkPen_ = Config::kStabilitySerialLinkPen;
            } else {
                linkPen_ = Config::kStabilityIdleWaitPen;
            }
            break;
        case LinkState::Stabilizing:
            linkPen_ = Config::kStabilitySyncWaitPen;
            break;
        case LinkState::Operational:
        default:
            if (!in.frameValid) {
                const uint16_t next =
                    static_cast<uint16_t>(linkPen_) + Config::kStabilityGlitchStep;
                linkPen_ = (next > Config::kStabilityLinkMaxPen)
                               ? Config::kStabilityLinkMaxPen
                               : static_cast<uint8_t>(next);
            } else {
                linkPen_ = (linkPen_ > Config::kStabilityRecoverStep)
                               ? static_cast<uint8_t>(linkPen_ - Config::kStabilityRecoverStep)
                               : 0U;
            }
            break;
    }

    // 2. STM32 lower-MCU bus health + hardware fault penalty
    if (!in.commHealthy) {
        const uint16_t next = static_cast<uint16_t>(commBusPen_) + Config::kStabilityCommStepPen;
        commBusPen_ = (next > Config::kStabilityCommBusMaxPen)
                          ? Config::kStabilityCommBusMaxPen
                          : static_cast<uint8_t>(next);
    } else {
        commBusPen_ = (commBusPen_ > Config::kStabilityRecoverStep)
                          ? static_cast<uint8_t>(commBusPen_ - Config::kStabilityRecoverStep)
                          : 0U;
    }
    const uint16_t rawCommPen = static_cast<uint16_t>(commBusPen_) +
                                (in.hwFault ? Config::kStabilityHwFaultPen : 0U);
    commPen_ = (rawCommPen > Config::kStabilityCommMaxPen)
                   ? Config::kStabilityCommMaxPen
                   : static_cast<uint8_t>(rawCommPen);

    // 3. Battery % reserve + voltage sag penalty
    powerPen_ = computePowerStabilityPenalty(in.extPower, in.battPct, in.batTier, in.compPct);

    // 4. Control loop timing jitter penalty
    const uint32_t targetMaxMs = Config::kControlPeriodMs + Config::kStabilityJitterSlackMs;
    const uint16_t overshoot   = (in.loopDtMs > targetMaxMs)
                                     ? static_cast<uint16_t>(
                                           (in.loopDtMs - Config::kControlPeriodMs > 1000UL)
                                               ? 1000UL
                                               : (in.loopDtMs - Config::kControlPeriodMs))
                                     : 0U;
    if (overshoot >= jitterMs_) {
        jitterMs_ = overshoot;
    } else {
        uint16_t decay = static_cast<uint16_t>(jitterMs_ / 8U);
        if (decay == 0U) {
            decay = 1U;
        }
        jitterMs_ = (jitterMs_ > decay) ? static_cast<uint16_t>(jitterMs_ - decay) : 0U;
    }

    const uint16_t rawLoopPen = static_cast<uint16_t>(jitterMs_ / 5U);
    loopPen_ = (rawLoopPen > Config::kStabilityLoopMaxPen)
                   ? Config::kStabilityLoopMaxPen
                   : static_cast<uint8_t>(rawLoopPen);

    const uint16_t totalPen = static_cast<uint16_t>(linkPen_) + commPen_ + powerPen_ + loopPen_;
    const uint8_t  rawScore = (totalPen >= 100U) ? 0U : static_cast<uint8_t>(100U - totalPen);

    if (rawScore <= scorePct_) {
        scorePct_ = rawScore; // Reflect degradation immediately
    } else {
        const uint16_t climbed = static_cast<uint16_t>(scorePct_) + Config::kStabilityRecoverStep;
        scorePct_ = (climbed > rawScore) ? rawScore : static_cast<uint8_t>(climbed);
    }

    return scorePct_;
}

} // namespace RobotLogic
