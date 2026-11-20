#include "robot_logic.h"

namespace RobotLogic {

namespace {

int32_t abs16To32(int16_t v) {
    return (v < 0) ? -static_cast<int32_t>(v) : static_cast<int32_t>(v);
}

int32_t abs32(int32_t v) {
    return (v < 0L) ? -v : v;
}

void desaturateAndClampDrive32(int32_t& left, int32_t& right, int16_t maxSpeed) {
    const int32_t limit = static_cast<int32_t>(clamp16(maxSpeed, 0, Config::kMotorMaxPower));
    if (limit == 0L) {
        left  = 0L;
        right = 0L;
        return;
    }

    const int32_t absL = abs32(left);
    const int32_t absR = abs32(right);
    const int32_t peak = (absL > absR) ? absL : absR;
    if (peak > limit) {
        left  = (left * limit) / peak;
        right = (right * limit) / peak;
    }

    left  = clampLong(left, -limit, limit);
    right = clampLong(right, -limit, limit);
}

void desaturateAndClampDrive(int16_t& left, int16_t& right, int16_t maxSpeed) {
    int32_t l32 = static_cast<int32_t>(left);
    int32_t r32 = static_cast<int32_t>(right);
    desaturateAndClampDrive32(l32, r32, maxSpeed);
    left  = static_cast<int16_t>(l32);
    right = static_cast<int16_t>(r32);
}

int16_t scaleStickToLimit(int16_t val, int16_t limit) {
    if (val == 0 || limit <= 0) {
        return 0;
    }
    const int32_t absVal = abs16To32(val);
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
    if ((current > 0 && target < 0 && static_cast<int32_t>(current) <= step) ||
        (current < 0 && target > 0 && -static_cast<int32_t>(current) <= step)) {
        return 0;
    }

    if (target > current) {
        const int32_t stepped = static_cast<int32_t>(current) + step;
        return (stepped > target) ? target : static_cast<int16_t>(stepped);
    }

    const int32_t stepped = static_cast<int32_t>(current) - step;
    return (stepped < target) ? target : static_cast<int16_t>(stepped);
}

DriveOutput applyDriveTargetsWithSlew(int16_t targetLeft,
                                        int16_t targetRight,
                                        int16_t currentLeft,
                                        int16_t currentRight,
                                        int16_t maxSpeed,
                                        int16_t accelSlewStep,
                                        int16_t decelSlewStep,
                                        uint8_t compPercent) {
    const int32_t cappedComp = static_cast<int32_t>(
        clamp16(static_cast<int16_t>(compPercent), 100, static_cast<int16_t>(Config::kBattCompMaxPct)));
    int32_t scaledL32 = static_cast<int32_t>(targetLeft);
    int32_t scaledR32 = static_cast<int32_t>(targetRight);
    if (cappedComp > 100L) {
        scaledL32 = (scaledL32 * cappedComp) / 100L;
        scaledR32 = (scaledR32 * cappedComp) / 100L;
    }
    desaturateAndClampDrive32(scaledL32, scaledR32, maxSpeed);

    DriveOutput out;
    out.left  = applyAsymmetricSlewStep(currentLeft, static_cast<int16_t>(scaledL32), accelSlewStep, decelSlewStep);
    out.right = applyAsymmetricSlewStep(currentRight, static_cast<int16_t>(scaledR32), accelSlewStep, decelSlewStep);
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
    const int32_t absV     = abs16To32(clamp16(linearVel, Config::kMotorMinPower, Config::kMotorMaxPower));
    const int32_t absW     = abs16To32(clampedW);
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
    if (clamped >= Config::kCoMLowArmDeg) {
        return 100U;
    }
    const int32_t span = static_cast<int32_t>(Config::kCoMLowArmDeg - Config::kRc2MinDeg);
    if (span <= 0L) {
        return 100U;
    }
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

    const int32_t absCmd = abs16To32(clamp16(cmdPower, static_cast<int16_t>(-limit), limit));
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
        const int32_t magR  = (abs16To32(right) * scale + 50L) / 100L;
        right               = static_cast<int16_t>(clampLong(signR * magR, -32767L, 32767L));
    } else {
        // Negative trim -> attenuate Left wheel by (100 - |trim|)% so robot steers slightly Left
        const int32_t scale = 100L + static_cast<int32_t>(clampedTrim);
        const int32_t signL = (left < 0) ? -1L : 1L;
        const int32_t magL  = (abs16To32(left) * scale + 50L) / 100L;
        left                = static_cast<int16_t>(clampLong(signL * magL, -32767L, 32767L));
    }
}

int16_t slewServoAngle(int16_t currentDeg, int16_t targetDeg, int16_t maxStepDeg) {
    const int16_t cur = clamp16(currentDeg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    const int16_t tgt = clamp16(targetDeg, Config::kRc2MinDeg, Config::kRc2MaxDeg);
    if (maxStepDeg <= 0 || cur == tgt) {
        return tgt;
    }
    if (tgt > cur) {
        const int32_t next = static_cast<int32_t>(cur) + maxStepDeg;
        return (next > tgt) ? tgt : static_cast<int16_t>(next);
    }
    const int32_t next = static_cast<int32_t>(cur) - maxStepDeg;
    return (next < tgt) ? tgt : static_cast<int16_t>(next);
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
    DriveOutput out = apply2DofKinodynamicSlew(scaledL,
                                               scaledR,
                                               currentLeft,
                                               currentRight,
                                               linAccel,
                                               linDecel,
                                               Config::kAngularAccelSlewStep,
                                               Config::kAngularDecelSlewStep);

    // Enforce power envelope so rapid forward-to-pivot 2-DOF slew transitions never
    // overshoot Precision/Battery caps while still allowing smooth ramp-down if already above limit.
    const int32_t curPeak  = (abs16To32(currentLeft) > abs16To32(currentRight))
                                 ? abs16To32(currentLeft)
                                 : abs16To32(currentRight);
    const int16_t envLimit = static_cast<int16_t>(
        clampLong((curPeak > static_cast<int32_t>(limit)) ? curPeak : static_cast<int32_t>(limit),
                  0L,
                  static_cast<long>(Config::kMotorMaxPower)));
    desaturateAndClampDrive(out.left, out.right, envLimit);
    return out;
}

} // namespace RobotLogic
