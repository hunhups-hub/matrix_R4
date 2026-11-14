#include <string.h>
#include <unity.h>

#include "config.h"
#include "robot_logic.h"
#include "serial_control.h"
#include "types.h"

namespace {

Ps2Frame makeValidFrame(uint8_t ly = 128U, uint8_t rx = 128U) {
    uint8_t raw[9] = {0xFFU, 0x73U, 0x5AU, 0xFFU, 0xFFU, rx, 128U, 128U, ly};
    Ps2Frame f;
    RobotLogic::decodePs2Payload(raw, f);
    return f;
}

} // namespace

void setUp(void) {
    NativeSerial::reset();
}
void tearDown(void) {}

// ============================================================================
// 1. Differential Drive Kinematics & Desaturation Tests
// ============================================================================

void test_axis_deadzone_and_extremes(void) {
    // Deadzone center band (127 - 18 = 109 to 128 + 18 = 146) produces exactly 0
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::computeAxisWithDeadzone(127U, 18U, false));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::computeAxisWithDeadzone(128U, 18U, true));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::computeAxisWithDeadzone(109U, 18U, false));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::computeAxisWithDeadzone(146U, 18U, false));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::computeAxisWithDeadzone(120U, 18U, false));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::computeAxisWithDeadzone(135U, 18U, true));

    // Full stick deflections produce exact +/-100%
    TEST_ASSERT_EQUAL_INT16(100, RobotLogic::computeAxisWithDeadzone(0U, 18U, true));
    TEST_ASSERT_EQUAL_INT16(-100, RobotLogic::computeAxisWithDeadzone(255U, 18U, true));
    TEST_ASSERT_EQUAL_INT16(100, RobotLogic::computeAxisWithDeadzone(255U, 18U, false));
    TEST_ASSERT_EQUAL_INT16(-100, RobotLogic::computeAxisWithDeadzone(0U, 18U, false));

    // Boundary edge cases: deadzone >= 127 returns 0 safely
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::computeAxisWithDeadzone(0U, 127U, false));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::computeAxisWithDeadzone(255U, 200U, true));
}

void test_differential_drive_zero_speed_boundary(void) {
    // Zero maxSpeed boundary (maxSpeed = 0) must clamp outputs strictly to (0, 0)
    const DriveOutput zeroFwd = RobotLogic::computeDifferentialDrive(0U, 128U, 0, 0, 0, 25, 50, 100U);
    TEST_ASSERT_EQUAL_INT16(0, zeroFwd.left);
    TEST_ASSERT_EQUAL_INT16(0, zeroFwd.right);

    const DriveOutput zeroTurn = RobotLogic::computeDifferentialDrive(128U, 255U, 0, 0, 0, 100, 100, 100U);
    TEST_ASSERT_EQUAL_INT16(0, zeroTurn.left);
    TEST_ASSERT_EQUAL_INT16(0, zeroTurn.right);

    // Negative maxSpeed boundary (e.g. -50) clamped to 0 -> produces (0, 0)
    const DriveOutput negLimit = RobotLogic::applyDriveTargetsWithSlew(100, 100, 0, 0, -50, 50, 50, 100U);
    TEST_ASSERT_EQUAL_INT16(0, negLimit.left);
    TEST_ASSERT_EQUAL_INT16(0, negLimit.right);

    // If currently moving, zero maxSpeed decelerates toward 0
    const DriveOutput stopping = RobotLogic::applyDriveTargetsWithSlew(100, 100, 50, -50, 0, 25, 50, 100U);
    TEST_ASSERT_EQUAL_INT16(0, stopping.left);
    TEST_ASSERT_EQUAL_INT16(0, stopping.right);
}

void test_differential_drive_saturation_and_ratio_preservation(void) {
    // High speed + high steering saturation preserving differential turning ratio:
    // targetLeft = 150, targetRight = 50 (ratio 3:1).
    // Desaturation scales by 100 / 150: scaledL = 100, scaledR = 33 (ratio 3.03:1 preserved).
    const DriveOutput satFwd = RobotLogic::applyDriveTargetsWithSlew(150, 50, 100, 33, 100, 100, 100, 100U);
    TEST_ASSERT_EQUAL_INT16(100, satFwd.left);
    TEST_ASSERT_EQUAL_INT16(33, satFwd.right);

    // Negative high speed + high steering:
    // targetLeft = -160, targetRight = -40 (ratio 4:1).
    // Peak = 160 > 100 -> scaledL = -100, scaledR = -25 (ratio 4:1 preserved).
    const DriveOutput satRev = RobotLogic::applyDriveTargetsWithSlew(-160, -40, -100, -25, 100, 100, 100, 100U);
    TEST_ASSERT_EQUAL_INT16(-100, satRev.left);
    TEST_ASSERT_EQUAL_INT16(-25, satRev.right);

    // Full forward + full right stick:
    // throttle = 100, turn = 100 -> raw targets (200, 0).
    // Desaturation keeps L = +100, R = 0.
    const DriveOutput fullTurn = RobotLogic::computeDifferentialDrive(0U, 255U, 0, 0, 100, 100, 100, 100U);
    TEST_ASSERT_EQUAL_INT16(100, fullTurn.left);
    TEST_ASSERT_EQUAL_INT16(0, fullTurn.right);

    // Precision limit (maxSpeed = 55) caps output while preserving equality
    const DriveOutput precision = RobotLogic::computeDifferentialDrive(0U, 128U, 55, 55, 55, 25, 50, 100U);
    TEST_ASSERT_EQUAL_INT16(55, precision.left);
    TEST_ASSERT_EQUAL_INT16(55, precision.right);

    // Battery torque compensation: 115% scaling on (50, -50) produces (57, -57)
    const DriveOutput comp = RobotLogic::applyDriveTargetsWithSlew(50, -50, 50, -50, 100, 25, 50, 115U);
    TEST_ASSERT_EQUAL_INT16(57, comp.left);
    TEST_ASSERT_EQUAL_INT16(-57, comp.right);
}

void test_differential_drive_pivot_turn(void) {
    // Pure pivot clockwise: throttle = 0 (lyRaw = 128), steer = +100 (rxRaw = 255)
    // Produces targetLeft = +100, targetRight = -100
    const DriveOutput pivotCw = RobotLogic::computeDifferentialDrive(128U, 255U, 0, 0, 100, 100, 100, 100U);
    TEST_ASSERT_EQUAL_INT16(100, pivotCw.left);
    TEST_ASSERT_EQUAL_INT16(-100, pivotCw.right);

    // Pure pivot counter-clockwise: throttle = 0 (lyRaw = 128), steer = -100 (rxRaw = 0)
    // Produces targetLeft = -100, targetRight = +100
    const DriveOutput pivotCcw = RobotLogic::computeDifferentialDrive(128U, 0U, 0, 0, 100, 100, 100, 100U);
    TEST_ASSERT_EQUAL_INT16(-100, pivotCcw.left);
    TEST_ASSERT_EQUAL_INT16(100, pivotCcw.right);

    // Progressive slew under pivot turn from stop (accelStep = 25):
    DriveOutput step1 = RobotLogic::computeDifferentialDrive(128U, 255U, 0, 0, 100, 25, 50, 100U);
    TEST_ASSERT_EQUAL_INT16(25, step1.left);
    TEST_ASSERT_EQUAL_INT16(-25, step1.right);

    DriveOutput step2 = RobotLogic::computeDifferentialDrive(128U, 255U, step1.left, step1.right, 100, 25, 50, 100U);
    TEST_ASSERT_EQUAL_INT16(50, step2.left);
    TEST_ASSERT_EQUAL_INT16(-50, step2.right);

    DriveOutput step3 = RobotLogic::computeDifferentialDrive(128U, 255U, step2.left, step2.right, 100, 25, 50, 100U);
    TEST_ASSERT_EQUAL_INT16(75, step3.left);
    TEST_ASSERT_EQUAL_INT16(-75, step3.right);

    DriveOutput step4 = RobotLogic::computeDifferentialDrive(128U, 255U, step3.left, step3.right, 100, 25, 50, 100U);
    TEST_ASSERT_EQUAL_INT16(100, step4.left);
    TEST_ASSERT_EQUAL_INT16(-100, step4.right);

    // Pivot turn in precision mode (maxSpeed = 55)
    DriveOutput precPivot = RobotLogic::computeDifferentialDrive(128U, 255U, 0, 0, 55, 100, 100, 100U);
    TEST_ASSERT_EQUAL_INT16(55, precPivot.left);
    TEST_ASSERT_EQUAL_INT16(-55, precPivot.right);
}

// ============================================================================
// 2. Asymmetric Slew & Zero Crossing Tests
// ============================================================================

void test_asymmetric_slew_limits_and_step_delta(void) {
    const int16_t accelStep = 25;
    const int16_t decelStep = 50;

    // Positive acceleration limits (step = 25)
    TEST_ASSERT_EQUAL_INT16(25, RobotLogic::applyAsymmetricSlewStep(0, 100, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(50, RobotLogic::applyAsymmetricSlewStep(25, 100, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(75, RobotLogic::applyAsymmetricSlewStep(50, 100, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(100, RobotLogic::applyAsymmetricSlewStep(75, 100, accelStep, decelStep));

    // Positive deceleration limits (step = 50)
    TEST_ASSERT_EQUAL_INT16(50, RobotLogic::applyAsymmetricSlewStep(100, 0, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyAsymmetricSlewStep(50, 0, accelStep, decelStep));

    // Negative acceleration limits (step = 25)
    TEST_ASSERT_EQUAL_INT16(-25, RobotLogic::applyAsymmetricSlewStep(0, -100, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(-50, RobotLogic::applyAsymmetricSlewStep(-25, -100, accelStep, decelStep));

    // Negative deceleration limits (step = 50)
    TEST_ASSERT_EQUAL_INT16(-50, RobotLogic::applyAsymmetricSlewStep(-100, 0, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyAsymmetricSlewStep(-50, 0, accelStep, decelStep));

    // Target within single slew step delta:
    // 10 -> 15 (delta = 5 < accelStep 25) reaches 15 immediately without overshoot
    TEST_ASSERT_EQUAL_INT16(15, RobotLogic::applyAsymmetricSlewStep(10, 15, accelStep, decelStep));
    // 15 -> 10 (delta = 5 < decelStep 50) reaches 10 immediately
    TEST_ASSERT_EQUAL_INT16(10, RobotLogic::applyAsymmetricSlewStep(15, 10, accelStep, decelStep));
    // -10 -> -15 (delta = 5 < accelStep 25) reaches -15 immediately
    TEST_ASSERT_EQUAL_INT16(-15, RobotLogic::applyAsymmetricSlewStep(-10, -15, accelStep, decelStep));
    // -15 -> -10 (delta = 5 < decelStep 50) reaches -10 immediately
    TEST_ASSERT_EQUAL_INT16(-10, RobotLogic::applyAsymmetricSlewStep(-15, -10, accelStep, decelStep));

    // Exact equality
    TEST_ASSERT_EQUAL_INT16(50, RobotLogic::applyAsymmetricSlewStep(50, 50, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyAsymmetricSlewStep(0, 0, accelStep, decelStep));

    // Non-positive step delta fallback
    TEST_ASSERT_EQUAL_INT16(80, RobotLogic::applyAsymmetricSlewStep(10, 80, 0, decelStep));
    TEST_ASSERT_EQUAL_INT16(10, RobotLogic::applyAsymmetricSlewStep(80, 10, accelStep, -5));
}

void test_asymmetric_slew_zero_crossing_trajectory(void) {
    const int16_t accelStep = 25;
    const int16_t decelStep = 50;

    // Full forward to reverse trajectory: current = 80, target = -80
    // Step 1: 80 -> 30 (decel step 50)
    int16_t speed = RobotLogic::applyAsymmetricSlewStep(80, -80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(30, speed);

    // Step 2: 30 -> 0 (zero-crossing intercept: current 30 <= decelStep 50)
    speed = RobotLogic::applyAsymmetricSlewStep(speed, -80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(0, speed);

    // Step 3: 0 -> -25 (accel step 25)
    speed = RobotLogic::applyAsymmetricSlewStep(speed, -80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(-25, speed);

    // Step 4: -25 -> -50
    speed = RobotLogic::applyAsymmetricSlewStep(speed, -80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(-50, speed);

    // Step 5: -50 -> -75
    speed = RobotLogic::applyAsymmetricSlewStep(speed, -80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(-75, speed);

    // Step 6: -75 -> -80 (delta = 5 < 25)
    speed = RobotLogic::applyAsymmetricSlewStep(speed, -80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(-80, speed);

    // Reverse to forward trajectory: current = -80, target = 80
    // Step 1: -80 -> -30 (decel step 50)
    speed = RobotLogic::applyAsymmetricSlewStep(-80, 80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(-30, speed);

    // Step 2: -30 -> 0 (zero-crossing intercept: -current 30 <= decelStep 50)
    speed = RobotLogic::applyAsymmetricSlewStep(speed, 80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(0, speed);

    // Step 3: 0 -> 25 (accel step 25)
    speed = RobotLogic::applyAsymmetricSlewStep(speed, 80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(25, speed);

    // Step 4: 25 -> 50
    speed = RobotLogic::applyAsymmetricSlewStep(speed, 80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(50, speed);

    // Step 5: 50 -> 75
    speed = RobotLogic::applyAsymmetricSlewStep(speed, 80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(75, speed);

    // Step 6: 75 -> 80
    speed = RobotLogic::applyAsymmetricSlewStep(speed, 80, accelStep, decelStep);
    TEST_ASSERT_EQUAL_INT16(80, speed);

    // Direct boundary landings on 0
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyAsymmetricSlewStep(50, -100, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyAsymmetricSlewStep(-50, 100, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyAsymmetricSlewStep(20, -80, accelStep, decelStep));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyAsymmetricSlewStep(-15, 60, accelStep, decelStep));
}

// ============================================================================
// 3. Failsafe State Transitions & PS2 Validation Tests
// ============================================================================

void test_ps2_decode_and_5stage_validation(void) {
    Ps2Frame valid = makeValidFrame(0U, 255U);
    TEST_ASSERT_TRUE(valid.linkOk);
    TEST_ASSERT_TRUE(RobotLogic::isPs2FrameValid(valid));

    // Corner-stick: both sticks at (0,0) or (255,255) with normal buttons must PASS
    uint8_t cornerMin[9] = {0xFFU, 0x73U, 0x5AU, 0xFFU, 0xFFU, 0U, 0U, 0U, 0U};
    uint8_t cornerMax[9] = {0xFFU, 0x73U, 0x5AU, 0xFEU, 0xFFU, 255U, 255U, 255U, 255U};
    Ps2Frame fMin, fMax;
    RobotLogic::decodePs2Payload(cornerMin, fMin);
    RobotLogic::decodePs2Payload(cornerMax, fMax);
    TEST_ASSERT_TRUE(fMax.select);
    TEST_ASSERT_TRUE(RobotLogic::isPs2FrameValid(fMin));
    TEST_ASSERT_TRUE(RobotLogic::isPs2FrameValid(fMax));

    // Stage 1: Corrupted mode byte or bad sync byte
    uint8_t badMode[9] = {0xFFU, 0x7FU, 0x5AU, 0xFFU, 0xFFU, 128U, 128U, 128U, 128U};
    Ps2Frame fBadMode;
    RobotLogic::decodePs2Payload(badMode, fBadMode);
    TEST_ASSERT_FALSE(RobotLogic::isPs2FrameValid(fBadMode));
    TEST_ASSERT_TRUE(RobotLogic::isPs2UnconfiguredMode(fBadMode));

    uint8_t badSync[9] = {0xFFU, 0x73U, 0x00U, 0xFFU, 0xFFU, 128U, 128U, 128U, 128U};
    Ps2Frame fBadSync;
    RobotLogic::decodePs2Payload(badSync, fBadSync);
    TEST_ASSERT_FALSE(RobotLogic::isPs2FrameValid(fBadSync));

    // Stage 2: Opposing D-pad buttons
    Ps2Frame badDpad = valid;
    badDpad.up   = true;
    badDpad.down = true;
    TEST_ASSERT_FALSE(RobotLogic::isPs2FrameValid(badDpad));

    badDpad.up    = false;
    badDpad.down  = false;
    badDpad.left  = true;
    badDpad.right = true;
    TEST_ASSERT_FALSE(RobotLogic::isPs2FrameValid(badDpad));

    // Stage 3: All 4 shape buttons pressed simultaneously (bus short)
    Ps2Frame allShapes = valid;
    allShapes.triangle = true;
    allShapes.circle   = true;
    allShapes.cross    = true;
    allShapes.square   = true;
    TEST_ASSERT_FALSE(RobotLogic::isPs2FrameValid(allShapes));

    // Stage 4 & 5: Bus floating high (all 0xFF) or short-to-ground (all 0x00)
    uint8_t stuckHigh[9] = {0xFFU, 0x73U, 0x5AU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    uint8_t stuckLow[9]  = {0x00U, 0x73U, 0x5AU, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    Ps2Frame fHigh, fLow;
    RobotLogic::decodePs2Payload(stuckHigh, fHigh);
    RobotLogic::decodePs2Payload(stuckLow, fLow);
    TEST_ASSERT_FALSE(RobotLogic::isPs2FrameValid(fHigh));
    TEST_ASSERT_FALSE(RobotLogic::isPs2FrameValid(fLow));
}

void test_failsafe_burst_glitch_absorption_and_trip(void) {
    RobotLogic::FailsafeStateMachine sm;
    sm.begin(false, 0UL);
    sm.onReinitResult(true, 100UL);

    // Arm link: 3 consecutive valid frames reach Operational
    sm.onFrame(true, 125UL);
    sm.onFrame(true, 150UL);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LinkState::Stabilizing), static_cast<uint8_t>(sm.state()));
    sm.onFrame(true, 175UL);
    TEST_ASSERT_TRUE(sm.isOperational());
    TEST_ASSERT_EQUAL_UINT16(0U, sm.badFrames());
    TEST_ASSERT_FALSE(sm.isLinkGlitching());

    // Rapid corrupted burst: bad frames 1, 2, 3 absorbed without tripping Failsafe
    sm.onFrame(false, 200UL);
    TEST_ASSERT_TRUE(sm.isOperational());
    TEST_ASSERT_TRUE(sm.isLinkGlitching());
    TEST_ASSERT_EQUAL_UINT16(1U, sm.badFrames());

    sm.onFrame(false, 225UL);
    TEST_ASSERT_TRUE(sm.isOperational());
    TEST_ASSERT_TRUE(sm.isLinkGlitching());
    TEST_ASSERT_EQUAL_UINT16(2U, sm.badFrames());

    sm.onFrame(false, 250UL);
    TEST_ASSERT_TRUE(sm.isOperational());
    TEST_ASSERT_TRUE(sm.isLinkGlitching());
    TEST_ASSERT_EQUAL_UINT16(3U, sm.badFrames());

    // Single valid frame absorbs glitch burst, resetting badFrames count
    sm.onFrame(true, 275UL);
    TEST_ASSERT_EQUAL_UINT16(0U, sm.badFrames());
    TEST_ASSERT_FALSE(sm.isLinkGlitching());

    // Re-arm to Operational
    sm.onFrame(true, 300UL);
    sm.onFrame(true, 325UL);
    TEST_ASSERT_TRUE(sm.isOperational());

    // Consecutive 4th bad frame triggers Failsafe trip
    sm.onFrame(false, 350UL); // Bad 1
    sm.onFrame(false, 375UL); // Bad 2
    sm.onFrame(false, 400UL); // Bad 3
    TEST_ASSERT_TRUE(sm.isOperational());

    sm.onFrame(false, 425UL); // Bad 4 -> trips Failsafe
    TEST_ASSERT_TRUE(sm.isFailsafe());
    TEST_ASSERT_FALSE(sm.isOperational());
    TEST_ASSERT_TRUE(sm.isConnected());

    // Recovery from Failsafe requires 3 consecutive good frames
    sm.onFrame(true, 450UL);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LinkState::Stabilizing), static_cast<uint8_t>(sm.state()));
    sm.onFrame(true, 475UL);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LinkState::Stabilizing), static_cast<uint8_t>(sm.state()));
    sm.onFrame(true, 500UL);
    TEST_ASSERT_TRUE(sm.isOperational());
}

void test_failsafe_persistent_loss_disconnect_and_reinit_cooldown(void) {
    RobotLogic::FailsafeStateMachine sm;
    sm.begin(false, 0UL);
    sm.onReinitResult(true, 100UL);
    sm.onFrame(true, 125UL);
    sm.onFrame(true, 150UL);
    sm.onFrame(true, 175UL);
    TEST_ASSERT_TRUE(sm.isOperational());

    // 120 bad frames (3000 ms at 25 ms cadence) triggers Disconnected
    uint32_t t = 200UL;
    for (uint16_t i = 1; i < Config::kPs2PersistentLossFrames; ++i, t += 25UL) {
        sm.onFrame(false, t);
        TEST_ASSERT_TRUE(sm.isFailsafe() || sm.isOperational());
        TEST_ASSERT_TRUE(sm.isConnected());
    }
    TEST_ASSERT_EQUAL_UINT16(119U, sm.badFrames());

    // 120th bad frame triggers Disconnect
    sm.onFrame(false, t);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LinkState::Disconnected), static_cast<uint8_t>(sm.state()));
    TEST_ASSERT_FALSE(sm.isConnected());

    // Reinit cooldown timing: after persistent loss disconnect, reinit is immediately armed
    TEST_ASSERT_TRUE(sm.shouldAttemptReinit(t));

    // When reinit attempt completes with failure, cooldown is armed for kPs2ReconnectPeriodMs (500 ms)
    sm.onReinitResult(false, t + 100UL);
    TEST_ASSERT_FALSE(sm.shouldAttemptReinit(t + 100UL + 499UL));
    TEST_ASSERT_TRUE(sm.shouldAttemptReinit(t + 100UL + 500UL));

    // Time-based disconnect test: > 3000 ms elapsed with bad frames drops link
    sm.onReinitResult(true, 10000UL);
    sm.onFrame(true, 10025UL);
    sm.onFrame(true, 10050UL);
    sm.onFrame(true, 10075UL);
    TEST_ASSERT_TRUE(sm.isOperational());
    sm.onFrame(false, 10100UL); // Glitch
    sm.onFrame(false, 13150UL); // > 3000 ms gap
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LinkState::Disconnected), static_cast<uint8_t>(sm.state()));

    // 3 consecutive mode reset frames (0x41) trigger immediate Disconnect with zero cooldown delay
    sm.onReinitResult(true, 20000UL);
    sm.onFrame(true, 20025UL);
    sm.onFrame(true, 20050UL);
    sm.onFrame(true, 20075UL);
    TEST_ASSERT_TRUE(sm.isOperational());

    sm.onFrame(false, 20100UL, true);
    sm.onFrame(false, 20125UL, true);
    TEST_ASSERT_TRUE(sm.isOperational());
    sm.onFrame(false, 20150UL, true); // 3rd mode reset frame
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(LinkState::Disconnected), static_cast<uint8_t>(sm.state()));
    TEST_ASSERT_TRUE(sm.shouldAttemptReinit(20150UL));
}

void test_servo_bounds_and_ps2_presets(void) {
    const ServoTargets defaults{};
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(Config::kServoMinDeg, defaults.rc3Deg);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kServoMaxDeg, defaults.rc3Deg);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(Config::kServoMinDeg, Config::kRc3L1Deg);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kServoMaxDeg, Config::kRc3L1Deg);

    ServoTargets bad;
    bad.rc1Deg = -20;
    bad.rc2Deg = 220;
    bad.rc3Deg = -100;
    bad.rc4Deg = 300;
    const ServoTargets clamped = RobotLogic::clampServoTargets(bad);
    TEST_ASSERT_EQUAL_INT16(0, clamped.rc1Deg);
    TEST_ASSERT_EQUAL_INT16(180, clamped.rc2Deg);
    TEST_ASSERT_EQUAL_INT16(0, clamped.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(180, clamped.rc4Deg);

    ServoTargets servos{};
    Ps2Frame frame = makeValidFrame();
    frame.l1 = true;
    frame.l2 = true;
    frame.up = true;
    RobotLogic::updateServoTargetsFromPs2(frame, servos, true);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3L1Deg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4L1Deg, servos.rc4Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc1L2Deg, servos.rc1Deg);
    const int16_t expectedUpDeg = Config::kRc2InvertDir
                                      ? static_cast<int16_t>(Config::kRc2InitDeg - Config::kRc2StepDeg)
                                      : static_cast<int16_t>(Config::kRc2InitDeg + Config::kRc2StepDeg);
    TEST_ASSERT_EQUAL_INT16(expectedUpDeg, servos.rc2Deg);

    // Opposite D-pad step returns cleanly to init angle
    const int16_t steppedBack = RobotLogic::computeArmStep(servos.rc2Deg, false, true);
    TEST_ASSERT_EQUAL_INT16(Config::kRc2InitDeg, steppedBack);

    // Idle frame preserves current servo targets
    Ps2Frame idle = makeValidFrame();
    RobotLogic::updateServoTargetsFromPs2(idle, servos, true);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3L1Deg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc1L2Deg, servos.rc1Deg);
}

// ============================================================================
// 4. Battery Sag & Hysteresis Tests
// ============================================================================

void test_battery_stabilizer_multiprofile_baselines(void) {
    RobotLogic::BatteryStabilizer batt;

    // 1. USB power profile (< 5500 mV)
    batt.reset();
    batt.update(4900U, true, 0UL);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Normal), static_cast<uint8_t>(batt.tier()));
    TEST_ASSERT_FALSE(batt.isExternalPower());
    TEST_ASSERT_EQUAL_UINT8(0U, batt.batteryPercent());
    TEST_ASSERT_EQUAL_INT16(100, batt.motorProfile().maxPowerCap);

    // 2. 2S LiPo profile (baseline <= 8700 mV: 6.6V empty, 8.4V full)
    batt.reset();
    batt.update(8400U, true, 0UL);
    TEST_ASSERT_TRUE(batt.isExternalPower());
    TEST_ASSERT_EQUAL_UINT16(8400U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT8(100U, batt.batteryPercent());

    // Drain filtered voltage to 7500 mV (50% for 2S LiPo)
    for (int i = 0; i < 25; ++i) {
        batt.update(7500U, true, 100UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(7500U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(50U, batt.batteryPercent());

    // Drain filtered voltage to 6600 mV (0% for 2S LiPo)
    for (int i = 0; i < 25; ++i) {
        batt.update(6600U, true, 1000UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(6600U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(0U, batt.batteryPercent());

    // 3. 9V DC Adapter profile (baseline <= 10400 mV: 7.0V empty, baseline full)
    batt.reset();
    batt.update(9000U, true, 0UL);
    TEST_ASSERT_TRUE(batt.isExternalPower());
    TEST_ASSERT_EQUAL_UINT16(9000U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT8(100U, batt.batteryPercent());

    for (int i = 0; i < 25; ++i) {
        batt.update(8000U, true, 100UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(8000U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(50U, batt.batteryPercent());

    for (int i = 0; i < 25; ++i) {
        batt.update(7000U, true, 1000UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(7000U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(0U, batt.batteryPercent());

    // 4. 3S LiPo profile (baseline <= 13200 mV: 9.8V empty, 12.6V full)
    batt.reset();
    batt.update(12600U, true, 0UL);
    TEST_ASSERT_TRUE(batt.isExternalPower());
    TEST_ASSERT_EQUAL_UINT16(12600U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT8(100U, batt.batteryPercent());

    for (int i = 0; i < 25; ++i) {
        batt.update(11200U, true, 100UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(11200U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(50U, batt.batteryPercent());

    for (int i = 0; i < 25; ++i) {
        batt.update(9800U, true, 1000UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(9800U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(0U, batt.batteryPercent());

    // 5. 4S LiPo profile (baseline <= 17200 mV: 13.2V empty, 16.8V full)
    batt.reset();
    batt.update(16800U, true, 0UL);
    TEST_ASSERT_TRUE(batt.isExternalPower());
    TEST_ASSERT_EQUAL_UINT16(16800U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT8(100U, batt.batteryPercent());

    for (int i = 0; i < 25; ++i) {
        batt.update(15000U, true, 100UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(15000U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(50U, batt.batteryPercent());

    for (int i = 0; i < 25; ++i) {
        batt.update(13200U, true, 1000UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(13200U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(0U, batt.batteryPercent());

    // 6. High-voltage 18V Adapter profile (baseline > 17200 mV: 78% empty, baseline full)
    batt.reset();
    batt.update(18000U, true, 0UL);
    TEST_ASSERT_TRUE(batt.isExternalPower());
    TEST_ASSERT_EQUAL_UINT16(18000U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT8(100U, batt.batteryPercent());

    // 18000 * 0.78 = 14040 mV empty
    for (int i = 0; i < 25; ++i) {
        batt.update(14040U, true, 1000UL + i * 25UL);
    }
    TEST_ASSERT_EQUAL_UINT16(14040U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(0U, batt.batteryPercent());
}

void test_battery_stabilizer_progressive_sag_tiers(void) {
    RobotLogic::BatteryStabilizer batt;
    batt.reset();
    batt.update(12600U, true, 0UL); // 3S LiPo baseline

    // Normal Tier profile
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Normal), static_cast<uint8_t>(batt.tier()));
    MotorPowerProfile prof = batt.motorProfile();
    TEST_ASSERT_EQUAL_INT16(Config::kMotorMaxPower, prof.maxPowerCap);
    TEST_ASSERT_EQUAL_INT16(Config::kMotorAccelSlewStep, prof.accelSlewStep);
    TEST_ASSERT_EQUAL_INT16(Config::kMotorDecelSlewStep, prof.decelSlewStep);
    TEST_ASSERT_FALSE(prof.slewLimited);

    // Drive voltage down to Low tier (sag threshold <= 83% of 12600 = 10458 mV)
    uint32_t t = 100UL;
    for (int i = 0; i < 6; ++i, t += 100UL) {
        batt.update(10000U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Low), static_cast<uint8_t>(batt.tier()));
    prof = batt.motorProfile();
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattLowMaxPower : Config::kMotorMaxPower, prof.maxPowerCap);
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattLowAccelSlew : Config::kMotorAccelSlewStep, prof.accelSlewStep);
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattLowDecelSlew : Config::kMotorDecelSlewStep, prof.decelSlewStep);
    TEST_ASSERT_EQUAL(Config::kEnableBatterySagThrottle, prof.slewLimited);

    // Drive voltage down to Critical tier (sag threshold <= 76% of 12600 = 9576 mV)
    for (int i = 0; i < 6; ++i, t += 100UL) {
        batt.update(9200U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Critical), static_cast<uint8_t>(batt.tier()));
    prof = batt.motorProfile();
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattCritMaxPower : Config::kMotorMaxPower, prof.maxPowerCap);
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattCritAccelSlew : Config::kMotorAccelSlewStep, prof.accelSlewStep);
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattCritDecelSlew : Config::kMotorDecelSlewStep, prof.decelSlewStep);
    TEST_ASSERT_EQUAL(Config::kEnableBatterySagThrottle, prof.slewLimited);

    // Drive voltage down to Emergency tier (sag threshold <= 69% of 12600 = 8694 mV)
    for (int i = 0; i < 6; ++i, t += 100UL) {
        batt.update(8200U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Emergency), static_cast<uint8_t>(batt.tier()));
    prof = batt.motorProfile();
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattEmergMaxPower : Config::kMotorMaxPower, prof.maxPowerCap);
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattEmergAccelSlew : Config::kMotorAccelSlewStep, prof.accelSlewStep);
    TEST_ASSERT_EQUAL_INT16(Config::kEnableBatterySagThrottle ? Config::kBattEmergDecelSlew : Config::kMotorDecelSlewStep, prof.decelSlewStep);
    TEST_ASSERT_EQUAL(Config::kEnableBatterySagThrottle, prof.slewLimited);
}

void test_battery_stabilizer_hysteresis_and_timed_recovery(void) {
    RobotLogic::BatteryStabilizer batt;
    batt.reset();
    batt.update(12600U, true, 0UL);

    // Drive into Emergency tier
    uint32_t t = 100UL;
    for (int i = 0; i < 12; ++i, t += 100UL) {
        batt.update(8200U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Emergency), static_cast<uint8_t>(batt.tier()));

    // Transient invalid readings must NOT corrupt current tier or baseline
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Emergency), static_cast<uint8_t>(batt.update(0U, false, t + 25UL)));
    TEST_ASSERT_EQUAL_UINT16(12600U, batt.baselineMv());

    // Exit threshold for Emergency is enter (8694 mV) + kBattHysteresisMv (350 mV) = 9044 mV.
    // Feed 12600 mV to warm filtered voltage above exit threshold.
    for (int i = 0; i < 14; ++i, t += 25UL) {
        batt.update(12600U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Emergency), static_cast<uint8_t>(batt.tier()));

    // Hysteresis recovery requires 1500 ms continuous hold:
    // At t + 1000 ms (< 1500 ms hold), tier must remain Emergency
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Emergency),
                            static_cast<uint8_t>(batt.update(12600U, true, t + 1000UL)));

    // At t + 1600 ms (>= 1500 ms hold), tier steps up by exactly ONE tier to Critical
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Critical),
                            static_cast<uint8_t>(batt.update(12600U, true, t + 1600UL)));

    // Next 1500 ms hold steps up to Low
    batt.update(12600U, true, t + 1650UL);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Low),
                            static_cast<uint8_t>(batt.update(12600U, true, t + 3250UL)));

    // Next 1500 ms hold steps up to Normal
    batt.update(12600U, true, t + 3300UL);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Normal),
                            static_cast<uint8_t>(batt.update(12600U, true, t + 4900UL)));

    // Hysteresis cancellation: if voltage dips back below exit threshold during hold, timer resets
    // Force back to Emergency
    t += 5000UL;
    for (int i = 0; i < 15; ++i, t += 100UL) {
        batt.update(7500U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Emergency), static_cast<uint8_t>(batt.tier()));

    // Re-warm above threshold at tArm
    uint32_t tArm = t + 500UL;
    for (int i = 0; i < 14; ++i, tArm += 25UL) {
        batt.update(12600U, true, tArm);
    }
    TEST_ASSERT_GREATER_THAN_UINT16(9044U, batt.filteredMv());

    // Advance 800 ms into the recovery hold
    batt.update(12600U, true, tArm + 800UL);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Emergency), static_cast<uint8_t>(batt.tier()));

    // Sag voltage deeply to pull filtered voltage below exit threshold (9044 mV)
    uint32_t tDip = tArm + 850UL;
    for (int i = 0; i < 10; ++i, tDip += 25UL) {
        batt.update(7000U, true, tDip);
    }
    TEST_ASSERT_LESS_THAN_UINT16(9044U, batt.filteredMv());

    // Re-warm voltage above threshold again at tReWarm
    uint32_t tReWarm = tDip + 100UL;
    for (int i = 0; i < 14; ++i, tReWarm += 25UL) {
        batt.update(12600U, true, tReWarm);
    }
    TEST_ASSERT_GREATER_THAN_UINT16(9044U, batt.filteredMv());

    // At tReWarm + 1000 ms (< 1500 ms since re-warming, even though > 2000 ms since initial tArm):
    // Tier MUST still be Emergency because the dip cancelled recoveryActive_
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Emergency),
                            static_cast<uint8_t>(batt.update(12600U, true, tReWarm + 1000UL)));

    // After full 1600 ms hold since re-warm, it steps up to Critical
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Critical),
                            static_cast<uint8_t>(batt.update(12600U, true, tReWarm + 1600UL)));
}

void test_battery_stabilizer_regen_spike_protection(void) {
    RobotLogic::BatteryStabilizer batt;

    // 1. Seed 2S LiPo pack at 8400 mV (max allowed baseline rise = +3% capped at +252 mV -> 8652 mV)
    uint32_t t = 1000UL;
    batt.update(8400U, true, t);
    TEST_ASSERT_EQUAL_UINT16(8400U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT8(100U, batt.batteryPercent());

    // 2. Transient large motor-braking regen spike (12500 mV for 250 ms) must not ratchet baseline or filteredMv
    for (int i = 0; i < 10; ++i) {
        t += 25UL;
        batt.update(12500U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT16(8400U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT16(8400U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(BatteryTier::Normal), static_cast<uint8_t>(batt.tier()));

    // 3. Sustained large regen spike (14000 mV for 3 seconds) must never cross into 3S baseline
    for (int i = 0; i < 120; ++i) {
        t += 25UL;
        batt.update(14000U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT16(8400U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT16(8400U, batt.filteredMv());
    TEST_ASSERT_EQUAL_UINT8(100U, batt.batteryPercent());

    // 4. Small in-band overshoot (8600 mV < 8652 mV) held < 1500 ms then returning to 8400 mV must not raise baseline
    for (int i = 0; i < 20; ++i) {
        t += 25UL;
        batt.update(8600U, true, t);
    }
    t += 25UL;
    batt.update(8400U, true, t);
    TEST_ASSERT_EQUAL_UINT16(8400U, batt.baselineMv());

    // 5. Genuine resting rebound within +3% (8600 mV) held >= 1500 ms updates baseline safely inside 2S tier
    for (int i = 0; i < 70; ++i) {
        t += 25UL;
        batt.update(8600U, true, t);
    }
    TEST_ASSERT_GREATER_THAN_UINT16(8400U, batt.baselineMv());
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(8652U, batt.baselineMv());

    // 6. 3S LiPo baseline (12600 mV) under transient 16500 mV regen spike stays locked at 12600 mV
    batt.reset();
    t = 10000UL;
    batt.update(12600U, true, t);
    for (int i = 0; i < 16; ++i) {
        t += 25UL;
        batt.update(16500U, true, t);
    }
    TEST_ASSERT_EQUAL_UINT16(12600U, batt.baselineMv());
    TEST_ASSERT_EQUAL_UINT16(12600U, batt.filteredMv());
}

// ============================================================================
// 5. Stability Monitor Tests
// ============================================================================

void test_stability_monitor_individual_penalties(void) {
    RobotLogic::StabilityMonitor stb;

    // 1. Link penalties across states
    RobotLogic::StabilityInput in{};
    in.commHealthy  = true;
    in.extPower     = true;
    in.battPct      = 100U;
    in.batTier      = BatteryTier::Normal;
    in.compPct      = 100U;
    in.loopDtMs     = Config::kControlPeriodMs;

    // Failsafe link penalty = 35%
    in.ps2Link = LinkState::Failsafe;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityLinkMaxPen, stb.linkPenalty());

    // Stabilizing link penalty = 10%
    in.ps2Link = LinkState::Stabilizing;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilitySyncWaitPen, stb.linkPenalty());

    // Disconnected (after ever connected) = 25%
    in.ps2Link = LinkState::Disconnected;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityLinkDropPen, stb.linkPenalty());

    // Disconnected before ever connected:
    stb.reset();
    in.ps2Link      = LinkState::Disconnected;
    in.activeSource = ControlSource::None;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityIdleWaitPen, stb.linkPenalty()); // 5%

    in.activeSource = ControlSource::SerialCmd;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilitySerialLinkPen, stb.linkPenalty()); // 2%

    // Operational link glitch penalty (+8% per bad frame up to 35%)
    stb.reset();
    in.ps2Link    = LinkState::Operational;
    in.frameValid = true;
    stb.update(in); // Initialize clean Operational baseline (linkPen_ = 0)
    TEST_ASSERT_EQUAL_UINT8(0U, stb.linkPenalty());

    in.frameValid = false;
    stb.update(in); // Tick 1: 0 + 8 = 8%
    TEST_ASSERT_EQUAL_UINT8(8U, stb.linkPenalty());
    stb.update(in); // Tick 2: 8 + 8 = 16%
    TEST_ASSERT_EQUAL_UINT8(16U, stb.linkPenalty());
    stb.update(in); // Tick 3: 16 + 8 = 24%
    TEST_ASSERT_EQUAL_UINT8(24U, stb.linkPenalty());
    stb.update(in); // Tick 4: 24 + 8 = 32%
    TEST_ASSERT_EQUAL_UINT8(32U, stb.linkPenalty());
    stb.update(in); // Tick 5: 32 + 8 = 40% -> capped at 35%
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityLinkMaxPen, stb.linkPenalty()); // Capped at 35%

    // 2. Comm & Hardware Fault penalties
    stb.reset();
    in.ps2Link     = LinkState::Operational;
    in.frameValid  = true;
    in.commHealthy = false;
    stb.update(in); // +10%
    TEST_ASSERT_EQUAL_UINT8(10U, stb.commPenalty());
    stb.update(in); // +10% = 20%
    TEST_ASSERT_EQUAL_UINT8(20U, stb.commPenalty());
    stb.update(in); // +10% = capped at 25%
    TEST_ASSERT_EQUAL_UINT8(25U, stb.commPenalty());

    in.hwFault = true; // +10% -> 35% -> capped at 30%
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityCommMaxPen, stb.commPenalty()); // 30%

    // 3. Power penalties
    stb.reset();
    in.commHealthy = true;
    in.hwFault     = false;
    in.extPower    = false; // USB bench penalty
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityUsbNoBattPen, stb.powerPenalty()); // 18%

    in.extPower = true;
    in.battPct  = 100U;
    in.batTier  = BatteryTier::Normal;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(0U, stb.powerPenalty());

    in.batTier = BatteryTier::Low;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityPowerLowPen, stb.powerPenalty()); // 12%

    in.batTier = BatteryTier::Critical;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityPowerCritPen, stb.powerPenalty()); // 20%

    in.batTier = BatteryTier::Emergency;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityPowerEmergPen, stb.powerPenalty()); // 30%

    in.battPct = 0U; // SoC penalty + Emergency tier -> capped at 45%
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityPowerMaxPen, stb.powerPenalty()); // 45%

    // 4. Loop timing jitter penalties
    stb.reset();
    in.batTier  = BatteryTier::Normal;
    in.battPct  = 100U;
    in.loopDtMs = Config::kControlPeriodMs; // 25 ms nominal
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(0U, stb.loopPenalty());

    in.loopDtMs = 30UL; // Within slack (25 + 5 ms)
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(0U, stb.loopPenalty());

    in.loopDtMs = 50UL; // Overshoot = 25 ms -> loopPen = 25 / 5 = 5%
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(5U, stb.loopPenalty());

    in.loopDtMs = 125UL; // Overshoot = 100 ms -> loopPen = 100 / 5 = 20% (capped at 20%)
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityLoopMaxPen, stb.loopPenalty()); // 20%
}

void test_stability_monitor_saturation_and_deterministic_recovery(void) {
    RobotLogic::StabilityMonitor stb;

    // Combined catastrophic failure saturates score strictly to 0%
    RobotLogic::StabilityInput fault{};
    fault.ps2Link     = LinkState::Failsafe;       // 35%
    fault.frameValid  = false;
    fault.commHealthy = false;                     // Comm bus penalty 10..25%
    fault.hwFault     = true;                      // HW fault 10% -> total comm 30%
    fault.extPower    = true;
    fault.battPct     = 0U;                        // 30% SoC
    fault.batTier     = BatteryTier::Emergency;    // 30% sag -> total power 45%
    fault.compPct     = 100U;
    fault.loopDtMs    = 125UL;                     // 20% loop penalty
    // Total raw penalty = 35 + 30 + 45 + 20 = 130% >= 100% -> rawScore = 0%
    for (int i = 0; i < 5; ++i) {
        stb.update(fault);
    }
    TEST_ASSERT_EQUAL_UINT8(0U, stb.scorePct());

    // Deterministic recovery rate: exactly +2% per tick
    RobotLogic::StabilityInput healthy{};
    healthy.ps2Link     = LinkState::Operational;
    healthy.frameValid  = true;
    healthy.commHealthy = true;
    healthy.hwFault     = false;
    healthy.extPower    = true;
    healthy.battPct     = 100U;
    healthy.batTier     = BatteryTier::Normal;
    healthy.compPct     = 100U;
    healthy.loopDtMs    = Config::kControlPeriodMs;

    // Each tick climbs by exactly kStabilityRecoverStep = 2%
    uint8_t prev = stb.scorePct();
    for (int tick = 1; tick <= 5; ++tick) {
        const uint8_t cur = stb.update(healthy);
        TEST_ASSERT_EQUAL_UINT8(prev + Config::kStabilityRecoverStep, cur);
        prev = cur;
    }

    // After 50 total recovery ticks from 0%, score saturates cleanly at 100%
    for (int i = 0; i < 50; ++i) {
        stb.update(healthy);
    }
    TEST_ASSERT_EQUAL_UINT8(100U, stb.scorePct());

    // Immediate degradation on risk detection (no damping on down-step)
    RobotLogic::StabilityInput glitch = healthy;
    glitch.frameValid = false; // Glitch penalty +8% -> score drops immediately to 92%
    const uint8_t degraded = stb.update(glitch);
    TEST_ASSERT_EQUAL_UINT8(92U, degraded);
}

// ============================================================================
// 6. Adversarial Bounds Clamping Tests
// ============================================================================

void test_clamp_long_and_adversarial_boundaries(void) {
    // Basic clamping within normal range
    TEST_ASSERT_EQUAL_INT32(0L, RobotLogic::clampLong(0L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(50L, RobotLogic::clampLong(50L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(-50L, RobotLogic::clampLong(-50L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(100L, RobotLogic::clampLong(100L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(-100L, RobotLogic::clampLong(-100L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(100L, RobotLogic::clampLong(101L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(-100L, RobotLogic::clampLong(-101L, -100L, 100L));

    // 16-bit boundary tests (prevent integer sign wrapping / truncation bug)
    TEST_ASSERT_EQUAL_INT32(100L, RobotLogic::clampLong(32767L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(100L, RobotLogic::clampLong(32768L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(-100L, RobotLogic::clampLong(-32768L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(-100L, RobotLogic::clampLong(-32769L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(100L, RobotLogic::clampLong(65536L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(100L, RobotLogic::clampLong(65586L, -100L, 100L));

    // Adversarial extremes for motor speed [-100, 100]
    TEST_ASSERT_EQUAL_INT32(100L, RobotLogic::clampLong(99999L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(-100L, RobotLogic::clampLong(-99999L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(100L, RobotLogic::clampLong(2147483647L, -100L, 100L));
    TEST_ASSERT_EQUAL_INT32(-100L, RobotLogic::clampLong(-2147483647L - 1L, -100L, 100L));

    // Servo angle bounds [0, 180] deg
    TEST_ASSERT_EQUAL_INT32(90L, RobotLogic::clampLong(90L, 0L, 180L));
    TEST_ASSERT_EQUAL_INT32(0L, RobotLogic::clampLong(0L, 0L, 180L));
    TEST_ASSERT_EQUAL_INT32(180L, RobotLogic::clampLong(180L, 0L, 180L));
    TEST_ASSERT_EQUAL_INT32(0L, RobotLogic::clampLong(-1L, 0L, 180L));
    TEST_ASSERT_EQUAL_INT32(180L, RobotLogic::clampLong(181L, 0L, 180L));
    TEST_ASSERT_EQUAL_INT32(180L, RobotLogic::clampLong(99999L, 0L, 180L));
    TEST_ASSERT_EQUAL_INT32(0L, RobotLogic::clampLong(-99999L, 0L, 180L));
    TEST_ASSERT_EQUAL_INT32(180L, RobotLogic::clampLong(2147483647L, 0L, 180L));
    TEST_ASSERT_EQUAL_INT32(0L, RobotLogic::clampLong(-2147483647L - 1L, 0L, 180L));
}

// ============================================================================
// 7. SerialControl Parser & State Machine Tests
// ============================================================================

void test_serial_wasd_hold_expiry_and_stop(void) {
    SerialControl sc;
    SerialActions act{};

    sc.begin(115200UL);

    // Lowercase & uppercase WASD
    NativeSerial::feed("w");
    sc.poll(1000UL, 60, act);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(60, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(60, act.driveCmd.targetRight);

    // Still active at +649 ms, expires cleanly at +650 ms
    sc.poll(1649UL, 60, act);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    sc.poll(1650UL, 60, act);
    TEST_ASSERT_FALSE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(0, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(0, act.driveCmd.targetRight);

    NativeSerial::feed("S");
    sc.poll(2000UL, 55, act);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(-55, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(-55, act.driveCmd.targetRight);

    // Space immediately stops
    NativeSerial::feed(" ");
    sc.poll(2050UL, 55, act);
    TEST_ASSERT_FALSE(act.driveCmd.active);

    NativeSerial::feed("a");
    sc.poll(2100UL, 45, act);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(-45, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(45, act.driveCmd.targetRight);

    // 'X' immediately stops
    NativeSerial::feed("X");
    sc.poll(2150UL, 45, act);
    TEST_ASSERT_FALSE(act.driveCmd.active);

    NativeSerial::feed("D");
    sc.poll(2200UL, 50, act);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(50, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(-50, act.driveCmd.targetRight);

    // External stopMotion() clears latched motion (used by triggerEstop / Cross override)
    sc.stopMotion();
    sc.poll(2220UL, 50, act);
    TEST_ASSERT_FALSE(act.driveCmd.active);
}

void test_serial_direct_motor_bounds_and_garbage_rejection(void) {
    SerialControl sc;
    SerialActions act{};

    // Adversarial clamping on m 99999 -99999
    NativeSerial::feed("m 99999 -99999\n");
    sc.poll(1000UL, 60, act);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(100, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(-100, act.driveCmd.targetRight);

    // Holds until 2500 ms (kSerialDirectHoldMs)
    sc.poll(3499UL, 60, act);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    sc.poll(3500UL, 60, act);
    TEST_ASSERT_FALSE(act.driveCmd.active);

    // Uppercase 'M', comma separator, trailing spaces accepted
    NativeSerial::feed("M -45, 75  \r\n");
    sc.poll(4000UL, 60, act);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(-45, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(75, act.driveCmd.targetRight);
    sc.stopMotion();

    // Trailing garbage or extra tokens must be rejected cleanly
    const char* const badMotorLines[] = {
        "m 50 60abc\n",
        "m 50abc 60\n",
        "m 50 60 70\n",
        "m 50\n",
        "M 10,\n",
    };
    for (size_t i = 0U; i < sizeof(badMotorLines) / sizeof(badMotorLines[0]); ++i) {
        NativeSerial::reset();
        NativeSerial::feed(badMotorLines[i]);
        sc.poll(5000UL + static_cast<uint32_t>(i * 50U), 60, act);
        TEST_ASSERT_FALSE(act.driveCmd.active);
        TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "Unknown line"));
    }
}

void test_serial_servo_commands_and_flip_exact_match(void) {
    SerialControl sc;
    SerialActions act{};

    NativeSerial::feed("c 1 135\n");
    sc.poll(1000UL, 60, act);
    TEST_ASSERT_TRUE(act.hasDirectServo);
    TEST_ASSERT_EQUAL_UINT8(1U, act.directServoCh);
    TEST_ASSERT_EQUAL_INT16(135, act.directServoDeg);

    // Uppercase 'C', adversarial degree clamping
    NativeSerial::feed("C 4, 99999\r\n");
    sc.poll(1050UL, 60, act);
    TEST_ASSERT_TRUE(act.hasDirectServo);
    TEST_ASSERT_EQUAL_UINT8(4U, act.directServoCh);
    TEST_ASSERT_EQUAL_INT16(180, act.directServoDeg);

    NativeSerial::feed("c 2 -50  \n");
    sc.poll(1100UL, 60, act);
    TEST_ASSERT_TRUE(act.hasDirectServo);
    TEST_ASSERT_EQUAL_UINT8(2U, act.directServoCh);
    TEST_ASSERT_EQUAL_INT16(0, act.directServoDeg);

    // Malformed / out-of-range channel / trailing garbage must fail
    const char* const badServoLines[] = {
        "c 1 90xyz\n",
        "c 0 90\n",
        "c 5 90\n",
        "c 2 45 90\n",
    };
    for (size_t i = 0U; i < sizeof(badServoLines) / sizeof(badServoLines[0]); ++i) {
        NativeSerial::feed(badServoLines[i]);
        sc.poll(1200UL + static_cast<uint32_t>(i * 25U), 60, act);
        TEST_ASSERT_FALSE(act.hasDirectServo);
    }

    // Exact f1/f2 (case-insensitive & trailing spaces allowed)
    NativeSerial::feed("f1\n");
    sc.poll(2000UL, 60, act);
    TEST_ASSERT_TRUE(act.toggleLeftReverse);
    TEST_ASSERT_FALSE(act.toggleRightReverse);

    NativeSerial::feed("F2  \r\n");
    sc.poll(2050UL, 60, act);
    TEST_ASSERT_FALSE(act.toggleLeftReverse);
    TEST_ASSERT_TRUE(act.toggleRightReverse);

    // Reject trailing garbage on f1/f2
    NativeSerial::feed("f1foo\n");
    sc.poll(2100UL, 60, act);
    TEST_ASSERT_FALSE(act.toggleLeftReverse);
    TEST_ASSERT_FALSE(act.toggleRightReverse);

    NativeSerial::feed("f2bar\n");
    sc.poll(2150UL, 60, act);
    TEST_ASSERT_FALSE(act.toggleLeftReverse);
    TEST_ASSERT_FALSE(act.toggleRightReverse);

    // "f " must abort dangling line collection AND stop active drive motion
    NativeSerial::feed("wf ");
    sc.poll(2200UL, 60, act);
    TEST_ASSERT_FALSE(sc.isCollectingLine());
    TEST_ASSERT_FALSE(act.driveCmd.active);
}

void test_serial_speed_batching_timeout_and_estop_interrupt(void) {
    SerialControl sc;
    SerialActions act{};

    // Multiple ']' in one poll batch step cumulatively (+15% each)
    NativeSerial::feed("]]");
    sc.poll(1000UL, 60, act);
    TEST_ASSERT_TRUE(act.speedPctChanged);
    TEST_ASSERT_EQUAL_INT16(90, act.newSpeedPct);

    // Multiple '[' in one poll batch clamp at kSerialMinSpeed (15%)
    NativeSerial::feed("[[[[");
    sc.poll(1050UL, 60, act);
    TEST_ASSERT_TRUE(act.speedPctChanged);
    TEST_ASSERT_EQUAL_INT16(Config::kSerialMinSpeed, act.newSpeedPct);

    // Speed change + immediate drive in one batch uses new speed right away
    NativeSerial::feed("]w");
    sc.poll(1100UL, 60, act);
    TEST_ASSERT_TRUE(act.speedPctChanged);
    TEST_ASSERT_EQUAL_INT16(75, act.newSpeedPct);
    TEST_ASSERT_TRUE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(75, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(75, act.driveCmd.targetRight);
    sc.stopMotion();

    // Line timeout (1500 ms): abandoned "m 50" expires cleanly before new single-key command
    NativeSerial::feed("m 50");
    sc.poll(2000UL, 60, act);
    TEST_ASSERT_TRUE(sc.isCollectingLine());
    TEST_ASSERT_EQUAL_UINT8(4U, sc.lineLen());

    NativeSerial::feed("w");
    sc.poll(2000UL + Config::kLineTimeoutMs, 60, act);
    TEST_ASSERT_FALSE(sc.isCollectingLine());
    TEST_ASSERT_TRUE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(60, act.driveCmd.targetLeft);
    sc.stopMotion();

    // '!' interrupts an active line buffer immediately and prevents stale line replay on '\n'
    NativeSerial::feed("m 80 ");
    sc.poll(4000UL, 60, act);
    TEST_ASSERT_TRUE(sc.isCollectingLine());

    NativeSerial::feed("80!\n");
    sc.poll(4020UL, 60, act);
    TEST_ASSERT_TRUE(act.estopRequested);
    TEST_ASSERT_FALSE(sc.isCollectingLine());
    TEST_ASSERT_FALSE(act.driveCmd.active);

    // Single-key actions + telemetry print smoke verification
    NativeSerial::feed("R");
    sc.poll(4100UL, 60, act);
    TEST_ASSERT_TRUE(act.resumeRequested);

    NativeSerial::feed("IKJLOPT");
    sc.poll(4150UL, 60, act);
    TEST_ASSERT_TRUE(act.armUpStep);
    TEST_ASSERT_TRUE(act.armDownStep);
    TEST_ASSERT_TRUE(act.gripOpen);
    TEST_ASSERT_TRUE(act.gripClose);
    TEST_ASSERT_TRUE(act.auxExtend);
    TEST_ASSERT_TRUE(act.auxRetract);
    TEST_ASSERT_TRUE(sc.streamTelemetry());

    SystemSnapshot snap{};
    snap.battVolt = 12.34f;
    snap.extPower = true;
    snap.oledAddr = 0x3DU;
    snap.oledOk   = true;
    NativeSerial::reset();
    sc.printSnapshot(snap);
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "[STAT] READY"));
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "oled=0x3D(ok=1"));
}

// ============================================================================
// Main Unity Test Runner
// ============================================================================

int main(int argc, char** argv) {
    (void)argc;
    (void)argv;
    UNITY_BEGIN();

    // 1. Differential Drive Kinematics & Desaturation
    RUN_TEST(test_axis_deadzone_and_extremes);
    RUN_TEST(test_differential_drive_zero_speed_boundary);
    RUN_TEST(test_differential_drive_saturation_and_ratio_preservation);
    RUN_TEST(test_differential_drive_pivot_turn);

    // 2. Asymmetric Slew & Zero Crossing
    RUN_TEST(test_asymmetric_slew_limits_and_step_delta);
    RUN_TEST(test_asymmetric_slew_zero_crossing_trajectory);

    // 3. Failsafe State Transitions & PS2 Validation
    RUN_TEST(test_ps2_decode_and_5stage_validation);
    RUN_TEST(test_failsafe_burst_glitch_absorption_and_trip);
    RUN_TEST(test_failsafe_persistent_loss_disconnect_and_reinit_cooldown);
    RUN_TEST(test_servo_bounds_and_ps2_presets);

    // 4. Battery Sag & Hysteresis
    RUN_TEST(test_battery_stabilizer_multiprofile_baselines);
    RUN_TEST(test_battery_stabilizer_progressive_sag_tiers);
    RUN_TEST(test_battery_stabilizer_hysteresis_and_timed_recovery);
    RUN_TEST(test_battery_stabilizer_regen_spike_protection);

    // 5. Stability Monitor
    RUN_TEST(test_stability_monitor_individual_penalties);
    RUN_TEST(test_stability_monitor_saturation_and_deterministic_recovery);

    // 6. Adversarial Bounds Clamping
    RUN_TEST(test_clamp_long_and_adversarial_boundaries);

    // 7. SerialControl Parser & State Machine
    RUN_TEST(test_serial_wasd_hold_expiry_and_stop);
    RUN_TEST(test_serial_direct_motor_bounds_and_garbage_rejection);
    RUN_TEST(test_serial_servo_commands_and_flip_exact_match);
    RUN_TEST(test_serial_speed_batching_timeout_and_estop_interrupt);

    return UNITY_END();
}
