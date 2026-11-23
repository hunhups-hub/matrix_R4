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

    // Single valid frame absorbs glitch burst without dropping out of Operational
    sm.onFrame(true, 275UL);
    TEST_ASSERT_EQUAL_UINT16(0U, sm.badFrames());
    TEST_ASSERT_FALSE(sm.isLinkGlitching());
    TEST_ASSERT_TRUE(sm.isOperational());

    // Continued valid frames remain Operational
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
    TEST_ASSERT_EQUAL_INT16(Config::kRc2MaxDeg, clamped.rc2Deg);
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

    // R2 retracts RC1 to kRc1R2Deg (0 deg) and clears any active Circle pose preset
    ArmPosePresetState poseState{};
    poseState.dropActive = true;
    Ps2Frame fR2 = makeValidFrame();
    fR2.r2 = true;
    RobotLogic::updateServoTargetsFromPs2(fR2, servos, false, nullptr, &poseState);
    TEST_ASSERT_EQUAL_INT16(Config::kRc1R2Deg, servos.rc1Deg);
    TEST_ASSERT_FALSE(poseState.dropActive);
}

void test_square_push_ground_toggle_and_l1_r1_override(void) {
    TEST_ASSERT_EQUAL_INT16(180, Config::kRc3SquareDeg);
    TEST_ASSERT_EQUAL_INT16(180, Config::kRc4SquareDeg);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(Config::kServoMinDeg, Config::kRc3SquareDeg);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kServoMaxDeg, Config::kRc3SquareDeg);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(Config::kServoMinDeg, Config::kRc4SquareDeg);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kServoMaxDeg, Config::kRc4SquareDeg);

    ServoTargets      servos{};
    FrontArmPushState push{};

    // Raw PS2 packet for Square pressed (buttons2 bit 7 active-low = 0x7F)
    uint8_t rawSq[9] = {0xFFU, 0x73U, 0x5AU, 0xFFU, 0x7FU, 128U, 128U, 128U, 128U};
    Ps2Frame fSquare{};
    RobotLogic::decodePs2Payload(rawSq, fSquare);
    TEST_ASSERT_TRUE(RobotLogic::isPs2FrameValid(fSquare));
    TEST_ASSERT_TRUE(fSquare.square);

    Ps2Frame idle = makeValidFrame();

    // 1. From boot init (30, 30): 1st Square press activates push-ground (30, 30)
    RobotLogic::updateServoTargetsFromPs2(fSquare, servos, true, &push);
    TEST_ASSERT_TRUE(push.active);
    TEST_ASSERT_TRUE(push.prevSquare);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3InitDeg, push.savedRc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4InitDeg, push.savedRc4Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3SquareDeg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4SquareDeg, servos.rc4Deg);

    // Holding Square across additional ticks must NOT re-toggle
    RobotLogic::updateServoTargetsFromPs2(fSquare, servos, true, &push);
    TEST_ASSERT_TRUE(push.active);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3SquareDeg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4SquareDeg, servos.rc4Deg);

    // Releasing Square latches (30, 30)
    RobotLogic::updateServoTargetsFromPs2(idle, servos, true, &push);
    TEST_ASSERT_TRUE(push.active);
    TEST_ASSERT_FALSE(push.prevSquare);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3SquareDeg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4SquareDeg, servos.rc4Deg);

    // 2. 2nd Square press cancels and restores previous (30, 30)
    RobotLogic::updateServoTargetsFromPs2(fSquare, servos, true, &push);
    TEST_ASSERT_FALSE(push.active);
    TEST_ASSERT_TRUE(push.prevSquare);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3InitDeg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4InitDeg, servos.rc4Deg);

    RobotLogic::updateServoTargetsFromPs2(idle, servos, true, &push);
    TEST_ASSERT_FALSE(push.prevSquare);

    // 3. From L1 (30, 30): 1st Square -> (30, 30), 2nd Square -> (30, 30)
    Ps2Frame fL1 = makeValidFrame();
    fL1.l1 = true;
    RobotLogic::updateServoTargetsFromPs2(fL1, servos, true, &push);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3L1Deg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4L1Deg, servos.rc4Deg);

    RobotLogic::updateServoTargetsFromPs2(fSquare, servos, true, &push);
    TEST_ASSERT_TRUE(push.active);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3SquareDeg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4SquareDeg, servos.rc4Deg);

    RobotLogic::updateServoTargetsFromPs2(idle, servos, true, &push);
    RobotLogic::updateServoTargetsFromPs2(fSquare, servos, true, &push);
    TEST_ASSERT_FALSE(push.active);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3L1Deg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4L1Deg, servos.rc4Deg);
    RobotLogic::updateServoTargetsFromPs2(idle, servos, true, &push);

    // 4. R1 override while push is active clears active state cleanly
    Ps2Frame fR1 = makeValidFrame();
    fR1.r1 = true;
    RobotLogic::updateServoTargetsFromPs2(fSquare, servos, true, &push);
    TEST_ASSERT_TRUE(push.active);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3SquareDeg, servos.rc3Deg);

    RobotLogic::updateServoTargetsFromPs2(fR1, servos, true, &push);
    TEST_ASSERT_FALSE(push.active);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3R1Deg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4R1Deg, servos.rc4Deg);

    // Next Square press is a fresh 1st press: saves R1 (140, 140) and goes to (30, 30)
    RobotLogic::updateServoTargetsFromPs2(fSquare, servos, true, &push);
    TEST_ASSERT_TRUE(push.active);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3R1Deg, push.savedRc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4R1Deg, push.savedRc4Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3SquareDeg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4SquareDeg, servos.rc4Deg);
    RobotLogic::updateServoTargetsFromPs2(idle, servos, true, &push);

    // 5. Simultaneous L1 + Square: L1 wins, active cleared, and releasing L1 while holding Square does not re-trigger
    Ps2Frame fBoth = fL1;
    fBoth.square = true;
    RobotLogic::updateServoTargetsFromPs2(fBoth, servos, true, &push);
    TEST_ASSERT_FALSE(push.active);
    TEST_ASSERT_TRUE(push.prevSquare);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3L1Deg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4L1Deg, servos.rc4Deg);

    RobotLogic::updateServoTargetsFromPs2(fSquare, servos, true, &push);
    TEST_ASSERT_FALSE(push.active);
    TEST_ASSERT_EQUAL_INT16(Config::kRc3L1Deg, servos.rc3Deg);
    TEST_ASSERT_EQUAL_INT16(Config::kRc4L1Deg, servos.rc4Deg);
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

    // 7. Auto pack detection never reads a drained pack as full:
    //    3S booted at 10.0 V -> 3S range (7%), not 9V-adapter 100%;
    //    4S booted at 13.0 V -> 4S range (0%), not full 3S 100%.
    batt.reset();
    batt.update(10000U, true, 0UL);
    TEST_ASSERT_EQUAL_UINT8(7U, batt.batteryPercent());
    batt.reset();
    batt.update(13000U, true, 0UL);
    TEST_ASSERT_EQUAL_UINT8(0U, batt.batteryPercent());
    uint16_t emptyMv = 0U;
    uint16_t fullMv  = 0U;
    RobotLogic::classifyBatteryPack(9300U, emptyMv, fullMv); // 9V adapter / NiMH band unchanged
    TEST_ASSERT_EQUAL_UINT16(7000U, emptyMv);
    TEST_ASSERT_EQUAL_UINT16(9300U, fullMv);
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
    TEST_ASSERT_EQUAL_UINT8(Config::kEnableBatterySagThrottle ? 110U : Config::kBattCompMaxPct, prof.compPercent);
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
    TEST_ASSERT_EQUAL_UINT8(Config::kEnableBatterySagThrottle ? 100U : Config::kBattCompMaxPct, prof.compPercent);
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
    // Emergency tier keeps torque compensation when sag throttle is off (full strength on weak battery)
    TEST_ASSERT_EQUAL_UINT8(Config::kEnableBatterySagThrottle ? 100U : Config::kBattCompMaxPct, prof.compPercent);
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

    // Disconnected (after ever connected) = 25% when idle, but 0% during Autonomous mode
    in.ps2Link = LinkState::Disconnected;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityLinkDropPen, stb.linkPenalty());

    in.activeSource = ControlSource::Auto;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(0U, stb.linkPenalty());

    // Disconnected before ever connected:
    stb.reset();
    in.ps2Link      = LinkState::Disconnected;
    in.activeSource = ControlSource::None;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityIdleWaitPen, stb.linkPenalty()); // 5%

    in.activeSource = ControlSource::SerialCmd;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilitySerialLinkPen, stb.linkPenalty()); // 2%

    in.activeSource = ControlSource::Auto;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(0U, stb.linkPenalty()); // 0% during autonomous run without PS2

    // Operational link glitch penalty (+8% per bad frame up to 35%)
    stb.reset();
    in.activeSource = ControlSource::Ps2;
    in.ps2Link      = LinkState::Operational;
    in.frameValid   = true;
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

    // 5. Drive risk penalty (D): arm low or stopped = 0; arm high + full power = tip max;
    //    half way scales; stall adds a fixed penalty; total capped.
    TEST_ASSERT_EQUAL_UINT8(0U, RobotLogic::computeDriveStabilityPenalty(100U, 100, 100, false));
    TEST_ASSERT_EQUAL_UINT8(0U, RobotLogic::computeDriveStabilityPenalty(Config::kCoMHighMinScalePct, 0, 0, false));
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityTipMaxPen,
                            RobotLogic::computeDriveStabilityPenalty(Config::kCoMHighMinScalePct, 100, -100, false));
    const uint8_t halfTip = RobotLogic::computeDriveStabilityPenalty(Config::kCoMHighMinScalePct, 50, 50, false);
    TEST_ASSERT_TRUE(halfTip > 0U && halfTip < Config::kStabilityTipMaxPen);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityStallPen, RobotLogic::computeDriveStabilityPenalty(100U, 80, 80, true));
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityDriveMaxPen,
                            RobotLogic::computeDriveStabilityPenalty(Config::kCoMHighMinScalePct, 100, 100, true));

    stb.reset();
    in.loopDtMs    = Config::kControlPeriodMs;
    in.comScalePct = Config::kCoMHighMinScalePct;
    in.driveLeft   = 100;
    in.driveRight  = 100;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(Config::kStabilityTipMaxPen, stb.drivePenalty());
    in.comScalePct = 100U;
    stb.update(in);
    TEST_ASSERT_EQUAL_UINT8(0U, stb.drivePenalty());
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

    // Multiple '[' in one poll batch clamp at kSerialMinSpeed (20%)
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

    snap.activeSource = ControlSource::Auto;
    NativeSerial::reset();
    sc.printSnapshot(snap);
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "src=AUTO"));
}

void test_auto_mode_triangle_start_and_cross_exit(void) {
    // Starts ONLY on a rising Triangle press while Ready, PS2 Operational, valid frame, and X clear.
    TEST_ASSERT_TRUE(RobotLogic::shouldStartAutoMode(true, true, true, true, false, false));

    // Must NOT start if held from previous tick (edge trigger), ESTOP, disconnected/sync, invalid, or X held.
    TEST_ASSERT_FALSE(RobotLogic::shouldStartAutoMode(true, true, true, true, true, false));
    TEST_ASSERT_FALSE(RobotLogic::shouldStartAutoMode(false, true, true, true, false, false));
    TEST_ASSERT_FALSE(RobotLogic::shouldStartAutoMode(true, false, true, true, false, false));
    TEST_ASSERT_FALSE(RobotLogic::shouldStartAutoMode(true, true, false, true, false, false));
    TEST_ASSERT_FALSE(RobotLogic::shouldStartAutoMode(true, true, true, false, false, false));
    TEST_ASSERT_FALSE(RobotLogic::shouldStartAutoMode(true, true, true, true, false, true));

    // Auto exit aborts ONLY on a valid frame with Cross (X) pressed.
    TEST_ASSERT_TRUE(RobotLogic::shouldAbortAutoMode(true, true));
    TEST_ASSERT_FALSE(RobotLogic::shouldAbortAutoMode(false, true));
    TEST_ASSERT_FALSE(RobotLogic::shouldAbortAutoMode(true, false));

    // End-to-end PS2 active-low raw packet decoding for Triangle (bit 4 = 0xEF) & Cross (bit 6 = 0xBF).
    uint8_t rawTri[9] = {0xFF, 0x73, 0x5A, 0xFF, 0xEF, 0x80, 0x80, 0x80, 0x80};
    Ps2Frame fTri{};
    RobotLogic::decodePs2Payload(rawTri, fTri);
    TEST_ASSERT_TRUE(RobotLogic::isPs2FrameValid(fTri));
    TEST_ASSERT_TRUE(fTri.triangle);
    TEST_ASSERT_FALSE(fTri.cross);
    TEST_ASSERT_TRUE(RobotLogic::shouldStartAutoMode(true, true, true, fTri.triangle, false, fTri.cross));
    TEST_ASSERT_FALSE(RobotLogic::shouldAbortAutoMode(true, fTri.cross));

    uint8_t rawCross[9] = {0xFF, 0x73, 0x5A, 0xFF, 0xBF, 0x80, 0x80, 0x80, 0x80};
    Ps2Frame fCross{};
    RobotLogic::decodePs2Payload(rawCross, fCross);
    TEST_ASSERT_TRUE(RobotLogic::isPs2FrameValid(fCross));
    TEST_ASSERT_TRUE(fCross.cross);
    TEST_ASSERT_FALSE(fCross.triangle);
    TEST_ASSERT_TRUE(RobotLogic::shouldAbortAutoMode(true, fCross.cross));

    // Simultaneous Triangle + Cross: Cross wins (never enter Auto).
    uint8_t rawBoth[9] = {0xFF, 0x73, 0x5A, 0xFF, 0xAF, 0x80, 0x80, 0x80, 0x80};
    Ps2Frame fBoth{};
    RobotLogic::decodePs2Payload(rawBoth, fBoth);
    TEST_ASSERT_TRUE(RobotLogic::isPs2FrameValid(fBoth));
    TEST_ASSERT_FALSE(RobotLogic::shouldStartAutoMode(true, true, true, fBoth.triangle, false, fBoth.cross));
    TEST_ASSERT_TRUE(RobotLogic::shouldAbortAutoMode(true, fBoth.cross));
}

// ============================================================================
// 9. Physics Kinodynamic & Electromechanical Models + Competition Controls
// ============================================================================

void test_cubic_expo_shaping_symmetry_and_bounds(void) {
    // Zero and full +/-100% fixed points for any expo percentage
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyCubicExpo(0, 0U));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyCubicExpo(0, 50U));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyCubicExpo(0, 100U));
    TEST_ASSERT_EQUAL_INT16(100, RobotLogic::applyCubicExpo(100, 35U));
    TEST_ASSERT_EQUAL_INT16(-100, RobotLogic::applyCubicExpo(-100, 35U));
    TEST_ASSERT_EQUAL_INT16(100, RobotLogic::applyCubicExpo(100, 100U));
    TEST_ASSERT_EQUAL_INT16(-100, RobotLogic::applyCubicExpo(-100, 100U));

    // 0% expo is pure linear identity
    TEST_ASSERT_EQUAL_INT16(42, RobotLogic::applyCubicExpo(42, 0U));
    TEST_ASSERT_EQUAL_INT16(-42, RobotLogic::applyCubicExpo(-42, 0U));

    // At 40% expo and x = 50: 0.6*50 + 0.4*(125000/10000) = 30 + 5 = 35 (< 50 -> softer center)
    TEST_ASSERT_EQUAL_INT16(35, RobotLogic::applyCubicExpo(50, 40U));
    TEST_ASSERT_EQUAL_INT16(-35, RobotLogic::applyCubicExpo(-50, 40U));

    // Strict odd symmetry and monotonicity across [-100, 100]
    int16_t prev = -100;
    for (int16_t x = -100; x <= 100; ++x) {
        const int16_t yPos = RobotLogic::applyCubicExpo(x, Config::kThrottleExpoPct);
        const int16_t yNeg = RobotLogic::applyCubicExpo(static_cast<int16_t>(-x), Config::kThrottleExpoPct);
        TEST_ASSERT_EQUAL_INT16(yPos, static_cast<int16_t>(-yNeg));
        TEST_ASSERT_GREATER_OR_EQUAL_INT16(prev, yPos);
        prev = yPos;
    }
}

void test_centripetal_traction_limiter_and_pivot_authority(void) {
    // Standstill (v = 0) preserves 100% pivot turn authority
    TEST_ASSERT_EQUAL_INT16(100, RobotLogic::limitCentripetalTurn(0, 100, 50));
    TEST_ASSERT_EQUAL_INT16(-100, RobotLogic::limitCentripetalTurn(0, -100, 50));

    // Low speed within traction circle (|v * w| / 100 = 40 * 80 / 100 = 32 <= 50) -> unthrottled
    TEST_ASSERT_EQUAL_INT16(80, RobotLogic::limitCentripetalTurn(40, 80, 50));
    TEST_ASSERT_EQUAL_INT16(-80, RobotLogic::limitCentripetalTurn(-40, -80, 50));

    // High speed exceeding traction circle (v = 100, w = 80 -> a_c = 80 > 50) -> capped to 50
    TEST_ASSERT_EQUAL_INT16(50, RobotLogic::limitCentripetalTurn(100, 80, 50));
    TEST_ASSERT_EQUAL_INT16(-50, RobotLogic::limitCentripetalTurn(100, -80, 50));
    TEST_ASSERT_EQUAL_INT16(62, RobotLogic::limitCentripetalTurn(80, 90, 50)); // 5000 / 80 = 62

    // Zero or negative maxLatAccel safely zeroes turn
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::limitCentripetalTurn(50, 50, 0));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::limitCentripetalTurn(50, 50, -10));
}

void test_com_elevation_dynamic_scaling(void) {
    // Lowered arm (140..180 deg) has lowest CoM -> 100% full dynamic scale
    TEST_ASSERT_EQUAL_UINT8(100U, RobotLogic::computeCoMDynamicScale(180));
    TEST_ASSERT_EQUAL_UINT8(100U, RobotLogic::computeCoMDynamicScale(140));

    // Fully raised arm (0 deg) has highest CoM -> kCoMHighMinScalePct (65%)
    TEST_ASSERT_EQUAL_UINT8(Config::kCoMHighMinScalePct, RobotLogic::computeCoMDynamicScale(0));

    // Mid-height arm (70 deg) scales halfway between 65% and 100% (~83%)
    const uint8_t midScale = RobotLogic::computeCoMDynamicScale(70);
    TEST_ASSERT_GREATER_THAN_UINT8(Config::kCoMHighMinScalePct, midScale);
    TEST_ASSERT_LESS_THAN_UINT8(100U, midScale);
}

void test_2dof_kinodynamic_slew_and_coulomb_stiction(void) {
    // Coulomb stiction feedforward: 0 stays 0, +/-1 jumps immediately to +/-12%, +/-100 stays +/-100%
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyCoulombStiction(0, 12, 100));
    TEST_ASSERT_EQUAL_INT16(12, RobotLogic::applyCoulombStiction(1, 12, 100));
    TEST_ASSERT_EQUAL_INT16(-12, RobotLogic::applyCoulombStiction(-1, 12, 100));
    TEST_ASSERT_EQUAL_INT16(100, RobotLogic::applyCoulombStiction(100, 12, 100));
    TEST_ASSERT_EQUAL_INT16(-100, RobotLogic::applyCoulombStiction(-100, 12, 100));
    TEST_ASSERT_EQUAL_INT16(55, RobotLogic::applyCoulombStiction(55, 12, 55));

    // Stiction pair keeps the L/R ratio: old per-wheel lift turned (13, 2) into (23, 13) = 1.8:1
    int16_t pl = 13;
    int16_t pr = 2;
    RobotLogic::applyCoulombStictionPair(pl, pr, 12, 100);
    TEST_ASSERT_EQUAL_INT16(23, pl);
    TEST_ASSERT_EQUAL_INT16(4, pr);
    pl = -13;
    pr = 2;
    RobotLogic::applyCoulombStictionPair(pl, pr, 12, 100);
    TEST_ASSERT_EQUAL_INT16(-23, pl);
    TEST_ASSERT_EQUAL_INT16(4, pr);
    // Equal-magnitude pivot matches the single-wheel mapping; a wheel at 0 stays 0; (0,0) stays (0,0)
    pl = 30;
    pr = -30;
    RobotLogic::applyCoulombStictionPair(pl, pr, 12, 100);
    TEST_ASSERT_EQUAL_INT16(RobotLogic::applyCoulombStiction(30, 12, 100), pl);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-RobotLogic::applyCoulombStiction(30, 12, 100)), pr);
    pl = 20;
    pr = 0;
    RobotLogic::applyCoulombStictionPair(pl, pr, 12, 100);
    TEST_ASSERT_EQUAL_INT16(RobotLogic::applyCoulombStiction(20, 12, 100), pl);
    TEST_ASSERT_EQUAL_INT16(0, pr);
    pl = 0;
    pr = 0;
    RobotLogic::applyCoulombStictionPair(pl, pr, 12, 100);
    TEST_ASSERT_EQUAL_INT16(0, pl);
    TEST_ASSERT_EQUAL_INT16(0, pr);

    // Arm raised (RC2=0): pivot angular slew is scaled by CoM scale, same as linear slew
    const DriveOutput pivotLow  = RobotLogic::computeKinodynamicDrive(
        128U, 255U, 0, 0, 100, 100, 100, 100U, 140, false, false, false);
    const DriveOutput pivotHigh = RobotLogic::computeKinodynamicDrive(
        128U, 255U, 0, 0, 100, 100, 100, 100U, 0, false, false, false);
    const int16_t expectHigh = static_cast<int16_t>(
        (static_cast<int32_t>(Config::kAngularAccelSlewStep) * Config::kCoMHighMinScalePct + 50L) / 100L);
    TEST_ASSERT_EQUAL_INT16(Config::kAngularAccelSlewStep, pivotLow.left);
    TEST_ASSERT_EQUAL_INT16(expectHigh, pivotHigh.left);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-expectHigh), pivotHigh.right);

    // 2-DOF Slew: pure straight acceleration (linAccel=25, angAccel=35) steps both wheels by 25
    const DriveOutput fwdStep = RobotLogic::apply2DofKinodynamicSlew(100, 100, 0, 0, 25, 50, 35, 60);
    TEST_ASSERT_EQUAL_INT16(25, fwdStep.left);
    TEST_ASSERT_EQUAL_INT16(25, fwdStep.right);

    // 2-DOF Slew: pure pivot turn (v=0, w=+100) uses rotational slew step (angAccel=35)
    const DriveOutput rotStep = RobotLogic::apply2DofKinodynamicSlew(100, -100, 0, 0, 25, 50, 35, 60);
    TEST_ASSERT_EQUAL_INT16(35, rotStep.left);
    TEST_ASSERT_EQUAL_INT16(-35, rotStep.right);

    // 2-DOF Slew: turning while holding constant forward velocity v=60 (from 60,60 to 90,30 -> v=60, w=30)
    // preserves exact forward speed v=60 while slewing angular rate w!
    const DriveOutput curveStep = RobotLogic::apply2DofKinodynamicSlew(90, 30, 60, 60, 20, 40, 15, 30);
    TEST_ASSERT_EQUAL_INT16(60, static_cast<int16_t>((curveStep.left + curveStep.right) / 2));
    TEST_ASSERT_EQUAL_INT16(75, curveStep.left);
    TEST_ASSERT_EQUAL_INT16(45, curveStep.right);
}

void test_kinodynamic_drive_reverse_front_and_dpad_micro_turn(void) {
    // Normal front vs Reverse front (R3 toggle): pushing stick UP (lyRaw=0) drives forward (+,+),
    // while reverseDrive=true drives backward (-,-) so rear gripper acts as the front!
    const DriveOutput normFwd = RobotLogic::computeKinodynamicDrive(
        0U, 128U, 0, 0, 100, 100, 100, 100U, 140, false, false, false);
    const DriveOutput revFwd = RobotLogic::computeKinodynamicDrive(
        0U, 128U, 0, 0, 100, 100, 100, 100U, 140, true, false, false);
    TEST_ASSERT_EQUAL_INT16(100, normFwd.left);
    TEST_ASSERT_EQUAL_INT16(100, normFwd.right);
    TEST_ASSERT_EQUAL_INT16(-100, revFwd.left);
    TEST_ASSERT_EQUAL_INT16(-100, revFwd.right);

    // D-Pad Left/Right micro-turn alignment when stick is centered (lyRaw=128, rxRaw=128)
    const DriveOutput nudgeRight = RobotLogic::computeKinodynamicDrive(
        128U, 128U, 0, 0, 100, 100, 100, 100U, 140, false, false, true);
    const DriveOutput nudgeLeft = RobotLogic::computeKinodynamicDrive(
        128U, 128U, 0, 0, 100, 100, 100, 100U, 140, false, true, false);
    TEST_ASSERT_EQUAL_INT16(Config::kMicroTurnPower, nudgeRight.left);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-Config::kMicroTurnPower), nudgeRight.right);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-Config::kMicroTurnPower), nudgeLeft.left);
    TEST_ASSERT_EQUAL_INT16(Config::kMicroTurnPower, nudgeLeft.right);

    // Combined forward throttle + right steering in Normal mode applies kTurnExpoPct (45%)
    // and kTurnMaxRatioPct (80%) so outer wheel leads inner wheel smoothly.
    const DriveOutput arcRight = RobotLogic::computeKinodynamicDrive(
        64U, 192U, 50, 50, 100, 100, 100, 100U, 140, false, false, false);
    TEST_ASSERT_GREATER_THAN_INT16(arcRight.right, arcRight.left);
    TEST_ASSERT_GREATER_THAN_INT16(0, arcRight.right);
}

void test_precision_mode_full_stick_scaling_and_micro_turn(void) {
    // 1. Full stick throw (lyRaw=0) in Precision Mode scales smoothly to kPrecisionMaxPower (55%)
    const DriveOutput precFull = RobotLogic::computeKinodynamicDrive(
        0U, 128U, 0, 0, Config::kMotorMaxPower, 100, 100, 100U, 140, false, false, false, 0, true);
    TEST_ASSERT_EQUAL_INT16(Config::kPrecisionMaxPower, precFull.left);
    TEST_ASSERT_EQUAL_INT16(Config::kPrecisionMaxPower, precFull.right);

    // 2. Full-range proportional scaling: outer stick travel remains responsive (50% < 80% < 100%),
    //    avoiding early flat-top saturation, and mid-stick is gentler than Normal mode.
    const DriveOutput normMid = RobotLogic::computeKinodynamicDrive(
        64U, 128U, 0, 0, Config::kMotorMaxPower, 100, 100, 100U, 140, false, false, false, 0, false);
    const DriveOutput precMid = RobotLogic::computeKinodynamicDrive(
        64U, 128U, 0, 0, Config::kMotorMaxPower, 100, 100, 100U, 140, false, false, false, 0, true);
    const DriveOutput precHigh = RobotLogic::computeKinodynamicDrive(
        22U, 128U, 0, 0, Config::kMotorMaxPower, 100, 100, 100U, 140, false, false, false, 0, true);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(Config::kMotorStictionMinPower, precMid.left);
    TEST_ASSERT_LESS_THAN_INT16(normMid.left, precMid.left);
    TEST_ASSERT_GREATER_THAN_INT16(precMid.left, precHigh.left);
    TEST_ASSERT_GREATER_THAN_INT16(precHigh.left, precFull.left);

    // 3. Pure pivot turn at full stick (rxRaw=255) steps by angular slew (35) then reaches +/-55%
    const DriveOutput precPivot1 = RobotLogic::computeKinodynamicDrive(
        128U, 255U, 0, 0, Config::kMotorMaxPower, 100, 100, 100U, 140, false, false, false, 0, true);
    TEST_ASSERT_EQUAL_INT16(Config::kAngularAccelSlewStep, precPivot1.left);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-Config::kAngularAccelSlewStep), precPivot1.right);
    const DriveOutput precPivot2 = RobotLogic::computeKinodynamicDrive(
        128U, 255U, precPivot1.left, precPivot1.right, Config::kMotorMaxPower, 100, 100, 100U, 140, false, false, false, 0, true);
    TEST_ASSERT_EQUAL_INT16(Config::kPrecisionMaxPower, precPivot2.left);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-Config::kPrecisionMaxPower), precPivot2.right);

    // 4. Precision Micro-Turn (D-Pad Left/Right) pivots at exact +/-18% (finer than 26% normal)
    const DriveOutput precNudgeR = RobotLogic::computeKinodynamicDrive(
        128U, 128U, 0, 0, Config::kMotorMaxPower, 100, 100, 100U, 140, false, false, true, 0, true);
    const DriveOutput precNudgeL = RobotLogic::computeKinodynamicDrive(
        128U, 128U, 0, 0, Config::kMotorMaxPower, 100, 100, 100U, 140, false, true, false, 0, true);
    TEST_ASSERT_EQUAL_INT16(Config::kPrecisionMicroTurnPower, precNudgeR.left);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-Config::kPrecisionMicroTurnPower), precNudgeR.right);
    TEST_ASSERT_EQUAL_INT16(static_cast<int16_t>(-Config::kPrecisionMicroTurnPower), precNudgeL.left);
    TEST_ASSERT_EQUAL_INT16(Config::kPrecisionMicroTurnPower, precNudgeL.right);

    // 5. Lower battery sag cap (e.g. 35%) overrides Precision Mode's 55% ceiling safely
    const DriveOutput sagCapped = RobotLogic::computeKinodynamicDrive(
        0U, 128U, 0, 0, 35, 100, 100, 100U, 140, false, false, false, 0, true);
    TEST_ASSERT_EQUAL_INT16(35, sagCapped.left);
    TEST_ASSERT_EQUAL_INT16(35, sagCapped.right);

    // 6. Telemetry snapshot reports prec=1 when active
    SystemSnapshot snap{};
    snap.precisionMode = true;
    SerialControl sc;
    NativeSerial::reset();
    sc.printSnapshot(snap);
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "prec=1"));
}

void test_triangle_sets_rc2_140_and_rc1_100(void) {
    ServoTargets       servos{};
    FrontArmPushState  pushState{};
    ArmPosePresetState poseState{};

    Ps2Frame fTri = makeValidFrame();
    fTri.triangle = true;
    RobotLogic::updateServoTargetsFromPs2(fTri, servos, false, &pushState, &poseState);
    TEST_ASSERT_EQUAL_INT16(100, servos.rc1Deg);
    TEST_ASSERT_EQUAL_INT16(140, servos.rc2Deg);

    // Holding Triangle does not override a later R2 gripper command
    Ps2Frame fR2 = fTri;
    fR2.r2 = true;
    RobotLogic::updateServoTargetsFromPs2(fR2, servos, false, &pushState, &poseState);
    TEST_ASSERT_EQUAL_INT16(Config::kRc1R2Deg, servos.rc1Deg);
}

void test_circle_pose_preset_and_serial_competition_shortcuts(void) {
    ServoTargets      servos{};
    FrontArmPushState pushState{};
    ArmPosePresetState poseState{};

    // First Circle rising edge -> RC2 moves now, RC1 release is delayed
    servos.rc1Deg    = 0;
    Ps2Frame fCircle = makeValidFrame();
    fCircle.circle   = true;
    RobotLogic::updateServoTargetsFromPs2(fCircle, servos, false, &pushState, &poseState, 1000UL);
    TEST_ASSERT_TRUE(poseState.dropActive);
    TEST_ASSERT_EQUAL_INT16(Config::kRc2PresetDropDeg, servos.rc2Deg);
    TEST_ASSERT_EQUAL_INT16(0, servos.rc1Deg);

    // Holding Circle: still waiting, then RC1 released after the delay
    RobotLogic::updateServoTargetsFromPs2(fCircle, servos, false, &pushState, &poseState,
                                          1000UL + Config::kPoseDropReleaseDelayMs - 1UL);
    TEST_ASSERT_EQUAL_INT16(0, servos.rc1Deg);
    RobotLogic::updateServoTargetsFromPs2(fCircle, servos, false, &pushState, &poseState,
                                          1000UL + Config::kPoseDropReleaseDelayMs);
    TEST_ASSERT_TRUE(poseState.dropActive);
    TEST_ASSERT_EQUAL_INT16(Config::kRc1PresetDropDeg, servos.rc1Deg);

    // Only Cross cancels a pending release; gripper button does not
    Ps2Frame fIdle = makeValidFrame();
    RobotLogic::updateServoTargetsFromPs2(fIdle, servos, false, &pushState, &poseState, 3000UL);
    servos.rc1Deg = 0;
    servos.rc2Deg = 0;
    RobotLogic::updateServoTargetsFromPs2(fCircle, servos, false, &pushState, &poseState, 4000UL);
    Ps2Frame fR2c = fCircle;
    fR2c.r2       = true;
    RobotLogic::updateServoTargetsFromPs2(fR2c, servos, false, &pushState, &poseState, 4100UL);
    TEST_ASSERT_EQUAL_INT16(Config::kRc1R2Deg, servos.rc1Deg);
    RobotLogic::updateServoTargetsFromPs2(fCircle, servos, false, &pushState, &poseState, 9000UL);
    TEST_ASSERT_EQUAL_INT16(Config::kRc1PresetDropDeg, servos.rc1Deg);

    // Cross cancels the pending release
    RobotLogic::updateServoTargetsFromPs2(fIdle, servos, false, &pushState, &poseState, 10000UL);
    servos.rc1Deg = 0;
    RobotLogic::updateServoTargetsFromPs2(fCircle, servos, false, &pushState, &poseState, 11000UL);
    Ps2Frame fX = fCircle;
    fX.cross    = true;
    RobotLogic::updateServoTargetsFromPs2(fX, servos, false, &pushState, &poseState, 11100UL);
    RobotLogic::updateServoTargetsFromPs2(fCircle, servos, false, &pushState, &poseState, 20000UL);
    TEST_ASSERT_EQUAL_INT16(0, servos.rc1Deg);

    // Verify new Serial competition shortcuts: 'g' (Auto), 'z' (Precision), 'b' (Reverse), 'q' (Pose)
    SerialControl sc;
    SerialActions act{};
    NativeSerial::feed("gzbq");
    sc.poll(5000UL, 65, act);
    TEST_ASSERT_TRUE(act.autoStartRequested);
    TEST_ASSERT_TRUE(act.togglePrecision);
    TEST_ASSERT_TRUE(act.toggleReverseDrive);
    TEST_ASSERT_TRUE(act.togglePosePreset);
}

void test_rc2_smooth_servo_slew(void) {
    // Slewing from 70 deg (Hold preset) toward 140 deg (Drop preset) at 6 deg/tick
    int16_t cur = 70;
    cur = RobotLogic::slewServoAngle(cur, 140, 6);
    TEST_ASSERT_EQUAL_INT16(76, cur);

    for (int i = 0; i < 11; ++i) {
        cur = RobotLogic::slewServoAngle(cur, 140, 6);
    }
    TEST_ASSERT_EQUAL_INT16(140, cur);

    // Slewing downward from 140 deg back to 70 deg
    cur = RobotLogic::slewServoAngle(140, 70, 6);
    TEST_ASSERT_EQUAL_INT16(134, cur);

    // Clamping to [0, 180] and instant jump when maxStepDeg <= 0
    TEST_ASSERT_EQUAL_INT16(Config::kRc2MaxDeg, RobotLogic::slewServoAngle(Config::kRc2MaxDeg - 2, 220, 6));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::slewServoAngle(3, -20, 6));
    TEST_ASSERT_EQUAL_INT16(140, RobotLogic::slewServoAngle(70, 140, 0));
}

void test_motor_balance_trim_and_stepping(void) {
    // Trim stepping clamps strictly to [-15, +15]%
    int8_t trim = 0;
    for (int i = 0; i < 25; ++i) {
        trim = RobotLogic::stepMotorTrim(trim, 1);
    }
    TEST_ASSERT_EQUAL_INT8(Config::kMaxMotorTrimPct, trim);

    for (int i = 0; i < 50; ++i) {
        trim = RobotLogic::stepMotorTrim(trim, -1);
    }
    TEST_ASSERT_EQUAL_INT8(static_cast<int8_t>(-Config::kMaxMotorTrimPct), trim);

    // Positive trim (+10%) attenuates Right wheel by 10% (100 -> 90)
    int16_t l = 100;
    int16_t r = 100;
    RobotLogic::applyMotorBalanceTrim(l, r, 10);
    TEST_ASSERT_EQUAL_INT16(100, l);
    TEST_ASSERT_EQUAL_INT16(90, r);

    // Negative trim (-10%) attenuates Left wheel by 10% (-100 -> -90)
    l = -100;
    r = -100;
    RobotLogic::applyMotorBalanceTrim(l, r, -10);
    TEST_ASSERT_EQUAL_INT16(-90, l);
    TEST_ASSERT_EQUAL_INT16(-100, r);

    // Verify integration in computeKinodynamicDrive at full forward throttle
    const DriveOutput trimmedDrive = RobotLogic::computeKinodynamicDrive(
        0U, 128U, 100, 100, 100, 100, 100, 100U, 140, false, false, false, 10);
    TEST_ASSERT_EQUAL_INT16(100, trimmedDrive.left);
    TEST_ASSERT_LESS_THAN_INT16(100, trimmedDrive.right);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(90, trimmedDrive.right);
}

void test_persistent_config_crc16_pack_unpack(void) {
    PersistentConfig src{};
    src.leftReversed     = true;
    src.rightReversed    = false;
    src.precisionMode    = true;
    src.rc2SmoothEnabled = false;
    src.motorTrimPct     = -7;

    uint8_t raw[8]{};
    RobotLogic::packPersistentConfig(src, raw);

    PersistentConfig dst{};
    TEST_ASSERT_TRUE(RobotLogic::unpackPersistentConfig(raw, dst));
    TEST_ASSERT_TRUE(dst.leftReversed);
    TEST_ASSERT_FALSE(dst.rightReversed);
    TEST_ASSERT_TRUE(dst.precisionMode);
    TEST_ASSERT_FALSE(dst.rc2SmoothEnabled);
    TEST_ASSERT_EQUAL_INT8(-7, dst.motorTrimPct);

    // Corrupt 1 bit in CRC -> must reject
    uint8_t badCrc[8];
    memcpy(badCrc, raw, 8U);
    badCrc[6] ^= 0x01U;
    TEST_ASSERT_FALSE(RobotLogic::unpackPersistentConfig(badCrc, dst));

    // Corrupt magic byte -> must reject
    uint8_t badMagic[8];
    memcpy(badMagic, raw, 8U);
    badMagic[0] = 'X';
    TEST_ASSERT_FALSE(RobotLogic::unpackPersistentConfig(badMagic, dst));

    // Blank/uninitialized EEPROM (all 0xFF) -> must reject
    const uint8_t blank[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
    TEST_ASSERT_FALSE(RobotLogic::unpackPersistentConfig(blank, dst));
}

void test_auto_encoder_stall_progress_logic(void) {
    // Delta < 3 deg/counts -> stalled (false)
    TEST_ASSERT_FALSE(RobotLogic::hasEncoderProgressed(100, 100, 3));
    TEST_ASSERT_FALSE(RobotLogic::hasEncoderProgressed(100, 102, 3));
    TEST_ASSERT_FALSE(RobotLogic::hasEncoderProgressed(100, 98, 3));

    // Delta >= 3 deg/counts (forward or reverse) -> progressing (true)
    TEST_ASSERT_TRUE(RobotLogic::hasEncoderProgressed(100, 103, 3));
    TEST_ASSERT_TRUE(RobotLogic::hasEncoderProgressed(100, 97, 3));
    TEST_ASSERT_TRUE(RobotLogic::hasEncoderProgressed(-500, -510, 3));

    // Auto DriveDC polarity resolution from saved EEPROM f1/f2 flags:
    bool revL = false;
    bool revR = true;
    RobotLogic::resolveAutoDrivePolarity(
        1U, 2U, Config::kMotorLeftReverseInit, Config::kMotorRightReverseInit, revL, revR);
    TEST_ASSERT_FALSE(revL);
    TEST_ASSERT_TRUE(revR);

    // Flipping left motor (f1) inverts revL while keeping revR
    revL = false;
    revR = true;
    RobotLogic::resolveAutoDrivePolarity(
        1U, 2U, !Config::kMotorLeftReverseInit, Config::kMotorRightReverseInit, revL, revR);
    TEST_ASSERT_TRUE(revL);
    TEST_ASSERT_TRUE(revR);

    // Flipping right motor (f2) inverts revR while keeping revL
    revL = false;
    revR = true;
    RobotLogic::resolveAutoDrivePolarity(
        1U, 2U, Config::kMotorLeftReverseInit, !Config::kMotorRightReverseInit, revL, revR);
    TEST_ASSERT_FALSE(revL);
    TEST_ASSERT_FALSE(revR);

    // Non-drive motor pair IDs (e.g. 3, 4) are left untouched
    revL = false;
    revR = true;
    RobotLogic::resolveAutoDrivePolarity(
        3U, 4U, !Config::kMotorLeftReverseInit, !Config::kMotorRightReverseInit, revL, revR);
    TEST_ASSERT_FALSE(revL);
    TEST_ASSERT_TRUE(revR);
}

void test_serial_phase2_trim_and_rc2_smooth_keys(void) {
    SerialControl sc;
    SerialActions act{};

    NativeSerial::feed(",.0y");
    sc.poll(6000UL, 65, act);
    TEST_ASSERT_TRUE(act.trimLeftStep);
    TEST_ASSERT_TRUE(act.trimRightStep);
    TEST_ASSERT_TRUE(act.trimReset);
    TEST_ASSERT_TRUE(act.toggleRc2Smooth);
}

void test_servo_direction_inversion(void) {
    // Non-inverted preserves logical angle and clamps out-of-range inputs
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyServoDirection(0, false));
    TEST_ASSERT_EQUAL_INT16(30, RobotLogic::applyServoDirection(30, false));
    TEST_ASSERT_EQUAL_INT16(70, RobotLogic::applyServoDirection(70, false));
    TEST_ASSERT_EQUAL_INT16(140, RobotLogic::applyServoDirection(140, false));
    TEST_ASSERT_EQUAL_INT16(180, RobotLogic::applyServoDirection(180, false));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyServoDirection(-20, false));
    TEST_ASSERT_EQUAL_INT16(180, RobotLogic::applyServoDirection(220, false));

    // Inverted mirrors deg -> (180 - deg) after clamping
    TEST_ASSERT_EQUAL_INT16(180, RobotLogic::applyServoDirection(0, true));
    TEST_ASSERT_EQUAL_INT16(150, RobotLogic::applyServoDirection(30, true));
    TEST_ASSERT_EQUAL_INT16(110, RobotLogic::applyServoDirection(70, true));
    TEST_ASSERT_EQUAL_INT16(40, RobotLogic::applyServoDirection(140, true));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyServoDirection(180, true));
    TEST_ASSERT_EQUAL_INT16(180, RobotLogic::applyServoDirection(-20, true));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyServoDirection(220, true));

    // Round-trip involution (used when syncing Auto hardware angle back to Manual logical target)
    TEST_ASSERT_EQUAL_INT16(70, RobotLogic::applyServoDirection(RobotLogic::applyServoDirection(70, true), true));
    TEST_ASSERT_EQUAL_INT16(180, RobotLogic::applyServoDirection(RobotLogic::applyServoDirection(180, true), true));

    // Config constants remain within valid servo range after direction mapping
    const int16_t rc3InitHw = RobotLogic::applyServoDirection(Config::kRc3InitDeg, Config::kRc3InvertDir);
    const int16_t rc4InitHw = RobotLogic::applyServoDirection(Config::kRc4InitDeg, Config::kRc4InvertDir);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(Config::kServoMinDeg, rc3InitHw);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kServoMaxDeg, rc3InitHw);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(Config::kServoMinDeg, rc4InitHw);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kServoMaxDeg, rc4InitHw);
    TEST_ASSERT_EQUAL_INT16(
        Config::kRc3InvertDir ? static_cast<int16_t>(Config::kServoMaxDeg - Config::kRc3InitDeg)
                              : Config::kRc3InitDeg,
        rc3InitHw);
    TEST_ASSERT_EQUAL_INT16(
        Config::kRc4InvertDir ? static_cast<int16_t>(Config::kServoMaxDeg - Config::kRc4InitDeg)
                              : Config::kRc4InitDeg,
        rc4InitHw);
}

void test_kinodynamic_envelope_and_overflow_safety(void) {
    // 1. Precision Mode (55%) forward-to-pivot transition with raised arm (RC2=0 -> CoM 65%)
    //    must NEVER overshoot the 55% ceiling during 2-DOF slew.
    const DriveOutput precPivot = RobotLogic::computeKinodynamicDrive(
        128U, 255U, 55, 55, Config::kMotorMaxPower, 25, 50, 100U, 0, false, false, false, 0, true);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kPrecisionMaxPower, precPivot.left);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(static_cast<int16_t>(-Config::kPrecisionMaxPower), precPivot.left);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kPrecisionMaxPower, precPivot.right);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(static_cast<int16_t>(-Config::kPrecisionMaxPower), precPivot.right);

    // 2. Battery Emergency cap (35%, decel=18) forward-to-pivot must NEVER overshoot 35%.
    const DriveOutput emergPivot = RobotLogic::computeKinodynamicDrive(
        128U, 255U, 35, 35, Config::kBattEmergMaxPower, 6, 18, 100U, 140, false, false, false, 0, false);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(Config::kBattEmergMaxPower, emergPivot.left);
    TEST_ASSERT_GREATER_OR_EQUAL_INT16(static_cast<int16_t>(-Config::kBattEmergMaxPower), emergPivot.right);

    // 3. Adversarial int16_t targets with 115% torque compensation must NOT overflow or flip sign.
    const DriveOutput highComp = RobotLogic::applyDriveTargetsWithSlew(
        30000, -32768, 0, 0, 100, 100, 100, 115U);
    TEST_ASSERT_GREATER_THAN_INT16(0, highComp.left);
    TEST_ASSERT_LESS_OR_EQUAL_INT16(100, highComp.left);
    TEST_ASSERT_EQUAL_INT16(-100, highComp.right);

    // 4. Extreme int16_t inputs in applyAsymmetricSlewStep, slewServoAngle, applyMotorBalanceTrim, decodePs2Payload.
    TEST_ASSERT_EQUAL_INT16(32767, RobotLogic::applyAsymmetricSlewStep(32700, 32767, 100, 100));
    TEST_ASSERT_EQUAL_INT16(-32768, RobotLogic::applyAsymmetricSlewStep(-32700, -32768, 100, 100));
    TEST_ASSERT_EQUAL_INT16(-1, RobotLogic::applyAsymmetricSlewStep(-32768, 100, 100, 32767));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::applyAsymmetricSlewStep(-32767, 100, 100, 32767));
    TEST_ASSERT_EQUAL_INT16(Config::kRc2MaxDeg, RobotLogic::slewServoAngle(90, 180, 32767));
    TEST_ASSERT_EQUAL_INT16(0, RobotLogic::slewServoAngle(90, 0, 32767));

    int16_t trimL = -32768;
    int16_t trimR = 32767;
    RobotLogic::applyMotorBalanceTrim(trimL, trimR, -10);
    TEST_ASSERT_LESS_THAN_INT16(0, trimL);
    TEST_ASSERT_EQUAL_INT16(32767, trimR);

    Ps2Frame nullFrame = makeValidFrame();
    RobotLogic::decodePs2Payload(nullptr, nullFrame);
    TEST_ASSERT_FALSE(nullFrame.linkOk);
    TEST_ASSERT_FALSE(RobotLogic::isPs2FrameValid(nullFrame));
}

void test_serial_estop_batch_help_and_telemetry_fields(void) {
    SerialControl sc;
    SerialActions act{};

    // 1. ESTOP '!' followed by drive key 'w' in the same RX batch must keep driveCmd stopped.
    NativeSerial::feed("!w");
    sc.poll(7000UL, 80, act);
    TEST_ASSERT_TRUE(act.estopRequested);
    TEST_ASSERT_FALSE(act.driveCmd.active);
    TEST_ASSERT_EQUAL_INT16(0, act.driveCmd.targetLeft);
    TEST_ASSERT_EQUAL_INT16(0, act.driveCmd.targetRight);

    // 2. Help text accurately matches o=180 / p=0 and documents 'v' I2C diag.
    NativeSerial::reset();
    NativeSerial::feed("?");
    sc.poll(7050UL, 80, act);
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "Aux RC1 180 / 0 deg"));
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "I2C diag"));

    // 3. printEvent handles nullptr safely without crashing.
    NativeSerial::reset();
    sc.printEvent(nullptr, nullptr);
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "[] "));

    // 4. Snapshot telemetry includes reverse-front (rf=) and CoM scale (com=).
    SystemSnapshot snap{};
    snap.reverseDrive = true;
    snap.comScalePct  = 78U;
    NativeSerial::reset();
    sc.printSnapshot(snap);
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "rf=1"));
    TEST_ASSERT_NOT_NULL(strstr(NativeSerial::output(), "com=78%"));
}

// ============================================================================
// 11. Manual stall cut-off
// ============================================================================

void test_manual_stall_detector_trip_rearm_and_fail_open(void) {
    RobotLogic::ManualStallDetector d;
    d.reset();
    const uint32_t step = Config::kManualStallPollMs;

    // Full command, wheels never move: arms on first sample, trips after the timeout.
    uint32_t t = 1000UL;
    TEST_ASSERT_FALSE(d.update(80, 80, true, true, 0, 0, t));
    bool tripped = false;
    uint32_t tripAt = 0UL;
    for (uint32_t i = 1U; i <= 20U && !tripped; ++i) {
        t += step;
        tripped = d.update(80, 80, true, true, 0, 0, t);
        if (tripped) {
            tripAt = t - 1000UL;
        }
    }
    TEST_ASSERT_TRUE(tripped);
    TEST_ASSERT_TRUE(tripAt >= Config::kManualStallTimeoutMs);
    TEST_ASSERT_TRUE(tripAt < Config::kManualStallTimeoutMs + 2U * step);

    // Stays latched while the stick is still pushed, even if the wheels start moving.
    TEST_ASSERT_TRUE(d.update(80, 80, true, true, 500, 500, t + step));
    // Mid command (between rearm and min) does not clear it either.
    TEST_ASSERT_TRUE(d.update(30, 30, false, false, 0, 0, t + 2U * step));
    // Back to neutral re-arms; pushing again does not trip immediately.
    TEST_ASSERT_FALSE(d.update(0, 0, false, false, 0, 0, t + 3U * step));
    TEST_ASSERT_FALSE(d.tripped());
    TEST_ASSERT_FALSE(d.update(80, 80, true, true, 500, 500, t + 4U * step));
}

void test_manual_stall_detector_progress_low_cmd_and_invalid_sample(void) {
    RobotLogic::ManualStallDetector d;
    d.reset();
    const uint32_t step = Config::kManualStallPollMs;

    // Wheels keep turning: never trips over 5 s.
    int32_t enc = 0;
    uint32_t t  = 0UL;
    for (uint32_t i = 0U; i < 25U; ++i) {
        enc += 40;
        TEST_ASSERT_FALSE(d.update(100, -100, true, true, enc, -enc, t));
        t += step;
    }

    // Below kManualStallMinCmdPct a frozen encoder is not a stall.
    d.reset();
    t = 0UL;
    for (uint32_t i = 0U; i < 25U; ++i) {
        TEST_ASSERT_FALSE(d.update(Config::kManualStallMinCmdPct - 1,
                                   Config::kManualStallMinCmdPct - 1, true, true, 0, 0, t));
        t += step;
    }

    // Failed encoder reads fail open and restart the timeout.
    d.reset();
    t = 0UL;
    for (uint32_t i = 0U; i < 25U; ++i) {
        TEST_ASSERT_FALSE(d.update(90, 90, true, false, 0, 0, t));
        t += step;
    }
    TEST_ASSERT_FALSE(d.tripped());
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
    RUN_TEST(test_square_push_ground_toggle_and_l1_r1_override);

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

    // 8. Auto Mode Trigger & Abort Logic
    RUN_TEST(test_auto_mode_triangle_start_and_cross_exit);

    // 9. Physics Kinodynamic & Electromechanical Models + Competition Controls
    RUN_TEST(test_cubic_expo_shaping_symmetry_and_bounds);
    RUN_TEST(test_centripetal_traction_limiter_and_pivot_authority);
    RUN_TEST(test_com_elevation_dynamic_scaling);
    RUN_TEST(test_2dof_kinodynamic_slew_and_coulomb_stiction);
    RUN_TEST(test_kinodynamic_drive_reverse_front_and_dpad_micro_turn);
    RUN_TEST(test_precision_mode_full_stick_scaling_and_micro_turn);
    RUN_TEST(test_triangle_sets_rc2_140_and_rc1_100);
    RUN_TEST(test_circle_pose_preset_and_serial_competition_shortcuts);

    // 10. Phase 2: RC2 Smooth Trajectory, Motor Trim, EEPROM CRC-16, Auto Stall Detection
    RUN_TEST(test_rc2_smooth_servo_slew);
    RUN_TEST(test_motor_balance_trim_and_stepping);
    RUN_TEST(test_persistent_config_crc16_pack_unpack);
    RUN_TEST(test_auto_encoder_stall_progress_logic);
    RUN_TEST(test_serial_phase2_trim_and_rc2_smooth_keys);
    RUN_TEST(test_servo_direction_inversion);
    RUN_TEST(test_kinodynamic_envelope_and_overflow_safety);
    RUN_TEST(test_serial_estop_batch_help_and_telemetry_fields);

    // 11. Manual stall cut-off
    RUN_TEST(test_manual_stall_detector_trip_rearm_and_fail_open);
    RUN_TEST(test_manual_stall_detector_progress_low_cmd_and_invalid_sample);

    return UNITY_END();
}
