#pragma once

#include <stdint.h>

// ============================================================================
//  user_settings.h  --  ค่าที่ผู้ใช้ปรับบ่อย (แก้ค่าที่หุ่นที่ไฟล์นี้ที่เดียว)
//  Everything a driver/team normally tunes. Hardware pins, protocol timings and
//  internal limits stay in config.h.
// ============================================================================
//  ที่ไหนแก้อะไร:
//   - ค่าความเร็ว/มุมเซอร์โว/ทิศมอเตอร์/Auto ...   -> ไฟล์นี้ (user_settings.h)
//   - ปุ่มจอย PS2 ทำหน้าที่อะไร                     -> include/controls.h
//   - โค้ดภารกิจ Auto จาก MATRIXblock R4            -> src/auto_block_code.cpp
//   - คีย์พิมพ์ผ่าน Serial Monitor                  -> src/serial_control.cpp
//   - พิน/คาบเวลา/ขีดจำกัดฮาร์ดแวร์                 -> include/config.h
//  หลังแก้ ให้รัน:  pio test -e native  แล้ว  pio run -e uno_r4_wifi  (หรือ _field)
//
//  เปิดฟีเจอร์ Gyro/ตัดกำลัง/Heading Hold โดยไม่แก้ไฟล์: ใช้ env:uno_r4_wifi_field
//  (platformio.ini ส่ง -DMATRIX_AUTO_GYRO_MODE=2 ฯลฯ ทับค่าด้านล่าง)
// ============================================================================

// Build-time overrides (platformio.ini -D flags). Defaults keep every gyro feature off.
#ifndef MATRIX_AUTO_GYRO_MODE
#define MATRIX_AUTO_GYRO_MODE 0
#endif
#ifndef MATRIX_MANUAL_STALL_CUT
#define MATRIX_MANUAL_STALL_CUT 0
#endif
#ifndef MATRIX_TELEOP_HEADING_HOLD
#define MATRIX_TELEOP_HEADING_HOLD 0
#endif

namespace Config {

// --- 1) ความรู้สึกการขับ (Drive feel)  ค่าความเร็ว/ความนุ่ม/การกลับทิศมอเตอร์ ---
constexpr bool     kMotorLeftReverseInit   = false;   // Inverted from MATRIX default to match chassis front
constexpr bool     kMotorRightReverseInit  = true;
constexpr bool     kEnableSmartHoldBrake   = true;    // Lock M1/M2 with H-bridge brake when stationary (0,0)
constexpr int16_t  kSerialDefaultSpeed     = 100;     // Full 100% speed by default (20..100%)
constexpr uint8_t  kStickDeadzoneRaw       = 18U;     // ~14% center deadband (127/128)
constexpr int16_t  kPrecisionMaxPower          = 55;   // Slow/precision mode full-stick speed limit (%)
constexpr uint8_t  kPrecisionThrottleExpoPct   = 55U;  // Gentler cubic expo on throttle in precision mode
constexpr uint8_t  kPrecisionTurnExpoPct       = 60U;  // Gentler cubic expo on steering in precision mode
constexpr int16_t  kPrecisionMicroTurnPower    = 18;   // D-Pad Left/Right fine alignment pivot in precision mode (%)
constexpr int16_t  kMicroTurnPower         = 26;      // D-Pad Left/Right fine alignment pivot power (%)
constexpr uint8_t  kThrottleExpoPct        = 35U;     // 35% cubic jerk-reducing curve on throttle
constexpr uint8_t  kTurnExpoPct            = 45U;     // 45% cubic curve on steering
constexpr uint8_t  kTurnMaxRatioPct        = 80U;     // Max turn authority ratio at normal speed (%)
constexpr int16_t  kMotorAccelSlewStep     = 25;      // 0 -> 100% in ~100 ms
constexpr int16_t  kMotorDecelSlewStep     = 50;      // 100% -> 0 in ~50 ms
constexpr int16_t  kAngularAccelSlewStep   = 35;      // Rotational 2-DOF slew step
constexpr int16_t  kAngularDecelSlewStep   = 60;      // Rotational 2-DOF braking slew step
constexpr int16_t  kMaxCentripetalLatAccel = 52;      // Traction circle limit |v * w| / 100 <= 52
constexpr int16_t  kMotorStictionMinPower  = 12;      // Coulomb static friction threshold (%)
constexpr uint8_t  kCoMHighMinScalePct     = 65U;     // Min accel/centripetal scale when arm raised (RC2->0)
constexpr int16_t  kCoMLowArmDeg           = 140;     // RC2 angle >= this has low CoM (100% dynamic scale)
constexpr int8_t   kMaxMotorTrimPct        = 15;      // Max straight-line L/R balance trim (+/-15%)

// --- 2) เซอร์โว RC1..RC4  มุมต่างๆ / กลับทิศ / ช่วงพัลส์ (ปุ่มที่สั่ง ดู controls.h) ---
constexpr int16_t  kRc1InitDeg             = 180;       // Aux servo default
constexpr int16_t  kRc1L2Deg               = 180;
constexpr int16_t  kRc1R2Deg               = 0;
constexpr int16_t  kRc1PresetHoldDeg       = 75;      // Circle preset A: Gripper hold/carry (matches Auto)
constexpr int16_t  kRc1PresetDropDeg       = 145;     // Circle preset B: Gripper open/release (matches Auto)
constexpr int16_t  kRc2InitDeg             = 90;      // Spare/Arm RC2 (D-Pad UP/DOWN)
constexpr int16_t  kRc2StepDeg             = 10;      // 10 deg / 50 ms (~200 deg/s)
constexpr bool     kRc2InvertDir           = true;    // Invert RC2: UP/Fwd decreases deg, DOWN/Back increases deg
constexpr int16_t  kRc2PresetHoldDeg       = 70;      // Circle preset A: Arm raised carry pose (matches Auto)
constexpr int16_t  kRc2PresetDropDeg       = 140;     // Circle preset B: Arm lowered place pose (matches Auto)
constexpr int16_t  kRc2SmoothStepDeg       = 6;       // RC2 slew step per 25ms tick (6 deg/25ms = 240 deg/s)
constexpr bool     kRc2SmoothDefault       = true;    // Smooth trajectory enabled by default for RC2 only
constexpr int16_t  kRc3InitDeg             = 30;      // Front L arm init (paired with RC4; clamped 0..180)
constexpr int16_t  kRc3L1Deg               = 30;      // L1 Push down (30 deg; lower deg = deeper; min 0)
constexpr int16_t  kRc3R1Deg               = 140;     // R1 Lift / mid stow up (140 deg)
constexpr int16_t  kRc3SquareDeg           = 30;      // Square Push ground toggle (30 deg)
constexpr bool     kRc3InvertDir           = false;   // Invert RC3 hardware output angle (deg -> 180 - deg)
constexpr int16_t  kRc4InitDeg             = 30;      // Front R arm init (paired with RC3)
constexpr int16_t  kRc4L1Deg               = 30;      // L1 Push down (30 deg; paired with RC3)
constexpr int16_t  kRc4R1Deg               = 140;     // R1 Lift / mid stow up (140 deg)
constexpr int16_t  kRc4SquareDeg           = 30;      // Square Push ground toggle (30 deg)
constexpr bool     kRc4InvertDir           = true;    // Invert RC4 hardware output angle (deg -> 180 - deg)
constexpr bool     kEnableRc34CustomPulse  = true;    // Apply SetServoPulseRange specifically to RC3 & RC4
constexpr uint16_t kRc34PulseMinUs         = 500U;    // Wide standard min pulse (us) for RC3/RC4
constexpr uint16_t kRc34PulseMaxUs         = 2500U;   // Wide standard max pulse (us) for RC3/RC4

// --- 3) แบตเตอรี่  นโยบายกำลังขับเมื่อไฟตก ---
constexpr bool     kEnableBatterySagThrottle = false; // Keep 100% motor power & slew even when battery is low
constexpr int16_t  kBattLowMaxPower        = 72;      // Power caps & slew steps per tier (when sag throttle enabled)
constexpr int16_t  kBattCritMaxPower       = 50;
constexpr int16_t  kBattEmergMaxPower      = 35;
constexpr uint8_t  kBattCompMaxPct         = 115U;    // Bounded torque compensation cap

// --- 4) โหมด Auto  เวลา / ตรวจล้อติด ---
constexpr uint32_t kAutoStartBtnHoldMs     = 1200UL;  // Single BTN_UP hold to start Auto without PS2
constexpr uint32_t kAutoMaxDurationMs      = 60000UL; // Max Auto safety duration (60s; 0 = unlimited)
constexpr uint32_t kAutoStallGraceMs       = 1250UL;  // Initial grace window per Auto command before stall check
constexpr uint32_t kAutoStallTimeoutMs     = 1600UL;  // Continuous stall window with < kAutoStallMinDeltaDeg
constexpr int32_t  kAutoStallMinDeltaDeg   = 3L;      // Minimum encoder delta to reset stall timer

// --- 5) Gyro ช่วย Auto (IMU บนบอร์ด)  ปิดเป็นค่าเริ่มต้น  เปิดด้วย env:uno_r4_wifi_field ---
// 0 = ปิด (MoveDegs ทำงานเหมือนเดิมทุกอย่าง)
// 1 = วิ่งตรงด้วย MoveGyroDegs + จำทิศจริงจาก IMU หลังหมุน
// 2 = ข้อ 1 + หมุนด้วย TurnGyro ไปมุมที่ปัดเป็นขั้น
constexpr uint8_t  kAutoGyroMode           = MATRIX_AUTO_GYRO_MODE;  // Default Off; env:uno_r4_wifi_field sets 2 (see CLAUDE.md)
constexpr int16_t  kAutoPivotWheelDegPerYawDegX100 = 283;   // wheel deg per 1 deg yaw, x100 (mission: 255 wheel deg ~ 90 deg)
constexpr int16_t  kAutoPivotYawSign       = 1;       // +1/-1: yaw sign when right wheel goes forward and left backward
constexpr int16_t  kAutoTurnSnapStepDeg    = 45;      // Snap pivot estimate to this step (<= 0 disables snapping)
constexpr uint8_t  kAutoTurnGyroMode       = 1U;      // TurnGyro mode: 0 = single motor, 1 = two motor (pivot)
constexpr bool     kAutoMoveTelemetry      = true;    // Print one [AUTO] line per DriveDC move on Serial

// --- 6) ตัดกำลังเมื่อล้อติดขัดตอนขับ Manual  (ปิดเป็นค่าเริ่มต้น) ---
constexpr bool     kEnableManualStallCut   = (MATRIX_MANUAL_STALL_CUT != 0);
constexpr int16_t  kManualStallMinCmdPct   = 60;      // Only judge stalls while |wheel cmd| >= this
constexpr int16_t  kManualStallRearmCmdPct = 15;      // Cut stays latched until |cmd| <= this (stick near neutral)
constexpr uint32_t kManualStallTimeoutMs   = 1500UL;  // No encoder progress for this long -> cut
constexpr uint32_t kManualStallPollMs      = 200UL;   // Encoder sample period (5 Hz) while demanding

// --- 7) ล็อกทิศด้วย Gyro ตอนขับ Teleop  (ปิดเป็นค่าเริ่มต้น) ---
constexpr bool     kEnableTeleopHeadingHold = (MATRIX_TELEOP_HEADING_HOLD != 0);
constexpr uint32_t kTeleopHoldLatchDelayMs  = 250UL;  // Straight input this long before heading is latched
constexpr uint32_t kTeleopHoldPollMs        = 100UL;  // IMU sample period (10 Hz) while holding
constexpr int16_t  kTeleopHoldDeadbandDeg   = 2;      // No correction inside +/- this yaw error
constexpr int16_t  kTeleopHoldKpX10         = 15;     // Turn correction % per deg yaw error, x10
constexpr int16_t  kTeleopHoldMaxCorrPct    = 15;     // Max correction added/removed per wheel

} // namespace Config
