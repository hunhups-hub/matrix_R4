#pragma once

#include <stdint.h>

// ============================================================================
//  user_settings.h  --  ค่าที่ผู้ใช้ปรับบ่อย (แก้ค่าที่หุ่นที่ไฟล์นี้ที่เดียว)
//  ค่าทั้งหมดที่คนขับ/ทีมปรับตามปกติ ส่วนพิน คาบเวลาโปรโตคอล และขีดจำกัด
//  ภายในอยู่ใน config.h
// ============================================================================
//  ที่ไหนแก้อะไร:
//   - ค่าความเร็ว/มุมเซอร์โว/ทิศมอเตอร์/Auto ...   -> ไฟล์นี้ (user_settings.h)
//   - ปุ่มจอย PS2 ทำหน้าที่อะไร                     -> include/controls.h
//   - โค้ดภารกิจ Auto จาก MATRIXblock R4            -> src/auto_block_code.cpp
//   - คีย์พิมพ์ผ่าน Serial Monitor                  -> src/serial_control.cpp
//   - พิน/คาบเวลา/ขีดจำกัดฮาร์ดแวร์                 -> include/config.h
//  หลังแก้ ให้รัน:  pio test -e native  แล้ว  pio run -e uno_r4_wifi
// ============================================================================
//  ปรับค่าการบังคับ (เรียงตามที่แก้บ่อย):
//   1) ความเร็วขับ          -> หมวด 1 ด้านล่าง
//   2) มุมเซอร์โว / แขนยก   -> หมวด 2 ด้านล่าง
//   3) ปุ่มจอย              -> include/controls.h
//  รูปแบบคอมเมนต์:  ช่วง a..b | ผลเมื่อเพิ่มค่า
//  ใส่ค่านอกช่วงแล้ว build จะหยุดพร้อมข้อความภาษาไทย (ตัวตรวจอยู่ท้าย config.h)
// ============================================================================

// ค่าที่ถูกทับตอน build ได้ด้วยแฟลก -D ใน platformio.ini (ค่าเริ่มต้นปิดการตัดกำลังเมื่อล้อติด)
#ifndef MATRIX_MANUAL_STALL_CUT
#define MATRIX_MANUAL_STALL_CUT 0
#endif

namespace Config {

// ============================================================================
// --- 1) ความเร็วขับ (Drive speed)  ปุ่มสลับโหมดดู controls.h ---
// ============================================================================
constexpr int16_t  kNormalMaxPower         = 100;     // ช่วง 1..100 | ความเร็วสูงสุดโหมดปกติ (%) เมื่อโยกก้านสุด
constexpr int16_t  kPrecisionMaxPower          = 55;   // ความเร็วสูงสุดเมื่อโยกก้านสุดในโหมดช้า/แม่นยำ (%)
                                                       // ช่วง 1..kNormalMaxPower | ปุ่ม kPrecisionToggle
constexpr int16_t  kMicroTurnPower         = 26;      // กำลังหมุนละเอียดเมื่อกด D-Pad ซ้าย/ขวา เพื่อจัดแนว (%)
                                                       // ช่วง 0..100 | ตอนนี้ไม่มีปุ่มจอยผูกกับหมุนละเอียด (D-Pad ซ้าย/ขวาว่าง)
constexpr int16_t  kPrecisionMicroTurnPower    = 18;   // กำลังหมุนละเอียดเมื่อกด D-Pad ซ้าย/ขวา ในโหมดแม่นยำ (%)
                                                       // ช่วง 0..100
constexpr uint8_t  kTurnMaxRatioPct        = 100U;    // สัดส่วนกำลังเลี้ยวสูงสุดที่ความเร็วปกติ (%)
                                                       // ช่วง 1..100 | เพิ่ม = เลี้ยวแรง/เร็วขึ้น
constexpr int16_t  kSerialDefaultSpeed     = 100;     // ความเร็วเริ่มต้นผ่าน Serial เต็ม 100% (ช่วง 20..100%)

// ============================================================================
// --- 2) เซอร์โว RC1..RC4  มุมต่างๆ / กลับทิศ / ช่วงพัลส์ (ปุ่มที่สั่ง ดู controls.h) ---
//  มุมทุกตัว ช่วง 0..180 องศา
// ============================================================================

// --- 2.1) RC1 กริปเปอร์  (ปุ่ม kGripperOpen / kGripperClose / kPosePreset) ---
constexpr int16_t  kRc1InitDeg             = 25;       // มุมเริ่มต้นของเซอร์โวเสริม (กริปเปอร์)
constexpr int16_t  kRc1L2Deg               = 25;     // ปุ่ม kGripperOpen (L2)
constexpr int16_t  kRc1R2Deg               = 145;       // ปุ่ม kGripperClose (R2)
constexpr int16_t  kRc1ZeroDeg             = 50;       // ปุ่ม kGripperZero (D-Pad ขวา): RC1 -> 0°
constexpr int16_t  kRc1PresetHoldDeg      = 100;      // พรีเซ็ตท่าถือ (ใช้เฉพาะ Serial q): กริปเปอร์จับ/ถือของ (ตรงกับ Auto)
constexpr int16_t  kRc1PresetDropDeg       = 150;     // พรีเซ็ตวงกลม (ปุ่มวางอย่างเดียว): กริปเปอร์เปิด/ปล่อยของ (ตรงกับ Auto)
constexpr uint32_t kPoseDropReleaseDelayMs = 600UL;   // ปุ่ม Circle: ขยับแขน RC2 ก่อน แล้วรอเท่านี้ (ms) ค่อยปล่อยกริปเปอร์ RC1
                                                       // ช่วง 0..5000 | 0 = ปล่อยพร้อมแขน | เพิ่ม = ปล่อยช้าลง (ให้แขนลงถึงก่อน)

// --- 2.1b) ท่าปุ่ม Triangle (kTrianglePose): กดครั้งเดียวตั้งมุมแล้วค้างไว้ ---
constexpr int16_t  kRcTrianglePoseRc1Deg   = 150;     // RC1 (แขน R1) 0..180
constexpr int16_t  kRcTrianglePoseRc2Deg   = 130;    // RC2 (แขน R2) ช่วง kRc2MinDeg..kRc2MaxDeg

// --- 2.2) RC2 แขนยก  (ปุ่ม kArmUp / kArmDown / kPosePreset / kSmoothToggle) ---
//  ความเร็วแขนเมื่อกดค้าง = kRc2StepDeg / kArmStepIntervalMs
//  (ค่าปัจจุบัน 10 องศา / 50 ms = 200 องศา/วินาที)
constexpr int16_t  kRc2InitDeg             = 90;      // มุมเริ่มต้นแขน RC2 (D-Pad ขึ้น/ลง)
constexpr int16_t  kRc2MinDeg              = 0;       // มุมสิ้นสุดต่ำสุดของแขน RC2 (ช่วง 0..180, ต้อง < kRc2MaxDeg)
constexpr int16_t  kRc2MaxDeg              = 175;     // มุมสิ้นสุดสูงสุดของแขน RC2 (ช่วง 0..180) | แขนไม่เกินมุมนี้
constexpr int16_t  kRc2StepDeg             = 10;      // 10 องศา / 50 ms (~200 องศา/วินาที)
                                                       // ช่วง 1..90 | เพิ่ม = แขนขยับเร็วขึ้น/หยาบขึ้น
constexpr uint32_t kArmStepIntervalMs      = 50UL;    // คาบขยับแขน RC2 หนึ่งขั้นเมื่อกดค้าง (ms)
                                                       // ช่วง >= 25 (คาบลูป) | เพิ่ม = แขนขยับช้าลง
constexpr bool     kRc2InvertDir           = true;    // กลับทิศ RC2: ขึ้น/หน้า ลดมุม, ลง/หลัง เพิ่มมุม
constexpr int16_t  kRc2PresetHoldDeg       = 0

;      // พรีเซ็ตท่าถือ (ใช้เฉพาะ Serial q): ท่ายกแขนถือของ (ตรงกับ Auto)
constexpr int16_t  kRc2PresetDropDeg       = 164;     // พรีเซ็ตวงกลม (ปุ่มวางอย่างเดียว): ท่าลดแขนวางของ (ตรงกับ Auto)
constexpr int16_t  kRc2SmoothStepDeg       = 6;       // ขั้น slew ของ RC2 ต่อ 25 ms (6 องศา/25 ms = 240 องศา/วินาที)
                                                       // ช่วง 1..90 | เพิ่ม = แขนกวาดเร็วขึ้นในโหมดนุ่ม
constexpr bool     kRc2SmoothDefault       = true;    // เปิดการเคลื่อนที่นุ่มนวลเป็นค่าเริ่มต้น เฉพาะ RC2

// --- 2.3) RC3 + RC4 แขนหน้าซ้าย/ขวา  (ปุ่ม kFrontArmsDown / kFrontArmsUp / kPushGround) ---
constexpr int16_t  kRc3InitDeg             = 80;      // มุมเริ่มต้นแขนหน้าซ้าย (คู่กับ RC4; จำกัด 0..180)
constexpr int16_t  kRc3L1Deg               = 80;      // L1 กดลง (80 องศา; มุมน้อย = แขนลงลึกกว่า; ต่ำสุด 0)
constexpr int16_t  kRc3R1Deg               = 145;     // R1 ยกขึ้น / เก็บกลาง (145 องศา)
constexpr int16_t  kRc3SquareDeg           = 30;      // Square สลับกดพื้น (30 องศา)
constexpr bool     kRc3InvertDir           = false;   // กลับมุมที่ส่งออกฮาร์ดแวร์ของ RC3 (องศา -> 180 - องศา)
constexpr int16_t  kRc4InitDeg             = 80;       // มุมเริ่มต้นแขนหน้าขวา (คู่กับ RC3)
constexpr int16_t  kRc4L1Deg               = 80;       // L1 กดลง (คู่กับ RC3)
constexpr int16_t  kRc4R1Deg               = 145;      // R1 ยกขึ้น / เก็บกลาง (คู่กับ RC3)
constexpr int16_t  kRc4SquareDeg           = 30;     // Square สลับกดพื้น (คู่กับ RC3)
constexpr bool     kRc4InvertDir           = true;    // กลับมุมที่ส่งออกฮาร์ดแวร์ของ RC4 (องศา -> 180 - องศา)
constexpr int16_t  kFrontArmSmoothStepDeg  = 0;       // ขั้น slew ของ RC3/RC4 ต่อ 25 ms (0 = ปิด slew, ตั้งเป็น 8 = 320 องศา/วินาที)
                                                       // ช่วง 0..180 | 0 = กระโดดทันที (ปิด slew) | ลด = ขยับนุ่มขึ้น กระแสพุ่งน้อยลง

// ============================================================================
// --- 3) ความรู้สึกการขับ (Drive feel, ขั้นสูง)  ค่าความนุ่ม/การกลับทิศมอเตอร์ ---
// ============================================================================
constexpr bool     kMotorLeftReverseInit   = false;   // กลับทิศจากค่าเริ่มต้นของ MATRIX ให้ตรงกับด้านหน้าของตัวถัง
constexpr bool     kMotorRightReverseInit  = true;
constexpr bool     kEnableSmartHoldBrake   = true;    // ล็อก M1/M2 ด้วยเบรก H-bridge ตอนหยุดนิ่ง (0,0)
constexpr uint8_t  kStickDeadzoneRaw       = 18U;     // เดดโซนกลางก้านจอย ~14% (127/128)
                                                       // ช่วง 0..126 | เพิ่ม = ต้องโยกก้านมากขึ้นก่อนหุ่นขยับ
constexpr uint8_t  kThrottleExpoPct        = 15U;     // เส้นโค้งลูกบาศก์ 15% ที่คันเร่ง ลดอาการกระชาก
                                                       // ช่วง 0..100 | เพิ่ม = ช่วงต้นก้านนุ่มขึ้น
constexpr uint8_t  kTurnExpoPct            = 30U;     // เส้นโค้งลูกบาศก์ 30% ที่การเลี้ยว
constexpr uint8_t  kPrecisionThrottleExpoPct   = 55U;  // เส้นโค้ง expo ลูกบาศก์ของคันเร่งให้นุ่มขึ้นในโหมดแม่นยำ
constexpr uint8_t  kPrecisionTurnExpoPct       = 60U;  // เส้นโค้ง expo ลูกบาศก์ของการเลี้ยวให้นุ่มขึ้นในโหมดแม่นยำ
// ขั้น slew ทุกตัว ช่วง 1..100 | เพิ่ม = ตอบสนองเร็วขึ้นแต่กระชากขึ้น
constexpr int16_t  kMotorAccelSlewStep     = 50;      // 0 -> 100% ใช้เวลา ~50 ms
constexpr int16_t  kMotorDecelSlewStep     = 50;      // 100% -> 0 ใช้เวลา ~50 ms
constexpr int16_t  kAngularAccelSlewStep   = 60;    // ขั้น slew ของการหมุน (2-DOF) ตอนเร่ง
constexpr int16_t  kAngularDecelSlewStep   = 60;      // ขั้น slew ของการหมุน (2-DOF) ตอนเบรก
constexpr int16_t  kMaxCentripetalLatAccel = 75;      // ขีดจำกัดวงกลมการยึดเกาะ |v * w| / 100 <= 75
constexpr int16_t  kMotorStictionMinPower  = 18;    // กำลังขั้นต่ำชนะแรงเสียดทานสถิต (%)
                                                       // ช่วง 0..50
constexpr uint8_t  kCoMHighMinScalePct     = 85U;   // สเกลเร่ง/แรงสู่ศูนย์กลางต่ำสุดเมื่อยกแขนสูง (RC2->0)
constexpr int16_t  kCoMLowArmDeg           = 140;     // มุม RC2 >= ค่านี้ ถือว่าจุดศูนย์ถ่วงต่ำ (สเกลไดนามิก 100%)
constexpr int8_t   kMaxMotorTrimPct        = 15;      // ค่าชดเชยซ้าย/ขวาสูงสุดเพื่อให้วิ่งตรง (+/-15%)
                                                       // ช่วง 0..50

// --- 4) แบตเตอรี่  นโยบายกำลังขับเมื่อไฟตก ---
constexpr bool     kEnableBatterySagThrottle = false; // คงกำลังมอเตอร์และ slew 100% แม้แบตต่ำ , มีโอกาศให้เปิด
constexpr int16_t  kBattLowMaxPower        = 72;      // เพดานกำลังและขั้น slew ของแต่ละระดับ (เมื่อเปิด sag throttle)
constexpr int16_t  kBattCritMaxPower       = 50;
constexpr int16_t  kBattEmergMaxPower      = 35;
constexpr uint8_t  kBattCompMaxPct         = 115U;    // เพดานการชดเชยแรงบิดแบบจำกัดขอบเขต

// ชดเชยแรงดันใน Auto: power ของ DriveDC (MoveDegs/MoveTime/MoveSync*/MoveGyro*/TurnGyro) คูณด้วย
// kAutoVoltRefMv / แรงดันแบต ล้อจึงหมุนเร็วเท่าตอนจูน mission ระยะและมุมเลี้ยวไม่เพี้ยนตอนแบตอ่อน
// ตั้งค่าอ้างอิง: เข้า Auto ด้วยแบตระดับที่ใช้จูน แล้วดู event "AUTO volt comp ...% @x.xxV" ใน Serial
constexpr bool     kEnableAutoVoltComp     = true;    // false = ส่ง power ตามที่เขียนใน mission ตรงๆ
constexpr uint16_t kAutoVoltRefMv          = 8200U;   // ช่วง 5500..25500 | แรงดันแบต (mV) ตอนจูน mission
constexpr uint8_t  kAutoCompMinPct         = 90U;     // ช่วง 50..100 | แบตสูงกว่าค่าอ้างอิง: ลด power ได้สุด 10%
constexpr uint8_t  kAutoCompMaxPct         = 130U;    // ช่วง 100..200 | แบตอ่อน: เพิ่ม power ได้สุด 30%

// --- 5) โหมด Auto  เวลา / ตรวจล้อติด ---
constexpr uint32_t kAutoStartBtnHoldMs     = 1200UL;  // กด BTN_UP ค้างเท่านี้เพื่อเริ่ม Auto โดยไม่ใช้ PS2
constexpr uint32_t kAutoMaxDurationMs      = 60000UL; // เวลา Auto สูงสุดเพื่อความปลอดภัย (60 วินาที; 0 = ไม่จำกัด)
constexpr uint32_t kAutoStallGraceMs       = 1250UL;  // ช่วงผ่อนผันแรกของแต่ละคำสั่ง Auto ก่อนเริ่มตรวจล้อติด
constexpr uint32_t kAutoStallTimeoutMs     = 1600UL;  // ช่วงที่ล้อติดต่อเนื่องโดยเปลี่ยนน้อยกว่า kAutoStallMinDeltaDeg
constexpr int32_t  kAutoStallMinDeltaDeg   = 3L;      // ค่าเปลี่ยนแปลง encoder ขั้นต่ำที่รีเซ็ตตัวจับเวลาล้อติด

// --- 6) ตัดกำลังเมื่อล้อติดขัดตอนขับ Manual  (ปิดเป็นค่าเริ่มต้น) ---
constexpr bool     kEnableManualStallCut   = (MATRIX_MANUAL_STALL_CUT != 0);
constexpr int16_t  kManualStallMinCmdPct   = 60;      // ตัดสินล้อติดเฉพาะเมื่อ |คำสั่งล้อ| >= ค่านี้
constexpr int16_t  kManualStallRearmCmdPct = 15;      // คงสถานะตัดกำลังไว้จนกว่า |คำสั่ง| <= ค่านี้ (ก้านใกล้กลาง)
constexpr uint32_t kManualStallTimeoutMs   = 1500UL;  // encoder ไม่ขยับนานเท่านี้ -> ตัดกำลัง
constexpr uint32_t kManualStallPollMs      = 200UL;   // คาบอ่าน encoder (5 Hz) ขณะสั่งกำลังอยู่

} // namespace Config
