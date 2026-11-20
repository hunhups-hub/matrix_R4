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

// ค่าที่ถูกทับตอน build ได้ด้วยแฟลก -D ใน platformio.ini (ค่าเริ่มต้นปิดการตัดกำลังเมื่อล้อติด)
#ifndef MATRIX_MANUAL_STALL_CUT
#define MATRIX_MANUAL_STALL_CUT 0
#endif

namespace Config {

// --- 1) ความรู้สึกการขับ (Drive feel)  ค่าความเร็ว/ความนุ่ม/การกลับทิศมอเตอร์ ---
constexpr bool     kMotorLeftReverseInit   = false;   // กลับทิศจากค่าเริ่มต้นของ MATRIX ให้ตรงกับด้านหน้าของตัวถัง
constexpr bool     kMotorRightReverseInit  = true;
constexpr bool     kEnableSmartHoldBrake   = true;    // ล็อก M1/M2 ด้วยเบรก H-bridge ตอนหยุดนิ่ง (0,0)
constexpr int16_t  kSerialDefaultSpeed     = 100;     // ความเร็วเริ่มต้นผ่าน Serial เต็ม 100% (ช่วง 20..100%)
constexpr uint8_t  kStickDeadzoneRaw       = 18U;     // เดดโซนกลางก้านจอย ~14% (127/128)
constexpr int16_t  kPrecisionMaxPower          = 55;   // ความเร็วสูงสุดเมื่อโยกก้านสุดในโหมดช้า/แม่นยำ (%)
constexpr uint8_t  kPrecisionThrottleExpoPct   = 55U;  // เส้นโค้ง expo ลูกบาศก์ของคันเร่งให้นุ่มขึ้นในโหมดแม่นยำ
constexpr uint8_t  kPrecisionTurnExpoPct       = 60U;  // เส้นโค้ง expo ลูกบาศก์ของการเลี้ยวให้นุ่มขึ้นในโหมดแม่นยำ
constexpr int16_t  kPrecisionMicroTurnPower    = 18;   // กำลังหมุนละเอียดเมื่อกด D-Pad ซ้าย/ขวา ในโหมดแม่นยำ (%)
constexpr int16_t  kMicroTurnPower         = 26;      // กำลังหมุนละเอียดเมื่อกด D-Pad ซ้าย/ขวา เพื่อจัดแนว (%)
constexpr uint8_t  kThrottleExpoPct        = 35U;     // เส้นโค้งลูกบาศก์ 35% ที่คันเร่ง ลดอาการกระชาก
constexpr uint8_t  kTurnExpoPct            = 45U;     // เส้นโค้งลูกบาศก์ 45% ที่การเลี้ยว
constexpr uint8_t  kTurnMaxRatioPct        = 80U;     // สัดส่วนกำลังเลี้ยวสูงสุดที่ความเร็วปกติ (%)
constexpr int16_t  kMotorAccelSlewStep     = 25;      // 0 -> 100% ใช้เวลา ~100 ms
constexpr int16_t  kMotorDecelSlewStep     = 50;      // 100% -> 0 ใช้เวลา ~50 ms
constexpr int16_t  kAngularAccelSlewStep   = 35;      // ขั้น slew ของการหมุน (2-DOF) ตอนเร่ง
constexpr int16_t  kAngularDecelSlewStep   = 60;      // ขั้น slew ของการหมุน (2-DOF) ตอนเบรก
constexpr int16_t  kMaxCentripetalLatAccel = 52;      // ขีดจำกัดวงกลมการยึดเกาะ |v * w| / 100 <= 52
constexpr int16_t  kMotorStictionMinPower  = 12;      // กำลังขั้นต่ำชนะแรงเสียดทานสถิต (%)
constexpr uint8_t  kCoMHighMinScalePct     = 65U;     // สเกลเร่ง/แรงสู่ศูนย์กลางต่ำสุดเมื่อยกแขนสูง (RC2->0)
constexpr int16_t  kCoMLowArmDeg           = 140;     // มุม RC2 >= ค่านี้ ถือว่าจุดศูนย์ถ่วงต่ำ (สเกลไดนามิก 100%)
constexpr int8_t   kMaxMotorTrimPct        = 15;      // ค่าชดเชยซ้าย/ขวาสูงสุดเพื่อให้วิ่งตรง (+/-15%)

// --- 2) เซอร์โว RC1..RC4  มุมต่างๆ / กลับทิศ / ช่วงพัลส์ (ปุ่มที่สั่ง ดู controls.h) ---
constexpr int16_t  kRc1InitDeg             = 180;       // มุมเริ่มต้นของเซอร์โวเสริม (กริปเปอร์)
constexpr int16_t  kRc1L2Deg               = 180;
constexpr int16_t  kRc1R2Deg               = 0;
constexpr int16_t  kRc1PresetHoldDeg       = 75;      // พรีเซ็ตวงกลม A: กริปเปอร์จับ/ถือของ (ตรงกับ Auto)
constexpr int16_t  kRc1PresetDropDeg       = 145;     // พรีเซ็ตวงกลม B: กริปเปอร์เปิด/ปล่อยของ (ตรงกับ Auto)
constexpr int16_t  kRc2InitDeg             = 90;      // มุมเริ่มต้นแขน RC2 (D-Pad ขึ้น/ลง)
constexpr int16_t  kRc2StepDeg             = 10;      // 10 องศา / 50 ms (~200 องศา/วินาที)
constexpr bool     kRc2InvertDir           = true;    // กลับทิศ RC2: ขึ้น/หน้า ลดมุม, ลง/หลัง เพิ่มมุม
constexpr int16_t  kRc2PresetHoldDeg       = 70;      // พรีเซ็ตวงกลม A: ท่ายกแขนถือของ (ตรงกับ Auto)
constexpr int16_t  kRc2PresetDropDeg       = 140;     // พรีเซ็ตวงกลม B: ท่าลดแขนวางของ (ตรงกับ Auto)
constexpr int16_t  kRc2SmoothStepDeg       = 6;       // ขั้น slew ของ RC2 ต่อ 25 ms (6 องศา/25 ms = 240 องศา/วินาที)
constexpr bool     kRc2SmoothDefault       = true;    // เปิดการเคลื่อนที่นุ่มนวลเป็นค่าเริ่มต้น เฉพาะ RC2
constexpr int16_t  kRc3InitDeg             = 30;      // มุมเริ่มต้นแขนหน้าซ้าย (คู่กับ RC4; จำกัด 0..180)
constexpr int16_t  kRc3L1Deg               = 30;      // L1 กดลง (30 องศา; มุมน้อย = ลึกกว่า; ต่ำสุด 0)
constexpr int16_t  kRc3R1Deg               = 140;     // R1 ยกขึ้น / เก็บกลาง (140 องศา)
constexpr int16_t  kRc3SquareDeg           = 30;      // Square สลับกดพื้น (30 องศา)
constexpr bool     kRc3InvertDir           = false;   // กลับมุมที่ส่งออกฮาร์ดแวร์ของ RC3 (องศา -> 180 - องศา)
constexpr int16_t  kRc4InitDeg             = 30;      // มุมเริ่มต้นแขนหน้าขวา (คู่กับ RC3)
constexpr int16_t  kRc4L1Deg               = 30;      // L1 กดลง (30 องศา; คู่กับ RC3)
constexpr int16_t  kRc4R1Deg               = 140;     // R1 ยกขึ้น / เก็บกลาง (140 องศา)
constexpr int16_t  kRc4SquareDeg           = 30;      // Square สลับกดพื้น (30 องศา)
constexpr bool     kRc4InvertDir           = true;    // กลับมุมที่ส่งออกฮาร์ดแวร์ของ RC4 (องศา -> 180 - องศา)

// --- 3) แบตเตอรี่  นโยบายกำลังขับเมื่อไฟตก ---
constexpr bool     kEnableBatterySagThrottle = false; // คงกำลังมอเตอร์และ slew 100% แม้แบตต่ำ , มีโอกาศให้เปิด
constexpr int16_t  kBattLowMaxPower        = 72;      // เพดานกำลังและขั้น slew ของแต่ละระดับ (เมื่อเปิด sag throttle)
constexpr int16_t  kBattCritMaxPower       = 50;
constexpr int16_t  kBattEmergMaxPower      = 35;
constexpr uint8_t  kBattCompMaxPct         = 115U;    // เพดานการชดเชยแรงบิดแบบจำกัดขอบเขต

// --- 4) โหมด Auto  เวลา / ตรวจล้อติด ---
constexpr uint32_t kAutoStartBtnHoldMs     = 1200UL;  // กด BTN_UP ค้างเท่านี้เพื่อเริ่ม Auto โดยไม่ใช้ PS2
constexpr uint32_t kAutoMaxDurationMs      = 60000UL; // เวลา Auto สูงสุดเพื่อความปลอดภัย (60 วินาที; 0 = ไม่จำกัด)
constexpr uint32_t kAutoStallGraceMs       = 1250UL;  // ช่วงผ่อนผันแรกของแต่ละคำสั่ง Auto ก่อนเริ่มตรวจล้อติด
constexpr uint32_t kAutoStallTimeoutMs     = 1600UL;  // ช่วงที่ล้อติดต่อเนื่องโดยเปลี่ยนน้อยกว่า kAutoStallMinDeltaDeg
constexpr int32_t  kAutoStallMinDeltaDeg   = 3L;      // ค่าเปลี่ยนแปลง encoder ขั้นต่ำที่รีเซ็ตตัวจับเวลาล้อติด

// --- 5) ตัดกำลังเมื่อล้อติดขัดตอนขับ Manual  (ปิดเป็นค่าเริ่มต้น) ---
constexpr bool     kEnableManualStallCut   = (MATRIX_MANUAL_STALL_CUT != 0);
constexpr int16_t  kManualStallMinCmdPct   = 60;      // ตัดสินล้อติดเฉพาะเมื่อ |คำสั่งล้อ| >= ค่านี้
constexpr int16_t  kManualStallRearmCmdPct = 15;      // คงสถานะตัดกำลังไว้จนกว่า |คำสั่ง| <= ค่านี้ (ก้านใกล้กลาง)
constexpr uint32_t kManualStallTimeoutMs   = 1500UL;  // encoder ไม่ขยับนานเท่านี้ -> ตัดกำลัง
constexpr uint32_t kManualStallPollMs      = 200UL;   // คาบอ่าน encoder (5 Hz) ขณะสั่งกำลังอยู่

} // namespace Config
