#pragma once

#include <stdint.h>

#include "types.h"

// ============================================================================
//  controls.h  --  แผนผังปุ่มจอย PS2 (เปลี่ยนปุ่มแก้ที่ไฟล์นี้ที่เดียว)
//  การกำหนดหน้าที่ของทุกปุ่ม PS2 อยู่ในไฟล์นี้ทั้งหมด
// ============================================================================
//  วิธีเปลี่ยนปุ่ม: แก้ค่าทางขวาของ  constexpr Btn kXxx = Btn::...;
//  ตัวอย่าง: ให้ปุ่ม Triangle เป็นตัวสลับ Precision แทน Select ->
//      constexpr Btn kPrecisionToggle = Btn::Triangle;
//  ข้อควรระวัง:
//   - ห้ามให้ปุ่มสองหน้าที่ซ้ำกัน ถ้าหน้าที่นั้นอยู่ในกลุ่ม "ใช้พร้อมกันได้" (เช่น
//     kFrontArmsDown กับ kFrontArmsUp ต้องเป็นคนละปุ่ม; kTrimLeft ซ้ำ kMicroTurnLeft ได้
//     เพราะตัวหนึ่งต้องกด kTrimModifier ค้าง)
//   - kBrakeOrExit (Cross) ยังเป็นปุ่มหยุดฉุกเฉินของ Auto ด้วย: ถ้าย้าย ให้ทดสอบ X-abort ในโหมด Auto
//   - ปุ่มหยุด/ฉุกเฉินบนบอร์ด (BTN_UP / BTN_DOWN) และคีย์ Serial ไม่อยู่ในไฟล์นี้
//     (คีย์ Serial: src/serial_control.cpp, ปุ่มบนบอร์ด: src/minir4_io.cpp)
//   - รูปแบบแพ็กเก็ตจอย (การถอดรหัส/ตรวจความถูกต้อง) อยู่ใน src/link_failsafe.cpp
//     และไม่ต้องแก้เมื่อเปลี่ยนปุ่ม
//   - ถ้าตั้งปุ่มซ้ำกัน build จะหยุดพร้อมข้อความภาษาไทย (ตัวตรวจอยู่ท้ายไฟล์นี้)
// ============================================================================
//  ผังปุ่มปัจจุบัน (ถ้าแก้ปุ่มด้านล่าง ให้แก้รูปนี้ตามด้วย):
//
//        L2 = กริปเปอร์เปิด (RC1)             R2 = กริปเปอร์ปิด (RC1)
//        L1 = แขนหน้าลง (RC3+RC4)             R1 = แขนหน้าขึ้น (RC3+RC4)
//     +-----------------------------------------------------------------+
//     |     Up   = แขน RC2 ขึ้น              Triangle = เริ่ม Auto      |
//     |  Left / Right = หมุนละเอียด         Square = สลับดันพื้น        |
//     |     Down = แขน RC2 ลง               Circle = สลับท่า ถือ/วาง    |
//     |                                     Cross  = เบรก / ออก Auto    |
//     |        Select = โหมด Precision    Start = ปุ่มร่วม trim         |
//     |   (ก้านซ้าย)  L3 = RC2 นุ่ม        R3 = สลับหน้า-หลัง (ก้านขวา) |
//     +-----------------------------------------------------------------+
//     ก้านซ้ายแกน Y = เดินหน้า/ถอยหลัง     ก้านขวาแกน X = เลี้ยว
//     Start + Left / Right = trim -1% / +1%     Start + L3 = รีเซ็ต trim
// ============================================================================

namespace Controls {

enum class Btn : uint8_t {
    Up,
    Down,
    Left,
    Right,
    L1,
    R1,
    L2,
    R2,
    L3,
    R3,
    Triangle,
    Circle,
    Cross,
    Square,
    Select,
    Start
};

// --- Servos -------------------------------------------------------------------
constexpr Btn kFrontArmsDown = Btn::L1;       // RC3+RC4 ลงไปที่ kRc3L1Deg / kRc4L1Deg
constexpr Btn kFrontArmsUp   = Btn::R1;       // RC3+RC4 ยกไปที่ kRc3R1Deg / kRc4R1Deg
constexpr Btn kPushGround    = Btn::Square;   // สลับท่าดันพื้น (กดครั้งที่ 2 คืนค่าเดิม)
constexpr Btn kGripperOpen   = Btn::L2;       // RC1 -> kRc1L2Deg
constexpr Btn kGripperClose  = Btn::R2;       // RC1 -> kRc1R2Deg
constexpr Btn kPosePreset    = Btn::Circle;   // สลับท่า ถือ/วาง (RC1+RC2)
constexpr Btn kArmUp         = Btn::Up;       // RC2 ขยับขึ้น (ทิศกลับด้วย kRc2InvertDir)
constexpr Btn kArmDown       = Btn::Down;     // RC2 ขยับลง

// --- Drive modes & trim ---------------------------------------------------------
constexpr Btn kPrecisionToggle = Btn::Select; // โหมดช้า Precision เปิด/ปิด
constexpr Btn kReverseToggle   = Btn::R3;     // สลับหน้า-หลังของหุ่น (Reverse-Front)
constexpr Btn kSmoothToggle    = Btn::L3;     // เปิด/ปิดการกวาด RC2 แบบนุ่ม (บันทึก EEPROM)
constexpr Btn kTrimModifier    = Btn::Start;  // กดค้างร่วมกับปุ่มด้านล่างเพื่อปรับสมดุลล้อ
constexpr Btn kTrimLeft        = Btn::Left;   // Start + ปุ่มนี้ = trim -1%
constexpr Btn kTrimRight       = Btn::Right;  // Start + ปุ่มนี้ = trim +1%
// Start + kSmoothToggle = รีเซ็ต trim เป็น 0
constexpr Btn kMicroTurnLeft   = Btn::Left;   // หมุนตัวละเอียดซ้าย (ไม่กด kTrimModifier)
constexpr Btn kMicroTurnRight  = Btn::Right;  // หมุนตัวละเอียดขวา

// --- Match control -------------------------------------------------------------
constexpr Btn kAutoStart   = Btn::Triangle;   // เริ่มโหมด Auto (ขอบขาขึ้น, ต้องไม่กด kBrakeOrExit)
constexpr Btn kBrakeOrExit = Btn::Cross;      // เบรกหยุดล้อค้างขณะกด + ออกจาก Auto ทันที

/** เป็นจริงขณะปุ่ม `b` ถูกกดค้างอยู่ในเฟรม PS2 ที่ถอดรหัสแล้ว */
inline bool held(const Ps2Frame& f, Btn b) {
    switch (b) {
        case Btn::Up:       return f.up;
        case Btn::Down:     return f.down;
        case Btn::Left:     return f.left;
        case Btn::Right:    return f.right;
        case Btn::L1:       return f.l1;
        case Btn::R1:       return f.r1;
        case Btn::L2:       return f.l2;
        case Btn::R2:       return f.r2;
        case Btn::L3:       return f.l3;
        case Btn::R3:       return f.r3;
        case Btn::Triangle: return f.triangle;
        case Btn::Circle:   return f.circle;
        case Btn::Cross:    return f.cross;
        case Btn::Square:   return f.square;
        case Btn::Select:   return f.select;
        case Btn::Start:    return f.start;
    }
    return false;
}

// --- ตัวตรวจปุ่มซ้ำตอน build (ไม่เปลี่ยนการทำงาน ทำงานเฉพาะตอนคอมไพล์) ---------------

/** นับจำนวนหน้าที่ใน list[i..n) ที่ใช้ปุ่ม b */
constexpr uint8_t countBtn(const Btn* list, uint8_t n, uint8_t i, Btn b) {
    return (i >= n) ? 0U
                    : static_cast<uint8_t>((list[i] == b ? 1U : 0U) +
                                                countBtn(list, n, static_cast<uint8_t>(i + 1U), b));
}

/** เป็นจริงเมื่อทุกหน้าที่ใน list[i..n) ใช้ปุ่มไม่ซ้ำกัน */
constexpr bool allUnique(const Btn* list, uint8_t n, uint8_t i) {
    return (i >= n) || (countBtn(list, n, 0U, list[i]) == 1U && allUnique(list, n, static_cast<uint8_t>(i + 1U)));
}

// หน้าที่ที่กดใช้แยกกันได้อิสระ จึงต้องเป็นคนละปุ่มทั้งหมด
// (kTrimLeft / kTrimRight ไม่อยู่ในลิสต์นี้ เพราะใช้คู่กับ kTrimModifier จึงซ้ำ kMicroTurn ได้)
constexpr Btn kIndependentButtons[] = {
    kFrontArmsDown, kFrontArmsUp,   kPushGround,     kGripperOpen,
    kGripperClose,  kPosePreset,    kArmUp,          kArmDown,
    kPrecisionToggle, kReverseToggle, kSmoothToggle, kTrimModifier,
    kMicroTurnLeft, kMicroTurnRight, kAutoStart,     kBrakeOrExit,
};
constexpr uint8_t kIndependentButtonCount =
    static_cast<uint8_t>(sizeof(kIndependentButtons) / sizeof(kIndependentButtons[0]));

static_assert(allUnique(kIndependentButtons, kIndependentButtonCount, 0U),
              "controls.h: มีสองหน้าที่ใช้ปุ่มเดียวกัน ให้แก้ปุ่มทางขวาของ constexpr Btn kXxx ให้ไม่ซ้ำ");
static_assert(kTrimLeft != kTrimRight,
              "controls.h: kTrimLeft กับ kTrimRight ต้องเป็นคนละปุ่ม");
static_assert(kTrimLeft != kTrimModifier && kTrimRight != kTrimModifier,
              "controls.h: kTrimLeft / kTrimRight ต้องไม่ใช่ปุ่มเดียวกับ kTrimModifier");
static_assert(kSmoothToggle != kTrimModifier,
              "controls.h: kSmoothToggle ต้องไม่ใช่ปุ่มเดียวกับ kTrimModifier (ใช้คู่กันเพื่อรีเซ็ต trim)");

} // namespace Controls
