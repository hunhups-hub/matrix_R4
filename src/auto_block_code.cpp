// ============================================================================
//  Auto Block Code (วางโค้ด Auto จาก MATRIXblock R4 ที่ไฟล์นี้ได้เลย)
// ============================================================================
//  วิธีใช้งาน:
//    1. เปิด MATRIXblock R4 -> คัดลอกโค้ด C++ ที่สร้างได้
//    2. นำโค้ดมาวางแทนที่ในกรอบด้านล่าง (ระหว่างเครื่องหมาย ====== ที่กำหนดไว้)
//       - วางได้ทั้ง ตัวแปร (Variables), ฟังก์ชันช่วย, void setup(), และ void loop()
//       - ปกติแนะนำให้ตัดบรรทัด #include <MatrixMiniR4.h> ออก
//         (หรือถ้าเผลอวางติดมาด้วย โค้ดก็ยังคอมไพล์ผ่านได้ตามปกติ)
//       - MiniR4.begin() และ Serial.begin(...) ใน Auto ถูกครอบเป็นตัวปลอดภัยให้แล้ว
//         ไม่ต้องกังวลว่าจะไปรีเซ็ตระบบหลักหรือตัดการเชื่อมต่อจอย PS2
//    3. กดปุ่ม สามเหลี่ยม (Triangle) บนจอย PS2 -> เริ่มเข้าโหมด Auto
//    4. กดปุ่ม X (Cross) บนจอย PS2 ได้ทุกเมื่อ -> หยุดมอเตอร์และออกจาก Auto กลับ Manual ทันที
//
//  หมายเหตุกรณีเขียนโค้ดเอง:
//    - หากวางโค้ด Auto ทั้งหมดไว้ใน void setup() แล้วปล่อย void loop() ว่างไว้
//      เมื่อทำงานครบทุกบรรทัด ระบบจะหยุดมอเตอร์และเด้งกลับโหมด Manual ให้อัตโนมัติทันที
//    - หากใส่โค้ดวนซ้ำไว้ใน void loop() ระบบจะทำงานวนต่อเนื่องจนกว่าจะกด X (หรือเรียก autoExit())
//    - delay(...) และบล็อก DriveDC ต่างๆ กด X ออกกลางคันได้ทันทีและไม่ทำให้ Watchdog รีเซ็ต
//    - ถ้าเขียน while(...) วนรอเอง ให้ใส่ delay(1); ข้างใน หรือเช็ก if (autoShouldExit()) break;
// ============================================================================

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wtype-limits"
#pragma GCC diagnostic ignored "-Wparentheses"

#ifndef ENABLE_DRIVEDC_BRAKE_DELAY
#define ENABLE_DRIVEDC_BRAKE_DELAY
#endif

#include <Arduino.h>
#include <EEPROM.h>
#include <MatrixMiniR4.h>
#include <SPI.h>
#include <Wire.h>
#include <utility>

#include "auto_mode.h"

namespace AutoBlockUser {

template <uint8_t ID>
struct AutoRcProxy {
    static inline MiniR4RC<ID>& real() {
        if constexpr (ID == 1U) {
            return ::MiniR4.RC1;
        } else if constexpr (ID == 2U) {
            return ::MiniR4.RC2;
        } else if constexpr (ID == 3U) {
            return ::MiniR4.RC3;
        } else {
            return ::MiniR4.RC4;
        }
    }

    bool begin() const { return real().begin(); }
    bool setHWDir(bool dir) const { return real().setHWDir(dir); }
    bool setAngle(uint16_t angle) const {
        const bool ok = real().setAngle(angle);
        if (ok) {
            AutoMode::recordServoAngle(ID, angle);
        }
        return ok;
    }
    operator MiniR4RC<ID>&() const { return real(); }
};

// ตัวครอบความปลอดภัยระดับคอนเซปต์ MATRIXblock R4:
// - MiniR4.begin(), PWR.setBattCell(), Serial.begin() เป็น safe no-op ไม่ทับค่า Config/PS2/STM32
// - MiniR4.I2C1..I2C4.MJ5.begin() บน Wire1 ข้าม Wire1.begin() เพื่อไม่ให้บัส OLED ค้าง/ช้าลง
// - MiniR4.DriveDC.begin() มี timeout 3 วินาทีป้องกันค้างวนลูปไม่รู้จบจากบั๊กไลบรารีบน STM32/IMU
// - MiniR4.PS2 อ่านค่าจอยจริงจาก Ps2Driver (D2+D3), ยิงสัญญาณ PSB_TRIANGLE 1 ช็อตตอนเข้า Auto และกลับสู่ Manual อัตโนมัติหลังจบรอบ
// - RC1..RC4 ซิงก์มุมเซอร์โวกลับสู่โหมด Manual อัตโนมัติ

struct AutoPowerProxy {
    static MiniR4Power& real() { return ::MiniR4.PWR; }

    bool setBattCell(uint8_t cell) const {
        (void)cell;
        return true;
    }
    float getBattVoltage() const { return real().getBattVoltage(); }
    float getBattPercentage() const { return real().getBattPercentage(); }

    operator MiniR4Power&() const { return real(); }
};

template <typename T>
struct Mj5IsVoid {
    static constexpr bool value = false;
};
template <>
struct Mj5IsVoid<void> {
    static constexpr bool value = true;
};

template <uint8_t ID, TwoWire* WIRE>
inline MiniR4I2C<ID, WIRE>& getRealI2c() = delete;

template <>
inline MiniR4I2C<static_cast<uint8_t>(-1), &Wire>& getRealI2c<static_cast<uint8_t>(-1), &Wire>() {
    return ::MiniR4.I2C0;
}
template <>
inline MiniR4I2C<0U, &Wire1>& getRealI2c<0U, &Wire1>() {
    return ::MiniR4.I2C1;
}
template <>
inline MiniR4I2C<1U, &Wire1>& getRealI2c<1U, &Wire1>() {
    return ::MiniR4.I2C2;
}
template <>
inline MiniR4I2C<2U, &Wire1>& getRealI2c<2U, &Wire1>() {
    return ::MiniR4.I2C3;
}
template <>
inline MiniR4I2C<3U, &Wire1>& getRealI2c<3U, &Wire1>() {
    return ::MiniR4.I2C4;
}

template <uint8_t ID, TwoWire* WIRE>
struct AutoMj5Proxy {
    static MatrixMJ5& real() { return getRealI2c<ID, WIRE>().MJ5; }

    bool begin() const {
        if constexpr (ID == static_cast<uint8_t>(-1)) {
            if constexpr (Mj5IsVoid<decltype(real().begin())>::value) {
                real().begin();
                return true;
            } else {
                return real().begin();
            }
        }
        return true;
    }

    operator MatrixMJ5&() const { return real(); }
};

template <uint8_t ID, TwoWire* WIRE>
struct AutoI2cProxy {
    static MiniR4I2C<ID, WIRE>& real() { return getRealI2c<ID, WIRE>(); }

    AutoI2cProxy()
        : MJ5{},
          MXLaser(real().MXLaser),
          MXLaserV2(real().MXLaserV2),
          MXColor(real().MXColor),
          MXColorV3(real().MXColorV3),
          MXGesture(real().MXGesture),
          MXCtrl(real().MXCtrl),
          GroveBME280(real().GroveBME280),
          HTCol(real().HTCol),
          MXLineTracer(real().MXLineTracer) {}

    AutoMj5Proxy<ID, WIRE>       MJ5;
    decltype(real().MXLaser)&      MXLaser;
    decltype(real().MXLaserV2)&    MXLaserV2;
    decltype(real().MXColor)&      MXColor;
    decltype(real().MXColorV3)&    MXColorV3;
    decltype(real().MXGesture)&    MXGesture;
    decltype(real().MXCtrl)&       MXCtrl;
    decltype(real().GroveBME280)&  GroveBME280;
    decltype(real().HTCol)&        HTCol;
    decltype(real().MXLineTracer)& MXLineTracer;

    operator MiniR4I2C<ID, WIRE>&() const { return real(); }
};

struct AutoDriveDCProxy {
    static MiniR4DriveDC<1>& real() { return ::MiniR4.DriveDC; }

    uint8_t begin(uint8_t leftMotorID,
                  uint8_t rightMotorID,
                  bool isLeftReverse,
                  bool isRightReverse) const {
        const MMLower::DIR dirL = isLeftReverse ? MMLower::DIR::REVERSE : MMLower::DIR::FORWARD;
        const MMLower::DIR dirR = isRightReverse ? MMLower::DIR::REVERSE : MMLower::DIR::FORWARD;

        for (uint8_t attempt = 0U; attempt < 30U; ++attempt) {
            AutoMode::service();
            if (!AutoMode::isRunning()) {
                return 0x09U;
            }
            if (mmL.Set_Drive2Motor_PARAM(leftMotorID, rightMotorID, dirL, dirR, 1U) ==
                MMLower::Drive_RESULT::OK) {
                return real().begin(leftMotorID, rightMotorID, isLeftReverse, isRightReverse);
            }
            for (uint8_t step = 0U; step < 4U; ++step) {
                AutoMode::service();
                if (!AutoMode::isRunning()) {
                    return 0x09U;
                }
                ::delay(25UL);
            }
        }
        return 0x09U;
    }

    bool setMoveSyncPID(float kp, float ki, float kd) const {
        return real().setMoveSyncPID(kp, ki, kd);
    }
    bool setMoveGyroPID(float kp, float ki, float kd) const {
        return real().setMoveGyroPID(kp, ki, kd);
    }
    bool setTurnGyroPID(float kp, float ki, float kd) const {
        return real().setTurnGyroPID(kp, ki, kd);
    }

    uint8_t Move(int16_t leftPower, int16_t rightPower) const {
        AutoMode::service();
        const uint8_t res = real().Move(leftPower, rightPower);
        AutoMode::service();
        return res;
    }
    uint8_t MoveDegs(int16_t leftPower,
                     int16_t rightPower,
                     uint16_t degrees,
                     bool brakeOnStop,
                     bool asyncWait = false) const {
        AutoMode::service();
        const uint8_t res = real().MoveDegs(leftPower, rightPower, degrees, brakeOnStop, asyncWait);
        AutoMode::service();
        return res;
    }
    uint8_t MoveTime(int16_t leftPower,
                     int16_t rightPower,
                     float seconds,
                     bool brakeOnStop,
                     bool asyncWait = false) const {
        AutoMode::service();
        const uint8_t res = real().MoveTime(leftPower, rightPower, seconds, brakeOnStop, asyncWait);
        AutoMode::service();
        return res;
    }
    uint8_t MoveSync(int16_t leftPower, int16_t rightPower) const {
        AutoMode::service();
        const uint8_t res = real().MoveSync(leftPower, rightPower);
        AutoMode::service();
        return res;
    }
    uint8_t MoveSyncDegs(int16_t leftPower,
                         int16_t rightPower,
                         uint16_t degrees,
                         bool brakeOnStop,
                         bool asyncWait = false) const {
        AutoMode::service();
        const uint8_t res =
            real().MoveSyncDegs(leftPower, rightPower, degrees, brakeOnStop, asyncWait);
        AutoMode::service();
        return res;
    }
    uint8_t MoveSyncTime(int16_t leftPower,
                         int16_t rightPower,
                         float seconds,
                         bool brakeOnStop,
                         bool asyncWait = false) const {
        AutoMode::service();
        const uint8_t res =
            real().MoveSyncTime(leftPower, rightPower, seconds, brakeOnStop, asyncWait);
        AutoMode::service();
        return res;
    }
    uint8_t MoveGyro(int16_t leftPower, int16_t rightPower) const {
        AutoMode::service();
        const uint8_t res = real().MoveGyro(leftPower, rightPower);
        AutoMode::service();
        return res;
    }
    uint8_t MoveGyroDegs(int16_t leftPower,
                         int16_t rightPower,
                         uint16_t degrees,
                         bool brakeOnStop,
                         bool asyncWait = false) const {
        AutoMode::service();
        const uint8_t res =
            real().MoveGyroDegs(leftPower, rightPower, degrees, brakeOnStop, asyncWait);
        AutoMode::service();
        return res;
    }
    uint8_t MoveGyroTime(int16_t leftPower,
                         int16_t rightPower,
                         float seconds,
                         bool brakeOnStop,
                         bool asyncWait = false) const {
        AutoMode::service();
        const uint8_t res =
            real().MoveGyroTime(leftPower, rightPower, seconds, brakeOnStop, asyncWait);
        AutoMode::service();
        return res;
    }
    uint8_t TurnGyro(int16_t leftPower,
                     int16_t rightPower,
                     uint8_t targetAngle,
                     bool brakeOnStop,
                     bool asyncWait = false) const {
        AutoMode::service();
        const uint8_t res =
            real().TurnGyro(leftPower, rightPower, targetAngle, brakeOnStop, asyncWait);
        AutoMode::service();
        return res;
    }

    int32_t getCounter() const { return real().getCounter(); }
    int32_t getDegrees() const { return real().getDegrees(); }
    uint8_t resetCounter() const { return real().resetCounter(); }
    uint8_t brake(bool state) const { return real().brake(state); }
    bool isPrevTaskDone() const { return real().isPrevTaskDone(); }

    operator MiniR4DriveDC<1>&() const { return real(); }
};

struct AutoPs2Proxy {
    bool read_gamepad() const { return pollInternal(); }

    bool read_gamepad(bool motor1, uint8_t motor2) const {
        (void)motor1;
        (void)motor2;
        return pollInternal();
    }

    bool Button(uint16_t button) const {
        ensurePolledOnAutoEntry();
        consumeEntryTriIfMatched(button);
        return (pressedMask_ & button) != 0U;
    }

    unsigned int ButtonDataByte() const {
        ensurePolledOnAutoEntry();
        consumeEntryTriIfMatched(static_cast<uint16_t>(PSB_TRIANGLE));
        return pressedMask_;
    }

    bool NewButtonState() const {
        ensurePolledOnAutoEntry();
        return (pressedMask_ ^ prevPressedMask_) != 0U;
    }

    bool NewButtonState(unsigned int button) const {
        ensurePolledOnAutoEntry();
        consumeEntryTriIfMatched(static_cast<uint16_t>(button));
        return ((pressedMask_ ^ prevPressedMask_) & static_cast<uint16_t>(button)) != 0U;
    }

    bool ButtonPressed(unsigned int button) const {
        return NewButtonState(button) && Button(static_cast<uint16_t>(button));
    }

    bool ButtonReleased(unsigned int button) const {
        return NewButtonState(button) &&
               ((prevPressedMask_ & static_cast<uint16_t>(button)) != 0U);
    }

    uint8_t Analog(uint8_t button) const {
        ensurePolledOnAutoEntry();
        switch (button) {
            case PSS_RX: return frame_.rx;
            case PSS_RY: return frame_.ry;
            case PSS_LX: return frame_.lx;
            case PSS_LY: return frame_.ly;
            default:     return 0U;
        }
    }

    uint8_t readType() const { return 1U; }

    uint8_t config_gamepad(uint8_t clk,
                           uint8_t cmd,
                           uint8_t att,
                           uint8_t dat,
                           bool pressures = false,
                           bool rumble    = false) const {
        (void)clk;
        (void)cmd;
        (void)att;
        (void)dat;
        (void)pressures;
        (void)rumble;
        return 0U;
    }

    void enableRumble() const {}
    bool enablePressures() const { return false; }
    void reconfig_gamepad() const {}

    operator PS2X&() const { return ::MiniR4.PS2; }

private:
    bool pollInternal() const {
        bool entryTri = false;
        const bool ok = AutoMode::pollPs2ForAuto(frame_, entryTri);
        uint16_t mask = 0U;
        if (ok) {
            mask = static_cast<uint16_t>(
                (static_cast<uint16_t>(static_cast<uint8_t>(~frame_.buttons2)) << 8U) |
                static_cast<uint16_t>(static_cast<uint8_t>(~frame_.buttons1)));
        }
        if (entryTri) {
            prevPressedMask_ = 0U;
            entryTriPending_ = true;
            mask |= static_cast<uint16_t>(PSB_TRIANGLE);
        } else {
            prevPressedMask_ = pressedMask_;
            entryTriPending_ = false;
        }
        pressedMask_ = mask;
        return ok || entryTri;
    }

    void ensurePolledOnAutoEntry() const {
        if (AutoMode::hasEntryTrianglePending()) {
            (void)pollInternal();
        }
    }

    void consumeEntryTriIfMatched(uint16_t mask) const {
        if (entryTriPending_ && ((mask & static_cast<uint16_t>(PSB_TRIANGLE)) != 0U)) {
            entryTriPending_ = false;
            AutoMode::notifyPs2TriangleMatched();
        }
    }

    mutable Ps2Frame frame_{};
    mutable uint16_t pressedMask_{0U};
    mutable uint16_t prevPressedMask_{0U};
    mutable bool     entryTriPending_{false};
};

struct AutoMiniR4Proxy {
    static constexpr AutoPowerProxy                             PWR{};
    static inline auto&                                         M1          = ::MiniR4.M1;
    static inline auto&                                         M2          = ::MiniR4.M2;
    static inline auto&                                         M3          = ::MiniR4.M3;
    static inline auto&                                         M4          = ::MiniR4.M4;
    static constexpr AutoDriveDCProxy                           DriveDC{};
    static constexpr AutoRcProxy<1>                             RC1{};
    static constexpr AutoRcProxy<2>                             RC2{};
    static constexpr AutoRcProxy<3>                             RC3{};
    static constexpr AutoRcProxy<4>                             RC4{};
    static inline auto&                                         BTN_UP      = ::MiniR4.BTN_UP;
    static inline auto&                                         BTN_DOWN    = ::MiniR4.BTN_DOWN;
    static inline auto&                                         LED         = ::MiniR4.LED;
    static inline auto&                                         Motion      = ::MiniR4.Motion;
    static inline auto&                                         Buzzer      = ::MiniR4.Buzzer;
    static inline auto&                                         OLED        = ::MiniR4.OLED;
    static inline auto&                                         WiFi        = ::MiniR4.WiFi;
    static inline const AutoI2cProxy<static_cast<uint8_t>(-1), &Wire>  I2C0{};
    static inline const AutoI2cProxy<0U, &Wire1>                I2C1{};
    static inline const AutoI2cProxy<1U, &Wire1>                I2C2{};
    static inline const AutoI2cProxy<2U, &Wire1>                I2C3{};
    static inline const AutoI2cProxy<3U, &Wire1>                I2C4{};
    static inline auto&                                         D1          = ::MiniR4.D1;
    static inline auto&                                         D2          = ::MiniR4.D2;
    static inline auto&                                         D3          = ::MiniR4.D3;
    static inline auto&                                         D4          = ::MiniR4.D4;
    static inline auto&                                         A1          = ::MiniR4.A1;
    static inline auto&                                         A2          = ::MiniR4.A2;
    static inline auto&                                         A3          = ::MiniR4.A3;
    static inline auto&                                         Uart        = ::MiniR4.Uart;
    static inline AutoPs2Proxy                                  PS2{};
    static inline auto&                                         Vernier     = ::MiniR4.Vernier;
    static inline auto&                                         Vision      = ::MiniR4.Vision;
    static inline auto&                                         MXMotorCtrl = ::MiniR4.MXMotorCtrl;

    bool begin() const { return true; }
};

// ตัวครอบ Serial: ทำให้ Serial.begin(...) ในโค้ดที่วางไม่ไปเปลี่ยน baudrate หลัก (115200)
struct AutoSerialProxy {
    template <typename... Args>
    void begin(Args&&...) const {}

    void end() const {}

    template <typename... Args>
    auto print(Args&&... args) const -> decltype(::Serial.print(std::forward<Args>(args)...)) {
        return ::Serial.print(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto println(Args&&... args) const -> decltype(::Serial.println(std::forward<Args>(args)...)) {
        return ::Serial.println(std::forward<Args>(args)...);
    }

    template <typename... Args>
    auto write(Args&&... args) const -> decltype(::Serial.write(std::forward<Args>(args)...)) {
        return ::Serial.write(std::forward<Args>(args)...);
    }

    int   available() const { return ::Serial.available(); }
    int   read() const { return ::Serial.read(); }
    int   peek() const { return ::Serial.peek(); }
    void  flush() const { ::Serial.flush(); }
    explicit operator bool() const { return static_cast<bool>(::Serial); }
    operator decltype(::Serial)&() const { return ::Serial; }
};

inline constexpr AutoMiniR4Proxy MiniR4{};
inline constexpr AutoSerialProxy Serial{};

// ============================================================================
// >>>>> วางโค้ด AUTO จาก MATRIXblock R4 ด้านล่างนี้ได้เลย <<<<<
// >>>>> PASTE MATRIXblock R4 AUTO CODE BELOW THIS LINE <<<<<
// ============================================================================

#define ENABLE_DRIVEDC_BRAKE_DELAY
#include "MatrixMiniR4.h"

void setup()
{
  MiniR4.begin();
  MiniR4.PWR.setBattCell(2);
  Serial.begin(9600);
  MiniR4.I2C1.MJ5.begin();
  MiniR4.DriveDC.begin(1, 2, false, true);
  MiniR4.RC2.setAngle(70);
  MiniR4.RC1.setAngle(75);
  if(MiniR4.PS2.Button(PSB_TRIANGLE))
  {
    MiniR4.DriveDC.MoveDegs((-50), (-50), 415, true);
    delay(1000);
    MiniR4.RC2.setAngle(165);
    delay(1000);
    MiniR4.RC1.setAngle(145);
    delay(1000);
    MiniR4.RC2.setAngle(0);
    delay(1000);
    MiniR4.DriveDC.MoveDegs((-50), 50, 275, true);
    delay(1000);
    MiniR4.DriveDC.MoveDegs((-50), (-50), 1300, true);
    delay(1000);
    MiniR4.RC2.setAngle(125);
    delay(500);
    MiniR4.RC1.setAngle(75);
    delay(1000);
    MiniR4.DriveDC.MoveDegs(50, 50, 300, true);
    delay(500);
    MiniR4.DriveDC.MoveDegs(50, (-50), 255, true);
    delay(500);
    MiniR4.DriveDC.MoveDegs((-50), (-50), 300, true);
    delay(500);
    MiniR4.DriveDC.MoveDegs(50, (-50), 255, true);
    delay(500);
    MiniR4.RC2.setAngle(140);
    delay(1000);
    MiniR4.DriveDC.MoveDegs((-50), (-50), 450, true);
    delay(500);
    MiniR4.RC1.setAngle(145);
    delay(1000);
    MiniR4.DriveDC.MoveDegs(50, 50, 300, true);
    delay(500);
    MiniR4.DriveDC.MoveDegs(50, (-50), 515, true);
    delay(500);
    MiniR4.DriveDC.MoveDegs((-50), (-50), 400, true);
    delay(500);
    MiniR4.RC2.setAngle(120);
    delay(1000);
    MiniR4.RC1.setAngle(75);
    delay(1000);
  }
}

void loop()
{
  MiniR4.PS2.read_gamepad(false, 0);

}




// ============================================================================
// >>>>> สิ้นสุดโซนวางโค้ด AUTO (ห้ามลบเครื่องหมาย } ด้านล่าง) <<<<<
// ============================================================================

} // namespace AutoBlockUser

#pragma GCC diagnostic pop
