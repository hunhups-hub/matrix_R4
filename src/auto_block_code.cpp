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
//    - การออกกลางคัน (กด X / E-Stop / Timeout) ใช้ longjmp จึงห้ามประกาศตัวแปรที่มี destructor
//      (เช่น String, std::string) ในโค้ดที่วาง — ใช้ char[] หรือชนิดพื้นฐานแทน
//    - มีฟังก์ชันช่วยเขียนภารกิจด้วย C++ เอง (driveStraight, turnTo, turnBy, servoTo, headingNow)
//      อยู่เหนือโซนวางโค้ด และเปิด Gyro Assist ได้ที่ Config::kAutoGyroMode ใน include/user_settings.h
// ============================================================================

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
inline void initMj5() {}

template <>
inline void initMj5<static_cast<uint8_t>(-1), &Wire>() {
    ::MiniR4.I2C0.MJ5.begin();
}

template <uint8_t ID, TwoWire* WIRE>
struct AutoMj5Proxy {
    static MatrixMJ5& real() { return getRealI2c<ID, WIRE>().MJ5; }

    bool begin() const {
        initMj5<ID, WIRE>();
        return true;
    }
    bool     update() const { return real().update(); }
    bool     isConnected() const { return real().isConnected(); }
    uint8_t  getLX() const { return real().getLX(); }
    uint8_t  getLY() const { return real().getLY(); }
    uint8_t  getRX() const { return real().getRX(); }
    uint8_t  getRY() const { return real().getRY(); }
    uint16_t getButtons() const { return real().getButtons(); }
    bool     Button(uint16_t mask) const { return real().Button(mask); }

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
        AutoMode::noteUserLoopActivity();
        AutoMode::resolveDrivePolarity(leftMotorID, rightMotorID, isLeftReverse, isRightReverse);
        const MMLower::DIR dirL = isLeftReverse ? MMLower::DIR::REVERSE : MMLower::DIR::FORWARD;
        const MMLower::DIR dirR = isRightReverse ? MMLower::DIR::REVERSE : MMLower::DIR::FORWARD;

        for (uint8_t attempt = 0U; attempt < 30U; ++attempt) {
            AutoMode::service();
            if (!AutoMode::isRunning()) {
                return 0x09U;
            }
            if (mmL.Set_Drive2Motor_PARAM(leftMotorID, rightMotorID, dirL, dirR, 1U) ==
                MMLower::Drive_RESULT::OK) {
                return 0x00U;
            }
            for (uint8_t step = 0U; step < 4U; ++step) {
                AutoMode::service();
                if (!AutoMode::isRunning()) {
                    return 0x09U;
                }
                ::delay(25UL);
            }
        }
        // ~3 s of STM32 retries exhausted: abort Auto safely instead of driving blind.
        ::autoAbortDriveInit();
    }

    bool setMoveSyncPID(float kp, float ki, float kd) const {
        AutoMode::noteUserLoopActivity();
        return real().setMoveSyncPID(kp, ki, kd);
    }
    bool setMoveGyroPID(float kp, float ki, float kd) const {
        AutoMode::noteUserLoopActivity();
        return real().setMoveGyroPID(kp, ki, kd);
    }
    bool setTurnGyroPID(float kp, float ki, float kd) const {
        AutoMode::noteUserLoopActivity();
        return real().setTurnGyroPID(kp, ki, kd);
    }

    uint8_t Move(int16_t leftPower, int16_t rightPower) const {
        return sendDriveCmd([&]() {
            return mmL.Set_Drive_Move_Func(leftPower, rightPower, 1U);
        });
    }
    uint8_t MoveDegs(int16_t leftPower,
                     int16_t rightPower,
                     uint16_t degrees,
                     bool brakeOnStop,
                     bool asyncWait = false) const {
        if (Config::kAutoGyroMode != Config::kAutoGyroOff && AutoMode::gyroAssistActive()) {
            return moveDegsGyroAssist(leftPower, rightPower, degrees, brakeOnStop, asyncWait);
        }
        return moveDegsLegacy(leftPower, rightPower, degrees, brakeOnStop, asyncWait);
    }
    static uint32_t secondsToMs(float seconds) {
        if (!(seconds > 0.0f)) {
            return 0UL;
        }
        const float ms = seconds * 1000.0f + 0.5f;
        if (ms >= 4294967040.0f) {
            return 4294967040UL;
        }
        return static_cast<uint32_t>(ms);
    }

    uint8_t MoveTime(int16_t leftPower,
                     int16_t rightPower,
                     float seconds,
                     bool brakeOnStop,
                     bool asyncWait = false) const {
        const uint32_t timeMs       = secondsToMs(seconds);
        const bool     expectMotion = (leftPower != 0 || rightPower != 0) && timeMs > 0UL;
        return runBlockingDriveCmd(asyncWait, expectMotion, [&]() {
            return mmL.Set_Drive_Move_Time(
                leftPower, rightPower, timeMs, brakeOnStop, asyncWait, 1U);
        });
    }
    uint8_t MoveSync(int16_t leftPower, int16_t rightPower) const {
        return sendDriveCmd([&]() {
            return mmL.Set_Drive_MoveSync_Func(leftPower, rightPower, 1U);
        });
    }
    uint8_t MoveSyncDegs(int16_t leftPower,
                         int16_t rightPower,
                         uint16_t degrees,
                         bool brakeOnStop,
                         bool asyncWait = false) const {
        const bool expectMotion = (leftPower != 0 || rightPower != 0) && degrees != 0U;
        return runBlockingDriveCmd(asyncWait, expectMotion, [&]() {
            return mmL.Set_Drive_MoveSync_Degs(
                leftPower, rightPower, degrees, brakeOnStop, asyncWait, 1U);
        });
    }
    uint8_t MoveSyncTime(int16_t leftPower,
                         int16_t rightPower,
                         float seconds,
                         bool brakeOnStop,
                         bool asyncWait = false) const {
        const uint32_t timeMs       = secondsToMs(seconds);
        const bool     expectMotion = (leftPower != 0 || rightPower != 0) && timeMs > 0UL;
        return runBlockingDriveCmd(asyncWait, expectMotion, [&]() {
            return mmL.Set_Drive_MoveSync_Time(
                leftPower, rightPower, timeMs, brakeOnStop, asyncWait, 1U);
        });
    }
    // Explicit gyro calls from the user's mission also move the virtual heading, so later
    // gyro-assisted straights keep following the direction the user asked for.
    uint8_t MoveGyro(int16_t power, int16_t targetYawDeg) const {
        const uint8_t res = sendDriveCmd([&]() {
            return mmL.Set_Drive_MoveGyro_Func(power, targetYawDeg, 1U);
        });
        if (res == 0x00U) {
            AutoMode::setHeadingTargetDeg(targetYawDeg);
        }
        return res;
    }
    uint8_t MoveGyroDegs(int16_t power,
                         int16_t targetYawDeg,
                         uint16_t degrees,
                         bool brakeOnStop,
                         bool asyncWait = false) const {
        const bool expectMotion = (power != 0) && degrees != 0U;
        const uint8_t res = runBlockingDriveCmd(asyncWait, expectMotion, [&]() {
            return mmL.Set_Drive_MoveGyro_Degs(
                power, targetYawDeg, degrees, brakeOnStop, asyncWait, 1U);
        });
        if (res == 0x00U) {
            AutoMode::setHeadingTargetDeg(targetYawDeg);
        }
        return res;
    }
    uint8_t MoveGyroTime(int16_t power,
                         int16_t targetYawDeg,
                         float seconds,
                         bool brakeOnStop,
                         bool asyncWait = false) const {
        const uint32_t timeMs       = secondsToMs(seconds);
        const bool     expectMotion = (power != 0) && timeMs > 0UL;
        const uint8_t  res = runBlockingDriveCmd(asyncWait, expectMotion, [&]() {
            return mmL.Set_Drive_MoveGyro_Time(
                power, targetYawDeg, timeMs, brakeOnStop, asyncWait, 1U);
        });
        if (res == 0x00U) {
            AutoMode::setHeadingTargetDeg(targetYawDeg);
        }
        return res;
    }
    uint8_t TurnGyro(int16_t power,
                     int16_t targetYawDeg,
                     uint8_t mode,
                     bool brakeOnStop,
                     bool asyncWait = false) const {
        const bool expectMotion = (power != 0);
        const uint8_t res = runBlockingDriveCmd(asyncWait, expectMotion, [&]() {
            return mmL.Set_Drive_TurnGyro(
                power, targetYawDeg, mode, brakeOnStop, asyncWait, 1U);
        });
        if (res == 0x00U) {
            AutoMode::setHeadingTargetDeg(targetYawDeg);
        }
        return res;
    }

    int32_t getCounter() const {
        AutoMode::noteUserLoopActivity();
        AutoMode::service();
        return real().getCounter();
    }
    int32_t getDegrees() const {
        AutoMode::noteUserLoopActivity();
        AutoMode::service();
        return real().getDegrees();
    }
    uint8_t resetCounter() const {
        AutoMode::noteUserLoopActivity();
        AutoMode::service();
        return real().resetCounter();
    }
    uint8_t brake(bool state) const {
        AutoMode::noteUserLoopActivity();
        AutoMode::service();
        return real().brake(state);
    }
    bool isPrevTaskDone() const {
        AutoMode::noteUserLoopActivity();
        AutoMode::service();
        bool stats[2] = {false, false};
        if (mmL.Get_Drive_isTaskDone(1U, stats) != MMLower::Drive_RESULT::OK) {
            return true;
        }
        return !stats[0];
    }

    operator MiniR4DriveDC<1>&() const { return real(); }

private:
    uint8_t moveDegsLegacy(int16_t leftPower,
                           int16_t rightPower,
                           uint16_t degrees,
                           bool brakeOnStop,
                           bool asyncWait) const {
        const bool expectMotion = (leftPower != 0 || rightPower != 0) && degrees != 0U;
        return runBlockingDriveCmd(asyncWait, expectMotion, [&]() {
            return mmL.Set_Drive_Move_Degs(
                leftPower, rightPower, degrees, brakeOnStop, asyncWait, 1U);
        });
    }

    // Gyro-assisted MoveDegs (Config::kAutoGyroMode != Off, IMU zeroed at Auto entry):
    //  - straight (L == R)      -> MoveGyroDegs holding the virtual heading
    //  - pivot (L == -R)        -> legacy pivot then re-latch heading from IMU (StraightHold), or
    //                              TurnGyro to a snapped absolute heading (StraightHoldAndTurn)
    //  - arcs / single wheel    -> legacy, heading re-latched from IMU afterwards
    // An IMU read failure never aborts: the heading falls back to the pivot estimate.
    uint8_t moveDegsGyroAssist(int16_t leftPower,
                               int16_t rightPower,
                               uint16_t degrees,
                               bool brakeOnStop,
                               bool asyncWait) const {
        const RobotLogic::AutoMoveKind kind =
            RobotLogic::classifyAutoMove(leftPower, rightPower, degrees);

        if (kind == RobotLogic::AutoMoveKind::Straight) {
            const uint8_t res =
                MoveGyroDegs(leftPower, AutoMode::headingTargetDeg(), degrees, brakeOnStop, asyncWait);
            AutoMode::logMove("STR", leftPower, rightPower, degrees, res);
            return res;
        }

        if (kind == RobotLogic::AutoMoveKind::Pivot) {
            const int16_t estimate = RobotLogic::estimatePivotYawDeg(leftPower, rightPower, degrees);
            if (Config::kAutoGyroMode == Config::kAutoGyroStraightHoldAndTurn) {
                const int16_t target = RobotLogic::snapYawDeg(
                    RobotLogic::wrapYawDeg(static_cast<int32_t>(AutoMode::headingTargetDeg()) + estimate),
                    Config::kAutoTurnSnapStepDeg);
                int16_t power = (leftPower < 0) ? static_cast<int16_t>(-leftPower) : leftPower;
                if (power > 100) {
                    power = 100;
                }
                const uint8_t res =
                    TurnGyro(power, target, Config::kAutoTurnGyroMode, brakeOnStop, asyncWait);
                AutoMode::logMove("TURN", leftPower, rightPower, degrees, res);
                return res;
            }
            const int16_t estTarget = RobotLogic::wrapYawDeg(
                static_cast<int32_t>(AutoMode::headingTargetDeg()) + estimate);
            const uint8_t res = moveDegsLegacy(leftPower, rightPower, degrees, brakeOnStop, asyncWait);
            if (res == 0x00U && !(!asyncWait && AutoMode::latchHeadingFromImu())) {
                AutoMode::setHeadingTargetDeg(estTarget);
            }
            AutoMode::logMove("PIV", leftPower, rightPower, degrees, res);
            return res;
        }

        const uint8_t res = moveDegsLegacy(leftPower, rightPower, degrees, brakeOnStop, asyncWait);
        if (res == 0x00U && !asyncWait) {
            (void)AutoMode::latchHeadingFromImu();
        }
        AutoMode::logMove("OTH", leftPower, rightPower, degrees, res);
        return res;
    }

    template <typename SendFn>
    uint8_t sendDriveCmd(SendFn sendFn) const {
        AutoMode::noteUserLoopActivity();
        AutoMode::service();
        MMLower::Drive_RESULT result = sendFn();
        uint8_t idx = 0U;
        while (result != MMLower::Drive_RESULT::OK) {
            ::delay(2UL);
            if (!AutoMode::isRunning()) {
                return 0x09U;
            }
            result = sendFn();
            idx++;
            if (result == MMLower::Drive_RESULT::ERROR_Drive_Define) {
                if (idx >= Drive_retry_Period) {
                    return 0x07U;
                }
            } else if (result != MMLower::Drive_RESULT::OK) {
                if (idx >= Drive_retry_Period) {
                    return 0x01U;
                }
            }
        }
        AutoMode::service();
        return 0x00U;
    }

    template <typename SendFn>
    uint8_t runBlockingDriveCmd(bool asyncWait, bool expectMotion, SendFn sendFn) const {
        const uint8_t res = sendDriveCmd(sendFn);
        if (res != 0x00U || asyncWait) {
            return res;
        }

        AutoMode::beginBlockingDriveTask(expectMotion);
        while (AutoMode::isRunning()) {
            ::delay(10UL);
            bool stats[2] = {true, false};
            if (mmL.Get_Drive_isTaskDone(1U, stats) == MMLower::Drive_RESULT::OK && !stats[0]) {
                break;
            }
        }
        AutoMode::endBlockingDriveTask();

#ifdef ENABLE_DRIVEDC_BRAKE_DELAY
        if (AutoMode::isRunning()) {
            ::delay(DRIVEDC_BRAKE_DELAY_MS);
        }
#endif
        AutoMode::service();
        return 0x00U;
    }
};

struct AutoPs2Proxy {
    bool read_gamepad() const { return pollInternal(); }

    bool read_gamepad(bool motor1, uint8_t motor2) const {
        (void)motor1;
        (void)motor2;
        return pollInternal();
    }

    bool Button(uint16_t button) const {
        AutoMode::noteUserLoopActivity();
        ensurePolledOnAutoEntry();
        consumeEntryTriIfMatched(button);
        return (pressedMask_ & button) != 0U;
    }

    unsigned int ButtonDataByte() const {
        AutoMode::noteUserLoopActivity();
        ensurePolledOnAutoEntry();
        consumeEntryTriIfMatched(static_cast<uint16_t>(PSB_TRIANGLE));
        return pressedMask_;
    }

    bool NewButtonState() const {
        AutoMode::noteUserLoopActivity();
        ensurePolledOnAutoEntry();
        return (pressedMask_ ^ prevPressedMask_) != 0U;
    }

    bool NewButtonState(unsigned int button) const {
        AutoMode::noteUserLoopActivity();
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
        AutoMode::noteUserLoopActivity();
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
            pressedMask_       = 0U;
            prevPressedMask_   = 0U;
            entryTriPending_   = true;
            entryTriPollsLeft_ = 2U;
            mask |= static_cast<uint16_t>(PSB_TRIANGLE);
        } else if (entryTriPending_ && entryTriPollsLeft_ > 0U) {
            entryTriPollsLeft_--;
            prevPressedMask_ = static_cast<uint16_t>(pressedMask_ & ~static_cast<uint16_t>(PSB_TRIANGLE));
            mask |= static_cast<uint16_t>(PSB_TRIANGLE);
        } else {
            entryTriPending_   = false;
            entryTriPollsLeft_ = 0U;
            prevPressedMask_   = pressedMask_;
        }
        pressedMask_ = mask;
        return ok || entryTri || entryTriPending_;
    }

    void ensurePolledOnAutoEntry() const {
        if (AutoMode::hasEntryTrianglePending()) {
            (void)pollInternal();
        }
    }

    void consumeEntryTriIfMatched(uint16_t mask) const {
        if (entryTriPending_ && ((mask & static_cast<uint16_t>(PSB_TRIANGLE)) != 0U)) {
            entryTriPending_   = false;
            entryTriPollsLeft_ = 0U;
            AutoMode::notifyPs2TriangleMatched();
        }
    }

    mutable Ps2Frame frame_{};
    mutable uint16_t pressedMask_{0U};
    mutable uint16_t prevPressedMask_{0U};
    mutable bool     entryTriPending_{false};
    mutable uint8_t  entryTriPollsLeft_{0U};
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

    int   available() const { return AutoMode::serialAvailable(); }
    int   read() const {
        AutoMode::noteUserLoopActivity();
        return AutoMode::serialRead();
    }
    int   peek() const { return AutoMode::serialPeek(); }
    void  flush() const { ::Serial.flush(); }
    explicit operator bool() const { return static_cast<bool>(::Serial); }
    operator decltype(::Serial)&() const { return ::Serial; }
};

inline constexpr AutoMiniR4Proxy MiniR4{};
inline constexpr AutoSerialProxy Serial{};

// ----------------------------------------------------------------------------
// Optional C++ mission helpers (alternative to pasting MATRIXblock code).
// They go through the same proxies, so stall detection, X/ESTOP abort and WDT keep working.
// turnTo()/turnBy() use the onboard IMU: set Config::kAutoGyroMode to 1 or 2 in include/config.h
// (with Off, turnBy() falls back to an open-loop pivot and turnTo() returns 0x01).
//
// Example (inside setup() in the paste zone, replacing MoveDegs/delay chains):
//   driveStraight(-50, 415);          // reverse 415 wheel deg, holds heading
//   servoTo(2, 165);                  // RC2 to 165 deg, waits 300 ms
//   turnBy(90);                       // pivot ~90 deg (sign: Config::kAutoPivotYawSign)
//   turnTo(0);                        // back to the heading at Auto start
// ----------------------------------------------------------------------------

// Current heading (deg) from the IMU, or the held virtual heading if the read fails.
inline int16_t headingNow() {
    int16_t yaw = 0;
    return AutoMode::readYawDeg(yaw) ? yaw : AutoMode::headingTargetDeg();
}

// Drives both wheels `wheelDegs` encoder degrees; negative power reverses. Holds heading when the
// gyro is active, otherwise plain MoveDegs.
inline uint8_t driveStraight(int16_t power, uint16_t wheelDegs, bool brake = true) {
    if (AutoMode::gyroAssistActive()) {
        return MiniR4.DriveDC.MoveGyroDegs(power, AutoMode::headingTargetDeg(), wheelDegs, brake);
    }
    return MiniR4.DriveDC.MoveDegs(power, power, wheelDegs, brake);
}

// Pivots to an absolute heading (-180..180, 0 = heading at Auto start). Needs the gyro active.
inline uint8_t turnTo(int16_t absoluteYawDeg, int16_t power = 40) {
    if (!AutoMode::gyroAssistActive()) {
        return 0x01U;
    }
    const int16_t p = (power < 0) ? static_cast<int16_t>(-power) : power;
    return MiniR4.DriveDC.TurnGyro(p > 100 ? static_cast<int16_t>(100) : p,
                                   RobotLogic::wrapYawDeg(absoluteYawDeg),
                                   Config::kAutoTurnGyroMode,
                                   true);
}

// Pivots by a relative angle (+ = Config::kAutoPivotYawSign direction). Gyro off -> open-loop pivot.
inline uint8_t turnBy(int16_t deltaDeg, int16_t power = 40) {
    if (AutoMode::gyroAssistActive()) {
        return turnTo(static_cast<int16_t>(AutoMode::headingTargetDeg() + deltaDeg), power);
    }
    const int32_t mag32 = ((deltaDeg < 0) ? -static_cast<int32_t>(deltaDeg) : deltaDeg) *
                          static_cast<int32_t>(Config::kAutoPivotWheelDegPerYawDegX100) / 100L;
    const uint16_t wheelDegs = static_cast<uint16_t>(mag32 > 65535L ? 65535L : mag32);
    const int16_t  p         = (power < 0) ? static_cast<int16_t>(-power) : power;
    const bool     rightFwd  = ((deltaDeg > 0) == (Config::kAutoPivotYawSign > 0));
    return MiniR4.DriveDC.MoveDegs(rightFwd ? static_cast<int16_t>(-p) : p,
                                   rightFwd ? p : static_cast<int16_t>(-p),
                                   wheelDegs,
                                   true);
}

// Moves RC1..RC4 to `deg`, then waits `settleMs` (abort-safe delay). Returns false for bad channel.
inline bool servoTo(uint8_t channel, uint16_t deg, uint16_t settleMs = 300U) {
    bool ok = true;
    switch (channel) {
        case 1U: ok = MiniR4.RC1.setAngle(deg); break;
        case 2U: ok = MiniR4.RC2.setAngle(deg); break;
        case 3U: ok = MiniR4.RC3.setAngle(deg); break;
        case 4U: ok = MiniR4.RC4.setAngle(deg); break;
        default: return false;
    }
    if (settleMs > 0U) {
        ::delay(settleMs);
    }
    return ok;
}

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
