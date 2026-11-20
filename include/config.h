#pragma once

#include <stdint.h>

#include "user_settings.h" // ค่าที่ปรับบ่อยอยู่ในไฟล์นั้น; ไฟล์นี้เก็บพิน/คาบเวลา/ขีดจำกัดฮาร์ดแวร์

// ============================================================================
// ค่าตั้งของ MATRIX Mini R4 + Arduino UNO R4 WiFi
// ============================================================================
// การจับคู่ฮาร์ดแวร์ (ตายตัวตามบอร์ดแคริเออร์ MATRIX Mini R4):
//   M1 / M2      : มอเตอร์ DC ขับเคลื่อน (STM32 โคโปรเซสเซอร์ ผ่าน UART D8/D9 @ 57600)
//   RC1..RC4     : เซอร์โว (RC1=เสริม, RC2=แขนสำรอง D-Pad, RC3=หน้าซ้าย, RC4=หน้าขวา)
//   BTN_UP/DOWN  : ปุ่มบนบอร์ดเหนือจอ OLED (STM32)
//   OLED (0x3D)  : จอ OLED I2C 128x32 ในตัว (Wire1)
//   LED (D7)     : RGB LED WS2812B ในตัว 2 ดวง
//   Buzzer (D6)  : บัซเซอร์เพียโซในตัว
//   PS2 / MJ2    : สาย 4p -> DIGITAL 2 (D5/D4), สาย 2p -> DIGITAL 3 (D12/D11)
// ============================================================================

namespace Config {

// --- จังหวะควบคุมและ Watchdog -------------------------------------------------
constexpr uint32_t kSerialBaud               = 115200UL;
constexpr uint32_t kControlPeriodMs          = 25UL;    // ลูปควบคุม 40 Hz
constexpr uint32_t kButtonPollPeriodMs       = 25UL;    // อ่านปุ่ม UP/DOWN สลับกัน
constexpr uint32_t kButtonUnhealthyPollPeriodMs = 250UL; // ลดความถี่อ่านปุ่มเมื่อการสื่อสารไม่ปกติ
constexpr uint32_t kLowerHealthIntervalMs    = 500UL;   // คาบตรวจสุขภาพ STM32 ตอนว่าง
constexpr uint32_t kLowerFastHealthMs        = 100UL;   // คาบตรวจ STM32 แบบเร็วตอนเคลื่อนที่/กำลังฟื้น
constexpr uint32_t kLowerResyncConfigMs      = 2500UL;  // ส่งค่าตั้งซ้ำเป็นระยะ กันไฟตกเงียบ (brownout)
constexpr uint32_t kOledRefreshPeriodMs      = 180UL;   // จังหวะรีเฟรช UI แบบไม่บล็อก
constexpr uint32_t kWireTimeoutUs            = 60000UL; // 60ms: >2.5 เท่าของการส่ง SSD1306 255 ไบต์ และ << WDT 3 วินาที
constexpr uint32_t kMotorKeepaliveMs         = 500UL;   // ส่งคำสั่งมอเตอร์ซ้ำเป็นระยะ
constexpr uint32_t kServoKeepaliveMs         = 1000UL;
constexpr uint32_t kFailsafeRefreshMs        = 250UL;   // ย้ำคำสั่งหยุดโดยไม่ท่วม UART
constexpr uint32_t kCommRetryIntervalMs      = 25UL;    // ช่วงห่างต่ำสุดระหว่างการส่งซ้ำไป STM32
constexpr uint32_t kCommUnhealthyRetryIntervalMs = 100UL; // ช่วงห่างการส่งซ้ำเมื่อการสื่อสารไม่ปกติ
constexpr uint32_t kTelemetryHeartbeatMs     = 500UL;
constexpr uint32_t kWdtTimeoutMs             = 3000UL;  // <= 5592 ms บน RA4M1

// --- มอเตอร์ขับเคลื่อน (M1 ซ้าย, M2 ขวา; ตรงกับ matrix_R3) -------------------
constexpr uint8_t  kMotorLeftId            = 1U;
constexpr uint8_t  kMotorRightId           = 2U;
constexpr int16_t  kMotorMinPower          = -100;
constexpr int16_t  kMotorMaxPower          = 100;

// --- พารามิเตอร์ฟิสิกส์ Kinodynamic และไฟฟ้า-กล -----------------------------

// --- ที่เก็บค่าตั้งใน EEPROM แบบไม่ลบเมื่อดับไฟ (เลี่ยงช่วง IMU cal ที่ 50..73) ---
constexpr uint16_t kEepromConfigAddr       = 128U;    // ออฟเซ็ตปลอดภัย 0x80 ใน data flash ของ Renesas RA4M1
constexpr uint8_t  kEepromMagic0           = 'M';
constexpr uint8_t  kEepromMagic1           = 'R';
constexpr uint8_t  kEepromVersion          = 1U;

// --- จอยไร้สาย PS2 / MJ2 (D2 + D3) ------------------------------------------
constexpr uint8_t  kPs2ClkPin              = 11U;     // D3 ขวา
constexpr uint8_t  kPs2CmdPin              = 4U;      // D2 ขวา
constexpr uint8_t  kPs2AttPin              = 12U;     // D3 ซ้าย
constexpr uint8_t  kPs2DatPin              = 5U;      // D2 ซ้าย
constexpr uint8_t  kPs2ExpectedMode        = 0x73U;   // ID โหมด DualShock analog (ไฟแดง)
constexpr uint8_t  kPs2ExpectedSync        = 0x5AU;   // ไบต์ซิงก์ของจอย
constexpr uint8_t  kPs2StableFrames        = 3U;      // จำนวนเฟรมที่ถูกต้องก่อนถือว่าลิงก์พร้อม
constexpr uint8_t  kPs2FailsafeBadFrames   = 4U;      // ทนเฟรมเสียได้ ~100 ms
constexpr uint8_t  kPs2ModeResetFrames     = 3U;      // เฟรม 0x41 ติดกันกี่เฟรม -> เชื่อมต่อใหม่เร็ว
constexpr uint16_t kPs2PersistentLossFrames = 120U;   // 120 * 25 ms = 3000 ms -> ถือว่าหลุด
constexpr uint32_t kPs2DisconnectMs        = 3000UL;
constexpr uint32_t kPs2ReconnectPeriodMs   = 500UL;   // ช่วงห่างการลองต่อใหม่แบบไม่บล็อก หลังจบรอบ
constexpr uint32_t kPs2BootTimeoutMs       = 400UL;   // งบเวลาเริ่มต้นจอยตอนบูตแบบมีขอบเขต
constexpr uint8_t  kPs2InitBurstAttempts   = 3U;      // จำนวนครั้งลองต่อแบบ burst (ใช้ได้ใน Ps2Driver::tryConnect; ตอนรันใช้ 1 รอบแบบไม่บล็อก)
constexpr uint8_t  kPs2ClockHalfPeriodUs   = 8U;      // สัญญาณนาฬิกา bit-bang SPI ~62 kHz
constexpr uint8_t  kPs2ByteGapUs           = 20U;     // ช่วงพักระหว่างไบต์
constexpr uint8_t  kPs2PacketGapUs         = 50U;     // ช่วง CS สูงระหว่างทรานแซกชัน
constexpr uint8_t  kPs2AttSettleUs         = 12U;     // เวลาตั้ง CS ต่ำก่อนสัญญาณนาฬิกาแรก
constexpr uint32_t kPs2InitRetryGapMs      = 12U;     // หน่วงระหว่างการลองเริ่มต้นแบบ burst

// --- เซอร์โว (RC1..RC4; RC3+RC4 จับคู่ที่ L1=ลง, R1=ขึ้น, Square=ดันพื้น) ---
constexpr int16_t  kServoMinDeg            = 0;
constexpr int16_t  kServoMaxDeg            = 180;
constexpr int16_t  kRc2MinDeg              = 0;
constexpr int16_t  kRc2MaxDeg              = 180;
constexpr uint32_t kArmStepIntervalMs      = 50UL;
constexpr uint32_t kServoBootStaggerMs     = 60UL;    // ลดกระแสพุ่งตอนเริ่มระบบ

// --- กำลังไฟ / อะแดปเตอร์ DC (รองรับอะแดปเตอร์ 9V-18V หรือแบต 2S-4S) -------
constexpr float    kPowerFullVolt          = 19.5f;   // <= 25.5V (uint8 หน่วยหนึ่งส่วนสิบ)
constexpr float    kPowerCutoffVolt        = 6.0f;    // ต่ำกว่า 6.4V คือระดับฉุกเฉิน เพื่อให้ซอฟต์แวร์ลดกำลังตามแรงดันตกทำงานก่อน
constexpr float    kPowerAlarmVolt         = 7.2f;
constexpr uint16_t kLowVoltWarnMv          = 7200U;   // เกณฑ์ต่ำ (ปลอดภัยสำหรับ 2S / 9V)
constexpr uint16_t kBattCritVoltMv         = 6800U;   // เกณฑ์วิกฤต
constexpr uint16_t kBattEmergVoltMv        = 6400U;   // เกณฑ์ฉุกเฉิน
constexpr uint16_t kBattHysteresisMv       = 350U;    // ฮิสเตอรีซิสตอนออกจากระดับ เหนือเกณฑ์ตอนเข้า
constexpr uint16_t kExtPowerPresentMv      = 5500U;   // ไฟจาก USB อย่างเดียวอ่าน VBAT ได้ < 5.5V
constexpr uint8_t  kBattLowSagPct          = 83U;     // <= 83% ของค่าฐานตอนพัก -> ระดับต่ำ
constexpr uint8_t  kBattCritSagPct         = 76U;     // <= 76% ของค่าฐานตอนพัก -> ระดับวิกฤต
constexpr uint8_t  kBattEmergSagPct        = 69U;     // <= 69% ของค่าฐานตอนพัก -> ระดับฉุกเฉิน
constexpr uint8_t  kBattEmaDownAlphaPct    = 35U;     // ตอบสนองเร็วเมื่อแรงดันตก
constexpr uint8_t  kBattEmaUpAlphaPct      = 8U;      // ตัวกรองฟื้นตัวช้า
constexpr uint8_t  kBattOffDebounceTicks   = 3U;      // ดีบาวซ์การเสียบ/ถอด USB อย่างเดียว
constexpr uint8_t  kBattInvalidFallbackCnt = 6U;      // จำนวนครั้งที่อ่าน ADC/UART พลาดก่อนถอยกลับค่าสำรอง
constexpr uint32_t kBattRecoveryHoldMs     = 1500UL;  // เวลาที่ต้องนิ่งก่อนยกระดับขึ้นหนึ่งขั้น
constexpr int16_t  kBattLowAccelSlew       = 15;
constexpr int16_t  kBattCritAccelSlew      = 9;
constexpr int16_t  kBattEmergAccelSlew     = 6;
constexpr int16_t  kBattLowDecelSlew       = 32;
constexpr int16_t  kBattCritDecelSlew      = 22;
constexpr int16_t  kBattEmergDecelSlew     = 18;

// --- ควบคุมสำรองด้วยมือผ่าน Serial -------------------------------------------
constexpr int16_t  kSerialMinSpeed         = 20;
constexpr int16_t  kSerialSpeedStep        = 15;
constexpr uint32_t kSerialKeyHoldMs        = 650UL;   // คร่อมช่องว่างของการกดคีย์ซ้ำของระบบปฏิบัติการ
constexpr uint32_t kSerialDirectHoldMs     = 2500UL;  // หน้าต่างความปลอดภัยของคำสั่ง `m L R`
constexpr uint32_t kLineTimeoutMs          = 1500UL;  // เวลาหมดอายุของคำสั่งแบบบรรทัด

// --- คอมโบอัปโหลดบนบอร์ด (BTN_UP + BTN_DOWN) และการกดค้างเริ่ม Auto ---------
constexpr uint32_t kButtonDebounceMs       = 30UL;
constexpr uint32_t kUploadComboHoldMs      = 2500UL;
constexpr uint32_t kAutoHoldBarDelayMs     = 250UL;   // หน่วงก่อนแสดงแถบ BTN_UP->AUTO บน OLED
constexpr uint32_t kAutoOledRefreshMs      = 250UL;   // รีเฟรช OLED นาฬิกาจับเวลาสดระหว่าง Auto
constexpr uint32_t kAutoStallPollMs        = 200UL;   // คาบอ่านความคืบหน้า encoder ระหว่าง DriveDC แบบบล็อก

// --- ดัชนีความน่าจะเป็นความเสถียรสด (0..100%) -------------------------------
constexpr uint8_t  kStabilityLinkMaxPen      = 35U;
constexpr uint8_t  kStabilityLinkDropPen     = 25U;
constexpr uint8_t  kStabilityIdleWaitPen     = 5U;
constexpr uint8_t  kStabilitySerialLinkPen   = 2U;
constexpr uint8_t  kStabilitySyncWaitPen     = 10U;
constexpr uint8_t  kStabilityGlitchStep      = 8U;
constexpr uint8_t  kStabilityCommMaxPen      = 30U;
constexpr uint8_t  kStabilityCommBusMaxPen   = 25U;
constexpr uint8_t  kStabilityCommStepPen     = 10U;
constexpr uint8_t  kStabilityHwFaultPen      = 10U;
constexpr uint8_t  kStabilityPowerMaxPen     = 45U;
constexpr uint8_t  kStabilityUsbNoBattPen    = 18U;
constexpr uint8_t  kStabilityPowerLowPen     = 12U;
constexpr uint8_t  kStabilityPowerCritPen    = 20U;
constexpr uint8_t  kStabilityPowerEmergPen   = 30U;
constexpr uint8_t  kStabilityLoopMaxPen      = 20U;
constexpr uint16_t kStabilityJitterSlackMs   = 5U;
constexpr uint8_t  kStabilityRecoverStep     = 2U;
constexpr uint8_t  kStabilityTipMaxPen       = 15U;     // แขน RC2 สูงสุด + ล้อเต็มกำลัง
constexpr uint8_t  kStabilityStallPen        = 15U;     // ล้อติด (stall detector ทำงาน)
constexpr uint8_t  kStabilityDriveMaxPen     = 25U;

} // namespace Config
