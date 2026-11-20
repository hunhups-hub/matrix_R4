#pragma once

#include <stdint.h>

#include "user_settings.h" // ค่าที่ปรับบ่อยอยู่ในไฟล์นั้น; ไฟล์นี้เก็บพิน/คาบเวลา/ขีดจำกัดฮาร์ดแวร์

// ============================================================================
// MATRIX Mini R4 + Arduino UNO R4 WiFi Configuration
// ============================================================================
// Hardware Mapping (fixed by MATRIX Mini R4 Carrier Shield):
//   M1 / M2      : Drive DC Motors (STM32 co-processor via D8/D9 UART @ 57600)
//   RC1..RC4     : Servos (RC1=Aux, RC2=Spare D-Pad, RC3=Front L, RC4=Front R)
//   BTN_UP/DOWN  : Onboard top buttons above OLED (STM32)
//   OLED (0x3D)  : Built-in 128x32 I2C OLED (Wire1)
//   LED (D7)     : Built-in 2x WS2812B RGB LEDs
//   Buzzer (D6)  : Built-in piezo buzzer
//   PS2 / MJ2    : 4p wire -> DIGITAL 2 (D5/D4), 2p wire -> DIGITAL 3 (D12/D11)
// ============================================================================

namespace Config {

// --- Control Cadence & Watchdog ---------------------------------------------
constexpr uint32_t kSerialBaud               = 115200UL;
constexpr uint32_t kControlPeriodMs          = 25UL;    // 40 Hz control loop
constexpr uint32_t kButtonPollPeriodMs       = 25UL;    // Alternating UP/DOWN poll
constexpr uint32_t kButtonUnhealthyPollPeriodMs = 250UL; // Throttled button polling when comm is unhealthy
constexpr uint32_t kLowerHealthIntervalMs    = 500UL;   // Idle STM32 health check period
constexpr uint32_t kLowerFastHealthMs        = 100UL;   // Fast STM32 check while moving/healing
constexpr uint32_t kLowerResyncConfigMs      = 2500UL;  // Periodic config defense vs silent brownout
constexpr uint32_t kOledRefreshPeriodMs      = 180UL;   // Non-blocking UI cadence
constexpr uint32_t kWireTimeoutUs            = 60000UL; // 60ms: >2.5x 255-byte SSD1306 transfer, << 3s WDT
constexpr uint32_t kMotorKeepaliveMs         = 500UL;   // Periodic actuator re-sync
constexpr uint32_t kServoKeepaliveMs         = 1000UL;
constexpr uint32_t kFailsafeRefreshMs        = 250UL;   // Re-assert stop without flooding UART
constexpr uint32_t kCommRetryIntervalMs      = 25UL;    // Min gap between STM32 write retries
constexpr uint32_t kCommUnhealthyRetryIntervalMs = 100UL; // Throttled retry gap when comm is unhealthy
constexpr uint32_t kTelemetryHeartbeatMs     = 500UL;
constexpr uint32_t kWdtTimeoutMs             = 3000UL;  // <= 5592 ms on RA4M1

// --- Drive Motors (M1 Left, M2 Right; matches matrix_R3) --------------------
constexpr uint8_t  kMotorLeftId            = 1U;
constexpr uint8_t  kMotorRightId           = 2U;
constexpr int16_t  kMotorMinPower          = -100;
constexpr int16_t  kMotorMaxPower          = 100;

// --- Physics Kinodynamic & Electromechanical Parameters --------------------

// --- Non-Volatile EEPROM Config Storage (avoids IMU cal at 50..73) ----------
constexpr uint16_t kEepromConfigAddr       = 128U;    // Safe offset 0x80 in Renesas RA4M1 data flash
constexpr uint8_t  kEepromMagic0           = 'M';
constexpr uint8_t  kEepromMagic1           = 'R';
constexpr uint8_t  kEepromVersion          = 1U;

// --- PS2 / MJ2 Wireless Controller (D2 + D3) --------------------------------
constexpr uint8_t  kPs2ClkPin              = 11U;     // D3 Right
constexpr uint8_t  kPs2CmdPin              = 4U;      // D2 Right
constexpr uint8_t  kPs2AttPin              = 12U;     // D3 Left
constexpr uint8_t  kPs2DatPin              = 5U;      // D2 Left
constexpr uint8_t  kPs2ExpectedMode        = 0x73U;   // DualShock analog red-mode ID
constexpr uint8_t  kPs2ExpectedSync        = 0x5AU;   // Controller sync byte
constexpr uint8_t  kPs2StableFrames        = 3U;      // Valid frames to arm link
constexpr uint8_t  kPs2FailsafeBadFrames   = 4U;      // ~100 ms glitch tolerance
constexpr uint8_t  kPs2ModeResetFrames     = 3U;      // Consecutive 0x41 frames -> fast reconnect
constexpr uint16_t kPs2PersistentLossFrames = 120U;   // 120 * 25 ms = 3000 ms -> Disconnected
constexpr uint32_t kPs2DisconnectMs        = 3000UL;
constexpr uint32_t kPs2ReconnectPeriodMs   = 500UL;   // Non-blocking retry gap after finish
constexpr uint32_t kPs2BootTimeoutMs       = 400UL;   // Bounded boot init budget
constexpr uint8_t  kPs2InitBurstAttempts   = 3U;      // Multi-attempt burst count (available to Ps2Driver::tryConnect; runtime uses 1 non-blocking cycle)
constexpr uint8_t  kPs2ClockHalfPeriodUs   = 8U;      // ~62 kHz bit-bang SPI clock
constexpr uint8_t  kPs2ByteGapUs           = 20U;     // Inter-byte settle gap
constexpr uint8_t  kPs2PacketGapUs         = 50U;     // CS high gap between transactions
constexpr uint8_t  kPs2AttSettleUs         = 12U;     // CS low setup time before first clock
constexpr uint32_t kPs2InitRetryGapMs      = 12U;     // Delay between init burst attempts

// --- Servos (RC1..RC4; RC3+RC4 paired on L1=Down, R1=Up, Square=Push Ground) ---
constexpr int16_t  kServoMinDeg            = 0;
constexpr int16_t  kServoMaxDeg            = 180;
constexpr int16_t  kRc2MinDeg              = 0;
constexpr int16_t  kRc2MaxDeg              = 180;
constexpr uint32_t kArmStepIntervalMs      = 50UL;
constexpr uint32_t kServoBootStaggerMs     = 60UL;    // Reduce startup current spike

// --- Power / DC Adapter (Supports 9V-18V DC Adapter or 2S-4S Pack) ----------
constexpr float    kPowerFullVolt          = 19.5f;   // <= 25.5V (uint8 tenths)
constexpr float    kPowerCutoffVolt        = 6.0f;    // Below 6.4V Emergency floor so software sag throttling triggers first
constexpr float    kPowerAlarmVolt         = 7.2f;
constexpr uint16_t kLowVoltWarnMv          = 7200U;   // Floor Low threshold (2S / 9V safe)
constexpr uint16_t kBattCritVoltMv         = 6800U;   // Floor Critical threshold
constexpr uint16_t kBattEmergVoltMv        = 6400U;   // Floor Emergency threshold
constexpr uint16_t kBattHysteresisMv       = 350U;    // Exit hysteresis above enter threshold
constexpr uint16_t kExtPowerPresentMv      = 5500U;   // USB-only reads < 5.5V on VBAT
constexpr uint8_t  kBattLowSagPct          = 83U;     // <= 83% of resting baseline -> Low
constexpr uint8_t  kBattCritSagPct         = 76U;     // <= 76% of resting baseline -> Critical
constexpr uint8_t  kBattEmergSagPct        = 69U;     // <= 69% of resting baseline -> Emergency
constexpr uint8_t  kBattEmaDownAlphaPct    = 35U;     // Fast reaction to voltage sag
constexpr uint8_t  kBattEmaUpAlphaPct      = 8U;      // Slow filter recovery
constexpr uint8_t  kBattOffDebounceTicks   = 3U;      // Debounce USB-only / unplug
constexpr uint8_t  kBattInvalidFallbackCnt = 6U;      // Missed ADC/UART reads before fallback
constexpr uint32_t kBattRecoveryHoldMs     = 1500UL;  // Stable hold required to step up a tier
constexpr int16_t  kBattLowAccelSlew       = 15;
constexpr int16_t  kBattCritAccelSlew      = 9;
constexpr int16_t  kBattEmergAccelSlew     = 6;
constexpr int16_t  kBattLowDecelSlew       = 32;
constexpr int16_t  kBattCritDecelSlew      = 22;
constexpr int16_t  kBattEmergDecelSlew     = 18;

// --- Serial Backup Manual Control -------------------------------------------
constexpr int16_t  kSerialMinSpeed         = 20;
constexpr int16_t  kSerialSpeedStep        = 15;
constexpr uint32_t kSerialKeyHoldMs        = 650UL;   // Bridges OS key-repeat gap
constexpr uint32_t kSerialDirectHoldMs     = 2500UL;  // Safety window for `m L R`
constexpr uint32_t kLineTimeoutMs          = 1500UL;  // Expiration window for line commands

// --- Onboard Upload Combo (BTN_UP + BTN_DOWN) & Auto Start Hold ------------
constexpr uint32_t kButtonDebounceMs       = 30UL;
constexpr uint32_t kUploadComboHoldMs      = 2500UL;
constexpr uint32_t kAutoHoldBarDelayMs     = 250UL;   // Hold delay before showing BTN_UP->AUTO bar on OLED
constexpr uint32_t kAutoOledRefreshMs      = 250UL;   // Live stopwatch OLED refresh during Auto
constexpr uint32_t kAutoStallPollMs        = 200UL;   // Encoder progress poll period during blocking DriveDC

// --- Auto Gyro Assist mode names (value chosen in user_settings.h: kAutoGyroMode) ---
constexpr uint8_t  kAutoGyroOff                 = 0U;
constexpr uint8_t  kAutoGyroStraightHold        = 1U;
constexpr uint8_t  kAutoGyroStraightHoldAndTurn = 2U;

// --- Live Stability Probability Index (0..100%) ----------------------------
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
