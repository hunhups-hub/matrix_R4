#pragma once

#include <stdint.h>

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
constexpr uint32_t kBatteryPollPeriodMs      = 100UL;   // STM32 voltage read period (active)
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
constexpr bool     kMotorLeftReverseInit   = false;   // Inverted from MATRIX default to match chassis front
constexpr bool     kMotorRightReverseInit  = true;
constexpr int16_t  kMotorMinPower          = -100;
constexpr int16_t  kMotorMaxPower          = 100;
constexpr int16_t  kPrecisionMaxPower      = 55;      // Slow/precision mode limit
constexpr int16_t  kMotorAccelSlewStep     = 25;      // 0 -> 100% in ~100 ms
constexpr int16_t  kMotorDecelSlewStep     = 50;      // 100% -> 0 in ~50 ms

// --- PS2 / MJ2 Wireless Controller (D2 + D3) --------------------------------
constexpr uint8_t  kPs2ClkPin              = 11U;     // D3 Right
constexpr uint8_t  kPs2CmdPin              = 4U;      // D2 Right
constexpr uint8_t  kPs2AttPin              = 12U;     // D3 Left
constexpr uint8_t  kPs2DatPin              = 5U;      // D2 Left
constexpr uint8_t  kPs2ExpectedMode        = 0x73U;   // DualShock analog red-mode ID
constexpr uint8_t  kPs2ExpectedSync        = 0x5AU;   // Controller sync byte
constexpr uint8_t  kStickDeadzoneRaw       = 18U;     // ~14% center deadband (127/128)
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

// --- Servos (RC1..RC4; RC3+RC4 paired on L1=Front Down, R1=Front Up) ----------
constexpr int16_t  kServoMinDeg            = 0;
constexpr int16_t  kServoMaxDeg            = 180;
constexpr int16_t  kRc1InitDeg             = 0;       // Aux servo default
constexpr int16_t  kRc1L2Deg               = 0;
constexpr int16_t  kRc1R2Deg               = 180;
constexpr int16_t  kRc2InitDeg             = 90;      // Spare/Arm RC2 (D-Pad UP/DOWN)
constexpr int16_t  kRc2MinDeg              = 0;
constexpr int16_t  kRc2MaxDeg              = 180;
constexpr int16_t  kRc2StepDeg             = 10;      // 10 deg / 50 ms (~200 deg/s)
constexpr bool     kRc2InvertDir           = true;    // Invert RC2: UP/Fwd decreases deg, DOWN/Back increases deg
constexpr uint32_t kArmStepIntervalMs      = 50UL;
constexpr int16_t  kRc3InitDeg             = 158;     // Front L arm stowed/up (mirror of RC4; clamped 0..180)
constexpr int16_t  kRc3L1Deg               = 5;       // L1 Push down (lower deg = deeper; min 0)
constexpr int16_t  kRc3R1Deg               = 160;     // R1 Lift / stow up
constexpr int16_t  kRc4InitDeg             = 140;      // Front R arm stowed/up
constexpr int16_t  kRc4L1Deg               = 175;     // L1 Push down (higher deg = deeper; max 180)
constexpr int16_t  kRc4R1Deg               = 140;      // R1 Lift / stow up
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
constexpr bool     kEnableBatterySagThrottle = false; // Keep 100% motor power & slew even when battery is low
constexpr int16_t  kBattLowMaxPower        = 72;      // Power caps & slew steps per tier (when sag throttle enabled)
constexpr int16_t  kBattCritMaxPower       = 50;
constexpr int16_t  kBattEmergMaxPower      = 35;
constexpr int16_t  kBattLowAccelSlew       = 15;
constexpr int16_t  kBattCritAccelSlew      = 9;
constexpr int16_t  kBattEmergAccelSlew     = 6;
constexpr int16_t  kBattLowDecelSlew       = 32;
constexpr int16_t  kBattCritDecelSlew      = 22;
constexpr int16_t  kBattEmergDecelSlew     = 18;
constexpr uint8_t  kBattCompMaxPct         = 115U;    // Bounded torque compensation cap

// --- Serial Backup Manual Control -------------------------------------------
constexpr int16_t  kSerialDefaultSpeed     = 65;      // Safe bench speed (15..100%)
constexpr int16_t  kSerialMinSpeed         = 20;
constexpr int16_t  kSerialSpeedStep        = 15;
constexpr uint32_t kSerialKeyHoldMs        = 650UL;   // Bridges OS key-repeat gap
constexpr uint32_t kSerialDirectHoldMs     = 2500UL;  // Safety window for `m L R`
constexpr uint32_t kLineTimeoutMs          = 1500UL;  // Expiration window for line commands

// --- Onboard Upload Combo (BTN_UP + BTN_DOWN) -------------------------------
constexpr uint32_t kButtonDebounceMs       = 30UL;
constexpr uint32_t kUploadComboHoldMs      = 2500UL;

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

} // namespace Config
