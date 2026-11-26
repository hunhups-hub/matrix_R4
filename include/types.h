#pragma once

#include <stdint.h>
#include "config.h"

enum class LinkState : uint8_t {
    Disconnected = 0,
    Failsafe,
    Stabilizing,
    Operational
};

enum class ControlSource : uint8_t {
    None = 0,
    Ps2,
    SerialCmd,
    Auto
};

enum class SystemState : uint8_t {
    Ready = 0,
    Estop
};

struct Ps2Frame {
    bool    linkOk   = false;
    uint8_t modeId   = 0xFF;
    uint8_t syncByte = 0xFF;
    uint8_t buttons1 = 0xFF;
    uint8_t buttons2 = 0xFF;
    uint8_t rx       = 128;
    uint8_t ry       = 128;
    uint8_t lx       = 128;
    uint8_t ly       = 128;

    bool up          = false;
    bool right       = false;
    bool down        = false;
    bool left        = false;
    bool l1          = false;
    bool r1          = false;
    bool l2          = false;
    bool r2          = false;
    bool l3          = false;
    bool r3          = false;
    bool triangle    = false;
    bool circle      = false;
    bool cross       = false;
    bool square      = false;
    bool select      = false;
    bool start       = false;
};

enum class BatteryTier : uint8_t {
    Normal = 0,
    Low,
    Critical,
    Emergency
};

struct MotorPowerProfile {
    int16_t maxPowerCap   = Config::kMotorMaxPower;
    int16_t accelSlewStep = Config::kMotorAccelSlewStep;
    int16_t decelSlewStep = Config::kMotorDecelSlewStep;
    uint8_t compPercent   = 100U;
    bool    slewLimited   = false;
};

struct DriveOutput {
    int16_t left  = 0;
    int16_t right = 0;
};

struct ServoTargets {
    int16_t rc1Deg = Config::kRc1InitDeg;
    int16_t rc2Deg = Config::kRc2InitDeg;
    int16_t rc3Deg = Config::kRc3InitDeg;
};

struct FrontArmPushState {
    bool    active      = false;
    bool    prevSquare  = false;
    int16_t savedRc3Deg = Config::kRc3InitDeg;
};

struct ArmPosePresetState {
    bool     dropActive = false;
    bool     prevCircle = false;
    bool     prevTriangle = false;
    bool     releasePending = false;  // Circle: RC2 already moved, RC1 release waiting
    uint32_t releaseAtMs = 0;         // time (ms) when RC1 should be released
};

struct PersistentConfig {
    bool   leftReversed     = Config::kMotorLeftReverseInit;
    bool   rightReversed    = Config::kMotorRightReverseInit;
    bool   precisionMode    = false;
    bool   rc2SmoothEnabled = Config::kRc2SmoothDefault;
    int8_t motorTrimPct     = 0;
};

struct ButtonEvents {
    bool  upPressed          = false;
    bool  downPressed        = false;
    bool  shortUp            = false;
    bool  shortDown          = false;
    bool  downPressEdge      = false; // Debounced BTN_DOWN press (fires immediately, not on release)
    bool  comboHolding       = false;
    float uploadHoldRatio    = 0.0f;
    bool  uploadRequested    = false;
    bool  autoHolding        = false;
    float autoHoldRatio      = 0.0f;
    bool  autoStartRequested = false;
};

struct SerialDriveCommand {
    bool    active      = false;
    int16_t targetLeft  = 0;
    int16_t targetRight = 0;
};

struct SystemSnapshot {
    SystemState   sysState       = SystemState::Ready;
    LinkState     ps2Link        = LinkState::Disconnected;
    uint8_t       ps2Mode        = 0xFF;
    uint8_t       ps2Sync        = 0xFF;
    uint8_t       ps2Ly          = 128;
    uint8_t       ps2Rx          = 128;
    bool          ps2Glitching   = false;
    ControlSource activeSource   = ControlSource::None;
    bool          precisionMode  = false;
    bool          reverseDrive   = false;
    bool          leftReversed     = Config::kMotorLeftReverseInit;
    bool          rightReversed    = Config::kMotorRightReverseInit;
    int8_t        motorTrimPct     = 0;
    bool          rc2SmoothEnabled = Config::kRc2SmoothDefault;
    bool          holdBrakeActive  = false;
    int16_t       serialSpeedPct   = Config::kSerialDefaultSpeed;
    DriveOutput   drive{};
    ServoTargets  servos{};
    uint8_t       comScalePct    = 100U;
    float         battVolt       = 0.0f;
    uint8_t       battPct        = 0U;
    bool          extPower       = false;
    BatteryTier   batTier        = BatteryTier::Normal;
    int16_t       powerCapPct    = Config::kMotorMaxPower;
    bool          lowVoltWarn    = false;
    bool          commHealthy    = true;
    uint8_t       stabilityPct   = 100U;
    uint16_t      loopJitterMs   = 0U;
    uint8_t       linkStabState  = 0U; // 0: '+' OK, 1: '~' wait/mild, 2: '!' fault
    uint8_t       powerStabState = 0U;
    uint8_t       commStabState  = 0U;
    uint8_t       loopStabState  = 0U;
    uint8_t       driveStabState = 0U; // D: arm-raised tip risk + wheel stall
    bool          wdtResetBoot   = false;
    bool          oledOk         = false;
    uint8_t       oledAddr       = 0x3D;
    uint8_t       oledErr3d      = 0xFF;
    uint8_t       oledErr3c      = 0xFF;
    bool          comboHolding   = false;
    float         uploadHoldPct  = 0.0f;
    bool          autoHolding    = false;
    float         autoHoldPct    = 0.0f;
    uint32_t      autoElapsedMs  = 0UL;
    uint8_t       servoRangeErr  = 0U;
};
