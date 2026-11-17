#pragma once

#include <stdint.h>
#include "ps2_driver.h"
#include "types.h"

class MiniR4IO {
public:
    bool begin();

    /** True if PS2 controller locked into Analog Red mode (0x73) during boot. */
    bool isPs2ReadyAtBoot() const { return ps2Ready_; }

    /** Bounded non-blocking 1-cycle Analog Red reconnect probe (only called in Disconnected). */
    bool tryReconnectPs2();

    /** Executes one bounded 9-byte PS2 SPI packet poll (~1.4 ms, zero internal retries). */
    void pollPs2(Ps2Frame& outFrame);

    /** Alternates BTN_UP / BTN_DOWN reads over STM32 UART to keep loop latency low. */
    void pollButtons(uint32_t nowMs, ButtonEvents& outEvents);

    /**
     * Checks STM32 lower-MCU health & supply voltage at adaptive cadence.
     * Returns true when a probe ran this tick (`outVoltValid` indicates valid read).
     * Automatically heals STM32 config (power params, motor polarity, servo ranges)
     * after UART faults, brownout recovery, or periodic idle verification.
     */
    bool verifyAndHealLowerMcu(uint32_t nowMs,
                                bool isMoving,
                                bool& outVoltValid,
                                uint16_t& outBattMv,
                                bool force = false);

    bool commHealthy() const { return commHealthy_; }

    /** Forces next applyMotors()/applyServos() pass to re-write hardware registers. */
    void invalidateActuatorCaches();

    /** Returns true when motor stop should be re-asserted during Estop/Failsafe. */
    bool shouldRefreshStoppedMotors(uint32_t nowMs) const;

    /** Sends motor power to M1/M2 when changed, unsynced, or on periodic keepalive. */
    void applyMotors(int16_t leftPower, int16_t rightPower, bool forceBrake, uint32_t nowMs);

    /** Immediately zeroes M1/M2 (and M3/M4 when unzeroed) with optional brake. */
    void forceMotorsStop(bool engageBrake, uint32_t nowMs);

    /** True when M1/M2 H-bridge electromagnetic hold-brake is currently engaged. */
    bool isBrakeEngaged() const { return lastBrakeEngaged_; }

    /** Restores PS2 pins, STM32 motor polarity/ranges, and stops all motors after Auto exits. */
    void restoreAfterAuto(uint32_t nowMs);

    /** Sends servo targets RC1..RC4 when changed, unsynced, or on periodic keepalive. */
    void applyServos(const ServoTargets& targets, uint32_t nowMs);

    /** Enables or disables smooth trajectory slew limiting strictly for arm RC2. */
    void setRc2SmoothEnabled(bool enabled) { rc2SmoothEnabled_ = enabled; }
    bool rc2SmoothEnabled() const { return rc2SmoothEnabled_; }

    /** Synchronizes cached servo angles (e.g. after Auto mode modifies servos directly). */
    void syncServoCache(const ServoTargets& current);

    void setMotorPolarity(bool leftReverse, bool rightReverse);

    /** Loads validated PersistentConfig from UNO R4 EEPROM at kEepromConfigAddr. */
    bool loadConfigFromEeprom(PersistentConfig& outCfg);

    /** Saves PersistentConfig to UNO R4 EEPROM (writes only changed bytes). */
    void saveConfigToEeprom(const PersistentConfig& cfg);

    /** Non-zero MMLower::RESULT code from servo angle range config, or 0 if OK. */
    uint8_t servoRangeErr() const { return servoRangeErr_; }

private:
    struct DebouncedBtn {
        bool     rawState      = false;
        bool     stableState   = false;
        uint32_t lastChangeMs  = 0;
        bool     pressLatched  = false;
    };

    bool updateButtonDebounce(DebouncedBtn& btn, bool sample, uint32_t nowMs, bool& outShortRelease);
    bool applyLowerMcuConfig(bool zeroAuxMotors);
    bool configureRc34PulseRange(uint8_t ch);
    bool configureServoRanges();
    void staggerBootServos();

    Ps2Driver    ps2_{};
    bool         ps2Ready_                 = false;

    DebouncedBtn btnUp_{};
    DebouncedBtn btnDown_{};
    bool         pollUpTurn_               = true;
    uint32_t     lastButtonPollMs_         = 0;
    bool         suppressUploadUntilClear_ = true;
    bool         comboPrevActive_          = false;
    bool         comboFired_               = false;
    uint32_t     comboStartMs_             = 0;
    bool         upHoldPrevActive_         = false;
    bool         upHoldFired_              = false;
    uint32_t     upHoldStartMs_            = 0;

    bool         leftReverse_              = Config::kMotorLeftReverseInit;
    bool         rightReverse_             = Config::kMotorRightReverseInit;
    bool         auxMotorsZeroed_          = false;
    bool         commHealthy_              = true;
    bool         commNeedsHeal_            = false;
    uint16_t     prevRawMv_                = 0U;
    uint32_t     lastLowerHealthMs_        = 0;
    uint32_t     lastConfigResyncMs_       = 0;

    uint32_t     lastMotorSendMs_          = 0;
    uint32_t     lastMotorAttemptMs_       = 0;
    uint32_t     lastStopRefreshMs_        = 0;
    uint32_t     lastServoSendMs_          = 0;
    uint32_t     lastServoAttemptMs_       = 0;
    int16_t      lastLeftPower_            = 0;
    int16_t      lastRightPower_           = 0;
    bool         lastBrakeEngaged_         = false;
    bool         motorsSynced_             = false;
    ServoTargets lastServos_               = {};
    bool         servosSynced_             = false;
    bool         rc34PulseSynced_          = false;
    uint8_t      rc34PulseErr_             = 0U;
    bool         rc2SmoothEnabled_         = Config::kRc2SmoothDefault;
    uint8_t      servoRangeErr_            = 0U;
};
