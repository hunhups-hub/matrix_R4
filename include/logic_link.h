#pragma once

// PS2 frame decode/validation, Auto start/abort gates, link failsafe state machine. (pure C++11, host-testable).

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

/** Decodes a raw 9-byte Sony PS2 SPI response packet into a Ps2Frame. */
void decodePs2Payload(const uint8_t raw[9], Ps2Frame& outFrame);

/** Returns true if the pad replied with valid sync (0x5A) in non-analog reset mode (e.g. 0x41). */
bool isPs2UnconfiguredMode(const Ps2Frame& frame);

/** Validates PS2 frame mode/sync bytes and rejects impossible button/bus-short patterns. */
bool isPs2FrameValid(const Ps2Frame& frame);

/** Returns true on a rising Start press (Controls::kAutoStart) while Ready, PS2 Operational, and X not held. */
bool shouldStartAutoMode(bool sysReady,
                         bool ps2Operational,
                         bool frameValid,
                         bool autoStartPressed,
                         bool prevAutoStartPressed,
                         bool crossPressed);

/** Returns true when a valid PS2 frame reports Cross (X) pressed to exit Auto mode. */
bool shouldAbortAutoMode(bool frameValid, bool crossPressed);

class FailsafeStateMachine {
public:
    void begin(bool initialReady, uint32_t nowMs);
    void onFrame(bool frameValid, uint32_t nowMs, bool modeReset = false);

    /** Returns true if Disconnected and bounded reconnect cooldown has elapsed. */
    bool shouldAttemptReinit(uint32_t nowMs) const;

    /** Records outcome & completion timestamp of a bounded reconnect attempt. */
    void onReinitResult(bool ready, uint32_t nowMs);

    LinkState state() const { return state_; }
    bool      isOperational() const { return state_ == LinkState::Operational; }
    bool      isFailsafe() const { return state_ == LinkState::Failsafe; }
    bool      isConnected() const { return state_ != LinkState::Disconnected; }
    bool      isLinkGlitching() const {
        return state_ == LinkState::Operational && badFrames_ > 0U;
    }
    uint8_t   goodFrames() const { return goodFrames_; }
    uint16_t  badFrames() const { return badFrames_; }

private:
    LinkState state_                = LinkState::Disconnected;
    uint8_t   goodFrames_           = 0;
    uint16_t  badFrames_            = 0;
    uint8_t   modeResetCnt_         = 0;
    uint32_t  lastValidMs_          = 0;
    uint32_t  lastReinitDoneMs_     = 0;
    bool      reinitCooldownArmed_  = false;
};

} // namespace RobotLogic
