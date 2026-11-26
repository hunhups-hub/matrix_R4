#include "robot_logic.h"

// PS2 packet decode/validation and link failsafe state machine (pure C++11).

namespace RobotLogic {

void decodePs2Payload(const uint8_t raw[9], Ps2Frame& outFrame) {
    if (raw == nullptr) {
        outFrame = Ps2Frame{};
        return;
    }
    outFrame.modeId   = raw[1];
    outFrame.syncByte = raw[2];
    outFrame.buttons1 = raw[3];
    outFrame.buttons2 = raw[4];
    outFrame.rx       = raw[5];
    outFrame.ry       = raw[6];
    outFrame.lx       = raw[7];
    outFrame.ly       = raw[8];

    // Sony PS2 button bytes are active-low (0 = pressed)
    const uint8_t b1 = static_cast<uint8_t>(~raw[3]);
    const uint8_t b2 = static_cast<uint8_t>(~raw[4]);

    outFrame.select   = (b1 & 0x01U) != 0U;
    outFrame.l3       = (b1 & 0x02U) != 0U;
    outFrame.r3       = (b1 & 0x04U) != 0U;
    outFrame.start    = (b1 & 0x08U) != 0U;
    outFrame.up       = (b1 & 0x10U) != 0U;
    outFrame.right    = (b1 & 0x20U) != 0U;
    outFrame.down     = (b1 & 0x40U) != 0U;
    outFrame.left     = (b1 & 0x80U) != 0U;

    outFrame.l2       = (b2 & 0x01U) != 0U;
    outFrame.r2       = (b2 & 0x02U) != 0U;
    outFrame.l1       = (b2 & 0x04U) != 0U;
    outFrame.r1       = (b2 & 0x08U) != 0U;
    outFrame.triangle = (b2 & 0x10U) != 0U;
    outFrame.circle   = (b2 & 0x20U) != 0U;
    outFrame.cross    = (b2 & 0x40U) != 0U;
    outFrame.square   = (b2 & 0x80U) != 0U;

    outFrame.linkOk = (outFrame.modeId == Config::kPs2ExpectedMode) &&
                      (outFrame.syncByte == Config::kPs2ExpectedSync);
}

bool isPs2UnconfiguredMode(const Ps2Frame& frame) {
    return (frame.syncByte == Config::kPs2ExpectedSync) &&
           (frame.modeId != Config::kPs2ExpectedMode);
}

bool isPs2FrameValid(const Ps2Frame& frame) {
    // Stage 1: Exact Sony analog red-mode (0x73) + valid sync (0x5A)
    if (!frame.linkOk ||
        frame.modeId != Config::kPs2ExpectedMode ||
        frame.syncByte != Config::kPs2ExpectedSync) {
        return false;
    }

    // Stage 2: Stuck floating-high SPI line (every byte == 0xFF)
    if (frame.buttons1 == 0xFFU && frame.buttons2 == 0xFFU &&
        frame.rx == 0xFFU && frame.ry == 0xFFU &&
        frame.lx == 0xFFU && frame.ly == 0xFFU) {
        return false;
    }

    // Stage 3: Short-to-ground SPI line (every byte == 0x00)
    // Note: pushing both sticks to (0,0) or (255,255) with normal buttons passes cleanly.
    if (frame.buttons1 == 0x00U && frame.buttons2 == 0x00U &&
        frame.rx == 0x00U && frame.ry == 0x00U &&
        frame.lx == 0x00U && frame.ly == 0x00U) {
        return false;
    }

    // Stage 4: Physically impossible opposing D-pad states
    if ((frame.up && frame.down) || (frame.left && frame.right)) {
        return false;
    }

    // Stage 5: Impossible all-4 shape buttons pressed simultaneously (bus short)
    if (frame.triangle && frame.circle && frame.cross && frame.square) {
        return false;
    }

    return true;
}

bool shouldStartAutoMode(bool sysReady,
                         bool ps2Operational,
                         bool frameValid,
                         bool autoStartPressed,
                         bool prevAutoStartPressed,
                         bool crossPressed) {
    return sysReady && ps2Operational && frameValid &&
           autoStartPressed && !prevAutoStartPressed && !crossPressed;
}

bool shouldAbortAutoMode(bool frameValid, bool crossPressed) {
    return frameValid && crossPressed;
}

void FailsafeStateMachine::begin(bool initialReady, uint32_t nowMs) {
    goodFrames_          = 0U;
    badFrames_           = 0U;
    modeResetCnt_        = 0U;
    lastValidMs_         = nowMs;
    lastReinitDoneMs_    = nowMs;
    reinitCooldownArmed_ = initialReady;
    state_               = initialReady ? LinkState::Stabilizing : LinkState::Disconnected;
}

void FailsafeStateMachine::onFrame(bool frameValid, uint32_t nowMs, bool modeReset) {
    if (!frameValid) {
        if (state_ != LinkState::Operational) {
            goodFrames_ = 0U;
        }
        if (badFrames_ < 0xFFFFU) {
            badFrames_++;
        }

        if (modeReset) {
            if (modeResetCnt_ < 255U) {
                modeResetCnt_++;
            }
            if (modeResetCnt_ >= Config::kPs2ModeResetFrames) {
                goodFrames_          = 0U;
                state_               = LinkState::Disconnected;
                reinitCooldownArmed_ = false; // Allow immediate re-config without waiting 500 ms
                return;
            }
        } else {
            modeResetCnt_ = 0U;
        }

        if (state_ == LinkState::Disconnected) {
            goodFrames_ = 0U;
            return;
        }

        if (badFrames_ >= Config::kPs2PersistentLossFrames ||
            static_cast<uint32_t>(nowMs - lastValidMs_) >= Config::kPs2DisconnectMs) {
            goodFrames_          = 0U;
            state_               = LinkState::Disconnected;
            reinitCooldownArmed_ = false;
            return;
        }

        if (state_ == LinkState::Stabilizing || badFrames_ >= Config::kPs2FailsafeBadFrames) {
            goodFrames_ = 0U;
            state_      = LinkState::Failsafe;
        }
        return;
    }

    lastValidMs_  = nowMs;
    badFrames_    = 0U;
    modeResetCnt_ = 0U;
    if (goodFrames_ < 255U) {
        goodFrames_++;
    }

    state_ = (goodFrames_ >= Config::kPs2StableFrames)
                 ? LinkState::Operational
                 : LinkState::Stabilizing;
}

bool FailsafeStateMachine::shouldAttemptReinit(uint32_t nowMs) const {
    if (state_ != LinkState::Disconnected) {
        return false;
    }
    if (!reinitCooldownArmed_) {
        return true;
    }
    return static_cast<uint32_t>(nowMs - lastReinitDoneMs_) >= Config::kPs2ReconnectPeriodMs;
}

void FailsafeStateMachine::onReinitResult(bool ready, uint32_t nowMs) {
    lastReinitDoneMs_    = nowMs;
    reinitCooldownArmed_ = true;
    goodFrames_          = 0U;
    badFrames_           = 0U;
    modeResetCnt_        = 0U;
    if (ready) {
        lastValidMs_ = nowMs;
        state_       = LinkState::Stabilizing;
    } else {
        state_ = LinkState::Disconnected;
    }
}

} // namespace RobotLogic
