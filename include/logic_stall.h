#pragma once

// Encoder progress, Auto drive polarity, ManualStallDetector. (pure C++11, host-testable).

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

/** Returns true when absolute encoder movement |curCount - prevCount| >= minDelta. */
bool hasEncoderProgressed(int32_t prevCount,
                          int32_t curCount,
                          int32_t minDelta = Config::kAutoStallMinDeltaDeg);

/**
 * Applies saved EEPROM motor polarity flips (relative to Config::kMotorLeft/RightReverseInit)
 * onto the polarity flags passed to DriveDC.begin(leftMotorID, rightMotorID, ...).
 */
void resolveAutoDrivePolarity(uint8_t leftMotorID,
                              uint8_t rightMotorID,
                              bool savedLeftReversed,
                              bool savedRightReversed,
                              bool& ioLeftReverse,
                              bool& ioRightReverse);

/**
 * Manual-drive wheel stall detector (opt-in via Config::kEnableManualStallCut).
 * Trips when |wheel cmd| >= kManualStallMinCmdPct and no encoder progress for kManualStallTimeoutMs.
 * Fails open: an invalid encoder sample never trips. Stays tripped until |cmd| <= kManualStallRearmCmdPct.
 */
class ManualStallDetector {
public:
    void reset();
    /**
     * Call every control tick. `sampleFresh` marks ticks that carry a new encoder reading
     * (`sampleValid` = the read succeeded). Returns true while the cut is latched.
     */
    bool update(int16_t cmdLeft,
                int16_t cmdRight,
                bool sampleFresh,
                bool sampleValid,
                int32_t encLeft,
                int32_t encRight,
                uint32_t nowMs);
    bool tripped() const { return tripped_; }

private:
    bool     tripped_    = false;
    bool     hasBase_    = false;
    int32_t  prevLeft_   = 0;
    int32_t  prevRight_  = 0;
    uint32_t lastMoveMs_ = 0UL;
};

} // namespace RobotLogic
