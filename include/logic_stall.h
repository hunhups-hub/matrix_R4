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

/**
 * Auto-mode wheel stall tracker (decision logic only; the caller does the STM32 reads and aborts).
 * Stalled = a blocking DriveDC task is running, wheel motion is expected, and neither the encoders
 * nor the drive degree counter moved by kAutoStallMinDeltaDeg for kAutoStallTimeoutMs.
 */
class AutoStallTracker {
public:
    /** Starts a fresh command window at nowMs (grace period restarts, baselines cleared). */
    void reset(uint32_t nowMs);

    /** Drops encoder/degree baselines (a drive task just ended or was never active). */
    void clearBaselines();

    /** True when a check is due (grace period over and poll interval elapsed); records the check time. */
    bool checkDue(uint32_t nowMs);

    /** No wheel motion expected right now: clear baselines and restart the no-progress timer. */
    void markIdle(uint32_t nowMs);

    /** True when no wheel progress was seen for kAutoStallTimeoutMs. */
    bool timedOut(uint32_t nowMs) const;

    /**
     * Feeds one sample. Returns true when the wheels are stalled (abort Auto).
     * With neither reading valid it only reports the no-progress timeout.
     */
    bool update(uint32_t nowMs,
                bool encOk,
                int32_t encLeft,
                int32_t encRight,
                bool degOk,
                int32_t driveDeg);

private:
    uint32_t cmdStartMs_       = 0UL;
    uint32_t lastCheckMs_      = 0UL;
    uint32_t lastWheelMoveMs_  = 0UL;
    int32_t  prevEncL_         = 0;
    int32_t  prevEncR_         = 0;
    int32_t  prevDriveDeg_     = 0;
    bool     hasEncBaseline_   = false;
    bool     hasDegBaseline_   = false;
};

} // namespace RobotLogic
