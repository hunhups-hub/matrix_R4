#pragma once

// Control-policy decisions lifted out of main.cpp so they can be host-tested (pure C++11).

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

/** Which source drives the wheels this tick, in priority order (highest first). */
enum class DriveSource : uint8_t {
    Estop = 0,
    Failsafe,
    CrossBrake,
    Serial,
    Ps2,
    Coast
};

/**
 * Picks the drive source: E-stop > PS2 failsafe > PS2 Cross brake > latched Serial > PS2 > coast.
 * Cross brake only counts while PS2 is Operational and the frame is valid.
 */
DriveSource selectDriveSource(bool estop,
                              bool ps2Failsafe,
                              bool ps2Operational,
                              bool frameValid,
                              bool crossHeld,
                              bool serialLatched);

/** Effective max drive power = min(mode cap [precision/normal], battery power cap). */
int16_t computeMaxDriveSpeed(bool precisionMode, int16_t batteryMaxPowerCap);

/** Maps a stability penalty to a UI state: 0 = none, 1 = minor (<= 10), 2 = major. */
uint8_t classifyStabPenalty(uint8_t penalty);

/**
 * Fixed-period scheduler. Returns true when a control tick is due and fills outDtMs with the
 * elapsed time. Advances lastTickMs by one period, or resyncs to nowMs after a stall of 2+ periods
 * (no burst catch-up).
 */
bool advanceControlTick(uint32_t nowMs, uint32_t& lastTickMs, uint32_t& outDtMs);

} // namespace RobotLogic
