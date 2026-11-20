#pragma once

#include <stdint.h>

#include "minir4_io.h"
#include "robot_logic.h"
#include "types.h"

class OledUI;

enum class AutoExitReason : uint8_t {
    None = 0,
    Completed,
    Ps2Cross,
    SerialStop,
    BtnStop,
    BtnEstop,
    SerialEstop,
    UploadCombo,
    SerialUpload,
    Timeout,
    StallTimeout,
    UserExit,
    DriveInitFail
};

namespace AutoBlockUser {
/** User's MATRIXblock R4 setup() entry point (defined in src/auto_block_code.cpp). */
void setup();
/** User's MATRIXblock R4 loop() entry point (defined in src/auto_block_code.cpp). */
void loop();
} // namespace AutoBlockUser

namespace AutoMode {

/**
 * Runs AutoBlockUser::setup() once and repeats AutoBlockUser::loop() while
 * keeping WDT alive and polling PS2 Cross (X), BTN_DOWN, and Serial exits.
 */
AutoExitReason execute(MiniR4IO& io,
                       RobotLogic::FailsafeStateMachine& linkSm,
                       SystemSnapshot& snap,
                       Ps2Frame& lastFrame,
                       OledUI* oled = nullptr);

/** Services WDT, Serial emergency keys, buttons, and PS2 Cross (X) during Auto. */
void service();

/** Syncs latest RC1..RC4 angle written by Auto so Manual keeps that pose on exit. */
void recordServoAngle(uint8_t channel, uint16_t angleDeg);

/** Returns true while Autonomous block mode is executing. */
bool isRunning();

/** Returns the human-readable label for a given AutoExitReason. */
const char* exitReasonStr(AutoExitReason reason);

/**
 * Services Auto safety loop and supplies the latest D2+D3 PS2 frame for
 * MATRIXblock Auto code calling MiniR4.PS2.read_gamepad(), including a one-shot
 * Triangle latch on Auto entry so `if (MiniR4.PS2.Button(PSB_TRIANGLE))` fires.
 */
bool pollPs2ForAuto(Ps2Frame& outFrame, bool& outEntryTrianglePulse);

/** Returns true if the current Auto session has not yet polled the entry Triangle pulse. */
bool hasEntryTrianglePending();

/**
 * Notifies AutoMode that a MATRIXblock Auto routine gated by the entry
 * PSB_TRIANGLE latch has matched, so Auto exits back to Manual cleanly
 * once the current loop() iteration completes.
 */
void notifyPs2TriangleMatched();

/** Applies EEPROM motor polarity flips relative to boot defaults to DriveDC.begin(). */
void resolveDrivePolarity(uint8_t leftMotorID,
                          uint8_t rightMotorID,
                          bool& ioLeftReverse,
                          bool& ioRightReverse);

/** Marks entry into a blocking DriveDC move wait so dead-UART and stall detection can abort safely. */
void beginBlockingDriveTask(bool expectWheelMotion = true);

/** Marks exit from a blocking DriveDC move wait. */
void endBlockingDriveTask();

/** Records that AutoBlockUser::loop() performed meaningful user activity beyond boilerplate polling. */
void noteUserLoopActivity();

/**
 * True when Config::kAutoGyroMode != Off and the STM32 IMU was zeroed on Auto entry.
 * False means every DriveDC call keeps its legacy open-loop behaviour (also the fallback
 * when the IMU reset failed).
 */
bool gyroAssistActive();

/** Virtual absolute heading (deg, -180..180, IMU zeroed at Auto entry) the robot should hold. */
int16_t headingTargetDeg();
void    setHeadingTargetDeg(int16_t deg);

/** Reads IMU yaw (rounded, wrapped to -180..180). Returns false on any STM32/IMU read failure. */
bool readYawDeg(int16_t& outDeg);

/** Re-latches the virtual heading to the current IMU yaw. Returns false (heading unchanged) on read failure. */
bool latchHeadingFromImu();

/** Prints one `[AUTO] #n ...` telemetry line per DriveDC move (Config::kAutoMoveTelemetry). */
void logMove(const char* kind, int16_t a, int16_t b, uint16_t degrees, uint8_t result);

/** Abort-safe Serial pass-through for Auto code (emergency keys are always intercepted first). */
int serialAvailable();
int serialRead();
int serialPeek();

} // namespace AutoMode

/** Optional helper for custom Auto loops to test whether X/ESTOP was requested. */
bool autoShouldExit();

/** Optional helper for Auto block code to finish and return to Manual mode. */
[[noreturn]] void autoExit();

/** Safely aborts Auto (motors braked, Manual restored) when DriveDC could not be configured. */
[[noreturn]] void autoAbortDriveInit();
