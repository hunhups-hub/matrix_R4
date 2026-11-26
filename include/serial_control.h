#pragma once

#include <stdint.h>
#include "types.h"

struct SerialActions {
    bool               uploadRequested     = false;
    bool               estopRequested      = false;
    bool               resumeRequested     = false;
    bool               autoStartRequested  = false;
    bool               togglePrecision     = false;
    bool               toggleReverseDrive  = false;
    bool               togglePosePreset    = false;
    bool               toggleRc2Smooth     = false;
    bool               trimLeftStep        = false;
    bool               trimRightStep       = false;
    bool               trimReset           = false;
    bool               toggleLeftReverse   = false;
    bool               toggleRightReverse  = false;
    bool               i2cScanRequested    = false;
    bool               armUpStep           = false;
    bool               armDownStep         = false;
    bool               gripOpen            = false;
    bool               gripClose           = false;
    bool               auxExtend           = false;
    bool               auxRetract          = false;
    bool               hasDirectServo      = false;
    uint8_t            directServoCh       = 0;
    int16_t            directServoDeg      = 90;
    bool               speedPctChanged     = false;
    int16_t            newSpeedPct         = Config::kSerialDefaultSpeed;
    SerialDriveCommand driveCmd            = {};
};

class SerialControl {
public:
    void begin(uint32_t baud);
    void printBanner();
    void printEvent(const char* tag, const char* msg);
    void printSnapshot(const SystemSnapshot& snap);

    /** Non-blocking parser for instant single-key WASD + line commands (`m L R`, `c ch deg`). */
    void poll(uint32_t nowMs, int16_t currentSpeedPct, SerialActions& outActions);

    /** Clear any active/latched serial drive motion (e.g. ESTOP or PS2 Cross brake override). */
    void stopMotion();

    bool streamTelemetry() const { return streamEnabled_; }
    bool isCollectingLine() const { return collectingLine_; }
    uint8_t lineLen() const { return lineLen_; }

private:
    enum class MotionMode : uint8_t {
        Stop = 0,
        Forward,
        Backward,
        TurnLeft,
        TurnRight,
        Direct
    };

    void printHelp();
    void parseLineCommand(uint32_t nowMs, SerialActions& outActions);
    void resetLine();
    void collectLineChar(char ch, uint32_t nowMs, SerialActions& outActions);
    void startMotion(MotionMode mode, uint32_t nowMs);
    /** Handles one single-key command. Returns true when polling must stop at once (upload key). */
    bool handleKey(char ch, uint32_t nowMs, int16_t& workingSpeedPct, SerialActions& outActions);
    void buildDriveCommand(int16_t spd, SerialActions& outActions) const;

    static constexpr uint8_t kLineBufSize = 40;
    char       lineBuf_[kLineBufSize]{};
    uint8_t    lineLen_         = 0;
    bool       collectingLine_   = false;
    uint32_t   lineStartTimeMs_ = 0;

    MotionMode activeMotion_  = MotionMode::Stop;
    uint32_t   motionExpireMs_ = 0;
    int16_t    directLeft_    = 0;
    int16_t    directRight_   = 0;
    bool       streamEnabled_ = false;
};

#ifndef ARDUINO
namespace NativeSerial {
void        reset();
void        feed(const char* data);
const char* output();
} // namespace NativeSerial
#endif
