#include "serial_control.h"

#include <stdlib.h>
#include <string.h>

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <stdio.h>

#ifndef F
#define F(x) (x)
#endif

#ifndef HEX
#define HEX 16
#endif

namespace {

class FakeSerialPort {
public:
    void reset() {
        rxLen_    = 0U;
        rxPos_    = 0U;
        txLen_    = 0U;
        txBuf_[0] = '\0';
    }

    void feed(const char* data) {
        if (data == nullptr) {
            return;
        }
        if (rxPos_ >= rxLen_) {
            rxPos_ = 0U;
            rxLen_ = 0U;
        }
        const size_t n = strlen(data);
        for (size_t i = 0U; i < n && (rxLen_ + 1U) < kRxCap; ++i) {
            rxBuf_[rxLen_++] = data[i];
        }
    }

    const char* output() const { return txBuf_; }

    void begin(uint32_t baud) { (void)baud; }
    explicit operator bool() const { return true; }

    int available() const {
        return static_cast<int>(rxLen_ - rxPos_);
    }

    int read() {
        if (rxPos_ >= rxLen_) {
            return -1;
        }
        return static_cast<int>(static_cast<uint8_t>(rxBuf_[rxPos_++]));
    }

    void print(const char* s) {
        if (s == nullptr) {
            return;
        }
        appendRaw(s, strlen(s));
    }

    void print(char c) {
        appendRaw(&c, 1U);
    }

    void print(int val) {
        char tmp[16];
        const int n = snprintf(tmp, sizeof(tmp), "%d", val);
        if (n > 0) {
            appendRaw(tmp, static_cast<size_t>(n));
        }
    }

    void print(unsigned int val) {
        char tmp[16];
        const int n = snprintf(tmp, sizeof(tmp), "%u", val);
        if (n > 0) {
            appendRaw(tmp, static_cast<size_t>(n));
        }
    }

    void print(long val) {
        char tmp[24];
        const int n = snprintf(tmp, sizeof(tmp), "%ld", val);
        if (n > 0) {
            appendRaw(tmp, static_cast<size_t>(n));
        }
    }

    void print(unsigned long val) {
        char tmp[24];
        const int n = snprintf(tmp, sizeof(tmp), "%lu", val);
        if (n > 0) {
            appendRaw(tmp, static_cast<size_t>(n));
        }
    }

    void print(uint8_t val, int base) {
        char tmp[16];
        const int n = (base == HEX)
                          ? snprintf(tmp, sizeof(tmp), "%X", static_cast<unsigned>(val))
                          : snprintf(tmp, sizeof(tmp), "%u", static_cast<unsigned>(val));
        if (n > 0) {
            appendRaw(tmp, static_cast<size_t>(n));
        }
    }

    void print(float val, int decimals) {
        char tmp[24];
        const int prec = (decimals < 0) ? 0 : ((decimals > 6) ? 6 : decimals);
        const int n    = snprintf(tmp, sizeof(tmp), "%.*f", prec, static_cast<double>(val));
        if (n > 0) {
            appendRaw(tmp, static_cast<size_t>(n));
        }
    }

    void println() { appendRaw("\n", 1U); }

    template <typename T>
    void println(const T& val) {
        print(val);
        println();
    }

private:
    void appendRaw(const char* s, size_t n) {
        for (size_t i = 0U; i < n && (txLen_ + 1U) < kTxCap; ++i) {
            txBuf_[txLen_++] = s[i];
        }
        txBuf_[txLen_] = '\0';
    }

    static constexpr size_t kRxCap = 256U;
    static constexpr size_t kTxCap = 1024U;
    char   rxBuf_[kRxCap]{};
    size_t rxLen_ = 0U;
    size_t rxPos_ = 0U;
    char   txBuf_[kTxCap]{};
    size_t txLen_ = 0U;
};

FakeSerialPort Serial;

} // namespace

namespace NativeSerial {
void reset() { Serial.reset(); }
void feed(const char* data) { Serial.feed(data); }
const char* output() { return Serial.output(); }
} // namespace NativeSerial
#endif

#include "robot_logic.h"

namespace {

const char* linkStateStr(LinkState s) {
    switch (s) {
        case LinkState::Disconnected: return "DISCONNECTED";
        case LinkState::Failsafe:     return "FAILSAFE";
        case LinkState::Stabilizing:  return "STABILIZING";
        case LinkState::Operational:  return "OPERATIONAL";
        default:                      return "?";
    }
}

const char* sourceStr(ControlSource s) {
    switch (s) {
        case ControlSource::None:      return "IDLE";
        case ControlSource::Ps2:       return "PS2";
        case ControlSource::SerialCmd: return "SERIAL";
        case ControlSource::Auto:      return "AUTO";
        default:                       return "?";
    }
}

const char* batTierStr(BatteryTier t) {
    switch (t) {
        case BatteryTier::Normal:    return "NORM";
        case BatteryTier::Low:       return "LOW";
        case BatteryTier::Critical:  return "CRIT";
        case BatteryTier::Emergency: return "EMERG";
        default:                     return "?";
    }
}

} // namespace

void SerialControl::begin(uint32_t baud) {
    Serial.begin(baud);
#ifdef ARDUINO
    const uint32_t startMs = millis();
    while (!Serial && static_cast<uint32_t>(millis() - startMs) < 1200UL) {
        delay(10);
    }
#endif
}

void SerialControl::stopMotion() {
    activeMotion_   = MotionMode::Stop;
    motionExpireMs_ = 0UL;
    directLeft_     = 0;
    directRight_    = 0;
}

void SerialControl::printHelp() {
    Serial.println(F("\n=== MATRIX Mini R4 Manual Control (115200) ==="));
    Serial.println(F("  w/s/a/d   : Drive Fwd/Back/Left/Right (auto-stops 650ms on release)"));
    Serial.println(F("  Space / x : Instant STOP motors   | ! : ESTOP | r : Resume | g : Run AUTO"));
    Serial.println(F("  z / b     : Toggle Precision(55% session) / Reverse-Front | [ / ] : Speed -/+ 15%"));
    Serial.println(F("  , / . / 0 : Motor Balance Trim L(-1%)/R(+1%)/Reset(0%) (saved to EEPROM)"));
    Serial.println(F("  i / k / y : RC2 Up(-10)/Dn(+10)/Toggle Smooth RC2 | j / l : Front RC3+RC4"));
    Serial.println(F("  o / p     : Aux RC1 0 / 180 deg | q : Toggle Hold/Drop | c <1..4> <deg>"));
    Serial.println(F("  m <L> <R> : Direct M1/M2 (-100..100 for 2.5s) | f1 / f2 : Flip M1/M2 dir"));
    Serial.println(F("  t         : Toggle status stream | u : Enter Upload Bootloader | ? : Help\n"));
}

void SerialControl::printBanner() {
    Serial.println(F("\n================================================"));
    Serial.println(F("  MATRIX Mini R4 Robot Firmware Ready"));
    Serial.println(F("  Drive: M1(L) + M2(R) | Servos: RC1..RC4"));
    Serial.println(F("  PS2/MJ2: D2(4p) + D3(2p) | Auto: Triangle=Start, X=Exit"));
    Serial.println(F("  Upload: BTN UP+DOWN 2.5s or 'u'"));
    Serial.println(F("================================================"));
    printHelp();
}

void SerialControl::printEvent(const char* tag, const char* msg) {
    Serial.print('[');
    Serial.print(tag);
    Serial.print(F("] "));
    Serial.println(msg);
}

void SerialControl::printSnapshot(const SystemSnapshot& snap) {
    Serial.print(F("[STAT] "));
    Serial.print(snap.sysState == SystemState::Estop ? F("ESTOP ") : F("READY "));
    Serial.print(F("src="));
    Serial.print(sourceStr(snap.activeSource));
    Serial.print(F(" ps2="));
    Serial.print(linkStateStr(snap.ps2Link));
    Serial.print(F("(0x"));
    Serial.print(snap.ps2Mode, HEX);
    Serial.print(F("/0x"));
    Serial.print(snap.ps2Sync, HEX);
    Serial.print(',');
    Serial.print(snap.ps2Ly);
    Serial.print(',');
    Serial.print(snap.ps2Rx);
    Serial.print(')');
    Serial.print(F(" vbat="));
    Serial.print(snap.battVolt, 2);
    Serial.print(F("V("));
    if (snap.extPower) {
        Serial.print(snap.battPct);
        Serial.print(F("% "));
        Serial.print(batTierStr(snap.batTier));
    } else {
        Serial.print(F("USB"));
    }
    Serial.print(F(" cap="));
    Serial.print(snap.powerCapPct);
    Serial.print(F("%) comm="));
    Serial.print(snap.commHealthy ? F("OK") : F("ERR"));
    Serial.print(F(" stb="));
    Serial.print(snap.stabilityPct);
    Serial.print(F("% jit="));
    Serial.print(snap.loopJitterMs);
    Serial.print(F("ms M=("));
    Serial.print(snap.drive.left);
    Serial.print(',');
    Serial.print(snap.drive.right);
    Serial.print(F(") rev=("));
    Serial.print(snap.leftReversed ? 1 : 0);
    Serial.print(',');
    Serial.print(snap.rightReversed ? 1 : 0);
    Serial.print(F(") trim="));
    Serial.print(static_cast<int>(snap.motorTrimPct));
    Serial.print(F("% prec="));
    Serial.print(snap.precisionMode ? 1 : 0);
    Serial.print(F(" brk="));
    Serial.print(snap.holdBrakeActive ? 1 : 0);
    Serial.print(F(" rc2Sm="));
    Serial.print(snap.rc2SmoothEnabled ? 1 : 0);
    Serial.print(F(" RC=("));
    Serial.print(snap.servos.rc1Deg);
    Serial.print(',');
    Serial.print(snap.servos.rc2Deg);
    Serial.print(',');
    Serial.print(snap.servos.rc3Deg);
    Serial.print(',');
    Serial.print(snap.servos.rc4Deg);
    Serial.print(F(") spd="));
    Serial.print(snap.serialSpeedPct);
    Serial.print(F("% rngErr="));
    Serial.print(snap.servoRangeErr);
    Serial.print(F(" oled=0x"));
    Serial.print(snap.oledAddr, HEX);
    Serial.print(F("(ok="));
    Serial.print(snap.oledOk ? 1 : 0);
    Serial.print(F(",e3d="));
    Serial.print(snap.oledErr3d);
    Serial.print(F(",e3c="));
    Serial.print(snap.oledErr3c);
    Serial.println(')');
}

void SerialControl::parseLineCommand(uint32_t nowMs, SerialActions& outActions) {
    while (lineLen_ > 0U && (lineBuf_[lineLen_ - 1U] == ' ' || lineBuf_[lineLen_ - 1U] == '\t')) {
        lineLen_--;
    }
    lineBuf_[lineLen_] = '\0';
    if (lineLen_ == 0U) {
        return;
    }

    for (uint8_t i = 0U; i < lineLen_; ++i) {
        if (lineBuf_[i] >= 'A' && lineBuf_[i] <= 'Z') {
            lineBuf_[i] = static_cast<char>(lineBuf_[i] + ('a' - 'A'));
        }
    }

    auto isSepChar = [](char c) -> bool {
        return (c == ' ' || c == '\t' || c == ',');
    };
    auto skipArgSep = [](const char* s) -> const char* {
        while (*s == ' ' || *s == '\t') {
            ++s;
        }
        if (*s == ',') {
            ++s;
            while (*s == ' ' || *s == '\t') {
                ++s;
            }
        }
        return s;
    };

    bool handled = false;
    if (strcmp(lineBuf_, "f1") == 0) {
        outActions.toggleLeftReverse = true;
        handled                      = true;
    } else if (strcmp(lineBuf_, "f2") == 0) {
        outActions.toggleRightReverse = true;
        handled                       = true;
    } else if (lineBuf_[0] == 'm' && isSepChar(lineBuf_[1])) {
        const char* p1     = skipArgSep(lineBuf_ + 1);
        char*       endPtr = nullptr;
        const long  l      = strtol(p1, &endPtr, 10);
        if (endPtr != nullptr && endPtr != p1 && isSepChar(*endPtr)) {
            const char* p2      = skipArgSep(endPtr);
            char*       endPtr2 = nullptr;
            const long  r       = strtol(p2, &endPtr2, 10);
            if (*p2 != '\0' && endPtr2 != nullptr && endPtr2 != p2 && *endPtr2 == '\0') {
                directLeft_     = static_cast<int16_t>(RobotLogic::clampLong(l, -100L, 100L));
                directRight_    = static_cast<int16_t>(RobotLogic::clampLong(r, -100L, 100L));
                activeMotion_   = MotionMode::Direct;
                motionExpireMs_ = nowMs + Config::kSerialDirectHoldMs;
                handled         = true;
            }
        }
    } else if (lineBuf_[0] == 'c' && isSepChar(lineBuf_[1])) {
        const char* p1     = skipArgSep(lineBuf_ + 1);
        char*       endPtr = nullptr;
        const long  ch     = strtol(p1, &endPtr, 10);
        if (endPtr != nullptr && endPtr != p1 && ch >= 1L && ch <= 4L && isSepChar(*endPtr)) {
            const char* p2      = skipArgSep(endPtr);
            char*       endPtr2 = nullptr;
            const long  deg     = strtol(p2, &endPtr2, 10);
            if (*p2 != '\0' && endPtr2 != nullptr && endPtr2 != p2 && *endPtr2 == '\0') {
                outActions.hasDirectServo = true;
                outActions.directServoCh  = static_cast<uint8_t>(ch);
                outActions.directServoDeg = static_cast<int16_t>(RobotLogic::clampLong(deg, 0L, 180L));
                handled                   = true;
            }
        }
    }

    if (!handled) {
        Serial.print(F("[SER] Unknown line: "));
        Serial.println(lineBuf_);
    }
}

void SerialControl::poll(uint32_t nowMs, int16_t currentSpeedPct, SerialActions& outActions) {
    outActions = SerialActions{};
    int16_t workingSpeedPct = currentSpeedPct;

    // Cancel stale or abandoned line buffering
    if (collectingLine_ &&
        static_cast<uint32_t>(nowMs - lineStartTimeMs_) >= Config::kLineTimeoutMs) {
        collectingLine_ = false;
        lineLen_        = 0U;
    }

    while (Serial.available() > 0) {
        const char ch = static_cast<char>(Serial.read());

        // Cancel stale line buffering if timeout elapsed while receiving characters
        if (collectingLine_ &&
            static_cast<uint32_t>(nowMs - lineStartTimeMs_) >= Config::kLineTimeoutMs) {
            collectingLine_ = false;
            lineLen_        = 0U;
        }

        // Emergency intercept: always immediate, even during line buffering!
        if (ch == '!') {
            collectingLine_           = false;
            lineLen_                  = 0U;
            activeMotion_             = MotionMode::Stop;
            outActions.estopRequested = true;
            continue;
        }

        if (ch == 'x' || ch == 'X') {
            collectingLine_ = false;
            lineLen_        = 0U;
            activeMotion_   = MotionMode::Stop;
            continue;
        }

        if (collectingLine_) {
            if (ch == '\n' || ch == '\r') {
                collectingLine_ = false;
                parseLineCommand(nowMs, outActions);
                lineLen_ = 0U;
            } else if (ch == ' ') {
                // Space immediately stops active motion; cancel dangling "f " abort cleanly while allowing spaces inside m/c/trailing f1.
                activeMotion_ = MotionMode::Stop;
                if (lineBuf_[0] == 'f' && lineLen_ == 1U) {
                    collectingLine_ = false;
                    lineLen_        = 0U;
                } else if (lineLen_ + 1U < kLineBufSize) {
                    lineBuf_[lineLen_++] = ch;
                } else {
                    // Buffer overrun protection
                    collectingLine_ = false;
                    lineLen_        = 0U;
                }
            } else if (lineLen_ + 1U < kLineBufSize) {
                lineBuf_[lineLen_++] = ch;
            } else {
                // Buffer overrun protection: cancel line collection on overflow
                collectingLine_ = false;
                lineLen_        = 0U;
            }
            continue;
        }

        // Start line-buffered multi-argument commands cleanly on m/M, c/C, or f/F.
        if (ch == 'm' || ch == 'M' || ch == 'c' || ch == 'C' || ch == 'f' || ch == 'F') {
            collectingLine_  = true;
            lineStartTimeMs_ = nowMs;
            lineLen_         = 1U;
            lineBuf_[0]      = (ch >= 'A' && ch <= 'Z') ? static_cast<char>(ch + ('a' - 'A')) : ch;
            continue;
        }

        switch (ch) {
            case 'w':
            case 'W':
                activeMotion_   = MotionMode::Forward;
                motionExpireMs_ = nowMs + Config::kSerialKeyHoldMs;
                break;
            case 's':
            case 'S':
                activeMotion_   = MotionMode::Backward;
                motionExpireMs_ = nowMs + Config::kSerialKeyHoldMs;
                break;
            case 'a':
            case 'A':
                activeMotion_   = MotionMode::TurnLeft;
                motionExpireMs_ = nowMs + Config::kSerialKeyHoldMs;
                break;
            case 'd':
            case 'D':
                activeMotion_   = MotionMode::TurnRight;
                motionExpireMs_ = nowMs + Config::kSerialKeyHoldMs;
                break;
            case ' ':
            case 'x':
            case 'X':
                activeMotion_ = MotionMode::Stop;
                break;
            case '!':
                activeMotion_             = MotionMode::Stop;
                outActions.estopRequested = true;
                break;
            case 'r':
            case 'R':
                outActions.resumeRequested = true;
                break;
            case 'i':
            case 'I':
                outActions.armUpStep = true;
                break;
            case 'k':
            case 'K':
                outActions.armDownStep = true;
                break;
            case 'j':
            case 'J':
                outActions.gripOpen = true;
                break;
            case 'l':
            case 'L':
                outActions.gripClose = true;
                break;
            case 'o':
            case 'O':
                outActions.auxExtend = true;
                break;
            case 'p':
            case 'P':
                outActions.auxRetract = true;
                break;
            case 'q':
            case 'Q':
                outActions.togglePosePreset = true;
                break;
            case 'g':
            case 'G':
                activeMotion_                 = MotionMode::Stop;
                outActions.autoStartRequested = true;
                break;
            case 'z':
            case 'Z':
                outActions.togglePrecision = true;
                break;
            case 'b':
            case 'B':
                outActions.toggleReverseDrive = true;
                break;
            case 'y':
            case 'Y':
                outActions.toggleRc2Smooth = true;
                break;
            case ',':
            case '<':
                outActions.trimLeftStep = true;
                break;
            case '.':
            case '>':
                outActions.trimRightStep = true;
                break;
            case '0':
                outActions.trimReset = true;
                break;
            case '[':
                workingSpeedPct = RobotLogic::clamp16(
                    static_cast<int16_t>(workingSpeedPct - Config::kSerialSpeedStep),
                    Config::kSerialMinSpeed,
                    Config::kMotorMaxPower);
                outActions.speedPctChanged = true;
                outActions.newSpeedPct     = workingSpeedPct;
                break;
            case ']':
                workingSpeedPct = RobotLogic::clamp16(
                    static_cast<int16_t>(workingSpeedPct + Config::kSerialSpeedStep),
                    Config::kSerialMinSpeed,
                    Config::kMotorMaxPower);
                outActions.speedPctChanged = true;
                outActions.newSpeedPct     = workingSpeedPct;
                break;
            case 't':
            case 'T':
                streamEnabled_ = !streamEnabled_;
                Serial.print(F("[SER] Telemetry stream: "));
                Serial.println(streamEnabled_ ? F("ON") : F("OFF"));
                break;
            case 'v':
            case 'V':
                outActions.i2cScanRequested = true;
                break;
            case 'u':
            case 'U':
                activeMotion_              = MotionMode::Stop;
                outActions.uploadRequested = true;
                return;
            case '?':
            case 'h':
            case 'H':
                printHelp();
                break;
            case '\r':
            case '\n':
                break;
            default:
                break;
        }
    }

    if (activeMotion_ != MotionMode::Stop &&
        static_cast<int32_t>(motionExpireMs_ - nowMs) <= 0) {
        activeMotion_ = MotionMode::Stop;
    }

    const int16_t spd   = outActions.speedPctChanged ? outActions.newSpeedPct : currentSpeedPct;
    outActions.driveCmd = SerialDriveCommand{};
    switch (activeMotion_) {
        case MotionMode::Forward:
            outActions.driveCmd.active      = true;
            outActions.driveCmd.targetLeft  = spd;
            outActions.driveCmd.targetRight = spd;
            break;
        case MotionMode::Backward:
            outActions.driveCmd.active      = true;
            outActions.driveCmd.targetLeft  = static_cast<int16_t>(-spd);
            outActions.driveCmd.targetRight = static_cast<int16_t>(-spd);
            break;
        case MotionMode::TurnLeft:
            outActions.driveCmd.active      = true;
            outActions.driveCmd.targetLeft  = static_cast<int16_t>(-spd);
            outActions.driveCmd.targetRight = spd;
            break;
        case MotionMode::TurnRight:
            outActions.driveCmd.active      = true;
            outActions.driveCmd.targetLeft  = spd;
            outActions.driveCmd.targetRight = static_cast<int16_t>(-spd);
            break;
        case MotionMode::Direct:
            outActions.driveCmd.active      = true;
            outActions.driveCmd.targetLeft  = directLeft_;
            outActions.driveCmd.targetRight = directRight_;
            break;
        case MotionMode::Stop:
        default:
            break;
    }
}
