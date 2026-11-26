#include "oled_ui.h"

#include <Arduino.h>
#include <MatrixMiniR4.h>
#include <WDT.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

#include "robot_logic.h"

using RobotLogic::batteryTierChar;
using RobotLogic::fnv1aHash;
using RobotLogic::headerTag;
using RobotLogic::packRgb;
using RobotLogic::stabFlagChar;

namespace {

// Send a real 2-byte SSD1306 command (Co=0, D/C=0, cmd) because Renesas RA4M1
// R_IIC_MASTER_Write rejects 0-byte writes with FSP_ERR_ASSERTION.
uint8_t probeSsd1306Cmd(TwoWire& bus, uint8_t addr, uint8_t cmd) {
    bus.beginTransmission(addr);
    bus.write(static_cast<uint8_t>(0x00));
    bus.write(cmd);
    return bus.endTransmission();
}

} // namespace

void OledUI::recoverBus(uint8_t& sdaBefore, uint8_t& sdaAfter, uint8_t& clocksSent) {
    Wire1.end();
    pinMode(WIRE1_SDA_PIN, INPUT_PULLUP);
    pinMode(WIRE1_SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(20);

    sdaBefore  = static_cast<uint8_t>(digitalRead(WIRE1_SDA_PIN));
    clocksSent = 0U;

    while (digitalRead(WIRE1_SDA_PIN) == LOW && clocksSent < 16U) {
        pinMode(WIRE1_SCL_PIN, OUTPUT);
        digitalWrite(WIRE1_SCL_PIN, LOW);
        delayMicroseconds(10);
        pinMode(WIRE1_SCL_PIN, INPUT_PULLUP);
        delayMicroseconds(10);
        clocksSent++;
    }

    // Issue I2C STOP: SDA low -> SCL high -> SDA high.
    pinMode(WIRE1_SDA_PIN, OUTPUT);
    digitalWrite(WIRE1_SDA_PIN, LOW);
    delayMicroseconds(10);
    pinMode(WIRE1_SCL_PIN, INPUT_PULLUP);
    delayMicroseconds(10);
    pinMode(WIRE1_SDA_PIN, INPUT_PULLUP);
    delayMicroseconds(20);

    sdaAfter = static_cast<uint8_t>(digitalRead(WIRE1_SDA_PIN));
}

bool OledUI::begin() {
    lastOledMs_      = 0;
    lastDisplayHash_ = 0;
    hasDisplayHash_  = false;
    lastLed1Color_   = 0xFFFFFFFFUL;
    lastLed2Color_   = 0xFFFFFFFFUL;
    oledAddr_        = MATRIXMINIR4_OLED_ADDRESS;

    // 60ms Wire1 timeout accommodates 255-byte SSD1306 chunks while staying << 3s WDT.
    Wire1.setWireTimeout(static_cast<unsigned int>(Config::kWireTimeoutUs), true);

    err3d_ = probeSsd1306Cmd(Wire1, MATRIXMINIR4_OLED_ADDRESS, SSD1306_DISPLAYON);
    if (err3d_ != 0U) {
        recoverBus(bootSdaBefore_, bootSdaAfter_, bootClocks_);
        Wire1.begin();
        Wire1.setWireTimeout(static_cast<unsigned int>(Config::kWireTimeoutUs), true);
        MiniR4.OLED.begin(SSD1306_SWITCHCAPVCC, MATRIXMINIR4_OLED_ADDRESS, false, false);
        err3d_ = probeSsd1306Cmd(Wire1, MATRIXMINIR4_OLED_ADDRESS, SSD1306_DISPLAYON);
    }

    if (err3d_ == 0U) {
        lastI2cErr_ = 0U;
    } else {
        err3c_ = probeSsd1306Cmd(Wire1, Config::kOledAddrPrimary, SSD1306_DISPLAYON);
        if (err3c_ == 0U) {
            oledAddr_   = Config::kOledAddrPrimary;
            lastI2cErr_ = 0U;
            MiniR4.OLED.begin(SSD1306_SWITCHCAPVCC, Config::kOledAddrPrimary, false, false);
        } else {
            lastI2cErr_ = err3d_;
        }
    }

    if (lastI2cErr_ == 0U) {
        MiniR4.OLED.ssd1306_command(SSD1306_DISPLAYON);
    }
    MiniR4.OLED.setTextWrap(false);
    MiniR4.OLED.setTextSize(1);
    MiniR4.OLED.setTextColor(SSD1306_WHITE);

    fastClockOk_ = (lastI2cErr_ == 0U);
    if (fastClockOk_) {
        Wire1.setClock(Config::kI2cFastHz);
    } else {
        Wire1.setClock(Config::kI2cSlowHz);
    }
    return (lastI2cErr_ == 0U);
}

uint8_t OledUI::flushBufferChecked(const uint8_t* buf, uint16_t len) {
    if (buf == nullptr || len == 0U) {
        return 0U;
    }

    auto transferFrame = [this, buf, len]() -> uint8_t {
        static const uint8_t kWindowCmds[] = {
            SSD1306_PAGEADDR,
            0x00U,
            0xFFU,
            SSD1306_COLUMNADDR,
            0x00U,
            127U,
        };
        Wire1.beginTransmission(oledAddr_);
        Wire1.write(static_cast<uint8_t>(0x00U));
        for (uint8_t i = 0U; i < sizeof(kWindowCmds); ++i) {
            Wire1.write(kWindowCmds[i]);
        }
        uint8_t err = Wire1.endTransmission();
        if (err != 0U) {
            return err;
        }

        // Send 64-byte pixel bursts (<< 255-byte Wire buffer) and verify every ACK.
        constexpr uint16_t kChunkBytes = 64U;
        uint16_t offset = 0U;
        while (offset < len) {
            const uint16_t rem   = static_cast<uint16_t>(len - offset);
            const uint16_t chunk = (rem > kChunkBytes) ? kChunkBytes : rem;
            Wire1.beginTransmission(oledAddr_);
            Wire1.write(static_cast<uint8_t>(0x40U));
            Wire1.write(buf + offset, chunk);
            err = Wire1.endTransmission();
            if (err != 0U) {
                return err;
            }
            offset = static_cast<uint16_t>(offset + chunk);
        }

        return 0U;
    };

    uint8_t err = transferFrame();
    if (err != 0U) {
        // Fallback to 100 kHz ONLY on error transition instead of thrashing FSP I2C per frame.
        // Preserve framebuffer because Adafruit_SSD1306::begin() calls clearDisplay().
        constexpr uint16_t kMaxSavedBytes = Config::kOledBufferBytes;
        uint8_t            savedBuf[kMaxSavedBytes];
        const uint16_t     copyLen = (len <= kMaxSavedBytes) ? len : kMaxSavedBytes;
        memcpy(savedBuf, buf, copyLen);

        fastClockOk_ = false;
        uint8_t sdaB = 1U;
        uint8_t sdaA = 1U;
        uint8_t clks = 0U;
        recoverBus(sdaB, sdaA, clks);
        Wire1.begin();
        Wire1.setClock(Config::kI2cSlowHz);
        Wire1.setWireTimeout(static_cast<unsigned int>(Config::kWireTimeoutUs), true);
        MiniR4.OLED.begin(SSD1306_SWITCHCAPVCC, oledAddr_, false, false);
        Wire1.setClock(Config::kI2cSlowHz);

        uint8_t* oledBuf = MiniR4.OLED.getBuffer();
        if (oledBuf != nullptr) {
            memcpy(oledBuf, savedBuf, copyLen);
        }
        err = transferFrame();
    }

    lastI2cErr_ = err;
    if (oledAddr_ == MATRIXMINIR4_OLED_ADDRESS) {
        err3d_ = err;
    } else if (oledAddr_ == Config::kOledAddrPrimary) {
        err3c_ = err;
    }
    return err;
}

void OledUI::showBootloaderScreen() {
    MiniR4.LED.setColor(1, 0, 120, 180);
    MiniR4.LED.setColor(2, 0, 120, 180);
    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setTextWrap(false);
    MiniR4.OLED.setTextSize(1);
    MiniR4.OLED.setTextColor(SSD1306_WHITE);
    MiniR4.OLED.setCursor(10, 6);
    MiniR4.OLED.print(F("UPLOAD BOOTLOADER"));
    MiniR4.OLED.setCursor(16, 19);
    MiniR4.OLED.print(F("Ready for Flash..."));
    flushBufferChecked(MiniR4.OLED.getBuffer(), Config::kOledBufferBytes);
    MiniR4.Buzzer.Tone(1500, 90);
}

void OledUI::beepBoot() {
    MiniR4.Buzzer.Tone(1100, 60);
}

void OledUI::beepConfirm() {
    MiniR4.Buzzer.Tone(1400, 45);
}

void OledUI::beepWarning() {
    MiniR4.Buzzer.Tone(520, 90);
}

void OledUI::updateRgbLeds(const SystemSnapshot& snap, uint32_t nowMs) {
    uint32_t c1;
    uint32_t c2 = packRgb(0, 45, 0);

    if (snap.comboHolding) {
        c1 = packRgb(180, 110, 0);
        c2 = packRgb(180, 110, 0);
    } else if (snap.sysState == SystemState::Estop || snap.ps2Link == LinkState::Failsafe) {
        const bool blinkOn = ((nowMs / 120UL) % 2UL) == 0UL;
        c1 = blinkOn ? packRgb(180, 0, 0) : packRgb(0, 0, 0);
        c2 = c1;
    } else if (snap.activeSource == ControlSource::Auto) {
        c1 = packRgb(140, 0, 180);
        c2 = packRgb(0, 140, 180);
    } else if (snap.ps2Link == LinkState::Operational) {
        c1 = snap.precisionMode ? packRgb(0, 120, 80) : packRgb(0, 140, 0);
    } else if (snap.ps2Link == LinkState::Stabilizing) {
        c1 = packRgb(180, 100, 0);
    } else if (snap.activeSource == ControlSource::SerialCmd) {
        c1 = packRgb(0, 90, 160);
    } else {
        const bool pulse = ((nowMs % 1000UL) < 160UL);
        c1 = pulse ? packRgb(120, 0, 0) : packRgb(15, 0, 0);
    }

    if (!snap.comboHolding &&
        snap.sysState != SystemState::Estop &&
        snap.ps2Link != LinkState::Failsafe) {
        if (!snap.commHealthy && ((nowMs % 500UL) < 250UL)) {
            c2 = packRgb(160, 0, 140);
        } else if ((snap.lowVoltWarn || snap.batTier != BatteryTier::Normal) &&
                   ((nowMs % 1000UL) < 250UL)) {
            c2 = (snap.batTier == BatteryTier::Critical || snap.batTier == BatteryTier::Emergency)
                     ? packRgb(180, 30, 0)
                     : packRgb(180, 90, 0);
        } else if (snap.drive.left != 0 || snap.drive.right != 0) {
            c2 = packRgb(0, 80, 160);
        }
    }

    if (c1 != lastLed1Color_) {
        lastLed1Color_ = c1;
        MiniR4.LED.setColor(1, c1);
    }
    if (c2 != lastLed2Color_) {
        lastLed2Color_ = c2;
        MiniR4.LED.setColor(2, c2);
    }
}

void OledUI::render(const SystemSnapshot& snap, uint32_t nowMs, bool force) {
    updateRgbLeds(snap, nowMs);

    if (!force &&
        static_cast<uint32_t>(nowMs - lastOledMs_) < Config::kOledRefreshPeriodMs) {
        return;
    }

    char line1[24];
    char line2[24];
    char line3[24];
    char line4[24];

    const float clampedV = (snap.battVolt < 0.0f) ? 0.0f : ((snap.battVolt > 99.9f) ? 99.9f : snap.battVolt);
    const int   vTenths  = static_cast<int>(clampedV * 10.0f + 0.5f);
    if (snap.extPower) {
        snprintf(line1, sizeof(line1), "%-8s %2d.%1dV %3u%%",
                 headerTag(snap),
                 vTenths / 10,
                 vTenths % 10,
                 static_cast<unsigned>(snap.battPct));
    } else {
        snprintf(line1, sizeof(line1), "%-8s %2d.%1dV  USB",
                 headerTag(snap),
                 vTenths / 10,
                 vTenths % 10);
    }

    snprintf(line2, sizeof(line2), "STB%3u%% L%cP%cC%cT%cD%c",
             static_cast<unsigned>(snap.stabilityPct),
             stabFlagChar(snap.linkStabState),
             stabFlagChar(snap.powerStabState),
             stabFlagChar(snap.commStabState),
             stabFlagChar(snap.loopStabState),
             stabFlagChar(snap.driveStabState));

    const char idleBrakeChar =
        (snap.drive.left == 0 && snap.drive.right == 0 && snap.holdBrakeActive) ? 'H' : ' ';
    const char modeChar = (snap.precisionMode && snap.reverseDrive)
                              ? 'B'
                              : (snap.precisionMode ? 'S' : (snap.reverseDrive ? 'R' : idleBrakeChar));
    snprintf(line3, sizeof(line3), "M1:%+04d M2:%+04d %c%c",
             static_cast<int>(snap.drive.left),
             static_cast<int>(snap.drive.right),
             modeChar,
             batteryTierChar(snap.batTier));

    const bool showAutoHold = snap.autoHolding && (snap.sysState == SystemState::Ready);
    auto clampHoldPct = [](float pct) -> float {
        return (pct < 0.0f) ? 0.0f : ((pct > 1.0f) ? 1.0f : pct);
    };

    if (snap.comboHolding) {
        snprintf(line4, sizeof(line4), "Upload in %3d%%",
                 static_cast<int>(clampHoldPct(snap.uploadHoldPct) * 100.0f + 0.5f));
    } else if (showAutoHold) {
        snprintf(line4, sizeof(line4), "Hold UP->AUTO %3d%%",
                 static_cast<int>(clampHoldPct(snap.autoHoldPct) * 100.0f + 0.5f));
    } else if (!snap.commHealthy) {
        snprintf(line4, sizeof(line4), "!STM32 COMM RETRY!");
    } else if (snap.activeSource == ControlSource::Auto) {
        const unsigned sec    = static_cast<unsigned>((snap.autoElapsedMs / 1000UL) % 100UL);
        const unsigned tenths = static_cast<unsigned>((snap.autoElapsedMs / 100UL) % 10UL);
        snprintf(line4, sizeof(line4), "AUTO %2u.%1us X:Exit", sec, tenths);
    } else if (snap.ps2Link == LinkState::Disconnected &&
               snap.activeSource == ControlSource::None) {
        snprintf(line4, sizeof(line4), "D2+D3:PS2 | USB:WASD");
    } else {
        snprintf(line4, sizeof(line4), "RC:%d/%d/%d",
                 static_cast<int>(snap.servos.rc1Deg),
                 static_cast<int>(snap.servos.rc2Deg),
                 static_cast<int>(snap.servos.rc3Deg));
    }

    uint32_t h = fnv1aHash(line1, 2166136261UL);
    h          = fnv1aHash(line2, h);
    h          = fnv1aHash(line3, h);
    h          = fnv1aHash(line4, h);
    if (snap.comboHolding) {
        h ^= static_cast<uint32_t>(clampHoldPct(snap.uploadHoldPct) * 32.0f);
    } else if (showAutoHold) {
        h ^= static_cast<uint32_t>(clampHoldPct(snap.autoHoldPct) * 32.0f);
    }

    if (!force && hasDisplayHash_ && h == lastDisplayHash_) {
        return;
    }
    lastOledMs_ = nowMs;

    MiniR4.OLED.clearDisplay();
    MiniR4.OLED.setTextWrap(false);
    MiniR4.OLED.setTextSize(1);
    MiniR4.OLED.setTextColor(SSD1306_WHITE);
    MiniR4.OLED.setCursor(0, 0);
    MiniR4.OLED.print(line1);
    MiniR4.OLED.setCursor(0, 8);
    MiniR4.OLED.print(line2);

    if (snap.comboHolding || showAutoHold) {
        const float holdPct = clampHoldPct(snap.comboHolding ? snap.uploadHoldPct : snap.autoHoldPct);
        MiniR4.OLED.setCursor(0, 17);
        MiniR4.OLED.print(line4);
        int barW = static_cast<int>(holdPct * 124.0f + 0.5f);
        if (barW > 124) {
            barW = 124;
        }
        MiniR4.OLED.drawRect(0, 27, 126, 4, SSD1306_WHITE);
        if (barW > 0) {
            MiniR4.OLED.fillRect(1, 28, barW, 2, SSD1306_WHITE);
        }
    } else {
        MiniR4.OLED.setCursor(0, 16);
        MiniR4.OLED.print(line3);
        MiniR4.OLED.setCursor(0, 24);
        MiniR4.OLED.print(line4);
    }

    const uint8_t flushErr = flushBufferChecked(MiniR4.OLED.getBuffer(), Config::kOledBufferBytes);
    if (flushErr == 0U) {
        lastDisplayHash_ = h;
        hasDisplayHash_  = true;
    } else {
        hasDisplayHash_ = false;
    }
}

namespace {

void scanBusAddrs(const char* label, TwoWire& bus) {
    Serial.print(F("[I2C] "));
    Serial.print(label);
    Serial.print(F(":"));
    bus.setWireTimeout(10000U, true);
    uint8_t found = 0U;
    for (uint8_t addr = 0x08U; addr <= 0x77U; ++addr) {
        WDT.refresh();
        const size_t rxOk = bus.requestFrom(addr, static_cast<size_t>(1));
        if (rxOk == 1U) {
            Serial.print(F(" 0x"));
            if (addr < 0x10U) {
                Serial.print('0');
            }
            Serial.print(addr, HEX);
            found++;
        } else if (addr == Config::kOledAddrPrimary || addr == Config::kOledAddrAlt) {
            const uint8_t txErr = probeSsd1306Cmd(bus, addr, 0xE3);
            if (txErr == 0U) {
                Serial.print(F(" 0x"));
                Serial.print(addr, HEX);
                Serial.print(F("(w)"));
                found++;
            }
        } else if (addr == Config::kI2cMuxAddr) {
            // Probe PCA9548A by disabling all channels (0x00) instead of writing SSD1306 commands.
            bus.beginTransmission(addr);
            bus.write(static_cast<uint8_t>(0x00U));
            if (bus.endTransmission() == 0U) {
                Serial.print(F(" 0x70(w)"));
                found++;
            }
        }
    }
    bus.setWireTimeout(static_cast<unsigned int>(Config::kWireTimeoutUs), true);
    if (found == 0U) {
        Serial.print(F(" <none>"));
    }
    Serial.println();
}

} // namespace

void OledUI::printI2cDiag() {
    Serial.print(F("[OLED-DIAG] sdaB="));
    Serial.print(bootSdaBefore_);
    Serial.print(F(" sdaA="));
    Serial.print(bootSdaAfter_);
    Serial.print(F(" clk="));
    Serial.print(bootClocks_);
    Serial.print(F(" P400=0x"));
    Serial.print(R_PFS->PORT[4].PIN[0].PmnPFS, HEX);
    Serial.print(F(" P401=0x"));
    Serial.println(R_PFS->PORT[4].PIN[1].PmnPFS, HEX);

    scanBusAddrs("Wire1@cur", Wire1);

    Wire1.setClock(Config::kI2cSlowHz);
    Wire1.setWireTimeout(static_cast<unsigned int>(Config::kWireTimeoutUs), true);
    scanBusAddrs("Wire1@100k", Wire1);

    // Check PCA9548A channels 0..3 if 0x70 exists on Wire1
    if (Wire1.requestFrom(static_cast<uint8_t>(Config::kI2cMuxAddr), static_cast<size_t>(1)) == 1U) {
        for (uint8_t ch = 0U; ch < 4U; ++ch) {
            WDT.refresh();
            Wire1.beginTransmission(static_cast<uint8_t>(Config::kI2cMuxAddr));
            Wire1.write(static_cast<uint8_t>(1U << ch));
            Wire1.endTransmission();
            Serial.print(F("[I2C] mux ch="));
            Serial.print(ch);
            scanBusAddrs(" Wire1", Wire1);
        }
        Wire1.beginTransmission(static_cast<uint8_t>(Config::kI2cMuxAddr));
        Wire1.write(static_cast<uint8_t>(0x00U));
        Wire1.endTransmission();
    }

    Wire.begin();
    Wire.setWireTimeout(static_cast<unsigned int>(Config::kWireTimeoutUs), true);
    scanBusAddrs("Wire@100k", Wire);

    Serial.print(F("[OLED-DIAG] e3d="));
    Serial.print(probeSsd1306Cmd(Wire1, static_cast<uint8_t>(Config::kOledAddrAlt), SSD1306_DISPLAYON));
    Serial.print(F(" e3c="));
    Serial.println(probeSsd1306Cmd(Wire1, static_cast<uint8_t>(Config::kOledAddrPrimary), SSD1306_DISPLAYON));

    Wire1.setClock(fastClockOk_ ? Config::kI2cFastHz : Config::kI2cSlowHz);
}
