// Bootloader entry, EEPROM load, PS2 boot init and the boot report.

#include <boot.h>
#include "app_state.h"
#include <WDT.h>

namespace App {

[[noreturn]] void enterUploadBootloader(const char* source) {
    WDT.refresh();
    g_io.forceMotorsStop(true, millis(), true);
    g_oled.showBootloaderScreen();
    g_serial.printEvent("BOOT", source ? source : "Entering upload bootloader...");
    Serial.flush();
    delay(Config::kBootloaderFlushDelayMs);
    WDT.refresh();

#ifdef NO_USB
    // On UNO R4 WiFi (-DNO_USB), USB clock may be off; trigger SAM-BA double-tap via VBTBKR[0].
    R_SYSTEM->PRCR       = static_cast<uint16_t>(BSP_PRV_PRCR_PRC1_UNLOCK);
    BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;
    R_SYSTEM->PRCR       = static_cast<uint16_t>(BSP_PRV_PRCR_LOCK);
    __DSB();
    NVIC_SystemReset();
#else
    goBootloader();
#endif
    for (;;) {
    }
}

/** Loads EEPROM config into the snapshot and applies it to the IO layer. */
bool loadPersistentConfig() {
    PersistentConfig savedCfg{};
    const bool loaded = g_io.loadConfigFromEeprom(savedCfg);
    if (loaded) {
        g_snap.leftReversed     = savedCfg.leftReversed;
        g_snap.rightReversed    = savedCfg.rightReversed;
        g_snap.precisionMode    = false; // Session-only
        g_snap.rc2SmoothEnabled = savedCfg.rc2SmoothEnabled;
        g_snap.motorTrimPct     = savedCfg.motorTrimPct;
        g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
    }
    g_io.setRc2SmoothEnabled(g_snap.rc2SmoothEnabled);
    return loaded;
}

/** Initializes the PS2 link state machine and seeds edge latches from the first frames. */
bool initPs2Link(uint32_t nowMs) {
    const bool ps2Ready = g_io.isPs2ReadyAtBoot();
    g_linkSm.begin(ps2Ready, nowMs);
    if (ps2Ready) {
        bool bootValid = false;
        for (uint8_t i = 0U; i < Config::kPs2StableFrames; ++i) {
            g_io.pollPs2(g_lastPs2Frame);
            bootValid            = RobotLogic::isPs2FrameValid(g_lastPs2Frame);
            const bool modeReset = !bootValid && RobotLogic::isPs2UnconfiguredMode(g_lastPs2Frame);
            g_linkSm.onFrame(bootValid, nowMs, modeReset);
            if (!bootValid) {
                break;
            }
            if (i + 1U < Config::kPs2StableFrames) {
                delay(Config::kBootPs2FrameGapMs);
            }
        }
        latchAllPs2Edges(bootValid);
    }
    if (g_linkSm.isOperational()) {
        g_snap.activeSource = ControlSource::Ps2;
    }
    syncPs2Snapshot();
    return ps2Ready;
}

void printOledBootLine(uint8_t sdaBefore, uint8_t sdaAfter, uint8_t recoverClk) {
    const bool oledOk = g_snap.oledOk;
    Serial.print(F("[OLED] SSD1306 addr=0x"));
    Serial.print(g_oled.i2cAddr(), HEX);
    Serial.print(oledOk ? F(" OK (Wire1 ") : F(" ERR="));
    if (!oledOk) {
        Serial.print(g_oled.lastI2cErr());
        Serial.print(F(", Wire1 "));
    }
    Serial.print(Config::kWireTimeoutUs / 1000UL);
    Serial.print(F("ms preSda="));
    Serial.print(sdaBefore);
    Serial.print(F("->"));
    Serial.print(sdaAfter);
    Serial.print(F(" clk="));
    Serial.print(recoverClk);
    Serial.print(F(" rec="));
    Serial.print(g_oled.bootSdaBefore());
    Serial.print(F("->"));
    Serial.print(g_oled.bootSdaAfter());
    Serial.print('/');
    Serial.println(g_oled.bootRecoverClocks());
}

void printServoBootLine() {
    if (g_io.servoRangeErr() != 0U) {
        Serial.print(F("[WARN] Servo range err="));
        Serial.println(g_io.servoRangeErr());
        return;
    }
    Serial.print(F("[CFG] RC1..RC3 range 0..180deg OK | L1(RC3)=("));
    Serial.print(Config::kRc3L1Deg);
    Serial.print(F(") | Sq=("));
    Serial.print(Config::kRc3SquareDeg);
    Serial.print(F(") | inv3=("));
    Serial.print(Config::kRc3InvertDir ? 1 : 0);
    Serial.println(')');
}

void printBootReport(bool miniOk,
                     bool eepromLoaded,
                     uint8_t sdaBefore,
                     uint8_t sdaAfter,
                     uint8_t recoverClk) {
    g_serial.printBanner();
    printOledBootLine(sdaBefore, sdaAfter, recoverClk);
    if (!miniOk) {
        g_serial.printEvent("WARN", "MiniR4 init returned non-OK");
    }
    printServoBootLine();
    if (g_snap.wdtResetBoot) {
        g_serial.printEvent("WARN", "Previous reset caused by Watchdog");
    }
    if (eepromLoaded) {
        g_serial.printEvent("EEPROM", "Loaded saved config (rev/trim/rc2Smooth); Precision=100%");
    }
    g_serial.printSnapshot(g_snap);
}

} // namespace App
