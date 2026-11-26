// Serial command actions and the config toggles shared with PS2.

#include "app_state.h"
#include <WDT.h>

namespace App {

void savePersistentConfig() {
    PersistentConfig cfg{};
    cfg.leftReversed     = g_snap.leftReversed;
    cfg.rightReversed    = g_snap.rightReversed;
    cfg.precisionMode    = false; // Session-only: always boot in 100% normal mode
    cfg.rc2SmoothEnabled = g_snap.rc2SmoothEnabled;
    cfg.motorTrimPct     = g_snap.motorTrimPct;
    g_io.saveConfigToEeprom(cfg);
}

void printTrimChanged(const char* src, int8_t trimPct) {
    Serial.print(F("[CFG] "));
    Serial.print(src);
    Serial.print(F(" Motor Trim = "));
    if (trimPct > 0) {
        Serial.print('+');
    }
    Serial.print(static_cast<int>(trimPct));
    Serial.println(F("% (saved to EEPROM)"));
}

/** Toggles Precision mode; the event text reads the power caps from Config. */
void togglePrecisionMode(const char* prefix) {
    g_snap.precisionMode = !g_snap.precisionMode;
    g_oled.beepConfirm();
    char msg[48];
    if (g_snap.precisionMode) {
        snprintf(msg, sizeof(msg), "%sPrecision %d%% ON (session)", prefix,
                 static_cast<int>(Config::kPrecisionMaxPower));
    } else {
        snprintf(msg, sizeof(msg), "%sPrecision OFF %d%%", prefix,
                 static_cast<int>(Config::kNormalMaxPower));
    }
    g_serial.printEvent("CFG", msg);
}

void toggleReverseDrive(const char* onMsg, const char* offMsg) {
    g_snap.reverseDrive = !g_snap.reverseDrive;
    g_oled.beepConfirm();
    g_serial.printEvent("CFG", g_snap.reverseDrive ? onMsg : offMsg);
}

void toggleRc2Smooth(const char* onMsg, const char* offMsg) {
    g_snap.rc2SmoothEnabled = !g_snap.rc2SmoothEnabled;
    g_io.setRc2SmoothEnabled(g_snap.rc2SmoothEnabled);
    savePersistentConfig();
    g_oled.beepConfirm();
    g_serial.printEvent("CFG", g_snap.rc2SmoothEnabled ? onMsg : offMsg);
}

void setMotorTrim(int8_t trimPct, const char* src) {
    g_snap.motorTrimPct = trimPct;
    savePersistentConfig();
    g_oled.beepConfirm();
    printTrimChanged(src, g_snap.motorTrimPct);
}

void toggleMotorReverse(bool& reversed, const char* onMsg, const char* offMsg) {
    reversed = !reversed;
    g_io.setMotorPolarity(g_snap.leftReversed, g_snap.rightReversed);
    savePersistentConfig();
    g_oled.beepConfirm();
    g_serial.printEvent("CFG", reversed ? onMsg : offMsg);
}

void applySerialConfigActions(const SerialActions& act) {
    if (act.togglePrecision) {
        togglePrecisionMode("");
    }
    if (act.toggleReverseDrive) {
        toggleReverseDrive("Reverse-Front ON", "Reverse-Front OFF");
    }
    if (act.toggleRc2Smooth) {
        toggleRc2Smooth("RC2 Smooth Slew ON (saved)", "RC2 Smooth Slew OFF (saved)");
    }

    if (act.trimLeftStep) {
        setMotorTrim(RobotLogic::stepMotorTrim(g_snap.motorTrimPct, -1), "Serial");
    } else if (act.trimRightStep) {
        setMotorTrim(RobotLogic::stepMotorTrim(g_snap.motorTrimPct, 1), "Serial");
    } else if (act.trimReset) {
        setMotorTrim(0, "Serial");
    }

    if (act.toggleLeftReverse) {
        toggleMotorReverse(g_snap.leftReversed, "M1 reverse=ON (saved)", "M1 reverse=OFF (saved)");
    }
    if (act.toggleRightReverse) {
        toggleMotorReverse(g_snap.rightReversed, "M2 reverse=ON (saved)", "M2 reverse=OFF (saved)");
    }

    if (act.speedPctChanged) {
        g_snap.serialSpeedPct = act.newSpeedPct;
        Serial.print(F("[SER] Speed = "));
        Serial.print(g_snap.serialSpeedPct);
        Serial.println('%');
    }
}

void applySerialServoActions(const SerialActions& act) {
    ServoTargets& servos = g_snap.servos;

    if (act.armUpStep) {
        setPoseServo(servos.rc2Deg, RobotLogic::computeArmStep(servos.rc2Deg, true, false));
    } else if (act.armDownStep) {
        setPoseServo(servos.rc2Deg, RobotLogic::computeArmStep(servos.rc2Deg, false, true));
    }

    if (act.gripOpen) {
        setGripServo(servos.rc3Deg, Config::kRc3L1Deg);
    } else if (act.gripClose) {
        setGripServo(servos.rc3Deg, Config::kRc3R1Deg);
    }

    if (act.auxExtend) {
        setPoseServo(servos.rc1Deg, Config::kRc1L2Deg);
    } else if (act.auxRetract) {
        setPoseServo(servos.rc1Deg, Config::kRc1R2Deg);
    } else if (act.togglePosePreset) {
        togglePosePreset();
    }

    if (act.hasDirectServo) {
        switch (act.directServoCh) {
            case 1: setPoseServo(servos.rc1Deg, act.directServoDeg); break;
            case 2: setPoseServo(servos.rc2Deg, act.directServoDeg); break;
            case 3: setGripServo(servos.rc3Deg, act.directServoDeg); break;
            default: break;
        }
    }

    g_snap.servos      = RobotLogic::clampServoTargets(g_snap.servos);
    g_snap.comScalePct = RobotLogic::computeCoMDynamicScale(g_snap.servos.rc2Deg);
}

void applySerialImmediateActions(const SerialActions& act, uint32_t nowMs) {
    if (act.i2cScanRequested) {
        g_oled.printI2cDiag();
    }
    if (act.uploadRequested) {
        enterUploadBootloader("Serial command 'u'");
    }
    if (act.estopRequested) {
        g_serialAutoRequested = false;
        triggerEstop("Serial '!'", nowMs);
    } else if (act.resumeRequested) {
        clearEstop("Serial 'r'");
    }

    if (act.autoStartRequested && g_snap.sysState == SystemState::Ready) {
        g_serialAutoRequested = true;
    }

    applySerialConfigActions(act);

    if (g_snap.sysState == SystemState::Estop) {
        cancelSerialDrive();
    } else {
        applySerialServoActions(act);
        g_serialDrive.latched = act.driveCmd.active;
        g_serialDrive.targetL = act.driveCmd.targetLeft;
        g_serialDrive.targetR = act.driveCmd.targetRight;
    }
}

} // namespace App
