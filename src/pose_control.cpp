// Servo pose presets (Circle/Triangle/grip) shared by PS2 and Serial.

#include "app_state.h"
#include <WDT.h>

namespace App {

/** Manual RC1/RC2 moves cancel the Circle drop/hold preset toggle. */
void setPoseServo(int16_t& targetDeg, int16_t deg) {
    targetDeg                    = deg;
    g_posePresetState.dropActive = false;
}

/** Manual RC3 moves cancel the Square push-ground hold. */
void setGripServo(int16_t& targetDeg, int16_t deg) {
    targetDeg             = deg;
    g_armPushState.active = false;
}

void togglePosePreset() {
    char msg[48];
    if (!g_posePresetState.dropActive) {
        g_snap.servos.rc1Deg         = Config::kRc1PresetDropDeg;
        g_snap.servos.rc2Deg         = Config::kRc2PresetDropDeg;
        g_posePresetState.dropActive = true;
        g_posePresetState.releasePending = false;
        snprintf(msg, sizeof(msg), "Drop/Place preset (RC1=%d, RC2=%d)",
                 static_cast<int>(Config::kRc1PresetDropDeg),
                 static_cast<int>(Config::kRc2PresetDropDeg));
    } else {
        g_snap.servos.rc1Deg         = Config::kRc1PresetHoldDeg;
        g_snap.servos.rc2Deg         = Config::kRc2PresetHoldDeg;
        g_posePresetState.dropActive = false;
        g_posePresetState.releasePending = false;
        snprintf(msg, sizeof(msg), "Hold/Carry preset (RC1=%d, RC2=%d)",
                 static_cast<int>(Config::kRc1PresetHoldDeg),
                 static_cast<int>(Config::kRc2PresetHoldDeg));
    }
    g_serial.printEvent("POSE", msg);
    g_oled.beepConfirm();
}

} // namespace App
