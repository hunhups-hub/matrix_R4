#pragma once

#include <stdint.h>
#include "types.h"

class OledUI {
public:
    static void recoverBus(uint8_t& sdaBefore, uint8_t& sdaAfter, uint8_t& clocksSent);

    bool begin();
    void showBootloaderScreen();
    void render(const SystemSnapshot& snap, uint32_t nowMs, bool force = false);

    uint8_t i2cAddr() const { return oledAddr_; }
    uint8_t lastI2cErr() const { return lastI2cErr_; }
    uint8_t err3d() const { return err3d_; }
    uint8_t err3c() const { return err3c_; }
    uint8_t bootSdaBefore() const { return bootSdaBefore_; }
    uint8_t bootSdaAfter() const { return bootSdaAfter_; }
    uint8_t bootRecoverClocks() const { return bootClocks_; }
    void    printI2cDiag();

    void beepConfirm();
    void beepWarning();
    void beepBoot();

private:
    void updateRgbLeds(const SystemSnapshot& snap, uint32_t nowMs);
    uint8_t flushBufferChecked(const uint8_t* buf, uint16_t len);

    uint32_t lastOledMs_      = 0;
    uint32_t lastDisplayHash_ = 0;
    uint32_t lastLed1Color_   = 0xFFFFFFFFUL;
    uint32_t lastLed2Color_   = 0xFFFFFFFFUL;
    uint8_t  oledAddr_        = 0x3D;
    uint8_t  lastI2cErr_      = 0U;
    uint8_t  err3d_           = 0xFF;
    uint8_t  err3c_           = 0xFF;
    uint8_t  bootSdaBefore_   = 1U;
    uint8_t  bootSdaAfter_    = 1U;
    uint8_t  bootClocks_      = 0U;
    bool     fastClockOk_     = true;
};
