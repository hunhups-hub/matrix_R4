#ifdef ARDUINO

#include "ps2_driver.h"

#include <Arduino.h>
#include "robot_logic.h"

namespace {

inline R_PORT0_Type* resolvePort(uint8_t pin, uint16_t& outMask) {
    if (pin >= PINS_COUNT) {
        outMask = 0U;
        return nullptr;
    }
    const uint16_t raw = g_pin_cfg[pin].pin;
    const uint8_t portIdx = static_cast<uint8_t>(raw >> 8);
    const uint8_t bitIdx  = static_cast<uint8_t>(raw & 0xFFU);

    if (bitIdx >= 16U) {
        outMask = 0U;
        return nullptr;
    }

    R_PORT0_Type* port = nullptr;
    switch (portIdx) {
        case 0U: port = R_PORT0; break;
        case 1U: port = R_PORT1; break;
        case 2U: port = R_PORT2; break;
        case 3U: port = R_PORT3; break;
        case 4U: port = R_PORT4; break;
        case 5U: port = R_PORT5; break;
        case 6U: port = R_PORT6; break;
        case 7U: port = R_PORT7; break;
        default:
            outMask = 0U;
            return nullptr;
    }

    outMask = static_cast<uint16_t>(1U << bitIdx);
    return port;
}

inline void fastSetHigh(void* portPtr, uint16_t mask, uint8_t fallbackPin) {
    if (portPtr != nullptr) {
        static_cast<R_PORT0_Type*>(portPtr)->POSR = mask;
    } else {
        digitalWrite(fallbackPin, HIGH);
    }
}

inline void fastSetLow(void* portPtr, uint16_t mask, uint8_t fallbackPin) {
    if (portPtr != nullptr) {
        static_cast<R_PORT0_Type*>(portPtr)->PORR = mask;
    } else {
        digitalWrite(fallbackPin, LOW);
    }
}

inline bool fastRead(const void* portPtr, uint16_t mask, uint8_t fallbackPin) {
    if (portPtr != nullptr) {
        return (static_cast<const R_PORT0_Type*>(portPtr)->PIDR & mask) != 0U;
    }
    return digitalRead(fallbackPin) != LOW;
}

} // namespace

bool Ps2Driver::begin(uint8_t clkPin,
                      uint8_t cmdPin,
                      uint8_t attPin,
                      uint8_t datPin,
                      uint32_t bootTimeoutMs) {
    clk_.pin  = clkPin;
    cmd_.pin  = cmdPin;
    att_.pin  = attPin;
    dat_.pin  = datPin;

    clk_.port = resolvePort(clkPin, clk_.mask);
    cmd_.port = resolvePort(cmdPin, cmd_.mask);
    att_.port = resolvePort(attPin, att_.mask);
    dat_.port = resolvePort(datPin, dat_.mask);

    initialized_ = true;
    restorePins();

    const uint32_t startMs = millis();
    for (;;) {
        if (tryConnectSingleCycle()) {
            return true;
        }
        if (static_cast<uint32_t>(millis() - startMs) >= bootTimeoutMs) {
            break;
        }
        delay(Config::kPs2InitRetryGapMs);
    }

    return false;
}

void Ps2Driver::restorePins() {
    if (!initialized_) {
        return;
    }
    // Preload idle HIGH levels before setting OUTPUT to avoid spurious CS edges
    digitalWrite(clk_.pin, HIGH);
    digitalWrite(cmd_.pin, HIGH);
    digitalWrite(att_.pin, HIGH);

    pinMode(clk_.pin, OUTPUT);
    pinMode(cmd_.pin, OUTPUT);
    pinMode(att_.pin, OUTPUT);
    // Keep internal pull-up active so floating DAT cleanly reads 0xFF when unplugged
    pinMode(dat_.pin, INPUT_PULLUP);
}

bool Ps2Driver::tryConnect(uint8_t maxAttempts) {
    if (!initialized_) {
        return false;
    }
    for (uint8_t attempt = 0U; attempt < maxAttempts; attempt++) {
        if (tryConnectSingleCycle()) {
            return true;
        }
        if (attempt + 1U < maxAttempts) {
            delay(Config::kPs2InitRetryGapMs);
        }
    }
    return false;
}

bool Ps2Driver::tryConnectSingleCycle() {
    if (!initialized_) {
        return false;
    }

    // 1. Enter configuration mode (probe bus before running full config/wait sequence)
    spiBegin();
    const uint8_t r0 = spiTxRx(0x01U);
    const uint8_t r1 = spiTxRx(0x43U);
    const uint8_t r2 = spiTxRx(0x00U);
    const uint8_t r3 = spiTxRx(0x01U);
    const uint8_t r4 = spiTxRx(0x00U);
    spiEnd();

    // If DAT stays idle HIGH (0xFF) or shorted LOW (0x00) across all 5 bytes, bail early (~0.8 ms).
    if ((r0 == 0xFFU && r1 == 0xFFU && r2 == 0xFFU && r3 == 0xFFU && r4 == 0xFFU) ||
        (r0 == 0x00U && r1 == 0x00U && r2 == 0x00U && r3 == 0x00U && r4 == 0x00U)) {
        return false;
    }

    // 2. Enable & lock Analog Red mode (0x73)
    spiBegin();
    (void)spiTxRx(0x01U);
    (void)spiTxRx(0x44U);
    (void)spiTxRx(0x00U);
    (void)spiTxRx(0x01U);
    (void)spiTxRx(0x03U);
    (void)spiTxRx(0x00U);
    (void)spiTxRx(0x00U);
    (void)spiTxRx(0x00U);
    (void)spiTxRx(0x00U);
    spiEnd();

    // 3. Exit configuration mode
    spiBegin();
    (void)spiTxRx(0x01U);
    (void)spiTxRx(0x43U);
    (void)spiTxRx(0x00U);
    (void)spiTxRx(0x00U);
    (void)spiTxRx(0x5AU);
    (void)spiTxRx(0x5AU);
    (void)spiTxRx(0x5AU);
    (void)spiTxRx(0x5AU);
    (void)spiTxRx(0x5AU);
    spiEnd();

    // Give wireless receivers (e.g. MJ2 2.4 GHz) time to lock analog mode
    delay(2);

    // 4. Verify Analog Red header (0x73 / 0x5A)
    spiBegin();
    (void)spiTxRx(0x01U);
    const uint8_t modeId   = spiTxRx(0x42U);
    const uint8_t syncByte = spiTxRx(0x00U);
    for (uint8_t i = 0U; i < 6U; i++) {
        (void)spiTxRx(0x00U);
    }
    spiEnd();

    return (modeId == Config::kPs2ExpectedMode) && (syncByte == Config::kPs2ExpectedSync);
}

void Ps2Driver::poll(Ps2Frame& outFrame) {
    if (!initialized_) {
        outFrame = Ps2Frame{};
        return;
    }

    uint8_t raw[9] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};

    spiBegin();
    raw[0] = spiTxRx(0x01U);
    raw[1] = spiTxRx(0x42U);
    for (uint8_t i = 2U; i < 9U; i++) {
        raw[i] = spiTxRx(0x00U);
    }
    spiEnd();

    RobotLogic::decodePs2Payload(raw, outFrame);
}

void Ps2Driver::spiBegin() {
    fastSetHigh(cmd_.port, cmd_.mask, cmd_.pin);
    fastSetHigh(clk_.port, clk_.mask, clk_.pin);
    fastSetLow(att_.port, att_.mask, att_.pin);
    delayMicroseconds(Config::kPs2AttSettleUs);
}

void Ps2Driver::spiEnd() {
    fastSetHigh(cmd_.port, cmd_.mask, cmd_.pin);
    fastSetHigh(clk_.port, clk_.mask, clk_.pin);
    fastSetHigh(att_.port, att_.mask, att_.pin);
    delayMicroseconds(Config::kPs2PacketGapUs);
}

uint8_t Ps2Driver::spiTxRx(uint8_t command) {
    uint8_t rxData = 0x00U;

    {
        // Mask interrupts strictly for one byte (~130 us); SysTick/UART run in byte gaps
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();

        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((command & static_cast<uint8_t>(1U << bit)) != 0U) {
                fastSetHigh(cmd_.port, cmd_.mask, cmd_.pin);
            } else {
                fastSetLow(cmd_.port, cmd_.mask, cmd_.pin);
            }
            fastSetLow(clk_.port, clk_.mask, clk_.pin);
            delayMicroseconds(Config::kPs2ClockHalfPeriodUs);

            if (fastRead(dat_.port, dat_.mask, dat_.pin)) {
                rxData |= static_cast<uint8_t>(1U << bit);
            }
            fastSetHigh(clk_.port, clk_.mask, clk_.pin);
            delayMicroseconds(Config::kPs2ClockHalfPeriodUs);
        }

        fastSetHigh(cmd_.port, cmd_.mask, cmd_.pin);
        __set_PRIMASK(primask);
    }

    delayMicroseconds(Config::kPs2ByteGapUs);
    return rxData;
}

#endif // ARDUINO
