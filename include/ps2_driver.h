#pragma once

#include <stdint.h>
#include "config.h"
#include "types.h"

/**
 * Deterministic, bounded non-blocking bit-bang SPI driver for PS2 / MJ2 controllers
 * on Arduino UNO R4 WiFi (Renesas RA4M1 @ 48 MHz, D2 + D3 ports).
 *
 * Replaces MatrixMiniR4's blocking PS2X::read_gamepad() (which stalls 150-240 ms on
 * retries and reconfig_gamepad inside the control loop):
 * - poll() executes exactly one 9-byte SPI packet (~1.4 ms) with zero internal retries.
 * - tryConnect() runs bounded analog-mode config bursts only when Disconnected.
 */
class Ps2Driver {
public:
    bool begin(uint8_t clkPin,
               uint8_t cmdPin,
               uint8_t attPin,
               uint8_t datPin,
               uint32_t bootTimeoutMs = Config::kPs2BootTimeoutMs);

    bool tryConnect(uint8_t maxAttempts = Config::kPs2InitBurstAttempts);

    /** Restores idle HIGH SPI output levels and DAT pull-up (e.g. after Auto mode). */
    void restorePins();

    void poll(Ps2Frame& outFrame);

private:
    bool    tryConnectSingleCycle();
    void    spiBegin();
    void    spiEnd();
    uint8_t spiTxRx(uint8_t command);

    struct FastPin {
        void*    port = nullptr; // R_PORT0_Type* on Renesas RA4M1
        uint16_t mask = 0U;
        uint8_t  pin  = 0xFFU;
    };

    FastPin clk_{};
    FastPin cmd_{};
    FastPin att_{};
    FastPin dat_{};
    bool    initialized_ = false;
};
