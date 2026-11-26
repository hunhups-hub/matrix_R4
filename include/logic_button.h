#pragma once

// BTN_UP / BTN_DOWN debounce and gesture logic (pure C++11, host-testable).

#include <stdint.h>
#include "types.h"

namespace RobotLogic {

struct DebouncedButton {
    bool     rawState     = false;
    bool     stableState  = false;
    uint32_t lastChangeMs = 0;
    bool     pressLatched = false;
};

/**
 * Turns alternating BTN_UP / BTN_DOWN samples into gesture events:
 *   - BTN_DOWN press edge (E-stop, fires at once, never suppressed)
 *   - short release of UP / DOWN
 *   - UP + DOWN held for kUploadComboHoldMs -> uploadRequested (with hold ratio)
 *   - UP alone held for kAutoStartBtnHoldMs -> autoStartRequested (with hold ratio)
 * A button held at boot suppresses all gestures until both buttons are released.
 */
class ButtonGestureTracker {
public:
    /** Seeds the initial held state at boot; gestures stay suppressed while either button is held. */
    void seedInitial(bool upHeld, bool downHeld);

    /** Forgets all state (STM32 offline at boot). */
    void clearAll();

    /** Feeds one valid sample (call only on ticks where that button was actually read). */
    void feedUp(bool sample, uint32_t nowMs);
    void feedDown(bool sample, uint32_t nowMs);

    /** A read failed: drop held state so a stale press can never trigger a gesture. */
    void failUp();
    void failDown();

    /** Derives this tick's events. `pollRan` is true when a button was sampled this tick. */
    void evaluate(uint32_t nowMs, bool pollRan, ButtonEvents& outEvents);

    bool upHeld() const { return btnUp_.stableState; }
    bool downHeld() const { return btnDown_.stableState; }

private:
    static void debounce(DebouncedButton& btn,
                         bool sample,
                         uint32_t nowMs,
                         bool& outShortRelease,
                         bool& outPressEdge);

    DebouncedButton btnUp_{};
    DebouncedButton btnDown_{};
    bool            suppress_        = true;
    bool            comboPrevActive_ = false;
    bool            comboFired_      = false;
    uint32_t        comboStartMs_    = 0;
    bool            upHoldPrevActive_ = false;
    bool            upHoldFired_     = false;
    uint32_t        upHoldStartMs_   = 0;
    bool            shortUp_         = false;
    bool            shortDown_       = false;
    bool            downEdge_        = false;
};

} // namespace RobotLogic
