#include "robot_logic.h"
#include "logic_button.h"

namespace RobotLogic {

void ButtonGestureTracker::debounce(DebouncedButton& btn,
                                    bool sample,
                                    uint32_t nowMs,
                                    bool& outShortRelease,
                                    bool& outPressEdge) {
    outShortRelease = false;
    outPressEdge    = false;
    if (sample != btn.rawState) {
        btn.rawState     = sample;
        btn.lastChangeMs = nowMs;
    }

    if (static_cast<uint32_t>(nowMs - btn.lastChangeMs) >= Config::kButtonDebounceMs &&
        btn.stableState != btn.rawState) {
        btn.stableState = btn.rawState;
        if (btn.stableState) {
            btn.pressLatched = true;
            outPressEdge     = true;
        } else if (btn.pressLatched) {
            btn.pressLatched = false;
            outShortRelease  = true;
        }
    }
}

void ButtonGestureTracker::seedInitial(bool upHeld, bool downHeld) {
    btnUp_.rawState    = upHeld;
    btnUp_.stableState = upHeld;
    btnDown_.rawState    = downHeld;
    btnDown_.stableState = downHeld;
    suppress_ = upHeld || downHeld;
}

void ButtonGestureTracker::clearAll() {
    btnUp_    = DebouncedButton{};
    btnDown_  = DebouncedButton{};
    suppress_ = false;
}

void ButtonGestureTracker::feedUp(bool sample, uint32_t nowMs) {
    bool pressEdge = false;
    debounce(btnUp_, sample, nowMs, shortUp_, pressEdge);
}

void ButtonGestureTracker::feedDown(bool sample, uint32_t nowMs) {
    debounce(btnDown_, sample, nowMs, shortDown_, downEdge_);
}

void ButtonGestureTracker::failUp() {
    btnUp_            = DebouncedButton{};
    comboPrevActive_  = false;
    comboFired_       = false;
    upHoldPrevActive_ = false;
    upHoldFired_      = false;
}

void ButtonGestureTracker::failDown() {
    btnDown_         = DebouncedButton{};
    comboPrevActive_ = false;
    comboFired_      = false;
}

void ButtonGestureTracker::evaluate(uint32_t nowMs, bool pollRan, ButtonEvents& outEvents) {
    outEvents = ButtonEvents{};

    const bool shortUp   = shortUp_;
    const bool shortDown = shortDown_;
    const bool downEdge  = downEdge_;
    shortUp_   = false;
    shortDown_ = false;
    downEdge_  = false;

    outEvents.upPressed = btnUp_.stableState;

    // E-stop must fire the instant BTN_DOWN is pressed (never suppressed, never waits for release).
    // The press is consumed here so releasing the button does not emit a second shortDown event.
    if (downEdge) {
        outEvents.downPressEdge = true;
        btnDown_.pressLatched   = false;
    }

    if (suppress_) {
        if (!btnUp_.stableState && !btnDown_.stableState) {
            suppress_             = false;
            btnUp_.pressLatched   = false;
            btnDown_.pressLatched = false;
        }
        return;
    }

    const bool bothHeld = btnUp_.stableState && btnDown_.stableState;
    if (bothHeld) {
        // Prevent single-button release actions after releasing an upload combo.
        btnUp_.pressLatched   = false;
        btnDown_.pressLatched = false;
        upHoldPrevActive_     = false;
        upHoldFired_          = false;

        if (!comboPrevActive_) {
            comboPrevActive_ = true;
            comboFired_      = false;
            comboStartMs_    = nowMs;
        }

        const uint32_t elapsed = static_cast<uint32_t>(nowMs - comboStartMs_);
        outEvents.comboHolding = true;
        float ratio            = static_cast<float>(elapsed) / static_cast<float>(Config::kUploadComboHoldMs);
        if (ratio > 1.0f) {
            ratio = 1.0f;
        }
        outEvents.uploadHoldRatio = ratio;

        if (!comboFired_ && elapsed >= Config::kUploadComboHoldMs) {
            comboFired_               = true;
            outEvents.uploadRequested = true;
        }
        return;
    }

    comboPrevActive_ = false;
    comboFired_      = false;

    // Single BTN_UP hold triggers Auto mode without requiring the PS2 controller.
    if (btnUp_.stableState && !btnDown_.stableState) {
        if (!upHoldPrevActive_) {
            upHoldPrevActive_ = true;
            upHoldFired_      = false;
            upHoldStartMs_    = nowMs;
        }
        const uint32_t elapsed = static_cast<uint32_t>(nowMs - upHoldStartMs_);
        if (elapsed >= Config::kAutoHoldBarDelayMs) {
            outEvents.autoHolding = true;
            float ratio           = static_cast<float>(elapsed) / static_cast<float>(Config::kAutoStartBtnHoldMs);
            if (ratio > 1.0f) {
                ratio = 1.0f;
            }
            outEvents.autoHoldRatio = ratio;
        }
        if (!upHoldFired_ && elapsed >= Config::kAutoStartBtnHoldMs) {
            upHoldFired_                 = true;
            btnUp_.pressLatched          = false; // Suppress shortUp release action after hold fires
            outEvents.autoStartRequested = true;
        }
    } else {
        upHoldPrevActive_ = false;
        upHoldFired_      = false;
    }

    if (pollRan) {
        outEvents.shortUp   = shortUp;
        outEvents.shortDown = shortDown;
    }
}

} // namespace RobotLogic
