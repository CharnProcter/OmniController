#include "OmniUartFlasher.h"

namespace omni {

namespace {
constexpr uint32_t kEnLowMs        = 50;   // hold EN low long enough for a clean reset
constexpr uint32_t kBootSettleMs   = 10;   // BOOT held low this long after EN release
constexpr uint32_t kPostReleaseMs  = 20;   // settle time after releasing strap pins
}  // namespace

const char* flasherActionName(FlasherAction a) {
    switch (a) {
        case FlasherAction::None:       return "none";
        case FlasherAction::Reset:      return "reset";
        case FlasherAction::Bootloader: return "bootloader";
        case FlasherAction::Flashing:   return "flashing";
    }
    return "unknown";
}

void OmniUartFlasher::begin(uint8_t enPin, uint8_t bootPin) {
    _enPin = enPin;
    _bootPin = bootPin;
    // Idle state: both pins floating high via internal pull-up. The hat's
    // external pull-up keeps the C6 running normally; leaving the S3 pins as
    // INPUT_PULLUP avoids fighting the hat and lets the BOOT line later
    // be driven low *by the C6* as HANDSHAKE in M-γ.
    pinMode(_enPin, INPUT_PULLUP);
    pinMode(_bootPin, INPUT_PULLUP);
    _began = true;
}

void OmniUartFlasher::resetTarget() {
    if (!_began) return;
    driveEnLow();
    delay(kEnLowMs);
    releaseEn();
    delay(kPostReleaseMs);
    recordAction(FlasherAction::Reset);
}

void OmniUartFlasher::enterBootloader() {
    if (!_began) return;
    // Sequence: EN low, BOOT low (so BOOT is sampled as 0 when EN releases),
    // EN release while BOOT still low, BOOT release after a short settle.
    driveBootLow();
    driveEnLow();
    delay(kEnLowMs);
    releaseEn();
    delay(kBootSettleMs);
    releaseBoot();
    delay(kPostReleaseMs);
    recordAction(FlasherAction::Bootloader);
}

void OmniUartFlasher::driveEnLow() {
    pinMode(_enPin, OUTPUT);
    digitalWrite(_enPin, LOW);
}

void OmniUartFlasher::releaseEn() {
    pinMode(_enPin, INPUT_PULLUP);
}

void OmniUartFlasher::driveBootLow() {
    pinMode(_bootPin, OUTPUT);
    digitalWrite(_bootPin, LOW);
}

void OmniUartFlasher::releaseBoot() {
    pinMode(_bootPin, INPUT_PULLUP);
}

void OmniUartFlasher::recordAction(FlasherAction a) {
    _lastAction = a;
    _lastActionMs = millis();
}

}  // namespace omni
