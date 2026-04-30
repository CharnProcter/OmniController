#pragma once

#include <Arduino.h>

// OmniUartFlasher — drives the C6 daughterboard's EN/BOOT lines and (in M-β.2)
// runs esp-serial-flasher to write firmware over UART.
//
// Pin contract:
//   en   — output. Held HIGH (or INPUT_PULLUP) idle. Pulsed LOW to reset the C6.
//   boot — dual-purpose. During UART flash sessions: output. Idle: caller can
//          repurpose as INPUT_PULLUP for HANDSHAKE in M-γ. This class always
//          leaves both pins in INPUT_PULLUP after each operation so post-flash
//          HANDSHAKE is unaffected.
//
// All sequence timings come from the ESP32 ROM bootloader's documented
// strap-pin requirements (≥10 ms BOOT-low before EN release; ≥50 ms EN-low for
// reliable reset).

namespace omni {

enum class FlasherAction : uint8_t {
    None       = 0,
    Reset      = 1,
    Bootloader = 2,
    Probe      = 3,  // M-β.2 smoke test — connect, read chip ID, disconnect
    Flashing   = 4,  // reserved for M-β.2 part B
};

const char* flasherActionName(FlasherAction a);

struct ProbeResult {
    bool        ok;
    int32_t     errorCode;     // raw esp_loader_error_t cast to int (0 = success)
    const char* chipName;      // resolved chip name (e.g. "ESP32-C6"), or "unknown"
    uint32_t    durationMs;
};

class OmniUartFlasher {
public:
    // esp-serial-flasher enforces size <= block_size on every esp_loader_flash_write
    // call. Callers feeding bytes from another source (USB-CDC, HTTP) must cap
    // their per-call chunk at this value to avoid ESP_LOADER_ERROR_INVALID_PARAM.
    static constexpr uint32_t kBlockSize = 2048;

    void begin(uint8_t enPin, uint8_t bootPin, uint8_t uartTxPin, uint8_t uartRxPin);

    // Pulse EN low to reset the C6 into normal application mode.
    void resetTarget();

    // Hold BOOT low while pulsing EN to enter the ROM serial bootloader.
    // Caller is responsible for any subsequent flash protocol (M-β.2).
    void enterBootloader();

    // M-β.2 smoke test: enter bootloader, run esp-serial-flasher's connect
    // handshake, query the target chip ID, deinit, return result. Intended
    // to verify the integration links and the protocol speaks to the C6
    // before wiring the full /omniC6Ota flash flow.
    ProbeResult probeC6Target();

    // Streaming flash session. Multipart upload handlers call flashBegin()
    // once on the first chunk, flashWrite() for every chunk, then flashFinish()
    // on the last. flashAbort() cleans up if the upload fails mid-stream.
    bool     flashBegin(uint32_t imageSize, uint32_t flashOffset);
    bool     flashWrite(const uint8_t* data, uint32_t len);
    bool     flashFinish(bool reboot);
    void     flashAbort();

    // Flash session state for /omniC6Status.
    bool     flashActive()    const { return _flashActive; }
    bool     lastFlashOk()    const { return _lastFlashOk; }
    int32_t  lastFlashError() const { return _lastFlashError; }
    uint32_t lastFlashSize()  const { return _lastFlashSize; }
    uint32_t lastFlashOffset()const { return _lastFlashOffset; }
    uint32_t lastFlashMs()    const { return _lastFlashMs; }

    // Status accessors.
    FlasherAction lastAction() const { return _lastAction; }
    uint32_t      lastActionMs() const { return _lastActionMs; }
    bool          began() const { return _began; }
    uint8_t       enPin() const { return _enPin; }
    uint8_t       bootPin() const { return _bootPin; }
    uint8_t       uartTxPin() const { return _uartTxPin; }
    uint8_t       uartRxPin() const { return _uartRxPin; }

private:
    void driveEnLow();
    void releaseEn();
    void driveBootLow();
    void releaseBoot();
    void releaseStrapPins();
    void recordAction(FlasherAction a);
    void detachUart0IoMux();
    bool initPersistentPort();
    void primeStrapPinsForFlash();

    uint8_t _enPin     = 0;
    uint8_t _bootPin   = 0;
    uint8_t _uartTxPin = 0;
    uint8_t _uartRxPin = 0;
    bool    _began      = false;
    bool    _portReady  = false;  // UART driver installed once and kept; see initPersistentPort
    FlasherAction _lastAction = FlasherAction::None;
    uint32_t      _lastActionMs = 0;

    // Flash session state.
    bool     _flashActive     = false;
    bool     _lastFlashOk     = false;
    int32_t  _lastFlashError  = 0;
    uint32_t _lastFlashSize   = 0;
    uint32_t _lastFlashOffset = 0;
    uint32_t _flashStartMs    = 0;
    uint32_t _lastFlashMs     = 0;
};

}  // namespace omni
