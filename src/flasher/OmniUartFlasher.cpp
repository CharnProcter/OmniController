#include "OmniUartFlasher.h"

extern "C" {
#include "esp_loader.h"
#include "esp32_port.h"
}

#include "driver/uart.h"

namespace omni {

namespace {
constexpr uint32_t kEnLowMs        = 50;   // hold EN low long enough for a clean reset
constexpr uint32_t kBootSettleMs   = 10;   // BOOT held low this long after EN release
constexpr uint32_t kPostReleaseMs  = 20;   // settle time after releasing strap pins

constexpr uart_port_t kFlasherUartPort = UART_NUM_1;
constexpr uint32_t   kFlasherBaudRate  = 115200;
constexpr uint32_t   kFlasherFastBaud  = 460800;  // post-stub bump for throughput
constexpr uint32_t   kFlasherBlockSize = 4096;    // stub block; ROM mode is limited to 1024

const char* targetName(target_chip_t t) {
    switch (t) {
        case ESP8266_CHIP:     return "ESP8266";
        case ESP32_CHIP:       return "ESP32";
        case ESP32S2_CHIP:     return "ESP32-S2";
        case ESP32C3_CHIP:     return "ESP32-C3";
        case ESP32S3_CHIP:     return "ESP32-S3";
        case ESP32C2_CHIP:     return "ESP32-C2";
        case ESP32C5_CHIP:     return "ESP32-C5";
        case ESP32H2_CHIP:     return "ESP32-H2";
        case ESP32C6_CHIP:     return "ESP32-C6";
        case ESP32P4_CHIP:     return "ESP32-P4";
        case ESP_UNKNOWN_CHIP: return "unknown";
        default:               return "unknown";
    }
}
}  // namespace

const char* flasherActionName(FlasherAction a) {
    switch (a) {
        case FlasherAction::None:       return "none";
        case FlasherAction::Reset:      return "reset";
        case FlasherAction::Bootloader: return "bootloader";
        case FlasherAction::Probe:      return "probe";
        case FlasherAction::Flashing:   return "flashing";
    }
    return "unknown";
}

void OmniUartFlasher::begin(uint8_t enPin, uint8_t bootPin, uint8_t uartTxPin, uint8_t uartRxPin) {
    _enPin = enPin;
    _bootPin = bootPin;
    _uartTxPin = uartTxPin;
    _uartRxPin = uartRxPin;
    releaseStrapPins();
    _began = true;
    // Install the UART driver once, here, on whichever task called us
    // (Arduino main on Core 1 in practice). Never deinit. Both probe and
    // every flash session reuse this driver state. Cycling install/deinit
    // between sessions hit a UART driver state-corruption bug where the
    // tx_mutex came back NULL on subsequent inits — most visible when
    // calls alternate between Core 0 (AsyncTCP) and Core 1 (Arduino).
    initPersistentPort();
}

bool OmniUartFlasher::initPersistentPort() {
    if (_portReady) return true;
    detachUart0IoMux();

    loader_esp32_config_t cfg{};
    cfg.baud_rate         = kFlasherBaudRate;
    cfg.uart_port         = kFlasherUartPort;
    cfg.uart_rx_pin       = (gpio_num_t)_uartRxPin;
    cfg.uart_tx_pin       = (gpio_num_t)_uartTxPin;
    cfg.reset_trigger_pin = (gpio_num_t)_enPin;
    cfg.gpio0_trigger_pin = (gpio_num_t)_bootPin;

    esp_loader_error_t err = loader_port_esp32_init(&cfg);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.printf("OmniUartFlasher: persistent port_init failed (%d)\n", (int)err);
        return false;
    }
    _portReady = true;
    Serial.println("OmniUartFlasher: UART driver installed (persistent)");
    // port_esp32_init leaves both strap pins as OUTPUT HIGH. Hand them back
    // to INPUT_PULLUP so the hat's external pull-ups + C6 internal pull-up
    // dominate the line at idle, leaving HANDSHAKE-from-C6 unblocked.
    releaseStrapPins();
    return true;
}

void OmniUartFlasher::primeStrapPinsForFlash() {
    // Before each flash session, hand the strap pins back to esp-serial-flasher.
    // It expects them as OUTPUT and drives the levels itself during connect.
    pinMode(_enPin, OUTPUT);
    digitalWrite(_enPin, HIGH);
    pinMode(_bootPin, OUTPUT);
    digitalWrite(_bootPin, HIGH);
}

void OmniUartFlasher::releaseStrapPins() {
    // Idle state: both pins floating high via internal pull-up. The hat's
    // external pull-up keeps the C6 running normally; leaving the S3 pins as
    // INPUT_PULLUP avoids fighting the hat and lets the BOOT line later
    // be driven low *by the C6* as HANDSHAKE in M-γ.
    pinMode(_enPin, INPUT_PULLUP);
    pinMode(_bootPin, INPUT_PULLUP);
}

void OmniUartFlasher::detachUart0IoMux() {
    // Workaround for Arduino-ESP32 v3 quirk: GPIO 43/44 are wired to UART0
    // via IO_MUX by default, fighting any UART_NUM_1/2 driver that tries to
    // claim them via GPIO matrix. Open UART0 once and end() so Arduino-ESP32
    // marks the IO_MUX route detachable. See UARTPassThrough.cpp:34-37.
    HardwareSerial uart0(0);
    uart0.begin(115200);
    uart0.end();
}

ProbeResult OmniUartFlasher::probeC6Target() {
    ProbeResult result{false, -1, "unknown", 0};
    if (!_began || !_portReady) {
        result.chipName = "flasher not ready";
        return result;
    }

    uint32_t t0 = millis();

    Serial.printf("OmniUartFlasher: probing C6 (en=%u boot=%u tx=%u rx=%u uart=%d baud=%lu)\n",
                  _enPin, _bootPin, _uartTxPin, _uartRxPin,
                  (int)kFlasherUartPort, (unsigned long)kFlasherBaudRate);

    primeStrapPinsForFlash();
    Serial.println("OmniUartFlasher: calling esp_loader_connect...");

    esp_loader_connect_args_t args = ESP_LOADER_CONNECT_DEFAULT();
    esp_loader_error_t err = esp_loader_connect(&args);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.printf("OmniUartFlasher: esp_loader_connect failed (%d)\n", (int)err);
        result.errorCode = (int32_t)err;
        result.chipName = "connect failed";
        result.durationMs = millis() - t0;
        releaseStrapPins();
        recordAction(FlasherAction::Probe);
        return result;
    }

    target_chip_t target = esp_loader_get_target();
    Serial.printf("OmniUartFlasher: connected, target=%d\n", (int)target);
    result.ok        = true;
    result.errorCode = 0;
    result.chipName  = targetName(target);
    result.durationMs = millis() - t0;

    releaseStrapPins();
    recordAction(FlasherAction::Probe);
    return result;
}

bool OmniUartFlasher::flashBegin(uint32_t imageSize, uint32_t flashOffset) {
    if (!_began || !_portReady || _flashActive) {
        _lastFlashError = -100;
        _lastFlashOk = false;
        return false;
    }

    Serial.printf("OmniUartFlasher: flashBegin size=%u offset=0x%X\n",
                  (unsigned)imageSize, (unsigned)flashOffset);

    _flashActive     = true;
    _lastFlashOk     = false;
    _lastFlashError  = 0;
    _lastFlashSize   = imageSize;
    _lastFlashOffset = flashOffset;
    _flashStartMs    = millis();
    _lastFlashMs     = 0;
    recordAction(FlasherAction::Flashing);

    primeStrapPinsForFlash();

    esp_loader_connect_args_t args = ESP_LOADER_CONNECT_DEFAULT();
    esp_loader_error_t err = esp_loader_connect_with_stub(&args);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.printf("OmniUartFlasher: flashBegin connect_with_stub failed (%d)\n", (int)err);
        _lastFlashError = (int32_t)err;
        _flashActive = false;
        releaseStrapPins();
        return false;
    }
    Serial.println("OmniUartFlasher: stub uploaded");

    // We tested change_transmission_rate_stub(115200 -> 460800) and the call
    // succeeded on the S3 side, but subsequent commands (flash_size detect,
    // flash_start) consistently timed out at the new rate. Most likely the
    // stub's CHANGE_BAUDRATE handler races with our follow-up commands or
    // the bodge-wire signal integrity isn't clean enough at 460800. Either
    // way the failure mode is "S3 thinks it bumped, C6 didn't" which leaves
    // both sides at mismatched rates and bricks the rest of the session.
    //
    // Sticking with 115200 — ~37 s for a 411 KB image. Reliable beats fast.
    // Revisit if/when we move from a bodge to a real PCB layout, or if
    // esp-serial-flasher gets a more robust baud-change path.
    Serial.printf("OmniUartFlasher: staying at %lu baud (stub baud-change is unreliable on this hat)\n",
                  (unsigned long)kFlasherBaudRate);

    err = esp_loader_flash_start(flashOffset, imageSize, kFlasherBlockSize);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.printf("OmniUartFlasher: flash_start failed (%d)\n", (int)err);
        _lastFlashError = (int32_t)err;
        _flashActive = false;
        releaseStrapPins();
        return false;
    }

    Serial.println("OmniUartFlasher: flash_start OK, ready to stream chunks");
    return true;
}

bool OmniUartFlasher::flashWrite(const uint8_t* data, uint32_t len) {
    if (!_flashActive) return false;

    esp_loader_error_t err = esp_loader_flash_write((void*)data, len);
    if (err != ESP_LOADER_SUCCESS) {
        Serial.printf("OmniUartFlasher: flash_write(%u) failed (%d)\n", (unsigned)len, (int)err);
        _lastFlashError = (int32_t)err;
        return false;
    }
    return true;
}

bool OmniUartFlasher::flashFinish(bool reboot) {
    if (!_flashActive) return false;

    // Verify before finishing. esp-serial-flasher accumulated an MD5 of every
    // byte we wrote during flashWrite; this asks the C6 stub to compute the
    // same hash on what's actually in flash and compare. Catches bytes
    // corrupted in transit on the UART (single-bit errors etc.) — which
    // would otherwise leave the C6 in a boot loop with a corrupt image.
    esp_loader_error_t verr = esp_loader_flash_verify();
    if (verr != ESP_LOADER_SUCCESS) {
        Serial.printf("OmniUartFlasher: flash_verify failed (%d) — image corrupted in transit\n",
                      (int)verr);
        _lastFlashMs = millis() - _flashStartMs;
        _lastFlashError = (int32_t)verr;
        _lastFlashOk = false;
        _flashActive = false;
        releaseStrapPins();
        recordAction(FlasherAction::None);
        return false;
    }

    esp_loader_error_t err = esp_loader_flash_finish(reboot);
    _lastFlashMs = millis() - _flashStartMs;
    _flashActive = false;

    // Don't deinit the UART driver — it's persistent. Just hand the strap
    // pins back to INPUT_PULLUP so HANDSHAKE-from-C6 (M-γ) isn't blocked.
    releaseStrapPins();
    recordAction(FlasherAction::None);

    if (err != ESP_LOADER_SUCCESS) {
        Serial.printf("OmniUartFlasher: flash_finish failed (%d) after %lu ms\n",
                      (int)err, (unsigned long)_lastFlashMs);
        _lastFlashError = (int32_t)err;
        _lastFlashOk = false;
        return false;
    }

    Serial.printf("OmniUartFlasher: flash done + verified in %lu ms\n",
                  (unsigned long)_lastFlashMs);
    _lastFlashOk = true;
    return true;
}

void OmniUartFlasher::flashAbort() {
    if (!_flashActive) return;
    Serial.println("OmniUartFlasher: flashAbort");
    _lastFlashMs = millis() - _flashStartMs;
    _lastFlashOk = false;
    _flashActive = false;
    releaseStrapPins();
    recordAction(FlasherAction::None);
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
