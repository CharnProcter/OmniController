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
}

void OmniUartFlasher::releaseStrapPins() {
    // Idle state: both pins floating high via internal pull-up. The hat's
    // external pull-up keeps the C6 running normally; leaving the S3 pins as
    // INPUT_PULLUP avoids fighting the hat and lets the BOOT line later
    // be driven low *by the C6* as HANDSHAKE in M-γ.
    pinMode(_enPin, INPUT_PULLUP);
    pinMode(_bootPin, INPUT_PULLUP);
}

ProbeResult OmniUartFlasher::probeC6Target() {
    ProbeResult result{false, -1, "unknown", 0};
    if (!_began) {
        result.chipName = "flasher not begun";
        return result;
    }

    uint32_t t0 = millis();

    loader_esp32_config_t cfg{};
    cfg.baud_rate          = kFlasherBaudRate;
    cfg.uart_port          = kFlasherUartPort;
    cfg.uart_rx_pin        = _uartRxPin;
    cfg.uart_tx_pin        = _uartTxPin;
    cfg.reset_trigger_pin  = _enPin;
    cfg.gpio0_trigger_pin  = _bootPin;

    esp_loader_error_t err = loader_port_esp32_init(&cfg);
    if (err != ESP_LOADER_SUCCESS) {
        result.errorCode = (int32_t)err;
        result.chipName = "port_init failed";
        result.durationMs = millis() - t0;
        loader_port_esp32_deinit();
        releaseStrapPins();
        recordAction(FlasherAction::Probe);
        return result;
    }

    esp_loader_connect_args_t args = ESP_LOADER_CONNECT_DEFAULT();
    err = esp_loader_connect(&args);
    if (err != ESP_LOADER_SUCCESS) {
        result.errorCode = (int32_t)err;
        result.chipName = "connect failed";
        result.durationMs = millis() - t0;
        loader_port_esp32_deinit();
        releaseStrapPins();
        recordAction(FlasherAction::Probe);
        return result;
    }

    target_chip_t target = esp_loader_get_target();
    result.ok        = true;
    result.errorCode = 0;
    result.chipName  = targetName(target);
    result.durationMs = millis() - t0;

    loader_port_esp32_deinit();
    releaseStrapPins();
    recordAction(FlasherAction::Probe);
    return result;
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
