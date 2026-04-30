#pragma once

#include <Arduino.h>
#include <functional>
#include <vector>

#include "FlexibleEndpoints.h"

#include "../src/flasher/OmniUartFlasher.h"
#include "../src/transport/OmniSpiMaster.h"

// Hardware contract for the C6 hat. Defaults match the AutomationPlatformPlus
// board (ESP32-S3 with the OmniController hat); host projects on different
// boards override by passing a custom OmniPins to begin(). Defaults exist so
// getUsedPins() returns a sensible list even before begin() runs — used by the
// pin-conflict registry to keep these pins reserved while the module is disabled.
//
// boot is dual-purpose: output during UART bootstrap flash (BOOT strap), and
// input-with-IRQ during normal SPI ops (HANDSHAKE asserted by C6 pulling low).
struct OmniPins {
    uint8_t en       = 13;
    uint8_t boot     = 12;
    uint8_t spi_cs   = 39;
    uint8_t spi_mosi = 40;
    uint8_t spi_miso = 42;
    uint8_t spi_clk  = 41;
    uint8_t uart_tx  = 43;
    uint8_t uart_rx  = 44;
};

class OmniController {
public:
    bool begin(FlexibleEndpoints* endpoints, const OmniPins& pins);

    std::vector<uint8_t> getUsedPins() const;

    bool began() const { return _began; }

    omni::OmniUartFlasher& flasher()    { return _flasher; }
    omni::OmniSpiMaster&   spiMaster()  { return _spiMaster; }

    // Handle one line of input from the host's USB-CDC Serial. Returns true
    // if the line was an OmniController serial-flash command (and was fully
    // consumed, including any subsequent binary stream). Returns false if
    // the line should fall through to the existing FlexibleEndpoints
    // command dispatch. Blocks until the receive + flash sequence finishes
    // when handling OMNI_C6_FLASH (~30 s for a typical image).
    bool handleSerialCommand(const String& line);

private:
    void registerEndpoints(FlexibleEndpoints* endpoints);
    bool handleSerialFlashCommand(const String& line);

    // Run `body` with the SPI link task suspended — stops the task, runs body
    // (which is free to drive GPIO12 / reset / bootloader / flash), then
    // restarts the task. Used by all flasher endpoints because the UART
    // flasher repurposes BOOT (= GPIO12 = HANDSHAKE) as an output, which
    // would otherwise fight the master task's IRQ wiring.
    void withSpiSuspended(std::function<void()> body);

    bool _began = false;
    OmniPins _pins{};
    omni::OmniUartFlasher _flasher;
    omni::OmniSpiMaster   _spiMaster;
};
