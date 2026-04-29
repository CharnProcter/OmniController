#pragma once

#include <Arduino.h>

namespace omni {

// OmniSpiMaster — S3 side of the inter-MCU SPI link.
//
// Push-A scope: synchronous transact() that clocks one fixed-size transaction.
// Both directions transferred per cycle (full duplex). The HANDSHAKE pin
// (S3 GPIO12, dual-purpose with BOOT) is configured INPUT_PULLUP for now;
// future pushes wire a falling-edge IRQ to drive the master task.
class OmniSpiMaster {
public:
    bool begin(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t clk, uint8_t handshake);

    // Synchronous full-duplex transaction. tx and rx must each be at least
    // omni::kSpiTransactionBytes long. Caller fills tx; rx is populated with
    // whatever the slave clocked back. Returns false if the bus isn't
    // initialised or the transaction failed.
    bool transact(const uint8_t* tx, uint8_t* rx);

    bool began() const { return _began; }

private:
    bool     _began = false;
    uint8_t  _cs = 0;
    uint8_t  _mosi = 0;
    uint8_t  _miso = 0;
    uint8_t  _clk = 0;
    uint8_t  _hs = 0;
    void*    _device = nullptr;  // spi_device_handle_t opaque
};

}  // namespace omni
