#include "OmniSpiMaster.h"
#include "OmniProto.h"

#include "driver/spi_master.h"

namespace omni {

namespace {
constexpr spi_host_device_t kHost = SPI3_HOST;
constexpr int kClockHz = 20 * 1000 * 1000;  // 20 MHz; raise after stability proven
}  // namespace

bool OmniSpiMaster::begin(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t clk, uint8_t handshake) {
    if (_began) return true;

    _cs = cs;
    _mosi = mosi;
    _miso = miso;
    _clk = clk;
    _hs = handshake;

    spi_bus_config_t bus{};
    bus.mosi_io_num     = mosi;
    bus.miso_io_num     = miso;
    bus.sclk_io_num     = clk;
    bus.quadwp_io_num   = -1;
    bus.quadhd_io_num   = -1;
    bus.max_transfer_sz = kSpiTransactionBytes;
    bus.flags           = 0;

    esp_err_t err = spi_bus_initialize(kHost, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means "already initialised" — that's fine.
        Serial.printf("OmniSpiMaster: bus init failed (%d)\n", (int)err);
        return false;
    }

    spi_device_interface_config_t dev{};
    dev.clock_speed_hz = kClockHz;
    dev.mode           = 0;  // CPOL=0, CPHA=0
    dev.spics_io_num   = cs;
    dev.queue_size     = 2;
    dev.flags          = 0;

    spi_device_handle_t handle = nullptr;
    err = spi_bus_add_device(kHost, &dev, &handle);
    if (err != ESP_OK) {
        Serial.printf("OmniSpiMaster: spi_bus_add_device failed (%d)\n", (int)err);
        return false;
    }
    _device = handle;

    // HANDSHAKE pin: input with pull-up. Bodge wire ties this through to the
    // C6's GPIO9, which the C6 firmware drives low when it has data ready.
    // Push-A doesn't actually wire the IRQ yet — caller initiates each
    // transaction synchronously. IRQ wiring lands with the link bring-up
    // in the next push.
    pinMode(_hs, INPUT_PULLUP);

    _began = true;
    Serial.printf(
        "OmniSpiMaster: SPI3 up @ %d MHz (CLK=%u MOSI=%u MISO=%u CS=%u HANDSHAKE=%u)\n",
        kClockHz / 1000000, clk, mosi, miso, cs, handshake);
    return true;
}

bool OmniSpiMaster::transact(const uint8_t* tx, uint8_t* rx) {
    if (!_began || _device == nullptr) return false;

    spi_transaction_t t{};
    t.length    = static_cast<size_t>(kSpiTransactionBytes) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_polling_transmit(
        static_cast<spi_device_handle_t>(_device), &t);
    if (err != ESP_OK) {
        Serial.printf("OmniSpiMaster: transact failed (%d)\n", (int)err);
        return false;
    }
    return true;
}

}  // namespace omni
