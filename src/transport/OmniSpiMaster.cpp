#include "OmniSpiMaster.h"

#include "driver/spi_master.h"

namespace omni {

namespace {
constexpr spi_host_device_t kHost     = SPI3_HOST;
constexpr int               kClockHz  = 20 * 1000 * 1000;  // 20 MHz; raise after stability proven
constexpr uint32_t          kIdlePeriodMs = 1000;          // wake at least once per second
// Minimum gap between transactions, even if HANDSHAKE keeps firing the IRQ.
// Caps the link at ~50 Hz, which is plenty for heartbeats + log forwarding,
// and prevents the bus from saturating when a chatty channel keeps the C6's
// HANDSHAKE asserted continuously (e.g. Push A echo mode where every CTRL
// keepalive produces an echo response that re-arms HANDSHAKE).
constexpr uint32_t          kMinTxGapMs   = 20;
constexpr uint8_t           kTxQueueDepth = 8;
constexpr uint32_t          kTaskStackBytes = 4096;
constexpr UBaseType_t       kTaskPriority   = 5;
constexpr BaseType_t        kTaskCore       = 1;            // pin to core 1, away from WiFi/AsyncTCP

// Each slot in the TX queue is one fully-encoded fixed-size transaction.
struct TxSlot {
    uint8_t bytes[kSpiTransactionBytes];
};

// Fallback idle frame — sent when the task wakes with nothing queued. Just a
// CTRL ping with empty payload; the C6 will echo it back. Encoded once at
// start() and reused.
constexpr uint16_t kIdleSeqBase = 0x8000;  // distinguishable from real frame seqs
}  // namespace

bool OmniSpiMaster::begin(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t clk, uint8_t handshake) {
    if (_began) return true;

    _cs   = cs;
    _mosi = mosi;
    _miso = miso;
    _clk  = clk;
    _hs   = handshake;

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

    // HANDSHAKE pin: input with pull-up. The C6 firmware drives it low when
    // it has data ready. The IRQ is attached in start(); here we just put the
    // pin in a sensible idle state.
    pinMode(_hs, INPUT_PULLUP);

    _statsMutex = xSemaphoreCreateMutex();

    _began = true;
    Serial.printf(
        "OmniSpiMaster: SPI3 up @ %d MHz (CLK=%u MOSI=%u MISO=%u CS=%u HANDSHAKE=%u)\n",
        kClockHz / 1000000, clk, mosi, miso, cs, handshake);
    return true;
}

bool OmniSpiMaster::start() {
    if (!_began) return false;
    if (_taskRunning) return true;

    if (_wakeSem == nullptr) _wakeSem = xSemaphoreCreateBinary();
    if (_txQueue == nullptr) _txQueue = xQueueCreate(kTxQueueDepth, sizeof(TxSlot));
    if (_wakeSem == nullptr || _txQueue == nullptr) {
        Serial.println("OmniSpiMaster: failed to allocate task primitives");
        return false;
    }
    // Drain any leftover frames from the previous run (e.g. queued just
    // before a reflash) so the freshly-armed link doesn't immediately send
    // stale traffic to the C6.
    xQueueReset(_txQueue);

    _taskShouldStop = false;
    BaseType_t ok = xTaskCreatePinnedToCore(
        &OmniSpiMaster::taskTrampoline,
        "OmniSpi",
        kTaskStackBytes,
        this,
        kTaskPriority,
        &_taskHandle,
        kTaskCore);
    if (ok != pdPASS) {
        Serial.println("OmniSpiMaster: task create failed");
        return false;
    }

    // Attach the HANDSHAKE IRQ AFTER the task is up so any spurious early
    // edge has somewhere to deliver its semaphore signal.
    attachInterruptArg(_hs, &OmniSpiMaster::handshakeIsr, this, FALLING);

    _taskRunning = true;
    Serial.println("OmniSpiMaster: link task started, HANDSHAKE IRQ armed");
    return true;
}

void OmniSpiMaster::stop() {
    if (!_taskRunning) return;
    detachInterrupt(_hs);
    _taskShouldStop = true;
    if (_wakeSem) xSemaphoreGive(_wakeSem);  // unblock the task so it can exit

    // Wait briefly for the task to exit on its own. If it doesn't, force it.
    for (int i = 0; i < 50 && _taskRunning; i++) vTaskDelay(pdMS_TO_TICKS(10));
    if (_taskRunning && _taskHandle) {
        vTaskDelete(_taskHandle);
        _taskRunning = false;
        _taskHandle  = nullptr;
    }
    Serial.println("OmniSpiMaster: link task stopped, HANDSHAKE IRQ released");
}

bool OmniSpiMaster::transact(const uint8_t* tx, uint8_t* rx) {
    if (_taskRunning) {
        Serial.println("OmniSpiMaster: transact() refused — link task owns the bus");
        return false;
    }
    return clockTransaction(tx, rx);
}

bool OmniSpiMaster::clockTransaction(const uint8_t* tx, uint8_t* rx) {
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

bool OmniSpiMaster::sendFrame(Channel channel, uint8_t flags, uint16_t seq,
                              const uint8_t* payload, uint16_t payloadLen) {
    if (!_taskRunning) return false;
    if (payloadLen > kMaxPayload) return false;

    TxSlot slot{};
    size_t encoded = encodeFrame(slot.bytes, sizeof(slot.bytes),
                                 channel, flags, seq, payload, payloadLen);
    if (encoded == 0) return false;

    // Non-blocking post — drop the frame if the queue is full rather than
    // stalling the caller's task. Stats track the drop so it's visible.
    if (xQueueSend(_txQueue, &slot, 0) != pdTRUE) {
        if (_statsMutex && xSemaphoreTake(_statsMutex, portMAX_DELAY) == pdTRUE) {
            _stats.tx_dropped++;
            xSemaphoreGive(_statsMutex);
        }
        return false;
    }
    if (_wakeSem) xSemaphoreGive(_wakeSem);
    return true;
}

OmniSpiMaster::LinkStats OmniSpiMaster::stats() const {
    LinkStats out{};
    if (_statsMutex && xSemaphoreTake(_statsMutex, portMAX_DELAY) == pdTRUE) {
        out = _stats;
        xSemaphoreGive(_statsMutex);
    }
    return out;
}

void OmniSpiMaster::taskTrampoline(void* arg) {
    static_cast<OmniSpiMaster*>(arg)->taskLoop();
    vTaskDelete(nullptr);
}

void OmniSpiMaster::taskLoop() {
    // Pre-encode the idle/keepalive frame once. Sent whenever the task wakes
    // with nothing pending in the TX queue — keeps the C6 talking back so
    // the link liveness detection has data to chew on.
    TxSlot idleFrame{};
    uint16_t idleSeq = kIdleSeqBase;
    encodeFrame(idleFrame.bytes, sizeof(idleFrame.bytes),
                Channel::Ctrl, 0, idleSeq, nullptr, 0);

    static TxSlot rx{};   // RX scratch (1040 B; static to keep stack small)
    static TxSlot tx{};
    uint32_t lastTxMs = 0;

    while (!_taskShouldStop) {
        // Enforce the minimum gap between transactions. If the previous
        // transaction was less than kMinTxGapMs ago, sleep the remainder
        // before checking the wake semaphore — otherwise a HANDSHAKE that
        // re-asserts every transaction would let us pump the bus at full
        // hardware speed.
        uint32_t now = millis();
        if (lastTxMs != 0 && now - lastTxMs < kMinTxGapMs) {
            vTaskDelay(pdMS_TO_TICKS(kMinTxGapMs - (now - lastTxMs)));
        }

        // Wait for: HANDSHAKE IRQ, sendFrame post, or the 1 s idle timer.
        xSemaphoreTake(_wakeSem, pdMS_TO_TICKS(kIdlePeriodMs));
        if (_taskShouldStop) break;

        // Pick the next outbound payload: queued frame, else idle keepalive.
        bool fromQueue = (xQueueReceive(_txQueue, &tx, 0) == pdTRUE);
        if (!fromQueue) {
            // Refresh the idle seq each tick so we can tell consecutive
            // idle frames apart in traces. No CRC update needed because we
            // never check CRCs of frames we just authored locally.
            idleSeq++;
            idleFrame.bytes[5] = static_cast<uint8_t>(idleSeq & 0xFF);
            idleFrame.bytes[6] = static_cast<uint8_t>((idleSeq >> 8) & 0xFF);
            // Recompute CRC over the (single) seq-byte change — easier to
            // just re-encode once per tick.
            encodeFrame(idleFrame.bytes, sizeof(idleFrame.bytes),
                        Channel::Ctrl, 0, idleSeq, nullptr, 0);
            memcpy(tx.bytes, idleFrame.bytes, sizeof(tx.bytes));
        }

        if (!clockTransaction(tx.bytes, rx.bytes)) {
            // Bus error — log once, brief sleep, retry on next wake.
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        const uint32_t txMs = millis();
        lastTxMs = txMs;
        if (_statsMutex && xSemaphoreTake(_statsMutex, 0) == pdTRUE) {
            _stats.tx_frames++;
            _stats.last_tx_ms = txMs;
            xSemaphoreGive(_statsMutex);
        }

        // Decode RX. Anything that doesn't have MAGIC is silently ignored —
        // the C6 may have clocked back 0xFF padding if it had no response
        // queued. Frames with bad CRC are counted but not dispatched.
        DecodedFrame f{};
        if (rx.bytes[0] != kFrameMagic) {
            if (_statsMutex && xSemaphoreTake(_statsMutex, 0) == pdTRUE) {
                _stats.bad_magic++;
                xSemaphoreGive(_statsMutex);
            }
            continue;
        }
        if (!decodeFrame(rx.bytes, kSpiTransactionBytes, f)) {
            if (_statsMutex && xSemaphoreTake(_statsMutex, 0) == pdTRUE) {
                _stats.bad_crc++;
                xSemaphoreGive(_statsMutex);
            }
            continue;
        }

        if (_statsMutex && xSemaphoreTake(_statsMutex, 0) == pdTRUE) {
            _stats.rx_frames++;
            _stats.last_rx_ms = txMs;
            xSemaphoreGive(_statsMutex);
        }

        if (_handler) _handler(f.channel, f.flags, f.seq, f.payload, f.payloadLen);
    }

    _taskRunning = false;
    _taskHandle  = nullptr;
}

void IRAM_ATTR OmniSpiMaster::handshakeIsr(void* arg) {
    auto* self = static_cast<OmniSpiMaster*>(arg);
    if (self == nullptr || self->_wakeSem == nullptr) return;
    BaseType_t hpw = pdFALSE;
    self->_stats.handshake_irqs++;  // racy but cheap; just for diag
    xSemaphoreGiveFromISR(self->_wakeSem, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

}  // namespace omni
