#pragma once

#include <Arduino.h>
#include <functional>

#include "OmniProto.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace omni {

// OmniSpiMaster — S3 side of the inter-MCU SPI link.
//
// Two operating modes:
//   1. Synchronous diag: call begin() then transact() directly. Used by the
//      M-γ Push A echo test endpoint. transact() refuses if the master task
//      is running.
//   2. Async transport (Push B onward): call begin() then start(). A worker
//      task owns the device handle, waits on a HANDSHAKE-driven semaphore
//      (or a 1 s idle timer), clocks one transaction per wake, decodes the
//      RX side, and dispatches valid frames to the registered FrameHandler.
//      Outbound frames go through sendFrame(), which encodes once, posts to
//      a queue, and signals the same semaphore.
class OmniSpiMaster {
public:
    using FrameHandler = std::function<void(Channel channel, uint8_t flags,
                                            uint16_t seq, const uint8_t* payload,
                                            uint16_t payloadLen)>;

    struct LinkStats {
        uint32_t tx_frames    = 0;
        uint32_t rx_frames    = 0;  // valid frames only
        uint32_t bad_magic    = 0;
        uint32_t bad_crc      = 0;
        uint32_t tx_dropped   = 0;  // sendFrame() called when queue full
        uint32_t handshake_irqs = 0;
        uint32_t last_rx_ms   = 0;  // millis() at last valid RX, 0 if never
        uint32_t last_tx_ms   = 0;
    };

    bool begin(uint8_t cs, uint8_t mosi, uint8_t miso, uint8_t clk, uint8_t handshake);

    // Spawn the master task and attach the HANDSHAKE IRQ. Idempotent.
    bool start();

    // Stop the master task and detach the IRQ. The bus stays initialised so
    // start() can be called again. Used by the UART flasher to reclaim
    // GPIO12 during a reflash.
    void stop();

    // Synchronous full-duplex transaction. Refuses if the task is running.
    // tx and rx must each be at least kSpiTransactionBytes long.
    bool transact(const uint8_t* tx, uint8_t* rx);

    // Enqueue a frame for the master task to send on the next transaction.
    // Returns false if the task isn't running, the payload is too large, or
    // the queue is full. Encodes immediately into a queue slot — the caller
    // can free `payload` as soon as this returns.
    bool sendFrame(Channel channel, uint8_t flags, uint16_t seq,
                   const uint8_t* payload, uint16_t payloadLen);

    // Register a callback for inbound frames. Called from the master task
    // context for every frame that decodes cleanly. Set once at startup;
    // not safe to swap while the task is running.
    void setFrameHandler(FrameHandler handler) { _handler = handler; }

    bool       began()       const { return _began; }
    bool       taskRunning() const { return _taskRunning; }
    LinkStats  stats()       const;

private:
    static void   taskTrampoline(void* arg);
    void          taskLoop();
    bool          clockTransaction(const uint8_t* tx, uint8_t* rx);
    static void IRAM_ATTR handshakeIsr(void* arg);

    bool     _began = false;
    uint8_t  _cs = 0;
    uint8_t  _mosi = 0;
    uint8_t  _miso = 0;
    uint8_t  _clk = 0;
    uint8_t  _hs = 0;
    void*    _device = nullptr;  // spi_device_handle_t opaque

    volatile bool   _taskRunning = false;
    volatile bool   _taskShouldStop = false;
    TaskHandle_t    _taskHandle = nullptr;
    SemaphoreHandle_t _wakeSem  = nullptr;  // signaled by IRQ + sendFrame
    QueueHandle_t   _txQueue    = nullptr;  // holds pre-encoded transactions
    FrameHandler    _handler;

    mutable SemaphoreHandle_t _statsMutex = nullptr;
    LinkStats _stats{};
};

}  // namespace omni
