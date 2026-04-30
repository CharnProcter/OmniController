#pragma once

#include <Arduino.h>
#include <functional>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "FlexibleEndpoints.h"
#include "OmniProto.h"

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

    // ── Streaming flash session API ────────────────────────────────────────
    //
    // Used by both the USB-CDC OMNI_C6_FLASH path and the HTTP /omniC6Ota
    // multipart upload path. The producer (HTTP upload callback or USB-CDC
    // reader) calls feedFlashStream() to push bytes; a dedicated worker task
    // on Core 1 drains them through OmniUartFlasher::flashWrite. Isolating
    // esp-serial-flasher's blocking UART work from AsyncTCP's task is what
    // makes the HTTP path safe — running flashWrite on AsyncTCP previously
    // produced heap corruption in the WiFi RX path (project memory:
    // project_http_c6_ota_deferred.md).
    //
    // Lifecycle: startFlashStream → repeated feedFlashStream → worker
    // detects `received == size` and finalises automatically.
    bool startFlashStream(uint32_t imageSize, uint32_t flashOffset,
                          uint32_t expectedCrc);
    size_t feedFlashStream(const uint8_t* data, size_t len, uint32_t timeoutMs);
    void abortFlashStream();
    bool flashStreamActive() const { return _flashStreamActive; }
    bool flashStreamLastOk() const { return _flashStreamOk; }
    int32_t flashStreamLastError() const { return _flashStreamErrorCode; }
    const char* flashStreamLastErrorMsg() const { return _flashStreamErrorMsg ? _flashStreamErrorMsg : ""; }
    uint32_t flashStreamBytesProcessed() const { return _flashStreamBytesProcessed; }
    uint32_t flashStreamDurationMs() const { return _flashStreamDurationMs; }
    uint32_t flashStreamRunningCrc() const { return _flashStreamRunningCrc; }

private:
    struct LinkState {
        bool     hello_acked      = false;
        uint32_t hello_acked_ms   = 0;
        int32_t  c6_proto         = -1;
        char     c6_fw[48]        = {};
        uint32_t hello_attempts   = 0;
        uint32_t last_pong_ms     = 0;
        uint32_t last_pong_rtt_ms = 0;
        uint32_t last_log_ms      = 0;
        uint32_t log_lines        = 0;
    };

    void registerEndpoints(FlexibleEndpoints* endpoints);
    bool handleSerialFlashCommand(const String& line);

    // Run `body` with the SPI link task suspended — stops the task, runs body
    // (which is free to drive GPIO12 / reset / bootloader / flash), then
    // restarts the task. Used by all flasher endpoints because the UART
    // flasher repurposes BOOT (= GPIO12 = HANDSHAKE) as an output, which
    // would otherwise fight the master task's IRQ wiring.
    void withSpiSuspended(std::function<void()> body);

    // Link-management plumbing (Push C). The frame handler runs on the SPI
    // master task; the pump task drives hello/ping cadence; everything that
    // mutates _link goes through _linkMutex.
    void onLinkFrame(omni::Channel channel, uint8_t flags, uint16_t seq,
                     const uint8_t* payload, uint16_t payloadLen);
    void handleCtrlFrame(uint8_t flags, uint16_t seq,
                         const uint8_t* payload, uint16_t payloadLen);
    void handleLogFrame(const uint8_t* payload, uint16_t payloadLen);
    void sendHello();
    void sendPing();
    static void ctrlPumpTrampoline(void* arg);
    void ctrlPumpLoop();

    bool _began = false;
    OmniPins _pins{};
    omni::OmniUartFlasher _flasher;
    omni::OmniSpiMaster   _spiMaster;

    LinkState         _link{};
    SemaphoreHandle_t _linkMutex   = nullptr;
    TaskHandle_t      _ctrlPumpTask = nullptr;
    uint16_t          _ctrlTxSeq    = 1;

    // Flash-stream session state (M-β.3 OTA refactor). The worker task
    // owns the active flash session; producers feed bytes through the
    // stream buffer. All volatile fields are read/written from both the
    // producer and worker tasks.
    static void flashWorkerTrampoline(void* arg);
    void flashWorkerLoop();
    void finishFlashWorker(bool ok, int32_t errorCode, const char* msg);

    StreamBufferHandle_t _flashStream     = nullptr;
    TaskHandle_t         _flashWorkerTask = nullptr;
    SemaphoreHandle_t    _flashWorkerDone = nullptr;
    volatile bool        _flashStreamActive   = false;
    volatile bool        _flashStreamAborted  = false;
    bool                 _flashStreamLinkWasRunning = false;
    uint32_t             _flashStreamSize     = 0;
    uint32_t             _flashStreamOffset   = 0;
    uint32_t             _flashStreamExpectedCrc = 0;
    uint32_t             _flashStreamStartMs  = 0;
    bool                 _flashStreamOk       = false;
    int32_t              _flashStreamErrorCode = 0;
    const char*          _flashStreamErrorMsg  = nullptr;
    uint32_t             _flashStreamBytesProcessed = 0;
    uint32_t             _flashStreamDurationMs = 0;
    uint32_t             _flashStreamRunningCrc = 0;
};
