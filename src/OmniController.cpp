#include "OmniController.h"

#include <ArduinoJson.h>
#include <esp_rom_crc.h>
#include "driver/uart.h"

#include "OmniProto.h"

#ifndef OMNI_S3_FW_VERSION
#define OMNI_S3_FW_VERSION "omni-0.1.0"
#endif

namespace {
// SPIFFS staging path. The HTTP upload callback (or USB-CDC reader)
// writes the inbound C6 image here as it arrives — same as the S3's
// own /updateOTA pattern, just to a flat file instead of an OTA
// partition. After the upload completes the worker task reads it back
// and feeds it to the C6 over UART. AsyncTCP only blocks per chunk for
// the SPIFFS write (~15-30 ms), well under any heap-pressure threshold.
constexpr const char* kFlashStagePath        = "/omni_c6_stage.bin";
constexpr uint32_t    kFlashWorkerStackBytes = 8192;
constexpr UBaseType_t kFlashWorkerPriority   = 5;
constexpr BaseType_t  kFlashWorkerCore       = 1;  // away from AsyncTCP on Core 0
}  // namespace

bool OmniController::begin(FlexibleEndpoints* endpoints, const OmniPins& pins) {
    if (_began) return true;

    _pins = pins;
    _flasher.begin(_pins.en, _pins.boot, _pins.uart_tx, _pins.uart_rx);
    _spiMaster.begin(_pins.spi_cs, _pins.spi_mosi, _pins.spi_miso, _pins.spi_clk, _pins.boot);

    // Push C: link state + frame dispatch. The handler is registered BEFORE
    // start() so the very first inbound frame finds a wired-up callback.
    _linkMutex = xSemaphoreCreateMutex();
    OmniController* selfPtr = this;
    _spiMaster.setFrameHandler([selfPtr](omni::Channel ch, uint8_t flags,
                                         uint16_t seq, const uint8_t* payload,
                                         uint16_t len) {
        selfPtr->onLinkFrame(ch, flags, seq, payload, len);
    });

    // Push B: launch the async link task. It owns the SPI device from here.
    _spiMaster.start();

    // Push C: pump task drives the hello/ping cadence so the link bring-up
    // doesn't depend on someone hitting an endpoint.
    xTaskCreatePinnedToCore(&OmniController::ctrlPumpTrampoline,
                            "OmniCtrlPump", 4096, this, 4, &_ctrlPumpTask, 1);

    // M-β.3: flash-stream worker uses SPIFFS staging via setFs(). The
    // completion semaphore is created here; the actual staging file is
    // opened/closed per session in startFlashStream / worker.
    _flashWorkerDone = xSemaphoreCreateBinary();
    if (_fs == nullptr) {
        Serial.println("OmniController: no FS attached — /omniC6Ota disabled (call setFs() before begin)");
    } else if (_fs->exists(kFlashStagePath)) {
        // Clean up any stale staging file from a previous boot (e.g. a
        // session that was interrupted by a reset).
        _fs->remove(kFlashStagePath);
    }

    if (endpoints) {
        registerEndpoints(endpoints);
    }

    _began = true;
    Serial.println("OmniController: initialized (M-gamma.C)");
    return true;
}

std::vector<uint8_t> OmniController::getUsedPins() const {
    return {
        _pins.en,
        _pins.boot,
        _pins.spi_cs,
        _pins.spi_mosi,
        _pins.spi_miso,
        _pins.spi_clk,
        _pins.uart_tx,
        _pins.uart_rx,
    };
}

void OmniController::withSpiSuspended(std::function<void()> body) {
    bool wasRunning = _spiMaster.taskRunning();
    if (wasRunning) _spiMaster.stop();
    body();
    if (wasRunning) _spiMaster.start();
}

void OmniController::registerEndpoints(FlexibleEndpoints* endpoints) {
    OmniController* self = this;

    endpoints->setTagGroup("Omni");
    endpoints->setLibraryName("Status");

    auto statusEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omniStatus")
        .summary("OmniController status")
        .description("Returns current OmniController state: firmware version, wire protocol "
                     "version, registered drivers, and module readiness. Used by the "
                     "dashboard to detect whether the Omni tab should be shown.")
        .params({})
        .responseType(JSON_RESPONSE)
        .responseDescription("JSON status object")
        .handler([self](std::map<String, String>& /*params*/) -> std::pair<String, int> {
            JsonDocument doc;
            doc["fw"] = OMNI_S3_FW_VERSION;
            doc["proto"] = OMNI_PROTO_VERSION;
            doc["began"] = self->_began;
            doc["milestone"] = "M-gamma.C";
            JsonArray drivers = doc["drivers"].to<JsonArray>();
            (void)drivers;  // populated as drivers register in M-zeta+
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });
    endpoints->addEndpoint(statusEndpoint);

    endpoints->setLibraryName("C6 Link");

    auto c6StatusEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omniC6Status")
        .summary("C6 daughterboard link status")
        .description("Returns current C6 link state: pin assignments, last manual action "
                     "(reset / bootloader / flash) with timestamp, and last flash result "
                     "(size, duration, error code). M-gamma adds SPI link health; "
                     "M-delta adds OTA history.")
        .params({})
        .responseType(JSON_RESPONSE)
        .responseDescription("JSON link status")
        .handler([self](std::map<String, String>& /*params*/) -> std::pair<String, int> {
            JsonDocument doc;
            doc["began"] = self->_flasher.began();
            doc["en_pin"] = self->_flasher.enPin();
            doc["boot_pin"] = self->_flasher.bootPin();
            doc["uart_tx_pin"] = self->_pins.uart_tx;
            doc["uart_rx_pin"] = self->_pins.uart_rx;
            doc["last_action"] = omni::flasherActionName(self->_flasher.lastAction());
            uint32_t ms = self->_flasher.lastActionMs();
            if (ms == 0) {
                doc["last_action_ms_ago"] = nullptr;
            } else {
                doc["last_action_ms_ago"] = (uint32_t)(millis() - ms);
            }
            JsonObject flash = doc["flash"].to<JsonObject>();
            flash["active"]      = self->_flasher.flashActive();
            flash["last_ok"]     = self->_flasher.lastFlashOk();
            flash["last_error"]  = self->_flasher.lastFlashError();
            flash["last_size"]   = self->_flasher.lastFlashSize();
            flash["last_offset"] = self->_flasher.lastFlashOffset();
            flash["last_ms"]     = self->_flasher.lastFlashMs();

            // Streaming flash session state (M-β.3). The HTTP /omniC6Ota
            // path is asynchronous — it returns 202 Accepted as soon as
            // the upload finishes, and clients poll these fields until
            // session_active goes false to learn the outcome.
            JsonObject session = doc["session"].to<JsonObject>();
            session["active"]            = self->flashStreamActive();
            session["last_ok"]           = self->flashStreamLastOk();
            session["last_error"]        = self->flashStreamLastError();
            session["last_error_msg"]    = self->flashStreamLastErrorMsg();
            session["bytes_processed"]   = self->flashStreamBytesProcessed();
            session["duration_ms"]       = self->flashStreamDurationMs();
            session["running_crc"]       = self->flashStreamRunningCrc();

            // SPI link telemetry — Push B onward. last_rx_ms_ago is the
            // primary "is the C6 talking?" indicator; a healthy link has
            // it under 2 s during the idle keepalive cadence.
            JsonObject link = doc["link"].to<JsonObject>();
            auto s = self->_spiMaster.stats();
            uint32_t now = millis();
            link["task_running"]   = self->_spiMaster.taskRunning();
            link["handshake_pin"]  = self->_pins.boot;
            link["handshake_state"] = digitalRead(self->_pins.boot) ? "high" : "low";
            link["tx_frames"]      = s.tx_frames;
            link["rx_frames"]      = s.rx_frames;
            link["bad_magic"]      = s.bad_magic;
            link["bad_crc"]        = s.bad_crc;
            link["tx_dropped"]     = s.tx_dropped;
            link["handshake_irqs"] = s.handshake_irqs;
            link["last_rx_ms_ago"] = (s.last_rx_ms == 0) ? (uint32_t)0 : (now - s.last_rx_ms);
            link["last_tx_ms_ago"] = (s.last_tx_ms == 0) ? (uint32_t)0 : (now - s.last_tx_ms);

            // Push C: hello/heartbeat handshake + log forwarding state.
            // link.up is the canonical "is the C6 actually responding to
            // our protocol?" indicator. Distinct from task_running, which
            // only confirms the master loop is alive.
            OmniController::LinkState ls{};
            if (self->_linkMutex && xSemaphoreTake(self->_linkMutex, portMAX_DELAY) == pdTRUE) {
                ls = self->_link;
                xSemaphoreGive(self->_linkMutex);
            }
            link["up"]               = ls.hello_acked;
            link["c6_proto"]         = ls.c6_proto;
            link["c6_fw"]            = (const char*)ls.c6_fw;
            link["hello_attempts"]   = ls.hello_attempts;
            link["last_pong_ms_ago"] = (ls.last_pong_ms == 0) ? (uint32_t)0 : (now - ls.last_pong_ms);
            link["last_pong_rtt_ms"] = ls.last_pong_rtt_ms;
            link["log_lines"]        = ls.log_lines;
            link["last_log_ms_ago"]  = (ls.last_log_ms == 0) ? (uint32_t)0 : (now - ls.last_log_ms);

            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });
    endpoints->addEndpoint(c6StatusEndpoint);

    auto c6ResetEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omniC6Reset")
        .summary("Reset the C6 into application mode")
        .description("Pulses the C6 EN pin LOW for ~50 ms, then releases. The C6 reboots "
                     "into whatever image its OTA partition table currently points at. "
                     "Returns immediately; the C6 takes a few hundred ms to come back up.")
        .params({})
        .responseType(JSON_RESPONSE)
        .responseDescription("JSON acknowledgement")
        .handler([self](std::map<String, String>& /*params*/) -> std::pair<String, int> {
            self->withSpiSuspended([self]() { self->_flasher.resetTarget(); });
            JsonDocument doc;
            doc["ok"] = true;
            doc["action"] = omni::flasherActionName(self->_flasher.lastAction());
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });
    endpoints->addEndpoint(c6ResetEndpoint);

    auto c6BootloaderEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omniC6Bootloader")
        .summary("Reset the C6 into ROM serial bootloader")
        .description("Holds the C6 BOOT pin LOW while pulsing EN, then releases BOOT. "
                     "The C6 enters its ROM serial bootloader and waits for the "
                     "esp-serial-flasher protocol on UART. Manual operation; the actual "
                     "flash workflow lives behind /omniC6Ota (lands in M-beta.2).")
        .params({})
        .responseType(JSON_RESPONSE)
        .responseDescription("JSON acknowledgement")
        .handler([self](std::map<String, String>& /*params*/) -> std::pair<String, int> {
            // Don't restart the link task here — the C6 is now in ROM
            // bootloader, won't speak our protocol until the user resets it
            // (which goes through withSpiSuspended again, restarting then).
            bool wasRunning = self->_spiMaster.taskRunning();
            if (wasRunning) self->_spiMaster.stop();
            self->_flasher.enterBootloader();
            JsonDocument doc;
            doc["ok"] = true;
            doc["action"] = omni::flasherActionName(self->_flasher.lastAction());
            doc["link_task_was_running"] = wasRunning;
            doc["link_task_running"] = false;
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });
    endpoints->addEndpoint(c6BootloaderEndpoint);

    auto c6ProbeEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omniC6Probe")
        .summary("Probe the C6 over UART via esp-serial-flasher")
        .description("M-beta.2 smoke test. Drives the C6 into ROM bootloader mode, runs "
                     "the esp-serial-flasher SYNC handshake, and reads back the chip ID. "
                     "Used to verify the integration links and that the protocol speaks "
                     "to the C6 before wiring the full /omniC6Ota flash workflow. "
                     "Takes ~1-2 seconds; the C6 is left in bootloader state and should "
                     "be reset (/omniC6Reset) afterwards if the application is needed.")
        .params({})
        .responseType(JSON_RESPONSE)
        .responseDescription("JSON probe result with chip name and timing")
        .handler([self](std::map<String, String>& /*params*/) -> std::pair<String, int> {
            // Probe leaves the C6 in bootloader state — same as the bootloader
            // endpoint, the link task can't restart yet. /omniC6Reset is the
            // way back to normal operation.
            bool wasRunning = self->_spiMaster.taskRunning();
            if (wasRunning) self->_spiMaster.stop();
            auto r = self->_flasher.probeC6Target();
            JsonDocument doc;
            doc["ok"] = r.ok;
            doc["error_code"] = r.errorCode;
            doc["chip"] = r.chipName;
            doc["duration_ms"] = r.durationMs;
            doc["link_task_was_running"] = wasRunning;
            doc["link_task_running"] = false;
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, r.ok ? 200 : 502);
        });
    endpoints->addEndpoint(c6ProbeEndpoint);

    // /omniC6EchoTest was the M-γ Push A acceptance test (synchronous round-
    // trip via two direct transact() calls). Push B replaces it with the
    // continuous link task — /omniC6Status now shows tx_frames and rx_frames
    // climbing as the idle keepalive bounces back from the C6, which is a
    // stronger ongoing signal than a one-shot test.

    auto c6PollHandshakeEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omniC6PollHandshake")
        .summary("Watch the HANDSHAKE pin for activity (alive check)")
        .description("Polls the C6_HANDSHAKE pin (S3 GPIO12 = bodge wire to "
                     "C6 GPIO9) for 2.5 seconds at 1 ms intervals and counts "
                     "transitions. Useful when SPI itself is failing — the C6 "
                     "firmware blinks GPIO9 ten times at boot before "
                     "SpiSlaveTask takes over, so a transition count of 20+ "
                     "means the C6 booted into our firmware. 0 transitions "
                     "and steady_state=high means either the C6 isn't booting "
                     "into our firmware or it booted long enough ago that the "
                     "blink window is over (reset the C6 first via "
                     "/omniC6Reset and immediately call this).")
        .params({})
        .responseType(JSON_RESPONSE)
        .responseDescription("transition count + steady-state level")
        .handler([self](std::map<String, String>& /*params*/) -> std::pair<String, int> {
            const uint32_t windowMs = 2500;
            const uint32_t startMs = millis();
            int prev = digitalRead(self->_pins.boot);
            int transitions = 0;
            int highCount = 0;
            int samples = 0;
            while (millis() - startMs < windowMs) {
                int v = digitalRead(self->_pins.boot);
                if (v != prev) transitions++;
                if (v == HIGH) highCount++;
                samples++;
                prev = v;
                delayMicroseconds(1000);
            }
            JsonDocument doc;
            doc["transitions"]  = transitions;
            doc["window_ms"]    = windowMs;
            doc["samples"]      = samples;
            doc["high_count"]   = highCount;
            doc["pct_high"]     = samples ? (highCount * 100 / samples) : 0;
            doc["steady_state"] = (transitions == 0)
                ? (prev == HIGH ? "high" : "low")
                : "varying";
            doc["pin"] = self->_pins.boot;
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });
    endpoints->addEndpoint(c6PollHandshakeEndpoint);

    auto c6CaptureBootLogEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omniC6CaptureBootLog")
        .summary("Reset C6 and capture its UART boot output")
        .description("Pulses EN to reset the C6, then reads UART_NUM_1 (which is "
                     "wired to the C6's UART0_TX = GPIO16 = default esp_log output) "
                     "for `ms` milliseconds. Returns whatever the C6 printed during "
                     "boot. Decodes ASCII bytes; non-printable bytes show as '?'. "
                     "Use this when SPI itself is failing to find out whether the "
                     "C6 is booting at all, in a boot loop, or stuck before our "
                     "app's first log line. Don't run during an active flash "
                     "session — UART_NUM_1 is shared with esp-serial-flasher.")
        .params({
            INT_PARAM("ms", "Capture window in ms (default 3000, max 10000)")
        })
        .responseType(JSON_RESPONSE)
        .responseDescription("captured text + raw byte count + first-N hex")
        .handler([self](std::map<String, String>& params) -> std::pair<String, int> {
            if (self->_flasher.flashActive()) {
                return std::pair<String, int>(
                    String("{\"ok\":false,\"error\":\"flash_active\"}"), 409);
            }
            uint32_t ms = 3000;
            if (params.find("ms") != params.end()) {
                ms = (uint32_t)params["ms"].toInt();
                if (ms < 100) ms = 100;
                if (ms > 10000) ms = 10000;
            }

            // Flush whatever's been sitting in the RX buffer so we only see
            // bytes that arrive AFTER the reset we're about to trigger.
            uart_flush_input(UART_NUM_1);

            // Reset the C6. flashes EN-low briefly. C6 reboots into its app
            // (or whatever's currently in its boot partition) and starts
            // printing log lines on its UART0_TX.
            self->_flasher.resetTarget();
            uint32_t startMs = millis();

            // Read bytes for the capture window, accumulating into a buffer.
            // Cap at 16 KB to keep the response payload sane.
            constexpr size_t kCapMax = 16384;
            static uint8_t accum[kCapMax];
            size_t accumLen = 0;
            uint8_t chunk[256];
            while (millis() - startMs < ms && accumLen < kCapMax) {
                int got = uart_read_bytes(
                    UART_NUM_1, chunk, sizeof(chunk), pdMS_TO_TICKS(50));
                if (got > 0) {
                    size_t toCopy = (size_t)got;
                    if (accumLen + toCopy > kCapMax) toCopy = kCapMax - accumLen;
                    memcpy(accum + accumLen, chunk, toCopy);
                    accumLen += toCopy;
                }
            }

            JsonDocument doc;
            doc["ok"]         = true;
            doc["bytes"]      = (uint32_t)accumLen;
            doc["window_ms"]  = ms;
            doc["truncated"]  = (accumLen == kCapMax);

            // Decode to printable text. CR stripped, LF kept, TAB kept,
            // other non-printable replaced by '?'. Cap at 12 KB of text to
            // keep JSON bounded.
            String text;
            text.reserve(accumLen);
            for (size_t i = 0; i < accumLen && text.length() < 12288; i++) {
                uint8_t b = accum[i];
                if (b == '\r') continue;
                if (b == '\n' || b == '\t') text += static_cast<char>(b);
                else if (b >= 0x20 && b < 0x7F) text += static_cast<char>(b);
                else text += '?';
            }
            doc["text"] = text;

            // Also include a hex dump of the first 64 bytes so the user can
            // sanity-check a boot ROM banner even if the baud is mid-transition.
            String hex;
            size_t hexLen = accumLen < 64 ? accumLen : 64;
            hex.reserve(hexLen * 3);
            for (size_t i = 0; i < hexLen; i++) {
                char tmp[4];
                snprintf(tmp, sizeof(tmp), "%02x ", accum[i]);
                hex += tmp;
            }
            if (hex.length()) hex.remove(hex.length() - 1);
            doc["first_64_hex"] = hex;

            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });
    endpoints->addEndpoint(c6CaptureBootLogEndpoint);
}

bool OmniController::handleSerialCommand(const String& line) {
    if (!line.startsWith("OMNI_C6_FLASH ")) return false;
    return handleSerialFlashCommand(line);
}

bool OmniController::handleSerialFlashCommand(const String& line) {
    // Parse "OMNI_C6_FLASH <size> <crc32hex>"
    int sp1 = line.indexOf(' ', 14);
    if (sp1 < 0) {
        Serial.println("OMNI_C6_FLASH_ERR bad_command");
        return true;
    }
    uint32_t size = (uint32_t)line.substring(14, sp1).toInt();
    String crcStr = line.substring(sp1 + 1);
    crcStr.trim();
    uint32_t expectedCrc = (uint32_t)strtoul(crcStr.c_str(), nullptr, 16);

    if (!_flasher.began()) {
        Serial.println("OMNI_C6_FLASH_ERR flasher_not_begun");
        return true;
    }

    // Best-effort RX buffer bump. Arduino-ESP32 v3 HWCDC ignores this once
    // begin() has been called (which happens at boot when ARDUINO_USB_CDC_ON_BOOT=1),
    // so this typically returns false. The actual buffer size is set via the
    // build-flag CONFIG_TINYUSB_CDC_RX_BUFSIZE (see platformio.ini). The call
    // is here so that if a future framework version honours runtime resize,
    // we'd benefit automatically.
    HWCDCSerial.setRxBufferSize(16384);

    // Hand off to the streaming session machinery. startFlashStream pauses
    // the SPI master, opens the C6 flash session via flashBegin, and spawns
    // the worker task. Producers (this loop, or the HTTP /omniC6Ota
    // upload callback) just push bytes through feedFlashStream.
    if (!startFlashStream(size, 0x0, expectedCrc)) {
        // startFlashStream prints the specific cause; surface a generic
        // protocol error to the host.
        Serial.printf("OMNI_C6_FLASH_ERR begin %d\n", (int)_flasher.lastFlashError());
        return true;
    }

    Serial.println("OMNI_C6_READY");
    Serial.flush();

    // Pump bytes from USB-CDC into the stream buffer until the worker has
    // consumed `size` bytes (which we observe by watching flashStreamActive).
    // The worker handles all the CRC, block-fill, flashWrite, finish/abort
    // logic — this loop is now just a producer.
    uint32_t fed = 0;
    uint32_t lastByteMs = millis();
    uint32_t lastProgressLog = 0;
    static uint8_t scratch[1024];
    while (_flashStreamActive && fed < size) {
        if (millis() - lastByteMs > 30000) {
            // Producer-side timeout — nothing has come from USB-CDC for 30 s.
            // Bail; the worker's own 30 s timeout would also catch this, but
            // surfacing it from the producer side gives a clearer error.
            abortFlashStream();
            Serial.printf("OMNI_C6_FLASH_ERR rx_timeout %u_of_%u\n",
                          (unsigned)fed, (unsigned)size);
            return true;
        }
        int avail = Serial.available();
        if (avail <= 0) {
            yield();
            continue;
        }
        size_t toRead = (size_t)avail;
        if (toRead > sizeof(scratch))   toRead = sizeof(scratch);
        if (toRead > (size_t)(size - fed)) toRead = (size_t)(size - fed);
        size_t got = Serial.readBytes((char*)scratch, toRead);
        if (got == 0) {
            yield();
            continue;
        }
        // Block up to 1 s pushing into the stream buffer. If the worker is
        // backlogged (UART can't drain fast enough) we briefly stall the
        // USB-CDC reader — that's fine, USB-CDC has its own RX queue.
        size_t pushed = feedFlashStream(scratch, got, 1000);
        if (pushed != got) {
            // Worker died or buffer permanently full — bail.
            abortFlashStream();
            Serial.printf("OMNI_C6_FLASH_ERR feed_short pushed=%u got=%u offset=%u\n",
                          (unsigned)pushed, (unsigned)got, (unsigned)fed);
            return true;
        }
        fed += (uint32_t)got;
        lastByteMs = millis();

        if (fed - lastProgressLog >= 32 * 1024 || fed == size) {
            Serial.printf("OmniSerial: rx %u/%u (%u%%)\n",
                          (unsigned)fed, (unsigned)size,
                          (unsigned)((uint64_t)fed * 100 / size));
            lastProgressLog = fed;
        }
    }

    // All bytes pushed. Wait for the worker to finalise (it handles CRC
    // verification, flashFinish/abort, link-master restart, etc).
    Serial.printf("OMNI_C6_RX_OK %u bytes\n", (unsigned)fed);
    Serial.flush();

    // Worker keeps _flashStreamActive true until it's fully drained the
    // stream buffer, called flashFinish (which does the EN reset), and
    // restarted the link master. Bound the wait at 90 s — that's well
    // beyond any normal flashFinish (verify + finish + EN pulse < 5 s).
    uint32_t waitStart = millis();
    while (_flashStreamActive && (millis() - waitStart) < 90000) {
        delay(50);
    }
    if (_flashStreamActive) {
        Serial.println("OMNI_C6_FLASH_ERR worker_timeout");
        abortFlashStream();
        return true;
    }

    if (!_flashStreamOk) {
        Serial.printf("OMNI_C6_FLASH_ERR %s code=%d\n",
                      _flashStreamErrorMsg ? _flashStreamErrorMsg : "unknown",
                      (int)_flashStreamErrorCode);
        return true;
    }

    Serial.printf("OMNI_C6_FLASH_OK size=%u flash_ms=%u\n",
                  (unsigned)size, (unsigned)_flashStreamDurationMs);
    // Worker has already done the EN reset (via flashFinish), reset link
    // state, and restarted the SPI master — nothing else to do here.
    return true;
}

// ── Link manager (Push C) ──────────────────────────────────────────────────

void OmniController::onLinkFrame(omni::Channel ch, uint8_t flags, uint16_t seq,
                                 const uint8_t* payload, uint16_t payloadLen) {
    switch (ch) {
        case omni::Channel::Ctrl:
            handleCtrlFrame(flags, seq, payload, payloadLen);
            break;
        case omni::Channel::Log:
            handleLogFrame(payload, payloadLen);
            break;
        // OTA (M-δ), Thread/Zigbee/Matter (M-θ/M-ι), Radio (M-κ) handlers
        // attach in their respective milestones. Drop unknowns silently.
        default:
            break;
    }
}

void OmniController::handleCtrlFrame(uint8_t /*flags*/, uint16_t /*seq*/,
                                     const uint8_t* payload, uint16_t payloadLen) {
    if (payload == nullptr || payloadLen == 0) return;

    JsonDocument doc;
    if (deserializeJson(doc, payload, payloadLen)) return;
    const char* op = doc["op"] | "";

    if (strcmp(op, omni::ctrl::kOpHelloAck) == 0) {
        bool firstUp = false;
        if (_linkMutex && xSemaphoreTake(_linkMutex, portMAX_DELAY) == pdTRUE) {
            firstUp = !_link.hello_acked;
            _link.hello_acked    = true;
            _link.hello_acked_ms = millis();
            _link.c6_proto       = doc["proto"] | -1;
            const char* fw = doc["fw"] | "";
            strncpy(_link.c6_fw, fw, sizeof(_link.c6_fw) - 1);
            _link.c6_fw[sizeof(_link.c6_fw) - 1] = '\0';
            xSemaphoreGive(_linkMutex);
        }
        if (firstUp) {
            Serial.printf("OmniController: link UP (c6_proto=%d c6_fw=%s)\n",
                          (int)(doc["proto"] | -1),
                          (const char*)(doc["fw"] | ""));
        }
    } else if (strcmp(op, omni::ctrl::kOpPong) == 0) {
        uint32_t origTs = doc["ts"] | 0u;
        uint32_t now    = millis();
        if (_linkMutex && xSemaphoreTake(_linkMutex, portMAX_DELAY) == pdTRUE) {
            _link.last_pong_ms     = now;
            _link.last_pong_rtt_ms = (origTs && now >= origTs) ? (now - origTs) : 0;
            xSemaphoreGive(_linkMutex);
        }
    }
    // Other CTRL ops (OTA, radio role changes) handled in later milestones.
}

void OmniController::handleLogFrame(const uint8_t* payload, uint16_t payloadLen) {
    if (payload == nullptr || payloadLen == 0) return;

    // Print "[c6] <line>". SerialTee captures Serial.print and replays it on
    // the WebSocket — so C6 logs land in the dashboard alongside S3 logs
    // automatically, no extra plumbing.
    Serial.print("[c6] ");
    for (uint16_t i = 0; i < payloadLen; i++) {
        char c = static_cast<char>(payload[i]);
        if (c == '\n' || c == '\t' || (c >= 0x20 && c < 0x7F)) Serial.write(c);
        else Serial.write('?');
    }
    Serial.println();

    if (_linkMutex && xSemaphoreTake(_linkMutex, 0) == pdTRUE) {
        _link.last_log_ms = millis();
        _link.log_lines++;
        xSemaphoreGive(_linkMutex);
    }
}

void OmniController::sendHello() {
    JsonDocument doc;
    doc["op"]    = omni::ctrl::kOpHello;
    doc["proto"] = OMNI_PROTO_VERSION;
    doc["fw"]    = OMNI_S3_FW_VERSION;
    char buf[200];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n == 0) return;
    if (_spiMaster.sendFrame(omni::Channel::Ctrl, omni::FlagAckReq,
                             _ctrlTxSeq++,
                             reinterpret_cast<const uint8_t*>(buf),
                             static_cast<uint16_t>(n))) {
        if (_linkMutex && xSemaphoreTake(_linkMutex, portMAX_DELAY) == pdTRUE) {
            _link.hello_attempts++;
            xSemaphoreGive(_linkMutex);
        }
    }
}

void OmniController::sendPing() {
    JsonDocument doc;
    doc["op"] = omni::ctrl::kOpPing;
    doc["ts"] = millis();
    char buf[80];
    size_t n = serializeJson(doc, buf, sizeof(buf));
    if (n == 0) return;
    _spiMaster.sendFrame(omni::Channel::Ctrl, 0, _ctrlTxSeq++,
                         reinterpret_cast<const uint8_t*>(buf),
                         static_cast<uint16_t>(n));
}

void OmniController::ctrlPumpTrampoline(void* arg) {
    static_cast<OmniController*>(arg)->ctrlPumpLoop();
    vTaskDelete(nullptr);
}

void OmniController::ctrlPumpLoop() {
    // Initial 250 ms grace so the SPI master task and the C6's slave task
    // both reach steady state before we start probing the link.
    vTaskDelay(pdMS_TO_TICKS(250));
    while (true) {
        bool acked = false;
        if (_linkMutex && xSemaphoreTake(_linkMutex, portMAX_DELAY) == pdTRUE) {
            acked = _link.hello_acked;
            xSemaphoreGive(_linkMutex);
        }
        if (!acked) {
            sendHello();
            vTaskDelay(pdMS_TO_TICKS(1000));
        } else {
            sendPing();
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

// ── Streaming flash session (M-β.3) ────────────────────────────────────────

bool OmniController::startFlashStream(uint32_t imageSize, uint32_t flashOffset,
                                      uint32_t expectedCrc) {
    if (_flashStreamActive) {
        Serial.println("OmniController: flash stream already active");
        return false;
    }
    if (imageSize == 0 || imageSize > 8 * 1024 * 1024) {
        Serial.printf("OmniController: bad flash size %u\n", (unsigned)imageSize);
        return false;
    }
    if (!_flasher.began()) {
        Serial.println("OmniController: flasher not begun");
        return false;
    }
    if (_fs == nullptr) {
        Serial.println("OmniController: no FS attached for OTA staging");
        return false;
    }
    if (_flashWorkerDone == nullptr) {
        Serial.println("OmniController: flash worker primitives not initialised (begin() must run first)");
        return false;
    }

    // Open the staging file fresh — truncate any partial leftovers from
    // an aborted previous session.
    if (_fs->exists(kFlashStagePath)) _fs->remove(kFlashStagePath);
    _stagingFile = _fs->open(kFlashStagePath, FILE_WRITE);
    if (!_stagingFile) {
        Serial.printf("OmniController: failed to open staging file %s\n", kFlashStagePath);
        return false;
    }

    // Clear stale completion signal so a new wait starts from known state.
    xSemaphoreTake(_flashWorkerDone, 0);

    _flashStreamSize           = imageSize;
    _flashStreamOffset         = flashOffset;
    _flashStreamExpectedCrc    = expectedCrc;
    _flashStreamAborted        = false;
    _flashStreamOk             = false;
    _flashStreamErrorCode      = 0;
    _flashStreamErrorMsg       = nullptr;
    _flashStreamBytesProcessed = 0;
    _flashStreamDurationMs     = 0;
    _flashStreamRunningCrc     = 0;
    _flashStreamStartMs        = millis();
    _flashStreamActive         = true;
    _flashStreamLinkWasRunning = false;  // we don't pause the SPI master
                                          // until the worker actually starts
                                          // flashing — keep the link alive
                                          // during the upload phase

    Serial.printf("OmniController: flash stream started size=%u offset=0x%X (staging %s)\n",
                  (unsigned)imageSize, (unsigned)flashOffset, kFlashStagePath);
    return true;
}

size_t OmniController::feedFlashStream(const uint8_t* data, size_t len,
                                       uint32_t /*timeoutMs*/) {
    if (!_flashStreamActive || !_stagingFile) return 0;
    if (data == nullptr || len == 0) return 0;
    if (_flashStreamAborted) return 0;

    // Append to the staging file. SPIFFS writes at ~50-100 KB/s with
    // ~15-30 ms per ~1.4 KB chunk — short enough that the upload callback
    // is not effectively blocking AsyncTCP for any meaningful duration.
    size_t written = _stagingFile.write(data, len);
    if (written != len) {
        Serial.printf("OmniController: staging write short (got=%u of %u)\n",
                      (unsigned)written, (unsigned)len);
        // Probably out of SPIFFS space — abort the session.
        _flashStreamAborted = true;
        return written;
    }
    _flashStreamRunningCrc = esp_rom_crc32_le(_flashStreamRunningCrc, data, len);
    _flashStreamBytesProcessed += len;

    // When the upload hits the expected size, close the file, kick off
    // the worker, and let the producer return — completion is asynchronous.
    if (_flashStreamBytesProcessed >= _flashStreamSize) {
        _stagingFile.flush();
        _stagingFile.close();

        // Pause the SPI master before the worker drives BOOT/HANDSHAKE.
        _flashStreamLinkWasRunning = _spiMaster.taskRunning();
        if (_flashStreamLinkWasRunning) _spiMaster.stop();

        BaseType_t ok = xTaskCreatePinnedToCore(
            &OmniController::flashWorkerTrampoline,
            "OmniFlash",
            kFlashWorkerStackBytes,
            this,
            kFlashWorkerPriority,
            &_flashWorkerTask,
            kFlashWorkerCore);
        if (ok != pdPASS) {
            Serial.println("OmniController: flash worker task create failed");
            finishFlashWorker(false, -1, "task_create_failed");
        }
    }
    return len;
}

void OmniController::abortFlashStream() {
    if (!_flashStreamActive) return;
    _flashStreamAborted = true;
    if (_stagingFile) {
        _stagingFile.close();
    }
    if (_fs && _fs->exists(kFlashStagePath)) {
        _fs->remove(kFlashStagePath);
    }
    // If a worker is running it'll see _flashStreamAborted on its next
    // iteration; if not, the upload was aborted before the worker spawned
    // — finish the session right here.
    if (_flashWorkerTask == nullptr) {
        finishFlashWorker(false, -2, "aborted");
    }
}

void OmniController::flashWorkerTrampoline(void* arg) {
    static_cast<OmniController*>(arg)->flashWorkerLoop();
    vTaskDelete(nullptr);
}

void OmniController::flashWorkerLoop() {
    // Producer-supplied CRC check before we touch the C6 — if the
    // upload was corrupted in transit (HTTP path: the CRC is 0 and
    // skipped; USB-CDC path: CRC32 over the bytes the script computed),
    // catch it here without burning a bootloader cycle.
    if (_flashStreamExpectedCrc != 0 &&
        _flashStreamRunningCrc != _flashStreamExpectedCrc) {
        finishFlashWorker(false, -3, "crc_mismatch");
        return;
    }

    if (_fs == nullptr) {
        finishFlashWorker(false, -4, "no_fs");
        return;
    }
    File staged = _fs->open(kFlashStagePath, FILE_READ);
    if (!staged) {
        finishFlashWorker(false, -5, "staging_open_failed");
        return;
    }

    // The worker owns the flash session for its lifetime. Open the C6
    // bootloader connection here (in the worker task's context, NOT on
    // AsyncTCP) so esp-serial-flasher's blocking UART work happens away
    // from any latency-sensitive callers.
    if (!_flasher.flashBegin(_flashStreamSize, _flashStreamOffset)) {
        staged.close();
        finishFlashWorker(false, _flasher.lastFlashError(), "flashBegin failed");
        return;
    }

    constexpr uint32_t kBlockSize = omni::OmniUartFlasher::kBlockSize;
    static uint8_t block[kBlockSize];
    uint32_t totalRead = 0;

    while (totalRead < _flashStreamSize) {
        if (_flashStreamAborted) {
            _flasher.flashAbort();
            staged.close();
            finishFlashWorker(false, -2, "aborted");
            return;
        }

        uint32_t toRead = _flashStreamSize - totalRead;
        if (toRead > kBlockSize) toRead = kBlockSize;

        int got = staged.read(block, toRead);
        if (got <= 0) {
            // Premature EOF — staging file truncated unexpectedly.
            _flasher.flashAbort();
            staged.close();
            finishFlashWorker(false, -6, "staging_read_short");
            return;
        }

        if (!_flasher.flashWrite(block, (uint32_t)got)) {
            _flasher.flashAbort();
            staged.close();
            finishFlashWorker(false, _flasher.lastFlashError(), "flashWrite failed");
            return;
        }

        totalRead += (uint32_t)got;
    }

    staged.close();

    // flashFinish handles MD5 verify + EN-pulse reset of the C6.
    if (!_flasher.flashFinish(true /*reboot*/)) {
        finishFlashWorker(false, _flasher.lastFlashError(), "flashFinish failed");
        return;
    }

    finishFlashWorker(true, 0, "ok");
}

void OmniController::finishFlashWorker(bool ok, int32_t errorCode, const char* msg) {
    _flashStreamOk         = ok;
    _flashStreamErrorCode  = errorCode;
    _flashStreamErrorMsg   = msg;
    _flashStreamDurationMs = millis() - _flashStreamStartMs;
    Serial.printf("OmniController: flash worker exit ok=%d err=%d msg=%s ms=%u\n",
                  ok ? 1 : 0, (int)errorCode, msg ? msg : "", (unsigned)_flashStreamDurationMs);

    // Make sure the staging file is closed (idempotent) and removed —
    // it's served its purpose, no reason to leave 400+ KB on SPIFFS.
    if (_stagingFile) _stagingFile.close();
    if (_fs && _fs->exists(kFlashStagePath)) {
        _fs->remove(kFlashStagePath);
    }

    // Restart the link master if it was running before the session opened.
    // The flashFinish above already did the EN pulse, so the C6 is rebooting
    // into the freshly-flashed firmware; give it 200 ms to settle before
    // we re-arm the IRQ.
    if (_flashStreamLinkWasRunning) {
        vTaskDelay(pdMS_TO_TICKS(200));
        if (_linkMutex && xSemaphoreTake(_linkMutex, portMAX_DELAY) == pdTRUE) {
            _link.hello_acked = false;
            _link.c6_proto    = -1;
            _link.c6_fw[0]    = '\0';
            xSemaphoreGive(_linkMutex);
        }
        _spiMaster.start();
    }

    // Clear the active flag LAST so any external waiter (USB-CDC handler,
    // /omniC6Status poller) sees the result fields populated before active
    // goes false.
    _flashStreamActive = false;
    _flashWorkerTask   = nullptr;
    if (_flashWorkerDone) xSemaphoreGive(_flashWorkerDone);
}
