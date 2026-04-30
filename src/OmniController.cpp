#include "OmniController.h"

#include <ArduinoJson.h>
#include <esp_rom_crc.h>

#include "OmniProto.h"

#ifndef OMNI_S3_FW_VERSION
#define OMNI_S3_FW_VERSION "omni-0.1.0"
#endif

bool OmniController::begin(FlexibleEndpoints* endpoints, const OmniPins& pins) {
    if (_began) return true;

    _pins = pins;
    _flasher.begin(_pins.en, _pins.boot, _pins.uart_tx, _pins.uart_rx);
    _spiMaster.begin(_pins.spi_cs, _pins.spi_mosi, _pins.spi_miso, _pins.spi_clk, _pins.boot);

    if (endpoints) {
        registerEndpoints(endpoints);
    }

    _began = true;
    Serial.println("OmniController: initialized (M-beta.1)");
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
            doc["milestone"] = "M-beta.1";
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
            self->_flasher.resetTarget();
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
            self->_flasher.enterBootloader();
            JsonDocument doc;
            doc["ok"] = true;
            doc["action"] = omni::flasherActionName(self->_flasher.lastAction());
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
            auto r = self->_flasher.probeC6Target();
            JsonDocument doc;
            doc["ok"] = r.ok;
            doc["error_code"] = r.errorCode;
            doc["chip"] = r.chipName;
            doc["duration_ms"] = r.durationMs;
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, r.ok ? 200 : 502);
        });
    endpoints->addEndpoint(c6ProbeEndpoint);

    auto c6EchoEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omniC6EchoTest")
        .summary("Round-trip echo test over the C6 SPI link")
        .description("M-gamma Push A acceptance test. Sends a fixed CTRL test "
                     "frame to the C6 over SPI, then clocks a second transaction "
                     "to read back the C6's echo (which arrives on the next "
                     "transaction since SPI is full-duplex but the slave only "
                     "knows what to echo after the first transaction completes). "
                     "Returns the round-trip details as JSON; payload_text "
                     "should read 'echo: <orig>' on success. Diagnostic fields "
                     "rx1_hex / rx2_hex show the first 32 bytes received on each "
                     "transaction — useful when frame_valid is false.")
        .params({})
        .responseType(JSON_RESPONSE)
        .responseDescription("JSON with sent/received seq, payload, validity")
        .handler([self](std::map<String, String>& /*params*/) -> std::pair<String, int> {
            static uint16_t seq = 1;
            static uint8_t txBuf[omni::kSpiTransactionBytes];
            static uint8_t rx1Buf[omni::kSpiTransactionBytes];
            static uint8_t rx2Buf[omni::kSpiTransactionBytes];

            auto hexDump = [](const uint8_t* buf, size_t n) {
                String s;
                s.reserve(n * 3);
                for (size_t i = 0; i < n; i++) {
                    char hex[4];
                    snprintf(hex, sizeof(hex), "%02x ", buf[i]);
                    s += hex;
                }
                if (s.length()) s.remove(s.length() - 1);  // trim trailing space
                return s;
            };

            const char* msg = "echo-test";
            const uint16_t msgLen = static_cast<uint16_t>(strlen(msg));

            // Transaction 1: deliver our test frame, receive whatever the slave
            // had pre-loaded (probably empty / no MAGIC on first call).
            size_t encoded = omni::encodeFrame(
                txBuf, sizeof(txBuf),
                omni::Channel::Ctrl, omni::FlagAckReq, seq,
                reinterpret_cast<const uint8_t*>(msg), msgLen);
            if (encoded == 0) {
                return std::pair<String, int>(
                    String("{\"ok\":false,\"error\":\"encode_failed\"}"), 500);
            }
            if (!self->_spiMaster.transact(txBuf, rx1Buf)) {
                return std::pair<String, int>(
                    String("{\"ok\":false,\"error\":\"transact_1_failed\"}"), 500);
            }

            // Small delay to give the C6 time to decode T1 and queue the echo
            // for T2. SpiSlaveTask runs in its own task so this is generous.
            delay(20);

            // Transaction 2: send another ping (so the slave clocks data), receive
            // the echo of transaction 1's payload from the slave's TX buffer.
            uint16_t pingSeq = static_cast<uint16_t>(seq + 1);
            encoded = omni::encodeFrame(
                txBuf, sizeof(txBuf),
                omni::Channel::Ctrl, 0, pingSeq,
                reinterpret_cast<const uint8_t*>("ping"), 4);
            if (encoded == 0) {
                return std::pair<String, int>(
                    String("{\"ok\":false,\"error\":\"encode_2_failed\"}"), 500);
            }
            if (!self->_spiMaster.transact(txBuf, rx2Buf)) {
                return std::pair<String, int>(
                    String("{\"ok\":false,\"error\":\"transact_2_failed\"}"), 500);
            }

            JsonDocument doc;
            doc["sent_seq"]     = seq;
            doc["sent_payload"] = msg;
            doc["rx1_hex"]      = hexDump(rx1Buf, 32);
            doc["rx2_hex"]      = hexDump(rx2Buf, 32);

            // Quick "is anything alive on MISO" check
            bool rx2_all_ones = true, rx2_all_zeros = true;
            for (size_t i = 0; i < 32; i++) {
                if (rx2Buf[i] != 0xFF) rx2_all_ones = false;
                if (rx2Buf[i] != 0x00) rx2_all_zeros = false;
            }
            if (rx2_all_ones)  doc["miso_state"] = "all_0xFF (slave silent or MISO floating high)";
            else if (rx2_all_zeros) doc["miso_state"] = "all_0x00 (slave silent or MISO pulled low)";
            else doc["miso_state"] = "varied (slave is sending bytes)";

            omni::DecodedFrame d{};
            const bool ok = omni::decodeFrame(rx2Buf, sizeof(rx2Buf), d);
            doc["frame_valid"] = ok;
            if (ok) {
                doc["echoed_channel"] = static_cast<uint8_t>(d.channel);
                doc["echoed_seq"]     = d.seq;
                doc["echoed_flags"]   = d.flags;
                doc["payload_len"]    = d.payloadLen;
                String payloadText;
                payloadText.reserve(d.payloadLen);
                for (uint16_t i = 0; i < d.payloadLen && i < 64; i++) {
                    char c = static_cast<char>(d.payload[i]);
                    payloadText += (c >= 0x20 && c < 0x7F) ? c : '.';
                }
                doc["payload_text"] = payloadText;
                doc["echo_match"] = (d.payloadLen == msgLen + 6 /* "echo: " */
                                     && memcmp(d.payload, "echo: ", 6) == 0
                                     && memcmp(d.payload + 6, msg, msgLen) == 0);
                doc["ok"] = true;
            } else {
                doc["ok"] = false;
                doc["error"] = "decode_failed_or_no_frame";
            }

            seq = static_cast<uint16_t>(seq + 2);
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });
    endpoints->addEndpoint(c6EchoEndpoint);

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

    if (size == 0 || size > 8 * 1024 * 1024) {
        Serial.printf("OMNI_C6_FLASH_ERR bad_size %u\n", (unsigned)size);
        return true;
    }
    if (!_flasher.began()) {
        Serial.println("OMNI_C6_FLASH_ERR flasher_not_begun");
        return true;
    }

    // Open the flash session before signalling READY — that way the C6 enters
    // bootloader mode and the stub uploads while the PC is still preparing
    // its stream. flashBegin can take a couple of seconds; do it first.
    if (!_flasher.flashBegin(size, 0x0)) {
        Serial.printf("OMNI_C6_FLASH_ERR begin %d\n", (int)_flasher.lastFlashError());
        return true;
    }

    // Best-effort RX buffer bump. Arduino-ESP32 v3 HWCDC ignores this once
    // begin() has been called (which happens at boot when ARDUINO_USB_CDC_ON_BOOT=1),
    // so this typically returns false. The actual buffer size is set via the
    // build-flag CONFIG_TINYUSB_CDC_RX_BUFSIZE (see platformio.ini). The call
    // is here so that if a future framework version honours runtime resize,
    // we'd benefit automatically.
    HWCDCSerial.setRxBufferSize(16384);

    Serial.println("OMNI_C6_READY");
    Serial.flush();

    // Streaming pipeline: read a chunk from USB-CDC, accumulate CRC32, hand
    // straight to flashWrite. No buffering of the whole image — works without
    // PSRAM and keeps the stack budget tiny. USB-CDC backpressures the host
    // so the host's send rate naturally matches our flashWrite throughput
    // (~46 KB/s at 460800 baud once the stub is loaded).
    constexpr uint32_t kChunkSize = 4096;
    static uint8_t chunk[kChunkSize];   // static to keep it off a small task stack
    uint32_t runningCrc = 0;
    uint32_t received = 0;
    uint32_t lastByteMs = millis();
    uint32_t lastProgressLog = 0;

    while (received < size) {
        if (millis() - lastByteMs > 15000) {
            _flasher.flashAbort();
            Serial.printf("OMNI_C6_FLASH_ERR rx_timeout %u_of_%u\n",
                          (unsigned)received, (unsigned)size);
            return true;
        }
        int avail = Serial.available();
        if (avail <= 0) {
            yield();
            continue;
        }
        uint32_t toRead = (uint32_t)avail;
        if (toRead > kChunkSize) toRead = kChunkSize;
        if (toRead > (size - received)) toRead = size - received;

        size_t got = Serial.readBytes((char*)chunk, toRead);
        if (got == 0) {
            yield();
            continue;
        }
        runningCrc = esp_rom_crc32_le(runningCrc, chunk, got);
        if (!_flasher.flashWrite(chunk, (uint32_t)got)) {
            _flasher.flashAbort();
            Serial.printf("OMNI_C6_FLASH_ERR write %d at_offset=%u\n",
                          (int)_flasher.lastFlashError(), (unsigned)received);
            return true;
        }
        received += got;
        lastByteMs = millis();

        // Progress log every ~32 KB. Diagnostic only — PC client filters
        // anything that's not OMNI_C6_*. Helps us see where stalls happen.
        if (received - lastProgressLog >= 32 * 1024 || received == size) {
            Serial.printf("OmniSerial: rx %u/%u (%u%%)\n",
                          (unsigned)received, (unsigned)size,
                          (unsigned)((uint64_t)received * 100 / size));
            lastProgressLog = received;
        }
    }

    if (runningCrc != expectedCrc) {
        _flasher.flashAbort();
        Serial.printf("OMNI_C6_CRC_FAIL expected=%08x actual=%08x size=%u\n",
                      (unsigned)expectedCrc, (unsigned)runningCrc, (unsigned)size);
        return true;
    }
    Serial.printf("OMNI_C6_RX_OK %u bytes crc=%08x\n",
                  (unsigned)size, (unsigned)runningCrc);
    Serial.flush();

    if (!_flasher.flashFinish(true /*reboot*/)) {
        Serial.printf("OMNI_C6_FLASH_ERR finish %d\n", (int)_flasher.lastFlashError());
        return true;
    }

    Serial.printf("OMNI_C6_FLASH_OK size=%u flash_ms=%u\n",
                  (unsigned)size, (unsigned)_flasher.lastFlashMs());
    return true;
}
