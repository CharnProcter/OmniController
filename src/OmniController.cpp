#include "OmniController.h"

#include <ArduinoJson.h>
#include <esp_rom_crc.h>
#include <esp_heap_caps.h>

#include "OmniProto.h"

#ifndef OMNI_S3_FW_VERSION
#define OMNI_S3_FW_VERSION "omni-0.1.0"
#endif

bool OmniController::begin(FlexibleEndpoints* endpoints, const OmniPins& pins) {
    if (_began) return true;

    _pins = pins;
    _flasher.begin(_pins.en, _pins.boot, _pins.uart_tx, _pins.uart_rx);

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

    uint8_t* buf = (uint8_t*)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (!buf) {
        Serial.printf("OMNI_C6_FLASH_ERR oom %u\n", (unsigned)size);
        return true;
    }

    Serial.println("OMNI_C6_READY");
    Serial.flush();

    // Tight loop: drain Serial until full size received or timeout. Blocking the
    // main loop is acceptable; AsyncTCP/WiFi run on Core 0 and keep ticking.
    uint32_t received = 0;
    uint32_t lastByteMs = millis();
    while (received < size) {
        if (millis() - lastByteMs > 10000) {
            free(buf);
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
        if (toRead > (size - received)) toRead = size - received;
        size_t got = Serial.readBytes(buf + received, toRead);
        received += got;
        lastByteMs = millis();
    }

    uint32_t actualCrc = esp_rom_crc32_le(0, buf, size);
    if (actualCrc != expectedCrc) {
        free(buf);
        Serial.printf("OMNI_C6_CRC_FAIL expected=%08x actual=%08x size=%u\n",
                      (unsigned)expectedCrc, (unsigned)actualCrc, (unsigned)size);
        return true;
    }
    Serial.printf("OMNI_C6_RX_OK %u bytes crc=%08x\n",
                  (unsigned)size, (unsigned)actualCrc);
    Serial.flush();

    // Hand the buffered image to the existing UART flasher session machinery.
    if (!_flasher.flashBegin(size, 0x0)) {
        free(buf);
        Serial.printf("OMNI_C6_FLASH_ERR begin %d\n", (int)_flasher.lastFlashError());
        return true;
    }

    constexpr uint32_t kStreamChunk = 4096;
    for (uint32_t off = 0; off < size; off += kStreamChunk) {
        uint32_t n = (off + kStreamChunk <= size) ? kStreamChunk : (size - off);
        if (!_flasher.flashWrite(buf + off, n)) {
            _flasher.flashAbort();
            free(buf);
            Serial.printf("OMNI_C6_FLASH_ERR write %d at_offset=%u\n",
                          (int)_flasher.lastFlashError(), (unsigned)off);
            return true;
        }
    }

    if (!_flasher.flashFinish(true /*reboot*/)) {
        free(buf);
        Serial.printf("OMNI_C6_FLASH_ERR finish %d\n", (int)_flasher.lastFlashError());
        return true;
    }

    free(buf);
    Serial.printf("OMNI_C6_FLASH_OK size=%u flash_ms=%u\n",
                  (unsigned)size, (unsigned)_flasher.lastFlashMs());
    return true;
}
