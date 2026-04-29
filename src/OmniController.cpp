#include "OmniController.h"

#include <ArduinoJson.h>

#include "OmniProto.h"

#ifndef OMNI_S3_FW_VERSION
#define OMNI_S3_FW_VERSION "omni-0.1.0"
#endif

bool OmniController::begin(FlexibleEndpoints* endpoints, const OmniPins& pins) {
    if (_began) return true;

    _pins = pins;
    _flasher.begin(_pins.en, _pins.boot);

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
        .route("/omni/status")
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
        .route("/omni/c6/status")
        .summary("C6 daughterboard link status")
        .description("Returns current C6 link state: pin assignments, last manual action "
                     "(reset / bootloader / flash) with timestamp. M-gamma adds SPI link "
                     "health; M-delta adds OTA history.")
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
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });
    endpoints->addEndpoint(c6StatusEndpoint);

    auto c6ResetEndpoint = FLEXIBLE_ENDPOINT()
        .route("/omni/c6/reset")
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
        .route("/omni/c6/bootloader")
        .summary("Reset the C6 into ROM serial bootloader")
        .description("Holds the C6 BOOT pin LOW while pulsing EN, then releases BOOT. "
                     "The C6 enters its ROM serial bootloader and waits for the "
                     "esp-serial-flasher protocol on UART. Manual operation; the actual "
                     "flash workflow lives behind /omni/c6/ota (lands in M-beta.2).")
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
}
