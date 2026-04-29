#include "OmniController.h"

#include <ArduinoJson.h>

#include "AutomationPlatformPlus.h"
#include "OmniProto.h"

#ifndef OMNI_S3_FW_VERSION
#define OMNI_S3_FW_VERSION "omni-0.1.0"
#endif

bool OmniController::begin(FlexibleEndpoints* endpoints) {
    if (_began) return true;

    if (endpoints) {
        registerEndpoints(endpoints);
    }

    _began = true;
    Serial.println("OmniController: initialized (M-alpha skeleton)");
    return true;
}

std::vector<uint8_t> OmniController::getUsedPins() const {
    return {
        C6_EN,
        C6_BOOT,
        LINK_SPI_CSn,
        LINK_SPI_MOSI,
        LINK_SPI_CLK,
        LINK_SPI_MISO,
        C6_UART_TX,
        C6_UART_RX,
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
            doc["milestone"] = "M-alpha";
            JsonArray drivers = doc["drivers"].to<JsonArray>();
            (void)drivers;  // empty in M-alpha; populated as drivers register in later milestones
            String out;
            serializeJson(doc, out);
            return std::pair<String, int>(out, 200);
        });

    endpoints->addEndpoint(statusEndpoint);
}
