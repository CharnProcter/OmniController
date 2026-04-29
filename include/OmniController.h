#pragma once

#include <Arduino.h>
#include <vector>

#include "FlexibleEndpoints.h"

class OmniController {
public:
    bool begin(FlexibleEndpoints* endpoints);

    std::vector<uint8_t> getUsedPins() const;

    bool began() const { return _began; }

private:
    void registerEndpoints(FlexibleEndpoints* endpoints);

    bool _began = false;
};
