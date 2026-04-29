# OmniController

Local-first IoT omni-controller for the AutomationPlatform. Pairs an ESP32-S3 (WiFi, BLE, web UI, driver framework) with an ESP32-C6 co-processor (Zigbee, Thread, Matter-over-Thread) carried on a hardware hat.

The S3 cannot do 802.15.4. The C6 can. The two share a SPI link plus a UART used for bootstrap flashing of the C6.

## Submodule init (one-time)

This library bundles Espressif's `esp-serial-flasher` v1.11.0 as a git submodule under `external/esp-serial-flasher`. After cloning (or pulling the first commit that adds it), run:

```
git submodule update --init --recursive
```

If you skip this, the submodule directory will be empty and the build will fail with `esp_loader.h: No such file or directory`. The submodule is required for the UART-based C6 flasher (M-β.2 onwards).

## Hardware contract (from `AutomationPlatformPlus`)

| Function       | S3 GPIO | Notes                                                                 |
|----------------|---------|-----------------------------------------------------------------------|
| `C6_EN`        | 13      | Output. Reset line.                                                   |
| `C6_BOOT`      | 12      | Dual-purpose: output during UART flash (BOOT strap), input-with-IRQ during normal ops (HANDSHAKE from C6). |
| `LINK_SPI_CSn` | 39      |                                                                        |
| `LINK_SPI_MOSI`| 40      |                                                                        |
| `LINK_SPI_CLK` | 41      |                                                                        |
| `LINK_SPI_MISO`| 42      |                                                                        |
| `C6_UART_TX`   | 43      | esp-serial-flasher only; idle otherwise.                              |
| `C6_UART_RX`   | 44      | esp-serial-flasher only; idle otherwise.                              |

## Module pattern

Follows the AutomationPlatform convention: a single `OmniController` instance is created at file scope in `main.cpp`, registered in `moduleRegistry[]`, runtime-toggled via NVS through the `/setModule?name=omniController` endpoint.

```cpp
#include "OmniController.h"
OmniController omniController;

// In setup():
if (modules.omniController) {
  omniController.begin(endpointsPtr);
}
```

`getUsedPins()` returns all eight C6 hat pins so the existing `checkPinConflicts()` machinery in `main.cpp` blocks any other module from claiming them.

## Milestone status

This library is being built in milestones. The current state corresponds to **M-α**: skeleton, module integration, and `/omni/status` endpoint only. SPI transport, UART flashing, discovery, and drivers land in later milestones (M-β through M-ι). See the project plan for the milestone table.

Public endpoints in M-α / M-β.1:

| Route               | Method | Returns                                                    |
|---------------------|--------|------------------------------------------------------------|
| `/omniStatus`       | GET    | JSON with `fw`, `proto`, `began`, `milestone`, `drivers`.  |
| `/omniC6Status`     | GET    | JSON with C6 link state and last GPIO action.              |
| `/omniC6Reset`      | GET    | Pulse EN; reboot C6 into application.                      |
| `/omniC6Bootloader` | GET    | Hold BOOT low + pulse EN; leave C6 in ROM serial bootloader.|

Routes are flat camelCase per the AutomationPlatform convention. M-β.2 adds `/omniC6Ota` for the multipart firmware upload + UART flash flow.

## Companion project

The C6 firmware lives in a sibling repository: `OmniControllerC6Firmware`. Both share `OmniProto.h` (frame format and channel IDs) so they remain wire-compatible. The S3 advertises its proto version on every CTRL `hello`; mismatches surface in `/omni/status`.
