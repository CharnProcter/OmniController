"""
PlatformIO extra script for OmniController.

Adds the esp-serial-flasher submodule's include paths to the build, plus the
compile-time defines that the library expects (interface selection, MD5,
reset/boot timing). Mirrors the upstream CMakeLists.txt for the ESP32-host UART
port. The submodule lives at external/esp-serial-flasher (v1.11.0) and must be
checked out — run `git submodule update --init` once after cloning.
"""

import os

Import("env")  # noqa: F821

_LIB_DIR = os.path.dirname(os.path.realpath(__file__))
_ESF_ROOT = os.path.join(_LIB_DIR, "external", "esp-serial-flasher")

env.Append(  # noqa: F821
    CPPPATH=[
        os.path.join(_ESF_ROOT, "include"),
        os.path.join(_ESF_ROOT, "port"),
        os.path.join(_ESF_ROOT, "private_include"),
    ],
    CPPDEFINES=[
        "SERIAL_FLASHER_INTERFACE_UART",
        ("MD5_ENABLED", "1"),
        ("SERIAL_FLASHER_RESET_HOLD_TIME_MS", "100"),
        ("SERIAL_FLASHER_BOOT_HOLD_TIME_MS", "50"),
        ("SERIAL_FLASHER_WRITE_BLOCK_RETRIES", "3"),
        # Active-low signalling — direct GPIO drive, no inverting transistors on the hat.
        ("SERIAL_FLASHER_RESET_INVERT", "false"),
        ("SERIAL_FLASHER_BOOT_INVERT", "false"),
    ],
)
