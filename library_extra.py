"""
PlatformIO extra script for OmniController.

Adds the esp-serial-flasher submodule's include paths to the build, plus the
compile-time defines that the library expects (interface selection, MD5,
reset/boot timing). Mirrors the upstream CMakeLists.txt for the ESP32-host UART
port. The submodule lives at external/esp-serial-flasher (v1.11.0) and must be
checked out — run `git submodule update --init --recursive` once after cloning.
"""

import os

Import("env")  # noqa: F821


def _resolve_lib_dir():
    # SCons doesn't set __file__ when execing SConscripts. Use SCons' Dir()
    # to resolve the script's directory. Try srcnode first (handles
    # VariantDir-mapped builds correctly); fall back to plain abspath; last
    # resort, derive from PIO's env vars.
    try:
        return Dir(".").srcnode().abspath  # noqa: F821
    except Exception:
        pass
    try:
        return Dir(".").abspath  # noqa: F821
    except Exception:
        pass
    libdeps = env.subst("$PROJECT_LIBDEPS_DIR")  # noqa: F821
    pioenv = env.subst("$PIOENV")  # noqa: F821
    return os.path.join(libdeps, pioenv, "OmniController")


_LIB_DIR = _resolve_lib_dir()
_ESF_ROOT = os.path.join(_LIB_DIR, "external", "esp-serial-flasher")

# Sanity check: did the user init the submodule?
_SENTINEL = os.path.join(_ESF_ROOT, "include", "esp_loader.h")
if not os.path.exists(_SENTINEL):
    print("=" * 78)
    print("ERROR: OmniController's esp-serial-flasher submodule is not initialised.")
    print(f"  Looked for:       {_SENTINEL}")
    print(f"  Resolved LIB_DIR: {_LIB_DIR}")
    print("  Fix this once:")
    print("    cd Libraries/OmniController")
    print("    git submodule update --init --recursive")
    print("  (After that, retry the build.)")
    print("=" * 78)
    raise FileNotFoundError(
        "esp-serial-flasher submodule is missing — see message above"
    )

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
        ("SERIAL_FLASHER_RESET_INVERT", "false"),
        ("SERIAL_FLASHER_BOOT_INVERT", "false"),
    ],
)
