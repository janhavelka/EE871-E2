# EE871-E2 ESP-IDF Port Implementation Notes

Created: 2026-05-31.
Last updated: 2026-06-02.
Branch: `hardening/ee871-e2-industry-readiness`.

Status: implementation files and CI contract are present. Pure ESP-IDF build
success is still unproven until local `idf.py` builds or GitHub Actions
`idf-build` matrix logs are recorded.

## Scope

- Kept `include/EE871/` and `src/EE871.cpp` as the framework-neutral E2 core.
- Added ESP-IDF component metadata for the core library.
- Added `examples/idf/common/E2GpioTransport.h`, an ESP-IDF GPIO open-drain
  adapter for the existing `Config` callbacks.
- Added `examples/idf/basic_bringup`, a native ESP-IDF diagnostic/basic
  bring-up CLI that owns GPIO setup for demonstration and mirrors the Arduino
  bring-up CLI command surface, formatting, diagnostics, health reporting,
  register/raw access, self-test, stress, probe, recover, and reset workflows.

## Files Added

- `CMakeLists.txt`
- `idf_component.yml`
- `examples/idf/common/E2GpioTransport.h`
- `examples/idf/basic_bringup/CMakeLists.txt`
- `examples/idf/basic_bringup/main/CMakeLists.txt`
- `examples/idf/basic_bringup/main/main.cpp`
- `examples/idf/basic_bringup/README.md`
- `tools/check_idf_example_contract.py`

## Architecture

- The driver core still does not include Arduino or ESP-IDF headers.
- GPIO ownership remains in the ESP-IDF example for demonstration. Production
  applications should integrate the E2 callbacks into their own GPIO or bus
  manager and externally serialize access if multiple tasks can touch the same
  `EE871` instance or E2 lines. The adapter configures SCL/SDA as open-drain
  outputs, releases both lines high, uses `gpio_get_level()` for reads, and
  uses `esp_rom_delay_us()` for E2 bit timing.
- EE871-E2 uses GPIO-style E2 signaling, not ESP-IDF `driver/i2c_master` or
  hardware I2C.
- External pull-ups remain the expected production configuration. The adapter
  exposes an explicit `enableInternalPullups` parameter for bench use.
- GPIO configuration failures are reported before `begin()`. The callback
  contract itself remains status-free, matching the existing public API.
- The IDF CLI uses `stdio` input/output with ANSI color sequences so the serial
  monitor prompt, help sections, status blocks, and diagnostics remain aligned
  with the Arduino example.

## Remaining Validation

- `idf.py` was not on PATH in this shell, so local native ESP-IDF builds for
  `esp32s3` and `esp32s2` remain pending.
- GitHub Actions contains the intended `idf-build` matrix, but a passing run was
  not available locally when last checked.
- Arduino ESP32-S3 safe HIL and persistent interval write/readback/restore have
  bench evidence. Pure ESP-IDF hardware HIL remains pending.
- Hardware smoke remains pending for pure ESP-IDF bus idle, line-level checks,
  `begin()`, `probe()`, status reads, CO2 reads, missing-device timeout, and
  stuck-bus behavior.
