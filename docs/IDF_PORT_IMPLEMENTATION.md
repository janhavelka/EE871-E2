# EE871-E2 ESP-IDF Port Implementation Notes

Date: 2026-05-19.
Branch: `feature/ee871-e2-idf-port`.

## Scope

- Kept `include/EE871/` and `src/EE871.cpp` as the framework-neutral E2 core.
- Added ESP-IDF component metadata for the core library.
- Added `examples/idf/common/E2GpioTransport.h`, an ESP-IDF GPIO open-drain
  adapter for the existing `Config` callbacks.
- Added `examples/idf/basic_bringup`, a small native ESP-IDF example that owns
  GPIO setup, initializes the driver, reads identity fields, and polls status
  plus averaged CO2.

## Files Added

- `CMakeLists.txt`
- `idf_component.yml`
- `examples/idf/common/E2GpioTransport.h`
- `examples/idf/basic_bringup/CMakeLists.txt`
- `examples/idf/basic_bringup/main/CMakeLists.txt`
- `examples/idf/basic_bringup/main/main.cpp`
- `examples/idf/basic_bringup/README.md`

## Architecture

- The driver core still does not include Arduino or ESP-IDF headers.
- GPIO ownership remains in the ESP-IDF example. The adapter configures SCL/SDA
  as open-drain outputs, releases both lines high, uses `gpio_get_level()` for
  reads, and uses `esp_rom_delay_us()` for E2 bit timing.
- External pull-ups remain the expected production configuration. The adapter
  exposes an explicit `enableInternalPullups` parameter for bench use.
- GPIO configuration failures are reported before `begin()`. The callback
  contract itself remains status-free, matching the existing public API.

## Remaining Validation

- `idf.py` is not on PATH in this shell, so native ESP-IDF builds for `esp32s3`
  and `esp32s2` are pending.
- Hardware smoke remains pending for bus idle, line-level checks, `begin()`,
  `probe()`, status reads, CO2 reads, missing-device timeout, and stuck-bus
  behavior.
