# EE871-E2 ESP-IDF v6.0.1 Port Guide

Last audited: 2026-06-02

Scope: first-class ESP-IDF support while keeping the Arduino/PlatformIO example
and public driver core usable. The framework-neutral driver core is shared; the
Arduino and ESP-IDF applications provide their own E2 GPIO adapters and CLIs.

Official ESP-IDF references used for the port guidance, verified on
2026-05-17 when the stable docs identify as v6.0.1:
- GPIO: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/gpio.html
- High resolution timer: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/esp_timer.html
- Build system: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html
- ESP-IDF 6.0 migration guide: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/migration-guides/release-6.x/6.0/index.html

## Current State

- Core library files are `include/EE871/*.h` and `src/EE871.cpp`.
- The reusable driver is already transport-callback based. `EE871::Config`
  accepts `setScl`, `setSda`, `readScl`, `readSda`, `delayUs`, and `busUser`.
- The core does not include `Arduino.h`, `Wire.h`, ESP-IDF headers, or own GPIO
  pins. It performs bit-banged E2 bus transactions through callbacks.
- Public lifecycle is `begin(const Config&)`, `tick(uint32_t nowMs)`, `end()`,
  `probe()`, and `recover()`.
- The current implementation is still managed synchronous: public E2 reads and
  writes block inside bounded callback-driven operations.
- Timing is callback owned. The driver validates E2 timing fields and caps flash
  write delays with `WRITE_DELAY_MAX_MS`.
- Arduino dependencies are isolated in examples and test stubs:
  - `examples/common/E2Transport.h` uses `pinMode`, `digitalWrite`,
    `digitalRead`, and `delayMicroseconds`.
  - `examples/01_basic_bringup_cli/main.cpp` and `examples/common/*.h` use
    `Serial`, `String`, `millis()`, and Arduino command helpers.
  - native tests use stubs under `test/stubs/`.
- `library.json` advertises Arduino and ESP-IDF framework support.
- `platformio.ini` owns Arduino example and native-test builds; ESP-IDF builds
  use the root `CMakeLists.txt`, `idf_component.yml`, and the
  `examples/idf/basic_bringup` CMake project.
- `.github/workflows/ci.yml` includes an `idf-build` matrix job for `esp32s3`
  and `esp32s2` and runs `tools/check_idf_example_contract.py` before the IDF
  action build.
- Local pure `idf.py` builds are still unproven in this workspace because
  `idf.py` was not available when checked. No GitHub Actions pass record was
  available locally.

## ESP-IDF Readiness Verdict

The core is structured for ESP-IDF component builds and remains
framework-neutral. ESP-IDF packaging, the GPIO E2 adapter, and an interactive
native IDF bring-up CLI are present. CI coverage for pure IDF builds is
configured, but acceptance still requires evidence that the CI matrix or local
`idf.py` builds passed for ESP32-S2 and ESP32-S3.

Hardware evidence currently exists for the Arduino ESP32-S3 diagnostic CLI, not
for pure ESP-IDF hardware HIL. Missing-device and stuck-bus behavior still need
bench or jig validation.

The driver should not be rewritten to call ESP-IDF GPIO or I2C APIs directly
from the core. EE871 E2 is represented by bit-level open-drain callbacks in
`Config`; IDF platform code lives in examples or application glue.

## Remaining Portability Risks

1. ESP-IDF v6.0.1 warnings-as-errors:
   - Keep unused parameters explicitly cast to void.
   - Avoid signed/unsigned mismatches in adapters.
   - Avoid implicit narrowing in GPIO numbers and timing conversions.
2. Write-delay scheduling:
   - Current public writes can block in bounded millisecond loops.
   - Accept this explicitly for the first IDF port or convert write completion
     into a `tick()`-driven state before claiming the driver is suitable for
     high-priority tasks.
3. Pure ESP-IDF build proof:
   - Component metadata and the native IDF example are present.
   - CI is configured for the `esp32s3` and `esp32s2` matrix.
   - Successful `idf.py` builds or GitHub Actions logs still need to be captured
     before claiming validated ESP-IDF build support.
4. Hardware validation:
   - Arduino ESP32-S3 safe and persistent interval HIL evidence is recorded in
     `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md`.
   - Pure ESP-IDF hardware HIL, bus timing, pull-up behavior, clock stretching,
     and recovery still require bench validation.

## Files To Change

Core files to keep framework-neutral:
- `include/EE871/Config.h`
- `include/EE871/EE871.h`
- `include/EE871/Status.h`
- `include/EE871/CommandTable.h`
- `src/EE871.cpp`

ESP-IDF support files:
- `CMakeLists.txt`
- `idf_component.yml` optional but recommended
- `examples/idf/basic_bringup/CMakeLists.txt`
- `examples/idf/basic_bringup/main/CMakeLists.txt`
- `examples/idf/basic_bringup/main/main.cpp`
- `examples/idf/common/E2GpioTransport.h` or component-local
  `examples/idf/basic_bringup/main/E2GpioTransport.h`

Files that should remain Arduino-only:
- `examples/01_basic_bringup_cli/main.cpp`
- `examples/common/E2Transport.h`
- `examples/common/Log.h`
- `examples/common/CliShell.h`
- `examples/common/E2Diagnostics.h`

## Recommended Architecture

Keep one core driver and two framework adapters:

```text
include/EE871/*.h + src/EE871.cpp
  framework-neutral, callback-driven E2 protocol and health state

examples/common/E2Transport.h
  Arduino GPIO adapter for existing examples

examples/idf/common/E2GpioTransport.h
  ESP-IDF GPIO adapter for IDF examples
```

The IDF adapter should own only application glue:
- GPIO initialization.
- Open-drain SCL/SDA drive/release.
- Line reads.
- Microsecond delay.
- Optional mutex when multiple tasks can call the driver.

The library core should continue to own:
- E2 command sequencing.
- Bounded clock-stretch waits.
- PEC calculation and validation.
- Feature-cache and health tracking.
- Validation of timing and device address.

## IDF Transport Adapter Contract

Suggested adapter state:

```cpp
struct Ee871IdfBus {
  gpio_num_t scl;
  gpio_num_t sda;
};
```

Suggested callback behavior:

```cpp
static void set_scl(bool level, void* user);
static void set_sda(bool level, void* user);
static bool read_scl(void* user);
static bool read_sda(void* user);
static void delay_us(uint32_t us, void* user);
```

Implementation notes:
- Configure SCL/SDA as open-drain GPIO with pull-ups disabled unless the board
  intentionally uses internal weak pull-ups for bench testing.
- Driving low: set output level 0.
- Releasing high: set output level 1 on open-drain output.
- Reads: call `gpio_get_level()` and return true for high.
- Delays: use `esp_rom_delay_us()` for sub-millisecond bit timing. For
  multi-millisecond write delays, the current core calls `delayUs(1000)` in a
  loop; the IDF callback may still use `esp_rom_delay_us(1000)`, but a later
  nonblocking write-delay state machine would be better if field usage exposes
  watchdog pressure.
- If the adapter is shared across tasks, wrap public driver calls with an
  application mutex. Do not put a mutex inside the core unless the API contract
  is explicitly changed.

Status mapping:
- Callback layer has no `esp_err_t` return path today. GPIO configuration errors
  should be caught before `begin()` and reported by the example before calling
  the driver.
- If future callbacks gain status returns, map invalid GPIO config to
  `INVALID_CONFIG`, stuck lines to `BUS_STUCK`, and timeout-like waits to
  `TIMEOUT`.

## Component Layout

Minimal root `CMakeLists.txt`:

```cmake
idf_component_register(
  SRCS "src/EE871.cpp"
  INCLUDE_DIRS "include"
  REQUIRES
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
```

The core component itself should not require `esp_driver_gpio` because it does
not include ESP-IDF headers. IDF examples that include the GPIO adapter should
declare:

```cmake
idf_component_register(
  SRCS "main.cpp"
  INCLUDE_DIRS "." "../../common"
  REQUIRES "EE871-E2" esp_driver_gpio esp_rom esp_timer freertos
)
```

If the IDF adapter is promoted into the component rather than example code,
place it in a separate optional source file and add `PRIV_REQUIRES
esp_driver_gpio esp_rom esp_timer`.

## ESP-IDF Example Behavior

`examples/idf/basic_bringup` provides a diagnostic/basic bring-up interactive
serial CLI equivalent to `examples/01_basic_bringup_cli`. It owns the E2 GPIO
lines for demonstration; production applications should integrate the callbacks
into their own GPIO or bus manager and externally serialize access if multiple
tasks can touch the same `EE871` instance or E2 lines. EE871-E2 uses GPIO-style
E2 signaling, not ESP-IDF `driver/i2c_master` or hardware I2C.

1. Configure target in code constants:
   - SCL GPIO.
   - SDA GPIO.
   - E2 address 0-7.
   - E2 timing fields using the same example defaults as Arduino.
2. Initialize GPIO open-drain pins through `examples/idf/common/E2GpioTransport.h`.
3. Build `EE871::Config` with IDF callbacks and `busUser`.
4. Call `device.begin(cfg)` and keep the CLI available even when the device is
   absent, so diagnostics can still run.
5. Call `device.tick(esp_timer_get_time() / 1000ULL)` in the command loop.
6. Provide the same help sections, command names and aliases, colors, prompt,
   status formatting, diagnostics, health/error reporting, probe/recover/reset,
   self-test/stress workflows, capabilities, and register/raw access as the
   Arduino CLI.

Keep the example deterministic:
- No infinite retry loops.
- Use bounded command attempts.
- Avoid heap-backed command parsing in first IDF example.

## Arduino Compatibility Plan

- Do not remove the existing Arduino example.
- Keep `examples/common/E2Transport.h` as the Arduino GPIO adapter.
- Do not add ESP-IDF includes to public headers.
- If adding helper adapter headers, keep framework-specific names explicit:
  `E2TransportArduino.h` and `E2TransportIdf.h`.
- Keep native tests using stubs, but add an IDF compile test separately.

## Test And Validation Plan

Host/native:
- `python -m platformio test -e native`
- Keep native tests for validation order, status helpers, and unsupported
  command prevalidation.

Arduino:
- `python -m platformio run -e ex_bringup_s3`
- `python -m platformio run -e ex_bringup_s2`

ESP-IDF:
- `idf.py -C examples/idf/basic_bringup set-target esp32s3 build`
- `idf.py -C examples/idf/basic_bringup set-target esp32s2 build`
- If local `idf.py` is unavailable, record the latest GitHub Actions
  `idf-build` matrix result instead.
- Hardware smoke:
  - Bus idle SCL/SDA high before begin.
  - `begin()` succeeds against a known EE871 device.
  - `probe()` succeeds without changing health counters.
  - Deliberate missing-device test returns a non-OK `Status` and does not hang.

Static checks:
- Build with default ESP-IDF v6.0.1 warnings-as-errors.
- Search core for accidental framework use:
  - `Arduino.h`
  - `Serial`
  - `String`
  - `Wire`
  - `gpio_`
  - `esp_`

## ESP-IDF v6.0.1 Hazards

- Deprecated legacy peripheral APIs have been removed in ESP-IDF 6. Use current
  GPIO APIs and avoid legacy RMT/timer APIs in examples.
- Default compiler warnings are treated as errors. Write adapter code with clean
  casts and no unused parameters.
- Do not use `ets_delay_us()` directly in new code; prefer the supported ROM
  delay include used by current IDF examples if microsecond busy waits are
  unavoidable.
- IDF logging format checks are stricter than Arduino `Serial.printf`; cast
  `uint32_t`/`size_t` values deliberately.
- GPIO open-drain behavior depends on external pull-ups. Document board-level
  pull-up requirements in the IDF example README.
- Long synchronous flash-write waits can still block the caller. The current
  design allows it, but do not call write-heavy operations from high-priority
  timing tasks.

## Ordered Validation Checklist

1. Run `python tools/check_core_timing_guard.py`.
2. Run `python tools/check_cli_contract.py`.
3. Run `python tools/check_idf_example_contract.py`.
4. Run `python -m platformio test -e native`.
5. Run `python -m platformio run -e ex_bringup_s3`.
6. Run `python -m platformio run -e ex_bringup_s2`.
7. Build the IDF example for ESP32-S3 and ESP32-S2 with `idf.py` when ESP-IDF
   is available, or record the passing GitHub Actions `idf-build` matrix.
8. Hardware-test `begin()`, `probe()`, status read, CO2 reads, bus diagnostics,
   self-test/stress workflows, missing-device timeout, and stuck-bus recovery.
