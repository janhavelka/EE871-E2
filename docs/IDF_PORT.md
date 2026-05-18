# EE871-E2 ESP-IDF v6.0.1 Port Audit

Last audited: 2026-05-17

Scope: implementation handoff for adding first-class ESP-IDF support while
keeping the current Arduino/PlatformIO examples and public API usable. This is
not a completed port.

Official ESP-IDF references for the future implementation, verified on
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
- `library.json` and `platformio.ini` currently declare Arduino/PlatformIO
  targets only.

## ESP-IDF Readiness Verdict

Core readiness is high for GPIO and timing portability, but the managed
synchronous write path remains an acceptance item. The main missing pieces are
packaging, an ESP-IDF GPIO E2 adapter, native IDF examples/tests, and an
explicit decision that bounded millisecond write waits are acceptable for the
first IDF release. The driver should not be rewritten to call ESP-IDF GPIO
directly from the core; keep `Config` as the portability boundary and add IDF
glue outside the library core.

## Portability Blockers

1. No ESP-IDF component metadata:
   - Add `CMakeLists.txt` at repo root or under a component directory.
   - Add `idf_component.yml` if publishing through the ESP Component Registry is
     planned.
2. No ESP-IDF E2 transport adapter:
   - Arduino example glue cannot compile under pure ESP-IDF.
   - Need a GPIO open-drain adapter using `driver/gpio.h`.
3. No ESP-IDF examples:
   - Need at least one `examples/idf/basic_bringup` app with `app_main()`.
4. Example CLI is Arduino-specific:
   - Uses `Serial`, `String`, Arduino USB CDC setup, `millis()`, and
     `delayMicroseconds()`.
   - Do not try to make this one source compile under both frameworks unless a
     small platform abstraction already exists.
5. ESP-IDF v6.0.1 warnings-as-errors:
   - Keep unused parameters explicitly cast to void.
   - Avoid signed/unsigned mismatches in adapters.
   - Avoid implicit narrowing in GPIO numbers and timing conversions.
6. Write-delay scheduling:
   - Current public writes can block in bounded millisecond loops.
   - Accept this explicitly for the first IDF port or convert write completion
     into a `tick()`-driven state before claiming the driver is suitable for
     high-priority tasks.

## Files To Change

Core files to keep framework-neutral:
- `include/EE871/Config.h`
- `include/EE871/EE871.h`
- `include/EE871/Status.h`
- `include/EE871/CommandTable.h`
- `src/EE871.cpp`

New ESP-IDF support files:
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
  REQUIRES ee871-e2 esp_driver_gpio esp_timer
)
```

If the IDF adapter is promoted into the component rather than example code,
place it in a separate optional source file and add `PRIV_REQUIRES
esp_driver_gpio esp_timer`.

## ESP-IDF Example Plan

Create `examples/idf/basic_bringup`:

1. Configure target in `sdkconfig.defaults` or code constants:
   - SCL GPIO.
   - SDA GPIO.
   - E2 address 0-7.
   - timing fields using existing defaults.
2. Initialize GPIO open-drain pins.
3. Build `EE871::Config` with IDF callbacks and `busUser`.
4. Call `device.begin(cfg)` and log status with `ESP_LOGI` / `ESP_LOGE`.
5. Call `device.tick(esp_timer_get_time() / 1000ULL)` before diagnostics.
6. Read and print group/subgroup/firmware/status/CO2 values.
7. Demonstrate `probe()`, `recover()`, and `getSettings()`.

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
- `idf.py set-target esp32s3`
- `idf.py build` in `examples/idf/basic_bringup`
- Repeat for `esp32s2` if the project target matrix requires it.
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

## Ordered Implementation Checklist

1. Add root `CMakeLists.txt` for the framework-neutral core.
2. Add optional `idf_component.yml` metadata.
3. Add an IDF GPIO E2 adapter in the IDF example tree.
4. Add `examples/idf/basic_bringup` with a minimal `app_main()`.
5. Build the IDF example for ESP32-S3 and ESP32-S2.
6. Run native PlatformIO tests and both Arduino example builds.
7. Hardware-test `begin()`, `probe()`, status read, and one CO2 read.
8. Verify missing-device and stuck-bus cases remain bounded.
9. Update README only after the IDF example actually builds.
10. Add release notes and bump SemVer minor when IDF support becomes public.
