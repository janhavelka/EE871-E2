# ESP-IDF integration guide

The library is a C++17 ESP-IDF component for ESP32-S2 and ESP32-S3. The
[component manifest](../idf_component.yml) requires ESP-IDF 6.0.1 or later;
[CI](../.github/workflows/ci.yml) builds the native diagnostic example for both
targets using ESP-IDF 6.0.1. Build and hardware results belong in the
[validation matrix](EE871_E2_HARDWARE_VALIDATION_MATRIX.md); support metadata and
a configured CI job do not establish hardware qualification for an application.

## Component and application boundary

The root [CMakeLists.txt](../CMakeLists.txt) registers `src/EE871.cpp` and the
public `include/` directory and requires C++17. The core has no Arduino or
ESP-IDF dependencies. It implements E2 framing, PEC validation, bounded timing,
identity and feature checks, persistent-write verification, and driver health.

An application supplies open-drain GPIO callbacks through `EE871::Config`.
The driver never configures pins, pull-ups, peripheral handles, tasks, or locks.
EE871 E2 uses GPIO-style signaling; it cannot use the hardware I2C peripheral.

The supplied framework adapters remain example code:

| File | Purpose |
| --- | --- |
| [examples/idf/common/E2GpioTransport.h](../examples/idf/common/E2GpioTransport.h) | Native ESP-IDF GPIO, microsecond timing and cooperative post-write wait callbacks. |
| [examples/idf/basic_bringup/main/main.cpp](../examples/idf/basic_bringup/main/main.cpp) | Native `app_main` diagnostic CLI with fixed command buffers and `stdio`. |
| [examples/common/E2Transport.h](../examples/common/E2Transport.h) | Arduino GPIO adapter used by the Arduino examples. |

The IDF CLI maintains command parity with the Arduino CLI through the
[IDF contract checker](../tools/check_idf_example_contract.py). Its implementation
uses native IDF APIs and does not depend on Arduino compatibility wrappers.

## Build and flash the diagnostic example

Use an initialized ESP-IDF environment and run these commands from the
repository root:

```sh
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup flash monitor
```

For ESP32-S2, replace `esp32s3` with `esp32s2` in the first command. Select the
serial port with `-p PORT` when necessary. Changing targets reconfigures the
example build; retain any application-specific configuration before doing so.

The example project locates the library through
`EXTRA_COMPONENT_DIRS "../../.."`. Its main component declares:

```cmake
idf_component_register(
  SRCS "main.cpp"
  INCLUDE_DIRS "." "../../common"
  REQUIRES "EE871-E2" esp_driver_gpio esp_rom esp_timer freertos
)

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
```

Keep the checkout directory named `EE871-E2`, or update the `REQUIRES` entry to
match its component name. When integrating into another IDF project, add the
library directory to that project's `EXTRA_COMPONENT_DIRS` or place it under
`components/`. Declare GPIO, ROM delay, timer, and FreeRTOS dependencies in the
application component that uses them; the core component needs none of those
framework dependencies.

See the [example README](../examples/idf/basic_bringup/README.md) for wiring and
CLI behavior.

## GPIO adapter contract

The supplied adapter uses `ee871_idf::E2GpioBus` to retain the two GPIO numbers.
Call `ee871_idf::init()` and check its `esp_err_t` result before calling
`device.begin()`. `ee871_idf::makeConfig(bus, address)` creates a configuration
with the adapter callbacks; the diagnostic CLI instead wraps them for tracing.
Keep the bus context alive throughout the driver session.

- Configure both lines as `GPIO_MODE_INPUT_OUTPUT_OD`. The input buffer must
  stay enabled so `gpio_get_level()` can observe idle levels and clock stretching.
- A callback level of `false` pulls the line low; `true` releases it for the
  external pull-up. Read callbacks return the physical line level.
- Use `esp_rom_delay_us()` for the supplied `delayUs` callback. Protocol
  signaling, SCL polling and optional fixed 1 ms retry pauses stay on this path.
- The adapter configuration also installs optional `delayMs` and `yield`
  callbacks for post-write quiet waits. `ee871_idf::delayMs()` uses
  `vTaskDelay(ceil(ms * configTICK_RATE_HZ / 1000) + 1)` ticks: the extra tick
  accounts for entry just before the next tick, avoiding a short wait.
  `ee871_idf::yieldTask()` calls `taskYIELD()` after the slice. These callbacks
  never replace protocol timing or flash STOP stretching. Omitting `delayMs`
  retains one `delayUs(1000)` call per quiet-wait
  millisecond; omitting both optional callbacks retains the legacy fallback.
- External pull-ups are required for the documented electrical interface.
  `init()` disables internal pull-ups by default; its optional
  `enableInternalPullups` argument is for bench use.
- Callbacks are bounded and must not recursively call the driver. GPIO setup
  failures are reported before `begin()`; the line callbacks themselves have no
  `esp_err_t` return channel.

Use a bidirectional open-drain level shifter between 3.3 V ESP32 GPIOs and the
sensor-side E2 bus. The sensor-side pull-ups are 4.7 kOhm to 100 kOhm, with bus
high at 3.6-5.2 V (recommended 4.5-5.0 V). Keep cable length within the 10 m
guideline. Consult the [protocol reference](EE871_E2_Protocol_and_Register_Map.md) and
retained vendor documentation for the full electrical requirements.

## Timing, ownership, and recovery

Use one owner task or externally serialize every driver operation and all
other users of the physical E2 lines. The adapter provides no mutex. Retain
ownership throughout synchronous calls, including quiet waits and callbacks.
Bus operations are task-context only; persistent writes can occupy the caller
through flash stretching, commit waits and verification.

The diagnostic example uses 100 us clock-low/high times, a 25 ms per-bit
stretch limit, a 35 ms byte deadline, 150/300 ms custom/interval write waits,
and the default 350 ms flash STOP/reset allowance. Ordinary reads and volatile
pointer writes keep the ordinary STOP limit. `readNackRetries` remains zero.

Only post-write quiet waits use the optional millisecond/yield callbacks.
The default 10 ms slices preserve the configured requested total; valid slice
sizes are 1..50 ms. Delay precedes yield, including the final partial slice;
zero waits invoke neither. Protocol signaling, flash STOP stretching and the
fixed retry pause still use microsecond timing. The adapter's tick rounding
above avoids under-waiting, but scheduler and callback overhead can increase
elapsed time. Void callbacks cannot report failure or let the core verify
elapsed time; no hardware scheduler or watchdog improvement is established.

The CLI calls `device.tick(nowMs())` using `esp_timer_get_time()` and yields
between command-loop iterations with `vTaskDelay()`. It does not implement a
production sampling or automatic recovery policy. The application owns
warm-up, freshness, status-read side effects and explicit recovery. OFFLINE
remains latched until full identity/capability recovery succeeds.

The framework-independent contracts are maintained in the library README:

- [Timing and blocking](../README.md#timing-and-blocking): callback limits,
  bounded read retries and caller-owned scheduling overhead.
- [Persistent configuration writes](../README.md#persistent-configuration-writes):
  dirty-state retention, cache-only operation evidence, and coherence resync
  versus verification of the requested outcome. CLI admission remains separate
  from the core's ability to accumulate pending targets.
- [Requested-delay bounds](../README.md#programmatic-requested-delay-bounds):
  pure calculation behind `bounds`; the separate `timing` command performs
  live bus tests.

See the [HIL/CLI guide](EE871_E2_HIL_RUNNER.md) for checked-sample commands,
maintenance admission and diagnostic evidence.

## Verification

Run the framework and CLI checks from the repository root:

```sh
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
```

On Windows, use the repository's VS Code-managed PlatformIO wrapper for host
tests and Arduino builds:

```powershell
.\scripts\pio.cmd test -e native
.\scripts\pio.cmd run -e ex_bringup_s3
.\scripts\pio.cmd run -e ex_bringup_s2
```

Native IDF builds use the `idf.py` commands above or the CI matrix. Record the
commit and actual target results when qualifying a candidate. Arduino hardware
evidence does not establish that the native IDF adapter was hardware-tested.
The [validation matrix](EE871_E2_HARDWARE_VALIDATION_MATRIX.md) records completed
scenarios and the remaining gaps; the [HIL guide](EE871_E2_HIL_RUNNER.md) describes
repeatable diagnostic capture.
