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
| [examples/idf/common/E2GpioTransport.h](../examples/idf/common/E2GpioTransport.h) | Native ESP-IDF GPIO and microsecond delay callbacks. |
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
- Use `esp_rom_delay_us()` for the supplied `delayUs` callback. It also handles
  the core's repeated 1 ms write-wait and optional retry pauses.
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

Public bus operations are synchronous, bounded, and task-context only.
Use one owner task or externally serialize every driver operation and all
other users of the physical E2 lines. The adapter does not provide a mutex.
Persistent writes can occupy the caller through flash stretching, commit waits,
and verification; include that duration in application scheduling and watchdog
budgets.

The diagnostic example uses 100 us clock-low/high times, a 25 ms per-bit
stretch limit, a 35 ms byte deadline, a 150 ms custom-write wait, and a 300 ms
interval-pair wait. It inherits `Config::flashStretchTimeoutUs = 350000` for
flash-write STOP and explicit bus reset. Ordinary reads and volatile custom-read
pointer writes use `bitTimeoutUs` for STOP. The flash allowance does not extend
ordinary bit or byte transfer limits.

`readNackRetries` defaults to zero, including in this example. An application
may explicitly allow up to three additional MV3/MV4/status attempts after a
control-byte NACK, successful STOP, idle-line checks, and a fixed 1 ms HAL pause.
Identity reads, custom reads, writes, PEC failures, and timeouts are never
retried. The optional `allowReadRetry` callback can veto attempts for a deadline,
cancellation, or a latched adapter error. See
[Config.h](../include/EE871/Config.h) for the callback contract.

The CLI calls `device.tick(nowMs())` using `esp_timer_get_time()` and yields
between command-loop iterations with `vTaskDelay()`. The application owns
sampling cadence, warm-up and freshness policy, and explicit recovery. An
OFFLINE session stays latched until `recover()` succeeds; ordinary bus
operations return the latched error. `probe()` is health-neutral diagnostics.
Status reads can trigger a measurement under the sensor's documented timing
conditions, so account for that side effect in a sampling design.

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
