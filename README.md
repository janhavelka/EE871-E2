# EE871 E2 Driver Library

Production-oriented EE871 CO2 sensor driver for the E2 bus on ESP32 using Arduino/PlatformIO or ESP-IDF.

## Features

- **E2 bus HAL injection** - no Wire/I2C dependency in library code
- **Framework-neutral core** - Arduino and ESP-IDF adapters live outside the driver
- **Health monitoring** - READY/DEGRADED/OFFLINE tracking
- **Deterministic behavior** - bounded loops, explicit timeouts
- **Managed synchronous** - blocking transfers with spec-compliant limits
- **Feature guards** - optional EE871 registers are checked from cached capability flags

## E2 Bus, Not Hardware I2C

EE871-E2 uses GPIO-style open-drain E2 signaling. The library does not use
Arduino `Wire`, ESP-IDF `driver/i2c_master`, or a hardware I2C peripheral.
Applications provide `setScl`, `setSda`, `readScl`, `readSda`, and `delayUs`
callbacks through `Config`.

`Config::deviceAddress` is the 0-7 E2 protocol address encoded into the E2
control byte. It is not an ESP-IDF or Arduino I2C device address.

## Installation

### PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps =
  https://github.com/janhavelka/EE871-E2.git
```

### Manual

Copy `include/EE871/` and `src/` to your project.

### ESP-IDF

Use this repository as an ESP-IDF component with `EXTRA_COMPONENT_DIRS` or the
metadata in `idf_component.yml`. The component builds only the framework-neutral
core. Applications own the open-drain GPIO lines and inject `setScl`, `setSda`,
`readScl`, `readSda`, and `delayUs` callbacks through `Config`. The included
ESP-IDF example uses GPIO callbacks, not `driver/i2c_master`.

## Quick Start

```cpp
#include <Arduino.h>
#include "EE871/EE871.h"

struct E2BusPins {
  int scl;
  int sda;
};

static E2BusPins bus{9, 8};

static void setScl(bool level, void* user) {
  auto* pins = static_cast<E2BusPins*>(user);
  digitalWrite(pins->scl, level ? HIGH : LOW);
}

static void setSda(bool level, void* user) {
  auto* pins = static_cast<E2BusPins*>(user);
  digitalWrite(pins->sda, level ? HIGH : LOW);
}

static bool readScl(void* user) {
  auto* pins = static_cast<E2BusPins*>(user);
  return digitalRead(pins->scl) != 0;
}

static bool readSda(void* user) {
  auto* pins = static_cast<E2BusPins*>(user);
  return digitalRead(pins->sda) != 0;
}

static void delayUs(uint32_t us, void* user) {
  (void)user;
  delayMicroseconds(us);
}

EE871::EE871 sensor;

void setup() {
  Serial.begin(115200);
  pinMode(bus.scl, OUTPUT_OPEN_DRAIN);
  pinMode(bus.sda, OUTPUT_OPEN_DRAIN);
  digitalWrite(bus.scl, HIGH);
  digitalWrite(bus.sda, HIGH);

  EE871::Config cfg;
  cfg.setScl = setScl;
  cfg.setSda = setSda;
  cfg.readScl = readScl;
  cfg.readSda = readSda;
  cfg.delayUs = delayUs;
  cfg.busUser = &bus;
  cfg.deviceAddress = 0;

  auto status = sensor.begin(cfg);
  if (!status.ok()) {
    Serial.printf("Init failed: %s\n", status.msg);
    return;
  }

  Serial.println("Device initialized");
}

void loop() {
  sensor.tick(millis());

  uint16_t ppm = 0;
  if (sensor.readCo2Average(ppm).ok()) {
    Serial.printf("CO2: %u ppm\n", ppm);
  }

  delay(1000);
}
```

## Health Monitoring

```cpp
if (sensor.state() == EE871::DriverState::OFFLINE) {
  sensor.recover();
}

Serial.printf("Failures: %u consecutive, %lu total\n",
              sensor.consecutiveFailures(),
              static_cast<unsigned long>(sensor.totalFailures()));
```

Validation and precondition errors return before E2 traffic and do not update health counters. `probe()` uses raw E2 reads and is diagnostic-only; normal reads/writes use tracked wrappers. `IN_PROGRESS` is treated as neutral for health if future scheduled operations use it.
`Config::offlineThreshold = 0` is normalized to one failed operation. Failed
`begin()` and `end()` paths clear stale runtime/cached feature state so later
diagnostics do not report old sensor capabilities.

Cache-only diagnostics are available through `SettingsSnapshot`,
`getSettings(SettingsSnapshot&)`, `getSettings()`, `isInitialized()`,
`getConfig()`, `driverState()`, `healthState()`, and `offlineThreshold()`.

## Timing And Blocking

The driver is managed synchronous: E2 transactions block for bounded protocol time, and `tick(nowMs)` only records the latest application timestamp for diagnostics. Clock stretching is bounded by `bitTimeoutUs` and `byteTimeoutUs`; flash writes are bounded by `writeDelayMs` or `intervalWriteDelayMs` with max 5000 ms validation.

The library never owns GPIO pins or an I2C/Wire instance. Applications provide `setScl`, `setSda`, `readScl`, `readSda`, and `delayUs` callbacks.

## Persistent Configuration Writes

Multi-byte persistent writes are not bus-atomic on EE871-E2. A low byte can
commit before a high byte fails, or a write can be accepted before a later
readback verify fails. If this happens, persistent sensor configuration may be
partially changed and should be treated as dirty until it is explicitly
resynced or inspected.

Use `persistentConfigDirty()` and `persistentConfigDirtyError()` to detect the
condition and retrieve the original failing `Status`. `SettingsSnapshot`
includes the same diagnostics. `resyncPersistentConfig()` re-reads the
persistent fields and clears the dirty state only after the values are readable
and coherent; unrelated successful reads do not clear it.

The bring-up CLIs expose this through safe diagnostic commands:

```text
dirty
resync
```

`dirty` prints `persistentConfigDirty`, the original dirty error status
code/detail/message, and whether resync is needed. `resync` prints dirty state
before and after calling `resyncPersistentConfig()`; it does not perform
arbitrary writes and does not clear dirty state unless the core API reports
successful verified resync. Normal safe commands such as `probe`, `status`,
`read`, `selftest`, `stress`, and `stress_mix` should not create persistent
dirty state.

Treat persistent writes such as measurement interval, part name, CO2 offset,
and CO2 gain as maintenance operations. The CLI `reg write <addr> <value>`
command can write arbitrary custom memory, including persistent/configuration
addresses, and is bench-only. These operations can have longer latency than
normal reads and may have sensor flash/endurance implications.

## Threading, ISR, And Callback Contract

`EE871::EE871` instances are not thread-safe. Use one owner task/context, or
protect all public calls with an external mutex or equivalent serialization,
including state-only accessors and `tick()`. Shared GPIO/E2 bus users must also
serialize access outside the library.

Public APIs that touch the E2 bus are blocking and are not ISR-safe because
they can perform E2 bus I/O and call the configured delay callback. Transport
callbacks must be bounded and deterministic, and must not call public methods
on the same `EE871` instance recursively.

## Main API

- Lifecycle: `begin`, `tick`, `end`
- Diagnostics: `probe`, `recover`, `resyncPersistentConfig`, `busReset`,
  `checkBusIdle`, `persistentConfigDirty`, `persistentConfigDirtyError`
- Identification: `readGroup`, `readSubgroup`, `readFirmwareVersion`, `readE2SpecVersion`
- Measurements: `readStatus`, `readCo2Fast`, `readCo2Average`, `readErrorCode`
- Custom memory/config: `customRead`, `customWrite`, `writeMeasurementInterval`, bus address, filter, operating mode, auto-adjust, calibration helpers
- Low-level command helpers: `cmd::makeControlRead`,
  `cmd::makeControlWrite`, `cmd::isReadMainCommandSupported`, and
  `cmd::co2ErrorCodeName`. Unsupported EE871 main-command reads return
  `NOT_SUPPORTED` before bus traffic.

## Examples

- `examples/01_basic_bringup_cli/` - Interactive CLI for testing
  - Status/error output decodes CO2 error-code names when the feature is
    available.
- `examples/idf/basic_bringup/` - ESP-IDF GPIO E2 diagnostic/basic bring-up CLI using
  `examples/idf/common/E2GpioTransport.h`, with the same user-visible command
  surface and diagnostics as the Arduino CLI. This example owns GPIO setup for
  bring-up and diagnostics; production applications should integrate the E2
  callbacks into their own GPIO or bus manager and externally serialize access
  if multiple tasks can touch the same `EE871` instance or E2 lines. EE871-E2
  uses GPIO-style E2 signaling, not ESP-IDF `driver/i2c_master` or hardware I2C.

## Building And Validation

```bash
pio test -e native
pio run -e ex_bringup_s3
pio run -e ex_bringup_s2
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
```

When ESP-IDF is installed, build the IDF example from
`examples/idf/basic_bringup`:

```bash
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup set-target esp32s2 build
```

Hardware validation has not been run in this hardening pass. Use
`docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` for the safe CLI recipe,
bench-only persistent-write warnings, and per-board validation matrix.

## Documentation

- `CHANGELOG.md` - full release history
- `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` - hardware validation plan and CLI recipe
- `docs/IDF_PORT.md` - ESP-IDF portability and validation guidance
- `docs/IDF_PORT_IMPLEMENTATION.md` - ESP-IDF implementation notes

## License

MIT License. See [LICENSE](LICENSE).
