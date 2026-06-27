# EE871 E2 Driver Library

Production-oriented EE871-E2 driver with framework-neutral core, injected
GPIO-style E2 transport, native fault-injection tests, Arduino and ESP-IDF
examples, and HIL validation evidence.

## Features

- **E2 bus HAL injection** - no Wire/I2C dependency in library code
- **Framework-neutral core** - Arduino and ESP-IDF adapters live outside the driver
- **Health monitoring** - READY/DEGRADED/OFFLINE tracking
- **Optional absent startup** - applications can start initialized/OFFLINE and
  recover when a probe appears
- **Checked CO2 sample helpers** - average and fast reads can validate the
  EE871 status byte without hiding raw MV3/MV4 access
- **Deterministic behavior** - bounded loops, explicit timeouts
- **Managed synchronous** - blocking transfers with spec-compliant limits
- **Cooperative long write delays** - optional millisecond delay/yield
  callbacks for maintenance-write paths
- **Feature guards** - optional EE871 registers are checked from cached capability flags
- **Dirty/resync diagnostics** - persistent multi-byte write failures are visible
- **HIL evidence tooling** - serial runner emits transcript, JSON, and Markdown reports

## Release And Validation Status

Version metadata is set to `1.1.0` for this release candidate. The driver is
production-oriented and validation-backed for the tested ESP32-S3/EE871 bench
setup, but it is not a fully field-proven driver across every physical fault
case.

Recorded evidence:

- Native tests: 49 passing in the current local 1.1.0 validation run.
- Arduino PlatformIO builds: `ex_bringup_s3` and `ex_bringup_s2` pass locally
  in the latest hardening/readiness runs.
- ESP32-S3 safe default HIL: PASS on `COM17`.
- ESP32-S3 extended safe HIL: PASS on `COM17`.
- ESP32-S3 persistent measurement interval write/readback/restore: PASS on
  `COM17`.
- Physical unplug/replug recovery: PASS, operator-confirmed manual test; no
  automated transcript is recorded.
- No 1.1.0 hardware HIL run has been recorded for absent-startup,
  checked-sample, identity-validation, or cooperative-delay behavior yet.

Remaining documented gaps:

- Pure ESP-IDF build success must be verified by GitHub Actions or local
  `idf.py` builds.
- ESP32-S2 hardware HIL and pure ESP-IDF hardware HIL are not recorded.
- Power-cycle persistence, CO2 calibration writes, bus-address write/recovery,
  and stuck-line fault-jig tests are not recorded.

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
  EE871::Co2ReadResult sample;
  if (sensor.readCo2AverageSample(sample).ok() && sample.ppmValid) {
    Serial.printf("CO2: %u ppm\n", sample.ppm);
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
`SettingsSnapshot` also reports the active `BeginPolicy`, accepted startup
probe status, and whether optional long-delay callbacks are configured.

`Config::beginPolicy = EE871::BeginPolicy::AllowAbsent` lets applications
initialize the driver into `OFFLINE` when startup identity probing fails because
the probe is absent or unreachable. Normal tracked bus APIs then return `BUSY`
without touching E2 lines until `recover()` validates identity and reloads
feature flags.

## CO2 Read Modes

Raw value APIs read exactly the selected EE871 measurement bytes:

- `readCo2Fast(uint16_t&)` reads MV3.
- `readCo2Average(uint16_t&)` reads MV4.

Checked sample APIs read the value first and the status byte second, matching
the E+E application-note sequence:

- `readCo2FastSample(Co2ReadResult&)`
- `readCo2AverageSample(Co2ReadResult&)`

The checked helpers report ppm validity, status validity, CO2 status-bit
errors, optional EE871 error code, and the individual read statuses. A sensor
status error returns `CO2_SENSOR_ERROR`; documented range validation returns
`OUT_OF_RANGE`. These sensor-level results do not count as E2 bus health
failures when all bus I/O succeeded.

`readStatus(uint8_t&)` and the checked sample helpers read the status byte.
Reading status can start or trigger an EE871 measurement and can reset the
sensor interval counter under the documented timing conditions. Use raw MV3/MV4
reads when the application intentionally does not want the checked status
sequence.

## Timing And Blocking

The driver is managed synchronous: E2 transactions block for bounded protocol time, and `tick(nowMs)` only records the latest application timestamp for diagnostics. Clock stretching is bounded by `bitTimeoutUs` and `byteTimeoutUs`; flash writes are bounded by `writeDelayMs` or `intervalWriteDelayMs` with max 5000 ms validation. Optional `Config::delayMs`, `Config::yield`, and `Config::longDelaySliceMs` apply only to millisecond write-delay paths; bit-level E2 timing still uses `delayUs` only.

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
`read`, `sampleavg`, `samplefast`, `selftest`, `stress`, and `stress_mix` should not create persistent
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
- Startup policy: `BeginPolicy::RequirePresent`, `BeginPolicy::AllowAbsent`
- Diagnostics: `probe`, `recover`, `resyncPersistentConfig`, `busReset`,
  `checkBusIdle`, `persistentConfigDirty`, `persistentConfigDirtyError`
- Identification: `readGroup`, `readSubgroup`, `readFirmwareVersion`, `readE2SpecVersion`
- Measurements: `readStatus`, `readCo2Fast`, `readCo2Average`,
  `readCo2FastSample`, `readCo2AverageSample`, `readErrorCode`
- Custom memory/config: `customRead`, `customWrite`, `writeMeasurementInterval`, bus address, filter, operating mode, auto-adjust, calibration helpers
- Low-level command helpers: `cmd::makeControlRead`,
  `cmd::makeControlWrite`, `cmd::isReadMainCommandSupported`, and
  `cmd::co2ErrorCodeName`. Unsupported EE871 main-command reads return
  `NOT_SUPPORTED` before bus traffic.

## Examples

- `examples/01_basic_bringup_cli/` - Interactive CLI for testing
  - Status/error output decodes CO2 error-code names when the feature is
    available.
  - `co2fast` and `co2avg` are raw MV3/MV4 reads; `samplefast` and `sampleavg`
    run the checked value-then-status sequence.
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

Use `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` for the current evidence
ledger, safe CLI recipe, bench-only persistent-write warnings, and remaining
per-board validation gaps.

For repeatable serial HIL evidence, build and upload the diagnostic CLI, then
run:

```bash
python tools/ee871_hil_runner.py --port COMx
python tools/ee871_hil_runner.py --port COMx --include-extended
python tools/ee871_hil_runner.py --port COMx --include-unplug-replug
python tools/ee871_hil_runner.py --port COMx --include-persistent-writes --confirm-persistent-writes
```

The default runner sequence is non-persistent and records `version`, `help`,
`probe`, `read`, `selftest`, `drv`, `dirty`, `stress 50`, final `drv`, and
final `dirty`. It writes a raw transcript, `summary.json`, and `summary.md`.
Dry-runs and operator/fault steps are never reported as hardware `PASS`.

## Documentation

- `docs/README.md` - documentation index and status map
- `CHANGELOG.md` - full release history
- `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` - hardware validation plan and CLI recipe
- `docs/EE871_E2_HIL_RUNNER.md` - automatic serial HIL runner usage and verdict rules
- `docs/IDF_PORT.md` - ESP-IDF portability and validation guidance
- `docs/IDF_PORT_IMPLEMENTATION.md` - ESP-IDF implementation notes
- `docs/EE871_E2_RELEASE_NOTES_1.1.0.md` - release notes and tagging checklist
- `docs/EE871_E2_RELEASE_NOTES_1.0.0.md` - previous release notes

## License

MIT License. See [LICENSE](LICENSE).
