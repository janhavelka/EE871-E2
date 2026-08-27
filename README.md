# EE871 E2 Driver Library

Production-oriented EE871-E2 driver with framework-neutral core, injected
GPIO-style E2 transport, native fault-injection tests, Arduino and ESP-IDF
examples, and HIL validation evidence.

## Features

- **E2 bus HAL injection** - no Wire/I2C dependency in library code
- **Framework-neutral core** - Arduino and ESP-IDF adapters live outside the driver
- **Health monitoring** - READY/DEGRADED/OFFLINE tracking
- **Deterministic behavior** - bounded loops, explicit timeouts
- **Managed synchronous** - blocking transfers with spec-compliant limits
- **Feature guards** - optional EE871 registers are checked from cached capability flags
- **Dirty/resync diagnostics** - persistent multi-byte write failures are visible
- **HIL evidence tooling** - serial runner emits transcript, JSON, and Markdown reports

## Release And Validation Status

The current source/package version is `1.0.1`. The driver is
production-oriented and validation-backed for the recorded ESP32-S3/EE871
bench setup. See `CHANGELOG.md` for release notes.

Recorded evidence:

- Native tests: 35 passing; consolidated HIL-runner/parser tests: 40 passing.
- The current example/HIL platform is exact-pinned to pioarduino
  `platform-espressif32` `55.03.311`, Arduino-ESP32 `3.3.11`, and ESP-IDF
  `5.5.5`. The earlier TunnelMonitor-node parity work on pioarduino
  `54.03.20` / Arduino-ESP32 `3.2.0` remains recorded as historical evidence.
- Current ESP32-S3 COM20 `55.03.311` HIL: 184/184 PASS from clean firmware
  commit `3bce89e`, covering safe, extended, identity/capability, range guards,
  GPIO/E2 diagnostics, trace/sniffer, repeated reads, and mixed stress. It
  finished READY with 3,109 tracked successes, zero transport failures, clean
  persistent state, `stress 500` at 500/500, and `stress_mix 500` at 500/500.
  The self-test reported 26 PASS / 0 FAIL / 1 unsupported-mode SKIP.
- Native-USB reattachment on the current COM20 stack is verified. The former
  timeout was a host-tool framing bug: it sent only a blank line, while the CLI
  intentionally ignores blank lines and therefore emitted no new prompt. The
  shared explicit `\ndirty\n` synchronization passed 10,000/10,000 replies
  from a new process immediately after the full HIL closed COM20,
  100/100 separate process open/close/reopen sessions, and an immediate
  184/184 full HIL rerun without a reset or physical replug.
- The current COM20 target was detected as ESP32-S3 revision 0.2 with 4 MB
  embedded flash and 2 MB embedded QSPI PSRAM; the S3 PlatformIO environment
  configures that QSPI PSRAM explicitly. The prior `55.03.39` firmware reported
  4,194,304 bytes flash and PSRAM ready with 2,097,152 bytes.
- Prior ESP32-S3 COM20 targeted `55.03.39` HIL: 144/144 PASS, including final READY
  state, zero transport failures, clean persistent state, and `stress 500`
  at 500/500. The flashed library identified itself as `1.0.1`.
- Prior ESP32-S3 COM20 serial-only discriminator on `55.03.39`: 10,000/10,000
  `dirty` command round trips passed, and every reply was the same 201 bytes.
  `dirty` does not touch the E2 bus, so this result qualifies CLI framing only.
- Prior ESP32-S3 COM20 accelerated scheduled-read regression on `55.03.39`: PASS
  over 108 sample cycles and 543.594 s, with 564 ordinary command passes, two
  fully framed control-byte NACK attempts recovered by one harness retry after
  1,500 ms, and zero hard failures, reviews, skips, reconnects, or counter
  regressions. Final state was READY and persistent state was clean. This is a
  targeted timing/policy regression, not a completed long soak.
- Prior ESP32-S3 COM20 unsupported operating-mode guard: PASS. `mode` returned
  `NOT_SUPPORTED`, did not decode the stale `0x55` memory value, and left
  tracked transport counters unchanged at 3,908 successes / 2 failures.
- Historical ESP32-S3 COM20 safe plus extended HIL on `54.03.20`: 33/33 PASS,
  including `selftest`
  27/27, repeated reads/recovery, `stress 50` 50/50, and `stress 500` 500/500.
- Historical ESP32-S3 COM20 same-value persistent write/readback HIL: 25/25
  PASS for interval `150 ds`, CO2 offset `0 ppm`, and CO2 gain `32768`; dirty
  state remained clean.
- Historical ESP32-S3 COM20 niche diagnostics: capability/range guards 11/11 PASS,
  trace/sniffer/`stress_mix 500` 11/11 PASS, address 0 found by the full scan,
  and all six in-spec timing points plus two out-of-spec characterization
  points responded.
- Historical `54.03.20` ESP32-S3 COM20 operator-assisted physical HIL:
  absent-sensor boot, hot
  unplug/OFFLINE/replug recovery, SDA stuck-low, SCL stuck-low timeout, and
  a complete sensor/MCU power cycle with measurement-interval persistence all
  PASS. The temporary measurement interval was restored from `160 ds` to its
  `150 ds` baseline.
- Historical `54.03.20` ESP32-S3 COM20 immediate warm-up HIL: PASS. Sampling
  began 0.250 s after COM20 reappeared; MV3/MV4 were `0 ppm` with status `0x08`
  through 4 s, then `678 ppm` with status `0x00` at 5 s. All 33 scheduled CLI
  commands were framed correctly; final health was READY with 65 transport
  successes, zero failures, clean persistent state, and interval `150 ds`. A
  separate delayed-start attempt recorded one bounded fast-read `NACK` at
  20.094 s and recovered immediately; it remains in the attempt ledger.
- The historical `54.03.20` strict 10-minute post-power-cycle stability
  capture recorded one bounded
  `NACK` at t=330 s and therefore remains FAIL under its zero-error criterion;
  the other 62 scheduled CLI commands succeeded. Manually normalized
  interactive output from immediate `stress_mix 1000` and `stress 1000`
  follow-ups recorded 1000/1000 for each, so the transient was not reproduced.
- The historical eight-hour `54.03.20` soak is also a strict FAIL: 2,376
  commands passed, 29 replies stalled mid-line in Arduino-ESP32 3.2.0 HWCDC,
  and 11 scheduled MV3 reads received a real NACK on the `0xC1` control byte.
  Those NACKs clustered at the same measurement phase; the mixed stress blocks
  otherwise passed. The old runner labeled the 11 NACKs as review-required
  because it checked for the success-value token before the parsed status.
  They are retained as NACK failures, not reclassified as passes.
- The HWCDC truncations match the upstream lost-wakeup defect fixed by
  [Arduino-ESP32 PR #12606](https://github.com/espressif/arduino-esp32/pull/12606)
  in 3.3.9. The 10,000-round-trip discriminator validates that targeted fix on
  this bench, but it is not a replacement for a completed long soak.
- ESP32-S3 safe default HIL: PASS on `COM17`.
- ESP32-S3 extended safe HIL: PASS on `COM17`.
- ESP32-S3 persistent measurement interval write/readback/restore: PASS on
  `COM17`.
- Historical COM17 physical unplug/replug recovery: PASS, operator-confirmed
  manual test; no automated transcript is recorded for that historical run.

The compact HIL evidence ledger in `hil_results/README.md` identifies the exact
firmware/build metadata. Results from different platform stacks are not
combined into one platform claim. The new
`55.03.311` pin has passed S2/S3 builds and the post-commit COM20 run above.
No completed long-soak result is currently claimed for this pin.

## E2 Bus, Not Hardware I2C

EE871-E2 uses GPIO-style open-drain E2 signaling. The library does not use
Arduino `Wire`, ESP-IDF `driver/i2c_master`, or a hardware I2C peripheral.
Applications provide `setScl`, `setSda`, `readScl`, `readSda`, and `delayUs`
callbacks through `Config`.

Provide external 4.7 kOhm to 100 kOhm pull-ups to a 3.6-5.2 V E2 bus supply.
Use a bidirectional open-drain level shifter between that bus and a 3.3 V
ESP32; do not rely on direct 5 V connection to ESP32 GPIO.

`Config::deviceAddress` is the 0-7 E2 protocol address encoded into the E2
control byte. It is not an ESP-IDF or Arduino I2C device address.

## Installation

### PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps =
  https://github.com/janhavelka/EE871-E2.git#v1.0.1
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
  delay(100);
}
```

This quick start proves transport setup and identity only. `readCo2Fast()` and
`readCo2Average()` are raw MV3/MV4 reads: they do not check status, warm-up,
freshness, or the product-specific valid ppm range. A sampling application
should wait for its warm-up policy, read the selected measured value first,
read `readStatus()` second, reject status bit 3 via `hasCo2Error()`, and apply
its own range/staleness policy. Reading status can itself trigger the next
measurement and reset the sensor interval counter.

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

The repository's Arduino example environments exact-pin pioarduino
`platform-espressif32` `55.03.311`, which supplies Arduino-ESP32 `3.3.11` and
ESP-IDF `5.5.5`. This supersedes the earlier TunnelMonitor-node parity pin
`54.03.20` / Arduino-ESP32 `3.2.0` for repository examples and HIL because
3.3.11 retains the upstream HWCDC lost-wakeup fix introduced in 3.3.9 and
includes later framework fixes. Historical `54.03.20` and `55.03.39` evidence
remains labeled with its exact stack.
Consuming applications retain control of their own platform pin; the
framework-neutral EE871 core does not depend on Arduino USB.
The S3 flash/PSRAM settings in this repository describe the tested 4 MB flash /
2 MB QSPI-PSRAM board; override them for a different ESP32-S3 module.

`compat_tunnelmonitor_s3` is a build-only compatibility environment pinned to
TunnelMonitor-node commit `0f240ab` and its older `54.03.20` stack. The current
`1.0.1` source builds under both pins without compatibility shims.
TunnelMonitor-node's production console uses ESP-IDF USB Serial/JTAG APIs
directly, so the Arduino HWCDC bug does not apply to that console path.

```bash
pio test -e native
pio run -e ex_bringup_s3
pio run -e ex_bringup_s2
pio run -e compat_tunnelmonitor_s3
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m unittest discover -s test -p "test_hil_runner_parser.py"
doxygen Doxyfile
```

GitHub Actions builds the native ESP-IDF example for ESP32-S3 and ESP32-S2 on
ESP-IDF v6.0.1. To reproduce those builds locally from
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
python tools/ee871_hil_runner.py --port COMx --include-niche
python tools/ee871_hil_runner.py --port COMx --include-unplug-replug
python tools/ee871_hil_runner.py --port COMx --include-persistent-writes --confirm-persistent-writes
```

The default runner sequence is non-persistent and records `version`, `help`,
`probe`, `read`, `selftest`, `drv`, `dirty`, `stress 50`, final `drv`, and
final `dirty`. It writes a raw transcript, `summary.json`, and `summary.md`.
The optional non-persistent niche plan adds identity/capability, parameter
guard, GPIO/E2 diagnostics, trace/sniffer, and mixed-stress coverage.
Dry-runs never report hardware `PASS`; acknowledging an operator prompt alone
is review-required and is not automatically promoted to PASS. Separately
reviewed manual fault evidence can be recorded in the hardware matrix.

The core library does not retry a failed E2 transfer. The separate soak harness
may retry only a scheduled MV3/MV4 control-byte `NACK` once after a configurable
delay (default 1,500 ms) and records the original attempt, retry, and
`SCHEDULED_CONTROL_NACK_RECOVERED` outcome. This is application-level cadence
policy, not hidden driver behavior. The result name deliberately does not infer
why the sensor NACKed.

## Documentation

- `docs/README.md` - documentation index and status map
- `CHANGELOG.md` - full release history
- `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` - hardware validation plan and CLI recipe
- `docs/EE871_E2_HIL_RUNNER.md` - automatic serial HIL runner usage and verdict rules
- `docs/IDF_PORT.md` - ESP-IDF portability and validation guidance
- `docs/IDF_PORT_IMPLEMENTATION.md` - ESP-IDF implementation notes

## License

MIT License. See [LICENSE](LICENSE).
