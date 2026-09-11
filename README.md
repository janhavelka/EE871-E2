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
- **Feature guards** - cached operating capabilities and on-demand calibration checks
- **Dirty/resync diagnostics** - uncertain single- and multi-byte writes are visible
- **HIL evidence tooling** - serial runner emits transcript, JSON, and Markdown reports

## Release And Validation Status

Source/package version `1.1.0` is available from the `v1.1.0` Git tag.
[CHANGELOG.md](CHANGELOG.md) covers migration, stricter startup/recovery,
ordinary STOP timing, opt-in control-NACK retries, calibration guards, and
persistent-write uncertainty.

Software verification of implementation commit `3d32ac3` on 2026-09-11 passed
93 native tests, 58 Python tests, the timing/CLI/IDF contracts, synchronized
version metadata, Doxygen, and all three local Arduino builds (ESP32-S3,
ESP32-S2, and the older TunnelMonitor compatibility stack).
[All six CI jobs passed](https://github.com/janhavelka/EE871-E2/actions/runs/34605633301),
including native ESP-IDF 6.0.1 builds for S2 and S3. These are software checks;
hardware results apply only to the revisions and scenarios recorded below.

A September 11 targeted COM11 run tested current library source `32dfb06`:
407 real sensor/API assertions passed, including calibration-support discovery,
typed reads, six timing configurations, 100 stress reads, and 61 scheduled
MV3/MV4/status cycles. There were no natural NACKs or transport failures in
those real-operation sessions. Retry recovery/exhaustion, OFFLINE behavior,
vetoes, PEC/timeout rejection and cleanup exclusions passed controlled HAL
tests. Follow-ups isolated an unexpected response after a custom-pointer
update: wrong-address frames produced an invalid PEC instead of NACK, which
the driver safely rejected. The final context comparison passed 303/303
assertions, including 12 custom-read NACKs without retries. The original failed
harness expectations remain recorded. All 93 fake-transport regressions also
passed on the ESP32, separately from real sensor I/O.

Recorded CO2Control ESP32-S3 hardware testing used the exact library commit
`a358f92`: a ten-minute stress run completed 562 successful owner operations;
a thirty-minute normal-acquisition run completed 120 successful readings,
including one real control-byte NACK recovered by its first retry without a
sensor fault or qualification loss. The subsequent five-minute sensor checks
also passed, but the overall follow-up harness verdict remains incomplete
because Web-session cleanup returned HTTP 403.

These observations establish one hardware retry recovery. They do not establish
a long-term fault rate or the physical cause of the NACK. Retry exhaustion and
non-NACK fault behavior were not forced in that earlier campaign; the newer
targeted run adds controlled HAL-injection evidence. Persistent write failures
and supported auto-adjust operations still have fake-transport coverage only.
The targeted run made no persistent sensor writes, verified 30 unchanged
registers, and restored the original healthy production firmware.
ESP32-S2 and native ESP-IDF hardware remain untested, and no completed long soak
is claimed for the current candidate. The
[validation matrix](docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md) records exact
images, evidence sources, historical results, and remaining coverage gaps.

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

Pin version `1.1.0` in `platformio.ini`:

```ini
lib_deps =
  https://github.com/janhavelka/EE871-E2.git#v1.1.0
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

This quick start checks transport setup, the EE871 group/subgroup, CO2
capability, and a complete, valid feature-cache read. `readCo2Fast()` and
`readCo2Average()` are raw MV3/MV4 reads: they do not check status, warm-up,
freshness, or the product-specific valid ppm range. A sampling application
should wait for its warm-up policy, read the selected measured value first,
read `readStatus()` second, reject status bit 3 via `hasCo2Error()`, and apply
its own range/staleness policy. Reading status can trigger the next measurement
and reset the interval counter only when the global interval exceeds 15 s and
the previous value is older than 10 s (AN1611-1 sections 4 and 10).

`begin()` and `recover()` reject reserved bits in feature bytes `0x07..0x09`
before installing the cache. `readOperatingMode()` rejects active mode bits
whose capability is absent. `readAutoAdjustStatus()` requires advertised
auto-adjust support and rejects reserved result bits instead of decoding
unsupported register replies as a running adjustment.

Offset/gain helpers first check CO2 support in `0x03`; calibration-point reads
check `0x04`. Each call adds one volatile pointer update and one byte read.
Missing or malformed support returns `NOT_SUPPORTED` before calibration access;
transfer failures retain their precise status. Raw `customRead()` remains
available for register diagnostics. These checks leave normal sampling and
startup/recovery transaction counts unchanged.

## Health Monitoring

```cpp
if (sensor.state() == EE871::DriverState::OFFLINE) {
  sensor.recover();
}

Serial.printf("Failures: %u consecutive, %lu total\n",
              sensor.consecutiveFailures(),
              static_cast<unsigned long>(sensor.totalFailures()));
```

Parameter validation and cache-only precondition checks return before E2 traffic.
During ordinary reads, validating returned metadata adds no health event;
preceding transfers remain tracked. `probe()` uses raw E2 reads and is diagnostic-only; normal
reads/writes use tracked wrappers. Health counts tracked bus transfers rather
than application sampling cycles: a failed low byte short-circuits a 16-bit
read before its high byte is attempted. `BUSY` means auto adjustment is already
running and must become idle before the requested operation. `IN_PROGRESS`
remains reserved and neutral for health.
`Config::offlineThreshold = 0` is normalized to one failed operation. Failed
`begin()` and `end()` paths clear stale runtime/cached feature state so later
diagnostics do not report old sensor capabilities.

OFFLINE is latched. Ordinary bus reads and writes return the last precise
tracked failure immediately, without touching the E2 lines or changing health
counters. Such replies retain the error code and detail but use the message
`Driver offline; call recover()` to distinguish them from fresh bus failures.
`lastError()` and the health snapshot retain the original message and timestamp.
Parameter and capability guards still run first and may return their own errors.
`probe()` and `busReset()` may still inspect/reset the bus without changing health
or clearing the latch; `resyncPersistentConfig()` is blocked until recovery.
`recover()` is the only path back to READY: it performs a bounded reset and
validates group, subgroup, and CO2 capability, atomically refreshes feature
flags, then records the complete recovery as one health event.
A failed recovery clears all cached capabilities and latches OFFLINE even if
the driver was READY or DEGRADED. A later ordinary read cannot bypass recovery.

Cache-only diagnostics are available through `SettingsSnapshot`,
`getSettings(SettingsSnapshot&)`, `getSettings()`, `isInitialized()`,
`getConfig()`, `driverState()`, `healthState()`, and `offlineThreshold()`.

## Timing And Blocking

The driver is managed synchronous: E2 transactions block for bounded protocol
time, and `tick(nowMs)` only records the latest application timestamp for
diagnostics. `begin()` validates the generated bit period
(`10 + clockLowUs + clockHighUs`) against the 500 Hz minimum and requires
`byteTimeoutUs` to exceed the nominal nine-bit byte time. During transfers,
clock stretching is bounded by `bitTimeoutUs` and the per-byte budget. A timeout
occurs before a bit starts or while waiting on a slave-held clock; a completed
byte stays within `byteTimeoutUs`. E2 transfer maxima are enforced at 25,000 us
per bit and 35,000 us per byte.

Ordinary read and volatile custom-pointer write STOPs use `bitTimeoutUs`
(default 25,000 us). `Config::flashStretchTimeoutUs` separately bounds STOPs
for direct custom-memory writes, and each SCL release during explicit bus
reset, which may encounter a pending flash commit. It defaults to 350,000 us
and accepts 300,000..5,000,000 us, bounded by the write-delay safety limits.
[AN1611-1](https://www.epluse.com/fileadmin/data/product/application_note/E2-Interface-CO2.pdf)
section 5/page 5 assigns the 150 ms single-byte or 300 ms interval-pair flash
extension to direct writes (0x10 at device address 0). Sections 7.1-7.2/page 8
identify 0x50 as a read-pointer update, which does not justify a flash wait.
The default flash allowance adds 50 ms margin. START and byte transfers retain
the generic budgets. Post-write waits remain `writeDelayMs` (default
150 ms) or `intervalWriteDelayMs` (default 300 ms), each limited to 5000 ms,
followed by readback verification. The new field is appended to `Config` so
existing positional initializers retain their field order; rebuild consumers.

A reset has nine clock releases plus STOP, so its maximum stretch allowance is
`10 * flashStretchTimeoutUs` plus nominal phases. Transaction bounds include
START, three read or four write byte budgets, and the command's STOP budget.
With default timing, an ordinary read's requested-delay bound is 155,610 us;
a volatile pointer write's is 190,610 us. Fast/averaged value plus status uses
three reads (466,830 us), or 813,050 us when followed by an error-code pointer
write and read, with retries disabled. These are
budgets for requested callback delays; callbacks must themselves remain bounded.

The library never owns GPIO pins or an I2C/Wire instance. Applications provide `setScl`, `setSda`, `readScl`, `readSda`, and `delayUs` callbacks.

## Optional Control-NACK Retries

`Config::readNackRetries` defaults to 0. Set it to 1, 2, or 3 to permit that
many additional attempts per MV3/MV4/status byte frame (main commands
0xC/0xD/0xE/0xF/0x7, at any device address). Three means four total attempts.
Only a control-byte NACK before data is eligible, after successful STOP and
idle-line checks, separated by a fixed 1,000 us HAL pause. No bus reset is
performed. Identity/custom reads, auto-increment pointers, all writes, PEC
failures, timeouts, and stuck-line failures are not retried. A high-byte retry
keeps the existing low-byte latch; it does not restart the value pair. A NACK
does not establish the sensor's internal reason or a physical fault cause.

The optional `Config::allowReadRetry(void* busUser)` callback can veto another
attempt for an owner deadline, cancellation, or latched HAL callback error.
It is called before and after the pause, and after the final idle read, before
the next frame. It must be bounded, must not access the bus, and must not call
the driver recursively. Null permits eligible retries. The line/delay callbacks
do not return error statuses; without this guard, the library cannot observe
their application-side error latches or enforce an external wall-clock deadline.

Health records each tracked frame's final result once, so transient NACKs do
not force OFFLINE during its retry sequence. `readRetryDiagnostics()` and
`getSettings().readRetry` copy fixed-size cached session diagnostics without
bus access: saturated `controlNacks`, actual `retries`, `recovered` frames, and
`exhausted` frames. Eligible NACKs count even with retries disabled or failed
STOP. `exhausted` counts only enabled retries ending in NACK with clean STOP
after the configured attempts; disabled, vetoed, and cleanup-blocked cases do
not increment it. Last-event fields stay tied to the latest eligible frame
that encountered NACK, across later successes. `lastError` is its final result
(OK if recovered); `lastCleanupError`, `cleanupBlocked`, `retryVetoed`, and
`lastRecovered` preserve the decision. End/begin clear the session snapshot;
explicit recovery preserves it.

With default timing, a NACK frame requests at most 85,610 us
(`START 25,300 + control byte 35,000 + STOP 25,310`). With three retries, a
final full read has the bound `3 * (85,610 + 1,000) + 155,610 = 415,440 us`;
four NACKs exhaust in at most 345,440 us. Fast/averaged value plus status can
therefore request 1,246,320 us, or 1,592,540 us including the non-retried
error-code pointer/read path. Custom/identity/reset bounds are unchanged.
These conservative requested-delay sums exclude callback, guard, and scheduler
overhead; applications must admit an adequate whole-operation time budget.

## Persistent Configuration Writes

Persistent writes can change hardware before a final ACK, STOP, or readback
fails. Multi-byte writes are also not bus-atomic: a low byte can commit before
the high byte fails. These cases mark configuration dirty, including direct
single-byte and raw custom-memory writes. Interrupted PEC transmission is
conservatively uncertain; a definite final PEC NACK does not mark a previously
clean single-byte write dirty.

Use `persistentConfigDirty()` and `persistentConfigDirtyError()` to detect the
condition and retrieve the original failing `Status`. `SettingsSnapshot`
includes the same diagnostics. Every uncertain target remains recorded across
further failures, recovery, and end/begin. `resyncPersistentConfig()` checks the
global interval, advertised offset/gain and part-name fields, and all pending
registers, including stored address/mode/status validation. Missing support for
a pending calibration or part-name register prevents resync. Raw targets get
readability checks; resync does not validate every raw register's meaning.
Only a complete successful readback clears dirty state. Budget up to 256
pending-target pointer/read pairs, plus baseline, capability and status checks;
this is a maintenance API.

`startAutoAdjust()` validates status before writing and returns `BUSY` when
adjustment is already running. Resync of an uncertain adjustment also requires
idle status before reading calibration. Resync establishes readable/coherent
current configuration; it cannot prove the requested change was applied or
that a failed calibration request ran or succeeded. Applications must compare
against their own expected baseline before trusting the outcome. Unrelated
reads and later successful writes do not clear earlier uncertainty.

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

The `sniff` command in both diagnostic CLIs decodes synchronously inside the
transport callbacks. Its console output perturbs E2 timing, so do not use
sniffer-enabled transactions as timing or protocol-stability evidence.

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
`1.1.0` source builds under both pins without compatibility shims.
TunnelMonitor-node's production console uses ESP-IDF USB Serial/JTAG APIs
directly, so the Arduino HWCDC bug does not apply to that console path.

On Windows, use `.\scripts\pio.cmd` in place of `pio`; it selects the existing
VS Code-managed PlatformIO installation.

```bash
pio test -e native
pio run -e ex_bringup_s3
pio run -e ex_bringup_s2
pio run -e compat_tunnelmonitor_s3
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m unittest discover -s test -p "test_*.py"
doxygen Doxyfile
```

GitHub Actions builds the native ESP-IDF example for ESP32-S3 and ESP32-S2 on
ESP-IDF v6.0.1. To reproduce those builds locally from the repository root:

```bash
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup set-target esp32s2 build
```

Use the [validation matrix](docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md) for the
current evidence ledger and remaining per-board validation gaps. The
[HIL runner guide](docs/EE871_E2_HIL_RUNNER.md) provides the safe CLI recipe
and explicit opt-ins for persistent writes and physical fault tests.

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

Core retries are disabled by default and can be explicitly enabled as described
above. The example configuration retains that default. The separate soak harness
may retry only a scheduled MV3/MV4 control-byte `NACK` once after a configurable
delay (default 1,500 ms) and records the original attempt, retry, and
`SCHEDULED_CONTROL_NACK_RECOVERED` outcome. This is application-level cadence
policy, not hidden driver behavior. The result name deliberately does not infer
why the sensor NACKed.

## Documentation

- [Documentation index](docs/README.md) - maintained guides and vendor references.
- [Changelog](CHANGELOG.md) - release history and migration notes.
- [Validation matrix](docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md) - current and historical evidence, with coverage limits.
- [HIL runner guide](docs/EE871_E2_HIL_RUNNER.md) - commands, framing, and verdict rules.
- [ESP-IDF guide](docs/IDF_PORT.md) - native integration and build instructions.
- [Protocol and register map](docs/EE871_E2_Protocol_and_Register_Map.md) - E2 timing, transactions, and EE871 registers.

Generate the API reference with `doxygen Doxyfile`, then open
`docs/doxygen/html/index.html`. Generated HTML is ignored by Git; public header
comments and the maintained guides are its source.

## License

MIT License. See [LICENSE](LICENSE).
