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
- **Validated discovery** - full identity and seven capability bytes publish atomically
- **Optional-device lifecycle** - narrow absent-at-start policy with explicit recovery
- **Checked CO2 samples** - ordered MV3/MV4, status, error, and range evidence
- **Feature guards** - optional EE871 registers are checked from cached capability flags
- **Mutation truth** - accepted, ambiguous, verified, and resynchronized
  persistent/maintenance effects remain observable
- **HIL evidence tooling** - serial runner emits transcript, JSON, and Markdown reports

## Release And Validation Status

Version metadata is set to `1.1.0` for this source candidate. The version in
`library.json` is authoritative and the repository version tool synchronizes
the generated header, ESP-IDF component metadata, and Doxygen project number.
The driver is production-oriented and validation-backed for the tested
ESP32-S3/EE871 bench setup, but it is not fully field-proven across every
physical fault case.

Recorded evidence:

- Native tests: 91/91 passing on the Prompt 04 final candidate.
- Arduino PlatformIO builds: `ex_bringup_s3` and `ex_bringup_s2` pass locally
  in the latest hardening/readiness runs.
- ESP32-S3 safe default HIL: PASS on `COM17`.
- ESP32-S3 historical legacy extended-safe HIL: PASS on `COM17`; this predates
  the current expanded alias.
- ESP32-S3 legacy persistent measurement interval write/readback/restore:
  PASS on `COM17`; the new full-image pre-restore/final comparison procedure
  has not been run on hardware.
- Physical unplug/replug recovery: PASS, operator-confirmed manual test; no
  automated transcript is recorded.

Remaining documented gaps:

- Pure ESP-IDF build success must be verified by GitHub Actions or local
  `idf.py` builds.
- ESP32-S2 hardware HIL and pure ESP-IDF hardware HIL are not recorded.
- The expanded checked-sample/health `--complete-safe` plan and strengthened
  full-image persistent restore plan are not yet recorded on hardware.
- Power-cycle persistence, CO2 calibration writes, bus-address write/recovery,
  and stuck-line fault-jig tests are not recorded.

## E2 Bus, Not Hardware I2C

EE871-E2 uses GPIO-style open-drain E2 signaling. The library does not use
Arduino `Wire`, ESP-IDF `driver/i2c_master`, or a hardware I2C peripheral.
Applications provide `setScl`, `setSda`, `readScl`, `readSda`, and `delayUs`
callbacks through `Config`. Optional task-context `delayMs` and `yield`
callbacks make long bounded completion waits cooperative; the core still owns
no task, scheduler, timebase, or GPIO.

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

The quick start above deliberately demonstrates a raw MV4 read. It does not
apply status, range, warm-up, freshness, or cadence policy.

## Checked CO2 Samples

`readCo2Average()` and `readCo2Fast()` remain source-compatible raw MV4/MV3
value reads. They read low then high bytes and return the unsigned value
without reading status or applying the checked range.

For a validated sensor procedure, use `readCo2AverageSample()` or
`readCo2FastSample()`. Both share one value-first procedure:

1. read the requested raw MV4 or MV3 value;
2. read status for that last value;
3. when status reports a CO2 error and cached capabilities advertise it, read
   detailed error code `0xC1`;
4. otherwise validate the raw value against the broad library range
   `cmd::CO2_PPM_MIN..cmd::CO2_PPM_MAX` (0..50,000 ppm).

`Co2ReadResult` retains raw ppm, status, detailed error code, exact per-step
`Status` values, and explicit attempted/valid flags. A default OK step status
does not mean the step ran; check its attempted flag. Documented codes map to
`Co2SensorError`, while an unrecognized code—or a status error without
detailed-code support—maps to `UNKNOWN`.

`CO2_SENSOR_ERROR` and checked `OUT_OF_RANGE` are sensor-domain results. They
do not create E2 transport failures or degrade a transport-healthy driver.
Failures while reading the value, status, pointer, or error code retain their
precise transport/protocol status and existing health behavior.

Reading status can start/trigger the next measurement and reset interval timing
under the device's documented conditions. This is why checked procedures read
the measured value first and status second. Power-up warm-up, readiness after
a trigger, freshness, plausibility, sampling cadence, and retry policy remain
application-owned. The 50,000 ppm guard is a broad library limit; applications
must still respect the range of their specific EE871 variant.

## Health Monitoring

```cpp
if (sensor.state() == EE871::DriverState::OFFLINE) {
  sensor.recover();
}

Serial.printf("Failures: %u consecutive, %lu total\n",
              sensor.consecutiveFailures(),
              static_cast<unsigned long>(sensor.totalFailures()));
```

Validation and precondition errors return before E2 traffic and do not update
health counters. `probe()` uses raw E2 reads and is diagnostic-only; normal
reads/writes use tracked wrappers. Health counters count those tracked E2
transfers, not public calls or samples: one checked sample normally contributes
three successes and can contribute five when detailed error acquisition runs.
`IN_PROGRESS` is health-neutral.
`Config::offlineThreshold = 0` is normalized to one failed operation. Failed
`begin()` and `end()` paths clear stale session/capability state so later
diagnostics do not report old sensor capabilities, but they preserve an
unresolved mutation diagnostic until explicit target-specific reconciliation.

Cache-only diagnostics are available through `SettingsSnapshot`,
`getSettings(SettingsSnapshot&)`, `getSettings()`, `isInitialized()`,
`getConfig()`, `identity()`, `capabilities()`, `driverState()`,
`healthState()`, and `offlineThreshold()`.

### Strict And Optional Startup

`Config::beginPolicy` defaults to `BeginPolicy::REQUIRE_PRESENT`. Strict
startup succeeds only after the driver validates group `0x0367`, subgroup
`0x09`, the advertised CO2 measurement bit, and all seven capability bytes
from custom memory `0x03..0x09`.

For a product where the sensor is genuinely optional,
`BeginPolicy::ALLOW_ABSENT` may accept only `DEVICE_NOT_FOUND` from an
authoritative presence mechanism. The current GPIO E2 transport has no such
mechanism. An identity-stage `NACK` therefore remains `NACK` under both begin
policies, performs no hidden retry or long wait, and leaves the driver
`UNINIT`. This matters because the E2 specification permits a responsive
sensor to NACK while measurement has priority; a clean STOP does not prove
physical absence.

If a future transport can authoritatively report `DEVICE_NOT_FOUND`, accepted
optional absence initializes a latched `OFFLINE` session with invalid
identity/capability caches and cache-only `beginProbeStatus` evidence. Until
then, an optional-device owner keeps its module alive after failed begin and
uses an explicit later begin attempt according to application retry policy.

Timeout, stuck bus, NACK, PEC mismatch, incompatible group/subgroup, missing
CO2 support, and any partial or semantically invalid capability read fail
`begin()` and leave the driver uninitialized. A responding but incompatible
device is never treated as absent.

While `OFFLINE`, normal reads and writes return `Err::OFFLINE` without touching
the E2 lines or changing health counters. `probe()` remains a raw,
health/cache-neutral identity diagnostic. Public `checkBusIdle()` and
`busReset()` also remain diagnostic and cannot restore online state. Only
`recover()` performs tracked reset, full identity validation, and a complete
capability reload before atomically entering `READY`. The application owns
retry cadence, backoff, power policy, and aggregate health decisions.

## Timing And Blocking

The driver is managed synchronous: E2 transactions block for bounded protocol
time, and `tick(nowMs)` only records the latest application timestamp for
diagnostics. It does no scheduling or timebase extension. An owner that wants
current health timestamps should call `tick()` immediately before each owned
library operation. Ordinary bit and byte stretches retain the E2 limits of
25 ms and 35 ms. Only the final PEC ACK and STOP of `0x10`/`0x50` writes can
consume the separate write-completion window. The first interval byte is
staged with ordinary timing; its high-byte commit uses the separate
interval-pair window.

Each completion window is one cumulative sensor allowance. SCL-low polling at
the final PEC ACK and STOP consumes that allowance, and the cooperative wait
after STOP uses only its remainder. The fixed, configuration-bounded master
ACK/STOP waveform is additional protocol tail, so a legal device-held-low
interval of exactly 150 ms or 300 ms remains valid. A split stretch across ACK
and STOP does not receive a second allowance, and a fully consumed allowance
does not cause another quiet wait.

`writeDelayMs` values below 150 ms normalize to 150 ms, and
`intervalWriteDelayMs` values below 300 ms normalize to 300 ms. Those values
are the recommended normal settings. Existing values through 5000 ms remain
valid, but can make multi-step convenience operations block for several
seconds.

Pointer writes complete before any dependent `0x51` read begins. Block reads
set the pointer once, wait once, and then use pointer auto-increment. Persistent
write readback mismatches return `VERIFY_MISMATCH`; they are not transport
failures.

Applications can supply optional `delayMs` and `yield` callbacks. Long
completion waits are divided into `longDelaySliceMs` slices (1..50 ms), use
`delayMs` when available, and yield after each completed slice. These callbacks
are task-context facilities and are not ISR-safe. They are never called from
bit-level signaling or ordinary clock-stretch polling.

Use either `operationTimingBound()` overload to obtain a conservative,
configuration-derived admission bound without E2 I/O. The static overload
validates a proposed `Config`; the instance overload uses the normalized active
configuration. Bounds assume callbacks honor requested delays and remain
bounded. See
[EE871_E2_OPERATION_TIMING_BOUNDS.md](docs/EE871_E2_OPERATION_TIMING_BOUNDS.md)
for formulas, count rules, and the exhaustive public-method map. The
`tools/check_public_timing_contract.py` source audit rejects a new public
callable without an explicit BUS or `NO_E2_IO` Doxygen classification.

The library never owns GPIO pins or an I2C/Wire instance. Applications provide
the open-drain line and delay callbacks.

## Persistent And Maintenance Mutations

Every effectful custom-memory API uses one mutation admission, frame-completion,
and effect-classification path. `MutationDiagnostic` records the
`MutationTarget`, best known `MutationEffect`, address range, requested,
acknowledged, observed, and matched element counts, retained value evidence,
and the first uncertainty cause. Effects distinguish definite no-effect,
acknowledged, indeterminate, verified, later resynchronized, and the narrow
auto-adjust operator acknowledgement.

Multi-byte persistent writes are not bus-atomic. A low byte can commit before a
high byte fails, or an accepted write can fail during STOP, completion, or
readback. When `mutationDiagnostic().unresolved` is true, every further
effectful API returns `PERSISTENT_STATE_UNCERTAIN` before E2 I/O. Normal reads,
diagnostics, probe, recovery, bus inspection/reset, and explicit resync remain
available. The driver never automatically replays a retained request.

`resyncPersistentConfig()` reads the exact unresolved target and compares it
with the retained fixed-size intent. A complete match becomes `VERIFIED`; a
coherent mismatch becomes `RESYNCHRONIZED`, leaving actual state inspectable.
In either case the application must compare device state with its own intended
baseline. When no mutation is unresolved, resync performs the complete
capability-aware coherence read and skips unsupported optional settings.

The source-compatible `persistentConfigDirty()` and
`persistentConfigDirtyError()` accessors mirror the mutation diagnostic. The
legacy error is OK whenever uncertainty is resolved, even if historical
observation evidence remains.

Bus-address change and auto-adjust have deliberately stricter procedures:

- An acknowledged bus-address write remains unresolved because activation
  timing is not safely inferable in-session. The application calls `end()`,
  performs its authorized device power procedure if required, supplies the
  candidate address in a new `Config`, calls `begin()`, then calls
  `resyncPersistentConfig()`. The driver never scans or guesses an address.
- `startAutoAdjust()` first observes `0xD9`, rejects an already-running action
  as `BUSY`, performs one non-replayable write, and observes status again.
  Running proves this request started. A clean not-running observation after an
  acknowledged request is historically ambiguous and remains unresolved until
  later proof or the cache-only, target-specific
  `acknowledgeAutoAdjustUncertainty()` decision.

Typed optional-setting reads and writes fail bus-silently when the validated
cached capabilities do not advertise their register. This includes part name,
address, interval, filter, operating mode, auto-adjust, and calibration
offset/gain. Capability bytes `0x03..0x09` must also have every reserved bit
clear before the seven-byte snapshot is published. A capability error returns
`NOT_SUPPORTED` with `Status::detail = (address << 8) | raw`; `0x07 == 0x55`
is valid because its reserved bit is clear. `0x02` remains a diagnostic E2
version read and is not a lifecycle compatibility gate.

Typed persisted reads validate the observed hardware value before publishing
it to the caller: address `0..7`, global interval `150..36000` deciseconds,
nonzero signed interval factor, D8 reserved/capability bits, and D9 reserved
bits. Failure leaves the output unchanged and retains the raw value in
`Status::detail`. The same validators govern typed writes, post-write
observations, unresolved-target resync, and full coherence resync. These
semantic results follow successful bus traffic, so they do not invent
transport failures. The CO2 filter stays opaque because its values are
product-specific.

Raw `customRead()` remains the explicit untyped diagnostic path.
`customWrite()` is an expert maintenance API: callers must derive address
semantics and restoration from authoritative vendor documentation. Its
protected dispatch prevents raw bypass of typed address, interval,
calibration, mode, factor, auto-adjust, and read-only-register safety, and a
raw memory dump must never be replayed as restoration.

Unresolved evidence survives `end()`, failed/repeated `begin()`, and a later
successful `begin()` on the same object. Destroying the object or losing
application RAM necessarily loses that evidence. Applications that must survive
restart must persist their maintenance workflow outside this library; the core
does not own NVS or a filesystem.

The bring-up CLIs expose this through safe diagnostic commands:

```text
dirty
resync
```

`dirty` prints the legacy mirror plus complete mutation
target/effect/progress evidence. `resync` prints state before and after
target-specific reconciliation; it does not perform arbitrary writes or
silently discard uncertainty. Normal safe commands such as `probe`, raw and
checked samples, `selftest`, `stress`, and `stress_mix` do not create
persistent state.

Treat persistent writes such as measurement interval, part name, CO2 offset,
and CO2 gain as maintenance operations. The CLI `reg write <addr> <value>`
command reaches only otherwise-unclassified writable bytes after protected
typed, paired, and read-only dispatch. It is an expert maintenance operation:
use it only with authoritative address/restoration semantics, never replay a
raw memory image, and account for sensor flash/endurance and longer write
latency.

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
  `checkBusIdle`, `mutationDiagnostic`, `persistentConfigDirty`,
  `persistentConfigDirtyError`, `acknowledgeAutoAdjustUncertainty`
- Admission: static and instance `operationTimingBound`
- Identification: `readGroup`, `readSubgroup`, `readFirmwareVersion`, `readE2SpecVersion`
- Measurements: raw `readCo2Fast`/`readCo2Average`, checked
  `readCo2FastSample`/`readCo2AverageSample`, `readStatus`, `readErrorCode`
- Cached CO2 capabilities: `hasCo2OffsetGain`, `hasCo2AdjustmentPoints`,
  `hasErrorCode`
- Custom memory/config: `customRead`, `customWrite`, `writeMeasurementInterval`, bus address, filter, operating mode, auto-adjust, calibration helpers
- Low-level command helpers: `cmd::makeControlRead`,
  `cmd::makeControlWrite`, `cmd::isReadMainCommandSupported`, and
  `cmd::co2ErrorCodeName`. Unsupported EE871 main-command reads return
  `NOT_SUPPORTED` before bus traffic.

## Examples

- `examples/01_basic_bringup_cli/` - Interactive CLI for testing
  - `co2fast`/`co2avg` remain raw; `samplefast`/`sampleavg` expose checked
    value/status/error evidence. Status and checked commands may trigger the
    next device measurement under documented conditions.
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
python tools/check_public_timing_contract.py
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
python tools/ee871_hil_runner.py --port COMx --complete-safe
python tools/ee871_hil_runner.py --port COMx --include-unplug-replug --board BOARD --target-name TARGET --operator OPERATOR --sensor-id SENSOR --fixture-id FIXTURE --electrical-authority PROCEDURE
python tools/ee871_hil_runner.py --port COMx --include-persistent-writes --confirm-persistent-writes --board BOARD --target-name TARGET --operator OPERATOR --sensor-id SENSOR --fixture-id FIXTURE --electrical-authority PROCEDURE
```

The default runner sequence is non-persistent and records `version`, `help`,
`probe`, `read`, `selftest`, `drv`, `dirty`, `stress 50`, final `drv`, and
final `dirty`. `--complete-safe` adds checked fast/average samples with adjacent
health-counter evidence, complete feature/capability reads, bus/line checks,
`stress_mix 100`, recovery, and resync. Warm-up and stale-measurement timing
remain separate controlled HIL rows.
Destructive plans are isolated behind separate exact opt-ins. They checkpoint
a complete 256-byte forensic baseline plus a pre-restore post-test image,
restore only typed settings from verified-clean state, journal every write
before transmission, and never replay raw custom memory. Dry-runs and
operator/fault steps are never reported as hardware `PASS`; see the runner
guide for calibration, address, auto-adjust, power, and stuck-line procedures.

## Documentation

- `docs/README.md` - documentation index and status map
- `CHANGELOG.md` - full release history
- `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` - hardware validation plan and CLI recipe
- `docs/EE871_E2_HIL_RUNNER.md` - automatic serial HIL runner usage and verdict rules
- `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md` - conservative blocking-bound formulas
- `docs/IDF_PORT.md` - ESP-IDF portability and validation guidance
- `docs/IDF_PORT_IMPLEMENTATION.md` - ESP-IDF implementation notes
- `docs/EE871_E2_RELEASE_NOTES_1.1.0.md` - current release notes and tagging checklist
- `docs/EE871_E2_RELEASE_NOTES_1.0.0.md` - historical 1.0.0 release notes

## License

MIT License. See [LICENSE](LICENSE).
