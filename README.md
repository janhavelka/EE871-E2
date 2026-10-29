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
- **Explicit checked samples** - MV3/MV4 evidence followed by sensor status
- **Feature guards** - cached operating capabilities and on-demand calibration checks
- **Dirty/resync diagnostics** - uncertain single- and multi-byte writes are visible
- **HIL evidence tooling** - serial runner emits transcript, JSON, and Markdown reports

## Release And Validation Status

The latest published release is
[v1.1.0](https://github.com/janhavelka/EE871-E2/releases/tag/v1.1.0).
This README describes the current source, including
[Unreleased changes](CHANGELOG.md#unreleased); use the documentation at the
release tag when consuming the published version.
[CHANGELOG.md](CHANGELOG.md) covers migration, stricter startup/recovery,
ordinary STOP timing, opt-in control-NACK retries, calibration guards, and
persistent-write uncertainty.

When moving from `v1.1.0` to Unreleased, review the example/HIL evidence and
serial-framing corrections in the changelog and pin the chosen source commit.
Rebuild consumers for the appended Config members and fixed driver storage;
existing positional Config bindings and error values retain their meaning.
The older migration from `v1.0.1` remains part of the published `1.1.0` notes.

The selected software work is complete at implementation commit
`5c920d00b7065a906b8fae7f4bf751a42796e955` (2026-10-29): HIL sender admission,
checked samples, cooperative maintenance waits, persistent-operation evidence,
and requested-delay bounds. Strict startup is preserved and verified.
Local verification passed 134 native tests, 100 Python tests, the timing/CLI/IDF
contracts, version consistency, Doxygen and all three Arduino builds (S3, S2,
TunnelMonitor compatibility). [All six CI jobs passed at that exact SHA](https://github.com/janhavelka/EE871-E2/actions/runs/40915629506),
including native ESP-IDF 6.0.1 S3/S2 builds. Local native-IDF builds were NOT RUN
because the SDK was unavailable. The later integrated closeout changes only
documentation and reuses these unchanged implementation results; its checks
cover documentation separately. [Prompt outcomes and verification details](https://github.com/janhavelka/EE871-E2/blob/main/docs/prompts/README.md)
retain the implementation traceability. No hardware qualification was added.

Earlier software verification of implementation commit `3d32ac3` on 2026-09-11 passed
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
freshness, or the product-specific valid ppm range. They retain their raw
contract, including successful transfers while sensor status reports an error.

For explicit value/status composition, use `readCo2FastSample()` or
`readCo2AverageSample()`. Each reads the value low byte then high byte, followed
by status. It reads error detail only when status bit 3 is set and cached
capabilities advertise the error-code register. A failed value read stops the
procedure before the side-effecting status read. Reading status can trigger
the next measurement and reset the interval counter only when the global
interval exceeds 15 s and the previous value is older than 10 s (AN1611-1
sections 4 and 10). Value-before-status ordering does not prove that a fresh
conversion occurred.

```cpp
// The owner applies warm-up, cadence and freshness policy before this call.
EE871::Co2ReadResult sample;
const EE871::Status checked = sensor.readCo2AverageSample(sample);
if (!checked.ok()) {
  // sample.valueValid can still preserve a raw word as diagnostic evidence.
  // sample.co2Error remains true if a later error-detail transfer failed.
} else {
  // Example application policy only: this installation accepts 400..10000 ppm.
  static constexpr uint16_t APP_MIN_PPM = 400;
  static constexpr uint16_t APP_MAX_PPM = 10000;
  const bool applicationRangeValid =
      sample.ppm >= APP_MIN_PPM && sample.ppm <= APP_MAX_PPM;
  if (applicationRangeValid) {
    // Consume sample.ppm under the owner's freshness/accuracy requirements.
  }
}
```

`Co2ReadResult` is replaced on every call. `valueValid` means the complete raw
word was transferred; `ppmValid` means the checked procedure succeeded with
clear sensor status. The driver accepts the entire 16-bit word, including 0
and 65535, without imposing a product range. Checked success certifies only
successful requested transfers and clear sensor status; warm-up, freshness,
concentration accuracy, product model, range and cadence remain application
policy.

The result retains the raw value, status byte and optional error code, with
explicit validity and attempted flags and exact per-step `Status` values.
Default OK status fields do not prove an unattempted step ran. Status or
error-detail transfer failure returns its exact code/detail while preserving
earlier evidence. A reported sensor error returns `CO2_SENSOR_ERROR` when
optional detail succeeds or is unsupported; known and unknown raw codes remain
visible. If detail fails, its transfer status is returned separately from the
retained sensor-status evidence.

`begin()` and `recover()` reject reserved bits in feature bytes `0x07..0x09`
before installing the cache. `readOperatingMode()` rejects active mode bits
whose capability is absent. `readAutoAdjustStatus()` requires advertised
auto-adjust support and rejects reserved result bits instead of decoding
unsupported register replies as a running adjustment.

Startup is strict: failed initialization from UNINIT returns its precise status,
leaves UNINIT and installs no partial capability cache. Calling `begin()` on an
active session returns `ALREADY_INITIALIZED` without changing that session.
NACK, PEC, timeout, stuck lines and wrong identity are never converted into
successful startup or confirmed
absence. `DEVICE_NOT_FOUND` remains reserved for compatibility. Applications
needing hot-plug startup own a bounded schedule for retrying `begin()`; the
driver adds no recurring retry policy. Persistent uncertainty and latest
operation evidence survive both successful and failed initialization.

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
read before its high byte is attempted. Checked sample helpers add no aggregate
health event: clean value/status uses three successful tracked reads, and
supported error detail adds a pointer write and read. `CO2_SENSOR_ERROR` is a
sensor-domain result and does not add a transport failure. Existing retry
rules and sticky diagnostics apply to each frame, without replaying the whole
sample. `BUSY` means auto adjustment is already
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
followed by readback verification. Zero quiet waits remain valid. Appended
`Config` fields preserve existing positional initializer order; rebuild consumers.

Optional `Config::delayMs` and `Config::yield` callbacks make only those
post-write quiet waits cooperative, after a successful persistent transfer and
before readback. `longDelaySliceMs` defaults to 10 ms and
must be 1..50 ms; `begin()` rejects zero or larger slices before GPIO traffic.
Each slice requests the smaller of the remaining wait and the configured
slice size, so requested delays sum exactly to the configured quiet wait,
including a final partial slice. With `delayMs` set, the driver invokes it once
per positive slice. Otherwise it retains one `delayUs(1000)` request per
millisecond. If `yield` is set, it runs after each completed slice, including
the final one. When both callbacks exist, delay runs first and yield second.
A zero wait invokes neither callback. Both callbacks default to null, retaining
the existing fallback delay sequence without requiring changes from callers.

These callbacks never run during START, STOP, bit/byte/ACK/PEC signaling,
SCL-release polling, reads, volatile pointer updates, or the fixed 1000 us
read-retry pause. Flash STOP stretching remains protocol signaling with its
separate timeout; it is not a quiet wait. The Arduino and native IDF examples
install FreeRTOS task-delay callbacks through their adapters, with conservative
upward tick rounding plus their framework's task yield, to avoid under-waiting
the requested milliseconds.

Callbacks belong to the application, run in task context under the same
external serialization as the driver, and must remain bounded and
non-reentrant. `delayMs` must wait at least the requested duration. A void
callback cannot report failure, and the core cannot independently prove its
wall-clock adherence or impose a cancellation/watchdog guarantee. Tick rounding,
scheduler delay and callback/yield overhead can lengthen the actual wait and
are excluded from requested-delay sums. Ownership remains with the calling
application; no background task or asynchronous engine is introduced.

### Programmatic requested-delay bounds

`EE871::operationTimingBound(config, kind, out, target, count)` calculates the
**sum of maximum requested protocol waits/delays under the configured callback
contract**, in `OperationTimingBound::requestedDelayUs` (`uint64_t`). The static
query works before `begin()`, including with null transport callbacks. It shares
the numeric configuration checks with `begin()`; only `begin()` requires the
transport callbacks to be installed. Calculation invokes no callback, accesses
no GPIO and changes no driver state. It does not sleep, retry or reset.

```cpp
EE871::Config planned; // Default timing; no GPIO callbacks needed for this query.
EE871::OperationTimingBound bound;
EE871::Status st = EE871::EE871::operationTimingBound(
    planned, EE871::OperationKind::CO2_CHECKED_SAMPLE, bound);
if (st.ok()) {
  const uint64_t requestedUs = bound.requestedDelayUs; // 813050 with defaults.
  // Add application-owned runtime margins before admitting the operation.
  (void)requestedUs;
}
```

The result is an integer microsecond sum without intermediate rounding. GPIO
callback execution, optional delay/yield/retry-guard overhead, scheduler
preemption and latency are outside this bound. A caller needing a wall-clock
deadline must supply and own bounds for those costs too. This query does not
predict sensor readiness, sample freshness, calibration completion or device
presence. Both diagnostic CLIs provide `bounds` to display calculations from
their configured settings without performing any selected operation. The
existing `timing` command performs live bus tests and remains separate.

`target` defaults to zero and `count` to one. Only these uses accept other
values:

| Operation selector | Target and count contract |
| --- | --- |
| `CONTROL_READ` | Target is a supported main-command nibble. Status/MV3/MV4 include configured control-NACK retries; identity/custom reads do not. Count is one. |
| `CUSTOM_POINTER`, `CUSTOM_WRITE` | Target is a custom address 0..255; count is one. `CUSTOM_WRITE` includes verification and handles the special interval-byte path at `0xC6`/`0xC7`. |
| `CUSTOM_READ` | Target is a custom address 0..255; count is 1..`256 - target`, including the initial pointer update. |
| All other selectors | Target is zero and count is one; the selected operation has a fixed size. |

Unknown selectors, zero count, non-unit counts for fixed operations and
inappropriate targets return `INVALID_PARAM`; unsupported EE871 command
nibbles return `NOT_SUPPORTED`. An address or custom-read span outside the
memory map, or arithmetic overflow, returns `OUT_OF_RANGE`. Invalid numeric settings return
`INVALID_CONFIG`. The output is unchanged on any error. Numeric validation
preserves zero post-write waits and validates `longDelaySliceMs` even when its
callbacks are absent. Capability-dependent selectors conservatively include
every supported branch because capabilities are unknown before initialization.

The derivation follows the current transfer helpers. All quantities below are
microseconds; `B = byteTimeoutUs` already includes nominal setup/high/low phases
and ACK/NACK. STOP budgets include cleanup after transfer failure. A failed
START returns earlier, while later failures consume only a prefix plus STOP.

| Primitive | Requested-delay formula | Default |
| --- | --- | ---: |
| START `S` | `bitTimeoutUs + 2*startHoldUs + clockLowUs` | 25,300 |
| Ordinary STOP `T` | `clockLowUs + 10 + bitTimeoutUs + 2*stopHoldUs` | 25,310 |
| Flash STOP `Tf` | `clockLowUs + 10 + flashStretchTimeoutUs + 2*stopHoldUs` | 350,310 |
| Ordinary read `R` | `S + 3*B + T` | 155,610 |
| Volatile pointer `P` | `S + 4*B + T` | 190,610 |
| Direct persistent frame `F` | `S + 4*B + Tf` | 515,610 |
| Control-NACK frame `N` | `S + B + T` | 85,610 |
| Eligible read `Q` | `R + readNackRetries*(N + 1000)` | 155,610 |
| Custom block `C(n)` | `P + n*R` | `C(1) = 346,220` |
| Verified direct byte `D` | `F + 1000*writeDelayMs + C(1)` | 1,011,830 |
| Reset `Z` | `9*(clockLowUs + flashStretchTimeoutUs + clockHighUs) + Tf` | 3,502,110 |

The full operation compositions below use those primitives and default
settings with retries disabled. They cover early failure and cleanup paths as
well as success; they are conservative budgets, not measured execution times.

| Operation selector / API | Composition | Default requested us |
| --- | --- | ---: |
| `CO2_PAIR_READ` / raw MV3 or MV4 | `2*Q` | 311,220 |
| `CO2_CHECKED_SAMPLE` / checked fast or average | `3*Q + C(1)`, including optional error detail | 813,050 |
| `CUSTOM_READ`, address 0, count 256 | `C(256)` | 40,026,770 |
| `CUSTOM_WRITE`, ordinary byte | `D` | 1,011,830 |
| `INTERVAL_WRITE` | `I = 2*F + 1000*intervalWriteDelayMs + C(2)` | 1,833,050 |
| `CUSTOM_WRITE`, address `0xC6` or `0xC7` | `C(1) + I`, including companion read | 2,179,270 |
| `CO2_ADJUSTMENT_READ` / offset or gain | `C(1) + C(2)`, including support read | 848,050 |
| `CO2_CAL_POINTS_READ` | `C(1) + C(4)`, including support read | 1,159,270 |
| `CO2_ADJUSTMENT_WRITE` / offset or gain | `C(1) + 2*D`, each byte verified | 2,369,880 |
| `PART_NAME_WRITE` | `16*D` | 16,189,280 |
| `AUTO_ADJUST_START` | `C(1) + D`, including status preflight | 1,358,050 |
| `PROBE` | `4*R`, strict identity and CO2 capability | 622,440 |
| `BEGIN`, `RECOVER` | `Z + 4*R + C(3)` | 4,781,990 |
| `BUS_RESET` | `Z` | 3,502,110 |
| `PERSISTENT_RESYNC` | `C(2) + C(4) + C(16) + 262*C(1)` | 94,704,890 |

Address, mode, factor and filter writes use `CUSTOM_WRITE` with their register
address. Their capability checks do not add bus traffic. `BEGIN` reserves a
reset for an initially non-idle bus; `RECOVER` always attempts it. `PROBE`
does not reset or reload feature flags. Resync reserves all 256 pending target
reads, four individual adjustment-point support reads, the possible auto-adjust
idle preflight, and baseline interval/support/calibration/part-name reads. The
bound neither exposes the pending bitmap nor assumes it contains one target.
Cached status/configuration/evidence getters, `tick()` and `end()` request zero delays
and are intentionally excluded from the selector. `checkBusIdle()` also
requests zero delay, but it reads GPIO and is not bus-silent.

Post-write callback slicing preserves the exact configured quiet-wait sum.
Changing slice size or callback presence therefore does not change these
results. With maximum accepted settings (low/high 1890/100 us, START/STOP holds
65535 us, bit/byte 25000/35000 us, flash 5000000 us, both quiet waits 5000 ms),
the resync bound is 240,365,570 us; checked wide arithmetic rejects overflow
before publishing a result. At minimum accepted settings (low/high 100/100 us,
START/STOP holds 4 us, bit/byte 1/1891 us, flash 300000 us, zero quiet waits),
ordinary read, pointer and resync bounds are 5,901, 7,792 and 3,740,764 us.

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

`getLastPersistentOperation()` returns a cache-only `PersistentOperationResult`
for the latest outer mutation call, including calls rejected before a write.
`PersistentOperation` identifies the raw or typed public operation;
`startAddress` and `length` identify its affected custom-memory range. Fixed
16-byte `requested` and `observed` arrays retain complete part-name intent and
partial results without allocation. Each bit in the following masks describes
the byte at `startAddress + bit_index`:

| Mask | Evidence |
| --- | --- |
| `requestedValidMask` | The requested byte is known. A raw interval-byte write obtains its companion byte before requesting the whole pair. |
| `attemptedMask` | A direct write procedure attempted that byte. Reads and volatile pointer updates do not count. |
| `possibleEffectMask` | PEC transmission started and no definite final PEC NACK proved rejection; includes acknowledged bytes. |
| `acknowledgedMask` | The final PEC ACK was sampled, even if the following clock-low phase or STOP failed. |
| `observedValidMask` | A post-write read frame completed successfully, including PEC and STOP. |
| `matchedMask` | A valid observed byte matched the requested byte. |

`result` preserves the operation's exact returned `Status`, including detail
and message. `effect()` summarizes the masks as `NONE`, `POSSIBLE`,
`ACKNOWLEDGED` or `VERIFIED`. All target bytes must match for `VERIFIED`, and
all must have a sampled final ACK for `ACKNOWLEDGED`; partial progress remains
`POSSIBLE`. `NONE` can mean no write attempt or a definitely
rejected attempt; inspect `attemptedMask` to distinguish them. A low-byte
success followed by high-byte rejection remains a partial operation, with its
earlier evidence intact. A mismatching readback remains distinct from the
requested value. Matching bytes verify only what was read, not atomicity,
physical calibration success or future retention.

This is one operation's history, not the unresolved-state owner. The next
outer mutation replaces the record; internal byte helpers do not. Ordinary
reads, `recover()`, `end()`/`begin()` and `resyncPersistentConfig()` retain it.
The pending-target bitmap and original dirty error remain independent: a later
successful or rejected operation cannot erase earlier pending targets. Existing
raw and typed core writes remain permitted while dirty, so applications must
retain their own baseline, intent and admission decision across operations.
Cached precondition and range rejections do not touch the bus; on-demand
calibration-support and auto-adjust-status checks can read before rejecting,
but do not attempt the mutation.

`startAutoAdjust()` validates status before writing and returns `BUSY` when
adjustment is already running. Resync of an uncertain adjustment also requires
idle status before reading calibration. Resync establishes readable/coherent
current configuration; it cannot prove the requested change was applied or
that a failed calibration request ran or succeeded. Applications must compare
against their own expected baseline before trusting the outcome. Unrelated
reads and later successful writes do not clear earlier uncertainty.

For address writes, matching `0xC0` readback verifies only the stored byte. The
driver does not establish when the active address changes or switch its active
session automatically; the application must choose and validate a candidate
session explicitly. For auto-adjust, matching `0xD9` running status is evidence
about the request/status only. Later idle status does not prove that an
uncertain one-shot request ran or succeeded, and no helper replays it.

The bring-up CLIs expose this through safe diagnostic commands:

```text
dirty
operation
resync
```

`dirty` prints `persistentConfigDirty`, the original dirty error status
code/detail/message, and whether resync is needed. `resync` prints dirty state
before and after calling `resyncPersistentConfig()`; it does not perform
arbitrary writes and does not clear dirty state unless the core API reports
successful current-state coherence. `operation` prints the latest operation,
requested/observed byte evidence and its original result without bus traffic,
health updates or state changes.

Both CLIs admit maintenance writes only while persistent state is clean and
the retained operation has no possibly affected byte lacking matching
readback. Any possible address-write effect also blocks further mutations
until an application-owned candidate-session decision. `recover` and `resync`
do not erase this evidence or silently reopen admission; there is no diagnostic
reset/admit command. The HIL runner requires a fresh `operation` response with
open admission immediately before each opted-in destructive send, alongside
its clean diagnostics and current target baseline. Its runtime confirmation
and separate campaign failure latch remain required; clean later diagnostics
never cancel that latch. Normal safe commands such as `probe`, `status`,
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
  `checkBusIdle`, `persistentConfigDirty`, `persistentConfigDirtyError`,
  `getLastPersistentOperation`, pure `operationTimingBound`
- Identification: `readGroup`, `readSubgroup`, `readFirmwareVersion`, `readE2SpecVersion`
- Measurements: `readStatus`, raw `readCo2Fast` / `readCo2Average`, checked
  `readCo2FastSample` / `readCo2AverageSample`, `readErrorCode`
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

Both CLIs expose explicit `co2fastsample` and `co2avgsample` diagnostics with
per-step evidence. These commands read status and can trigger the next
measurement under the conditions above. Existing `read`, `co2fast` and
`co2avg` commands remain raw; automated HIL/soak plans do not add checked reads.
See the [manual checked-command guide](docs/EE871_E2_HIL_RUNNER.md#manual-checked-co2-diagnostics)
and [native IDF example guide](examples/idf/basic_bringup/README.md).

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

- [Software implementation prompts](https://github.com/janhavelka/EE871-E2/blob/main/docs/prompts/README.md) - executable software follow-ups, optional feature scopes, and final closeout.
- [Retained branch review](https://github.com/janhavelka/EE871-E2/blob/main/docs/RETAINED_BRANCH_AUDIT.md) - remaining HIL preflight gap, capability decisions, and retain/retire workflow.
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
