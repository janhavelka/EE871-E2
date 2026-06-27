# Prompt: EE871-E2 TunnelMonitor Production Fit Hardening

Date: 2026-06-27

## Role

You are an AI coding agent working in the `EE871-E2` library repository.
Your job is to make the EE871-E2 library a clean, simple, production-friendly
fit for later use inside TunnelMonitor as a dedicated E2/EE871 owner task.

Do not edit the TunnelMonitor firmware repository in this prompt. This prompt
is for the EE871-E2 library only.

## Repositories And Context

Primary repository to edit:

```text
C:\Users\HonzovoSpectre\Documents\Projects\EE871-E2
```

Compatibility target repository to audit but not edit:

```text
C:\Users\HonzovoSpectre\Documents\Projects\TunnelMonitor-node
```

Relevant sibling library patterns to compare against:

```text
C:\Users\HonzovoSpectre\Documents\Projects\RV3032-C7
C:\Users\HonzovoSpectre\Documents\Projects\MB85RC
C:\Users\HonzovoSpectre\Documents\Projects\BME280
C:\Users\HonzovoSpectre\Documents\Projects\SHT3x-main
C:\Users\HonzovoSpectre\Documents\Projects\INA228
C:\Users\HonzovoSpectre\Documents\Projects\SSD1315
```

Current EE871-E2 baseline found during planning:

- library version: `1.0.0`
- branch: `main`
- commit observed during planning: `a395df2`
- native tests observed passing: `31/31`
- core driver already uses injected E2 GPIO callbacks, not Arduino `Wire`,
  ESP-IDF hardware I2C, or owned bus objects.

## High-Level Goal

Make EE871-E2 a better fit for a TunnelMonitor production owner task by adding:

1. absent-device startup support,
2. two production CO2 read helpers, one for averaged CO2 and one for fast CO2,
3. cooperative long-delay support for write-delay paths,
4. small diagnostic/state additions needed for robust owner integration,
5. focused docs and tests.

Prefer simple, safe, robust changes. Reuse the existing code, private helpers,
fake transport, status model, and docs structure. Do not introduce a framework,
generic scheduler, dynamic allocation, or firmware-specific abstractions.

## Non-Goals

Do not implement TunnelMonitor firmware integration in this prompt:

- no `Ee871Task` in TunnelMonitor,
- no `BoardPins` changes,
- no PlatformIO dependency changes in TunnelMonitor,
- no sample schema, CSV, replay, cloud, web, CLI, or settings changes in
  TunnelMonitor,
- no hardware HIL execution unless explicitly requested by the operator.

Do not add library-level field-mode write lockout in this prompt. The current
decision is that TunnelMonitor will control which library APIs it calls. Keep
the existing persistent-write APIs, but document them clearly as maintenance
operations and keep dirty/resync diagnostics intact.

Do not convert every timestamp to 64-bit. Treat EE871 like the sibling I2C
libraries: use 32-bit millisecond timing where the timestamp is only for short
driver-local intervals or diagnostics. TunnelMonitor can translate its 64-bit
owner deadlines outside the library.

Do not create an async/nonblocking protocol engine. The target is still a
managed synchronous library that can run inside a dedicated owner task.

## Current Gaps To Close

### Gap 1: `begin()` Cannot Support Optional/Absent Startup

Current behavior:

- `begin()` validates callbacks, probes the device, caches feature flags, and
  fails if the device is absent.
- `probe()` and `recover()` require successful `begin()`.

TunnelMonitor fit issue:

- A production owner task must be able to start with the probe absent, keep the
  callbacks configured, report the device as absent/offline, and later recover
  without rebooting the owner.

Required design:

Add an explicit begin policy:

```cpp
enum class BeginPolicy : uint8_t {
  RequirePresent = 0,
  AllowAbsent = 1,
};
```

Add to `EE871::Config`:

```cpp
BeginPolicy beginPolicy = BeginPolicy::RequirePresent;
```

Behavior:

- `RequirePresent` preserves current behavior exactly: absent or identity
  failure returns the failure status and leaves the driver uninitialized.
- `AllowAbsent` validates and stores callbacks/config even if the identity probe
  fails.
- In `AllowAbsent`, when the device is absent:
  - `begin()` returns `Status::Ok()`;
  - `isInitialized()` returns true;
  - `state()` returns `DriverState::OFFLINE`;
  - feature flags are zero;
  - the root absent/probe status is retained in a new cache-only diagnostic
    field;
  - normal tracked bus operations return the existing offline `BUSY` status
    until `recover()` succeeds;
  - `probe()` remains diagnostic/raw and may be called after absent startup;
  - `recover()` must be able to transition the driver from this configured
    offline state to `READY`.

Add a cached diagnostic:

```cpp
Status beginProbeStatus = Status::Ok();
```

to `SettingsSnapshot`, backed by a private member:

```cpp
Status _beginProbeStatus = Status::Ok();
```

Meaning:

- `Status::Ok()` when the last `begin()` identity/presence path succeeded.
- The raw identity/probe failure when `AllowAbsent` accepted startup despite
  the absent probe.
- Reset by `end()` and reset-runtime state.

Do not use `beginProbeStatus` for health counters. It is a cache-only startup
diagnostic.

Important recovery requirement:

- `recover()` must not only read the group ID. If the driver started absent and
  feature flags are zero, a successful `recover()` must reload feature flags
  using the same feature-read sequence currently used by `begin()`.

### Gap 2: Missing Production CO2 Read Helpers

Current behavior:

- `readStatus(uint8_t&)` reads status and can trigger a measurement.
- `readCo2Fast(uint16_t&)` reads MV3 fast response.
- `readCo2Average(uint16_t&)` reads MV4 averaged response.
- The application must manually sequence status, optional error code, and value
  reads.

TunnelMonitor fit issue:

- The owner task should call one simple API for the normal production read.
- The user decision is to add two explicit APIs, one averaged and one fast.

Add this enum to `EE871.h`:

```cpp
enum class Co2ValueKind : uint8_t {
  Fast = 0,
  Average = 1,
};
```

Add this result struct to `EE871.h`:

```cpp
struct Co2ReadResult {
  Co2ValueKind kind{Co2ValueKind::Average};
  uint16_t ppm{0};
  bool ppmValid{false};
  uint8_t statusByte{0};
  bool statusValid{false};
  bool co2Error{false};
  uint8_t errorCode{0};
  bool errorCodeValid{false};
  Status statusReadStatus{Status::Ok()};
  Status valueReadStatus{Status::Ok()};
  Status errorCodeReadStatus{Status::Ok()};
};
```

Add these public APIs:

```cpp
Status readCo2AverageSample(Co2ReadResult& out);
Status readCo2FastSample(Co2ReadResult& out);
```

Keep existing APIs unchanged:

```cpp
Status readCo2Average(uint16_t& ppm);
Status readCo2Fast(uint16_t& ppm);
```

Add a private helper to avoid duplicate code:

```cpp
Status _readCo2Sample(Co2ValueKind kind, Co2ReadResult& out);
```

Required sequencing for `_readCo2Sample()`:

1. Clear `out` to a known default and set `out.kind`.
2. Call `readStatus(statusByte)` first.
   - Document that reading status can trigger a new measurement in the slave.
   - This is intentional because the status byte describes the last CO2
     measurement.
3. If the status read fails:
   - set `statusReadStatus`;
   - return that status;
   - do not read the value.
4. If status bit 3 indicates a CO2 error:
   - set `statusValid=true`, `co2Error=true`;
   - if `hasErrorCode()` is true, read `readErrorCode(errorCode)`;
   - return a sensor-error status, not `Status::Ok()`;
   - do not mark `ppmValid=true`.
5. If status is OK, read the selected value:
   - `Fast` uses `readCo2Fast(ppm)`;
   - `Average` uses `readCo2Average(ppm)`.
6. Validate the value against the documented EE871 E2 range.
7. On success:
   - set `statusValid=true`;
   - set `ppmValid=true`;
   - return `Status::Ok()`.

Add this new error code append-only at the end of `Err`:

```cpp
CO2_SENSOR_ERROR
```

Do not renumber existing `Err` values. Existing implicit numeric values must
remain stable.

Recommended mapping:

- If CO2 status bit is set and `readErrorCode()` succeeds:
  - return `Status::Error(Err::CO2_SENSOR_ERROR, "CO2 sensor status error",
    errorCode)`.
- If CO2 status bit is set but error-code support is not advertised:
  - return `Status::Error(Err::CO2_SENSOR_ERROR, "CO2 sensor status error",
    statusByte)`.
- If CO2 status bit is set and `readErrorCode()` fails:
  - return the `readErrorCode()` status because the communication failed while
    collecting the diagnostic.

Add documented range constants to `CommandTable.h`:

```cpp
static constexpr uint16_t CO2_PPM_MIN = 0;
static constexpr uint16_t CO2_PPM_MAX = 50000;
```

Use these only in the new sample helpers. Do not change the existing raw
`readCo2Fast(uint16_t&)` or `readCo2Average(uint16_t&)` behavior beyond any
bug fixes required by tests.

If the selected value exceeds `CO2_PPM_MAX`, return:

```cpp
Status::Error(Err::OUT_OF_RANGE, "CO2 ppm out of range", ppm)
```

with `ppmValid=false`.

### Gap 3: Long Delay Paths Busy-Wait Too Hard

Current behavior:

- `sleepMs()` loops over `cfg.delayUs(1000, ...)`.
- This is bounded, but it keeps the owner task busy during persistent write
  delays.

TunnelMonitor fit issue:

- A dedicated owner task can tolerate bounded blocking, but long maintenance
  write delays should allow cooperative yielding or RTOS delay integration.

Add optional callbacks to `Config.h`:

```cpp
using E2DelayMsFn = void (*)(uint32_t ms, void* user);
using E2YieldFn = void (*)(void* user);
```

Add fields:

```cpp
E2DelayMsFn delayMs = nullptr;
E2YieldFn yield = nullptr;
uint8_t longDelaySliceMs = 1;
```

Validation:

- `longDelaySliceMs == 0` normalizes to `1`.
- `longDelaySliceMs > 50` returns `Err::INVALID_CONFIG`.

Replace `sleepMs()` with a helper such as:

```cpp
static void delayLongMs(const Config& cfg, uint32_t delayMs);
```

Behavior:

- Loop in slices of at most `longDelaySliceMs`.
- If `cfg.delayMs` is present, call it for the slice.
- Otherwise call `cfg.delayUs(slice * 1000, cfg.busUser)`.
- After each slice, call `cfg.yield(cfg.busUser)` if non-null.
- Keep the existing bounded maximum write-delay validation.

Do not call `yield` from bit-level E2 timing paths. Only long millisecond write
delays should use it.

### Gap 4: Small State And Settings Consistency Gaps

Add the following if missing or incomplete:

```cpp
DriverState driverState() const { return state(); }
```

`healthState()` already exists. Keep it.

Extend `SettingsSnapshot` so it exposes all new cache-only state:

```cpp
BeginPolicy beginPolicy{BeginPolicy::RequirePresent};
Status beginProbeStatus{Status::Ok()};
bool hasDelayMs = false;
bool hasYield = false;
uint8_t longDelaySliceMs = 1;
```

Keep `getSettings(SettingsSnapshot&)` strictly cache-only. It must not touch
E2 lines.

### Gap 5: Duplicate Code That Should Be Simplified

Refactor only where it directly supports this prompt:

- Extract config validation from `begin()` into a private helper:

  ```cpp
  Status _validateConfig(const Config& config, Config& normalized) const;
  ```

- Extract bus idle/reset code shared by `begin()` and `busReset()` into a
  private helper:

  ```cpp
  Status _busResetRaw();
  ```

  Keep public `busReset()` requiring initialization.

- Extract identity probe into:

  ```cpp
  Status _readIdentityRaw(uint16_t& group);
  ```

- Extract feature flag read into:

  ```cpp
  Status _readFeatureFlagsRaw();
  ```

- Add `_readCo2Sample()` so the averaged and fast public APIs are wrappers only.

Do not perform broad style-only refactors.

## Sibling Library Pattern Alignment

Match the sibling driver conventions where practical:

- Public headers remain framework-neutral and free of Arduino, ESP-IDF,
  FreeRTOS, `Wire`, and owned bus objects.
- Driver instances remain non-copyable and non-movable.
- `Config` uses non-owning callbacks and `void* busUser`.
- `Status` uses static strings only.
- `probe()` remains diagnostic/raw and does not update health counters.
- Normal tracked operations update health counters.
- `DriverState` remains:

  ```cpp
  UNINIT, READY, DEGRADED, OFFLINE
  ```

- `offlineThreshold` zero normalizes to one.
- `getSettings()` remains cache-only.
- No heap allocation in core driver code.
- No dynamic STL containers in core driver code.
- `tick(uint32_t nowMs)` stays void and bounded.
- Use wrap-safe 32-bit elapsed checks for driver-local short intervals if new
  interval logic is added.

## TunnelMonitor Compatibility Audit To Preserve In The Prompt Output

The implementation report should mention these downstream TunnelMonitor gaps,
but this prompt must not implement them:

- EE871 is E2 over GPIO, not ESP32 hardware I2C. It must not be integrated as a
  normal `I2cTask` device unless a future architecture decision explicitly
  makes `I2cTask` own those lines.
- If EE871 gets its own TunnelMonitor owner task, it needs dedicated E2 GPIOs
  in `BoardPins`.
- TunnelMonitor needs a driver-free public contract such as
  `include/TunnelMonitor/contracts/Ee871.h`.
- Suggested future TunnelMonitor names:
  - `Ee871Task`
  - `DeviceId::Ee871`
  - `ServiceId::Ee871Task`
  - `BusId::E2`
  - `ErrorDomain::Ee871`
  - `Ee871Operation::Probe`, `ReadAverage`, `ReadFast`, `ReadStatus`,
    `BusRecover`
  - `Ee871ResultStatus::Ok`, `Queued`, `Timeout`, `DeviceAbsent`,
    `SensorError`, `BusStuck`, `Failed`
  - `Ee871Co2Mode::Average`, `Fast`
- Suggested future TunnelMonitor owner defaults for first integration:
  - `kEe871RequestQueueDepth = 8`
  - `kEe871ResultQueueDepth = 8`
  - `kEe871TraceCapacity = 16`
  - `kEe871RuntimeTaskStackBytes = 8192`, verify with HIL high-water
  - `kEe871RuntimeTaskPriority = 2`
  - `kEe871RuntimePollIntervalMs = 20`
  - `kEe871ReadDeadlineMs = 2000`
  - `kEe871WarmupDeadlineMs = 20000`
  - `kEe871PowerUpMaxMs = 10000`
  - `kEe871MeasurementMs = 700`
  - `kEe871MinimumTriggerIntervalMs = 10000`
  - `kEe871StatusStaleMs = 30000`
  - `kEe871StatusFaultMs = 120000`
- TunnelMonitor sample schema currently has no CO2 fields. Later integration
  must decide whether to store:
  - averaged CO2 only,
  - fast CO2 only,
  - both averaged and fast CO2,
  - status/error code as numeric fields or diagnostics only.
- Suggested future append-only sample fields if both are stored:
  - `SampleFieldId::Co2AveragePpm = 37`
  - `SampleFieldId::Co2FastPpm = 38`
  - `SampleFieldId::Count = 39`
- If CO2 is added to stored samples, TunnelMonitor must update CSV header,
  replay JSON builder, cloud body format, native contract enum tests,
  `MeasurementAssembler`, `MeasurementRuntime`, web status, and CLI surfaces.
- Future profile name if both existing data and CO2 are stored:

  ```text
  tm.v1.vw8_shzk16_env_power_co2
  ```

## Open Questions And Decisions For Later

Do not block this library prompt on these. List them in the implementation
report as downstream decisions.

1. Dedicated E2 pins for TunnelMonitor hardware revision 2.0.0.
2. Whether EE871 is optional or required for aggregate health.
3. Whether TunnelMonitor stores average CO2 only or both average and fast CO2.
4. Whether the owner task powers the EE871 probe or only reads it.
5. Whether the owner task should expose maintenance writes at all.
6. Exact TunnelMonitor sample schema/profile bump.
7. Exact web/CLI wording and settings persistence fields.
8. HIL timing table and stack high-water evidence.

## Tests To Add Or Update

Use the existing native fake transport. Extend it only as needed with small
test helpers:

```cpp
void setStatusByte(uint8_t status);
void setCo2FastPpm(uint16_t ppm);
void setCo2AveragePpm(uint16_t ppm);
void setFeatureFlags(uint8_t operatingFunctions,
                     uint8_t operatingModeSupport,
                     uint8_t specialFeatures);
uint32_t delayMsCalls() const;
uint32_t yieldCalls() const;
```

Required native tests:

1. `Config` defaults:
   - `beginPolicy == BeginPolicy::RequirePresent`
   - `delayMs == nullptr`
   - `yield == nullptr`
   - `longDelaySliceMs == 1`

2. `begin()` with `RequirePresent` and absent fake preserves current failure
   behavior:
   - returns non-OK,
   - `isInitialized() == false`,
   - `state() == UNINIT`.

3. `begin()` with `AllowAbsent` and absent fake:
   - returns OK,
   - `isInitialized() == true`,
   - `state() == OFFLINE`,
   - `SettingsSnapshot::beginProbeStatus` contains the absent failure,
   - feature flags are zero,
   - normal reads return offline `BUSY`.

4. `recover()` after absent startup:
   - fake starts absent, `begin(AllowAbsent)` succeeds offline,
   - fake is made present,
   - `recover()` returns OK,
   - `state() == READY`,
   - feature flags are populated.

5. `readCo2AverageSample()` success:
   - status byte is clean,
   - averaged ppm is valid,
   - `kind == Average`,
   - `ppmValid == true`,
   - `statusValid == true`,
   - return OK.

6. `readCo2FastSample()` success:
   - status byte is clean,
   - fast ppm is valid,
   - `kind == Fast`,
   - return OK.

7. CO2 status error with error-code support:
   - fake status bit 3 set,
   - fake error code set to one documented code,
   - helper returns `Err::CO2_SENSOR_ERROR`,
   - `co2Error == true`,
   - `errorCodeValid == true`,
   - `ppmValid == false`.

8. CO2 status error without error-code support:
   - fake status bit 3 set,
   - feature bit for error code cleared,
   - helper returns `Err::CO2_SENSOR_ERROR`,
   - `errorCodeValid == false`.

9. PPM out of documented range:
   - fake returns `50001`,
   - helper returns `Err::OUT_OF_RANGE`,
   - `ppmValid == false`.

10. Long delay callbacks:
    - configure `delayMs`, `yield`, and `longDelaySliceMs`,
    - run a write-delay path using zero-risk fake persistent write,
    - assert delay/yield callbacks are used,
    - assert bit-level normal reads do not call `yield`.

11. Snapshot cache-only behavior:
    - `getSettings()` reports new fields,
    - no fake bus transaction count changes while reading settings.

12. Header/API compile tests:
    - new enums/structs compile from public headers,
    - existing code using `readCo2Average(uint16_t&)` and
      `readCo2Fast(uint16_t&)` still compiles.

## Docs And Examples

Update:

- `README.md`
- `docs/EE871_E2_RELEASE_NOTES_1.0.0.md` or create a new release-note draft if
  version metadata is bumped
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`
- `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` only to say the new APIs need
  HIL evidence later, not to claim HIL passed
- Arduino and IDF example CLI docs where they describe safe commands

Example CLI updates are allowed if kept small:

- keep existing `read`, `co2fast`, `co2avg`, and `status`;
- optionally add `sampleavg` and `samplefast` to print the new
  `Co2ReadResult` struct fields;
- do not add new field-write commands.

## Validation Commands

Run the relevant checks after implementation:

```powershell
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
git diff --check
```

If `idf.py` is available, also run the pure ESP-IDF example build. If it is not
available, state that clearly.

Do not claim hardware HIL passed unless it was actually run with captured
artifacts.

## Acceptance Criteria

The implementation is complete when:

- existing public APIs remain source compatible;
- absent startup is supported through `BeginPolicy::AllowAbsent`;
- absent startup can later recover to `READY` without rebuilding the driver;
- feature flags reload after recovery from absent startup;
- two new production read helpers exist:
  - `readCo2AverageSample(Co2ReadResult&)`
  - `readCo2FastSample(Co2ReadResult&)`
- CO2 status bit and optional error code are surfaced consistently;
- documented CO2 ppm range is enforced by the new sample helpers;
- long write delays can yield/cooperate through optional callbacks;
- `getSettings()` exposes the new cache-only state and remains bus-free;
- native fake tests cover success, absence, recovery, CO2 sensor-error, range,
  and long-delay behavior;
- docs clearly separate safe reads from maintenance writes;
- the final report lists TunnelMonitor downstream integration decisions without
  claiming they were implemented.

