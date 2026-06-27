# Prompt: EE871-E2 Production API Hardening

Date: 2026-06-27

## Role

You are an AI coding agent working in the `EE871-E2` library repository.
Edit this repository only.

Implement the changes below exactly. Keep the design simple, synchronous,
bounded, framework-neutral, and production-friendly for an application that owns
the EE871 instance from one task/context.

## Read First

Read these files before editing:

- `AGENTS.md`
- `include/EE871/EE871.h`
- `include/EE871/Config.h`
- `include/EE871/Status.h`
- `include/EE871/CommandTable.h`
- `src/EE871.cpp`
- `test/test_basic.cpp`
- `test/support/FakeE2Transport.h`
- `docs/EE871_E2_Protocol_and_Register_Map.md`
- `docs/pdf-extracted-md/E2_interface_utilising_AN0105.md`
- `docs/pdf-extracted-md/EE871_E2_CO2_interface_AN1611-1.md`
- `docs/pdf-extracted-md/E2_interface_specification_v4_1.md`

Before implementation, record the current branch, current commit, current
`library.json` version, and baseline test/build result in the final report.

## Scope

Implement only these library changes:

1. optional absent-device startup,
2. stronger identity validation,
3. two checked CO2 sample helpers, one averaged and one fast,
4. separate checked-read example commands for averaged and fast reads,
5. cooperative long-delay callbacks for millisecond write-delay paths,
6. small cache-only settings additions,
7. focused private helper extraction where it directly supports this work,
8. tests, docs, version metadata, and final report updates.

Do not add a scheduler, async engine, RTOS task, application queue, sample
cache, dynamic allocation, Arduino dependency, ESP-IDF dependency, hardware I2C
owner, or board-specific pin setup.

Do not add new driver states. Keep the existing four-state model:

```cpp
UNINIT, READY, DEGRADED, OFFLINE
```

## Implementation Requirements

### 1. Add Absent-Device Startup

Add this enum in the public API:

```cpp
enum class BeginPolicy : uint8_t {
  RequirePresent = 0,
  AllowAbsent = 1,
};
```

Add this field to `EE871::Config`:

```cpp
BeginPolicy beginPolicy = BeginPolicy::RequirePresent;
```

Implement behavior:

- `RequirePresent` preserves current `begin()` behavior: if config validation,
  bus idle/reset, identity validation, or required startup probing fails,
  `begin()` returns the error and leaves the driver uninitialized.
- `AllowAbsent` still rejects invalid config and bus-stuck-after-reset errors.
- `AllowAbsent` accepts only presence/transport failures from identity
  validation. Do not accept incompatible identity or unsupported CO2 capability.
- For an accepted absent startup:
  - `begin()` returns `Status::Ok()`;
  - `isInitialized()` returns true;
  - `state()` returns `DriverState::OFFLINE`;
  - feature flags are zero;
  - `consecutiveFailures()` is set to normalized `offlineThreshold`;
  - `totalFailures()` is not incremented by accepted absent startup;
  - the original presence/transport failure is stored for diagnostics;
  - normal tracked bus APIs return `Err::BUSY` without touching E2 lines until
    `recover()` succeeds.
- `probe()` remains diagnostic/raw and does not update health counters.
- `recover()` remains callable from `OFFLINE`.
- After an accepted absent startup, a successful `recover()` must:
  - validate identity;
  - reload feature flags using the same feature-read sequence as successful
    `begin()`;
  - transition the driver to `READY`;
  - clear consecutive failures.

Add cache-only startup diagnostic state:

```cpp
Status beginProbeStatus = Status::Ok();
```

in `SettingsSnapshot`, backed by:

```cpp
Status _beginProbeStatus = Status::Ok();
```

Required meaning:

- `Status::Ok()` after a successful identity/presence startup path.
- The accepted presence/transport failure after `AllowAbsent` startup.
- Reset to `Status::Ok()` by `end()` and stopped-state reset.
- Do not use `beginProbeStatus` as a total-failure counter input.

### 2. Latch OFFLINE For Normal Operations

Implement an OFFLINE pre-bus guard for normal tracked E2 operations:

- If `_initialized == true` and `_driverState == DriverState::OFFLINE`, normal
  tracked bus APIs must return:

  ```cpp
  Status::Error(Err::BUSY, "Driver is offline; call recover()")
  ```

- The guard must return before touching E2 lines.
- The guard applies regardless of how `OFFLINE` was reached.
- The guard must not block `probe()`, `recover()`, `busReset()`, or
  `checkBusIdle()`.
- `recover()` may use a scoped private bypass for the tracked identity
  validation it performs.
- If `recover()` starts from `OFFLINE` and fails, reassert `OFFLINE` and ensure
  `consecutiveFailures >= offlineThreshold`.

Do not add a new `DEVICE_ABSENT` error. Use existing status codes and the
cached startup diagnostic.

### 3. Strengthen Identity Validation

Before declaring the driver `READY`, validate all required EE871 identity
fields:

- group ID equals `cmd::SENSOR_GROUP_ID`,
- subgroup ID equals `cmd::SENSOR_SUBGROUP_ID`,
- available measurements include `cmd::AVAILABLE_MEAS_MASK`.

Apply this validation in:

- `begin()`,
- `probe()`,
- `recover()`.

Use these status mappings:

- group mismatch:

  ```cpp
  Status::Error(Err::NOT_SUPPORTED, "Unexpected group id", group)
  ```

- subgroup mismatch:

  ```cpp
  Status::Error(Err::NOT_SUPPORTED, "Unexpected subgroup id", subgroup)
  ```

- missing CO2 available-measurement bit:

  ```cpp
  Status::Error(Err::NOT_SUPPORTED, "CO2 measurement not advertised", bits)
  ```

Presence/transport errors remain the underlying transport status.

Add a private identity result if useful:

```cpp
struct IdentitySnapshot {
  uint16_t group = 0;
  uint8_t subgroup = 0;
  uint8_t availableMeasurements = 0;
};
```

Keep it private to the implementation unless tests require a public type.

### 4. Add Checked CO2 Sample Helpers

Add this enum to the public API:

```cpp
enum class Co2ValueKind : uint8_t {
  Fast = 0,
  Average = 1,
};
```

Add this result struct:

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
  Status valueReadStatus{Status::Ok()};
  Status statusReadStatus{Status::Ok()};
  Status errorCodeReadStatus{Status::Ok()};
};
```

Add these public APIs:

```cpp
Status readCo2AverageSample(Co2ReadResult& out);
Status readCo2FastSample(Co2ReadResult& out);
```

Keep these existing APIs unchanged and source-compatible:

```cpp
Status readCo2Average(uint16_t& ppm);
Status readCo2Fast(uint16_t& ppm);
Status readStatus(uint8_t& status);
Status readErrorCode(uint8_t& code);
```

Add this private helper:

```cpp
Status _readCo2Sample(Co2ValueKind kind, Co2ReadResult& out);
```

Required `_readCo2Sample()` behavior:

1. Reset `out` to a known default and set `out.kind`.
2. Read the selected CO2 value first:
   - `Co2ValueKind::Average` uses MV4 through `readCo2Average(ppm)`.
   - `Co2ValueKind::Fast` uses MV3 through `readCo2Fast(ppm)`.
3. Store the raw ppm in `out.ppm`.
4. Store the value-read status in `out.valueReadStatus`.
5. If the value read fails, return that status with `ppmValid=false` and do not
   read status.
6. Read `readStatus(statusByte)` after the value read.
7. Store the status-read status in `out.statusReadStatus`.
8. If the status read fails, return that status with `ppmValid=false`.
9. Set `out.statusByte` and `out.statusValid=true`.
10. If status bit 3 indicates a CO2 error:
    - set `out.co2Error=true`;
    - if `hasErrorCode()` is true, call `readErrorCode(errorCode)`;
    - if `readErrorCode()` fails, return that communication status;
    - if the error code read succeeds, set `errorCode` and `errorCodeValid`;
    - return `Err::CO2_SENSOR_ERROR`;
    - keep `ppmValid=false`.
11. If status is clean, validate ppm against the documented EE871 range.
12. On success, set `ppmValid=true` and return `Status::Ok()`.

Add this error code append-only at the end of `Err`:

```cpp
CO2_SENSOR_ERROR
```

Do not renumber existing `Err` values.

Use these return mappings:

- status bit set and error code read succeeds:

  ```cpp
  Status::Error(Err::CO2_SENSOR_ERROR, "CO2 sensor status error", errorCode)
  ```

- status bit set and error-code support is not advertised:

  ```cpp
  Status::Error(Err::CO2_SENSOR_ERROR, "CO2 sensor status error", statusByte)
  ```

- status bit set and error-code read fails: return the error-code read status.

Add these constants to `CommandTable.h`:

```cpp
static constexpr uint16_t CO2_PPM_MIN = 0;
static constexpr uint16_t CO2_PPM_MAX = 50000;
```

Use these constants only in the new sample helpers. Do not change raw
`readCo2Average(uint16_t&)` or `readCo2Fast(uint16_t&)` range behavior.

If the selected value exceeds `CO2_PPM_MAX`, return:

```cpp
Status::Error(Err::OUT_OF_RANGE, "CO2 ppm out of range", ppm)
```

Set `out.ppm` to the raw value for diagnostics and keep `out.ppmValid=false`.

Do not read status before MV3/MV4 in the new sample helpers. The E+E
application-note sequence is: read required measured values first, then read
status so validity of the last measured values can be evaluated and the next
measurement can be started.

Keep the `readStatus()` documentation explicit: reading the status byte can
start or trigger a measurement and can reset the sensor interval counter under
the documented timing conditions.

Keep sensor-status errors separate from E2 bus health:

- Returning `Err::CO2_SENSOR_ERROR` must not increment E2 health failures or
  move `DriverState`.
- Returning `Err::OUT_OF_RANGE` from sample validation must not increment E2
  health failures or move `DriverState`.
- Failed E2 communication while reading value, status, or error code still
  updates health through the existing tracked transport path.

### 5. Add Cooperative Long-Delay Callbacks

Add optional callbacks to `Config.h`:

```cpp
using E2DelayMsFn = void (*)(uint32_t ms, void* user);
using E2YieldFn = void (*)(void* user);
```

Add these fields to `Config`:

```cpp
E2DelayMsFn delayMs = nullptr;
E2YieldFn yield = nullptr;
uint8_t longDelaySliceMs = 1;
```

Validation and normalization:

- `longDelaySliceMs == 0` normalizes to `1`.
- `longDelaySliceMs > 50` returns `Err::INVALID_CONFIG`.
- Existing maximum write-delay validation stays in place.

Replace the current millisecond sleep helper with a private helper such as:

```cpp
static void delayLongMs(const Config& cfg, uint32_t delayMs);
```

Required behavior:

- Loop in slices of at most `longDelaySliceMs`.
- If `cfg.delayMs` is present, call it once for each slice.
- If `cfg.delayMs` is absent, call `cfg.delayUs(slice * 1000, cfg.busUser)`.
- After each slice, call `cfg.yield(cfg.busUser)` when non-null.
- Use this helper only for millisecond write-delay paths.
- Do not call `yield` from bit-level E2 timing paths.

Keep all new delay math bounded in `uint32_t`.

### 6. Keep Timer Model 32-Bit

Do not convert EE871 library timing or timestamp fields to `uint64_t` in this
prompt.

Keep these as `uint32_t`:

- `tick(uint32_t nowMs)`,
- `SettingsSnapshot::nowMs`,
- `SettingsSnapshot::lastOkMs`,
- `SettingsSnapshot::lastErrorMs`,
- `_nowMs`,
- `_lastOkMs`,
- `_lastErrorMs`,
- `Config` timeout fields,
- `Config` write-delay fields,
- `delayLongMs(... uint32_t delayMs)`.

Reason: current timestamps are driver-local diagnostics, and current protocol
timers are bounded microsecond counters. This prompt adds no long-lived
scheduling inside the library.

Rules:

- Do not add sample timestamps.
- Do not add a `NowMsFn`.
- If any new elapsed check is required, keep it short-lived, bounded, and
  wrap-safe.

### 7. Extend Cache-Only Settings

Extend `SettingsSnapshot` with:

```cpp
BeginPolicy beginPolicy{BeginPolicy::RequirePresent};
Status beginProbeStatus{Status::Ok()};
bool hasDelayMs = false;
bool hasYield = false;
uint8_t longDelaySliceMs = 1;
```

Populate these fields from the normalized active config and cached runtime
state. `getSettings(SettingsSnapshot&)` and `getSettings()` must remain
cache-only and must not touch E2 lines.

Verify existing `driverState()` and `healthState()` aliases remain
source-compatible.

### 8. Extract Only Useful Private Helpers

Refactor only where it directly supports the requirements above.

Add these private helpers or equivalently named private helpers:

```cpp
Status _validateConfig(const Config& input, Config& normalized) const;
Status _busResetRaw();
Status _readIdentityRaw(IdentitySnapshot& out);
Status _readFeatureFlagsRaw();
Status _readCo2Sample(Co2ValueKind kind, Co2ReadResult& out);
```

Required behavior:

- `_validateConfig()` centralizes config checks and normalization.
- `_busResetRaw()` contains the bus idle/reset sequence shared by `begin()` and
  `busReset()`.
- Public `busReset()` still requires initialization.
- `_readIdentityRaw()` reads and validates group, subgroup, and available
  measurements without health updates.
- `_readFeatureFlagsRaw()` clears cached feature flags before reading them and
  leaves them zero if the feature-read sequence fails.
- `_readCo2Sample()` removes duplication between average and fast sample APIs.

Do not perform broad style-only refactors.

## Pattern Alignment Decisions

Apply these constraints:

- Public headers stay framework-neutral.
- Core code does not include Arduino, ESP-IDF, FreeRTOS, `Wire`, or hardware I2C
  driver types.
- Driver instances remain non-copyable and non-movable.
- `Config` uses non-owning callbacks and one user pointer.
- `Status` messages remain static strings.
- `probe()` remains raw and health-neutral.
- Normal bus operations remain tracked.
- `offlineThreshold == 0` normalizes to one.
- No heap allocation in core driver code.
- No dynamic STL containers in core driver code.
- No new generic dirty/uncertain state is needed.
- No generic measurement struct or cached sample state is needed.
- No extra driver state such as `ABSENT`, `PROBING`, `RECOVERING`, or
  `MEASUREMENT_NOT_READY` is needed.

## Tests

Use the existing native fake transport. Extend it only with small helpers needed
for assertions.

Add or update fake support for:

```cpp
void setPresent(bool present);
void setCo2FastPpm(uint16_t ppm);
void setCo2AveragePpm(uint16_t ppm);
void setStatusByte(uint8_t status);
void setErrorCode(uint8_t code);
void setFeatureFlags(uint8_t operatingFunctions,
                     uint8_t operatingModeSupport,
                     uint8_t specialFeatures);
uint32_t delayMsCalls() const;
uint32_t yieldCalls() const;
uint32_t controlReadCount(uint8_t mainCommandNibble) const;
uint32_t totalBusTransactions() const;
```

Required native tests:

1. Config defaults:
   - `beginPolicy == BeginPolicy::RequirePresent`
   - `delayMs == nullptr`
   - `yield == nullptr`
   - `longDelaySliceMs == 1`

2. Config validation:
   - `longDelaySliceMs == 0` normalizes to `1`
   - `longDelaySliceMs > 50` returns `Err::INVALID_CONFIG`
   - `offlineThreshold == 0` still normalizes to `1`

3. `begin()` with `RequirePresent` and absent fake:
   - returns non-OK
   - `isInitialized() == false`
   - `state() == DriverState::UNINIT`

4. `begin()` with `AllowAbsent` and absent fake:
   - returns OK
   - `isInitialized() == true`
   - `state() == DriverState::OFFLINE`
   - `consecutiveFailures() == offlineThreshold()`
   - feature flags are zero
   - `totalFailures() == 0`
   - `SettingsSnapshot::beginProbeStatus` contains the accepted absence error
   - a normal value read returns `Err::BUSY`
   - the normal value read does not increase fake bus transaction count

5. `AllowAbsent` rejects incompatible identity:
   - wrong group, wrong subgroup, or missing CO2 bit returns non-OK
   - driver remains uninitialized

6. `probe()` after absent startup:
   - remains callable
   - uses raw diagnostic reads
   - does not change health counters

7. `recover()` after absent startup:
   - fake starts absent
   - `begin(AllowAbsent)` succeeds offline
   - fake becomes present
   - `recover()` returns OK
   - `state() == DriverState::READY`
   - feature flags are populated
   - consecutive failures are zero

8. `recover()` failure from `OFFLINE`:
   - returns non-OK
   - remains `DriverState::OFFLINE`
   - `consecutiveFailures() >= offlineThreshold()`

9. Normal tracked operations while OFFLINE:
   - return `Err::BUSY`
   - do not touch E2 lines
   - work the same whether OFFLINE came from absent startup or health failures

10. Identity validation:
    - `begin()`, `probe()`, and `recover()` reject wrong group
    - `begin()`, `probe()`, and `recover()` reject wrong subgroup
    - `begin()`, `probe()`, and `recover()` reject missing CO2 bit

11. `readCo2AverageSample()` success:
    - returns OK
    - `kind == Co2ValueKind::Average`
    - `ppmValid == true`
    - selected ppm is returned
    - `statusValid == true`
    - `co2Error == false`
    - fake order shows MV4 low/high reads occur before `MAIN_STATUS`

12. `readCo2FastSample()` success:
    - returns OK
    - `kind == Co2ValueKind::Fast`
    - `ppmValid == true`
    - selected ppm is returned
    - `statusValid == true`
    - `co2Error == false`
    - fake order shows MV3 low/high reads occur before `MAIN_STATUS`

13. Value-read failure:
    - sample helper returns the underlying value-read status
    - `ppmValid == false`
    - `valueReadStatus` matches the returned status
    - status is not read after the value-read failure

14. CO2 status error with error-code support:
    - fake value read succeeds
    - fake status bit 3 is set
    - fake error code is one documented code
    - helper returns `Err::CO2_SENSOR_ERROR`
    - `co2Error == true`
    - `errorCodeValid == true`
    - `ppmValid == false`
    - E2 health failure counters do not increment because bus I/O succeeded

15. CO2 status error without error-code support:
    - fake value read succeeds
    - fake status bit 3 is set
    - feature bit for error-code support is cleared
    - helper returns `Err::CO2_SENSOR_ERROR`
    - `errorCodeValid == false`
    - E2 health failure counters do not increment because bus I/O succeeded

16. Error-code read communication failure:
    - fake value read succeeds
    - fake status bit 3 is set
    - error-code support is advertised
    - error-code read fails
    - helper returns the error-code read status
    - E2 health failure counters update through the tracked communication path

17. PPM out of documented range:
    - fake returns `50001`
    - status is clean
    - helper returns `Err::OUT_OF_RANGE`
    - `ppmValid == false`
    - `ppm == 50001`
    - E2 health failure counters do not increment because bus I/O succeeded

18. Explicit status behavior:
    - existing `readStatus()` still reads `MAIN_STATUS`
    - `hasCo2Error()` behavior remains covered
    - docs/help state that `status` can start or trigger a measurement

19. Separate checked-read commands:
    - Arduino example exposes `sampleavg`
    - Arduino example exposes `samplefast`
    - ESP-IDF example exposes `sampleavg`
    - ESP-IDF example exposes `samplefast`
    - `sampleavg` calls `readCo2AverageSample(Co2ReadResult&)`
    - `samplefast` calls `readCo2FastSample(Co2ReadResult&)`
    - existing `co2avg` and `co2fast` continue to call the raw value APIs
    - help text documents raw versus checked reads clearly
    - command-contract tests cover command presence and help output

20. Long delay callbacks:
    - configure `delayMs`, `yield`, and `longDelaySliceMs`
    - run a fake persistent-write path
    - assert millisecond delay callbacks are used in slices
    - assert yield callbacks are used after slices
    - assert normal bit-level reads do not call `yield`

21. Snapshot cache-only behavior:
    - `getSettings()` reports new fields
    - fake bus transaction count does not change while reading settings

22. Public header compatibility:
    - new enums and structs compile from public headers
    - existing code using `readCo2Average(uint16_t&)`,
      `readCo2Fast(uint16_t&)`, `readStatus(uint8_t&)`, and
      `readErrorCode(uint8_t&)` still compiles

## Docs, Examples, And Versioning

Update:

- `README.md`
- `CHANGELOG.md`
- release notes for the new version
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`
- `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` only to mark new HIL coverage
  as pending unless hardware was actually run
- Arduino and ESP-IDF example command implementations, help text, and docs for
  raw and checked CO2 reads
- command-contract tests that validate `sampleavg` and `samplefast`

Versioning requirements:

- Bump `library.json` to the next minor version from the current version.
- Regenerate `include/EE871/Version.h` with repository tooling.
- If the current version is `1.0.0`, create
  `docs/EE871_E2_RELEASE_NOTES_1.1.0.md`.
- Do not edit old release notes as if they are the current release.
- Update any repository version references that are intended to track the active
  release.

Example CLI requirements:

- Keep existing `status`, `co2avg`, and `co2fast` commands source-compatible.
- Add exactly these read-only checked-sample commands:
  - `sampleavg`
  - `samplefast`
- `sampleavg` and `samplefast` must use the new checked sample helpers.
- `co2avg` and `co2fast` must remain raw MV4/MV3 value reads.
- Keep help text clear that `status`, `sampleavg`, and `samplefast` can trigger
  a new measurement because they read status.
- Keep help text clear that `co2avg` and `co2fast` do not perform the checked
  status sequence.
- Do not add new field-write commands.

Docs must clearly separate:

- raw MV3/MV4 reads,
- checked average/fast sample reads that read value first and status second,
- explicit status reads with measurement-trigger side effects,
- maintenance/persistent-write APIs.

Do not add library-level field-use policy. Applications decide which maintenance
APIs they expose.

## Validation Commands

Run:

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

## Final Report

The implementation is complete when the final report states:

- files changed,
- current branch and commit used as baseline,
- version bump performed,
- test/build command results,
- whether `idf.py` was available,
- no HIL claim unless HIL was run,
- downstream integration decisions left to consuming firmware:
  - selected pins,
  - owner task policy,
  - warmup/read cadence,
  - whether and when to call raw value reads versus checked sample helpers,
  - storage/schema/UI exposure,
  - HIL timing thresholds.

## Acceptance Criteria

- Existing public APIs remain source-compatible.
- `BeginPolicy::AllowAbsent` supports configured offline startup.
- Normal tracked bus APIs return offline `BUSY` without touching E2 lines until
  recovery succeeds.
- `OFFLINE` is latched for normal operations regardless of how it was reached.
- `begin()`, `probe()`, and `recover()` validate group, subgroup, and CO2
  availability.
- `recover()` can transition from accepted absent startup to `READY`.
- Feature flags reload after recovery from absent startup.
- `readCo2AverageSample(Co2ReadResult&)` exists.
- `readCo2FastSample(Co2ReadResult&)` exists.
- Sample helpers read MV3/MV4 first and status second.
- Sample helpers do not read status if the value read fails.
- Sample helpers surface CO2 status-bit errors through append-only
  `Err::CO2_SENSOR_ERROR`.
- Sensor status and range validation failures do not count as E2 health
  failures when bus I/O succeeded.
- Sample helpers enforce `CO2_PPM_MAX`.
- Long write delays can use `delayMs` and `yield` callbacks.
- Bit-level E2 timing paths do not call `yield`.
- `getSettings()` exposes new cache-only state and remains bus-free.
- Native tests cover defaults, identity validation, absent startup, offline
  guard, recovery, checked CO2 reads, ordering, sensor status errors, range
  validation, long delays, and cache-only settings.
- Docs separate raw value reads, checked sample reads, status side effects, and
  maintenance writes.
