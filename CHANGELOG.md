# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Complete Prompt 04 safe HIL coverage for raw/checked samples, status,
  feature/cache consistency, bus levels, mixed stress, recovery, and resync.
- Structured checked-sample, full mutation-diagnostic, typed-setting, and
  complete 256-byte custom-memory parsers/validators.
- Atomic per-step HIL checkpoints, pre-transmission destructive-command
  journaling, and forensic custom-memory baseline JSON/hex artifacts.
- Separately authorized runner plans for reversible configuration, calibration,
  same-object bus-address candidate reconciliation/restoration, one-shot
  auto-adjust observation, sensor-only power-cycle, and distinct SDA/SCL
  stuck-line faults.
- Exact `partnamehex` read/write and `addr rebegin` maintenance commands in the
  Arduino and native ESP-IDF diagnostic examples.

### Changed

- `ALLOW_ABSENT` now accepts only authoritative `DEVICE_NOT_FOUND`; GPIO E2
  identity NACK remains NACK under both policies and leaves the driver
  uninitialized because measurement-priority NACK cannot prove absence.
- Capability bytes `0x03..0x09` now validate their documented reserved-zero
  masks before atomic publication. Recovery semantic incompatibility clears
  live claims and latches `OFFLINE` from READY, DEGRADED, or OFFLINE.
- Typed address, interval, factor, operating-mode, and auto-adjust procedures
  now share fail-closed validators across normal reads, writes, post-write
  observations, unresolved-target resync, and full resync. Filter remains an
  opaque vendor-specific byte.
- Diagnostic CLI help now labels arbitrary custom-memory writes as expert
  maintenance requiring authoritative address and restoration semantics.
- Persistent runner tests now use typed test/readback/diagnostic, complete
  post-test image, restore, and final-image sequences; raw memory is never
  replayed.
- Hazardous live plans require structured board, target, sensor, fixture,
  operator, and electrical-authority metadata and cannot be combined in one
  run. Calibration and ordinary configuration mutations are separate runs.
- Example self-tests capability-skip optional settings, and feature commands
  report the final precise status across all three feature reads.

### Fixed

- Invalid persisted typed values no longer publish to caller outputs or settle
  mutation uncertainty. Semantic failures preserve raw detail and original
  mutation cause without being counted as transport failures.
- Zero CO2 interval factor, D8/D9 reserved bits, and unadvertised defined D8
  modes now fail before unsafe mutation; protected `customWrite()` routing
  cannot bypass these checks.
- Exact documented 150 ms `0x10`/`0x50` and 300 ms interval-commit
  device-held-low completion limits now succeed; final-ACK and STOP polling
  share one sensor allowance while their bounded master waveform is accounted
  separately in public blocking bounds.
- Diagnostic CLI persistent numeric input now rejects malformed and
  out-of-range values instead of converting them to zero or wrapping before a
  typed write.
- A failed/uncertain destructive HIL step, stale dirty observation, missing
  baseline, or failed verification now blocks every later write in that run.
- Address, auto-adjust, and physical fault phases now require each preceding
  diagnostic/operator transition to pass; skipped or mistyped physical steps
  abort instead of advancing.
- Checked-sample validation now rejects MV-kind, status-bit, error-detail, and
  sensor-enum contradictions, and compares adjacent transport health counters.
- Mutation validation now checks exact target register ranges and detects
  transient writes outside the selected target before restoration.
- Typed writes now require a baseline that the same typed API can restore;
  serial/parser exceptions emit final FAIL artifacts while preserving
  destructive in-flight uncertainty.

## [1.1.0] - 2026-07-28

### Added

- Optional task-context `delayMs`/`yield` callbacks with bounded long-delay
  slicing.
- Bus-silent `operationTimingBound()` admission queries and public
  `OperationKind`/`OperationTimingBound` types.
- `VERIFY_MISMATCH` for completed writes whose readback differs.
- Phase-aware native stretch injection and transaction-order instrumentation.
- `BeginPolicy`, `DeviceIdentity`, and `CapabilitySnapshot` public contracts
  for strict or optional-device startup with cache-only diagnostics.
- `Err::OFFLINE` and lifecycle timing kinds for begin, probe, and complete
  recovery admission bounds.
- Checked MV3/MV4 procedures with `Co2ReadResult`, `Co2ValueKind`,
  `Co2SensorError`, append-only `CO2_SENSOR_ERROR`, and per-step attempt/status
  evidence.
- Broad checked CO2 range constants, cache-only calibration capability
  helpers, and checked-sample timing kinds 13 and 14.
- `MutationTarget`, `MutationEffect`, `MutationDiagnostic`, cache-only
  `mutationDiagnostic()`, and the narrow
  `acknowledgeAutoAdjustUncertainty()` procedure.
- `PERSISTENT_STATE_UNCERTAIN = 18` and persistent/maintenance timing kinds
  15 through 18.
- An exhaustive public timing-method map and
  `tools/check_public_timing_contract.py` source audit.
- Checked `sampleavg` and `samplefast` commands with equivalent Arduino and
  native ESP-IDF output.

### Changed

- Centralized timing validation now enforces the E2 500..5000 Hz envelope,
  25 ms bit limit, 35 ms byte limit, and safe long-delay slice range.
- `0x10` and `0x50` writes use one 150 ms completion budget; the committing
  interval high byte uses one 300 ms pair-completion budget. Configured values
  below those protocol minima normalize upward.
- Custom-pointer completion is ordered before every dependent `0x51` read, and
  block reads use pointer auto-increment.
- Startup now validates group, subgroup, CO2 availability, and all capability
  bytes `0x03..0x09`; identity and capabilities publish only after complete
  success.
- Optional startup accepts only definite absence/NACK and enters a latched
  `OFFLINE` state. Normal transfers are bus-silent while offline, raw `probe()`
  is non-mutating, and explicit `recover()` is the sole route back to `READY`.
- Raw MV3/MV4/status/error-code APIs remain unchanged. Checked procedures read
  value before side-effecting status, capability-gate detailed error reads,
  and keep sensor/range outcomes separate from E2 transport health.
- All effectful custom-memory APIs now use one admission, frame-completion,
  effect-classification, fixed intent, and diagnostic path. Further mutations
  are bus-silently blocked while evidence is unresolved.
- Persistent resync is target-specific when uncertainty exists and otherwise
  performs a complete capability-aware coherence read.
- Bus-address changes retain an explicit candidate for application-owned
  end/power/rebegin reconciliation; auto-adjust uses pre/post status evidence
  and is never replayed automatically.
- Typed optional-setting reads and writes now require validated cached
  capability support before bus I/O, including offset/gain calibration.
- Version tooling synchronizes the generated header, ESP-IDF component
  metadata, and Doxygen project number from `library.json`.
- Version tooling is library-local and no longer carries dormant
  TunnelMonitor dependency-header generation.

### Fixed

- SDA low before START is reported as `BUS_STUCK` without creating a false
  START.
- Legal final-ACK/STOP clock stretching no longer fails under ordinary byte
  timing, while ordinary transfers retain their tighter limits.
- Cleanup failures preserve an earlier, more precise transport failure.
- Ordinary STOP timing applies `bitTimeoutUs` only to SCL-high polling, so
  valid configured STOP hold times do not create false timeouts.
- Public diagnostic `busReset()` remains health-neutral; tracked recovery stays
  explicit through `recover()`.
- Final-PEC NACK cleanup uses the same write-completion budget and preserves
  `NACK` as the primary result.
- Optional startup accepts an identity NACK only when its cleanup STOP
  completes; a NACK with failed cleanup remains a failed, uninitialized begin.
- Raw custom writes can no longer bypass typed interval, address, calibration,
  auto-adjust, or documented read-only-address safety.
- Unresolved mutation evidence now survives `end()`, failed/repeated begin, and
  a later successful begin on the same object.
- Interval writes no longer attempt invalid low-byte equality verification
  before the deferred pair commit.
- Multi-byte verification observes every target element before returning the
  first mismatch, so deferred interval verification always compares both
  bytes.
- Mutation effect classification preserves final ACK/NACK evidence sampled
  before a later completion-deadline failure.
- Unresolved mutation admission retains
  `PERSISTENT_STATE_UNCERTAIN` precedence while the driver is offline, and
  operating-mode resync remains capability-gated before bus I/O.
- Raw signed interval-factor and CO2-offset decoding is deterministic under
  C++17, and invalid typed mutation arguments retain precise range errors even
  when the corresponding optional capability is absent.

## [1.0.0] - 2026-06-02

### Added
- Framework-neutral EE871-E2 core with injected GPIO-style E2 callbacks.
- Arduino and native ESP-IDF diagnostic/basic bring-up examples.
- Deterministic native fake E2 transport for host-side runtime fault injection.
- Runtime fault tests for stuck SCL timeout, PEC mismatch, device absence,
  write verify mismatch, offline/recover, and probe health side effects.
- Persistent configuration dirty diagnostics:
  `persistentConfigDirty()`, `persistentConfigDirtyError()`,
  `resyncPersistentConfig()`, and matching `SettingsSnapshot` fields.
- Arduino and ESP-IDF diagnostic CLI commands for `dirty` and `resync`.
- Pure ESP-IDF GitHub Actions `idf-build` matrix job for `esp32s3` and
  `esp32s2`, plus an IDF example contract checker.
- `tools/ee871_hil_runner.py`, a Python serial HIL evidence runner with safe,
  extended-safe, persistent-write opt-in, and operator-fault plans.
- Hardware validation matrix, HIL runner documentation, documentation index, and
  1.0.0 release notes.
- `SettingsSnapshot`, `getSettings()`, `isInitialized()`, `getConfig()`,
  `driverState()`, `healthState()`, and `offlineThreshold()` for cache-only
  runtime and health inspection.
- Command-table helpers for supported main-command read checks and CO2
  error-code names.

### Changed
- Public docs clarify that EE871-E2 uses GPIO-style E2 signaling, not Arduino
  `Wire`, ESP-IDF `driver/i2c_master`, or a hardware I2C peripheral.
- Public API contracts document blocking behavior, timing bounds,
  thread-safety, ISR-safety, callback restrictions, and shared-bus
  serialization requirements.
- `EE871::EE871` is non-copyable and non-movable.
- `SettingsSnapshot` includes persistent dirty diagnostics.
- Package/docs describe the driver as managed synchronous and bounded rather
  than non-blocking.
- `library.json` and `idf_component.yml` now advertise both Arduino and ESP-IDF
  framework/component support.
- ESP-IDF port documentation describes the interactive diagnostic CLI and
  validation checklist.
- `Config::offlineThreshold = 0` normalizes to one, and `begin()` / `end()`
  reset stale cached runtime and feature state.
- High-level optional-feature helpers consistently return `NOT_INITIALIZED`
  before parameter or capability checks when called before `begin()`.
- `writeOperatingMode()` validates unsupported bit fields before capability
  checks.

### Fixed
- Byte-timeout accounting in E2 bit helpers uses saturating arithmetic and avoids
  overflow in the elapsed-time accumulator.
- Unsupported EE871 main-command reads return `NOT_SUPPORTED` before E2 traffic,
  including two-byte reads.
- `IN_PROGRESS` statuses are neutral for health tracking instead of counting as
  communication failures.
- Dirty-state tracking covers first-byte accepted/readback-failed cases for
  multi-byte persistent writes.

### Validation
- Native tests: 31 passing.
- Arduino ESP32-S3/S2 PlatformIO builds: passing in local readiness runs.
- ESP32-S3 safe HIL: PASS.
- ESP32-S3 extended safe HIL: PASS.
- ESP32-S3 persistent interval write/readback/restore: PASS.
- Physical unplug/replug recovery: PASS, operator-confirmed manual test with no
  automated transcript artifact.
- Pure ESP-IDF build: CI coverage is configured, but local `idf.py` and GitHub
  Actions proof remain unverified in this workspace.
- Remaining unrun items: ESP32-S2 hardware HIL, pure ESP-IDF hardware HIL,
  stuck-line fault-jig tests, power-cycle persistence, CO2 calibration writes,
  and bus-address write/recovery.

### Compatibility
- Source compatibility break: code that copies or moves `EE871::EE871`
  instances by value must keep drivers in stable storage and pass references or
  pointers instead.
- `SettingsSnapshot` layout changed. ABI/layout-sensitive users must rebuild
  and should not persist or externally share raw snapshot layouts.
- Normal one-instance users should only need to rebuild.

### Known Limitations
- Physical fault-jig validation is incomplete.
- Pure ESP-IDF build proof depends on a passing GitHub Actions matrix or local
  ESP-IDF environment.
- CO2 calibration writes were intentionally not tested on hardware.
- Power-cycle persistence was not proven.

## [0.3.0] - 2026-03-01

### Changed
- Refreshed `docs/IDF_PORT.md` so the ESP-IDF migration guidance matches current implementation details.

### Removed
- Outdated unification template document no longer representing the active workflow.

## [0.2.1] - 2026-02-28

### Added
- Unified bringup CLI helper files under `examples/common/*` for consistent setup/diagnostics flow
- `docs/UNIFICATION_STANDARD.md` and repository-level CLI/timing guard tools

### Changed
- `examples/01_basic_bringup_cli` output/help style aligned with the shared I2C CLI scheme
- Self-test and stress outputs now provide clearer NOT_INITIALIZED/absent-device diagnostics instead of ambiguous summaries

### Fixed
- Public API/type namespace and include-path consistency issues that caused compile breaks during example builds
- Release metadata synchronized for `v0.2.1`

## [0.2.0] - 2026-02-22

### Added
- CO2 error code constants: `CO2_ERROR_SUPPLY_VOLTAGE_LOW` (1), `CO2_ERROR_SENSOR_COUNTS_LOW` (200), `CO2_ERROR_SENSOR_COUNTS_HIGH` (201), `CO2_ERROR_SUPPLY_VOLTAGE_BREAKDOWN` (202)
- `hasCo2Error(statusByte)` static convenience for checking status bit3
- Config validation upper bounds for `writeDelayMs` and `intervalWriteDelayMs` (max 5000 ms)
- `setCustomPointer()` address range validation (rejects > 0xFF)
- Write delay safety limits in `CommandTable.h` (`WRITE_DELAY_MAX_MS`, `INTERVAL_WRITE_DELAY_MAX_MS`)

### Fixed
- `begin()` bus reset now handles clock stretching and generates proper STOP condition
- `recover()` performs bus reset before probing to clear stuck bus states

## [0.1.1] - 2026-02-03

### Changed
- Rename PlatformIO CLI example environments to `ex_cli_s2` and `ex_cli_s3`
- Set default PlatformIO environment to `ex_cli_s3`

### Removed
- Compile-only CI environments

## [0.1.0] - 2026-01-10

### Added
- Initial release with template structure
- ESP32-S2 and ESP32-S3 support

[Unreleased]: https://github.com/janhavelka/EE871-E2/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/janhavelka/EE871-E2/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/janhavelka/EE871-E2/compare/v0.3.0...v1.0.0
[0.3.0]: https://github.com/janhavelka/EE871-E2/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/janhavelka/EE871-E2/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/janhavelka/EE871-E2/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.1
[0.1.0]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.0
