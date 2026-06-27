# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

No changes yet.

## [1.1.0] - 2026-06-27

### Added
- `BeginPolicy::AllowAbsent` for initialized/OFFLINE startup when the probe is
  absent or unreachable, with cached `beginProbeStatus` diagnostics and
  recovery back to READY after identity validation.
- Full startup/recovery/probe identity validation for group, subgroup, and CO2
  available-measurement support.
- Checked CO2 sample helpers:
  `readCo2AverageSample(Co2ReadResult&)` and
  `readCo2FastSample(Co2ReadResult&)`.
- Append-only `Err::CO2_SENSOR_ERROR` for EE871 status-byte CO2 errors.
- `CO2_PPM_MIN` and `CO2_PPM_MAX` constants for checked-sample validation.
- Optional `delayMs`, `yield`, and `longDelaySliceMs` callbacks for
  millisecond persistent-write delay paths.
- `sampleavg` and `samplefast` commands in Arduino and ESP-IDF examples.

### Changed
- Normal tracked bus APIs now latch OFFLINE and return `BUSY` before touching
  E2 lines until `recover()` succeeds.
- `SettingsSnapshot` exposes begin policy, startup probe status, delay callback
  presence, and normalized long-delay slice without bus traffic.
- Example help now separates raw MV3/MV4 reads from checked value-then-status
  reads and documents status-read measurement side effects.
- Native fake transport gained focused helpers for absent devices, identity
  mismatches, feature flags, read ordering, and cooperative-delay assertions.

### Fixed
- `recover()` from OFFLINE reasserts OFFLINE and the configured failure floor
  when recovery fails.
- Feature flags are cleared before reload and stay zero if the feature-read
  sequence fails.

### Validation
- Native tests: 49 passing in local validation.
- Arduino and ESP-IDF command-contract checks: passing in local validation.
- Hardware HIL for the new 1.1.0 behavior is pending; no new hardware PASS is
  claimed for absent startup, checked samples, identity mismatch, or
  cooperative long-delay callbacks.

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
