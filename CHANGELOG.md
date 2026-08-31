# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added native coverage for generated-clock and byte-budget boundaries,
  stuck-line startup diagnostics, reset recovery and health neutrality, and
  tracked-failure short-circuit behavior during an absent-sensor sample burst.
- Added a completed finding-by-finding code-audit resolution report.
- Added a native test asserting low/high byte order for every adjacent
  custom-memory register pair (firmware version, global interval, CO2 offset,
  CO2 gain), which the zero-valued fake defaults previously left unverified.

### Changed

- Consolidated startup and public E2 recovery clocks in one private raw reset
  helper, with a stretch-aware STOP and bounded SCL polling shared with normal
  transfers.
- `begin()` now validates the actual generated E2 period against the 500 Hz
  minimum and rejects byte budgets that cannot exceed a nominal nine-bit byte.
- ESP-IDF scanner and library-command diagnostics now match the Arduino
  hardening: five scan attempts, PEC-gated discovery, separate invalid-response
  reporting, and production driver reads for `libtest`.
- Both example CLIs explicitly report overlength input and warn that synchronous
  sniffer output perturbs E2 timing; the ineffective IDF poll sniffer was
  removed and IDF component-name coupling is documented.
- Simplified static contract tooling to direct forbidden/dispatch checks and
  removed HIL summary keys that only belong to the soak runner.
- Adjacent custom-memory register pairs (firmware version, global interval,
  CO2 offset, CO2 gain, and the interval write verify) are now read with one
  pointer set plus auto-increment reads, matching the AN1611-1 procedure and
  halving the bus traffic for those operations.

### Fixed

- START now verifies that SDA actually goes low, reporting `BUS_STUCK` for a
  line held high instead of a later misleading control-byte `NACK`.
- Clock-stretch polling now clips its final poll to the remaining deadline
  instead of overshooting non-multiple-of-five timeouts.
- Whole-byte deadlines now cover both clock stretching and nominal bit phases.
- Fixed the ESP-IDF example GPIO transport: `GPIO_MODE_OUTPUT_OD` disables the
  pad input buffer so `gpio_get_level()` always read 0, breaking clock-stretch
  detection and bus-idle checks; now `GPIO_MODE_INPUT_OUTPUT_OD`.
- Fixed the ESP-IDF example pin defaults, which had SCL/SDA swapped relative
  to the HIL-validated Arduino reference wiring (E2 DATA = GPIO6,
  E2 CLOCK = GPIO7).
- Both example CLIs now reject unparseable arguments to persistent-write
  commands (`addr`, `interval`, `filter`, `mode`, `offset`, `gain`, `factor`)
  instead of silently writing 0 to the sensor when parsing failed. Parse
  bounds are the target type's range only; semantic range policy stays in the
  driver, so in-type out-of-range values still reach it and return
  `OUT_OF_RANGE` as before.
- The Arduino CLI `stress` command no longer accepts arbitrary suffixes
  (`stressXYZ` previously ran `stress 100`).
- HIL tooling: soak runner and serial discriminator now record the worktree as
  `unknown` (not `dirty`) when git itself fails; the soak runner's
  `stress_mix` timeout now carries the same margin as the HIL runner (the old
  60 s budget was under 2x the observed runtime); soak metadata uses the same
  12-character commit hashes as the other tools.
- Corrected API documentation: `setCustomPointer()` performs no flash write
  delay (0x50 is a pointer update, not a flash write), `Config::writeDelayMs`
  applies to 0x10 custom writes only, and `Err::IN_PROGRESS` is documented as
  reserved (nothing in the current driver returns it).
- The zero-offline-threshold native test now verifies the normalized threshold
  through a successful fake-transport `begin()` instead of only checking that
  validation passed.

### Removed

- Removed the superseded AI prompt series and dated one-shot audit/report
  snapshots from `docs/`; release history stays in `CHANGELOG.md` and bench
  evidence in `hil_results/README.md` plus
  `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md`. All removed files remain
  recoverable from git history.

## [1.0.1] - 2026-07-31

### Added

- Added bounded HIL tooling for safe, extended, niche, persistent-maintenance,
  physical-fault, scheduled-soak, and serial-only framing tests.
- Added native fake-transport coverage for feature-cache failures, validation
  order, unsupported operating-mode access, persistent dirty-state transitions,
  and recovery behavior.
- Added synchronized version generation for `Version.h`, `idf_component.yml`,
  and Doxygen from the `library.json` version source of truth.

### Changed

- Exact-pinned Arduino example and HIL builds to pioarduino
  `platform-espressif32` `55.03.311` (Arduino-ESP32 `3.3.11`, ESP-IDF `5.5.5`)
  and configured the qualified ESP32-S3 target for 4 MB flash with 2 MB QSPI
  PSRAM.
- Retained a build-only TunnelMonitor compatibility environment on pioarduino
  `54.03.20`; the framework-neutral library source builds on both platform
  generations without compatibility shims.
- Clarified raw measurement versus side-effecting status-read behavior,
  application-owned retry policy, cached-feature semantics, persistent-write
  diagnostics, and task/ISR ownership contracts.
- Replaced unbounded Arduino CLI input accumulation with a 127-byte bounded
  line reader and condensed generated HIL artifacts into a short evidence
  ledger.

### Fixed

- Validate bus-address and measurement-interval ranges before capability
  checks, returning `OUT_OF_RANGE` without bus traffic for invalid values.
- Commit optional-feature cache bytes only after the complete cache read
  succeeds, preventing partially updated capabilities during `begin()`.
- Fail closed with `NOT_SUPPORTED` for operating-mode reads and writes when the
  sensor does not advertise the feature; reject returned reserved bits.
- Require ACK plus valid PEC before the diagnostic address scanner reports a
  device, and route library-command diagnostics through the production
  clock-stretch-aware, PEC-validating driver path.
- Preserve complete sensor NACKs as hard failures. The core performs no hidden
  retry; the soak harness records its one optional scheduled retry separately.
- Require command-specific completion plus a newline-terminated CLI prompt in
  HIL tools, preventing truncated or shifted responses.
- Synchronize every HIL, soak, and serial-discriminator attachment with
  `\ndirty\n`, fixing the false native-USB reattachment timeout caused by a
  blank-line probe that the CLI intentionally ignored.
- Drain queued trace output before printing the next prompt and classify
  parsed non-OK status before checking for success-only value fields.

### Removed

- Removed obsolete Arduino/Wire native stubs, unused compatibility wrappers,
  legacy PlatformIO aliases, duplicate diagnostic transaction code, and
  superseded generated HIL transcripts.

### Validation

- GitHub Actions passes Arduino ESP32-S3/S2 builds, native tests, library
  validation, and native ESP-IDF v6.0.1 ESP32-S3/S2 example builds.
- Native driver tests: 34/34 passing. HIL/parser tooling tests: 40/40 passing.
- ESP32-S3 COM20 full safe/extended/niche HIL on `55.03.311`: 184/184 PASS;
  self-test 26 PASS / 0 FAIL / 1 unsupported-feature SKIP; repeated stress
  500/500; mixed stress 500/500; final READY and persistent state clean with
  3,109 successful transfers and zero failures.
- Native-USB process reattachment: 100/100 separate open/close sessions PASS.
  After the full HIL closed COM20, a new process completed 10,000/10,000
  identical state-only replies without reset or cable replug.
- Physical HIL passed sensor absence, unplug to OFFLINE plus explicit recovery,
  SDA/SCL held-low faults, and measurement-interval power-cycle persistence
  with restoration to the original value.

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

[Unreleased]: https://github.com/janhavelka/EE871-E2/compare/v1.0.1...HEAD
[1.0.1]: https://github.com/janhavelka/EE871-E2/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/janhavelka/EE871-E2/compare/v0.3.0...v1.0.0
[0.3.0]: https://github.com/janhavelka/EE871-E2/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/janhavelka/EE871-E2/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/janhavelka/EE871-E2/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.1
[0.1.0]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.0
