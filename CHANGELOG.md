# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- ESP-IDF component metadata (`CMakeLists.txt`, `idf_component.yml`) for the
  framework-neutral E2 driver core.
- ESP-IDF GPIO E2 adapter and `examples/idf/basic_bringup` application-owned
  transport example.
- ESP-IDF bring-up CLI now mirrors the Arduino CLI command surface, formatting,
  diagnostics, health/error reporting, raw/register access, probe/recover/reset,
  self-test, stress, and demo workflows.
- `tools/check_idf_example_contract.py` verifies Arduino/ESP-IDF CLI help parity
  and guards against stale IDF example wording.
- `SettingsSnapshot`, `getSettings()`, `isInitialized()`, `getConfig()`, `driverState()`, `healthState()`, and `offlineThreshold()` for cache-only runtime and health inspection.
- Command-table helpers for supported main-command read checks and CO2 error-code names.
- Bring-up CLI status/error output now decodes CO2 error-code names.
- Native fake E2 transport under `test/support/` for deterministic host-side
  runtime fault injection at the GPIO-style callback boundary.
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
- Hardware validation matrix and HIL runner documentation.

### Changed
- `library.json` now advertises both Arduino and ESP-IDF framework support.
- Release metadata now matches the published repository, maintainer contact,
  and `0.3.0` changelog baseline.
- Reference documentation now uses human-readable vendor PDF names and separates compact protocol notes from full PDF extractions under `docs/extracted-md/` and `docs/pdf-extracted-md/`.
- `Config::offlineThreshold = 0` now normalizes to one, and `begin()` / `end()` reset stale cached runtime and feature state.
- High-level optional-feature helpers now consistently return `NOT_INITIALIZED` before parameter or capability checks when called before `begin()`.
- `writeOperatingMode()` now validates unsupported bit fields before capability checks.
- README and `Config` Doxygen now document timing bounds, health behavior, feature guards, and callback ownership more explicitly.
- ESP-IDF port documentation now describes the interactive CLI and validation
  checklist instead of a one-way periodic logger.
- `AGENTS.md` now explicitly requires native ESP-IDF examples and forbids
  Arduino compatibility facades in IDF example code.
- `EE871::EE871` is now explicitly non-copyable and non-movable.
- Public docs now state the managed synchronous driver contract, callback
  boundedness expectations, thread/ISR constraints, shared-bus serialization
  requirements, and E2-versus-hardware-I2C boundary.
- Multi-byte persistent writes now preserve the precise failing `Status` while
  also recording that persistent sensor configuration may need resync or
  operator inspection.

### Fixed
- Byte-timeout accounting in E2 bit helpers now uses saturating arithmetic and avoids overflow in the elapsed-time accumulator.
- Unsupported EE871 main-command reads now return `NOT_SUPPORTED` before E2 traffic, including two-byte reads.
- `IN_PROGRESS` statuses are neutral for health tracking instead of counting as communication failures.
- Dirty-state tracking now covers first-byte accepted/readback-failed cases for
  multi-byte persistent writes.

### Compatibility
- Source compatibility break: code that copies or moves `EE871::EE871`
  instances by value must be updated to keep drivers in stable storage and pass
  references or pointers.
- `SettingsSnapshot` layout changed. ABI/layout-sensitive users must rebuild
  and should not persist or externally share raw snapshot layouts.
- The current package version remains `0.3.0` because this is not a release
  commit and release readiness is still blocked. The next release should use a
  major-version bump under the repository's SemVer policy if these
  source-compatibility changes are released.

### Validation
- Native host test coverage includes 31 PlatformIO test cases.
- Arduino PlatformIO example builds for `ex_bringup_s3` and `ex_bringup_s2`
  pass locally in the latest readiness run.
- Pure ESP-IDF CI coverage is configured, but no GitHub Actions run exists for
  `hardening/ee871-e2-industry-readiness` as of the latest readiness check.
- Real hardware HIL evidence is not present in the repository yet; `hil_results/`
  is absent and hardware matrix rows remain `NOT RUN`.

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

[Unreleased]: https://github.com/janhavelka/EE871-E2/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/janhavelka/EE871-E2/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/janhavelka/EE871-E2/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/janhavelka/EE871-E2/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.1
[0.1.0]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.0
