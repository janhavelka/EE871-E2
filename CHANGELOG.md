# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Exact-pinned the Arduino example and HIL environments to pioarduino
  `platform-espressif32` `55.03.311` (Arduino-ESP32 `3.3.11`, ESP-IDF `5.5.5`).
  This supersedes the earlier TunnelMonitor-node parity pin `54.03.20` /
  Arduino-ESP32 `3.2.0`, whose HIL evidence remains retained and labeled.
- Configured the ESP32-S3 example/HIL target explicitly for its detected
  4 MB embedded flash and 2 MB QSPI PSRAM.
- Documented that the repository pin controls its example/HIL firmware while
  consuming applications continue to own their platform version.
- Consolidated the HIL-runner parser regressions into one test module and added
  parser/version checks to CI.
- Added a read-only soak harness that checkpoints its evidence and permits one
  explicitly recorded application-level retry only for a scheduled MV3/MV4
  control-byte NACK. The default retry delay is 1,500 ms; the core driver
  remains retry-free and the harness does not infer the sensor-internal cause.
- Added a fixed, non-persistent niche HIL plan for identity/capability,
  parameter guards, GPIO/E2 diagnostics, trace/sniffer behavior, and mixed
  stress, plus a reusable state-only native-USB framing discriminator.
- Condensed retained HIL evidence from 4.53 MB to 1.21 MB: ordinary successful
  runs keep their complete Markdown command ledgers, while raw serial is
  retained for unique failure and time-series evidence. Future structured
  summaries omit ordinary PASS payloads and retain bounded abnormal excerpts.
- Version tooling now synchronizes `Version.h`, `idf_component.yml`, and the
  Doxygen project number from `library.json`.
- Restricted Doxygen input to maintained public documentation and enabled
  undocumented-member/parameter warnings as build-breaking errors.
- Clarified raw MV3/MV4 and side-effecting status-read semantics, persistent
  write contracts, feature-cache limits, electrical integration, and current
  qualification boundaries.
- Replaced the Arduino CLI's unbounded input accumulation with the shared
  127-byte bounded line reader while preserving prompt/trace draining.

### Fixed

- Validate bus-address and measurement-interval ranges before optional-feature
  capability checks, preserving the documented `OUT_OF_RANGE` result without
  touching the bus even when the feature is unsupported.
- Commit the three optional-feature bytes atomically during `begin()` so a
  mid-cache read failure leaves every optional capability disabled.
- Retry E2 address scans a bounded five times so a single transient NACK after
  another diagnostic operation does not falsely report an absent sensor.
- Require a complete CLI prompt and the command-specific value line in the
  Python HIL runner, settle native USB briefly between commands, and stop the
  plan on a framing timeout instead of shifting later responses onto the wrong
  command.
- Classify a parsed non-OK status before checking for a success-value token, so
  a complete MV3 NACK response is a hard failure rather than operator review.
- Fail closed for both operating-mode reads and writes when capability is
  absent, and reject returned `0xD8` values containing reserved bits, instead
  of decoding or writing the unsupported `0x55` register.
- Upgrade the S3 example/HIL build baseline to Arduino-ESP32 3.3.11, which retains the
  upstream HWCDC lost-wakeup/data-loss fix introduced in 3.3.9 by
  Arduino-ESP32 PR #12606.
- Deassert serial DTR/RTS before opening a live HIL port to avoid intentionally
  resetting native-USB targets; no-reset attachment was verified on COM20.
- Drain queued bus-trace output before emitting the next CLI prompt.
- Terminate each diagnostic CLI prompt with a newline so native USB CDC does
  not strand a short final packet and shift rapid command responses.
- Report detected MCU revision, flash size, and initialized PSRAM size in the
  Arduino diagnostic CLI `version` output.
- Removed an ESP32 revision-1 PSRAM cache workaround from the ESP32-S3 build;
  the tested S3 target uses its explicit QIO/QSPI memory configuration.

### Removed

- Removed unused example compatibility wrappers, the empty sniffer tick/class
  wrapper, obsolete native Arduino/Wire stubs, legacy PlatformIO CLI aliases,
  and TunnelMonitor-only dependency-header generation code.
- Removed 39 superseded COM20 runner attempts and wrapper logs while retaining
  the authoritative positive and negative evidence under a documented index.

### Validation

- Native tests: 34 passing.
- Consolidated HIL-runner/parser tests: 38 passing.
- Current Arduino example builds: ESP32-S3 and ESP32-S2 pass on pioarduino
  `55.03.311`; the build-only `compat_tunnelmonitor_s3` environment also passes
  on TunnelMonitor-node commit `0f240ab`'s pioarduino `54.03.20` pin without
  source shims.
- Prior ESP32-S3 COM20 targeted HIL on pioarduino `55.03.39`: 144/144
  PASS; final READY state, zero transport failures, clean persistent state,
  `stress 500` at 500/500, and library version `1.0.1`. Selftest reported
  26 PASS / 0 FAIL / 1 unsupported-mode SKIP.
- Prior ESP32-S3 COM20 serial-only discriminator on pioarduino `55.03.39`:
  10,000/10,000 `dirty` round trips PASS, with 10,000 identical 201-byte
  replies. Because `dirty` performs no E2 traffic, this is CLI-framing evidence
  only, not sensor transport or long-soak evidence.
- Prior `55.03.39` ESP32-S3 COM20 accelerated scheduled-read regression: PASS over
  108 sample cycles and 543.594 seconds. It recorded 564 ordinary passes, two
  fully framed control-byte NACK attempts each recovered by one application
  retry after 1,500 ms, and zero hard failures, reviews, skips, reconnects, or
  transport-counter regressions; final state was READY and clean. The two
  NACKs occurred about 105 seconds apart at nearly identical phase within the
  15-second configured interval. This records schedule-correlated recurrence
  at the E2 control-byte ACK boundary, but it does not identify the electrical
  or sensor-internal cause.
- Prior `55.03.39` ESP32-S3 COM20 operating-mode guard: PASS. An unsupported `mode`
  command returned `NOT_SUPPORTED`, decoded no value, left tracked transport
  counters unchanged, and preserved READY state.
- Historical Arduino ESP32-S3 and ESP32-S2 example builds: passing with
  pioarduino `54.03.20`.
- Historical ESP32-S3 COM20 automated safe/extended HIL on `54.03.20`: 33/33
  PASS, including `selftest` 27/27, `stress 50` 50/50, and `stress 500`
  500/500.
- Historical `54.03.20` ESP32-S3 COM20 runtime-memory smoke HIL: 10/10 PASS;
  firmware reported 4,194,304 bytes flash and PSRAM ready with 2,097,152 bytes.
- Historical `54.03.20` ESP32-S3 COM20 same-value persistent HIL: 25/25 PASS
  for interval `150 ds`, CO2 offset `0 ppm`, and CO2 gain `32768`, with
  verified readback, resync, and clean dirty state.
- Historical `54.03.20` ESP32-S3 COM20 niche checks: range/capability guards
  11/11 PASS; bus trace/sniffer/mixed-stress 11/11 PASS; `stress_mix 500`
  500/500; full bus diagnostics found address 0 and completed six in-spec
  timing points plus two out-of-spec characterization points.
- Historical `54.03.20` ESP32-S3 COM20 operator-assisted physical HIL:
  sensor-absent boot PASS; hot
  unplug transitioned to OFFLINE at threshold 5 and explicit replug recovery
  restored READY; 470-ohm SDA and SCL stuck-low tests returned bounded precise
  failures and recovered; interval `160 ds` persisted across a complete
  sensor/MCU power cycle and the `150 ds` baseline was restored.
- Historical `54.03.20` ESP32-S3 COM20 immediate warm-up HIL: PASS after the
  newline-framed CLI fix.
  Sampling started 0.250 s after COM20 reappeared; MV3/MV4 were `0 ppm` with
  status `0x08` through 4 s and became `678 ppm` with status `0x00` at 5 s.
  All 33 scheduled commands were correctly framed; final driver health was
  READY with 65 transport successes, zero failures, clean persistent state,
  and interval `150 ds`. A separate delayed-start attempt recorded one bounded
  fast-read `NACK` at 20.094 s and recovered immediately; it remains retained
  in the attempt ledger. A post-fix automated safe run also passed 10/10.
- Historical `54.03.20` ESP32-S3 COM20 10-minute post-power-cycle
  stability/stale characterization:
  strict result FAIL because one of 63 scheduled CLI commands returned a bounded
  `NACK` on the t=330 s fast read; the other 62 scheduled CLI commands and the
  status/stale sequence succeeded. This earlier capture's first sample was
  approximately 66 seconds after MCU boot, so it is classified separately from
  the later immediate warm-up PASS. Manually normalized interactive output
  from immediate `stress_mix 1000` and `stress 1000` follow-ups recorded
  1000/1000 for each; the NACK was not reproduced and no hidden core retry was
  added.
- Historical ESP32-S3 COM20 eight-hour soak on pioarduino `54.03.20`: strict
  FAIL with 2,376 PASS, 29 HWCDC mid-line reply stalls, and 11 real MV3
  `0xC1` control-byte NACKs. The old runner labeled those NACKs as review due
  to validator order; they remain failures. NACKs clustered at the same
  measurement phase, while mixed stress blocks otherwise passed. Arduino-ESP32
  PR #12606 identifies and fixes the separate HWCDC TX lost-wakeup mechanism in
  3.3.9.
- The current `55.03.311` source passed ESP32-S2/S3 builds before the
  post-commit COM20 rerun. No completed long-soak result is claimed for it.

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

[Unreleased]: https://github.com/janhavelka/EE871-E2/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/janhavelka/EE871-E2/compare/v0.3.0...v1.0.0
[0.3.0]: https://github.com/janhavelka/EE871-E2/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/janhavelka/EE871-E2/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/janhavelka/EE871-E2/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.1
[0.1.0]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.0
