# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- Example health percentages use a 64-bit sum of lifetime counters, preventing
  the display from wrapping when the combined count exceeds `UINT32_MAX`.
- HIL capture checks reject missing, truncated or duplicate health records,
  missing stress operation/health totals, and partial measurement/status values
  even when the final CLI prompt arrives. Expected negative responses retain
  their existing status validation. Incomplete latest summaries cannot reuse
  previous results, and initial soak synchronization rejects crashes or an
  incomplete or dirty configuration response.
- Soak evidence reports the selected scheduled-host-retry policy accurately
  when retries are disabled.
- Host serial commands reject short writes and wait for their bounded response
  instead of calling the serial driver's potentially unbounded drain.
- The Arduino CLI prompt no longer flushes the USB output queue. Arduino's
  transient disconnected indication can make that flush discard pending text.
- HIL status and health parsers recognize error names containing digits,
  including the driver's `E2_ERROR`, without discarding their code/detail.

## [1.1.0] - 2026-09-11

### Migration from 1.0.1

- Rebuild consumers: `Config` and `SettingsSnapshot` have appended fields.
  Driver storage also includes fixed pending-write state. Existing positional
  configuration initializers retain their field order.
- Check `begin()` failures before sampling. Incompatible identity, missing CO2
  capability, or an incomplete or malformed feature-cache read now fail
  initialization.
- Calibration helpers now check support on demand before accessing calibration
  registers. Budget one additional pointer update and byte read per call;
  missing or malformed support flags return `NOT_SUPPORTED`.
- Single-byte and raw writes can now require explicit resync after uncertain
  completion. Recovery and end/begin retain that uncertainty. Allow resync's
  pending-target and capability/status traffic in the maintenance budget;
  auto-adjust start or resync can return `BUSY` while adjustment is running.
- Treat OFFLINE as latched until explicit `recover()` succeeds. Ordinary
  operations fail immediately with the retained error code/detail and the
  message `Driver offline; call recover()`; original health diagnostics remain
  available. Failed recovery clears capabilities and leaves the driver OFFLINE.
- Keep timing within the validated generated period of at most 2,000 us,
  high/low phases of at least 100 us, bit timeout at most 25,000 us, and byte
  timeout at most 35,000 us and greater than nominal byte time.
- Retries remain disabled by default. Applications enabling `readNackRetries`
  must budget for the extra attempts and rebuild their deadline policy. With
  default timing and three retries, value plus status can request up to
  1,246,320 us of HAL delays, or 1,592,540 us including an error-code read.
  Callback and scheduler overhead is additional; the retry guard can veto
  another attempt but cannot interrupt a synchronous frame already in progress.

### Added

- `Config::readNackRetries`: zero to three additional MV3/MV4/status frame
  attempts after a control-byte NACK, successful STOP and idle-line checks,
  separated by a fixed 1 ms HAL pause. Optional `allowReadRetry(busUser)`
  supports application cancellation, deadlines, and latched callback errors.
  Identity/custom reads, writes, PEC failures, and timeouts are never retried.
- Cached `ReadRetryDiagnostics` through `readRetryDiagnostics()` and
  `SettingsSnapshot::readRetry`: saturated NACK/retry/recovery/exhaustion
  counters and sticky last-event results, including cleanup and veto decisions.
  Eligible NACKs count even with retries disabled; health records the final
  frame result once. End/begin reset retry diagnostics; recovery preserves them.
- Separate `Config::flashStretchTimeoutUs` for direct custom-memory write
  STOPs and explicit reset, default 350,000 us and valid 300,000..5,000,000 us.
  It accommodates documented 150/300 ms flash extensions without relaxing
  ordinary bit/byte budgets or replacing post-write waits and verification.
- Native regressions for retries, byte latching, timing boundaries, identity,
  OFFLINE/recovery, cleanup error precedence, calibration/capability guards,
  auto-adjust preflight, and persistent dirty-state/resync transitions;
  Python lexical and CLI contract mutation coverage.

### Changed

- Adjacent custom-memory register pairs use one pointer update followed by
  auto-increment reads, preserving low/high order with fewer bus transactions.
- Both diagnostic scanners use full production initialization plus status/PEC
  validation for five bounded attempts. `libtest` uses tracked driver reads
  and explains OFFLINE suppression.
- Maintained documentation consolidates installation, IDF integration,
  protocol/runner guidance, and hardware evidence. September 11 CO2Control
  retry observations are recorded separately from historical bench results.
- API reference, maintenance guides, and both CLI help texts describe current
  capability and resync contracts without promising address-activation timing
  or an unsupported fixed auto-adjustment duration.

### Fixed

- Uncertain single-byte and raw persistent writes now mark configuration dirty,
  including PEC/ACK, STOP and verification failures. Definite PEC NACKs retain
  their precise error without claiming a possible write. A fixed pending-address
  bitmap keeps every affected target until complete resync, across recovery and
  end/begin; unrelated readback cannot clear the uncertainty. Resync retains
  dirty state if support for pending calibration or part-name fields disappears.
- Auto adjustment checks validated status before issuing a start and returns
  `BUSY` if already running. Resync of an uncertain start requires idle status
  before calibration readback; it does not certify calibration success.
- Offset/gain reads and writes require CO2 support in custom-memory `0x03`;
  calibration-point reads require support in `0x04`. Malformed support bytes
  cannot authorize calibration writes; transport errors remain distinguishable.
- Startup and recovery reject reserved bits in cached feature bytes `0x07..0x09`
  before installing capabilities.
- Auto-adjust status reads require advertised support and reject reserved
  result bits. Operating-mode reads reject active unadvertised mode bits.
  Failed reads preserve caller outputs; raw `customRead()` remains available
  for diagnostics.
- Ordinary read and volatile pointer-write STOPs use `bitTimeoutUs`; the
  flash allowance applies only to direct custom-memory writes at every
  supported device address and to explicit reset for a pending commit.
- Cleanup preserves the first NACK, PEC, or transfer error and releases both
  master lines on failure. START checks actual SDA/SCL transitions; STOP
  validates SDA levels and establishes a complete low phase.
- Clock-stretch polling clips the final poll to the deadline. Whole-byte
  accounting includes nominal phases and reserves the final high/low phases.
- Recovery installs identity/capability caches atomically; partial successful
  transfers and incompatible responding devices cannot clear OFFLINE.
- Both scanners retain `NOT_SUPPORTED` over later transport faults. Timing
  discovery offers six valid candidates and includes the 10 us setup phase
  in its generated-frequency calculation.
- Native IDF GPIO uses input/output open-drain mode so line reads work, with
  reference DATA/CLOCK pins corrected to GPIO6/GPIO7.
- Diagnostic CLIs reject malformed persistent-write arguments and overlength
  input; Arduino `stress` no longer accepts arbitrary command suffixes.
  Synchronous sniffer output is explicitly identified as timing-disturbing.
- Source scanning handles comments, literals, digit separators and continued
  lines without hiding real code. HIL/soak tooling preserves unknown Git state,
  consistent commit identifiers, and adequate mixed-stress timeout margins.
- Protocol/API guidance distinguishes volatile pointer updates from flash
  writes and limits status-triggered measurements to interval >15 s with a
  previous value >10 s old.

### Removed

- Superseded prompt series, completed audit reports, the separate IDF
  implementation report, and the duplicate HIL summary. Useful engineering
  guidance and evidence now live in maintained guides and one validation
  matrix; previous report versions remain available in Git history.
- Ineffective IDF polling sniffer; callback-based diagnostic sniffing remains.

Implementation `3d32ac3` passed 93 native tests, 58 Python tests, three local
Arduino builds, and [all six CI jobs](https://github.com/janhavelka/EE871-E2/actions/runs/34605633301),
including native ESP-IDF 6.0.1 on S2/S3. Targeted COM11 HIL at `32dfb06` passed
407 real sensor/API assertions and 15 injected-error assertions. Follow-ups
isolated a wrong-address response dependent on a pending custom pointer;
the final 303/303 comparison passed, including 12 custom NACKs without retry.
Initial failed NACK expectations remain recorded; the driver correctly
rejected the observed invalid PEC, requiring no code change. All 93 fake tests
also passed on the MCU. Calibration discovery and typed
reads passed on the sensor; persistent-write failure and supported auto-adjust
paths remain emulated-only evidence. Commands, exact firmware evidence
and unrun scenarios are maintained in the
[validation matrix](docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md).

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

### Earlier development work

Includes earlier development work that had no separate published release.

#### Added
- Initial release with template structure
- ESP32-S2 and ESP32-S3 support

[Unreleased]: https://github.com/janhavelka/EE871-E2/compare/v1.1.0...HEAD
[1.1.0]: https://github.com/janhavelka/EE871-E2/compare/v1.0.1...v1.1.0
[1.0.1]: https://github.com/janhavelka/EE871-E2/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/janhavelka/EE871-E2/compare/v0.3.0...v1.0.0
[0.3.0]: https://github.com/janhavelka/EE871-E2/compare/v0.2.1...v0.3.0
[0.2.1]: https://github.com/janhavelka/EE871-E2/compare/v0.2.0...v0.2.1
[0.2.0]: https://github.com/janhavelka/EE871-E2/compare/v0.1.1...v0.2.0
[0.1.1]: https://github.com/janhavelka/EE871-E2/releases/tag/v0.1.1
