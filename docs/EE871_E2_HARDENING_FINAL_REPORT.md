# EE871-E2 Hardening Final Report

Started: 2026-05-31
Last updated: 2026-06-02
Branch: `hardening/ee871-e2-industry-readiness`

## Current Status Summary

- Core hardening, native fake E2 fault tests, persistent dirty diagnostics, CLI
  dirty/resync diagnostics, IDF example contract checks, and the Python HIL
  runner are implemented.
- ESP32-S3 Arduino hardware evidence is recorded for safe default HIL, extended
  safe HIL, manual dirty/resync, and persistent interval write/readback/restore.
- Physical unplug/replug recovery is recorded as PASS from an
  operator-confirmed manual test on 2026-06-02; no automated transcript artifact
  exists for that step.
- Pure ESP-IDF build success remains unproven until either GitHub Actions shows
  the `idf-build` matrix passing for `esp32s3` and `esp32s2`, or local `idf.py`
  builds pass for both targets.
- ESP32-S2 hardware HIL, pure ESP-IDF hardware HIL, power-cycle persistence, and
  stuck-line fault/jig validation remain open.
- This report is chronological. For the latest hardware evidence table, use
  [EE871_E2_HARDWARE_VALIDATION_MATRIX.md](EE871_E2_HARDWARE_VALIDATION_MATRIX.md).

## Prompt 01 - Core Contracts And Public API Safety

Scope:
- Created the hardening branch from `main`.
- Tightened EE871-specific engineering rules in `AGENTS.md`.
- Made `EE871::EE871` non-copyable and non-movable while preserving default construction.
- Documented public thread-safety, ISR-safety, callback reentrancy, and shared-bus serialization contracts.
- Clarified that EE871-E2 uses GPIO-style E2 signaling, not hardware I2C.
- Added native compile-time copy/move contract checks.

Files changed:
- `AGENTS.md`
- `README.md`
- `include/EE871/Config.h`
- `include/EE871/EE871.h`
- `test/test_basic.cpp`
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`

Tests run:
- `python tools/check_core_timing_guard.py`: PASS, `Core timing guard PASSED`.
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python tools/check_idf_example_contract.py`: PASS, `IDF example contract PASSED`.
- `python scripts/generate_version.py check`: PASS, `include\EE871\Version.h` up to date.
- `python -m platformio test -e native`: PASS, 14 test cases succeeded.
- `python -m platformio run -e ex_bringup_s3`: PASS, `SUCCESS` in 00:00:36.232.
- `python -m platformio run -e ex_bringup_s2`: PASS, `SUCCESS` in 00:00:31.967.
- `git diff --check`: PASS; only Git line-ending conversion warnings were emitted.

Hardware validation:
- Not run. No hardware behavior is claimed in this report.

Subsequent hardening prompts:
- Prompt 02: native fake E2 transport and runtime fault tests.
- Prompt 03: persistent multi-byte dirty-state diagnostics.
- Prompt 04: pure ESP-IDF build coverage, CI, and IDF example documentation.
- Prompt 05: hardware validation matrix, final integration review, and release/merge report.

## Prompt 02 - Native Fake E2 Fault Harness And Runtime Fault Tests

Fake transport design summary:
- Added `test/support/FakeE2Transport.h`, a deterministic native-only E2 fake
  that implements the driver callback boundary: `setScl`, `setSda`, `readScl`,
  `readSda`, and `delayUs`.
- The fake models START/STOP detection, MSB-first byte writes, ACK/NACK
  sampling, read data/PEC bits, custom pointer auto-increment, custom writes,
  clock-stretch timeout by holding SCL low, forced device absence/NACK,
  corrupted read PEC, write drops for verify-mismatch testing, and replug-style
  present/absent transitions.
- The fake uses fixed-size storage and standard C++ only. It does not include
  Arduino, ESP-IDF, FreeRTOS, logging, heap allocation, or global bus objects.

Runtime fault tests added:
- Successful fake-device `begin()` and feature-cache sanity check.
- Clock-stretch / stuck-SCL timeout on a tracked public read, including bounded
  fake elapsed time and DEGRADED health transition.
- PEC mismatch on raw `probe()` with no health side effects, followed by a
  tracked read that updates health.
- Device absent / NACK on raw `probe()` with no health side effects, followed by
  a tracked read that increments failure counters.
- `customWrite()` accepted write with readback verify mismatch returning
  `Err::E2_ERROR`, `Write verify failed`, and the actual readback detail.
- Offline threshold transition through repeated tracked failures, then replug and
  successful `recover()` returning health to READY.

Behavior changes:
- No library behavior changes were made in `include/` or `src/`.
- Native tests no longer include the Arduino/Wire stubs directly; Prompt 02
  runtime fault tests use only the native fake and the public EE871 API.
- Existing behavior observed by tests: `probe()` is raw and does not mutate
  health; verify mismatch is returned as a precise `E2_ERROR` diagnostic and is
  not itself health-tracked because the underlying write/readback transfers
  succeeded.

Tests run:
- `python tools/check_core_timing_guard.py`: PASS, `Core timing guard PASSED`.
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python tools/check_idf_example_contract.py`: PASS, `IDF example contract PASSED`.
- `python scripts/generate_version.py check`: PASS, `include\EE871\Version.h` up to date.
- `python -m platformio test -e native`: PASS, 20 test cases succeeded in 00:00:01.788.
- `python -m platformio run -e ex_bringup_s3`: PASS, `SUCCESS` in 00:00:27.808.
- `python -m platformio run -e ex_bringup_s2`: PASS, `SUCCESS` in 00:00:18.752.
- `git diff --check`: PASS; only Git line-ending conversion warnings were emitted.

## Prompt 03 - Persistent Multi-Byte Dirty-State Diagnostics

API changes:
- Added `persistentConfigDirty()` and `persistentConfigDirtyError()` public
  diagnostics.
- Added `persistentConfigDirty` and `persistentConfigDirtyError` to
  `SettingsSnapshot`.
- Added `resyncPersistentConfig()` to re-read persistent fields and clear the
  dirty diagnostic only after the values are readable and coherent.

Behavior changes:
- `writeMeasurementInterval()` marks persistent configuration dirty when the
  low byte has succeeded and a high-byte write, verify read, or verify compare
  fails. A low-byte write failure before any byte succeeds remains clean.
- `writeCo2Offset()` and `writeCo2Gain()` mark dirty when the low byte succeeds
  and the high-byte write/verify path fails.
- `writePartName()` marks dirty when at least one byte has succeeded and a
  later byte write fails.
- The dirty diagnostic preserves the first/root failing `Status` and does not
  replace the precise write-function return status.
- Dirty state is not cleared by unrelated successful reads, `recover()`,
  OFFLINE transitions, `end()`, or failed `begin()` cleanup. It is cleared only
  by successful `resyncPersistentConfig()`.

Fake transport/test harness changes:
- Extended `test/support/FakeE2Transport.h` with one-shot address-matched
  custom-write NACKs and one-shot dropped write commits. This keeps the fake at
  the E2 callback boundary while allowing exact low-byte, high-byte, and
  verify-mismatch scripts.

Runtime fault tests added:
- `writeMeasurementInterval()` low-byte failure leaves dirty false.
- `writeMeasurementInterval()` high-byte failure marks dirty with the original
  NACK status.
- `writeMeasurementInterval()` verify mismatch marks dirty and an unrelated
  successful `readStatus()` does not clear it.
- `writeCo2Offset()` high-byte failure marks dirty.
- `writeCo2Gain()` high-byte failure marks dirty.
- Dirty diagnostics preserve the first failing status when later dirty-capable
  operations also fail.
- `resyncPersistentConfig()` fails without clearing dirty when interval bytes
  are incoherent, then clears dirty after the fake memory is made coherent.
- Dirty state survives OFFLINE.

Docs updated:
- README now documents that multi-byte persistent writes are not bus-atomic,
  persistent state can be partially changed on failure, dirty diagnostics should
  be checked, and persistent writes are maintenance operations with latency and
  endurance implications.

Remaining limitations:
- Dirty state is global, not a per-field bitmap. It intentionally reports that
  persistent configuration needs resync/inspection without guessing which field
  is authoritative.
- `resyncPersistentConfig()` validates that persistent fields can be read and
  that the global interval is in range; it cannot prove the values match an
  external operator-intended configuration unless the application compares them.
- Prompt 05 later added private write-accepted tracking so first-byte
  post-accept failures can mark dirty. The remaining limitation is diagnostic
  granularity: dirty state is still global and does not identify a single
  authoritative field.

Tests run:
- `python tools/check_core_timing_guard.py`: PASS, `Core timing guard PASSED`.
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python tools/check_idf_example_contract.py`: PASS, `IDF example contract PASSED`.
- `python scripts/generate_version.py check`: PASS, `include\EE871\Version.h` up to date.
- `python -m platformio test -e native`: PASS, 28 test cases succeeded in 00:00:02.538.
- `python -m platformio run -e ex_bringup_s3`: PASS, `SUCCESS` in 00:00:20.415.
- `python -m platformio run -e ex_bringup_s2`: PASS, `SUCCESS` in 00:00:27.054.
- `git diff --check`: PASS; only Git line-ending conversion warnings were emitted.

## Prompt 04 - ESP-IDF Build Coverage, CI, And Example Contract Honesty

CI changes:
- Added a dedicated GitHub Actions `idf-build` matrix job for `esp32s3` and
  `esp32s2`.
- The job runs `python tools/check_idf_example_contract.py` before building.
- The job uses `espressif/esp-idf-ci-action@v1` with `esp_idf_version:
  v6.0.1`, `target: ${{ matrix.target }}`, and `path:
  examples/idf/basic_bringup`, which maps to `idf.py build` for the example
  project.

IDF component/example metadata:
- Root `CMakeLists.txt` remains framework-neutral: only `src/EE871.cpp` and
  `include/` are registered for the core component.
- `idf_component.yml` already targets `esp32s2` and `esp32s3` with `idf >=6.0.1`.
- `examples/idf/basic_bringup` keeps `EXTRA_COMPONENT_DIRS "../../.."` so the
  root component is built as the local dependency.
- Added the explicit `esp_rom` dependency to the IDF example component because
  `examples/idf/common/E2GpioTransport.h` includes `esp_rom_sys.h`.

IDF example/docs honesty:
- README and IDF docs now describe the IDF example as a diagnostic/basic
  bring-up example.
- Docs state that the example owns GPIO setup for demonstration, production
  users should integrate callbacks into their own GPIO or bus manager, and
  external serialization is required if multiple tasks can access the same
  `EE871` instance or E2 lines.
- Docs explicitly state that EE871-E2 uses GPIO-style E2 signaling, not
  ESP-IDF `driver/i2c_master` or hardware I2C.

Local IDF availability:
- `idf.py --version`: FAIL. PowerShell reported: `The term 'idf.py' is not
  recognized as the name of a cmdlet, function, script file, or operable
  program.`
- Because `idf.py` is unavailable locally, the following local pure ESP-IDF
  builds were not run:
  - `idf.py -C examples/idf/basic_bringup set-target esp32s3 build`
  - `idf.py -C examples/idf/basic_bringup set-target esp32s2 build`

Remaining IDF validation gaps:
- Local pure `idf.py` build success is still unverified on this workstation
  until ESP-IDF is installed or available on `PATH`.
- CI coverage has been added but will only be proven after GitHub Actions runs
  the new `idf-build` matrix.
- Hardware validation remains pending for real ESP32-S2/S3 boards with an EE871
  sensor.

Commands run:
- `git checkout hardening/ee871-e2-industry-readiness`: already on branch.
- `git pull --ff-only`: already up to date.
- `git status --short`: clean before edits.
- `python tools/check_core_timing_guard.py`: PASS, `Core timing guard PASSED`.
- `python tools/check_idf_example_contract.py`: PASS, `IDF example contract PASSED`.
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python scripts/generate_version.py check`: PASS, `include\EE871\Version.h` up to date.
- `python -m platformio test -e native`: PASS, 28 test cases succeeded in 00:00:01.409.
- `python -m platformio run -e ex_bringup_s3`: PASS, `SUCCESS` in 00:00:19.613.
- `python -m platformio run -e ex_bringup_s2`: PASS, `SUCCESS` in 00:00:19.908.
- `idf.py --version`: FAIL, command not found as described above.
- `git diff --check`: PASS; only Git line-ending conversion warnings were emitted.

## Prompt 05 - Final Integration Review And Hardware Validation Plan

Starting audit findings:
- The original audit classified the core architecture as sound but not yet
  industry-grade because validation evidence was missing.
- High-severity gaps were pure ESP-IDF build coverage, native E2 runtime fault
  coverage, and dirty-state diagnostics for partially applied multi-byte
  persistent writes.
- Medium-severity gaps included public copy/move safety, thread/ISR contract
  clarity, and documentation honesty around EE871-E2 GPIO signaling versus
  hardware I2C.
- Hardware validation was unknown; no real ESP32-S2/S3 plus EE871 bench results
  existed in the audit.

Full branch diff review:
- `git diff --stat main...HEAD`: reviewed; branch changes are limited to CI,
  docs, public contracts, dirty-state implementation, native fake transport,
  and native tests.
- `git diff --check`: PASS.
- `git status --short`: clean before Prompt 05 edits.
- No generated build artifacts, package tarballs, firmware binaries, or
  accidental archives are part of the intended diff.
- No Arduino, ESP-IDF, FreeRTOS, logging, GPIO, or hardware I2C headers leaked
  into `include/` or `src/`.
- The large test addition is scoped to native tests under `test/` and uses the
  callback-boundary fake E2 transport.
- Documentation does not claim hardware validation, local pure `idf.py` success,
  or field/industry-grade readiness.

Prompt 05 code/test correction:
- Final diff review found that first-byte `customWrite()` verify/readback
  failures in multi-byte persistent writes could be dirty but were treated as
  clean by `writeCo2Offset()`, `writeCo2Gain()`, and `writePartName()`.
- Added private write-accepted tracking for `_writeCommandRaw()`,
  `_writeCommandTracked()`, and `_customWriteDirect()` so callers can distinguish
  a clean pre-accept failure from a post-accept failure.
- Added native tests for first-byte accepted/readback-failed cases:
  `test_co2_offset_low_byte_verify_failure_sets_dirty`,
  `test_co2_gain_low_byte_verify_failure_sets_dirty`, and
  `test_part_name_first_byte_verify_failure_sets_dirty`.
- Final native coverage is 31 test cases.

Public API changes across the branch:
- `EE871::EE871` is now non-copyable and non-movable.
- Added `resyncPersistentConfig()`.
- Added `persistentConfigDirty()` and `persistentConfigDirtyError()`.
- Added `persistentConfigDirty` and `persistentConfigDirtyError` to
  `SettingsSnapshot`.
- No new `Err` enum value was added; write methods continue returning the
  precise failing `Status`.

Core changes:
- Public comments now state that EE871-E2 is GPIO-style E2 signaling, not
  Arduino Wire, ESP-IDF hardware I2C, or an owned bus.
- Thread-safety, ISR-safety, callback boundedness, and callback reentrancy
  contracts are documented.
- Health state behavior remains wrapper-driven: public tracked operations update
  health through tracked wrappers, while `probe()` remains raw/no-health.
- Persistent dirty state is set when multi-byte persistent configuration may
  have partially applied and is cleared only by verified
  `resyncPersistentConfig()`.

Test infrastructure changes:
- Added `test/support/FakeE2Transport.h`, a deterministic native E2 fake at the
  line-callback boundary.
- Added runtime fault tests for SCL timeout, PEC mismatch, device absence,
  verify mismatch, offline/recover, probe health side effects, dirty-state
  transitions, and dirty-state resync behavior.
- Native tests no longer depend on Arduino/Wire stubs.

ESP-IDF and CI changes:
- Added CI `idf-build` matrix for `esp32s3` and `esp32s2` using
  `espressif/esp-idf-ci-action@v1` and `esp_idf_version: v6.0.1`.
- The IDF CI job runs `python tools/check_idf_example_contract.py` before the
  build.
- Added explicit `esp_rom` dependency to the IDF example component because the
  example transport includes `esp_rom_sys.h`.
- Local pure `idf.py` builds were not run because `idf.py` is not installed or
  not on `PATH` in this shell.

Documentation changes:
- README documents the E2-versus-hardware-I2C boundary, managed synchronous
  timing, thread/ISR constraints, persistent dirty diagnostics, and IDF example
  honesty.
- IDF docs now describe the example as diagnostic/basic bring-up and state that
  production users should integrate callbacks into their own GPIO or bus
  manager.
- IDF port docs now state that pure `idf.py` build proof remains an acceptance
  gap until captured locally or in CI.
- Package metadata now describes the driver as bounded/synchronous rather than
  non-blocking, matching the managed synchronous API contract.
- IDF port audit metadata was refreshed for this hardening branch.
- Added `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md` with concrete
  per-board/per-scenario hardware validation rows, all marked `NOT RUN`, plus a
  safe CLI recipe and persistent-write warnings.
- The hardware validation matrix now labels its safe recipe as
  non-persistent, keeps warm-up classification as an operator/application
  validation concern, and requires configured-address follow-up for bus address
  validation.

Hardware validation status:
- NOT RUN during Prompt 05. No real ESP32-S2, ESP32-S3, EE871 sensor, wiring
  fault, pull-up, warm-up, stale-sample, unplug/replug, or persistent-write bench
  validation was performed during that pass.
- At that point, the hardware validation matrix was a plan only. Later sections
  in this report record ESP32-S3 safe HIL and persistent interval
  write/readback/restore evidence.
- The diagnostic CLIs now expose `dirty` and `resync` commands for
  `persistentConfigDirty`, `persistentConfigDirtyError`, and
  `resyncPersistentConfig()` hardware-validation flow.

Exact commands run in Prompt 05:
- `git checkout hardening/ee871-e2-industry-readiness`: already on branch.
- `git pull --ff-only`: already up to date.
- `git status --short`: clean before edits.
- `git diff --stat main...HEAD`: reviewed.
- `git diff --check`: PASS before edits and PASS after final edits; only Git
  line-ending conversion warnings were emitted after edits.
- `python tools/check_core_timing_guard.py`: PASS, `Core timing guard PASSED`.
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python tools/check_idf_example_contract.py`: PASS, `IDF example contract PASSED`.
- `python scripts/generate_version.py check`: PASS, `include\EE871\Version.h` up to date.
- `python -m platformio test -e native`: PASS, 31 test cases succeeded in 00:00:01.344.
- `python -m platformio run -e ex_bringup_s3`: first attempt FAIL in 00:00:52.390
  while compiling Arduino framework object `esp32-hal-bt.c.o`; no project
  source diagnostic was emitted in captured output.
- `python -m platformio run -e ex_bringup_s3`: rerun PASS, `SUCCESS` in 00:00:19.308.
- `python -m platformio run -e ex_bringup_s2`: PASS, `SUCCESS` in 00:00:17.874.
- `python -m platformio pkg pack`: PASS, wrote `ee871-e2-0.3.0.tar.gz`.
- `Remove-Item -LiteralPath ...\ee871-e2-0.3.0.tar.gz`: generated package
  artifact removed; it is not intended to be tracked.
- `idf.py --version`: FAIL. PowerShell reported: `The term 'idf.py' is not
  recognized as the name of a cmdlet, function, script file, or operable
  program.`

Commands not run and why:
- `idf.py -C examples/idf/basic_bringup set-target esp32s3 build`: not run
  because `idf.py` is unavailable locally.
- `idf.py -C examples/idf/basic_bringup set-target esp32s2 build`: not run
  because `idf.py` is unavailable locally.
- GitHub Actions result verification: not claimed in this report. The new CI
  job must be proven by a PR or workflow run.
- Hardware CLI validation: not run during Prompt 05; later sections record
  ESP32-S3 safe HIL and persistent interval write/readback/restore evidence.

Known remaining gaps:
- Pure ESP-IDF builds must pass in GitHub Actions or a local ESP-IDF
  environment.
- Broader real hardware validation must still be executed using
  `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md`.
- Dirty/resync CLI diagnostics now have ESP32-S3 bench evidence; broader target
  and physical-fault coverage remains open.
- Persistent interval write/readback/restore now has bench evidence; power-cycle
  persistence and failed-write operator recovery remain open.
- `library.json` version and `CHANGELOG.md` were not updated for a release.
- CI currently runs on main/master push and PR events; the new IDF job still
  needs a PR/workflow run to prove it.

Future work for a full industry-grade claim:
- Prove pure ESP-IDF S2/S3 builds in CI.
- Run and record the hardware matrix on both ESP32-S2 and ESP32-S3 with real
  EE871 hardware.
- Add hardware or jig-based tests for stuck SCL/SDA, wrong wiring/no sensor,
  unplug/replug, PEC/fault behavior if observable, warm-up, and stale sample
  behavior.
- Run the dirty/resync CLI flow on hardware and record output in the hardware
  validation matrix.
- Add release notes, versioning decision, and changelog entries before tagging.

Compatibility and breaking-change notes:
- Deleting copy/move operations is a source-compatibility break for users who
  copied or moved `EE871` instances by value.
- `SettingsSnapshot` layout changed, so layout-sensitive or ABI-sensitive users
  must rebuild.
- Added diagnostics are backward-compatible for normal source users.
- No behavior changes intentionally retarget bus ownership, timing callbacks, or
  framework neutrality.

Merge readiness verdict:
- Ready to merge after CI passes. The branch is PR-ready and materially improves
  hardening with core contracts, native E2 fault injection, persistent dirty
  diagnostics, IDF CI coverage, and honest validation docs.

Release readiness verdict:
- Not release-ready yet. Release still needs successful pure ESP-IDF CI results,
  release/version/changelog decisions, and at least the intended release-level
  hardware validation evidence.

Field/industry-grade verdict:
- Not yet field/industry-grade. Production-oriented hardening is complete for
  this branch, but physical fault validation and pure IDF CI evidence remain
  required before making an industry-grade claim.

## Focused IDF Verification Pass - 2026-06-01

Scope:
- Verify whether the new pure ESP-IDF build coverage is proven locally or by
  GitHub Actions.
- Do not refactor the driver.

Starting state:
- Branch: `hardening/ee871-e2-industry-readiness`.
- `git status --short`: clean.
- Latest commits before this verification update:
  - `bea6bb5 docs: prepare EE871 release readiness notes`
  - `b063df9 test: add EE871 Python HIL runner`
  - `c2ca189 test: require EE871 IDF dirty resync-needed output`
  - `6bc102c feat: expose EE871 persistent dirty diagnostics in CLI`
  - `953fec9 docs: record EE871 IDF build verification status`

CI workflow status:
- `.github/workflows/ci.yml` contains an `idf-build` job with a matrix for
  `esp32s3` and `esp32s2`.
- The job runs `python tools/check_idf_example_contract.py` before the IDF
  action build.
- The job uses `espressif/esp-idf-ci-action@v1` with
  `esp_idf_version: v6.0.1` and `path: examples/idf/basic_bringup`.
- YAML syntax check passed with `yaml.safe_load(...)`: `YAML parse OK`.
- This proves the intended CI coverage is present and the workflow parses
  locally. It does not prove that GitHub Actions has successfully run it.

GitHub Actions status:
- `gh run list --limit 5`: FAIL locally because PowerShell reported `gh` is
  not recognized as a cmdlet, function, script file, or operable program.
- `gh run view <id> --log`: not run because `gh` is unavailable and no run ID
  was available locally.
- No GitHub Actions result was available locally during this pass.
- Therefore, GitHub Actions does not currently prove pure ESP-IDF build success
  for this branch.

Local pure ESP-IDF status:
- `idf.py --version`: FAIL locally. PowerShell reported: `The term 'idf.py' is
  not recognized as the name of a cmdlet, function, script file, or operable
  program.`
- `idf.py -C examples/idf/basic_bringup set-target esp32s3 build`: not run
  because `idf.py` is unavailable locally.
- `idf.py -C examples/idf/basic_bringup set-target esp32s2 build`: not run
  because `idf.py` is unavailable locally.

Verification result:
- Pure ESP-IDF build coverage is configured in CI, but pure ESP-IDF build
  success is still unproven.
- Do not claim pure IDF success until either:
  - GitHub Actions shows the `idf-build` matrix passing for `esp32s3` and
    `esp32s2`; or
  - local `idf.py` builds pass for both targets.

Focused-pass validation commands:
- `python tools/check_idf_example_contract.py`: PASS,
  `IDF example contract PASSED`.
- `python tools/check_core_timing_guard.py`: PASS,
  `Core timing guard PASSED`.
- `python -m platformio test -e native`: PASS, 31 test cases succeeded in
  00:00:01.317.
- `git diff --check`: PASS; only Git line-ending conversion warnings were
  emitted after editing the report.

Remaining gap:
- Open a PR or otherwise trigger GitHub Actions for this branch, then record the
  `idf-build` matrix result for both `esp32s3` and `esp32s2`.
- Alternatively install ESP-IDF locally and run both `idf.py -C
  examples/idf/basic_bringup ... build` commands.

## Release Readiness Pass - 2026-06-01

Scope:
- Prepare the branch for a merge/release decision after the hardening, CI, and
  HIL-runner work.
- Do not tag a release.
- Do not claim hardware or CI evidence that is not present.

Version inspection:
- `library.json`: current package version is `0.3.0`.
- `include/EE871/Version.h`: generated from `library.json`, reports
  `0.3.0`, `VERSION_MAJOR = 0`, `VERSION_MINOR = 3`, `VERSION_PATCH = 0`.
- `scripts/generate_version.py`: generator supports `check`, `sync`, `set`,
  and `bump`; `check` confirms `Version.h` is up to date.
- README and docs contain no separate release badge/version override requiring
  synchronization.

Version decision:
- Version was not bumped in this pass because this is not a release commit and
  the branch is not release-ready.
- A future release containing the current public API changes should use a
  major-version bump under this repository's SemVer policy. The reason is source
  compatibility: `EE871::EE871` is now non-copyable and non-movable, and
  `SettingsSnapshot` layout changed.
- Normal users that construct one driver instance and rebuild should not need
  source changes, but code that copied or moved `EE871` by value will no longer
  compile.

CHANGELOG status:
- `CHANGELOG.md` now records core contract hardening, native fake E2 fault
  tests, persistent dirty diagnostics, CLI dirty/resync diagnostics, IDF CI
  coverage, the Python HIL runner, hardware validation status, and compatibility
  notes.

CI status:
- Local `gh` is unavailable in this PowerShell session, so GitHub Actions
  results were not checked locally.
- The CI workflow contains the intended pure-IDF `idf-build` matrix for
  `esp32s3` and `esp32s2`, but no local GitHub Actions result proves it passed.
- Local pure `idf.py` builds remain unproven because `idf.py` was not available.

HIL status:
- `tools/ee871_hil_runner.py` exists and is covered by host parser tests.
- Safe ESP32-S3 HIL evidence is now recorded in the later
  "Safe Hardware HIL Validation - 2026-06-01" section.
- Persistent interval write/readback/restore evidence is now recorded in the
  later "Persistent Configuration Hardware Validation - 2026-06-01" section.
- Earlier attempts to start hardware HIL were stopped because PlatformIO listed
  multiple plausible USB serial ports and no target EE871 CLI port was
  identified. This was later resolved with user-confirmed `COM17`.

Current validation commands:
- `python tools/check_core_timing_guard.py`: PASS, `Core timing guard PASSED`.
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python tools/check_idf_example_contract.py`: PASS,
  `IDF example contract PASSED`.
- `python scripts/generate_version.py check`: PASS, `include\EE871\Version.h`
  up to date.
- `python -m platformio test -e native`: PASS, 31 test cases succeeded in
  00:00:08.323.
- `python -m platformio run -e ex_bringup_s3`: PASS, `SUCCESS` in
  00:01:07.574.
- `python -m platformio run -e ex_bringup_s2`: PASS, `SUCCESS` in
  00:01:20.791.
- `python -m platformio pkg pack`: PASS, wrote `ee871-e2-0.3.0.tar.gz`.
- Generated package tarball removed after packing; it is not intended to be
  tracked.

Remaining gaps:
- Trigger and record GitHub Actions for this branch, including the pure-IDF
  `idf-build` matrix for `esp32s3` and `esp32s2`.
- Run ESP32-S2 hardware HIL and pure ESP-IDF hardware HIL if those targets are
  required for the release evidence set.
- Persistent interval write/readback/restore is now recorded in the later
  "Persistent Configuration Hardware Validation - 2026-06-01" section.
  Power-cycle persistence, CO2 calibration writes, and bus-address write/recovery
  remain unproven.
- Run physical fault/jig validation for stuck SCL/SDA, no response, and
  unplug/replug recovery before any field/industry-grade claim.
- Decide the release version and regenerate `Version.h` when an actual release
  commit is made.

Merge readiness verdict:
- Closer, but not fully evidence-backed yet. Local guards, PlatformIO builds,
  and ESP32-S3 safe HIL pass; the branch still lacks GitHub Actions proof and
  broader hardware/fault evidence.

Release readiness verdict:
- Not release-ready. Release is still blocked on CI proof, remaining
  release-level hardware evidence as required, a major-version release decision,
  and final release notes/tag instructions.

Field/industry-grade verdict:
- Not ready to claim industry-grade. The codebase is materially hardened, but
  CI proof, power-cycle persistent-write evidence, ESP32-S2/ESP-IDF hardware
  coverage, and physical fault validation remain open.

## Safe Hardware HIL Validation - 2026-06-01

Scope:
- Run the safe default and extended non-persistent EE871 HIL runner on real
  hardware.
- Do not run persistent-write tests.
- Record actual evidence in `hil_results/` and update the hardware validation
  matrix.

Hardware and firmware:
- Serial port: `COM17`.
- PlatformIO device list identified `COM17` as USB VID:PID `303A:1001`, serial
  `1C:DB:D4:80:C9:40`.
- Target: ESP32-S3, PlatformIO environment `ex_bringup_s3`.
- Upload command used before final evidence capture:
  `python -m platformio run -e ex_bringup_s3 -j 1 -t upload --upload-port COM17`.
- Firmware build: `Jun  1 2026 20:57:04`.
- EE871 library: `0.3.0 (84a46b6, 2026-06-01 20:57:01, clean)`.

Runner correction before evidence:
- The first post-upload HIL attempt showed the ESP32-S3 serial reset boot text
  could offset the first commands. `tools/ee871_hil_runner.py` was updated to
  wait for the CLI prompt during startup synchronization.
- Regression coverage was added in `test/test_hil_runner_parser.py`.
- Commit: `84a46b6 test: sync EE871 HIL runner after serial reset`.

Safe default HIL result:
- Command:
  `python tools/ee871_hil_runner.py --port COM17 --baud 115200 --output-dir hil_results/safe_default`
- Final verdict: PASS.
- Counts: 10 PASS, 0 FAIL, 0 SKIP, 0 OPERATOR_REVIEW_REQUIRED.
- Covered commands: `version`, `help`, `probe`, `read`, `selftest`, `drv`,
  `dirty`, `stress 50`, `drv`, `dirty`.
- Key results: `read` Status OK with CO2 avg `567 ppm`; `probe` Status OK;
  `selftest` pass=27 fail=0 skip=0; `drv` READY/online yes/zero consecutive
  failures; `stress 50` success=50 errors=0; persistent dirty stayed clean.
- Artifacts:
  - `hil_results/safe_default/ee871_20260601T185912Z/serial_transcript.txt`
  - `hil_results/safe_default/ee871_20260601T185912Z/summary.json`
  - `hil_results/safe_default/ee871_20260601T185912Z/summary.md`

Extended safe HIL result:
- Command:
  `python tools/ee871_hil_runner.py --port COM17 --baud 115200 --include-extended --output-dir hil_results/extended_safe`
- Final verdict: PASS.
- Counts: 33 PASS, 0 FAIL, 0 SKIP, 0 OPERATOR_REVIEW_REQUIRED.
- Covered commands include the safe default sequence, `stress 500`, ten repeated
  `read` commands, three `probe`/`read`/`selftest` cycles, `recover`, `drv`,
  and `dirty`.
- Key results: `stress 500` success=500 errors=0; repeated reads returned
  Status OK; repeated selftests reported pass=27 fail=0 skip=0; final driver
  state READY/online yes/zero consecutive failures; persistent dirty stayed
  clean.
- Artifacts:
  - `hil_results/extended_safe/ee871_20260601T185921Z/serial_transcript.txt`
  - `hil_results/extended_safe/ee871_20260601T185921Z/summary.json`
  - `hil_results/extended_safe/ee871_20260601T185921Z/summary.md`

Manual resync check:
- Commands: `dirty`, `resync`, `dirty`.
- Final verdict: PASS.
- Result: pre-resync dirty clean, `resync` Status OK, post-resync dirty clean.
- Artifacts:
  - `hil_results/manual_resync/ee871_20260601T190024Z/serial_transcript.txt`
  - `hil_results/manual_resync/ee871_20260601T190024Z/summary.json`
  - `hil_results/manual_resync/ee871_20260601T190024Z/summary.md`

CLI alias note:
- Help advertises `version / ver`; it does not advertise `rv`.
- No `rv` alias was added because the current documented command surface is
  clear and the HIL flow does not depend on `rv`.

Remaining hardware gaps after this pass:
- Persistent interval write/readback/restore is recorded in the next section,
  but power-cycle persistence, CO2 calibration writes, and bus-address
  write/recovery remain unproven.
- Physical unplug/replug recovery was later operator-confirmed as PASS on
  2026-06-02. No automated transcript artifact exists for that manual step.
- Stuck-line fault/jig validation was not run.
- ESP32-S2 hardware HIL was not run.
- Pure ESP-IDF hardware HIL was not run.

## Persistent Configuration Hardware Validation - 2026-06-01

Scope:
- Validate persistent configuration behavior on approved bench hardware.
- Use actual CLI commands and record baseline values before writing.
- Prove measurement interval write/readback/restore without modifying CO2
  calibration or bus address.

Approval and hardware:
- Operator approval: bench unit and configuration changes approved in chat.
- Serial port: `COM17`, baud `115200`.
- Local time: 2026-06-01 21:35 CEST (`2026-06-01T19:35:00Z`).
- Target: ESP32-S3, PlatformIO environment `ex_bringup_s3`.
- Firmware build: `Jun  1 2026 20:57:04`.
- EE871 library: `0.3.0 (84a46b6, 2026-06-01 20:57:01, clean)`.

Baseline persistent values:
- Measurement interval: `150 ds` (`15.0 s`).
- CO2 interval factor: `85`.
- Operating mode: `0x55` (low power, measurement priority).
- Bus address: `0`.
- Part name: `EE871`.
- Serial: `1920935602368A..`
  (`31 39 32 30 39 33 35 36 30 32 33 36 38 41 00 00`).
- CO2 offset: `0 ppm`.
- CO2 gain: `32768`.
- Baseline dirty state from `drv`: `persistentConfigDirty: no`,
  `persistentConfigDirtyError: OK`, `resyncNeeded: no`.

Commands and results:
- Baseline read commands: `version`, `interval`, `factor`, `mode`, `addr`,
  `partname`, `serial`, `offset`, `gain`, `drv`.
- Measurement interval test: `interval 160`, wait `0.6 s`, `interval`,
  `dirty`, `resync`, `dirty`.
- Restore test: `interval 150`, wait `0.6 s`, `interval`, `dirty`, `resync`,
  `dirty`.
- Final read commands: `interval`, `factor`, `mode`, `addr`, `partname`,
  `serial`, `offset`, `gain`, `drv`.
- Final verdict: PASS, 31 PASS / 0 FAIL / 0 SKIP / 0 review.
- `interval 160` returned OK and read back `160 ds`.
- `interval 150` returned OK and restored/read back `150 ds`.
- Dirty diagnostics stayed clean after the test write, after `resync`, after
  restore, and at final `drv`.
- Final driver state was READY, online yes, zero consecutive failures, zero
  total failures.

Scope boundaries:
- Power-cycle persistence was not proven. No operator power-cycle step was
  executed during the automated run.
- CO2 offset/gain were read only and not modified, even with broad approval,
  because the prompt prefers read-only calibration validation and the interval
  write already proves persistent write/readback/restore on this bench unit.
- Bus address was read only and not modified because no automated
  post-power-cycle retarget/recovery path was used during this run.
- Failed-write/operator recovery was not physically induced. Native fake tests
  remain the evidence for partial persistent-write dirty/resync behavior.

Artifacts:
- `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/serial_transcript.txt`
- `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/summary.json`
- `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/summary.md`

Validation commands for this pass:
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python -m platformio test -e native`: PASS, 31 test cases succeeded in
  00:00:01.351.
- `git diff --check`: PASS; only Git line-ending conversion warnings were
  emitted for edited Markdown files.

## Final Release Polish - 1.0.0

Scope:
- Final documentation and release-readiness polish before merge/tag decision.
- No driver behavior changes were made in this pass.
- Version metadata was bumped to `1.0.0` through the repository version tooling.

Doxygen and public API docs:
- Public header Doxygen now documents the GPIO-style E2 boundary, the fact that
  the core is not Arduino `Wire` or ESP-IDF hardware I2C, callback ownership,
  blocking/timing expectations, thread/ISR safety, callback reentrancy limits,
  health tracking, non-copyable/non-movable driver compatibility, and the
  persistent dirty/resync model.
- `CommandTable.h`, `Config.h`, `EE871.h`, `Status.h`, and the generated
  `Version.h` template were polished for useful API documentation.
- `Doxyfile` project version is `1.0.0`; generated HTML output remains ignored
  and was removed after the Doxygen run.

README/docs/changelog/release notes:
- `README.md` now uses production-oriented wording without claiming full
  field-proof status.
- `docs/README.md` indexes the maintained docs and separates current guidance
  from historical audit records and PDF extracts.
- `CHANGELOG.md` contains the `1.0.0` release section and compare links.
- `docs/EE871_E2_RELEASE_NOTES_1.0.0.md` records release summary,
  compatibility, validation evidence, known limitations, migration notes, HIL
  artifact paths, and a release checklist.

Hardware matrix finalization:
- ESP32-S3 Arduino safe HIL remains recorded as PASS.
- ESP32-S3 Arduino extended safe HIL remains recorded as PASS.
- ESP32-S3 persistent measurement interval write/readback/restore remains
  recorded as PASS.
- Physical unplug/replug recovery is recorded as PASS from an
  operator-confirmed manual test on 2026-06-02. It is not automated HIL
  evidence, and no raw transcript artifact exists for that manual physical
  step.
- Hardware transcripts were captured before the final `1.0.0` version
  metadata/docs polish and still show their exact firmware/library version.

Final validation commands:
- `python tools/check_core_timing_guard.py`: PASS.
- `python tools/check_cli_contract.py`: PASS.
- `python tools/check_idf_example_contract.py`: PASS.
- `python scripts/generate_version.py check`: PASS, `Version.h` up to date.
- `python -m py_compile tools/ee871_hil_runner.py`: PASS.
- `doxygen --version`: `1.15.0`.
- `doxygen Doxyfile`: PASS, no warnings emitted.
- `python -m platformio test -e native`: PASS, 31/31 tests in 00:00:01.405.
- `python -m platformio run -e ex_bringup_s3`: PASS in 00:00:21.670.
- `python -m platformio run -e ex_bringup_s2`: PASS in 00:00:21.579.
- `python -m platformio pkg pack`: PASS; generated
  `ee871-e2-1.0.0.tar.gz` was removed before staging.
- `git diff --check`: PASS; Git reported only expected line-ending conversion
  warnings for edited files.
- Core framework-boundary scan of `include/` and `src/`: PASS; no Arduino,
  `Wire`, hardware I2C, `String`, `Serial`, `TwoWire`, or `HardwareSerial`
  usage in core code.

Unavailable local verification:
- `idf.py --version`: unavailable in this workspace,
  `CommandNotFoundException`.
- Local pure IDF builds for `esp32s3` and `esp32s2`: not run because `idf.py`
  is unavailable.
- `gh run list --limit 5`: unavailable in this workspace,
  `CommandNotFoundException`; GitHub Actions IDF matrix status is not visible
  locally through `gh`.

Remaining limitations:
- Pure ESP-IDF build success still needs proof from GitHub Actions or local
  `idf.py` builds.
- ESP32-S2 hardware HIL is not recorded.
- Pure ESP-IDF hardware HIL is not recorded.
- Power-cycle persistence is not proven.
- CO2 calibration writes and bus-address write/recovery were intentionally not
  tested on hardware.
- Stuck-line fault-jig validation for SDA/SCL is not recorded.

Merge readiness verdict:
- Ready after final checks and CI pass.

Release readiness verdict:
- Ready for `1.0.0` if maintainers accept the documented limitations: pure IDF
  CI must be verified by GitHub Actions or local IDF, ESP32-S2 hardware HIL and
  fault-jig tests remain future validation, and the existing HIL transcripts
  predate the final version metadata/docs polish.

Field/industry-grade wording:
- Use "production-oriented and validation-backed for the tested
  ESP32-S3/EE871 bench setup."
- Do not claim "fully field-proven" or equivalent coverage across all physical
  fault cases.
