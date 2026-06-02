# EE871-E2 Release Notes 1.0.0

Date: 2026-06-02
Branch: `hardening/ee871-e2-industry-readiness`

## Summary

`1.0.0` is the first production-oriented EE871-E2 release candidate from the
hardening branch. It keeps the core driver framework-neutral, uses injected
GPIO-style E2 callbacks, adds native fault-injection coverage, exposes
persistent dirty/resync diagnostics, and records repeatable hardware evidence
for the tested ESP32-S3/EE871 bench setup.

This release should be described as production-oriented and validation-backed
for the tested bench setup. It should not be described as fully field-proven
across every physical fault case.

## Key Features

- Framework-neutral EE871-E2 core with injected `setScl`, `setSda`, `readScl`,
  `readSda`, and `delayUs` callbacks.
- Arduino diagnostic CLI example and native ESP-IDF basic bring-up example.
- Managed synchronous driver contract with documented blocking, timing,
  thread-safety, ISR-safety, and callback restrictions.
- READY/DEGRADED/OFFLINE health tracking for tracked E2 operations.
- Deterministic native fake E2 transport and runtime fault-injection tests.
- Persistent dirty-state diagnostics for non-atomic multi-byte persistent
  writes.
- `resyncPersistentConfig()` to re-read coherent persistent fields and clear
  dirty state only after successful verification.
- Python HIL runner with raw transcript, JSON summary, Markdown report, and
  conservative PASS/FAIL/REVIEW/INCOMPLETE verdicts.
- Hardware validation matrix and documentation index.

## Compatibility

- Source compatibility break: `EE871::EE871` is non-copyable and non-movable.
  Keep driver instances in stable storage and pass references or pointers.
- `SettingsSnapshot` layout changed to include persistent dirty diagnostics.
  ABI/layout-sensitive users must rebuild and must not persist raw snapshot
  layouts across versions.
- Normal single-instance Arduino or ESP-IDF users should only need to rebuild.
- The core still does not own GPIO, pins, bus handles, tasks, locks, Arduino
  `Wire`, ESP-IDF hardware I2C handles, or framework objects.

## Validation Evidence

- Native host tests: 31 passing in the latest hardening/readiness runs.
- Arduino PlatformIO builds: `ex_bringup_s3` and `ex_bringup_s2` passing in
  local readiness runs.
- ESP32-S3 safe default HIL on `COM17`: PASS.
- ESP32-S3 extended safe HIL on `COM17`: PASS.
- ESP32-S3 persistent measurement interval write/readback/restore on `COM17`:
  PASS.
- Physical unplug/replug recovery: PASS as an operator-confirmed manual test on
  2026-06-02. No automated HIL transcript artifact is recorded for this step.
- Hardware artifact version note: the automated hardware transcripts were
  captured before the final `1.0.0` version metadata/docs polish and report the
  exact firmware/library version shown in the hardware matrix. The release
  polish pass did not add a new hardware transcript.
- Pure ESP-IDF build proof: CI coverage is configured, but this workspace still
  needs either a passing GitHub Actions matrix or local `idf.py` builds to prove
  pure IDF success.

## Hardware Artifacts

- Safe default HIL transcript:
  `hil_results/safe_default/ee871_20260601T185912Z/serial_transcript.txt`
- Safe default HIL JSON/Markdown:
  `hil_results/safe_default/ee871_20260601T185912Z/summary.json`,
  `hil_results/safe_default/ee871_20260601T185912Z/summary.md`
- Extended safe HIL transcript:
  `hil_results/extended_safe/ee871_20260601T185921Z/serial_transcript.txt`
- Extended safe HIL JSON/Markdown:
  `hil_results/extended_safe/ee871_20260601T185921Z/summary.json`,
  `hil_results/extended_safe/ee871_20260601T185921Z/summary.md`
- Manual resync transcript:
  `hil_results/manual_resync/ee871_20260601T190024Z/serial_transcript.txt`
- Manual resync JSON/Markdown:
  `hil_results/manual_resync/ee871_20260601T190024Z/summary.json`,
  `hil_results/manual_resync/ee871_20260601T190024Z/summary.md`
- Persistent interval validation transcript:
  `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/serial_transcript.txt`
- Persistent interval validation JSON/Markdown:
  `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/summary.json`,
  `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/summary.md`

## Known Limitations

- Pure ESP-IDF build success remains unproven until GitHub Actions or local
  `idf.py` builds pass for `esp32s3` and `esp32s2`.
- ESP32-S2 hardware HIL is not recorded.
- Pure ESP-IDF hardware HIL is not recorded.
- Power-cycle persistence of persistent settings is not proven.
- CO2 offset/gain calibration writes were intentionally not tested on hardware.
- Bus-address write/recovery was intentionally not tested on hardware.
- Stuck-line fault-jig tests for SDA/SCL are not recorded.
- Physical unplug/replug recovery is operator-confirmed manual evidence, not an
  automated HIL transcript.

## Migration From 0.3.0

1. Rebuild all applications against `1.0.0`.
2. Replace by-value copies or moves of `EE871::EE871` with stable storage and
   references or pointers.
3. Treat `SettingsSnapshot` as a runtime inspection struct only; do not persist
   or externally serialize raw layouts across versions.
4. Review application threading. Use one owner task/context or external
   serialization for every public driver call.
5. Ensure transport callbacks are bounded, deterministic, and do not recurse
   into the same driver instance.
6. Use `persistentConfigDirty()`, `persistentConfigDirtyError()`, `dirty`, and
   `resync` to inspect persistent dirty/resync state after maintenance writes.

## Release Checklist

- `python scripts/generate_version.py check`
- `python tools/check_core_timing_guard.py`
- `python tools/check_cli_contract.py`
- `python tools/check_idf_example_contract.py`
- `python -m py_compile tools/ee871_hil_runner.py`
- `python -m platformio test -e native`
- `python -m platformio run -e ex_bringup_s3`
- `python -m platformio run -e ex_bringup_s2`
- `python -m platformio pkg pack`
- `git diff --check`
- `doxygen Doxyfile` if Doxygen is installed.
- `idf.py -C examples/idf/basic_bringup set-target esp32s3 build` and
  `idf.py -C examples/idf/basic_bringup set-target esp32s2 build` if ESP-IDF is
  installed.
- Verify GitHub Actions IDF matrix if `gh` or the GitHub UI is available.
- Remove generated package archives and generated Doxygen HTML before commit.
- Tag `v1.0.0` only after maintainers accept the documented limitations and
  required CI evidence.
