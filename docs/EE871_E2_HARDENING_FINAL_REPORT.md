# EE871-E2 Hardening Final Report

Date: 2026-05-31
Branch: `hardening/ee871-e2-industry-readiness`

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

Remaining prompts/work planned:
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
- The implementation marks dirty at the multi-byte wrapper boundary after a
  first byte returns success. It does not add a lower-level transaction-phase
  API to distinguish every possible post-PEC/STOP ambiguity.

Tests run:
- `python tools/check_core_timing_guard.py`: PASS, `Core timing guard PASSED`.
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python tools/check_idf_example_contract.py`: PASS, `IDF example contract PASSED`.
- `python scripts/generate_version.py check`: PASS, `include\EE871\Version.h` up to date.
- `python -m platformio test -e native`: PASS, 28 test cases succeeded in 00:00:02.538.
- `python -m platformio run -e ex_bringup_s3`: PASS, `SUCCESS` in 00:00:20.415.
- `python -m platformio run -e ex_bringup_s2`: PASS, `SUCCESS` in 00:00:27.054.
- `git diff --check`: PASS; only Git line-ending conversion warnings were emitted.
