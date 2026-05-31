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
