# EE871 IDF-Merged Industry-Readiness Audit

Date: 2026-05-29
Repository: `C:\Users\HonzovoSpectre\Documents\Projects\EE871-E2`
Branch: `audit/ee871-idf-merged-industry-readiness`
Audit mode: report-only / no implementation
IDF merge classification: `QUALIFYING_IDF_MERGED`

## Executive Summary

The ESP-IDF port is merged into `main` and the core is mostly well shaped for production use: it is framework-neutral, transport-injected, and does not own Arduino, ESP-IDF, GPIO setup, or global bus objects. This repository is not a normal I2C device driver; it is an EE871 E2 GPIO-style bus driver, so the production risk is deterministic GPIO timing, clock-stretch handling, flash-write delays, and recovery behavior.

The main readiness gaps are not architecture fundamentals. They are validation gaps: no pure `idf.py` build was run locally or in CI, native tests are shallow and do not fault-inject the E2 protocol, and multi-byte persistent settings can leave the sensor partially changed if a later write or verify step fails. The library should not be called industry-grade until those gaps are closed and real EE871 hardware fault cases are captured.

## IDF Merge Evidence

- Default branch evidence: `origin/HEAD -> refs/heads/main`.
- Merge commit: `34f3143e85e280c5d385ddcbc7089c912951777c`.
- Merge message: `Merge pull request #2 from janhavelka:feature/ee871-e2-idf-port`.
- Merged branch tip: `58d23fde9ba956dc1619d8a74c09188932351a19`.
- Branch ancestry: `feature/ee871-e2-idf-port` is an ancestor of `main`.
- IDF artifacts present on `main`:
  - `CMakeLists.txt`
  - `idf_component.yml`
  - `examples/idf/basic_bringup/main/main.cpp`
  - `examples/idf/basic_bringup/main/CMakeLists.txt`
  - `examples/idf/common/E2GpioTransport.h`
  - `tools/check_idf_example_contract.py`
  - `docs/IDF_PORT.md`
  - `docs/IDF_PORT_IMPLEMENTATION.md`
- Limitation: the merge evidence is strong, but the native ESP-IDF example was not built with `idf.py` in this audit because `idf.py` is not installed or not on `PATH`.

## Readiness Classification

Engineering-grade with major gaps.

The core architecture is suitable for hardening. The IDF example is native IDF and the Arduino S2/S3 PlatformIO builds pass. The library still needs E2 fault-injection tests, partial persistent-state handling, explicit thread/ISR contracts, pure ESP-IDF CI, and hardware validation before a production or industry-grade claim.

## Scope Reviewed

- `include/EE871/`
- `src/`
- `examples/01_basic_bringup_cli/`
- `examples/idf/basic_bringup/`
- `examples/idf/common/`
- `examples/common/`
- `test/`
- `tools/`
- `docs/`
- `README.md`
- `platformio.ini`
- `library.json`
- `CMakeLists.txt`
- `idf_component.yml`
- `.github/workflows/ci.yml`

## Datasheet / Documentation Sources Found

- `docs/E2_interface_specification_v4_1.pdf`
- `docs/E2_interface_utilising_AN0105.pdf`
- `docs/EE871_digital_interface_user_guide.pdf`
- `docs/EE871_E2_CO2_interface_AN1611-1.pdf`
- `docs/EE871_E2_interface_addendum.pdf`
- `docs/EE871_E2_Protocol_and_Register_Map.md`
- `docs/EE871_EE240_wireless_user_guide.pdf`
- `docs/IDF_PORT.md`
- `docs/IDF_PORT_IMPLEMENTATION.md`

## Scorecard

| Area | Rating | Notes |
| --- | --- | --- |
| IDF merge evidence | Strong | Merge commit and IDF artifacts are present on `main`. |
| Core framework neutrality | Strong | No Arduino, Wire, ESP-IDF, FreeRTOS, logging, heap, or global bus dependencies found in `include/` and `src/`. |
| I2C ownership/injection | Good | E2 line control and delay callbacks are injected through `Config`; this is not hardware I2C. |
| ESP-IDF component correctness | Medium | Component files exist, but no local/CI `idf.py build` evidence. |
| ESP-IDF example correctness | Good | Native `app_main`, IDF GPIO, `esp_timer`, FreeRTOS delay, fixed buffers; no Arduino compatibility path found. |
| Status/error model | Good | Public fallible APIs return `Status`; PEC and E2 errors exist. Runtime fault coverage is weak. |
| Timing/determinism | Medium | Bit waits are bounded, but flash-write delays can block up to configured limits. |
| Device-specific correctness | Good | Local protocol docs match reviewed control bytes, PEC sum, MV3/MV4 mapping, write delays, and E2 timing. |
| Partial hardware state handling | Weak | Multi-byte persistent settings can partially apply without a dirty/sync-needed diagnostic. |
| Health/recovery behavior | Medium | Health model exists, but runtime E2 fault paths are not fake-tested. |
| Thread/ISR contract | Weak | No explicit public thread-safety or ISR-safety contract found. |
| Tests/fault injection | Weak | Native tests pass but are shallow; no fake E2 bus/fault harness. |
| ESP-IDF build coverage | Weak | Contract checker passes locally; pure `idf.py build` missing locally and in CI. |
| Arduino ESP32-S2/S3 readiness | Good | `ex_bringup_s3` and `ex_bringup_s2` PlatformIO builds passed. |
| Documentation honesty | Medium | IDF and timing notes exist; thread/ISR and partial-state behavior need stronger contracts. |
| Hardware validation | Unknown | No hardware commands were run in this audit. |

## What Is Strong

- Core dependency boundary is clean: no Arduino, Wire, ESP-IDF, FreeRTOS, `String`, logging macros, global bus, or heap allocation were found in `include/` and `src/`.
- The E2 transport is injected through `Config::setScl`, `setSda`, `readScl`, `readSda`, `delayUs`, and `busUser` in `include/EE871/Config.h`.
- Clock-stretch waits are bounded by `bitTimeoutUs` and `byteTimeoutUs` (`include/EE871/Config.h:44`, `src/EE871.cpp:248`, `src/EE871.cpp:925`).
- The IDF bring-up example uses native IDF entry and timing: `app_main()` at `examples/idf/basic_bringup/main/main.cpp:2433`, `esp_timer`-based time at `examples/idf/basic_bringup/main/main.cpp:60`, and FreeRTOS delay at `examples/idf/basic_bringup/main/main.cpp:65`.
- The IDF console input is nonblocking and the main loop calls `device.tick(nowMs())` every loop (`examples/idf/basic_bringup/main/main.cpp:2352`, `examples/idf/basic_bringup/main/main.cpp:2470`).
- Local protocol documentation is present and specific enough to audit EE871 E2 framing and register behavior.

## High-Severity Findings

### H1. Pure ESP-IDF build is not validated locally or in CI

Severity: High

Evidence:
- `CMakeLists.txt`, `idf_component.yml`, and `examples/idf/basic_bringup/` exist on `main`.
- `.github/workflows/ci.yml` installs and runs PlatformIO workflows, but no `idf.py build` job was found.
- Local command `idf.py --version` failed: `The term 'idf.py' is not recognized as the name of a cmdlet, function, script file, or operable program.`

Impact:
- A native IDF consumer can still receive a component/example that passes Arduino/PlatformIO builds but fails under ESP-IDF CMake, target selection, linker dependencies, or IDF version changes.

Recommended remediation:
- Add CI jobs that run:
  - `idf.py -C examples/idf/basic_bringup set-target esp32s3 build`
  - `idf.py -C examples/idf/basic_bringup set-target esp32s2 build`
- Keep `tools/check_idf_example_contract.py` in CI as a fast boundary check.

Suggested tests:
- Add CI matrix coverage for ESP32-S2 and ESP32-S3 pure IDF builds.
- Capture the exact ESP-IDF version in the final hardening report.

### H2. Runtime E2 transport faults are not covered by native fake-transport tests

Severity: High

Evidence:
- Local native tests passed, but coverage is mostly status/config/precondition checks.
- The core uses tracked E2 operations and health state, but tests do not fault-inject runtime line reads, clock-stretch timeout, NACK/PEC mismatch, read/write verify failure, offline transition, or recovery.
- E2 byte-level reads/writes are implemented in `src/EE871.cpp:974` and `src/EE871.cpp:1040`.

Impact:
- Production failures in cable, pull-up, clock stretch, line stuck, PEC corruption, or device reset paths may not map to the intended `Status` and health transitions.

Recommended remediation:
- Add a native fake E2 transport with deterministic SCL/SDA line behavior and scripted failures.
- Test raw-vs-tracked health behavior, `probe()`, `recover()`, stuck-SCL timeout, PEC mismatch, and verify failure.

Suggested tests:
- Fake SCL held low until `bitTimeoutUs`.
- Fake PEC mismatch on every read path.
- Fake write ACK but verify mismatch for `customWrite()`.
- Offline threshold and manual recovery transitions.

### H3. Multi-byte persistent settings can leave partial hardware state

Severity: High

Evidence:
- `writeMeasurementInterval()` writes low byte then high byte, then delays and verifies both bytes (`src/EE871.cpp:557`, `src/EE871.cpp:561`, `src/EE871.cpp:566`, `src/EE871.cpp:570`).
- `writeCo2Offset()` and `writeCo2Gain()` write low and high bytes through separate `customWrite()` calls (`src/EE871.cpp:867`, `src/EE871.cpp:871`, `src/EE871.cpp:890`, `src/EE871.cpp:894`).
- No dirty/sync-needed diagnostic or partial-state field was found.

Impact:
- A failure after the first byte can leave persistent sensor configuration half-updated. The caller receives an error, but later diagnostics may not state that the sensor needs explicit resync or operator inspection.

Recommended remediation:
- Track persistent-setting dirty state when a multi-byte write fails after any earlier byte may have committed.
- Preserve the original failing `Status`.
- Clear dirty state only after a full readback/resync proves the persistent fields are coherent.

Suggested tests:
- Fail each transaction position in `writeMeasurementInterval()`.
- Fail high-byte writes in CO2 offset/gain.
- Verify diagnostics expose dirty state and recovery/resync clears it only after successful readback.

## Medium-Severity Findings

### M1. Public copy/move operations are not explicitly disabled

Severity: Medium

Evidence:
- `class EE871` starts at `include/EE871/EE871.h:41`.
- No deleted copy or move constructor/assignment was found.
- The instance stores callbacks, user context, cached features, and health state.

Impact:
- Accidental copies can duplicate runtime state while sharing the same external bus callback context. That can make health, cached feature bits, and timing state misleading.

Recommended remediation:
- Delete copy and move operations unless a reviewed use case requires moving:
  - `EE871(const EE871&) = delete;`
  - `EE871& operator=(const EE871&) = delete;`
  - `EE871(EE871&&) = delete;`
  - `EE871& operator=(EE871&&) = delete;`

Suggested tests:
- Add `static_assert(!std::is_copy_constructible_v<EE871::EE871>)`.

### M2. Thread-safety and ISR-safety contracts are not explicit enough

Severity: Medium

Evidence:
- No explicit public thread/ISR contract was found in public headers.
- Public APIs can perform E2 bus I/O and call injected delays.

Impact:
- A user may call the driver from multiple tasks or an ISR and corrupt state, block an interrupt context, or recursively enter the same transport.

Recommended remediation:
- Document that instances are not thread-safe, all public APIs must be externally serialized, public APIs are not ISR-safe, and callbacks must not re-enter the same driver instance.

Suggested tests:
- Documentation/guard-script check for a thread/ISR section in README and Doxygen headers.

### M3. Flash-write delay paths block cooperatively only through `delayUs`

Severity: Medium

Evidence:
- `sleepMs()` calls `cfg.delayUs(1000, cfg.busUser)` in a loop (`src/EE871.cpp:191`).
- Defaults are `writeDelayMs = 150` and `intervalWriteDelayMs = 300` (`include/EE871/Config.h:47`).
- Configuration limits allow up to 5000 ms (`include/EE871/CommandTable.h:159`).

Impact:
- In a shared-bus or soft real-time system, a persistent write can block the calling task for hundreds of milliseconds or longer.

Recommended remediation:
- Keep blocking APIs, but document them as explicit long-latency maintenance operations.
- Consider a nonblocking persistent-write state machine only if field use requires it.

Suggested tests:
- Validate max-delay rejection and elapsed-delay behavior with a fake delay callback.

## Low-Severity Findings

### L1. The repository is an E2 bus driver, not a hardware I2C driver

Severity: Low

Evidence:
- README states EE871 uses an E2 bus and no `Wire`/I2C dependency.
- IDF example owns GPIO, not IDF I2C.

Impact:
- Cross-repository audit language that says "I2C" can mislead future reviewers. Production concerns differ from I2C sensors.

Recommended remediation:
- Keep all reports and README wording explicit: EE871-E2 uses bit-banged E2 GPIO signaling.

Suggested tests:
- Documentation guard that avoids claiming hardware I2C support for EE871-E2.

## Device-Specific Correctness Checklist

| Item | Status | Notes |
| --- | --- | --- |
| Exact transport/protocol identified | PASS | E2 GPIO-style bus, not hardware I2C. |
| I2C address assumptions avoided | PASS | `deviceAddress` is E2 protocol address, not IDF I2C device address. |
| Command/control-byte mapping | PASS | Reviewed against local protocol docs. |
| PEC/checksum handling | PASS | PEC validation is implemented and documented. |
| CO2 MV3/MV4 scaling | PASS | No mismatch found against local protocol notes. |
| Warm-up behavior | PARTIAL | Documentation exists, but no hardware validation run. |
| Measurement-not-ready semantics | PARTIAL | Status paths exist; runtime fault tests are missing. |
| Persistent custom write timing | PARTIAL | Delay is bounded; nonblocking option and partial-state tracking are missing. |
| NVM/write endurance warnings | PARTIAL | Needs stronger production-facing warning for custom writes. |
| Power-up and recovery | PARTIAL | Recovery exists; hardware/fault injection not validated. |
| Stale sample handling | UNKNOWN | No hardware sample freshness validation was run. |
| Hardware validation | UNKNOWN | No hardware commands were run in this audit. |

## API Latency / Blocking Model

| API | I/O transactions | Other waits | Worst-case bound | Notes |
| --- | ---: | --- | --- | --- |
| `begin()` | Up to 2 raw group reads + 1 pointer write + 3 reads | Bus reset clocks with SCL polling | Bounded by E2 bit/byte timeout per bit/byte plus callback duration | May cache feature groups. |
| `tick(nowMs)` | 0 | None | O(1) | Records time only. |
| `readStatus()` / CO2 reads | 1 E2 read each | Bit-level delays | Bounded by `bitTimeoutUs` / `byteTimeoutUs` | Blocking. |
| `customRead(addr, len)` | 1 pointer write + `len` reads | Bit-level delays | Scales linearly with `len` | 16-byte identity reads perform 17 transactions. |
| `customWrite(addr, value)` | Write + readback verify | `writeDelayMs` | Up to write delay plus E2 transaction bounds | Verify mismatch returns `E2_ERROR`. |
| `writeMeasurementInterval()` | 2 writes + 2 readbacks | `intervalWriteDelayMs` | Up to interval delay plus 4 E2 transactions | Partial hardware state possible. |
| `writeCo2Offset()` / `writeCo2Gain()` | Two `customWrite()` calls | Two write delays | Scales with two full write/verify cycles | Partial hardware state possible. |
| `probe()` | Raw E2 reads | Bit-level delays | Bounded by E2 timeouts | Diagnostic only; should not update health. |
| `recover()` | Probe/resync traffic | Bit-level delays | Bounded by E2 timeouts | Needs fault-injection coverage. |

## Partial-State / Cache Consistency Assessment

Single-byte `customWrite()` verifies readback and is clear. Multi-byte persistent writes are the weak point. `writeMeasurementInterval()`, `writeCo2Offset()`, and `writeCo2Gain()` can commit one byte and fail on the second byte or readback. The current API returns the failing `Status`, but no diagnostic flag says the hardware may be partially configured. A dirty/resync-needed field should be added before production claims.

## Tests and Build Coverage

Run locally:
- `git status --short`: clean before report edits.
- `python --version`: `Python 3.13.12`.
- `python -m platformio --version`: `PlatformIO Core, version 6.1.19`.
- `python tools/check_core_timing_guard.py`: PASS, `Core timing guard PASSED`.
- `python tools/check_cli_contract.py`: PASS, `CLI contract PASSED`.
- `python tools/check_idf_example_contract.py`: PASS, `IDF example contract PASSED`.
- `python scripts/generate_version.py check`: PASS, `include\EE871\Version.h` up to date.
- `python -m platformio test -e native`: PASS, `14 test cases: 14 succeeded`.
- `python -m platformio run -e ex_bringup_s3`: PASS, `SUCCESS`.
- `python -m platformio run -e ex_bringup_s2`: PASS, `SUCCESS`.
- `python -m platformio pkg pack`: PASS, wrote `ee871-e2-0.3.0.tar.gz`; tarball removed after audit.
- `idf.py --version`: FAIL, command not found.

Present in CI:
- PlatformIO S2/S3 builds.
- Native tests.
- Core timing guard.
- CLI contract.
- `library.json` JSON validation.

Not run:
- `idf.py -C examples/idf/basic_bringup set-target esp32s3 build`: not run because `idf.py` is unavailable.
- `idf.py -C examples/idf/basic_bringup set-target esp32s2 build`: not run because `idf.py` is unavailable.
- Hardware validation: not run.

Missing:
- Native fake E2 protocol/fault-injection suite.
- Pure IDF build in CI.
- Dedicated CI invocation of `tools/check_idf_example_contract.py`.

## ESP-IDF Port Assessment

- Pure ESP-IDF component: Present (`idf_component.yml`, root `CMakeLists.txt`).
- Pure ESP-IDF example: Present (`examples/idf/basic_bringup`).
- Native IDF APIs, not Arduino: Yes. Uses `app_main`, IDF GPIO, `esp_timer`, FreeRTOS delay, nonblocking stdin, and fixed buffers.
- External bus ownership: Yes. Example owns GPIO setup and injects line callbacks.
- IDF I2C API: Not applicable; this device uses E2 GPIO signaling.
- Error mapping: Partially applicable. GPIO init errors are handled in the example, but line callbacks themselves have no error return.
- Locking: No shared-bus locking shown; likely acceptable for a single-threaded diagnostic example, not a production bus manager.
- CI IDF build: Missing.

## Documentation Assessment

Missing or incomplete documentation contracts:
- Explicit thread-safety and ISR-safety in public headers.
- Explicit partial persistent-state behavior after failed multi-byte writes.
- Stronger "diagnostic/bring-up, not production bus manager" labeling for the IDF example.
- Hardware validation matrix with exact commands/results.
- Clear warning that EE871-E2 is not hardware I2C.

## Hardware Validation Needed

| Scenario | Target | Expected evidence |
| --- | --- | --- |
| Power-up `begin()` | ESP32-S2 and ESP32-S3 | Status, feature cache, health counters. |
| CO2 fast/average reads | Known-good EE871 | Values in expected range, PEC checks pass. |
| E2 line diagnostics | Normal wiring | SDA/SCL idle and toggle behavior captured. |
| Stuck SCL/SDA | Fault injection or wiring jig | Timeout status and health transition. |
| PEC corruption | Fake transport or hardware fault jig | `PEC_MISMATCH`, no false success. |
| Persistent interval write | Bench sensor | Write, delay, readback, power-cycle persistence. |
| Interrupted multi-byte write | Fault injection | Dirty/resync-needed diagnostic after partial write. |
| Recovery after unplug/replug | Hardware | Offline transition and manual recover behavior. |

## Recommended Implementation Plan

### P0 - Must fix before production claim

- Add native fake E2 transport with fault injection.
- Add dirty/resync-needed diagnostics for multi-byte persistent writes.
- Add pure ESP-IDF S2/S3 CI builds or record local `idf.py` builds.
- Add thread/ISR public API contract.

### P1 - Should fix before release/merge

- Delete copy/move operations.
- Wire `tools/check_idf_example_contract.py` into CI.
- Add tests for clock-stretch timeout, PEC mismatch, write verify failure, and recovery.
- Expand README latency table for all persistent write APIs.

### P2 - Nice hardening / later

- Consider nonblocking persistent-write state machine if production usage cannot tolerate long blocking writes.
- Add documentation guard for EE871-E2 vs hardware I2C wording.

## Proposed Branch for Future Hardening

`hardening/ee871-e2-industry-readiness`

## Final Verdict

Not ready for an industry-grade claim as-is. It is ready for a focused hardening pass, with the largest gaps in E2 fault tests, pure ESP-IDF build proof, partial persistent-state diagnostics, and hardware validation.
