# Prompt 03 Checked CO2 Samples Handoff

Date: 2026-07-28
Branch: `feature/ee871-hardening-series`
Baseline commit: `432519862430703a056d4ab37247adcf5fbbc1d1`
Version: `1.0.0` (unchanged)

## Baseline

The branch was clean and synchronized with its upstream before Prompt 03.
Prompt 01 and Prompt 02 handoffs existed and their corrective commits were
present. Baseline native validation passed 65/65 tests. No dirty files were
present.

## Public Contracts

- Appended `Err::CO2_SENSOR_ERROR = 17`, preserving
  `VERIFY_MISMATCH = 15` and `OFFLINE = 16`.
- Added `Co2ValueKind`, `Co2SensorError`, and `Co2ReadResult` with mandatory
  attempted flags and exact value/status/error-code step results.
- Added checked range constants `cmd::CO2_PPM_MIN = 0` and
  `cmd::CO2_PPM_MAX = 50000`. The maximum is a broad library guard, not a
  claim that every physical EE871 variant has that range.
- Added cache-only `hasCo2OffsetGain()` and `hasCo2AdjustmentPoints()`.
- Appended `CHECKED_CO2_AVERAGE = 13` and `CHECKED_CO2_FAST = 14`.
- Existing raw `readCo2Average()`, `readCo2Fast()`, `readStatus()`, and
  `readErrorCode()` signatures and raw behavior remain unchanged.

## Checked Procedure

Both public checked methods call one private `_readCo2Sample()` implementation.
The exact step and return behavior is:

| Condition | Attempted steps | Returned status | Retained evidence |
| --- | --- | --- | --- |
| Value low/high transfer fails | Value only | Original transfer/protocol status | `valueReadStatus`; ppm invalid; later steps unattempted |
| Value succeeds, status fails | Value, status | Original status-read failure | Raw ppm retained invalid; status invalid; error detail unattempted |
| Valid status reports CO2 error, no detail capability | Value, status | `CO2_SENSOR_ERROR`, detail = status byte | Status valid, `sensorError=UNKNOWN`, detail step unattempted |
| CO2 error and detail pointer/read fails | Value, status, error detail | Original pointer/read failure | Raw ppm and status retained; `sensorError=UNKNOWN`; exact detail-step failure |
| CO2 error and detail succeeds | Value, status, error detail | `CO2_SENSOR_ERROR`, detail = raw code | Raw code valid; documented normalized error or `UNKNOWN` |
| Clean status, ppm above 50000 | Value, status | `OUT_OF_RANGE`, detail = raw ppm | Raw ppm retained invalid; status valid |
| Clean status, ppm at or below 50000 | Value, status | `OK` | Raw ppm and status valid; `sensorError=NONE` |

Codes 1, 200, 201, and 202 map respectively to supply-voltage-low,
sensor-counts-low, sensor-counts-high, and supply-voltage-breakdown-at-peak.
Every other detailed code, including zero while the status error bit is set,
maps to `UNKNOWN` while retaining the raw code.

## Ordering Evidence

The existing fake transaction record proves exact ordered events after
activity counters are reset:

| Index | Average success | Fast success | Error-detail branch |
| ---: | --- | --- | --- |
| 0 | MV4 low (`0xE`) read | MV3 low (`0xC`) read | Selected low read |
| 1 | MV4 high (`0xF`) read | MV3 high (`0xD`) read | Selected high read |
| 2 | Status (`0x7`) read | Status (`0x7`) read | Status read |
| 3 | - | - | Custom pointer (`0x5`) write to `0xC1` |
| 4 | - | - | Custom-data (`0x51`) read at `0xC1` |

Deterministic transfer failure injection covers indexes 0 through 4. Tests
prove that no status follows either value-byte failure and no later
error-detail transaction follows pointer failure.

## Health Evidence

Checked procedures reuse existing tracked raw methods:

- clean status performs three successful tracked transfers;
- successful detailed error acquisition performs five successful tracked
  transfers;
- `CO2_SENSOR_ERROR` and checked `OUT_OF_RANGE` add no transport failure and
  leave a transport-healthy driver `READY`;
- a failed error-code pointer or data read increments the existing transport
  failure counter and preserves its precise status;
- an offline checked call records a value attempt with `OFFLINE`, performs
  zero line I/O, and changes no health counter.

No sample-level success/failure counter was added.

## Timing Bounds

Both new operation kinds use:

```text
CHECKED = 2*READ(value) + READ(status)
          + COMPLETION_WRITE(writeDelayMs) + READ(error code)
        = 4*READ + COMPLETION_WRITE(writeDelayMs)
```

For the minimum-hold reference configuration this is 935,612 us, ceiling
divided to 936 ms. Static and instance queries are bus-silent. Native tests
exercise the complete five-transaction error-detail branch with near-byte-limit
data stretching and verify observed fake time remains within the bound.

## Files

- `include/EE871/CommandTable.h`
- `include/EE871/Status.h`
- `include/EE871/EE871.h`
- `src/EE871.cpp`
- `test/support/FakeE2Transport.h`
- `test/test_basic.cpp`
- `README.md`
- `CHANGELOG.md`
- `AGENTS.md`
- `docs/EE871_E2_Protocol_and_Register_Map.md`
- `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md`
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`
- this handoff

No example, version, library metadata, build-profile, product, firmware,
schema, task, queue, pin, or persistence implementation was changed.

## Validation

- `python tools/check_core_timing_guard.py`: PASS.
- `python tools/check_cli_contract.py`: PASS.
- `python tools/check_idf_example_contract.py`: PASS.
- `python scripts/generate_version.py check`: PASS; `Version.h` is current.
- `python -m platformio test -e native`: PASS, 75/75.
- `python -m platformio run -e ex_bringup_s3`: PASS.
- `python -m platformio run -e ex_bringup_s2`: PASS.
- `doxygen Doxyfile`: PASS.
- `git diff --check`: PASS; only Git line-ending conversion warnings.
- `idf.py`: unavailable on `PATH`, so local pure ESP-IDF builds were not run.

No HIL, physical sensor, waveform, network, Cloud, or long-run validation was
performed for Prompt 03.

## Post-Publication Audit Correction

A fresh audit of published Prompt 03 commit `82b656e` found no production-code
or test defect. It corrected one stale guideline sentence that described the
status-read measurement trigger as unconditional. `AGENTS.md` now consistently
states that status evaluates the last measured value and may start the next
measurement sequence under the documented conditions.

The correction changes no API, timing, transport, health, persistence,
firmware, product, example, build-profile, or version behavior. The full
Prompt 03 software validation set was rerun after the wording correction and
remained passing. `idf.py` was still unavailable, and no HIL or physical
hardware validation was performed.

## Explicit Deferrals

- Prompt 04 owns persistent calibration capability guards, final examples,
  version/release preparation, and maintenance completion.
- Application firmware owns warm-up, trigger readiness, freshness, cadence,
  retry policy, cached samples, schemas, storage, Cloud, and operator behavior.
- Co2Control-only E2/EE871 production composition and negative isolation for
  TunnelMonitor and all other products remain downstream firmware work.
