# Prompt 03: EE871 Checked CO2 Sample Procedures

## Role and Scope

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `EE871-E2`. Prompts 01 and 02 must already be complete with
passing handoffs.

Add reusable EE871 sample semantics only. Do not add warm-up timestamps,
freshness, sample cadence, retry policy, background polling, cached samples,
firmware schemas, or a task/job engine.

## Read First

Read:

- `AGENTS.md`;
- `docs/prompts/README.md`;
- Prompts 01 and 02 and their handoffs;
- the checked-sample and measurement-policy sections of
  `docs/EE871_TUNNELMONITOR_NODE_SUITABILITY_AUDIT_2026-07-28.md`;
- all public headers and `src/EE871.cpp`;
- current fake/native tests;
- the status/measurement portions of
  `docs/EE871_E2_Protocol_and_Register_Map.md`;
- `docs/pdf-extracted-md/EE871_E2_CO2_interface_AN1611-1.md`.

Record baseline branch, commit, version, dirty state, and tests.

## Objective

Keep existing raw MV3/MV4 access source-compatible, and add two explicit
checked procedures that:

1. read the requested measured value first;
2. read status second to validate that value and trigger the next measurement
   according to device semantics;
3. optionally read the detailed CO2 error code;
4. distinguish sensor-domain failure from E2 transport failure;
5. enforce the documented library-level supported ppm range;
6. retain per-step evidence for diagnostics.

## Public Types

Add:

```cpp
enum class Co2ValueKind : uint8_t {
  FAST = 0,
  AVERAGE = 1,
};

enum class Co2SensorError : uint8_t {
  NONE = 0,
  SUPPLY_VOLTAGE_LOW = 1,
  SENSOR_COUNTS_LOW = 200,
  SENSOR_COUNTS_HIGH = 201,
  SUPPLY_VOLTAGE_BREAKDOWN_AT_PEAK = 202,
  UNKNOWN = 255,
};

struct Co2ReadResult {
  Co2ValueKind kind{Co2ValueKind::AVERAGE};

  uint16_t ppm{0};
  bool ppmValid{false};

  uint8_t statusByte{0};
  bool statusValid{false};
  bool co2Error{false};

  uint8_t errorCode{0};
  bool errorCodeValid{false};
  Co2SensorError sensorError{Co2SensorError::NONE};

  bool valueReadAttempted{false};
  bool statusReadAttempted{false};
  bool errorCodeReadAttempted{false};

  Status valueReadStatus{Status::Ok()};
  Status statusReadStatus{Status::Ok()};
  Status errorCodeReadStatus{Status::Ok()};
};
```

The attempted flags are mandatory. An unattempted step must not be
indistinguishable from a successful step merely because its default `Status`
is OK.

Append:

```cpp
CO2_SENSOR_ERROR = 17
```

to `Err`, preserving Prompt 01's `VERIFY_MISMATCH=15` and Prompt 02's
`OFFLINE=16`.

Add:

```cpp
static constexpr uint16_t CO2_PPM_MIN = 0;
static constexpr uint16_t CO2_PPM_MAX = 50000;
```

Reuse and verify the existing
`cmd::STATUS_CO2_ERROR_MASK = 0x08`; do not introduce a duplicate
definition.

Use the range only in checked sample procedures. Existing raw value APIs keep
returning the raw `uint16_t` without status/range policy.

## Public Methods

Add:

```cpp
Status readCo2AverageSample(Co2ReadResult& out);
Status readCo2FastSample(Co2ReadResult& out);
```

Keep these existing methods unchanged and source-compatible:

```cpp
Status readCo2Average(uint16_t& ppm);
Status readCo2Fast(uint16_t& ppm);
Status readStatus(uint8_t& status);
Status readErrorCode(uint8_t& code);
```

Do not silently change the existing methods to checked procedures. In Doxygen,
call them raw MV4/MV3 value reads.

Use one private implementation:

```cpp
Status _readCo2Sample(Co2ValueKind kind, Co2ReadResult& out);
```

Do not duplicate average and fast control flow.

## Exact Procedure

At entry:

1. replace `out` with a known default;
2. set `out.kind`;

Then:

1. set `valueReadAttempted=true`;
2. call:
   - `readCo2Average()` for `AVERAGE`/MV4; or
   - `readCo2Fast()` for `FAST`/MV3;
3. copy the returned status to `valueReadStatus`;
4. on value transport failure, return it immediately:
   - keep `ppmValid=false`;
   - do not read status;
5. store the raw ppm in `out.ppm`, but do not mark it valid yet;
6. set `statusReadAttempted=true`;
7. call `readStatus()` after the value;
8. copy the returned status to `statusReadStatus`;
9. on status transport failure, return it and keep `ppmValid=false`;
10. store status and set `statusValid=true`;
11. set `co2Error` from `STATUS_CO2_ERROR_MASK`.

If `co2Error` is true:

- keep `ppmValid=false`;
- if cached capabilities advertise `hasErrorCode()`:
  1. set `errorCodeReadAttempted=true`;
  2. call `readErrorCode()`;
  3. copy its status;
  4. if its transport/protocol read fails, return that precise status;
  5. otherwise store the code, set `errorCodeValid=true`, and map it through a
     pure helper to `Co2SensorError`;
- if error-code support is not advertised, leave the attempt flag false and
  error-code validity false;
- return:

```cpp
Status::Error(
    Err::CO2_SENSOR_ERROR,
    "CO2 sensor status error",
    errorCodeValid ? errorCode : statusByte);
```

Every code other than `1`, `200`, `201`, and `202`, including zero while the
status error bit is set, maps to `Co2SensorError::UNKNOWN` while retaining the
raw `errorCode`. `NONE` is used only when status is clean.

If status is clean:

- leave `sensorError=NONE`;
- reject `ppm > CO2_PPM_MAX` with:

```cpp
Status::Error(
    Err::OUT_OF_RANGE,
    "CO2 ppm out of range",
    ppm);
```

- retain the raw ppm for diagnostics and keep `ppmValid=false` on range
  failure;
- otherwise set `ppmValid=true` and return OK.

`CO2_PPM_MIN` is documented for symmetry and future type changes; the current
unsigned raw type cannot be below it.

## Status Side Effect

Document on:

- `readStatus()`;
- both checked sample methods;
- examples in Prompt 04.

The documentation must say that status applies to the last measured values,
can start/trigger a new measurement, and can reset interval timing under the
documented conditions. This is why the checked helper reads MV3/MV4 first and
status second.

Do not read status merely to make an existing raw value helper appear safer.

## Error and Health Semantics

- Transport/protocol failures from value, status, or error-code reads keep
  their original `NACK`, `TIMEOUT`, `BUS_STUCK`, `PEC_MISMATCH`, or other
  precise status and update health through existing tracked transfers.
- `CO2_SENSOR_ERROR` is a sensor-domain terminal result. Do not call
  `_updateHealth()` for it and do not increment transport failures.
- checked `OUT_OF_RANGE` is a sensor-domain validation result. Do not call
  `_updateHealth()` for it and do not increment transport failures.
- Successful transfers already update transport success according to existing
  wrapper rules; do not add separate fake "sample success" counts to library
  transport health.
- Returning a sensor-domain error must not move a transport-healthy driver to
  `DEGRADED` or `OFFLINE`.

## Capability Helpers

Prompt 02 cached custom byte `0x03`. Add:

```cpp
bool hasCo2OffsetGain() const;
bool hasCo2AdjustmentPoints() const;
```

Both are cache-only:

- offset/gain checks `FEATURE_CO2_CUSTOM_ADJUSTMENT`;
- points checks `FEATURE_CO2_ADJUSTMENT_POINT`.

Do not yet change persistent calibration writes; Prompt 04 owns those guards
and uncertainty semantics.

## Timing-Bound Extension

Append to `OperationKind`:

```cpp
CHECKED_CO2_AVERAGE = 13
CHECKED_CO2_FAST = 14
```

Their conservative bound includes:

- two value control reads;
- one status read;
- worst-case optional error-code custom pointer plus read;
- all pointer completion timing from Prompt 01.

The query remains cache-only. Test actual fake elapsed time at timing boundaries
against the advertised bound.

## Native Fake Additions

Add small helpers:

```cpp
void setCo2FastPpm(uint16_t ppm);
void setCo2AveragePpm(uint16_t ppm);
void setStatusByte(uint8_t status);
void setErrorCode(uint8_t code);
uint32_t controlReadCount(uint8_t mainCommandNibble) const;
```

Record ordered transaction events so tests can prove value-before-status and
that no later step ran after an earlier failure.

## Required Tests

Add:

1. default `Co2ReadResult` has all validity/attempt flags false;
2. average success reads MV4 low/high before status;
3. fast success reads MV3 low/high before status;
4. both success results set exact raw/status/validity/attempt fields;
5. value failure returns the value status and performs no status read;
6. status failure retains raw ppm but leaves it invalid;
7. status error with code 1 maps to `SUPPLY_VOLTAGE_LOW`;
8. code 200 maps to `SENSOR_COUNTS_LOW`;
9. code 201 maps to `SENSOR_COUNTS_HIGH`;
10. code 202 maps to `SUPPLY_VOLTAGE_BREAKDOWN_AT_PEAK`;
11. unknown code, including zero while the status bit is set, maps to `UNKNOWN`
    and retains the raw code;
12. status error with error-code capability reads `0xC1`;
13. status error without capability does not attempt `0xC1`;
14. error-code transfer failure returns its precise transfer status;
15. `50000` ppm with clean status succeeds;
16. `50001` ppm returns `OUT_OF_RANGE` and retains raw value invalid;
17. raw average/fast APIs still return `50001` without checked-range policy;
18. `CO2_SENSOR_ERROR` with all bus transfers successful does not increment
    transport failures or degrade state;
19. checked range error does not increment transport failures or degrade state;
20. an error-code transport failure does update transport health;
21. checked methods return `OFFLINE` with zero line I/O while latched offline;
22. capability helpers are correct and bus-silent;
23. checked timing bounds are conservative and bus-silent;
24. existing raw APIs remain source-compatible in a public-header compile test.

Use deterministic fail-at-transfer injection for every step.

## Documentation

Update:

- public Doxygen;
- `README.md`;
- `CHANGELOG.md` under `Unreleased`;
- `docs/EE871_E2_Protocol_and_Register_Map.md`;
- `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md`;
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`.

Clearly distinguish:

- MV3 fast raw value;
- MV4 average raw value;
- checked fast sample;
- checked average sample;
- explicit status read and its side effect;
- sensor-domain versus transport-domain failure;
- application-owned warm-up, freshness, trigger readiness, and cadence.

Do not change examples or version metadata here; Prompt 04 owns the final
example/release update.

## Validation

Run:

```powershell
python tools/check_core_timing_guard.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
git diff --check
```

Build the pure ESP-IDF example if `idf.py` is available.

## Handoff

Create:

```text
docs/reports/ee871_prompt_03_checked_samples_handoff_YYYYMMDD.md
```

Include the exact step/return matrix, ordering evidence, health-counter
evidence, timing-bound changes, files, and validation results.

## Acceptance Criteria

- existing raw APIs are unchanged;
- checked average and fast APIs share one implementation;
- value is always read before status;
- no status read follows value failure;
- error-code read is capability-gated;
- all documented codes remain distinguishable;
- raw ppm remains available when checked validity fails;
- sensor/range errors do not falsify E2 transport health;
- status-trigger side effects are explicit;
- no warm-up/freshness/cadence or firmware concern entered the library.
