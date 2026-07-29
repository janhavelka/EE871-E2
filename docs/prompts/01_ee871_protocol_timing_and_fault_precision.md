# Prompt 01: EE871 Protocol Timing, Bounds, and Fault Precision

## Role and Working Directory

You are an AI coding agent working from:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Implement this prompt only in:

```text
EE871-E2
```

This is reusable library work. Do not edit `TunnelMonitor-node` and do not add
firmware tasks, queues, pins, GPIO setup, logging, sample caches, or product
policy.

## Read Before Editing

Read completely:

- `AGENTS.md`
- `docs/prompts/README.md`
- `docs/EE871_TUNNELMONITOR_NODE_SUITABILITY_AUDIT_2026-07-28.md`
- `include/EE871/Config.h`
- `include/EE871/CommandTable.h`
- `include/EE871/Status.h`
- `include/EE871/EE871.h`
- `src/EE871.cpp`
- `test/test_basic.cpp`
- `test/support/FakeE2Transport.h`
- `docs/EE871_E2_Protocol_and_Register_Map.md`
- `docs/pdf-extracted-md/E2_interface_specification_v4_1.md`
- `docs/pdf-extracted-md/E2_interface_utilising_AN0105.md`

Record the current branch, commit, `library.json` version, dirty files, and
baseline validation results in the handoff report.

## Objective

Correct the E2 transaction mechanics before adding higher-level APIs:

1. handle legal long completion/stretch behavior for `0x10` and `0x50`;
2. never issue `0x51` before the custom-pointer operation has completed;
3. preserve precise SDA/SCL stuck, timeout, NACK, PEC, and verify failures;
4. centralize and strengthen configuration validation;
5. make long millisecond waits cooperative;
6. publish conservative, configuration-derived public-operation blocking
   bounds for owner-worker admission;
7. add focused fake fault coverage.

Keep the driver synchronous. Do not add a library scheduler or generic async
job engine.

## Mandatory Targeted Refactor

The current implementation duplicates timing validation and uses one ordinary
clock-stretch limit for every phase. Replace only the duplicated mechanics
needed by this prompt with small private helpers.

Use these private concepts, with equivalent private names allowed:

```cpp
enum class ClockWaitClass : uint8_t {
  NORMAL_BIT = 0,
  WRITE_COMPLETION = 1,
  INTERVAL_COMMIT = 2,
};

struct ByteDeadline {
  uint32_t elapsedUs{0};
  uint32_t limitUs{0};
};

enum class WriteEffect : uint8_t {
  NONE = 0,
  NO_EFFECT = 1,
  ACKNOWLEDGED = 2,
  INDETERMINATE = 3,
  VERIFIED = 4,
};

struct WriteProgress {
  bool pecTransferred{false};
  bool requestAcknowledged{false};
  bool stopCompleted{false};
  uint32_t completionElapsedUs{0};
  WriteEffect effect{WriteEffect::NONE};
};

static Status _validateConfig(const Config& input, Config& normalized);
Status _busResetRaw();
Status _setCustomPointerRaw(uint8_t address);
Status _setCustomPointerTracked(uint8_t address);
```

`ClockWaitClass`, `WriteEffect`, and `WriteProgress` remain private in this
prompt. Use the one existing raw/tracked write-command path with a progress
output; do not duplicate a second write implementation. `pecTransferred`
followed by anything other than a definite request NACK is conservatively
`INDETERMINATE`; a completed PEC ACK is `ACKNOWLEDGED`; matching readback is
`VERIFIED`. Do not expose a generic transaction framework in public headers.

Refactor the low-level SCL-high wait so callers explicitly select:

- the ordinary per-bit/per-byte limits;
- the normal write-completion limit;
- the interval-pair commit limit.

Use saturating `uint32_t` arithmetic for microsecond accounting and checked
millisecond-to-microsecond conversion. Never wrap a timeout multiplication.

## Public Configuration Additions

Follow the repository enum-value convention (`CAPS_CASE`).

Add optional task-context long-wait callbacks:

```cpp
using E2DelayMsFn = void (*)(uint32_t ms, void* user);
using E2YieldFn = void (*)(void* user);
```

Add to `Config`:

```cpp
E2DelayMsFn delayMs = nullptr;
E2YieldFn yield = nullptr;
uint8_t longDelaySliceMs = 1;
```

Append these fields after all existing `Config` members; do not reorder or
insert into the middle of the aggregate, so existing positional aggregate
initializers retain their field mapping in this minor release.

Keep the single existing `busUser` pointer. Do not add callback-owning objects
or a second user pointer.

Define public timing constants in `CommandTable.h`:

```cpp
static constexpr uint16_t CLOCK_LOW_MIN_US = 100;
static constexpr uint16_t CLOCK_HIGH_MIN_US = 100;
static constexpr uint32_t CLOCK_PERIOD_MAX_US = 2000;
static constexpr uint32_t BIT_TIMEOUT_MAX_US = 25000;
static constexpr uint32_t BYTE_TIMEOUT_MAX_US = 35000;
static constexpr uint8_t LONG_DELAY_SLICE_MAX_MS = 50;
```

Keep the existing finite 5,000 ms accepted maxima for backward compatibility,
but normalize unsafe values below the protocol-derived completion windows:

```cpp
static constexpr uint32_t WRITE_DELAY_PROTOCOL_MIN_MS = 150;
static constexpr uint32_t INTERVAL_WRITE_DELAY_PROTOCOL_MIN_MS = 300;
```

The current defaults remain unchanged. Inputs `0..149` normalize to 150 ms;
interval inputs `0..299` normalize to 300 ms. Values through the existing
5,000 ms maxima remain accepted so current `Config` values do not become
invalid in a minor release. The operation-bound API makes any multi-second
choice explicit, and a consuming owner must reject work that does not fit its
deadline. Document 150/300 ms as the recommended normal values.

The normalized active config must satisfy all of:

- all five required E2 callbacks are non-null;
- device address is `0..7`;
- `clockLowUs >= CLOCK_LOW_MIN_US`;
- `clockHighUs >= CLOCK_HIGH_MIN_US`;
- effective bit period
  `clockLowUs + clockHighUs + DATA_SETUP_US` is at most
  `CLOCK_PERIOD_MAX_US`, using widened arithmetic;
- `startHoldUs >= 4`;
- `stopHoldUs >= 4`;
- `1 <= bitTimeoutUs <= BIT_TIMEOUT_MAX_US`;
- `bitTimeoutUs <= byteTimeoutUs <= BYTE_TIMEOUT_MAX_US`;
- `writeDelayMs <= WRITE_DELAY_MAX_MS`, then values below
  `WRITE_DELAY_PROTOCOL_MIN_MS` normalize upward;
- `intervalWriteDelayMs <= INTERVAL_WRITE_DELAY_MAX_MS`, then values below
  `INTERVAL_WRITE_DELAY_PROTOCOL_MIN_MS` normalize upward;
- `longDelaySliceMs == 0` normalizes to `1`;
- `longDelaySliceMs > LONG_DELAY_SLICE_MAX_MS` is `INVALID_CONFIG`;
- `offlineThreshold == 0` normalizes to `1`.

Do not silently clamp any invalid nonzero value other than the two explicitly
documented normalizations.

## Cooperative Long Wait

Replace `sleepMs()` with one bounded helper:

```cpp
static void delayLongMs(const Config& cfg, uint32_t totalMs);
```

Required behavior:

1. Divide the wait into slices no larger than normalized
   `longDelaySliceMs`.
2. Call `cfg.delayMs(slice, cfg.busUser)` when `delayMs` exists.
3. Otherwise call `cfg.delayUs(slice * 1000U, cfg.busUser)`.
4. Call `cfg.yield(cfg.busUser)` after each completed slice when non-null.
5. Use it only for millisecond completion/quiet waits.
6. Never call `yield` from START, STOP, bit, byte, ACK, PEC, or ordinary
   clock-stretch polling.

The helper may cooperatively yield, but each public library API remains
synchronous and blocking until its documented completion.

## START and STOP Fault Semantics

Before generating START:

1. release SDA;
2. release SCL and wait for SCL high;
3. verify SDA is physically high;
4. if SDA remains low, return:

```cpp
Status::Error(Err::BUS_STUCK, "SDA low before START")
```

Do not drive SDA low to create START when the line is already stuck low.

Keep cleanup bounded. A cleanup STOP failure must never erase the primary
failure if the transaction already has a more precise primary failure.
Conversely, when all payload bytes and PEC were acknowledged and only final
completion/STOP fails, preserve that the mutation may have been accepted for
Prompt 04's effect diagnostics.

Ordinary reads and non-effectful writes use ordinary 25 ms per-bit and 35 ms
per-byte limits. Do not globally raise those values to 150/300 ms.

## Correct `0x10` / `0x50` Completion

> **Superseded timing detail:** Prompt 04A supersedes only the accounting in
> steps 2-4 below. Final-ACK and STOP SCL-low polling share one cumulative
> sensor-completion allowance, while deterministic master ACK/STOP waveform
> delays are bounded separately and do not reduce the legal 150/300 ms
> device-held-low duration. After STOP, wait only the allowance remainder.
> Use Prompt 04A for implementation and exact-boundary acceptance criteria.

The write-command helper must accept an explicit completion class or completion
timeout. Required mappings:

- normal custom byte write (`0x10`) uses `writeDelayMs`;
- custom-pointer write (`0x50`) uses `writeDelayMs`;
- the committing high-byte write for global interval `0xC6/0xC7` uses
  `intervalWriteDelayMs`;
- the first interval byte is an `INTERVAL_STAGE`: it uses ordinary bounded
  frame completion, does not wait 150 ms as a standalone flash commit, and
  does not pretend the pair is committed. Send the committing high byte within
  the same public interval procedure, then apply the 300 ms pair-completion
  bound.

Permit the selected write-specific deadline only in the phase where the
device may legally perform the long internal operation. Do not allow every
ordinary data bit in a write to stretch for 150/300 ms.

Use one total completion budget, not a long stretch plus a second full delay:

1. start the selected 150/300/configured completion budget when the complete
   PEC has been transferred;
2. only the final PEC ACK and the final STOP SCL-high wait may use the
   write-specific stretch limit; only SCL-low polling increments consume the
   one cumulative sensor-completion allowance;
3. account deterministic DATA setup, clock-high/low, and STOP setup/hold as a
   separate, configuration-bounded master protocol tail, and record completion
   allowance consumption in `WriteProgress`;
4. after STOP, cooperatively wait only the remaining completion budget;
5. if the eligible phases already consume the budget, do not add another full
   wait;
6. verification or a dependent transaction begins only after that one total
   budget is satisfied.

The core has no monotonic clock callback, so this is deliberately
driver-accounted elapsed, not a measurement of callback runtime or scheduler
oversleep. Transport delay callbacks must wait at least the requested duration
and remain bounded as already required. Oversleep is safe and may make the
operation finish later; the public timing bound applies only while callbacks
honor their documented bounds.

A dependent transaction must never begin early. This intentionally avoids an
ACK-poll/next-START probe during the completion window.

Apply this to every pointer path:

- public `setCustomPointer()`;
- one-byte `customRead()`;
- block `customRead()`;
- startup feature discovery;
- error-code reads;
- persistent resync;
- any private read helper that sets `0x50` and follows with `0x51`.

For block reads, set the pointer once, wait once, then use auto-increment reads.
Do not re-set the pointer before every byte.

Do not implement an ACK-poll loop unless the vendor documents a bounded,
unambiguous ACK-poll procedure. The simple safe implementation is a bounded
cooperative completion wait plus the existing verified readback.

## Verification Error

Append this value at the end of `Err` without renumbering any existing value:

```cpp
VERIFY_MISMATCH = 15
```

When write transport and completion succeed but readback differs, return:

```cpp
Status::Error(
    Err::VERIFY_MISMATCH,
    "Write verification mismatch",
    actualReadback);
```

Do not use `E2_ERROR` for a value mismatch. A mismatch is a completed transport
with failed verification.

## Conservative Blocking-Bound API

Add a cache-only/public-input-only timing query. It must perform no E2 I/O:

```cpp
enum class OperationKind : uint8_t {
  CONTROL_READ = 0,
  CUSTOM_POINTER_WRITE = 1,
  CUSTOM_BYTE_READ = 2,
  CUSTOM_BLOCK_READ = 3,
  CUSTOM_BYTE_WRITE_VERIFY = 4,
  INTERVAL_WRITE_VERIFY = 5,
  PART_NAME_WRITE_VERIFY = 6,
  RAW_CO2_READ = 7,
  BUS_RESET = 8,
};

struct OperationTimingBound {
  OperationKind kind{OperationKind::CONTROL_READ};
  uint16_t elementCount{1};
  uint32_t maxBlockingMs{0};
};

Status operationTimingBound(
    OperationKind kind,
    uint16_t elementCount,
    OperationTimingBound& out) const;

static Status operationTimingBound(
    const Config& config,
    OperationKind kind,
    uint16_t elementCount,
    OperationTimingBound& out);
```

Requirements:

- The static overload validates/normalizes the supplied config and is pure and
  bus-silent. It exists so an external owner can admit the initial `begin()`
  call before a driver session exists.
- The instance overload uses the active normalized config; before `begin()` it
  returns `NOT_INITIALIZED`.
- Neither overload touches E2 lines.
- `elementCount` must be `1` for fixed-size operations.
- For `CUSTOM_BLOCK_READ`, `elementCount` is `1..256`.
- Use conservative formulas based on configured byte timeout, START/STOP
  holds, write completion waits, and number of E2 transactions.
- `RAW_CO2_READ` includes the low-byte and high-byte control reads.
- `BUS_RESET` includes all reset pulses, line waits, and STOP generation.
- Prompts 02 and 03 append lifecycle and checked-sample operation kinds after
  those procedures exist; do not reserve undocumented numeric gaps.
- Calculate in `uint64_t`, convert microseconds to milliseconds with ceiling
  division, and return `OUT_OF_RANGE` without publishing a wrapped result if a
  future formula ever exceeds `UINT32_MAX`.
- This is an admission bound, not a promise that a callback that violates its
  callback contract can be preempted.

Keep this enum device/library-oriented. Do not include TunnelMonitor command
names or RTOS concepts.

Document each formula in:

```text
docs/EE871_E2_OPERATION_TIMING_BOUNDS.md
```

Both overloads use one shared private calculation path; do not duplicate magic
formulas in multiple functions or tests.

## Health and Side-Effect Rules

- Configuration and parameter errors do not update health.
- START detecting SDA stuck, normal clock-stretch timeout, NACK, PEC mismatch,
  and cleanup/STOP transport failures update health only when called through a
  tracked path.
- Diagnostic raw paths remain health-neutral.
- `VERIFY_MISMATCH` is not a transport failure and must not increment transport
  health when all E2 transfers succeeded.
- Do not implement Prompt 04's full persistent-field uncertainty mask here,
  but retain enough internal accepted/indeterminate evidence for Prompt 04 to
  consume without reworking low-level transport again.

## Required Native Fake Features

Extend the existing fake minimally. Add phase-aware controls instead of
sleep-based tests:

```cpp
enum class StretchPhase : uint8_t {
  NONE = 0,
  DATA_BIT,
  ACK_BIT,
  FINAL_ACK,
  STOP,
  NEXT_START,
};
```

The fake must be able to:

- hold SCL low for an exact simulated duration in a selected phase;
- hold SDA low before START;
- count line writes, line reads, transactions, long-delay slices, and yields;
- record transaction main command and custom-memory address;
- return a wrong readback value;
- expose whether `0x51` began before the pointer-completion wait ended.

Equivalent deterministic fake controls are acceptable. Do not add wall-clock
sleeps to native tests.

## Required Tests

Add native tests for at least:

1. Default config remains valid.
2. A 2,000 us effective period (500 Hz) is accepted.
3. `clockLowUs=100`, `clockHighUs=100`, plus 10 us setup is accepted as the
   fastest realizable configuration (about 4,762 Hz and therefore below the
   5,000 Hz ceiling).
4. A slower-than-500-Hz effective period is rejected, and either high/low phase
   below 100 us is rejected before it could exceed 5,000 Hz.
5. `bitTimeoutUs=0`, `>25000`, `byteTimeoutUs<bitTimeoutUs`, and
   `byteTimeoutUs>35000` are rejected.
6. write delays `0` and `149` normalize to 150 ms; interval delays `0` and
   `299` normalize to 300 ms.
7. 150/300 defaults and the existing 5,000 ms maxima remain accepted;
   values above the existing maxima are rejected.
8. `longDelaySliceMs=0` normalizes to one; 50 is accepted; 51 is rejected.
9. SDA low before START returns `BUS_STUCK` without generating a false START.
10. SCL stuck uses `BUS_STUCK` during idle/reset and `TIMEOUT` during a bounded
   in-transaction stretch as documented.
11. one bit stretching exactly 25 ms succeeds when total byte elapsed stays
    within 35 ms; distributed stretch totaling exactly 35 ms succeeds; one
    poll step beyond either bound fails.
12. Stretch exceeding an ordinary limit returns `TIMEOUT`.
13. a legal near-150-ms `0x10` final-PEC-ACK stretch succeeds and only the
    remaining completion budget is delayed.
14. `0x10` completion above its configured bound returns `TIMEOUT`.
15. a legal near-150-ms `0x50` STOP stretch succeeds and only the remaining
    completion budget is delayed.
16. `0x51` is not issued until pointer completion/wait ends.
17. Startup feature reads honor the same pointer completion rule.
18. A legal near-300-ms interval commit succeeds.
19. An interval commit above the bound fails without pretending verification
    succeeded.
20. Block custom read sets the pointer once and auto-increments.
21. Wrong readback returns `VERIFY_MISMATCH` and leaves transport failure
    counters unchanged.
22. Long waits are sliced through `delayMs`, with one yield after every slice.
23. Fallback long waits use `delayUs` when `delayMs` is null.
24. Bit-level timing never calls `yield`.
25. Every `OperationKind` timing formula has an exact boundary test.
26. both `operationTimingBound()` overloads are bus-silent and the static
    overload validates config without mutating driver state.
27. Invalid timing-bound element counts are rejected without bus I/O.
28. the largest valid config/count calculation is exact and never wraps.

Keep all existing protocol/PEC/unsupported-MV tests passing.

## Documentation

Update:

- `README.md`;
- `CHANGELOG.md` under `Unreleased`;
- `docs/EE871_E2_Protocol_and_Register_Map.md`;
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`;
- Doxygen for every new public type/field;
- the new timing-bound document.

Document:

- E2 is not hardware I2C;
- normal and write-completion stretch limits are deliberately separate;
- pointer writes are completion-ordered before custom reads;
- long callbacks are task-context only and not ISR-safe;
- the timing query is conservative and callback-contract dependent;
- multi-byte convenience helpers can have much larger bounds than one
  transaction;
- persistent writes remain explicit maintenance operations.

Do not bump the release version in this prompt. Leave changes in `Unreleased`
for Prompt 04.

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

If `idf.py` is available, also build the native ESP-IDF example. If it is not
available, report that fact.

## Handoff Report

Create:

```text
docs/reports/ee871_prompt_01_protocol_timing_handoff_YYYYMMDD.md
```

Include:

- baseline branch/commit/version and dirty files;
- exact private refactor performed;
- timing formula table;
- files changed;
- command results;
- test counts;
- intentional deferrals to Prompts 02-04;
- HIL explicitly not run unless real evidence was captured.

## Acceptance Criteria

- `0x10` and `0x50` use correct bounded completion handling.
- No `0x51` begins before pointer completion.
- Ordinary byte timing was not globally relaxed to write timing.
- SDA-low-before-START is detected precisely.
- verify mismatch is distinct from transport failure.
- configuration rejects unsafe timing and delay values.
- long waits are cooperative without yielding in bit timing.
- timing bounds are queryable without bus I/O and covered at boundaries.
- no task, queue, pin, framework, or product policy entered the library.
- all required software validation passes, or each failure is reported with
  evidence and no false completion claim.
