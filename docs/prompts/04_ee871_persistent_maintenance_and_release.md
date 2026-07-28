# Prompt 04: EE871 Persistent Maintenance Coherence and Release

## Role and Scope

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `EE871-E2`. Prompts 01-03 and their handoffs must be complete and
passing.

This prompt closes the remaining general-purpose library gaps and creates the
release-ready source suitable for downstream firmware evaluation. Do not edit
`TunnelMonitor-node`, create an owner task, choose pins, or add application
policy.

## Read First

Read completely:

- `AGENTS.md`;
- `docs/prompts/README.md`;
- Prompts 01-03 and their handoffs;
- the persistent, calibration, validation, and production-gate sections of
  `docs/EE871_TUNNELMONITOR_NODE_SUITABILITY_AUDIT_2026-07-28.md`;
- all public headers and `src/EE871.cpp`;
- fake/native tests;
- both framework examples and their contract checks;
- `library.json`, `CHANGELOG.md`, `README.md`;
- version-generation tooling;
- current release notes and hardware validation matrix.

Record branch, commit, current version, dirty state, and full baseline.

## Objective

Complete mutation truth without adding a second competing state system:

1. route every effectful custom-memory API through one common mutation frame,
   completion, and effect-classification path;
2. retain whether the device definitely rejected, acknowledged, may have
   accepted, verified, or was later resynchronized after a mutation;
3. extend the existing sticky persistent diagnostic to single-byte and
   maintenance operations;
4. block further mutations while state is unresolved;
5. add explicit bounded resynchronization;
6. guard calibration APIs from cached capability data;
7. finish examples, docs, fault tests, versioning, and release evidence.

Normal sampling must never perform a persistent or maintenance write.

## Public Mutation Types

Add:

```cpp
enum class MutationTarget : uint8_t {
  NONE = 0,
  RAW_CUSTOM_BYTE = 1,
  PART_NAME = 2,
  BUS_ADDRESS = 3,
  GLOBAL_INTERVAL = 4,
  CO2_INTERVAL_FACTOR = 5,
  CO2_FILTER = 6,
  OPERATING_MODE = 7,
  AUTO_ADJUST = 8,
  CO2_OFFSET = 9,
  CO2_GAIN = 10,
};

enum class MutationEffect : uint8_t {
  NONE = 0,
  NO_EFFECT = 1,
  ACKNOWLEDGED = 2,
  INDETERMINATE = 3,
  VERIFIED = 4,
  RESYNCHRONIZED = 5,
  OPERATOR_ACKNOWLEDGED = 6,
};

struct MutationDiagnostic {
  bool unresolved{false};
  MutationTarget target{MutationTarget::NONE};
  MutationEffect effect{MutationEffect::NONE};
  uint8_t firstAddress{0};
  uint8_t lastAddress{0};
  uint16_t elementsRequested{0};
  uint16_t elementsAcknowledged{0};
  uint16_t elementsObserved{0};
  uint16_t elementsMatched{0};
  uint8_t attemptedValue{0};
  uint8_t preObservedValue{0};
  bool preObservedValueValid{false};
  uint8_t observedValue{0};
  bool observedValueValid{false};
  Status cause{Status::Ok()};
};
```

This is one general EE871 mutation diagnostic, not a firmware job result and
not a generic framework.

Append:

```cpp
PERSISTENT_STATE_UNCERTAIN = 18
```

to `Err`.

Add cache-only:

```cpp
MutationDiagnostic mutationDiagnostic() const;
```

Append `MutationDiagnostic mutation{}` to `SettingsSnapshot`.

Retain:

```cpp
bool persistentConfigDirty() const;
Status persistentConfigDirtyError() const;
```

for source compatibility. They mirror the unresolved state and cause:

```cpp
persistentConfigDirty() == mutation.unresolved
persistentConfigDirtyError() ==
    (mutation.unresolved ? mutation.cause : Status::Ok())
```

The diagnostic may retain historical observation/mismatch evidence after
resynchronization, but the legacy error must be OK once unresolved is false.
Update Doxygen to explain that the legacy name also covers an unresolved
explicit maintenance action such as auto-adjust.

## One Mutation Path

Refactor all custom-memory mutations through one internal frame/completion
helper and one effect classifier, with equivalent names allowed:

```cpp
Status _writeCustomByteEffectful(
    uint8_t address,
    uint8_t value,
    MutationTarget target,
    MutationProgress& progress);

void _classifyMutationEffect(
    const MutationProgress& progress,
    MutationDiagnostic& diagnostic);
```

`MutationProgress` is private and consumes Prompt 01's low-level write
progress. The common path owns mutation admission, write-frame completion, and
effect classification. Verification is target-specific:

- immediate equality readback for ordinary persistent bytes;
- deferred-pair verification for the global interval: write `0xC6`, write
  `0xC7`, wait for the one pair-commit budget, then read and compare both;
- action-status observation for auto-adjust at `0xD9`;
- explicit address-change reconciliation described below.

Do not force all targets through a byte-at-a-time "write then immediately read"
algorithm. In particular, an immediate read after `0xC6` is not valid
verification of the deferred `0xC6/0xC7` commit. Do not add dirty/effect
handling independently to every public wrapper.

Retain the unresolved requested intent in one fixed private record so a later
resync can truthfully decide match versus coherent mismatch:

```cpp
struct MutationIntent {
  MutationTarget target{MutationTarget::NONE};
  uint8_t firstAddress{0};
  uint8_t elementCount{0};
  uint8_t values[16]{};
};
```

Sixteen bytes covers the longest existing target (`PART_NAME`). Validate the
count; do not add heap allocation or a generic mutation journal. Clear the
private intent only when uncertainty is resolved or when replacing a fully
settled diagnostic with a new admitted mutation.

All these APIs must use the common path:

- `customWrite()`;
- `writePartName()`;
- `writeBusAddress()`;
- `writeMeasurementInterval()`;
- `writeCo2IntervalFactor()`;
- `writeCo2Filter()`;
- `writeOperatingMode()`;
- `startAutoAdjust()`;
- `writeCo2Offset()`;
- `writeCo2Gain()`.

Multi-byte APIs coordinate common effectful byte transfers but own one
operation-level diagnostic with exact requested, acknowledged, observed, and
matched element counts.

### Raw custom-write protected-address routing

`customWrite()` must not bypass typed safety rules. Before any I/O, route known
special addresses through one private dispatch table:

| Address | `customWrite()` behavior |
| --- | --- |
| `0xC0` | call the bus-address algorithm with the supplied `0..7` value |
| `0xC6`, `0xC7` | return `NOT_SUPPORTED` and require `writeMeasurementInterval()`; one byte cannot safely express the pair commit |
| `0xCB` | call the typed CO2 interval-factor algorithm after exact `int8_t` interpretation/validation |
| `0xD3` | call the typed filter algorithm |
| `0xD8` | call the typed operating-mode algorithm |
| `0xD9` | value `1` calls the auto-adjust algorithm; every other value returns `NOT_SUPPORTED` |
| `0x58..0x5B` | return `NOT_SUPPORTED` and require the paired offset/gain API |

Also reject documented read-only identity, capability, firmware, serial-number,
and error-code addresses with `NOT_SUPPORTED` before line I/O. Keep one
constexpr address-classification helper and tests; do not duplicate a switch
in public wrappers.

All remaining writable custom bytes use `RAW_CUSTOM_BYTE` with immediate
equality verification. Typed dispatch targets call their private algorithm
directly, not public `customWrite()`, so no recursion occurs.

## Exact Effect Classification

Use:

- `NONE`: validation/offline/uncertain-state guard rejected before an E2 frame;
- `NO_EFFECT`: a definite NACK or failure before a complete effectful request
  was transferred;
- `ACKNOWLEDGED`: payload and PEC were ACKed, but later completion or
  verification did not prove the requested final state;
- `INDETERMINATE`: complete PEC was transferred but final ACK/STOP outcome
  cannot prove whether the target accepted it;
- `VERIFIED`: bounded target-specific readback/status evidence matched the
  requested state;
- `RESYNCHRONIZED`: a later full, coherent target readback established actual
  device state but did not prove the original requested state took effect;
- `OPERATOR_ACKNOWLEDGED`: the narrow auto-adjust reconciliation procedure
  below accepted an irreducibly ambiguous historical outcome after a
  successful post-failure observation.

Set `unresolved=true` and preserve the original failure when:

- complete PEC transfer is followed by final-ACK timeout/ambiguity;
- an acknowledged write is followed by STOP failure;
- an acknowledged write exceeds its completion deadline;
- dependent readback fails;
- readback completes but returns a mismatch;
- a later element of a multi-byte write fails after an earlier element was
  acknowledged/verified.

Do not mark unresolved for:

- invalid config/parameter;
- not initialized/offline;
- capability rejection;
- a definite NACK before the request was accepted;
- an ordinary read failure unrelated to a mutation;
- a mutation that completed and fully verified.

Preserve the first unresolved cause until explicit resync. Do not clear it
because an unrelated read, probe, begin, or health success occurred.

Prompt 01's `VERIFY_MISMATCH` remains the return for completed readback with a
different value. Preserve the actual value in status detail and the mutation
diagnostic.

## Mutation Admission Guard

When `mutation.unresolved` is true:

- reject every further effectful public API before bus I/O with:

```cpp
Status::Error(
    Err::PERSISTENT_STATE_UNCERTAIN,
    "Persistent state unresolved; call resyncPersistentConfig()");
```

- allow normal measurement reads, diagnostics, `probe()`, `recover()`,
  `checkBusIdle()`, `busReset()`, cache getters, and
  `resyncPersistentConfig()`;
- do not automatically replay the intended mutation;
- do not hide the state during normal recovery.

This guarantees at most one unresolved intent and avoids a heap, list, or
fixed array of guessed writes.

## Resynchronization

Refactor `resyncPersistentConfig()` around the `MutationTarget`.

When no unresolved mutation exists, it may retain its current full
configuration-coherence read behavior, but it must consult cached capability
flags and skip unsupported targets. It must never fail merely because an
optional unsupported setting was not read.

When unresolved:

- `RAW_CUSTOM_BYTE`: read `firstAddress`;
- `PART_NAME`: read all 16 bytes;
- `BUS_ADDRESS`: use the explicit candidate-address reconciliation described
  below;
- `GLOBAL_INTERVAL`: read and range-check `0xC6/0xC7`;
- `CO2_INTERVAL_FACTOR`: read `0xCB`;
- `CO2_FILTER`: read `0xD3`;
- `OPERATING_MODE`: read `0xD8` and validate reserved bits/support;
- `AUTO_ADJUST`: call the read-only status procedure for `0xD9`;
- `CO2_OFFSET`: read both `0x58/0x59`;
- `CO2_GAIN`: read both `0x5A/0x5B`.

Use the common pointer helper and all Prompt 01 timing rules.

On full target readback success:

- retain the observed byte and validity in the diagnostic for inspection;
- if every observed element matches the originally requested value, set
  `VERIFIED`;
- otherwise set `RESYNCHRONIZED`: actual device state is now coherent and
  inspectable, but the original intent was not verified;
- clear `unresolved` for ordinary configuration targets;
- clear legacy dirty/error fields.

On failure:

- preserve the original first cause;
- return the new precise resync failure;
- keep unresolved true.

Document that the application must compare configuration with its own intended
baseline after resync. Do not invent an application baseline inside the
library.

`recover()` does not clear mutation uncertainty unless it explicitly completes
the same target-specific readback. Prefer leaving this to the named resync API.

Auto-adjust is the exception because a later clear `0xD9` value cannot
distinguish "completed" from "never started":

- a successful post-failure `0xD9` observation that reports running proves the
  action started; set `VERIFIED` and clear unresolved;
- a successful observation that reports not running records the observation
  but keeps unresolved and returns `PERSISTENT_STATE_UNCERTAIN`;
- expose only this narrow cache-only reconciliation:

```cpp
Status acknowledgeAutoAdjustUncertainty();
```

It succeeds only when the current unresolved target is `AUTO_ADJUST` and a
successful post-failure not-running observation is recorded. It sets
`OPERATOR_ACKNOWLEDGED` and clears unresolved. It performs no I/O and must not
be generalized into a "clear dirty" API.

## Bus Address Semantics

`writeBusAddress()` must:

- validate `0..7`;
- require address-configuration capability;
- write custom byte `0xC0` using the current session address;
- not assume, without authoritative device evidence, whether activation is
  immediate or requires a power cycle;
- not attempt readback through the old address after an acknowledged write;
- retain the candidate address and mark the mutation unresolved even after a
  clean acknowledgement, because in-session verification is not safely
  defined;
- return `PERSISTENT_STATE_UNCERTAIN` after an acknowledged address request,
  with the effect recorded as `ACKNOWLEDGED`;
- document that the application explicitly calls `end()`, follows the
  vendor/product power procedure if required, updates `Config::deviceAddress`
  to the candidate, and calls `begin()` at that address;
- complete target-specific resync by reading `0xC0` through the active
  explicitly configured candidate session and comparing it with the requested
  address;
- if the candidate `begin()` fails, allow the application to retry only an
  explicitly authorized known address; the driver never selects one;
- never guess that the sensor has already moved to either address.

Do not add hidden probing of all eight addresses or silently rewrite the active
session address.

## Capability Guards

Use Prompt 02's valid capability snapshot:

- `readCo2Offset()` and `writeCo2Offset()` require
  `hasCo2OffsetGain()`;
- `readCo2Gain()` and `writeCo2Gain()` require
  `hasCo2OffsetGain()`;
- future adjustment-point APIs require `hasCo2AdjustmentPoints()`, but do not
  add those APIs in this prompt;
- existing part-name/address/interval/filter/mode/auto-adjust guards remain
  precise and cache-only.

Unsupported reads/writes return `NOT_SUPPORTED` before bus I/O.

Do not infer support merely because an address happens to return a byte.

## Auto-Adjust Safety

Keep `startAutoAdjust()` explicit and document:

- it is a maintenance action;
- before writing, read `0xD9`; if already running, return `BUSY` with no write,
  and if the pre-read fails return its precise status with no mutation;
- writing one starts an operation that cannot be cancelled through E2;
- measured values may remain held during adjustment;
- with external serialization and a successful pre-observation of not-running,
  an immediate post-write `0xD9` running status verifies this request started
  and returns OK;
- an acknowledged request followed by a clean not-running status is
  irreducibly ambiguous: return `PERSISTENT_STATE_UNCERTAIN`, retain the
  observation, and require the narrow operator acknowledgement if later
  observation still cannot prove history;
- an ambiguous accepted write must not be replayed automatically;
- `resyncPersistentConfig()` can prove the action started only while status
  reports running and the retained pre-write observation proved it was not
  already running;
- a later clear status cannot prove whether a prior adjustment completed or
  never started and requires the explicit operator acknowledgement above.

Do not expose auto-adjust through the ordinary sample example commands.

## Stopped State

Split session/cache reset from mutation-evidence reset. `end()`, any failed or
repeated `begin()`, stopped-state reset, and a later successful `begin()` must
not clear an unresolved mutation diagnostic. After a new successful begin,
effectful operations remain blocked until target-specific
resync/verification succeeds. Read-only identity/diagnostic work remains
available.

Destroying the driver object or losing power necessarily loses RAM evidence;
document that applications which must survive application restart have to
persist the maintenance workflow state outside the library. Do not add
filesystem/NVS ownership to the library and do not claim an uncertain physical
write was reversed.

## Timing-Bound Completion

Append, without renumbering Prompt 01-03 values:

```cpp
CUSTOM_BLOCK_WRITE_VERIFY = 15
RESYNC_PERSISTENT_CONFIG = 16
AUTO_ADJUST_MAINTENANCE = 17
BUS_ADDRESS_CHANGE = 18
```

Extend the one shared `operationTimingBound()` calculator. Bounds must include
the target-specific algorithms in this prompt, including deferred interval
commit, all part-name elements, auto-adjust pre/post observations, and
address-write completion.

`RESYNC_PERSISTENT_CONFIG` uses `elementCount=1` and publishes one fixed
worst-case bound that covers the complete capability-aware no-target resync of
every supported setting. Every target-specific resync is therefore bounded by
the same conservative value. Do not claim target-exact timing from
`elementCount`, because equal-sized targets have different procedures. Reject
any other element count. Keep target details in `MutationTarget`; do not add
firmware concepts to the timing API.

Make `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md` exhaustive. It must map every
public bus-touching method to an `OperationKind` and formula, including methods
that share a class:

- lifecycle/diagnostic: `begin`, `probe`, `recover`, `busReset`,
  `checkBusIdle`;
- primitive reads/writes: `readControlByte`, `readU16`, `setCustomPointer`,
  both `customRead` overloads, and `customWrite`;
- identity/capability/settings reads;
- raw and checked CO2 reads;
- all typed persistent writes and `startAutoAdjust`;
- `resyncPersistentConfig`.

Add a lightweight source audit such as
`tools/check_public_timing_contract.py` plus a consistent Doxygen marker on
every bus-touching public declaration. The check must fail if a future public
bus operation lacks a timing classification/documentation row. Cache-only
getters, `tick()`, `end()`, and the cache-only operator acknowledgement are
explicitly marked non-blocking/no-I/O rather than assigned a fabricated bus
bound.

Also close the timing/health ownership documentation:

- health success/failure counters count tracked E2 transfers, not samples or
  public API calls; one checked sample can add several transport successes;
- `tick(uint32_t nowMs)` stores only the caller-supplied timestamp and performs
  no hidden scheduling or timebase extension;
- an external owner should call `tick()` immediately before each owned library
  operation when it wants current diagnostic timestamps;
- the core library does not add a 64-bit scheduler, task clock, retry cadence,
  or freshness policy.

## Examples

Keep current raw commands:

- `co2avg`;
- `co2fast`;
- `status`.

Add to both Arduino and native ESP-IDF examples:

- `sampleavg`;
- `samplefast`.

Rules:

- `co2avg` calls raw MV4;
- `co2fast` calls raw MV3;
- `sampleavg` calls `readCo2AverageSample()`;
- `samplefast` calls `readCo2FastSample()`;
- checked output prints step validity and precise sensor/error code;
- help says status/checked samples may trigger the next measurement;
- no example adds arbitrary custom-write or calibration commands.

Example-only adapters map `delayMs`/`yield` appropriately in task context.
Core code remains framework-neutral.

## Native Tests

Extend deterministic fake coverage for every target and every mutation phase.
At minimum test:

1. public enum numeric values and default diagnostic;
2. successful ordinary single-byte write reports verified/not unresolved;
3. validation failure reports no effect and no dirty state;
4. definite pre-acceptance NACK reports no effect and no dirty state;
5. PEC transferred/final ACK timeout reports indeterminate/unresolved;
6. acknowledged/STOP failure reports acknowledged/unresolved;
7. acknowledged/completion timeout reports acknowledged/unresolved;
8. readback transport failure reports acknowledged/unresolved;
9. mismatching readback returns `VERIFY_MISMATCH`, stores actual value, and
   remains unresolved;
10. every listed typed API routes through the common frame/completion/effect
    path and the correct target-specific verification policy;
11. part-name failure at every element retains exact requested, acknowledged,
    observed, and matched counts;
12. interval failure before any acceptance is clean;
13. interval failure after low byte and after committing high byte is
    unresolved with exact count;
14. offset and gain second-byte failures are unresolved;
15. a second mutation is rejected bus-silently while unresolved;
16. ordinary checked/raw reads still work while mutation state is unresolved;
17. successful matching target-specific resync reports `VERIFIED` and clears
    unresolved state;
18. failed resync keeps first cause and returns the resync error;
19. successful coherent nonmatching resync reports `RESYNCHRONIZED`, clears
    unresolved for ordinary targets, and retains actual/mismatch evidence;
20. recover alone does not silently clear uncertainty;
21. bus-address write never reads through or silently changes the old active
    session; only an explicitly configured candidate session can reconcile it;
22. offset/gain support guards are bus-silent;
23. auto-adjust ambiguity is not replayed; running observation verifies it,
    not-running observation remains unresolved, and only the narrow
    cache-only operator acknowledgement clears that case;
24. auto-adjust pre-observation failure or already-running status performs no
    write; a retained not-running pre-observation is required before a later
    running status can verify this request;
25. `end()`, failed begin, and successful re-begin preserve unresolved
    diagnostics until target-specific reconciliation;
26. `getSettings()` and `mutationDiagnostic()` remain bus-silent;
27. fake time for worst-case part-name, interval, custom, checked sample, begin,
    and recover stays within advertised bounds;
28. every bus-touching public API has timeout/NACK or equivalent failure
    coverage;
29. every bus-touching public API is covered by the timing-contract source
    audit;
30. global interval performs no invalid immediate verification between `0xC6`
    and `0xC7`, uses one deferred pair-commit budget, then verifies both;
31. legacy dirty error is OK whenever unresolved is false, even if diagnostic
    history remains;
32. raw custom writes dispatch every protected typed address, reject
    unsafe pair/read-only writes bus-silently, and cannot bypass address,
    interval, calibration, or auto-adjust rules;
33. public-header compile compatibility for existing callers.

Update CLI contract tests for both frameworks.

## Documentation and Release

Update:

- `README.md`;
- `CHANGELOG.md`;
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`;
- `docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md`;
- `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md`;
- Arduino and ESP-IDF example documentation;
- Doxygen for all public additions.

Update the hardware matrix only to mark new physical cases pending unless they
were actually run. Do not convert native fake results into HIL claims.

Versioning:

1. read the current `library.json` version;
2. bump to the next minor version because public backward-compatible APIs and
   error codes were added;
3. regenerate `include/EE871/Version.h` using repository tooling;
4. create a new versioned release-notes document;
5. never rewrite old release notes as the new release;
6. keep explicit enum numeric values stable;
7. do not commit, tag, or push unless the user separately asks.

Therefore this prompt can make the repository release-ready but cannot by
itself create the immutable remote dependency required by Prompt 05. The
authorized release/tag/push step must happen separately, and Prompt 05 must
verify it rather than assume it.

If the starting version is `1.0.0`, the target is `1.1.0` and the notes file is:

```text
docs/EE871_E2_RELEASE_NOTES_1.1.0.md
```

## Validation

Run:

```powershell
python tools/check_core_timing_guard.py
python tools/check_public_timing_contract.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
git diff --check
```

If `idf.py` exists, build the pure ESP-IDF example. Report unavailable tools
and any failing command exactly. Do not claim HIL.

Perform a fresh post-change audit against every P0/P1 library finding in the
2026-07-28 audit. Fix all in-scope misses before exit; do not merely list them.

## Final Handoff

Create:

```text
docs/reports/ee871_prompt_04_release_handoff_YYYYMMDD.md
```

Include:

- baseline and final version;
- files changed;
- targeted refactors and deleted duplication;
- mutation effect/state table;
- exact public API/enum additions;
- test/build command output summary;
- native test count;
- `idf.py` availability;
- audit closure table;
- remaining firmware-owned work;
- HIL status.

## Acceptance Criteria

- all effectful APIs share one frame/completion/effect path and use the correct
  target-specific verification policy;
- accepted/indeterminate single-byte and multi-byte failures remain observable;
- no further mutation occurs while state is unresolved;
- explicit target-specific resync is the normal clearing route, with only the
  narrow documented auto-adjust operator acknowledgement exception;
- bus-address and auto-adjust side effects are not hidden or replayed;
- calibration APIs are capability-gated;
- raw and checked example commands are visibly distinct;
- version metadata and release notes describe the complete change;
- P0 library findings are closed in software;
- no firmware task/pin/schema/cadence policy entered the reusable library;
- no HIL or release claim is invented.
