# Prompt 02: EE871 Lifecycle, Identity, Capabilities, and Explicit Recovery

## Role and Repository

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `EE871-E2`. This prompt depends on Prompt 01. Do not begin until
Prompt 01's handoff exists and its required validation passes.

This remains general library work. Do not edit any consuming firmware and do
not add task, queue, pin, cadence, warm-up, schema, or retry policy.

## Read First

Read completely:

- `AGENTS.md`
- `docs/prompts/README.md`
- `docs/prompts/01_ee871_protocol_timing_and_fault_precision.md`
- the Prompt 01 handoff under `docs/reports/`;
- `docs/EE871_TUNNELMONITOR_NODE_SUITABILITY_AUDIT_2026-07-28.md`;
- all public headers under `include/EE871/`;
- `src/EE871.cpp`;
- `test/test_basic.cpp`;
- `test/support/FakeE2Transport.h`.

Record the new baseline branch, commit, version, dirty files, and validation
results. Preserve Prompt 01 behavior.

## Objective

Implement one coherent lifecycle suitable for both simple applications and a
dedicated firmware owner:

1. a strict default startup that requires a verified EE871;
2. an explicit optional-device startup mode that may retain a configured but
   offline driver only for authoritative `DEVICE_NOT_FOUND`;
3. full group/subgroup/CO2-capability validation;
4. atomic capability caching;
5. normal-operation fast-fail while offline;
6. explicit, bounded recovery as the only route from `OFFLINE` to online;
7. raw diagnostic probing that does not mutate health or live cache.

Keep exactly the existing four `DriverState` values.

## Public Types

Add in a public header before `Config`:

```cpp
enum class BeginPolicy : uint8_t {
  REQUIRE_PRESENT = 0,
  ALLOW_ABSENT = 1,
};
```

Append to `Config`:

```cpp
BeginPolicy beginPolicy = BeginPolicy::REQUIRE_PRESENT;
```

Add:

```cpp
struct DeviceIdentity {
  uint16_t group{0};
  uint8_t subgroup{0};
  uint8_t availableMeasurements{0};
  bool co2Available{false};
  bool valid{false};
};

struct CapabilitySnapshot {
  uint8_t customAdjustmentSupport{0};       // custom byte 0x03
  uint8_t adjustmentPointSupport{0};        // custom byte 0x04
  uint8_t adjustmentTimeGeneralSupport{0};  // custom byte 0x05
  uint8_t adjustmentTimeSupport{0};         // custom byte 0x06
  uint8_t operatingFunctions{0};            // custom byte 0x07
  uint8_t operatingModeSupport{0};           // custom byte 0x08
  uint8_t specialFeatures{0};                // custom byte 0x09
  bool valid{false};
};
```

Add cache-only accessors:

```cpp
DeviceIdentity identity() const;
CapabilitySnapshot capabilities() const;
```

They return copies and perform no E2 I/O.

Append this precise error to `Err` without renumbering existing values:

```cpp
OFFLINE = 16
```

Normal operations blocked by the offline latch return:

```cpp
Status::Error(
    Err::OFFLINE,
    "Driver is offline; call recover()");
```

Do not use generic `BUSY` for this state. `BUSY` remains available for a
different temporary-busy condition.

## Settings Snapshot

Append without reordering existing fields:

```cpp
BeginPolicy beginPolicy{BeginPolicy::REQUIRE_PRESENT};
Status beginProbeStatus{Status::Ok()};
DeviceIdentity identity{};
CapabilitySnapshot capabilities{};
```

Retain the legacy individual feature-byte fields and populate them from the
atomic `CapabilitySnapshot` so existing callers remain source-compatible.

`getSettings()` remains cache-only.

## Private Helpers

Use one set of helpers for `begin()`, `probe()`, and `recover()`:

```cpp
Status _readAndValidateIdentityRaw(DeviceIdentity& out);
Status _readCapabilitiesRaw(CapabilitySnapshot& out);
Status _readAndValidateIdentityTracked(DeviceIdentity& out);
Status _readCapabilitiesTracked(CapabilitySnapshot& out);
Status _busResetTracked();
void _publishIdentityAndCapabilities(
    const DeviceIdentity& identity,
    const CapabilitySnapshot& capabilities);
void _clearIdentityAndCapabilities();
void _latchSemanticOffline(const Status& cause);
bool _normalOperationAllowed(Status& status) const;
```

Equivalent names are allowed. Do not duplicate three identity procedures.
`_busResetTracked()` is the tracked wrapper around Prompt 01's one raw reset
implementation. It is classified as a tracked transport wrapper and is the
only reset path allowed to call `_updateHealth()`.

All reads go into local zero-initialized candidates. Publish neither identity
nor capabilities until the complete applicable procedure succeeds. Never clear
or partially update the live cache before a candidate read.

## Identity Rules

Before declaring the device compatible, validate:

- `group == cmd::SENSOR_GROUP_ID`;
- `subgroup == cmd::SENSOR_SUBGROUP_ID`;
- `(availableMeasurements & cmd::AVAILABLE_MEAS_MASK) != 0`.

Return:

```cpp
Status::Error(Err::NOT_SUPPORTED, "Unexpected group id", group);
Status::Error(Err::NOT_SUPPORTED, "Unexpected subgroup id", subgroup);
Status::Error(
    Err::NOT_SUPPORTED,
    "CO2 measurement not advertised",
    availableMeasurements);
```

A responding incompatible device is not `DEVICE_NOT_FOUND`.

On success:

- set all raw identity values;
- set `co2Available=true`;
- set `valid=true`.

On failure, the caller's candidate remains invalid and no live cache changes.

## Capability Rules

Read custom-memory bytes `0x03..0x09` with one pointer write and sequential
auto-increment reads. Prompt 01's pointer-completion ordering applies.

The capability snapshot is valid only when all seven bytes read successfully.
A partial read must not publish any byte.

A strict or recovered startup must not become `READY` when capability loading
fails. Return the original transport/protocol status. Do not silently continue
with all optional features disabled, because that hides a failed capability
transaction.

Add constants:

```cpp
static constexpr uint8_t CUSTOM_ADJUSTMENT_SUPPORT = 0x03;
static constexpr uint8_t CUSTOM_ADJUSTMENT_POINT_SUPPORT = 0x04;
static constexpr uint8_t CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT = 0x05;
static constexpr uint8_t CUSTOM_ADJUSTMENT_TIME_SUPPORT = 0x06;

static constexpr uint8_t FEATURE_CO2_CUSTOM_ADJUSTMENT = 0x08;
static constexpr uint8_t FEATURE_CO2_ADJUSTMENT_POINT = 0x08;
static constexpr uint8_t FEATURE_CUSTOM_ADJUSTMENT_TIME_GENERAL = 0x01;
static constexpr uint8_t FEATURE_CO2_ADJUSTMENT_TIME = 0x08;
```

Prompt 04 will use the calibration bits. Do not add calibration policy here.

## `begin()` State Machine

`BeginPolicy::REQUIRE_PRESENT`:

1. reject double initialization;
2. reset stopped state;
3. validate/normalize config without publishing partial state;
4. store the normalized config needed for bounded raw work;
5. require idle bus or successful raw bus reset;
6. raw-read and validate full identity;
7. raw-read all capabilities;
8. atomically publish both;
9. set initialized;
10. enter `READY`;
11. set `beginProbeStatus=OK`.

Any failure returns that failure, clears the stopped/cache state, and leaves
`UNINIT`.

Config validation accepts only the two defined `BeginPolicy` values. Any cast
or corrupted value returns `INVALID_CONFIG` before line I/O. Prompt 01's static
timing query uses the same validation.

`BeginPolicy::ALLOW_ABSENT` follows the same procedure, with exactly one narrow
exception:

- only `Err::DEVICE_NOT_FOUND` from an authoritative presence mechanism may be
  accepted as absent.

The current GPIO E2 path has no authoritative physical-presence mechanism.
Identity NACK therefore remains NACK under both policies, performs no retry or
long delay, clears stopped/cache state, and leaves `UNINIT`. A clean STOP does
not change this: D8 measurement-priority operation permits a responsive slave
to NACK while measuring.

Do not accept any of these as absence:

- `INVALID_CONFIG`;
- `BUS_STUCK`;
- `TIMEOUT`;
- `PEC_MISMATCH`;
- `NACK`;
- `NOT_SUPPORTED`;
- a capability-read failure after identity responded;
- any cleanup or internal error not proving absence.

On accepted absent startup:

- return `Status::Ok()`;
- set `isInitialized()==true`;
- set `state()==DriverState::OFFLINE`;
- keep identity/capabilities invalid and zero;
- set normalized `consecutiveFailures` to `offlineThreshold`;
- keep `totalFailures==0`;
- keep transport `lastError`/`lastErrorMs` unchanged;
- store the accepted authoritative absence in `beginProbeStatus`;
- perform no retry.

This policy is reusable optional-device lifecycle behavior. Do not add an
`ABSENT` driver state.

## Latched OFFLINE Guard

Every normal tracked E2 operation must check the offline latch before touching
lines. Put the check in the common tracked wrapper path so a future public
helper cannot accidentally bypass it.

The guard:

- applies when initialized and `DriverState::OFFLINE`;
- returns `Err::OFFLINE`;
- performs zero line callbacks and zero transaction-count changes;
- does not update health counters.

The guard must not block:

- `probe()`;
- `recover()`;
- public `checkBusIdle()`;
- public `busReset()`;
- cache-only getters.

Public `busReset()` and `checkBusIdle()` remain raw, diagnostic, and
health-neutral. A successful public reset cannot revive `OFFLINE`.
`_busResetTracked()` is used only inside `recover()`.

Use a narrowly scoped private recovery bypass for tracked transfers during
`recover()`. Do not expose the bypass publicly.

No successful ordinary call may silently move `OFFLINE` to `READY`.

## `probe()`

`probe()`:

- requires initialization;
- is callable while offline;
- uses raw helpers;
- validates full identity, not group only;
- does not require or publish feature cache;
- does not change driver state;
- does not change any health counter or timestamp;
- does not change `beginProbeStatus`;
- does not change live identity/capability cache.

Return the original precise status.

## `recover()`

`recover()` is the only operation that can restore an initialized `OFFLINE`
driver.

Required procedure:

1. require initialization;
2. perform the bounded bus-idle/reset procedure and return its failure
   immediately;
3. using a scoped offline bypass, tracked-read and validate full identity into
   a local candidate;
4. tracked-read all capabilities into a local candidate;
5. publish both only after complete success;
6. clear `beginProbeStatus` to OK;
7. transition to `READY`;
8. clear consecutive failures according to existing health rules.

If any recovery step fails after recovery entered from `OFFLINE`:

- do not publish either candidate;
- clear live identity/capabilities rather than retaining stale claims;
- retain the original precise failure;
- ensure state remains `OFFLINE`;
- ensure `consecutiveFailures >= offlineThreshold`;
- do not perform a hidden retry.

Recovery from `DEGRADED` follows the same full validation/reload procedure. A
transport failure follows the existing per-transfer health semantics.
Successful recovery sub-transfers legitimately reset the prior failure streak;
a later failed transfer starts a new streak. Do not restore a stale entry
streak, batch a recovery into a fake single transfer, or force one later
transfer failure offline.

A fully responding but unsupported identity or missing CO2 capability is
different. Add one small private semantic-offline latch used by this case and
accepted absence. It:

- sets `DriverState::OFFLINE`;
- normalizes `consecutiveFailures` to at least `offlineThreshold` solely to
  preserve the existing four-state invariant;
- does not increment `totalFailures`, set a fake transfer timestamp, or call
  `_updateHealth()`;
- records the semantic `NOT_SUPPORTED` diagnostic;
- clears live identity/capability claims.

Document that this normalized streak is a state latch, not invented failed
wire traffic. In all cases, do not leave recovery spuriously `READY` merely
because an earlier sub-transfer succeeded.

## Health Rules

- `_updateHealth()` remains called only inside tracked transport wrappers.
- begin's pre-initialization raw work does not increment lifetime counters.
- authoritative accepted absent startup does not increment counters.
- probe remains health-neutral.
- tracked recovery transport success/failure updates health.
- offline-guard rejection itself is not a new transport failure.
- identity/capability semantic `NOT_SUPPORTED` is not a bus-transfer failure
  when the underlying transfers succeeded; explicitly set final state without
  falsifying transport counters.

Do not collapse semantic identity failure into NACK/absence.

## Timing-Bound Extension

Append, without renumbering existing `OperationKind` values:

```cpp
BEGIN_REQUIRE_PRESENT = 9
BEGIN_ALLOW_ABSENT = 10
PROBE_IDENTITY = 11
RECOVER_IDENTITY_AND_CAPABILITIES = 12
```

Update `operationTimingBound()` and its documentation/tests:

- strict begin includes bus reset worst case, identity, pointer completion, and
  seven capability reads;
- allow-absent begin uses the same conservative upper bound;
- probe includes full identity;
- recover includes bus reset, full identity, and all capabilities.

Observed fake elapsed time for every path must remain no greater than the
advertised bound.

## Native Tests

Add tests for:

1. config default is `REQUIRE_PRESENT`;
2. public enum numeric values are stable;
3. an invalid cast `BeginPolicy` is `INVALID_CONFIG` with zero line I/O;
4. strict begin with absent fake fails and remains `UNINIT`;
5. allow-absent begin with NACK preserves NACK and remains `UNINIT`;
6. strict and optional NACK perform one attempt with no hidden retry/long wait;
7. authoritative `DEVICE_NOT_FOUND`, only if an injectable mechanism exists,
   is the sole accepted absence;
8. allow-absent rejects timeout;
9. allow-absent rejects SCL/SDA bus stuck;
10. allow-absent rejects PEC mismatch;
11. allow-absent rejects wrong group;
12. allow-absent rejects wrong subgroup;
13. allow-absent rejects missing CO2 bit;
14. begin/probe/recover all validate group, subgroup, and CO2 bit;
15. a feature-read failure after valid identity fails begin and leaves
    `UNINIT`;
16. failure on each of the seven feature bytes never publishes a partial
    capability snapshot;
17. successful begin publishes identity and all seven capabilities atomically;
18. cache-only identity/capability/settings access performs zero line I/O;
19. normal reads/writes while offline return `OFFLINE` with zero line I/O;
20. offline reached through health failures has the same latch behavior as any
    authoritative accepted absence;
21. probe while offline works and changes no health/cache state;
22. a later explicit begin after an uninitialized NACK can publish fresh
    identity/features; recover remains for initialized sessions;
23. failed bus reset in recovery entered from offline returns immediately and
    remains offline;
24. failure at every identity/capability stage in recovery entered from
    offline remains offline and publishes no partial cache;
25. recovery entered from degraded follows per-transfer reset/failure
    semantics, while responding unsupported identity uses the semantic offline
    latch without incrementing lifetime transport failures;
26. no normal call can revive offline;
27. `end()` clears policy-derived runtime diagnostics/caches to defaults;
28. appended timing kinds are exact, cache-only, and conservative.

Extend the fake with settable group, subgroup, available-measurements, and all
seven capability bytes plus a deterministic fail-at-transfer index.

## Documentation

Update:

- public Doxygen;
- `README.md`;
- `CHANGELOG.md` under `Unreleased`;
- `docs/EE871_E2_OPERATION_TIMING_BOUNDS.md`;
- `docs/EE871_E2_HARDENING_FINAL_REPORT.md`.

Explain:

- strict versus optional-device startup;
- only authoritative `DEVICE_NOT_FOUND` can be accepted offline;
- E2 NACK is not physical-absence evidence and leaves begin uninitialized;
- responding incompatible devices and partial capability reads fail closed;
- `OFFLINE` is latched and explicit recovery is required;
- probe is diagnostic/raw and non-mutating;
- identity and capabilities are atomically cached;
- application retry cadence remains outside the library.

Do not bump the version in this prompt.

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

Build the pure ESP-IDF example when `idf.py` is available. Report availability
and results exactly.

## Handoff

Create:

```text
docs/reports/ee871_prompt_02_lifecycle_identity_handoff_YYYYMMDD.md
```

Include baseline, files, state-transition table, accepted/rejected absent
status table, cache atomicity design, timing-bound changes, test/build results,
and explicit deferrals.

## Acceptance Criteria

- default begin remains strict;
- optional begin accepts only authoritative `DEVICE_NOT_FOUND`;
- all responding devices require full EE871 CO2 identity;
- all seven capabilities publish atomically;
- feature-read failure cannot produce READY;
- normal calls are bus-silent while offline;
- probe is health/cache neutral;
- recover is the sole route back online and reloads all identity/capability
  state;
- no firmware policy or async framework was added;
- all required tests/builds pass or are truthfully reported.
