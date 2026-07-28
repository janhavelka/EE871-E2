# Prompt 05: Co2Control E2 Product and Electrical Decision Gate

## Role

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `TunnelMonitor-node`.

This is a mandatory decision/documentation gate. Do not add runtime code,
contracts, pins, dependencies, profile rows, schema fields, tasks, or stubs in
this prompt.

The product allocation is already decided:

```text
TARGET_PRODUCT = Co2Control
```

E2 and EE871 must not be enabled, instantiated, pinned, assigned pins, exposed,
or used by the existing `TunnelMonitor` product or any other production
product. This prompt resolves the remaining Co2Control details; it must not
reopen product ownership.

## Required Invocation Inputs

The person running this prompt must replace every `<REQUIRED>` value with
explicit approved authority:

```text
TARGET_PRODUCT = Co2Control
CO2CONTROL_PROFILE_KEY = <exact compile-time profile key>
TARGET_BOARD_PROFILE = <exact board profile key>
E2_COMPILE_GATE = <exact repository feature macro>
E2_PRODUCTION_ENVIRONMENTS = <exact Co2Control PlatformIO environment list>
E2_VALIDATION_ENVIRONMENTS = <exact native/HIL test environment list>
NON_CO2CONTROL_PRODUCTION_ENVIRONMENTS =
    <complete list; every entry must compile with E2 disabled>
CO2CONTROL_EE871_DEVICE_ID = <exact non-conflicting DeviceId>
E2_CLK_GPIO = <exact GPIO>
E2_DATA_GPIO = <exact GPIO>
E2_INTERNAL_PULLUPS = <true | false with electrical authority>
E2_DEVICE_ADDRESS = <0..7>
E2_CLOCK_LOW_US = <exact value >=100>
E2_CLOCK_HIGH_US = <exact value >=100, combined period within 500..5000 Hz>
E2_OPERATION_DEADLINE_MS = <exact library-bound-plus-margin value>
E2_BACKEND_RESTORE_BUDGET_MS = <exact conservative value>
E2_RECOVERY_BACKOFF = <exact initial/max/multiplier-or-table policy>
E2_OFFLINE_THRESHOLD = <exact 1..255 value>
E2_DEVICE_ROLE = <Required | Optional>
E2_ENABLE_MODE = <Always | RuntimeToggle>
E2_DEFAULT_ENABLED = <true | false>
E2_ACQUIRE = <true | false>
CO2_READING_SELECTION = <AVERAGE_ONLY | FAST_ONLY | BOTH_DISTINCT_FIELDS>
CO2_WARMUP_MS = <approved value>
CO2_SAMPLE_STALE_MS = <0 for no ReadLast or exact approved age>
CO2_SAMPLE_POLICY = <scheduled-sample-only or exact separate refresh policy>
EE871_RELEASE = <exact immutable tag/version>
EE871_COMMIT = <exact immutable commit resolving that release>
ELECTRICAL_AUTHORITY = <schematic/review/evidence reference>
DATA_PROFILE_VERSION_DECISION = <exact new profile/schema identity or explicit no-data-change>
MAINTENANCE_SURFACE = <disabled or exact authorized operations/interfaces>
```

If any value is absent, contradictory, or not backed by project authority:

1. perform read-only repository checks;
2. create only the blocker report named below;
3. list the missing decisions and affected downstream files;
4. stop without changing guidelines or code.

Do not invent GPIOs, reuse TunnelMonitor pins/profile IDs, or convert a future
reservation into runtime authority. If any requested authority assigns E2 or
EE871 to a product other than Co2Control, treat it as a contradiction and
produce the blocker report.

## Read First

Read completely:

- `AGENTS.md`;
- `docs/guidelines/overview.md` if present;
- `docs/guidelines/implementation_plan.md`;
- `docs/guidelines/open_questions.md`;
- `docs/guidelines/decisions.md`;
- `docs/guidelines/target_architecture.md`;
- `docs/guidelines/interfaces.md`;
- `docs/guidelines/ownership.md`;
- `docs/guidelines/states.md`;
- `docs/guidelines/measurement_data.md`;
- `docs/guidelines/time_health_watchdog.md`;
- `docs/guidelines/dependency_policy.md`;
- `docs/guidelines/reference/hardware_and_build_facts.md`;
- `include/TunnelMonitor/BoardPins.h`;
- selected build-profile headers;
- `platformio.ini`;
- the EE871 audit and Prompt 04 release handoff in sibling `EE871-E2`.

Record branch, commit, dirty state, the current Co2Control reservation, and the
currently selected production product/profile facts.

## Existing Authority That Must Be Reconciled

At the 2026-07-28 baseline:

- `SystemResource::E2Bus` is declaration-only;
- `BusId` has no E2 value;
- `ServiceId` has no E2 owner;
- `DeviceKind` has no EE871;
- `BoardPins.h` has no E2 lines;
- no EE871 dependency is pinned;
- `open_questions.md` assigns E2 to future Co2Control composition;
- the selected TunnelMonitor profile has no CO2 device/field.

These are not defects to work around. Concretizing Co2Control supersedes only
the future-Co2Control reservation. The selected TunnelMonitor profile's lack
of CO2/E2 remains intentional and must be preserved.

## Decision Rules

### Product

Concretize exactly one compile-time `Co2Control` product/profile. Do not
implement a runtime product selector and do not add EE871 to the existing
`TunnelMonitor` profile.

Freeze a negative product matrix:

- only approved Co2Control production environments define the E2 production
  gate as `1`;
- every other production environment defines it as `0` or omits E2 sources
  and the EE871 dependency entirely, according to repository convention;
- native/HIL validation environments may compile E2 only for tests and are not
  production product ownership;
- non-Co2Control profiles contain no E2 `DeviceSpec`, binding, pins, schema
  mapping, health row, CLI/Web capability, or runtime startup call.

Validate the approved row against existing `DeviceSpec` and schema contracts
before accepting it:

- `E2_ENABLE_MODE=Always` requires `E2_DEFAULT_ENABLED=true`;
- `E2_ACQUIRE=true` requires durable CO2 authorization, an EE871 reading
  catalog, and at least one mapped reading;
- `DATA_PROFILE_VERSION_DECISION=no-data-change` requires
  `E2_ACQUIRE=false`;
- `E2_ACQUIRE=false` must not change selected sample field/schema/CSV/Cloud
  counts.

Record the exact `deviceSpecsValid()`/`readingSpecsValid()` implications so an
invalid combination is rejected at this decision gate rather than late in
Prompt 08.

### Board and electrical interface

Validate the selected GPIOs against:

- boot-strapping restrictions;
- flash/PSRAM pins;
- SD, UART/RS485, I2C, display/button, LED, outputs, and future reserved pins;
- duplicate active-GPIO assertions;
- the exact board profile.

The authority must state:

- E2 CLK and DATA are open-drain;
- MCU high means release/input, low means output-low;
- external pull-up resistance is within 4.7 kOhm to 100 kOhm;
- bus-high voltage is 3.6-5.2 V;
- a bidirectional open-drain level shifter is used for the 3.3 V ESP32;
- intended cable length and the `<=10 m` guideline;
- whether internal pull-ups are disabled;
- how rise time and released-high voltage will be measured.

Do not treat software as evidence that the electrical design is safe.

### Product defaults

If the Co2Control product owner has not requested different values, present these as
recommendations for approval, not hidden assumptions:

- optional device role;
- `AVERAGE_ONLY` checked MV4 as the durable `co2_ppm` field;
- `CO2_WARMUP_MS=10000`;
- scheduled sample only;
- no generic five-second live refresh;
- no arbitrary custom-memory writes;
- maintenance disabled in the first production integration.

MV3 fast and MV4 average become separate reading IDs if both are selected.
However, the Prompt 04 library release provides one checked procedure at a
time. If `BOTH_DISTINCT_FIELDS` is selected, stop downstream integration until
a separate library prompt specifies and tests one coherent paired-value/status
procedure; do not fake it with two sequential status-triggering sample calls.
Never hide their difference behind an unversioned runtime switch.

### Timing

Record:

- selected E2 clock timing within 500-5,000 Hz;
- normal 25 ms bit and 35 ms byte limits;
- 150 ms normal/pointer completion;
- 300 ms interval-pair completion;
- checked-sample and recovery bounds from the released library;
- the owner command deadline and margin;
- recovery backoff and maximum failure policy;
- warm-up, status-trigger, next-ready, and stale policy.

Do not copy the I2C owner's 20 ms timing.

### Durable data

If CO2 enters the Co2Control sample/storage/replay/Cloud contract:

- allocate an explicit new data-profile/schema identity;
- document CSV header and Cloud shape changes;
- define replay/header mismatch behavior;
- require updated byte-golden tests.

Do not silently mutate a deployed data-profile identity.

If no durable data changes are authorized, Prompts 07-08 may implement only
Co2Control diagnostic/module infrastructure and must not publish CO2 as an
existing field. In both cases, the TunnelMonitor schema/profile IDs, CSV,
replay, Cloud projection, Web surface, and goldens remain byte-for-byte
unchanged.

### Dependency

Verify that the separately authorized release operation used Prompt 04's
release-ready source, and that the exact remote tag resolves to the specified
commit. Record whether the remote immutable tag actually exists. Prompt 04
itself does not commit, tag, or push.
Do not pin a mutable branch, local path, or unpushed commit in production.
Authorize this dependency only for Co2Control production and explicit E2
validation environments. Every non-Co2Control production dependency graph must
remain EE871-free.

### Maintenance

The first integration should normally expose no writes. If any are authorized,
list exact operations, interfaces, authentication/physical-access
requirements, deadlines, mutation diagnostics, audit events, verification,
and recovery steps.

Never authorize an arbitrary address/value custom-write command for production
CLI or web.

This eight-prompt series implements only
`MAINTENANCE_SURFACE=disabled`. If maintenance is required, this decision
report must authorize a separate concrete maintenance implementation prompt
before Prompt 08; that prompt must allocate exact command/result contracts and
deadlines. Prompt 08 must not invent maintenance types ad hoc.

## Deliverables When Fully Authorized

Update only the architecture authority documents needed to make the later
implementation unambiguous:

- `docs/guidelines/decisions.md`;
- `docs/guidelines/open_questions.md`;
- `docs/guidelines/implementation_plan.md`;
- the relevant hardware/build facts reference;
- relevant ownership/interface/measurement documents.

Add one dated report:

```text
docs/reports/co2control_ee871_e2_product_decision_YYYYMMDD.md
```

The report must contain:

- the complete approved input block;
- source/authority for each fact;
- product and board/profile decision;
- exact Co2Control production and validation environment lists;
- negative matrix proving every non-Co2Control product remains E2-disabled;
- pin-conflict matrix;
- electrical design assumptions and pending HIL;
- library tag/commit evidence;
- sample/cadence/warm-up/status-trigger policy;
- exact durable profile/schema decision;
- health role;
- maintenance decision;
- exact downstream Prompt 06-08 scope;
- unresolved physical validation items.

Do not add `E2Task`, `Ee871Module`, profile rows, or runtime enums in this
decision prompt.

## Deliverable When Blocked

Create only:

```text
docs/reports/co2control_ee871_e2_product_decision_blocked_YYYYMMDD.md
```

Do not modify guideline authority as though a decision was made.

## Validation

Run documentation/reference checks already provided by the repository plus:

```powershell
git diff --check
```

Use `rg` to prove no runtime E2 implementation or pin/dependency change entered
this prompt.

## Acceptance Criteria

- Co2Control is the sole production product owning E2/EE871;
- exact E2-enabled build environments are explicit;
- every non-Co2Control production environment is explicitly E2-disabled;
- pins are authoritative and conflict-checked;
- electrical level shifting/pull-ups/cable assumptions are explicit;
- role, readings, cadence, warm-up, deadlines, schema, and maintenance are
  explicit;
- exact immutable library release/commit is verified;
- current guideline contradictions are resolved only with real authority;
- the TunnelMonitor profile, pins, data schema, runtime composition, health
  inventory, and operator surface remain outside the approved change;
- no runtime placeholder or speculative profile change was added;
- missing authority causes a documented stop, not an invented implementation.
