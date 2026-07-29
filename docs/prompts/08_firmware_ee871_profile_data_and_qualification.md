# Prompt 08: Co2Control EE871 Product, Data, Health, and Qualification

## Role and Scope

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `TunnelMonitor-node`.

Prompts 05-07 must be complete and passing. This prompt performs the authorized
vertical product integration and qualification work. It must use the exact
Co2Control board, pins, role, sample mode, timing, schema identity, and
maintenance authority from Prompt 05.

Do not change those decisions silently. If current code/guidelines conflict
with the decision report, stop and report the conflict.

This prompt must not add E2/EE871 to the TunnelMonitor product or any other
production product. Co2Control is the sole production owner.

## Read First

Read completely:

- `AGENTS.md`;
- Prompt 05 decision report;
- Prompt 06-07 handoffs;
- current `docs/guidelines/` authority;
- selected build profile, sample schema, runtime composition, Web surface;
- `BoardPins.h` and GPIO validation;
- `MeasurementRuntime`, profile collection, and sample builder;
- storage CSV/replay codecs and golden tests;
- Cloud codec/projection and golden tests;
- health projection/coordinator and capacities;
- System/App startup and owner task wrappers;
- CLI command/router/device status paths;
- HIL runner/evidence conventions.

Record branch/commit/dirty state and run the complete baseline required by the
repository before edits.

## Product Parameterization

Implement only `TARGET_PRODUCT=Co2Control`. Use the exact
`CO2CONTROL_EE871_DEVICE_ID` approved by Prompt 05 and add one Co2Control
profile row:

```cpp
inline static constexpr DeviceId kEe871{
    /* CO2CONTROL_EE871_DEVICE_ID */};

{Co2ControlBuildProfile::kEe871,
 DeviceKind::Ee871,
 "co2",
 BusId::E2,
 true,
 /* exact approved HealthRole */,
 /* exact approved DeviceEnableMode */,
 /* exact approved defaultEnabled */,
 /* exact approved acquire */}
```

If Co2Control types do not yet exist, create a concrete
`Co2ControlBuildProfile`, `Co2ControlBoardPins`,
`Co2ControlRuntimeComposition`, and product-selected sample schema following
the existing compile-time product interfaces. Do not implement Co2Control as
`#if E2` branches inside `TunnelMonitorBuildProfile`,
`TunnelMonitorRuntimeComposition`, or `TunnelMonitorSampleSchema`; that would
make E2 a variant of the wrong product and is forbidden.

Derive and compile-time validate all Co2Control device/catalog/reading/health
counts from the approved composition. Do not reuse the current TunnelMonitor
counts (`8/5/5/38` or `16/10/8`); they describe another product.

If Co2Control is diagnostics-only (`E2_ACQUIRE=false`), do not add a durable
sample mapping or Co2Control CSV/Cloud field. Regardless of acquisition mode,
the existing TunnelMonitor device counts, reading catalogs, sample fields,
schema/profile identities, CSV, replay, Cloud, Web, and goldens must remain
unchanged.

Use the exact `E2_COMPILE_GATE` approved by Prompt 05:

- value `1` only in approved Co2Control production environments and explicit
  native/HIL E2 validation environments;
- value `0` in every other production environment, including
  `tunnelmonitor_wifi` and `tunnelmonitor_wifi_hil`;
- gate EE871 dependency resolution, E2 implementation sources, runtime
  instantiation/startup, health rows, CLI/Web availability, and binding counts;
- contract-only enum declarations may remain globally visible, but no
  non-Co2Control binary may contain or advertise an E2 runtime/device.

## Board Pins and Electrical Contract

Only now add the exact approved E2 clock/data GPIO constants to the
Co2Control board profile.

Requirements:

- add both GPIOs to the one active-pin table/source of truth;
- update uniqueness/conflict/strapping assertions;
- document released=input and low=output-low behavior;
- configure no internal pull-ups unless Prompt 05 explicitly approved them;
- identify the external pull-ups and bidirectional open-drain level shifter in
  board docs;
- do not drive either line before the E2 owner starts;
- leave both released on startup failure/shutdown;
- do not repurpose a pin by deleting another owner.

Do not add conditional E2 aliases to the TunnelMonitor pin table. Build-time
checks must prove TunnelMonitor and every other non-Co2Control board/profile
have no active E2 pins.

Software checks do not close voltage/rise-time/level-shifter HIL.

## Runtime Composition

Extend only the Co2Control permanent runtime composition with exactly:

- one `Ee871Module`;
- one immutable `E2DeviceBinding`;
- selected E2 config from Prompt 05.

The Co2Control composition owns the module, immutable config, and binding array
and exposes fixed binding/config accessors; it never exposes the module to
consumers. In the selected
E2 runtime translation unit, instantiate in lifetime order exactly one static
`IdfE2GpioBackend` and one static `E2Runtime` referencing it. The runtime owns
the owner, ingress queue, tracked-terminal store, and worker and is the
backend's sole logical caller. Do not put a second backend/owner/worker in
product composition.

Expose runtime command/status facade functions, not the backend, module, or
`EE871::EE871`.

Create the selected static-instance facade using:

```text
include/TunnelMonitor/e2/E2RuntimeFacade.h
src/e2/E2RuntimeFacade.cpp
```

Compile and link this facade only when the approved E2 gate is enabled.
Non-Co2Control production builds must not receive a dormant global facade or
stub task; their call sites are excluded at compile time.

Expose the Prompt 07 wrapper through the existing cross-task patterns:

```cpp
bool e2RuntimeEnabled();
bool e2RuntimeConfigure(ErrorDetail&);
bool e2RuntimeBegin(uint64_t nowMs);
void e2RuntimeEnd(uint64_t nowMs);
E2RuntimeSubmission e2RuntimeSubmit(
    const E2Command&, uint64_t nowMs);
DeviceModuleStartResult e2RuntimeSubmitDeviceCommand(
    const DeviceCommand&, uint64_t nowMs);
DeviceModuleStartResult e2RuntimeCancelDeviceCommand(
    DeviceId, RequestId, uint64_t nowMs, ErrorDetail);
bool e2RuntimeTakeDeviceResult(
    DeviceId, RequestId, DeviceMeasurementResult&);
bool e2RuntimeDeviceRequestOutstanding(DeviceId, RequestId);
E2TrackedResultStatus e2RuntimeTakeOrReclaim(
    const E2RuntimeRequestIdentity&, uint64_t nowMs, E2Result&);
bool e2RuntimeCopyBusStatus(E2Status&);
bool e2RuntimeCopyDeviceStatus(DeviceId, DeviceModuleStatus&);
bool e2RuntimeCopyEe871Status(DeviceId, Ee871DeviceStatus&);
bool e2RuntimeApplyDeviceEnableState(
    const DeviceEnableState&, uint64_t nowMs);
ServiceHealth e2RuntimeServiceHealth(uint64_t nowMs);
SystemResourceHealth e2RuntimeResourceHealth(uint64_t nowMs);
```

`e2RuntimeCopyEe871Status()` reads only the product-family publication cache;
it does not extend the generic owner or call the module.
`e2RuntimeConfigure()` applies the immutable selected module/backend/binding
configuration and performs no GPIO, queue, or task operation; startup must
call it successfully before `e2RuntimeBegin()`.

Start order:

1. board/pin contract validation;
2. module configuration and immutable binding construction;
3. bus-silent `E2Runtime::configure()` with backend config, clock, bindings,
   and publication hook;
4. worker start; only its owner context calls backend/owner begin;
5. measurement/CLI consumers.

An optional absent sensor must not fail system boot or terminate the owner.

Update exact Co2Control binding-coverage validation. Add negative compile-time
coverage proving all non-Co2Control compositions select zero E2 bindings. Do
not add a runtime device registry or heap-owned graph.

Add a product/device-family EE871 status publication adapter beside
`Ee871Status.h` and the Co2Control composition:

- the E2 worker invokes one fixed, bus-silent publication hook after owner
  work;
- that hook calls `Ee871Module::copyStatus()` in owner context and publishes a
  fixed `Ee871DeviceStatus` cache under the repository's short critical
  section pattern;
- CLI/Web/health copy the published cache and never call the module;
- do not add `copyEe871Status` to `E2Task`, `E2DeviceBinding`, or another
  chip-neutral contract.

## Measurement Request Identity Refactor

The current measurement request range is bus-neutral in use but named I2C.
Perform a targeted refactor:

```cpp
kMeasurementDeviceRequestIdBase = 0x4D000000UL;
kMeasurementDeviceRequestIdLimit = 0x4E000000UL;
```

Move these to a bus-neutral measurement contract/config home. Preserve exact
numeric values. Existing I2C names may remain only as deprecated constexpr
aliases if another compiled public contract requires source compatibility;
otherwise delete the misleading names and update all callers.

After inventorying all current producer ranges, reserve and test:

```cpp
kE2CliRequestIdBase = 0x44000000UL;
kE2CliRequestIdLimit = 0x45000000UL;
kE2RecoveryRequestIdBase = 0x4F000000UL;
kE2RecoveryRequestIdLimit = 0x50000000UL;
```

Use wrap-safe next-ID helpers. Cross-owner/pending identity includes the
request source and the runtime's private submission token; request ID alone
must not satisfy a stale completion.

Do not create a generic bus registry.

## Measurement Runtime

Extend the existing fixed `DeviceSpec::bus` dispatch with `BusId::E2` for:

- deadline calculation;
- submit;
- take-result;
- cancellation;
- outstanding/settle behavior;
- due-sample collection;
- disabled-state behavior.

Compile the E2 dispatch branch only under the approved gate. The common
measurement code may know the append-only `BusId::E2` value, but
non-Co2Control builds must have zero selected E2 devices and no references to
the E2 runtime facade.

Use the exact profile-owned E2 operation deadline derived from released library
timing bounds plus the approved margin. Do not copy
`kOptionalDeviceOperationDeadlineMs` or the I2C outer-deadline workaround unless
the measured E2 bound proves it sufficient.

Preserve:

- exact request/device/kind/deadline identity;
- one active request per device;
- fixed FIFO owner admission;
- bounded cancellation;
- profile-ordered result collection;
- one call to the existing `ProfileSampleBuilder`.

Do not add an E2-specific branch inside the generic sample builder. The static
reading mapping is the extension point.

## CO2 Cadence and Validity Policy

Implement exactly Prompt 05's policy.

For the recommended scheduled-average policy:

- `Measure` is requested only as part of a scheduled/manual sample cycle;
- no generic five-second live refresh is added;
- MV4 checked average is the only durable source;
- warm-up is 10,000 ms from present initialization/recovery;
- pre-warm-up result is stale/invalid, never zero/valid;
- status-trigger side effects are accepted only as part of the named checked
  sample;
- the next scheduled attempt follows the application cadence;
- no catch-up burst occurs;
- raw MV3/MV4 never enter the sample builder.

Define a selected stale threshold based on the approved cadence and acquisition
deadline. Use wrap-safe 64-bit project helpers.

Recovery cadence belongs to product/runtime policy:

- submit explicit `RecoverDevice` only while module is offline;
- use the approved bounded backoff;
- no normal sample call implicitly recovers;
- no blind retry after an ambiguous maintenance mutation;
- hotplug becomes usable only after successful recovery and a new warm-up.

## Sample and Durable Schema

When Prompt 05 authorizes durable CO2, append:

```cpp
SampleFieldId::Co2Ppm = 37
SampleFieldId::Count = 38
```

at the current global append point, after rechecking the baseline. This
append-only declaration does not authorize the field in any existing product.
Map it only in the Co2Control schema; do not add it to the TunnelMonitor
reading map, valid/required masks, CSV, Cloud projection, or Web model.

Add the EE871 catalog and reading mapping:

```cpp
{{Co2ControlBuildProfile::kEe871,
  toDeviceReadingId(Ee871ReadingId::Co2Ppm)},
 {37,
  "co2.ppm",
  "co2_ppm",
  "ppm",
  "co2",
  "ppm",
  0,
  CloudFieldShape::ObjectField},
 kAllReadingSinks}
```

Use the exact schema/profile IDs approved by Prompt 05. If the approved current
Co2Control decision adopted the recommended values, add:

```cpp
inline constexpr const char* kSampleSchemaCo2ControlV1 =
    "co2control.sample.v1";
inline constexpr const char* kSampleProfileCo2ControlV1 =
    "co2control.v1.co2";
```

Create/use the concrete Co2Control build-profile and sample-schema types
approved by Prompt 05. Do not add an EE871 member to
`TunnelMonitorBuildProfile` or `TunnelMonitorSampleSchema`.

Do not mutate the meaning or output bytes of existing TunnelMonitor identities,
including:

```text
tm.sample.v0
tm.v1.vw8_shzk16_env_power
```

Update:

- sample field count/masks/capacity assertions;
- Co2Control reading/catalog/spec validation;
- Co2Control profile sample builder tests;
- Co2Control CSV header/order/value formatting;
- Co2Control storage/replay codec and version/header mismatch behavior;
- Co2Control Cloud object shape;
- Co2Control Web/profile instrumentation, only if that surface is authorized;
- Co2Control manual synthetic sample fixtures;
- new Co2Control byte-golden files;
- negative TunnelMonitor goldens proving no byte or schema change.

The new CSV field is `co2_ppm`. The new Cloud projection is:

```json
{"co2":{"ppm":1234}}
```

within the approved Co2Control envelope conventions. Preserve null/empty behavior
for disabled, stale, error, and absent values according to current sink rules.

Do not hand-code a second CSV/Cloud formatter.

## Health and Status

Derive Co2Control service/resource/device health counts from its actual emitted
rows and prove capacity with static assertions. Do not reuse or change the
TunnelMonitor `16/10/8` expectations. Every E2-disabled/non-Co2Control product
retains its existing counts and emits no E2 health row.

Publish:

- `ServiceId::E2Task` from the owner heartbeat/counters;
- `SystemResource::E2Bus` with `BusId::E2` from backend/transport health;
- the exact selected EE871 `DeviceHealth` from module cache.

Freeze roles:

- when E2 is compiled/running, `ServiceId::E2Task` is required liveness;
- `SystemResource::E2Bus` is required only if at least one selected,
  enabled-required E2 device exists; otherwise it is optional;
- the EE871 device role is exactly the Prompt 05 profile decision.

Keep domains separate:

- absent/present belongs to device presence;
- only authoritative accepted `DEVICE_NOT_FOUND` is device absence and does
  not fault the E2 resource;
- NACK remains exact device transport context, is not proof of physical
  absence, and may reflect measurement-priority operation;
- timeout/stuck/PEC/backend/protocol faults affect the E2 resource and device
  transport health;
- other NACK outcomes retain exact device transport context without turning an
  optional absence into an aggregate required-bus failure;
- CO2 sensor status and checked range errors belong to sample/device-domain
  health and do not fault the bus;
- stale/warm-up belongs to sample/device freshness;
- unresolved maintenance mutation is a maintenance/device fault, not a fake bus
  timeout.

Apply Prompt 05's optional/required role exactly. Runtime disabled remains
visible, line-silent, and excluded from aggregate required health only when E2
is compiled and the Co2Control EE871 device uses `RuntimeToggle`. When E2 is
compile-disabled, emit no E2 service/resource/device row and retain the
environment's prior counts.

Update shared exhaustive enum-name mappings as required, but add E2
service/resource/device rows and Web/CLI capability only to Co2Control.

## Operator Interface

Add bounded read-only production commands:

```text
e2 status
e2 probe co2
e2 recover co2
```

Register and compile these commands only in Co2Control. The TunnelMonitor and
all other production command registries/help/availability tables must contain
no `e2` topic or command.

On the frozen baseline where `CliCommandId::VerboseStorage=97`, append:

```cpp
CliCommandId::E2Status = 98
CliCommandId::E2Probe = 99
CliCommandId::E2Recover = 100
CliOwnerResultSource::E2 = 5
CliPendingDisplayIntent::E2OperationResult = 7
```

If the baseline changed, inventory and append the next explicit values without
renumbering; record the discrepancy in the handoff.

Rules:

- probe/recover submit through the E2 runtime facade; they never call
  `E2Task`, module, library, or backend directly;
- CLI pending state retains command ID, request source, request/device,
  deadline, and returned private runtime submission identity, following the
  existing tracked-result pattern;
- append `E2RuntimeRequestIdentity e2Identity{}` to the fixed pending record;
  match/reclaim it as a whole and never accept request-ID-only completion;
- `e2 status` reads `E2Status`, generic `DeviceModuleStatus`, and the
  product-family `Ee871DeviceStatus` publication caches only;
- help explains recovery is explicit;
- help distinguishes bus errors, absence, sensor error, range, warm-up, and
  unresolved mutation.

Do not add production:

- arbitrary custom-memory read/write;
- address/value write;
- filter/interval/calibration/auto-adjust;
- raw fast/average commands presented as validated samples.

Raw reads may exist only in a separately authorized HIL build and must be
labelled diagnostic, with status-trigger implications documented. Do not make
them part of normal storage/Cloud/UI.

Require `MAINTENANCE_SURFACE=disabled` for this prompt unless Prompt 05 also
authorized and a separate concrete maintenance prompt has already allocated
and implemented exact authenticated command/result types, deadlines,
confirmation, mutation diagnostics, audit events, and reconciliation. If not,
stop rather than inventing maintenance operations here. Never automatically
retry a maintenance mutation.

## Settings

If `DeviceEnableMode::RuntimeToggle` was approved:

- add one typed Co2Control-profile enable binding;
- preserve persistence format/version rules;
- update default/copy/validation/redaction/status tests;
- disabling cancels/settles work and releases lines;
- re-enabling requires explicit initialize/recover and warm-up.

If `Always` was approved, do not add a redundant setting.

Do not add an EE871/E2 setting key, persisted field, default, or UI control to
TunnelMonitor or another product.

Cadence and maintenance configuration must not be smuggled into unrelated
settings fields.

## Restart and Shutdown

Extend only the Co2Control App/restart coordinator sequence:

1. quiesce Measurement and CLI recovery producers;
2. call `e2RuntimeEnd()`, which closes ingress and asks the still-live worker
   to cancel/settle accepted work within its published bound;
3. the worker calls `E2Task::end()` in owner context and acknowledges;
4. the runtime joins/stops the worker;
5. verify both physical lines are released.

Do not delete a worker while another producer can enqueue or leave a terminal
reservation orphaned.

## Native Software Matrix

Add cross-layer tests for:

1. exact Co2Control device row/ID/kind/key/bus/role/enable/acquire;
2. binding coverage and counts;
3. exact pins and uniqueness checks;
4. owner/module/backend startup order;
5. optional-device NACK or authoritative absence does not fail boot;
6. NACK at boot then explicit begin/recover, warm-up, and valid sample;
7. wrong identity never publishes a sample;
8. timeout/NACK/stuck/PEC and explicit recovery backoff;
9. disabled device has zero line activity;
10. scheduled checked average/fast mode exactly as approved;
11. no unapproved five-second refresh;
12. warm-up result stale/invalid;
13. status sensor error and out-of-range masks;
14. bus resource remains healthy for sensor/range-only failures;
15. 64-bit deadline/completion/stale boundaries;
16. queued/active cancellation;
17. sample builder maps only valid `Co2Ppm`;
18. `SampleFieldId`, catalog, mapping, and counts;
19. exact new Co2Control schema/profile strings;
20. Co2Control CSV header/order/empty/error/value goldens;
21. Co2Control replay encoding/header mismatch/version behavior;
22. Co2Control Cloud `co2.ppm` value/null goldens;
23. Co2Control Web/profile/status capability and device row when authorized;
24. service/resource/device health counts and aggregation;
25. read-only CLI routing/results/help;
26. no arbitrary write command exists;
27. TunnelMonitor and every other non-Co2Control product retain exact prior
    device/pin/health/schema/CSV/replay/Cloud/Web goldens;
28. compile-time E2 gate, dependency graph, source inclusion, selected owner
    binding counts, and CLI availability are exact in every environment:
    enabled only for Co2Control production and explicit validation;
29. restart/shutdown quiesces producers, settles results, and releases both
    lines;
30. typed EE871 status is worker-published/cache-only and no UI calls the
    module;
31. binary/map/source checks find no E2 worker, EE871 dependency, E2 pins,
    `co2_ppm`, or `e2` operator surface in non-Co2Control production builds;
32. complete Prompt 05 production/validation environment matrix and all
    repository-required profile/Web/build checks.

Perform a fresh post-change audit for duplicated owner/module/schema/status
paths. Delete superseded targeted code rather than leaving forwarding shims.

## Hardware HIL

Create/update the authorized HIL plan and runner. Run only when the exact board
and fixture are available.

This is a Co2Control HIL plan. Do not expose its commands, pins, or fault hooks
in TunnelMonitor firmware.

Required evidence:

- exact firmware commit, library tag/commit, board revision, sensor identity,
  level shifter, pull-ups, supply, cable, and test timestamp;
- CLK/DATA waveform at approved minimum and maximum clock;
- released-high voltage and rise time;
- proof neither ESP32 pin drives high;
- real clock stretching;
- absent boot and later hotplug;
- unplug/replug online;
- SCL stuck low and SDA stuck low;
- PEC fault injection where feasible;
- warm-up first-valid timing;
- status-trigger-to-next-ready timing;
- selected average/fast behavior;
- sensor-error cases where feasible;
- legal 150/300 ms write timing in a maintenance/HIL-only build;
- owner heartbeat/queue/deadline behavior during long/fault cases;
- intended cable length;
- long-run soak with no unexpected reset, line lock, result leak, false-valid
  CO2, or unbounded queue growth.

Store raw and condensed evidence in the repository-prescribed paths. If HIL is
not run, leave every physical gate explicitly pending.

## Documentation and Handoff

Update all active Co2Control guideline, dependency, board, measurement, health,
operator, schema, Cloud/storage, and validation documents. Update shared
authority only where append-only contracts require it. Do not rewrite
TunnelMonitor product documentation as if it owned E2.

Create:

```text
docs/reports/ee871_product_integration_handoff_YYYYMMDD.md
```

Include:

- exact Prompt 05 decisions;
- files and targeted refactors;
- memory/stack/queue sizes;
- operation deadline table;
- sample/status/error/health mapping;
- durable schema migration;
- complete command/test/build results;
- exact HIL status and evidence links;
- remaining release blockers.

## Validation

Run every repository-required native, HIL-build, profile, Web, storage/Cloud
golden, generator, and firmware build check. At minimum:

```powershell
python -m platformio test -e native
python -m platformio test -e native_hil_fram
python -m platformio run -e tunnelmonitor_wifi
python -m platformio run -e tunnelmonitor_wifi_hil
git diff --check
```

Additionally run every exact Co2Control production and validation environment
listed by Prompt 05. The two TunnelMonitor builds above are mandatory negative
isolation checks: they must resolve no EE871 dependency and expose no E2
runtime/pins/health/CLI/data surface. Repeat the same dependency, source/filter,
link-map or symbol, profile, and operator-surface checks for every entry in
`NON_CO2CONTROL_PRODUCTION_ENVIRONMENTS`; a successful build alone is not proof
that dormant E2 code was excluded. Never claim an unrun command/HIL pass.

## Production Acceptance Gate

Production consideration requires all of:

- Prompts 01-04 P0 library gates closed and exact immutable release pinned;
- Co2Control product/pins/electrical/schema authority explicit;
- E2/EE871 dependency, pins, runtime, device, health, data, and operator
  surfaces absent from every non-Co2Control production product;
- sole E2 owner and no direct hardware access elsewhere;
- checked sample only in durable data;
- warm-up/status-trigger/recovery/stale policy tested;
- precise transport/sensor/range/mutation health separation;
- static queue/result/deadline/cancellation tests passing;
- schema/storage/replay/Cloud/Web changes versioned and golden-tested;
- maintenance absent or explicitly authorized/verified/audited;
- required physical waveform/electrical/hotplug/fault/soak HIL complete.

If physical HIL is pending, report software integration complete but production
field acceptance pending. Do not weaken the gate.
