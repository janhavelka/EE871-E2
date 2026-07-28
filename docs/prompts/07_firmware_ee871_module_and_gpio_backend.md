# Prompt 07: Firmware EE871 Module and ESP32 E2 Backend

## Role and Scope

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `TunnelMonitor-node`.

Preconditions:

- Prompt 04 produced release-ready source and handoff;
- the separately authorized release/tag/push gate was completed;
- Prompt 05 verified the exact immutable EE871 tag and commit;
- Prompt 05 contains a fully authorized product/electrical decision;
- Prompt 06's generic E2 owner is implemented and passing.

This prompt exact-pins the library, implements a reusable EE871 device module,
an ESP32 open-drain backend, and the E2 worker runtime wrapper. Do not yet add a
selected product row, board pins, durable sample field, CSV/Cloud/Web mapping,
or health inventory row. Prompt 08 owns the vertical product cutover.

Co2Control is the sole production target. The module/backend/runtime may be
compiled by explicit native/HIL validation environments, but they must remain
compile-disabled and absent from every non-Co2Control production firmware,
including the existing TunnelMonitor environments.

If Prompt 05 selected `BOTH_DISTINCT_FIELDS`, stop: the current library release
lacks the required coherent paired checked-sample procedure.

## Read First

Read:

- `AGENTS.md`;
- Prompt 04 release-ready handoff;
- Prompt 05 decision report;
- Prompt 06 handoff and all new E2 contracts/owner files;
- current I2C modules under `src/devices/i2c/`;
- `Rv3032Module`, `Ina228Module`, and their native tests;
- I2C ESP-IDF backend and runtime wrapper;
- product-independent reading catalog patterns;
- `platformio.ini` dependency policy;
- applicable architecture guidelines.

Record baseline branch/commit/dirty files and required test/build matrix.

Before creating code, inspect for a smaller existing helper that can be reused
without coupling E2 to hardware I2C. Do not share an abstraction merely because
both buses have clock/data wires.

## Dependency

Add the exact immutable release verified by Prompt 05 to the explicit
native/HIL validation environments that compile the module. If the approved
Co2Control production environment already exists, add it there too. Otherwise
create only a named Co2Control-specific dependency block that no existing
production environment inherits; Prompt 08 attaches it when creating the
authorized Co2Control environment.

Do not add the dependency to TunnelMonitor or any other production
environment. Use source filters/conditional dependencies consistent with
repository policy, and verify the remote tag/version resolves to the recorded
commit.

Do not use:

- a branch;
- a broad semver range;
- a mutable local path in production;
- a commit that has not been pushed;
- Arduino `Wire` or ESP-IDF I2C.

Record resolved package metadata in dependency documentation.

## Device Kind and Reading Catalog

Append:

```cpp
DeviceKind::Ee871 = 10
```

and:

```cpp
case DeviceKind::Ee871:
  return "ee871";
```

Do not renumber existing kinds.

Create:

```text
include/TunnelMonitor/device/readings/Ee871Readings.h
```

Use:

```cpp
enum class Ee871ReadingId : uint8_t {
  Co2Ppm = 0,
  Count = 1,
};

constexpr DeviceReadingId toDeviceReadingId(Ee871ReadingId id) {
  return static_cast<DeviceReadingId>(id);
}

inline constexpr DeviceReadingDescriptor kEe871Readings[] = {
    {toDeviceReadingId(Ee871ReadingId::Co2Ppm),
     "co2_ppm",
     "ppm"},
};
```

Provide a validated `DeviceReadingCatalog{DeviceKind::Ee871, ...}`.

The selected sample mode may be average or fast, but the normalized reading is
one explicitly configured `co2_ppm` field. The module's cached status must
retain which mode produced it; do not imply MV3 and MV4 are interchangeable.

## Project-Level Module Types

Create:

```text
include/TunnelMonitor/e2/Ee871Status.h
```

No EE871 library type may enter this header.

Use:

```cpp
enum class Ee871SampleMode : uint8_t {
  Average = 0,
  Fast = 1,
};

enum class Ee871ModuleState : uint8_t {
  Unbound = 0,
  NeedsInitialization = 1,
  Ready = 2,
  Offline = 3,
  Busy = 4,
  Disabled = 5,
};

struct Ee871ModuleConfig {
  DeviceId deviceId{kInvalidDeviceId};
  uint8_t deviceAddress{0};
  Ee871SampleMode sampleMode{Ee871SampleMode::Average};
  uint32_t warmupMs{10000};
  uint32_t sampleStaleMs{0};
  uint16_t clockLowUs{100};
  uint16_t clockHighUs{100};
  uint16_t startHoldUs{100};
  uint16_t stopHoldUs{100};
  uint32_t bitTimeoutUs{25000};
  uint32_t byteTimeoutUs{35000};
  uint8_t offlineThreshold{5};
};

struct Ee871DeviceStatus {
  DeviceId deviceId{kInvalidDeviceId};
  Ee871ModuleState state{Ee871ModuleState::Unbound};
  Ee871SampleMode sampleMode{Ee871SampleMode::Average};
  PresenceState presence{PresenceState::Unknown};
  HealthState health{HealthState::Unknown};
  ErrorDetail lastError{};
  bool driverInitialized{false};
  bool driverOnline{false};
  bool identityValid{false};
  bool capabilitiesValid{false};
  bool warmupActive{false};
  bool sampleValid{false};
  float co2Ppm{0.0F};
  uint8_t sensorErrorCode{0};
  bool sensorErrorCodeValid{false};
  bool persistentStateUncertain{false};
  uint64_t readyAfterUptimeMs{0};
  uint64_t lastAttemptUptimeMs{0};
  uint64_t lastSampleUptimeMs{0};
  uint64_t lastSuccessUptimeMs{0};
  uint64_t lastErrorUptimeMs{0};
  uint32_t successCount{0};
  uint32_t errorCount{0};
  uint32_t timeoutCount{0};
};
```

Use repository saturated-counter helpers and static layout checks where
applicable.

## EE871 Module

Create:

```text
src/devices/e2/Ee871Module.h
src/devices/e2/Ee871Module.cpp
```

The module privately owns one stable, non-moved `EE871::EE871` instance.
Third-party headers stay private to the module implementation boundary and do
not enter common contracts.

Required permanent shape:

```cpp
class Ee871Module {
 public:
  bool configure(const Ee871ModuleConfig&, ErrorDetail&);
  bool bind(const E2OwnerTransport&, ErrorDetail&);
  DeviceModuleStartResult start(const DeviceCommand&, uint64_t nowMs);
  DeviceModulePollState poll(uint64_t nowMs);
  bool takeResult(DeviceMeasurementResult&);
  DeviceModuleStartResult startCommand(
      const E2Command&, uint64_t nowMs);
  bool takeCommandResult(E2Result&);
  uint32_t requiredDeviceBudgetMs(DeviceRequestKind) const;
  uint32_t requiredCommandBudgetMs(E2Operation) const;
  void setEnabled(bool, uint64_t nowMs);
  void cancel(uint64_t nowMs, ErrorDetail reason);
  void onBusInvalidated(uint64_t nowMs, ErrorDetail reason);
  DeviceModuleStatus snapshot() const;
  const DeviceReadingCatalog& readingCatalog() const;
  bool copyStatus(Ee871DeviceStatus&) const;
};
```

Align minor signature details with Prompt 06's binding contract; do not add a
parallel adapter.

## Binding and Callback Rules

`configure()` and `bind()`:

- validate/copy fixed state only;
- perform zero E2/backend I/O;
- install static callback thunks into an `EE871::Config`;
- use `BeginPolicy::ALLOW_ABSENT`;
- set `delayMs` and cooperative yield;
- keep one `busUser` pointing to the stable module/transport context;
- calculate and cache all operation timing bounds using the library's static,
  bus-silent timing API;
- enter `NeedsInitialization`.

`sampleStaleMs==0` is a fail-closed policy: a retained reading is never served
as a valid `ReadLast` value. A nonzero value uses wrap-safe 64-bit age checks.
Prompt 08 supplies the approved Co2Control value; the module does not invent
it.

The two required-budget callbacks are cache-only and conservative across
lifecycle state:

| Request | Required bound |
| --- | --- |
| device `Probe` / admin `ProbeDevice` | max of `BEGIN_ALLOW_ABSENT` and `PROBE_IDENTITY` |
| device `Measure` | max of `BEGIN_ALLOW_ABSENT` and the selected checked-sample bound |
| device `ReadLast` | zero physical-operation budget |
| admin `RecoverDevice` | max of `BEGIN_ALLOW_ABSENT` and `RECOVER_IDENTITY_AND_CAPABILITIES` |

Add only the approved owner/runtime scheduling margin outside these library
bounds. Do not return a smaller number based on optimistic current state if
queued state could change before execution.

Callback thunks:

- call only the supplied `E2OwnerTransport`;
- never call another module public method recursively;
- never log;
- never allocate;
- never access GPIO directly.

Because the EE871 callback signatures cannot directly return project backend
errors, keep a per-call first-callback-error latch in the module:

1. clear it immediately before each synchronous library call;
2. setters latch the first failed transport `ErrorDetail`;
3. failed line reads latch the first error and return protocol-safe low to the
   library callback;
4. after the library returns, prefer the latched backend `ErrorDetail` over a
   secondary timeout/bus-stuck status caused by that callback failure;
5. never let later callback failures overwrite the first cause.

Before every synchronous library call:

```cpp
driver.tick(static_cast<uint32_t>(ownerNowMs));
```

After every call, read the owner's 64-bit clock again for completion/status.
The 32-bit cast is only for library-local diagnostic timestamps.

## Module Operation Semantics

Admission performs no I/O and retains one exact command. `poll()` advances at
most one bounded library public procedure per call.

### First initialization

The first `Probe`, `Measure`, or explicit diagnostics request may schedule one
initialization phase. In `poll()`:

- call `driver.begin(config)` exactly once;
- `ALLOW_ABSENT` NACK produces an initialized module in `Offline`, not owner
  failure;
- valid identity/capabilities produce `Ready`;
- incompatible identity is a terminal `E2UnsupportedIdentity`;
- bus-stuck/timeout/PEC remain precise failures;
- every non-accepted begin failure leaves
  `state=Offline`, `driverInitialized=false`, and latches the precise cause;
- no hidden retry follows.

On successful present initialization:

```text
readyAfterUptimeMs = actualCompletionUptimeMs + warmupMs
```

using saturating 64-bit project helpers.

Initialization terminalizes the request that caused it:

- an initializing `Probe` completes from the validated begin identity and does
  not perform a second probe;
- an initializing `Measure` completes `Stale` with no reading and does not
  survive into warm-up or perform a checked sample;
- a later independently admitted `Measure` after warm-up performs the checked
  sample.

Therefore the admission bound is the maximum of alternative procedures, not
their sum. Add an explicit test so this lifecycle rule cannot drift.

### `DeviceRequestKind::Probe`

- if state is `NeedsInitialization`, perform the one initial begin attempt;
- if a prior non-accepted begin failure left the module `Offline` and
  uninitialized, report the latched state bus-silently; only `RecoverDevice`
  may attempt begin again;
- if initialized, call the library's raw/health-neutral `probe()`;
- never call `recover()` implicitly;
- preserve NACK as an exact attempt result;
- initial accepted absence sets cached presence `Absent`;
- one later online NACK does not immediately invent permanent absence;
- return no measurement reading.

### `DeviceRequestKind::Measure`

- fail bus-silently as offline when driver is latched offline;
- before `readyAfterUptimeMs`, return `DeviceResultStatus::Stale` with no valid
  reading and no checked sample I/O;
- for average mode, call only `readCo2AverageSample()`;
- for fast mode, call only `readCo2FastSample()`;
- only `ppmValid` produces the `Co2Ppm` valid bit/value;
- only a valid newly completed sample updates `lastSampleUptimeMs`;
- sensor error sets error mask and `E2SensorError`, not bus failure;
- checked range error sets error mask and `E2ValueOutOfRange`, not bus failure;
- transport errors retain precise E2 error mapping;
- never publish raw MV3/MV4 as a valid measurement.

### `DeviceRequestKind::ReadLast`

- is entirely cache-only;
- returns valid data only when the module has one retained valid sample,
  `sampleStaleMs` is nonzero, and its 64-bit age is within that configured
  bound (`ageMs <= sampleStaleMs`);
- otherwise returns `Stale` with the stale bit set;
- performs no E2 I/O.

Idle owner polls may age the cache/status using `nowMs`; that bookkeeping is
bus-silent.

### Admin operations

- `ProbeDevice`: same explicit probe procedure;
- `RecoverDevice`: the only reinitialization/recovery path:
  - call `driver.begin(config)` when `driverInitialized=false`;
  - call `driver.recover()` only when the driver is initialized;
  - never automatically retry begin from ordinary `Measure` cadence after a
    non-accepted initialization failure.

Status and diagnostics are copied only through cache-only owner/runtime
publication APIs; there is no `ReadDiagnostics` admin command or binary
payload.

Successful recovery:

- revalidates library identity/capabilities;
- sets `readyAfterUptimeMs=completion+warmupMs`;
- enters `Ready` but does not publish a sample during warm-up.

No admin path exposes arbitrary custom writes in this prompt.

### Disable, cancel, and bus invalidation

- `setEnabled(false)`, `cancel()`, and `onBusInvalidated()` perform no line I/O;
- queued/active work completes with the exact cancellation/invalidation reason;
- bus invalidation retains the configured driver instance, sets a module-local
  `recoveryRequired` gate, releases no GPIO directly, and enters `Offline`;
- do not call `driver.end()` merely to synthesize offline state because that
  would make the intended explicit `driver.recover()` path unavailable;
- re-enable or backend restoration does not perform implicit bus work;
  `Measure` fast-fails while recovery is required; an explicit probe may
  observe presence, but only successful explicit recovery clears the gate.

## Error Mapping

Use a `switch` on `EE871::Err`, never message parsing:

Use one private contextual mapper rather than scattered special cases:

```cpp
enum class Ee871CallContext : uint8_t {
  CONFIGURATION = 0,
  IDENTITY = 1,
  RAW_TRANSPORT = 2,
  CHECKED_SAMPLE = 3,
  MAINTENANCE = 4,
};

ErrorDetail mapEe871Status(
    const EE871::Status&, Ee871CallContext);
```

| EE871 status | Project error |
| --- | --- |
| `OK` | `Ok` |
| `INVALID_CONFIG`, `INVALID_PARAM`, `OUT_OF_RANGE` parameter context | `InvalidArgument` |
| `NOT_INITIALIZED` | `Unavailable` |
| `ALREADY_INITIALIZED`, `BUSY` | `Busy` |
| `IN_PROGRESS` returned where a terminal was required | `E2ProtocolError` |
| `E2_ERROR` | `E2ProtocolError` |
| `NACK`, `DEVICE_NOT_FOUND` | `E2Nack` |
| `TIMEOUT` | `E2Timeout` |
| `BUS_STUCK` | `E2BusStuck` |
| `PEC_MISMATCH` | `E2PecMismatch` |
| `NOT_SUPPORTED` during identity/capability | `E2UnsupportedIdentity` |
| `NOT_SUPPORTED` for an admitted operation | `Unsupported` |
| `CO2_SENSOR_ERROR` | `E2SensorError` |
| checked sample `OUT_OF_RANGE` | `E2ValueOutOfRange` |
| `OFFLINE` | `E2DriverOffline` |
| `VERIFY_MISMATCH` | `E2WriteVerifyMismatch` |
| `PERSISTENT_STATE_UNCERTAIN` | `E2PersistentStateUncertain` |
| any future/unexpected code | `E2ProtocolError` |

Copy `EE871::Status::detail` exactly into `ErrorDetail::backendCode`; do not
replace it with the enum numeric value and do not expose the static message
pointer. Retain the CO2 sensor code in `sensorErrorCode` separately.

Bus-resource health counts timeout/stuck/PEC/backend/protocol transport errors.
An accepted `ALLOW_ABSENT` `NACK`/`DEVICE_NOT_FOUND` is device absence, not a
bus-resource fault. Sensor/range/warm-up results affect the device/sample
result only. Prompt 08 freezes the required/optional resource projection for
the Co2Control profile.

## ESP32 Backend

Create:

```text
src/e2/IdfE2GpioBackend.h
src/e2/IdfE2GpioBackend.cpp
```

Use native ESP-IDF GPIO/FreeRTOS APIs, not Arduino.

Exact electrical behavior:

- begin validates distinct nonnegative approved GPIOs;
- configure no internal pull-up unless Prompt 05 explicitly authorized one
  (normal production value is false);
- released state is high-impedance input;
- low state is output-low;
- set output level low before changing direction to output, avoiding a high
  glitch;
- never drive a high output;
- begin/end leave both lines released;
- read physical levels through `gpio_get_level`;
- microsecond delay uses the repository-approved native IDF primitive;
- millisecond delay sleeps/yields in task context and uses a wrap-safe
  monotonic deadline so elapsed time is at least the requested value even when
  one RTOS tick is not one millisecond;
- cooperative yield uses the approved FreeRTOS primitive;
- every backend error is bounded and mapped to `ErrorDetail`;
- `requiredRestoreBudgetMs()` returns a named conservative bound covering
  release/end plus one GPIO begin/configuration attempt, with native boundary
  tests and the selected owner deadline accounting for it;
- backend configures no hardware I2C peripheral.

Prompt 08 supplies actual Co2Control board-pin values.

## Runtime Worker Wrapper

Add a reusable `E2Runtime` wrapper type analogous to the permanent I2C runtime,
but do not instantiate or start a production static object in this prompt.
Prompt 08 owns the one Co2Control production instance after pins/bindings
exist.

Create:

```text
include/TunnelMonitor/e2/E2RuntimeTypes.h
src/e2/E2Runtime.h
src/e2/E2Runtime.cpp
```

`E2RuntimeTypes.h` contains only the fixed tracked-result identity/submission
types below and no backend/EE871 header. The concrete non-thread-safe wrapper
class remains internal under `src/e2/`.

The wrapper type owns:

- one `E2Task`;
- one fixed ingress queue of `kE2RequestQueueDepth`;
- one fixed tracked-terminal store of `kE2ResultQueueDepth`;
- one worker task handle;
- short-critical-section publication caches for `E2Status` and generic
  `DeviceModuleStatus`.

It is constructed with one non-owning `E2Backend&`. The referenced backend
storage must outlive the wrapper, and the wrapper is its sole logical caller.
Production passes one statically allocated `IdfE2GpioBackend`; native tests
pass the Prompt 06 fake through the same constructor. Do not add a second
runtime implementation, platform factory, heap allocation, or subclassed
runtime.

It may accept one fixed bus-silent owner-context publication hook
`void (*)(void*, uint64_t)` plus context. The runtime invokes it after owner
work; it is a product extension point for Prompt 08's typed EE871 status cache,
not a device registry.

Freeze its cross-task rules:

- callers never invoke the non-thread-safe `E2Task`, module, or backend
  directly;
- reserve a terminal slot before enqueueing accepted admin or measurement
  work;
- assign a private nonzero submission token and retain exact request, device,
  operation, deadline, and token identity across the runtime;
- the worker alone calls `E2Task::begin/poll/end`, drains terminal results, and
  publishes cache snapshots;
- an accepted request can never lose its terminal slot;
- stale completions cannot satisfy a reused request ID;
- queue/result capacities, tracked-result reclamation lifetime, stack/core
  priority, poll cadence, and startup-ready timeout are named constants and
  tested;
- backend `begin()` failure leaves the worker alive and retries owner
  initialization at the existing project
  `kOwnerInitializationRetryIntervalMs` (5000 ms), without task recreation or
  operator acceleration;
- this backend-initialization retry is distinct from device
  `RecoverDevice`, which remains an explicit single attempt;
- initial optional sensor absence does not terminate the worker;
- runtime `end()` first closes ingress, then asks the still-live worker to
  terminalize/cancel retained work and call `E2Task::end()` in owner context;
  only after an acknowledgement does it join/stop the worker and verify both
  lines released. A caller never invokes `E2Task::end()` directly.

Expose instance methods that Prompt 08 can wrap with the project runtime
facade, matching existing I2C semantics:

```cpp
struct E2RuntimeRequestIdentity {
  RequestId requestId{0};
  DeviceId deviceId{kInvalidDeviceId};
  E2Operation operation{E2Operation::ProbeDevice};
  Deadline64 deadline{};
  uint64_t submissionToken{0};  // Assigned by the runtime, never the caller.
};

struct E2RuntimeSubmission {
  E2Result result{};
  E2RuntimeRequestIdentity identity{};
};

enum class E2TrackedResultStatus : uint8_t {
  Pending = 0,
  Taken = 1,
  Reclaimed = 2,
  Stale = 3,
};

struct E2RuntimeConfig {
  E2BackendConfig backendConfig{};
  const E2DeviceBinding* bindings{nullptr};
  uint8_t bindingCount{0};
  uint64_t (*nowMs)(void*){nullptr};
  void* clockContext{nullptr};
  void (*publicationHook)(void*, uint64_t){nullptr};
  void* publicationContext{nullptr};
};

explicit E2Runtime(E2Backend& backend);
bool configure(const E2RuntimeConfig&, ErrorDetail&);
bool begin(uint64_t nowMs);
void end(uint64_t nowMs);
E2RuntimeSubmission submit(const E2Command&, uint64_t nowMs);
DeviceModuleStartResult submitDeviceCommand(
    const DeviceCommand&, uint64_t nowMs);
DeviceModuleStartResult cancelDeviceCommand(
    DeviceId, RequestId, uint64_t nowMs, ErrorDetail);
bool takeDeviceResult(
    DeviceId, RequestId, DeviceMeasurementResult&);
bool deviceRequestOutstanding(DeviceId, RequestId) const;
E2TrackedResultStatus takeOrReclaim(
    const E2RuntimeRequestIdentity&, uint64_t nowMs, E2Result&);
bool copyBusStatus(E2Status&) const;
bool copyDeviceStatus(DeviceId, DeviceModuleStatus&) const;
bool applyDeviceEnableState(
    const DeviceEnableState&, uint64_t nowMs);
ServiceHealth serviceHealth(uint64_t nowMs) const;
SystemResourceHealth resourceHealth(uint64_t nowMs) const;
```

`configure()` validates and copies only fixed references/configuration,
attaches the backend/clock/bindings to `E2Task`, and performs no GPIO, module,
queue, or task operation. It is rejected while running. `begin()` requires a
successful configure. The binding array, backend, clock context, and
publication context must outlive the runtime session.

Do not put an EE871-family typed status function in the chip-neutral wrapper.
Prompt 08 adds a product/family publication adapter fed by the worker.

Do not copy I2C's 20 ms assumptions. Use the library's operation bounds and
Prompt 05's owner deadlines.

## Native Test Fixture

Create a test-only line-level EE871 simulator or reuse an exported immutable
library test fixture if one truly exists. Do not copy production protocol into
firmware source.

The fixture must model:

- open-drain line release;
- identity/capability bytes;
- MV3/MV4/status/error code;
- PEC;
- NACK/timeout/stuck/pointer completion;
- clock stretching;
- transaction/line counts.

It is a test double, not another production EE871 implementation.

## Native Tests

Prove:

1. exact dependency version/commit guard;
2. `DeviceKind::Ee871=10` and key;
3. catalog validity and exact `co2_ppm`/`ppm`;
4. project status headers contain no library types;
5. configure/bind/admission perform zero I/O;
6. callback mapping uses release/input and drive-low only;
7. first present initialization;
8. optional absent initialization leaves owner alive/module offline;
9. wrong group/subgroup/CO2 bit maps unsupported identity;
10. partial capability failure never produces ready;
11. non-accepted begin failure stays offline/uninitialized and ordinary
    probe/measure never retries begin;
12. explicit recovery calls begin when uninitialized, calls recover when
    initialized, restores ready, and restarts warm-up;
13. explicit probe is health-neutral, never recovers, and preserves exact
    absence;
14. measurement before warm-up is stale and bus-silent;
15. average selection calls only checked average;
16. fast selection calls only checked fast;
17. only checked valid ppm sets the reading bit;
18. sensor error and range error do not fault bus resource;
19. NACK/timeout/stuck/PEC mappings are exact;
20. offline measurement is bus-silent;
21. `ReadLast` boundary ages and all diagnostic/status copies are bus-silent;
22. completion timestamp is read after simulated 150/300 ms work;
23. queued/active cancellation boundaries;
24. required-budget preflight for initialize/probe/measure/recover;
25. disabled module is line-silent;
26. backend begin/end leave lines released;
27. backend never drives high and uses no internal pull-ups;
28. failed setter/read callbacks preserve the first backend `ErrorDetail` and
    override secondary library timeout/stuck mapping;
29. the runtime reserves terminal capacity before enqueue, rejects overflow
    precisely, and rejects stale completions for reused request IDs;
30. runtime configuration is bus/task-silent, validates all lifetimes/shapes,
    and the same constructor accepts the native fake or production backend;
31. backend initialization retries only at the 5000 ms lifecycle interval;
32. runtime shutdown settles work, has the live worker call owner end, then
    releases both lines before worker join;
33. no static production runtime instance and no product
    row/schema/BoardPins change entered this prompt;
34. every non-Co2Control production environment builds with the E2 gate off,
    no EE871 dependency, and no module/backend/runtime symbol.

## Documentation, Validation, and Handoff

Update relevant dependency, ownership, E2, and module guidelines, without
claiming the module is active in Co2Control before Prompt 08.

Create:

```text
docs/reports/ee871_module_e2_backend_handoff_YYYYMMDD.md
```

Run the full software matrix required by repository guidelines, including:

```powershell
python -m platformio test -e native
python -m platformio run -e tunnelmonitor_wifi
python -m platformio run -e tunnelmonitor_wifi_hil
git diff --check
```

Report physical HIL as not run unless it actually ran with retained evidence.

## Acceptance Criteria

- exact immutable library is pinned;
- generic owner remains chip-neutral;
- EE871 module privately owns one stable driver;
- all module admission/status paths are bus-silent;
- lifecycle begin/end may configure/release GPIO, but only owner-context poll
  performs E2 protocol/device transactions;
- initialization supports optional absence;
- recovery is explicit;
- reusable runtime wrapper is implemented without a second owner path or early
  product instance;
- EE871 dependency and implementation remain absent from all
  non-Co2Control production firmware;
- only checked sample validity enters normalized results;
- warm-up is firmware policy and prevents false-valid data;
- GPIO backend implements real open-drain release/low behavior;
- no Co2Control product/schema/pin mutation occurred early;
- full existing software behavior remains passing.
