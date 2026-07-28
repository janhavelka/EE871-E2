# Prompt 06: Generic Firmware E2 Owner and Contracts

## Role and Scope

You are an AI coding agent starting in:

```text
C:\Users\HonzovoSpectre\Documents\Projects
```

Work only in `TunnelMonitor-node`.

Prompt 05 must have a fully authorized decision report. If it is blocked or
missing, stop. This prompt adds a real chip-neutral E2 owner with native tests,
not a placeholder.

Do not include `EE871/` headers, add `DeviceKind::Ee871`, exact-pin a sensor
library, select a product row, change the sample schema, or edit board pins.
Prompt 07 owns the concrete device/backend and Prompt 08 owns product data.

Prompt 05 has fixed Co2Control as the sole production E2 product. This prompt
may add shared chip-neutral contract declarations, but:

- E2 owner implementation/runtime sources are compiled only in approved
  Co2Control production environments and explicit native/HIL validation
  environments;
- no production runtime is instantiated or started yet;
- every non-Co2Control production environment must continue to build without
  an E2 owner object, task, health row, CLI surface, or EE871 dependency;
- do not edit the TunnelMonitor product profile/composition, pins, schema, or
  Web surface.

## Read First

Read:

- `AGENTS.md`;
- Prompt 05 decision report;
- all applicable `docs/guidelines/` authority;
- `include/TunnelMonitor/contracts/Health.h`;
- `Error.h`, `Event.h`, `FieldBus.h`, `DeviceMeasurement.h`, `Capacities.h`,
  and `All.h`;
- `include/TunnelMonitor/settings/DeviceSettings.h`;
- `include/TunnelMonitor/i2c/I2cTask.h`;
- `I2cDeviceBinding.h`, `I2cOwnerTransport.h`, `I2cDiagnostics.h`,
  `I2cBackend.h`;
- matching I2C source/native tests;
- the RS485 owner/binding for a second ownership pattern;
- runtime/task wrapper patterns;
- selected health/event policy and string mapping code.

Record branch/commit/dirty state and the full required baseline matrix.

Before adding a file or class, inspect whether a small shared bus-neutral
helper already exists. Reuse it only when doing so simplifies both owners
without changing frozen I2C/RS485 behavior. Do not create a generic bus
registry, service locator, virtual device hierarchy, or dynamic owner graph.

## Stable Contract Values

Append explicit values:

```cpp
// Health.h
ServiceId::E2Task = 21
BusId::E2 = 6

// Error.h
ErrorDomain::E2 = 15

// Event.h
EventComponent::E2 = 24
```

Do not renumber existing values.

Allocate:

```cpp
E2Nack = 1400,
E2Timeout = 1401,
E2BusStuck = 1402,
E2PecMismatch = 1403,
E2UnsupportedIdentity = 1404,
E2SensorError = 1405,
E2ValueOutOfRange = 1406,
E2DriverOffline = 1407,
E2WriteVerifyMismatch = 1408,
E2PersistentStateUncertain = 1409,
E2RecoveryFailed = 1410,
E2ProtocolError = 1411,
```

in a new E2-owned `1400..1499` `ErrorCode` band. Do not place E2 errors in
`500..599`; that existing range belongs to I2C and I2C-attached device errors.
Extend the contract range test with
`errorCodeInRange(ErrorCode::E2Nack, 1400, 1499)` and the last allocated E2
code.

Allocate:

```cpp
E2_TIMEOUT = 3050,
E2_RECOVERY_START = 3051,
E2_RECOVERY_OK = 3052,
E2_RECOVERY_FAIL = 3053,
E2_QUEUE_OVERFLOW = 3054,
E2_DEVICE_ABSENT = 3055,
E2_DEVICE_RETURNED = 3056,
E2_PEC_ERROR = 3057,
E2_SENSOR_ERROR = 3058,
E2_PERSISTENT_STATE_UNCERTAIN = 3059,
```

in `EventCode`.

Add exhaustive string/range/policy mappings and compile-time tests wherever
current contracts require them.

Use the stable external key `"e2"` for the new service, bus, error domain, and
event component mappings. Do not let separate surfaces invent spelling.

`SystemResource::E2Bus=12` already exists. Do not add another value. It remains
without a production health row until the selected runtime in Prompt 08
actually starts the owner.

## Capacities

Add:

```cpp
inline constexpr uint8_t kE2RequestQueueDepth = 8;
inline constexpr uint8_t kE2ResultQueueDepth = 8;
inline constexpr uint8_t kE2EventTraceCapacity = 8;
```

Every E2 command/result/queue type must be standard-layout and trivially
copyable where the analogous existing contracts require that property.

Do not increase unrelated production capacities yet.

## E2 Command Contract

Create:

```text
include/TunnelMonitor/contracts/E2.h
```

and include it from the normal contract umbrella.

Use:

```cpp
enum class E2Operation : uint8_t {
  ProbeDevice = 0,
  RecoverDevice = 1,
};

enum class E2ResultStatus : uint8_t {
  Ok = 0,
  Queued = 1,
  Timeout = 2,
  Nack = 3,
  BusStuck = 4,
  DeviceAbsent = 5,
  DriverOffline = 6,
  RecoveryStarted = 7,
  RecoveryFailed = 8,
  Cancelled = 9,
  Failed = 10,
};

struct E2Command {
  RequestId requestId{0};
  DeviceId deviceId{};
  E2Operation operation{E2Operation::ProbeDevice};
  Deadline64 deadline{};
};

struct E2Result {
  RequestId requestId{0};
  DeviceId deviceId{};
  E2Operation operation{E2Operation::ProbeDevice};
  E2ResultStatus status{E2ResultStatus::Failed};
  ErrorDetail error{};
  uint32_t recoveryCount{0};
  uint64_t completedUptimeMs{0};
};
```

Validate exact enum ranges, identity, active deadlines, and final-status
helpers. The admin contract intentionally has no arbitrary payload, priority,
or public submission token. Probe and recovery need no bytes, and diagnostics
are read from publication caches through `E2Task::status()` and
`copyDeviceStatus()`.

No GPIO or third-party library types may enter this contract.

## Backend Boundary

Create:

```text
include/TunnelMonitor/e2/E2Backend.h
```

Use the same small non-virtual operation-table seam as the permanent I2C
backend:

```cpp
struct E2BackendConfig {
  int8_t clockGpio{-1};
  int8_t dataGpio{-1};
  bool internalPullups{false};
};

struct E2BackendOperations {
  bool (*begin)(void*, const E2BackendConfig&, ErrorDetail&){nullptr};
  void (*end)(void*){nullptr};
  uint32_t (*requiredRestoreBudgetMs)(const void*){nullptr};
  bool (*setClockReleased)(void*, bool, ErrorDetail&){nullptr};
  bool (*setDataReleased)(void*, bool, ErrorDetail&){nullptr};
  bool (*readClock)(void*, bool&, ErrorDetail&){nullptr};
  bool (*readData)(void*, bool&, ErrorDetail&){nullptr};
  void (*delayUs)(void*, uint32_t){nullptr};
  void (*delayMs)(void*, uint32_t){nullptr};
  void (*cooperativeYield)(void*){nullptr};
};

class E2Backend {
 protected:
  constexpr E2Backend(void* context, E2BackendOperations operations);

 public:
  bool begin(const E2BackendConfig&, ErrorDetail&);
  void end();
  uint32_t requiredRestoreBudgetMs() const;
  bool setClockReleased(bool, ErrorDetail&);
  bool setDataReleased(bool, ErrorDetail&);
  bool readClock(bool& high, ErrorDetail&);
  bool readData(bool& high, ErrorDetail&);
  void delayUs(uint32_t);
  void delayMs(uint32_t);
  void cooperativeYield();

 private:
  void* const context_;
  const E2BackendOperations operations_;
};
```

Provide a constexpr operations-shape validator. `E2Task` is the attached
backend's sole logical caller, but `attach(E2Backend*)` is non-owning: backend
storage must outlive the attachment/session. No other runtime component may
call it. Do not add a virtual base, RTTI, heap ownership, or a device plugin
system.

The backend contract says:

- `released=true` means high-impedance/open-drain release;
- `released=false` means drive low;
- backend methods are task-context, bounded, and non-recursive;
- Prompt 07's module latches the first callback failure for the current
  bounded library call; the owner then retains the resulting project
  `ErrorDetail` and performs bus invalidation policy;
- `requiredRestoreBudgetMs()` is cache-only and conservatively covers one
  backend end/begin restoration; the owner adds it to a recovery preflight
  only while backend fault is latched;
- backend calls perform no retry.

## Owner Transport

Create:

```text
include/TunnelMonitor/e2/E2OwnerTransport.h
```

Use a fixed non-owning function table:

```cpp
struct E2OwnerTransport {
  void* context{nullptr};
  bool (*setClockReleased)(void*, bool, ErrorDetail&){nullptr};
  bool (*setDataReleased)(void*, bool, ErrorDetail&){nullptr};
  bool (*readClock)(void*, bool& high, ErrorDetail&){nullptr};
  bool (*readData)(void*, bool& high, ErrorDetail&){nullptr};
  void (*delayUs)(void*, uint32_t){nullptr};
  void (*delayMs)(void*, uint32_t){nullptr};
  void (*cooperativeYield)(void*){nullptr};
  uint64_t (*nowMs)(void*){nullptr};
};
```

Add a constexpr shape validator. Transport callbacks may be called only while
one binding operation is active in the E2 owner context.

Do not collapse a failed line read into a logical low. The transport reports
success separately from the sampled level. Prompt 07's EE871 adapter must
clear a first-callback-error latch before every library call, latch the first
transport failure, return the safest protocol fallback value required by the
third-party callback signature, and prefer the latched project/backend error
over a secondary library timeout or bus-stuck result after the call.

Unlike I2C, this transport exposes line primitives because the device library
owns the E2 bit protocol. It must not expose ESP-IDF GPIO handles in shared
contracts.

## Device Binding

Create:

```text
include/TunnelMonitor/e2/E2DeviceBinding.h
```

Mirror the small fixed I2C binding shape, not its I2C transfer semantics:

```cpp
struct E2DeviceBinding {
  DeviceId deviceId{};
  void* module{nullptr};
  bool (*bind)(void*, const E2OwnerTransport&, ErrorDetail&){nullptr};
  DeviceModuleStartResult (*start)(
      void*, const DeviceCommand&, uint64_t){nullptr};
  DeviceModulePollState (*poll)(void*, uint64_t){nullptr};
  bool (*takeResult)(void*, DeviceMeasurementResult&){nullptr};
  DeviceModuleStartResult (*startCommand)(
      void*, const E2Command&, uint64_t){nullptr};
  bool (*takeCommandResult)(void*, E2Result&){nullptr};
  uint32_t (*requiredDeviceBudgetMs)(
      const void*, DeviceRequestKind){nullptr};
  uint32_t (*requiredCommandBudgetMs)(
      const void*, E2Operation){nullptr};
  void (*setEnabled)(void*, bool, uint64_t){nullptr};
  void (*cancel)(void*, uint64_t, ErrorDetail){nullptr};
  void (*onBusInvalidated)(void*, uint64_t, ErrorDetail){nullptr};
  DeviceModuleStatus (*snapshot)(const void*){nullptr};
  const DeviceReadingCatalog* (*readingCatalog)(const void*){nullptr};
};
```

Provide a type-safe `makeE2DeviceBinding` template with compile-time shape
checks. Both budget callbacks are cache-only and let the owner reject work that
cannot finish inside its immutable deadline before any I/O. Keep the
measurement and admin callbacks separate so callers never manufacture an
irrelevant dummy enum value.

Do not add device-family typed copy functions to the generic binding in this
prompt.

## E2 Owner

Create:

```text
include/TunnelMonitor/e2/E2Task.h
include/TunnelMonitor/e2/E2Diagnostics.h
src/e2/E2Task.cpp
src/e2/E2Diagnostics.cpp
```

`E2Task` is the sole logical owner. Follow existing fixed queue/result
reservation patterns.

Use one private fixed tagged FIFO, not independently prioritized admin and
measurement queues:

```cpp
enum class E2WorkKind : uint8_t {
  DEVICE = 0,
  ADMIN = 1,
};

struct E2PendingWork {
  E2WorkKind kind{E2WorkKind::DEVICE};
  DeviceCommand device{};
  E2Command admin{};
};
```

Only the field selected by `kind` is interpreted. Validate the unused field is
ignored; do not compare its default identity. The eight-slot capacity is
shared by both work kinds.

Required public shape:

```cpp
class E2Task {
 public:
  void reset();
  bool configure(const E2BackendConfig& config);
  void attach(E2Backend* backend);
  void attachBindings(const E2DeviceBinding* bindings, uint8_t count);
  void attachClock(uint64_t (*nowMs)(void*), void* context);
  bool begin(uint64_t nowMs);
  void end();
  void poll(uint64_t nowMs);
  bool applyDeviceEnableState(
      const DeviceEnableState&, uint64_t nowMs);
  bool deviceRequestOutstanding(
      DeviceId, RequestId) const;

  E2Result validateSubmission(const E2Command&, uint64_t nowMs) const;
  E2Result submit(const E2Command&, uint64_t nowMs);
  DeviceModuleStartResult submitDeviceCommand(
      const DeviceCommand&, uint64_t nowMs);
  DeviceModuleStartResult cancelDeviceCommand(
      DeviceId, RequestId, uint64_t nowMs, ErrorDetail reason);

  bool readResult(E2Result&);
  bool readResultFor(
      RequestId, DeviceId, E2Operation, E2Result&);
  bool takeDeviceResult(
      DeviceId, RequestId, DeviceMeasurementResult&);
  bool readEvent(E2EventRecord&);
  E2Status status() const;
  bool copyDeviceStatus(DeviceId, DeviceModuleStatus&) const;
  ServiceHealth serviceHealth(uint64_t nowMs) const;
  SystemResourceHealth resourceHealth(uint64_t nowMs) const;
};
```

Equivalent naming is allowed only to align exactly with current permanent owner
contracts.

## Owner Behavior

- `attach`, binding `bind`, request validation, and request admission perform
  no line I/O.
- `begin` initializes the backend and binds modules in owner context.
- failed `begin` releases the backend, retains validated configuration and
  bindings, leaves the owner stopped/retryable, and admits no work;
- `end` deterministically cancels/terminalizes retained queued/active work,
  disables bound modules, calls backend end/release in owner context, and
  leaves the owner stopped; it is idempotent.
- One active binding operation exists at a time.
- Application/admin commands and measurement commands share the same fixed
  result-capacity reservation discipline.
- Retained terminal results are consume-once and exact
  request/device/deadline identities are preserved.
- A private monotonically changing admission token distinguishes reused
  request IDs internally; do not expose it as an application-supplied field.
- Queued and active cancellation is exact.
- Before starting physical work, reject an expired deadline.
- Before each module poll that may perform a synchronous call, compare
  remaining `Deadline64` budget with the applicable device/admin budget
  callback. For recovery while a backend fault is latched, add the backend's
  restoration bound using checked/saturating arithmetic.
- Once a bounded library call starts, allow it to finish; cancellation cannot
  preempt E2 bit signaling.
- Observe completion time from the attached owner clock after the synchronous
  call, not the stale `poll(nowMs)` argument.
- Backend failures are precise and bus invalidation is delivered to bindings.
- Recovery is explicit and retry-free in the generic owner. It executes one
  admitted `RecoverDevice` request and reports one exact terminal result. If a
  runtime backend fault is latched, that request first performs one
  backend `end()`/`begin()` restoration attempt using the stored validated
  config; only after backend restoration does it invoke the target binding's
  recovery. Failure at either phase is terminal and precise.
  Preserve `E2Nack`, `E2Timeout`, `E2BusStuck`, `E2PecMismatch`, or the precise
  backend error when known; use `E2RecoveryFailed` only when the recovery
  procedure itself fails without a more specific released cause.
  Product/runtime code owns device recovery due time and backoff. The runtime
  worker's backend-initialization retry is a separate lifecycle policy.
- Use one fixed FIFO admission order across measurement and admin work. Do not
  add a priority field or hidden scheduling class.
- Queue overflow, absence/return, timeout, PEC, recovery, sensor, and mutation
  events use the allocated codes with bounded edge-triggered policy.
- Sensor-domain device failures do not become E2 bus-resource failures.
- Disabled modules are visible but bus-silent.

The owner interprets only generic queue/deadline/recovery/result behavior. It
contains no EE871 command nibble, custom address, CO2 status bit, or library
error enum.

## Diagnostics

Define fixed `E2Status` and `E2EventRecord` with:

- initialized/backend-ready;
- queue/result/event depths and saturated overflow counters;
- active request/device/kind/deadline;
- last completion/error times;
- precise last `ErrorDetail`;
- success/error/timeout/recovery counters;
- backend-fault-latched state;
- owner heartbeat.

All copy/status APIs are hardware-free.

`E2Diagnostics` in this prompt contains chip-neutral types, names, and owner
helpers only. Do not create a production static backend, worker task, product
binding, or global runtime facade here; Prompts 07-08 own those layers.

## Native Tests

Use a fake backend and two fake bound modules. Prove:

1. stable enum/error/event numeric values;
2. contract standard-layout/trivial-copy guarantees;
3. invalid binding/config rejection;
4. attach/configure/admission/status is line-I/O-free and binding itself does
   not signal E2 lines;
5. only owner-context `begin()`, `end()`, and `poll()` call the backend/module;
   lifecycle begin/end may configure or release lines, while only `poll()`
   performs E2 protocol/device transactions;
6. queue depth eight and one deterministic FIFO policy;
7. queue overflow produces exact result/counter/event;
8. result reservation prevents accepted work without terminal capacity;
9. terminal result retained and consumed exactly once;
10. wrong request/device identity cannot consume a result;
11. immutable deadline preserved end to end;
12. expired-before-I/O performs no line activity;
13. insufficient required budget rejects before I/O;
14. completion time is re-read after a long synchronous fake call;
15. queued cancellation;
16. active cancellation before next physical call;
17. cancellation cannot pretend to preempt an already-running call;
18. backend failure invalidates bindings once;
19. one explicit recovery request performs at most one latched-backend
    restoration and one device recovery, with exact phase failure and no owner
    retry/backoff;
20. absence and return edge events;
21. device sensor-domain error leaves bus resource healthy;
22. disabled binding is bus-silent;
23. two bindings never operate concurrently;
24. all status/health/event copies are hardware-free;
25. enable-state application and outstanding-query semantics match existing
    measurement ownership contracts;
26. `end()` terminalizes accepted work once, is idempotent, and releases the
    backend;
27. failed line setters and failed reads preserve their backend
    `ErrorDetail`; a failed read is not mistaken for a valid low;
28. no EE871/third-party include or protocol constant exists in generic owner
    files;
29. approved native validation compiles and exercises the owner, while every
    existing non-Co2Control production environment has the Prompt 05 E2 gate
    disabled and contains no E2 owner/runtime/health/operator symbol.

## Guidelines and Report

Update applicable architecture documents to describe the now-real generic E2
owner while keeping its sole production activation deferred to the Co2Control
integration in Prompt 08. Add the Prompt 05 compile gate with value `0` to
every existing non-Co2Control production environment and value `1` only to the
explicit native validation environment. Do not add a Co2Control production
environment speculatively in this prompt.

Create:

```text
docs/reports/e2_generic_owner_handoff_YYYYMMDD.md
```

Include files, enum allocations, owner state/queue diagrams, memory sizes,
deadline policy, tests/builds, and explicit omissions.

## Validation

Run all repository contract/native/profile/Web checks required by `AGENTS.md`,
at minimum:

```powershell
python -m platformio test -e native
python -m platformio run -e tunnelmonitor_wifi
python -m platformio run -e tunnelmonitor_wifi_hil
git diff --check
```

Do not claim hardware HIL.

## Acceptance Criteria

- E2 has one real chip-neutral owner;
- fixed commands, results, binding, transport, backend, diagnostics, and tests
  exist;
- bind/admission/status are bus-silent;
- deadlines and terminal-result lifetime are exact;
- recovery is explicit and retry/backoff remains product-owned;
- no device protocol or product schema entered the owner;
- no non-Co2Control product enables or instantiates the owner;
- no generic registry/framework was added;
- existing I2C/RS485 behavior and full software matrix remain unchanged.
