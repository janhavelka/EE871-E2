# EE871-E2 Suitability Audit for TunnelMonitor-node

> Point-in-time audit of EE871-E2 commit `b5588be`. The later
> pioarduino-platform alignment and COM20 HIL runs add compatibility and bench
> evidence; they do not, by themselves, implement or close the P0 API and
> integration findings recorded here.

Date: 2026-07-28
Audit mode: cross-repository, report only
Hardware used during this audit: none

## Executive Verdict

The EE871-E2 library has the correct basic architecture for use behind a
dedicated TunnelMonitor E2 owner worker:

- GPIO-style E2 line access is injected;
- the core is framework-neutral;
- calls are synchronous and bounded by configuration;
- the library does not own pins, an RTOS task, logging, queues, or application
  policy;
- the instance is explicitly non-thread-safe and therefore naturally fits a
  single-owner worker.

However, **EE871-E2 v1.0.0 is not yet suitable for unattended production use in
TunnelMonitor-node**. It does not include all of the worker-facing lifecycle,
recovery, measurement-validation, and write-timing behavior required for a
field node.

The production recommendation is:

> **NO-GO for pinning v1.0.0 into production TunnelMonitor firmware.**
>
> **GO as the protocol-library foundation after the P0 library findings in this
> report are fixed, covered by native fault tests, and qualified with the
> intended E2 electrical interface and real EE871 hardware.**

TunnelMonitor-node also does not currently contain an E2 worker or an EE871
device integration. Its current architecture intentionally reserves
`SystemResource::E2Bus` as declaration-only future work. Adding EE871 to the
current TunnelMonitor product is therefore a deliberate product/profile
extension, not just a dependency change.

## Overall Scorecard

| Area | Rating | TunnelMonitor conclusion |
| --- | --- | --- |
| Core ownership boundary | Strong | Good fit for a separate single-owner E2 worker. |
| Framework neutrality | Strong | No Arduino, ESP-IDF, FreeRTOS, or hardware-I2C dependency in the core. |
| E2 framing and PEC | Good | Core read/write framing and PEC formulas are present. |
| Basic boundedness | Good with gaps | Bit/byte waits are bounded, but safe upper bounds and operation WCET contracts are incomplete. |
| Device lifecycle | Blocking gap | No accepted absent-at-boot state; `recover()` cannot be used after failed `begin()`. |
| OFFLINE behavior | Blocking gap | Normal reads/writes can touch the bus and silently move `OFFLINE` to `READY`. |
| Identity validation | Blocking gap | Startup/probe/recovery validate group only, not subgroup and CO2 capability. |
| CO2 sample semantics | Blocking gap | Only raw MV3/MV4 reads exist; no checked value/status/error/range procedure. |
| Write and pointer timing | Blocking gap | Legal long write behavior and `0x50` pointer completion are not handled correctly. |
| Cooperative worker behavior | Weak | Long waits busy-call `delayUs`; there is no millisecond wait/yield callback. |
| Persistent-write diagnostics | Partial | Multi-byte dirty handling exists, but accepted single-byte write uncertainty is not complete. |
| Native fault coverage | Partial | 31 tests pass, but the TunnelMonitor-critical cases are missing. |
| TunnelMonitor integration | Not implemented | No E2 worker, E2 pins, EE871 device kind, schema field, dependency, or health projection exists. |

## Audit Baselines

### EE871-E2

- Repository: `EE871-E2`
- Branch: `main`
- Commit: `b5588be0f2c6320069097f6e72ad3173929f829b`
- Package version: `1.0.0` from `library.json`
- Core source reviewed:
  - `include/EE871/`
  - `src/EE871.cpp`
  - `test/`
  - `examples/`
  - current protocol, hardening, HIL, and prior audit documents

### TunnelMonitor-node

- Repository: sibling `../TunnelMonitor-node`
- Branch: `prompt-45-platformization`
- Commit: `2412dc3fef8766a4175e2b622e6ec2e3de967a73`
- Relevant areas reviewed:
  - I2C owner, worker wrapper, binding, and fake patterns
  - measurement runtime and generic device contracts
  - selected product profile and sample schema
  - health, error, event, settings, CLI, web, storage, and Cloud boundaries
  - authoritative architecture documents under `docs/guidelines/`

Unrelated existing dirty files in TunnelMonitor-node were treated as user-owned
and were not modified.

### Validation Run

The following EE871 checks passed in this checkout:

```text
python tools/check_core_timing_guard.py
python tools/check_cli_contract.py
python tools/check_idf_example_contract.py
python scripts/generate_version.py check
python -m platformio test -e native
python -m platformio run -e ex_bringup_s3
python -m platformio run -e ex_bringup_s2
```

Results:

- Core timing guard: PASS
- Arduino CLI contract: PASS
- ESP-IDF example contract: PASS
- Generated version check: PASS
- Native tests: PASS, 31/31
- Arduino ESP32-S3 example build: SUCCESS
- Arduino ESP32-S2 example build: SUCCESS

These results prove the current tested contracts and build surfaces. They do
not cover the missing lifecycle, checked-sample, long-write, worker-deadline, or
TunnelMonitor integration behavior below.

No EE871 or TunnelMonitor hardware HIL was run during this audit.

### Relationship to existing EE871 audit material

`docs/EE871_CO2_LIBRARY_PRODUCTION_AUDIT_2026-07-01.md` already identified
several of the core protocol and state-machine problems. This audit revalidated
the TunnelMonitor-relevant findings against the current code and added the
cross-repository worker/profile/schema analysis.

`docs/prompts/ee871_tunnelmonitor_fit_prompt_20260627.md` is a detailed proposed
implementation specification, not implemented functionality. The current
public headers still contain no `BeginPolicy`, `Co2ReadResult`,
`readCo2AverageSample()`, `readCo2FastSample()`, millisecond delay callback, or
yield callback.

## What the Library Already Provides

The following library capabilities are appropriate and should be retained.

### Clean owner/transport boundary

`Config` accepts non-owning callbacks for SCL/SDA set/read and microsecond
delay, plus one user pointer (`include/EE871/Config.h:10-75`). The public class
documents one-owner/external serialization, blocking calls, and no ISR safety
(`include/EE871/EE871.h:46-62`).

This is the correct boundary for an `E2Task`:

```text
TunnelMonitor E2Task
  -> Ee871Module
    -> EE871::EE871
      -> injected E2 GPIO/timing callbacks
```

The core library should not acquire FreeRTOS, TunnelMonitor contracts, a queue,
or board pins.

### Core E2 protocol implementation

The library provides:

- control-byte construction and supported EE871 main-command checks
  (`include/EE871/CommandTable.h:23-87`);
- group/subgroup/CO2 capability constants
  (`include/EE871/CommandTable.h:90-103`);
- low-byte-first MV3/MV4 reads (`src/EE871.cpp:659-705`);
- read and write PEC calculation (`src/EE871.cpp:181-187`);
- synchronous read/write transactions with ACK/NACK and clock-stretch checks
  (`src/EE871.cpp:1071-1229`);
- raw status and error-code access (`include/EE871/EE871.h:510-535`);
- custom memory and typed configuration helpers
  (`include/EE871/EE871.h:224-504`).

### Useful diagnostics and persistent-state support

The library has:

- `READY`, `DEGRADED`, and `OFFLINE` health state;
- success/failure counters and last-error timestamps;
- raw diagnostic `probe()`;
- `busReset()` and `checkBusIdle()`;
- persistent dirty diagnostics and an explicit resync path;
- readback verification for writes;
- no heap allocation or dynamic containers in the core.

These are useful subordinate diagnostics for a TunnelMonitor module. The
firmware should still publish its own 64-bit request completion, presence,
sample validity, and worker heartbeat data.

## Required Capability Matrix

| Capability needed by a TunnelMonitor E2 owner | Current EE871 v1.0.0 | Classification |
| --- | --- | --- |
| One stable, externally owned driver instance | Present | Library complete |
| Injected open-drain GPIO and timing HAL | Present | Library complete |
| No task, pin, queue, logging, or scheduler ownership | Present | Correct library boundary |
| Synchronous, bounded transaction calls | Present, but bounds need hardening/documentation | Library gap |
| Start successfully when an optional sensor is absent | Missing | P0 library gap |
| Keep absence diagnostics while initialized `OFFLINE` | Missing | P0 library gap |
| Fast-fail normal calls while `OFFLINE` until explicit recovery | Missing | P0 library gap |
| Recover and reload identity/capabilities after replug | Incomplete | P0 library gap |
| Validate group, subgroup, and advertised CO2 capability | Incomplete | P0 library gap |
| Raw MV3 and MV4 reads | Present | Library complete |
| Checked value-first/status-second CO2 sample procedure | Missing | P0 library gap |
| Preserve CO2 sensor error separately from E2 transport health | Missing | P0 library gap |
| Enforce an absolute supported CO2 range in checked reads | Missing | P0 library gap |
| Handle legal `0x10`/`0x50` write completion/stretch timing | Incomplete | P0 library gap |
| Cooperative long millisecond waits for an RTOS worker | Missing | P0 library gap |
| Complete uncertainty/dirty handling for persistent maintenance | Partial | P1 library gap |
| Warm-up, freshness, trigger cadence, and retry policy | Intentionally absent | TunnelMonitor responsibility |
| Static RTOS queue, command/result lifetime, cancellation | Intentionally absent | TunnelMonitor responsibility |
| Board pins, level shifter, and pull-ups | Intentionally absent | Board/firmware responsibility |
| Device profile, reading schema, CSV/Cloud/UI exposure | Intentionally absent | TunnelMonitor responsibility |

## P0 Library Findings

P0 means the finding should be closed before a production TunnelMonitor E2
integration is accepted.

### P0-1: Optional absent-at-boot operation is missing

#### Evidence

- `Config` has no startup-presence policy
  (`include/EE871/Config.h:49-75`).
- `begin()` returns on the first identity transfer failure, resets the object to
  `UNINIT`, and leaves no accepted offline session
  (`src/EE871.cpp:197-319`).
- Both `probe()` and `recover()` require `_initialized == true`
  (`src/EE871.cpp:383-412`).

#### TunnelMonitor impact

An optional, disconnected, unpowered, or later-hotplugged EE871 cannot enter a
stable owner-managed offline state at boot. The worker would have to repeatedly
call `begin()` from scratch and maintain all absence/retry state outside the
library. That is possible as a workaround, but it defeats the library health
state and makes recovery behavior differ between boot absence and runtime
failure.

#### Required library behavior

Add an explicit startup policy:

```cpp
enum class BeginPolicy : uint8_t {
  RequirePresent = 0,
  AllowAbsent = 1,
};
```

`RequirePresent` should preserve strict current behavior. `AllowAbsent` should:

- continue rejecting invalid configuration and a physically stuck bus;
- accept only a real presence/transport failure;
- reject a responding but incompatible identity or missing CO2 capability;
- return OK with `isInitialized() == true`;
- enter `DriverState::OFFLINE`;
- retain the original startup probe error in a cache-only diagnostic;
- leave all feature flags unknown/zero;
- allow later explicit `recover()`.

A successful recovery after accepted absence must validate full identity, load
features, and transition to `READY`.

The repository already contains a detailed unimplemented design for this at
`docs/prompts/ee871_tunnelmonitor_fit_prompt_20260627.md:59-143`.

### P0-2: `OFFLINE` is not latched for normal operations

#### Evidence

- Normal tracked wrappers always execute the raw transfer
  (`src/EE871.cpp:1132-1135`, `1225-1229`).
- Any successful tracked transfer clears failures and sets `READY`
  (`src/EE871.cpp:1239-1246`).
- Public normal calls such as `readStatus()` do not have an offline pre-bus
  guard (`src/EE871.cpp:466-478`, `685-686`).

#### TunnelMonitor impact

The E2 owner cannot enforce its recovery cadence and backoff. A normal
measurement request can touch a known-offline bus and silently recover the
driver. This makes owner-level queue, recovery-event, and health transitions
non-deterministic.

A failed multi-read recovery also has an unsafe state interaction: a successful
early tracked byte can clear the failure count before a later byte fails, so a
failed recovery is not guaranteed to leave the driver latched `OFFLINE`.

#### Required library behavior

When initialized and `OFFLINE`, normal tracked operations should return a
precise cache-only failure such as:

```cpp
Status::Error(Err::BUSY, "Driver is offline; call recover()")
```

The return must occur before touching SCL or SDA. Only explicit diagnostic and
recovery paths should bypass the guard:

- `probe()`;
- `recover()`;
- `busReset()`;
- `checkBusIdle()`;
- cache-only accessors.

Failed recovery from `OFFLINE` must reassert `OFFLINE` and retain the original
precise recovery failure.

### P0-3: Startup, probe, and recovery do not prove EE871 CO2 identity

#### Evidence

- `begin()` validates only group `0x0367`
  (`src/EE871.cpp:271-292`).
- `probe()` also validates only group (`src/EE871.cpp:388-405`).
- `recover()` performs only `readGroup()` (`src/EE871.cpp:409-424`).
- Subgroup `0x09` and available-CO2 bit `0x08` are defined but not required in
  those paths (`include/EE871/CommandTable.h:94-96`).
- A wrong responding identity is reported as `DEVICE_NOT_FOUND`
  (`src/EE871.cpp:287-291`, `402-405`, `659-677`).
- Feature reads at startup are non-fatal. A partial read can also leave a
  partially populated cache despite the comment saying features remain
  disabled (`src/EE871.cpp:294-315`).
- `recover()` does not reload feature flags after replug or sensor replacement.
- `recover()` ignores a failed `busReset()` and can replace a proven
  `BUS_STUCK` with a later NACK or timeout (`src/EE871.cpp:414-420`).

#### TunnelMonitor impact

The worker can report a device as ready when:

- it is the wrong E2 device;
- it has the wrong subgroup;
- it does not advertise CO2;
- its capabilities are unknown after a failed feature read;
- a replacement device has different capabilities.

Wrong identity is also indistinguishable from definite absence when mapped to
`DEVICE_NOT_FOUND`.

#### Required library behavior

Use one shared identity procedure in `begin()`, `probe()`, and `recover()`:

1. read group low then high and require `0x0367`;
2. read subgroup and require `0x09`;
3. read available measurements and require the CO2 bit;
4. preserve NACK, timeout, bus-stuck, and PEC errors;
5. return `NOT_SUPPORTED` for a responding incompatible device;
6. load feature flags into temporary values and publish them atomically only
   after every required feature read succeeds.

Recovery should return a proven bus-reset failure immediately. On successful
recovery it should reload the capability cache before declaring `READY`.

### P0-4: No checked CO2 sample procedure exists

#### Evidence

- Public CO2 APIs are only raw `readCo2Fast()` and `readCo2Average()`
  (`include/EE871/EE871.h:527-535`).
- Both implementations only read MV3/MV4 low then high and return transport
  success (`src/EE871.cpp:699-705`).
- `Err` has no CO2 sensor-domain error
  (`include/EE871/Status.h:14-30`).
- No result preserves the validity and failure of the value, status, and error
  code separately.
- README quick start prints a raw averaged value every second without a
  warm-up/status/range policy (`README.md:143-152`).

#### Required procedure

Keep the raw APIs, but add checked average and fast helpers returning structured
diagnostics. The procedure must:

1. read MV4 or MV3 low/high first;
2. stop immediately if the value transport read fails;
3. read status second;
4. treat the status read as explicitly side-effecting;
5. if CO2 status bit 3 is set, read error code when advertised;
6. return a distinct sensor-domain error without incrementing E2 transport
   health when all bus I/O succeeded;
7. validate the value against the library's documented maximum supported EE871
   range;
8. retain the raw value and component statuses for diagnostics even when the
   sample is invalid.

Recommended source-compatible additions:

```cpp
enum class Co2ValueKind : uint8_t {
  Fast = 0,
  Average = 1,
};

struct Co2ReadResult {
  Co2ValueKind kind{Co2ValueKind::Average};
  uint16_t ppm{0};
  bool ppmValid{false};
  uint8_t statusByte{0};
  bool statusValid{false};
  bool co2Error{false};
  uint8_t errorCode{0};
  bool errorCodeValid{false};
  Status valueReadStatus{Status::Ok()};
  Status statusReadStatus{Status::Ok()};
  Status errorCodeReadStatus{Status::Ok()};
};

Status readCo2AverageSample(Co2ReadResult& out);
Status readCo2FastSample(Co2ReadResult& out);
```

Append a sensor-domain `CO2_SENSOR_ERROR` to `Err` without renumbering existing
values. Add explicit CO2 range constants, including a maximum of 50,000 ppm for
the broad library guard. Product-specific expected range and plausibility
remain TunnelMonitor policy.

This checked helper should not add a scheduler, sample cache, timestamps,
freshness inference, or warm-up policy.

### P0-5: Legal write/pointer completion timing is not handled correctly

#### Evidence

- Normal per-bit/per-byte stretch defaults are 25 ms/35 ms
  (`include/EE871/Config.h:67-71`).
- `_writeCommandRaw()` uses those same ordinary limits through final ACK and
  STOP (`src/EE871.cpp:1137-1222`).
- The configured 150 ms or 300 ms sleep occurs only after a successful write
  transaction/STOP (`src/EE871.cpp:589`, `635`).
- `setCustomPointer()` documents waiting for the configured write delay, but its
  implementation returns immediately after `0x50`
  (`include/EE871/EE871.h:241-244`, `src/EE871.cpp:507-519`).
- `customRead()` immediately begins `0x51` after the pointer write
  (`src/EE871.cpp:525-547`).
- Startup feature discovery also writes pointer `0x07` and immediately reads
  (`src/EE871.cpp:300-312`).

The EE871 documentation permits up to 150 ms for `0x10`/`0x50` write behavior
and up to 300 ms for the measurement interval pair. Communication during flash
work can hold SCL low.

#### TunnelMonitor impact

A valid device can be reported as timed out/degraded/offline during a legal
write. Pointer reads can begin before pointer completion. This affects not only
maintenance writes, but also ordinary custom reads, feature discovery, error
code reads, and initialization.

#### Required library behavior

- Add a write-specific bounded completion/stretch deadline rather than using
  only normal read byte timing.
- Apply it to both `0x10` and `0x50`.
- Use the interval-pair deadline for the committing interval write.
- Make `setCustomPointer()` and startup feature discovery use the same pointer
  helper.
- Do not start `0x51` until pointer completion is valid.
- Add fake tests for legal just-under-deadline and illegal
  just-over-deadline SCL holds.

The implementation should remain synchronous and simple.

### P0-6: Long waits are not cooperative enough for a production owner worker

#### Evidence

`sleepMs()` loops once per millisecond and calls `delayUs(1000)` every time
(`src/EE871.cpp:189-193`). `Config` has no millisecond delay or cooperative
yield callback.

At defaults, ordinary write paths can occupy the worker for 150-300 ms. A
multi-byte maintenance operation can occupy it much longer. Configuration
currently permits up to 5,000 ms of post-write delay per write.

#### Required library behavior

Add optional long-delay callbacks:

```cpp
using E2DelayMsFn = void (*)(uint32_t ms, void* user);
using E2YieldFn = void (*)(void* user);
```

Use bounded slices for millisecond write-delay paths:

- normalize slice zero to 1 ms;
- impose a small safe maximum slice;
- use `delayMs` when supplied, otherwise fall back to `delayUs`;
- call `yield` after each long-delay slice;
- never call `yield` in E2 bit-level timing.

The TunnelMonitor adapter can map these callbacks to a task-context bounded wait
and cooperative yield while preserving non-yielding microsecond signaling.

## P1 Library Findings

### P1-1: Safe configuration limits and public-operation WCET are incomplete

The implementation checks minimum high/low timing but not the 500 Hz lower bus
frequency bound (`src/EE871.cpp:213-224`). It does not impose production-safe
upper bounds on bit/byte timeouts. Byte elapsed accounting is enforced mainly
while SCL is low, not as a complete measured wall-time operation deadline
(`src/EE871.cpp:29-53`).

Composite calls repeatedly reset per-byte budgets. Examples include:

- a two-control-byte MV3/MV4 read;
- pointer write plus custom read;
- a 16-byte part-name write with per-byte write delay and readback.

For TunnelMonitor:

- document a conservative maximum blocking time for every public operation
  under a normalized `Config`;
- enforce configuration limits compatible with a useful worker deadline;
- let the E2 module check its immutable 64-bit command deadline before each
  synchronous library call;
- stage multi-step maintenance work between polls where practical.

Cancellation cannot interrupt a synchronous E2 transaction. It can be exact
before I/O and between calls. That is acceptable only when each call has a
known, tested upper bound. A generic async engine is not required.

### P1-2: Persistent single-byte uncertainty is incomplete

Multi-byte interval, offset, gain, and part-name paths can mark persistent
configuration dirty. Single-byte wrappers do not consistently do so:

- bus address (`src/EE871.cpp:801-811`);
- CO2 interval factor (`src/EE871.cpp:845-852`);
- filter (`src/EE871.cpp:864-871`);
- operating mode (`src/EE871.cpp:879-894`);
- auto-adjust start (`src/EE871.cpp:912-920`);
- arbitrary `customWrite()`.

`_customWriteDirect()` can know that a write was accepted, then fail at STOP or
verification, but `customWrite()` discards that uncertainty
(`src/EE871.cpp:551-599`).

`resyncPersistentConfig()` also checks only interval, offset, gain, and optional
part name (`src/EE871.cpp:427-463`).

Before TunnelMonitor exposes any maintenance write, the library should:

- classify persistent/configuration addresses;
- mark accepted-but-unverified writes dirty;
- include each supported persistent setting in a documented verification or
  resync path;
- preserve bus-address and irreversible auto-adjust semantics explicitly.

Normal sampling must never perform these writes implicitly.

### P1-3: Some runtime bus faults lose precision

- `e2Start()` waits for SCL high but does not require released SDA to be high
  before generating START (`src/EE871.cpp:59-71`). SDA stuck low can become a
  later NACK/timeout instead of `BUS_STUCK`.
- Wrong group/subgroup is currently `DEVICE_NOT_FOUND`, even though the device
  responded.
- Generic readback mismatch returns `E2_ERROR`
  (`src/EE871.cpp:591-598`) without a distinct typed verify-mismatch code.

TunnelMonitor health and events need to preserve:

- NACK/definite absence;
- SCL/SDA bus stuck;
- clock-stretch timeout;
- PEC mismatch;
- responding unsupported identity/capability;
- CO2 sensor status error and error code;
- validated out-of-range data;
- persistent write accepted but verification uncertain.

The worker must not parse `Status::msg` to recover error identity.

### P1-4: Calibration capability guards are incomplete

The E2 specification exposes supported-function bytes for custom adjustment,
but the library does not cache the relevant support data. CO2 offset/gain writes
go directly to custom memory (`src/EE871.cpp:942-991`).

If TunnelMonitor will expose calibration, add the narrow capability read and
return `NOT_SUPPORTED` when the function is not advertised. Calibration should
remain an explicit maintenance workflow with authorization, readback, event
logging, and HIL.

### P1-5: Library health counters are transfer counters, not sample counters

Each low/high/status/custom-memory transaction updates health independently.
For example, a successful 16-bit value read contributes two tracked successes
(`src/EE871.cpp:481-504`, `1231-1262`).

`tick(uint32_t)` only stores the last application-supplied time, so completion
timestamps reflect the last tick value rather than an actual internal clock
read (`src/EE871.cpp:322-324`).

TunnelMonitor must therefore:

- call `tick()` immediately before owner work if library timestamps are used;
- treat library counters as protocol-transfer diagnostics only;
- maintain its own 64-bit logical request/sample attempts, completions,
  presence, success, timeout, sensor-error, and recovery counters.

No library 64-bit scheduler or hidden time source is required.

### P1-6: Native fake coverage is insufficient for the field-facing surface

The current suite registers 31 passing tests
(`test/test_basic.cpp:585-618`). It has good basic coverage for transport faults,
health transitions, and some persistent dirty paths, but it does not prove:

- accepted absent startup;
- offline fast-fail with zero line activity;
- failed-recovery state invariants;
- wrong subgroup and missing CO2 capability;
- atomic feature-cache failure/reload;
- checked MV3/MV4 value-before-status ordering;
- status error with and without error-code support;
- error-code transfer failure;
- 50,000/50,001 ppm boundary behavior;
- legal long `0x10`/`0x50` SCL stretching;
- delayed pointer readiness;
- cooperative long delays;
- accepted single-byte persistent uncertainty;
- realistic whole-operation blocking bounds.

The fake should gain only the small deterministic controls and counters needed
to cover those cases.

## TunnelMonitor-node Integration Status

The current firmware intentionally has no E2 runtime:

- `SystemResource::E2Bus` is declaration-only
  (`../TunnelMonitor-node/include/TunnelMonitor/contracts/Health.h:79-105`).
- `BusId` contains no E2 bus (`Health.h:98-105`).
- `ServiceId` contains I2C and RS485 owners but no E2 owner
  (`Health.h:54-77`).
- `DeviceKind` has no EE871 kind
  (`../TunnelMonitor-node/include/TunnelMonitor/contracts/DeviceProfile.h:19-30`).
- The selected TunnelMonitor profile contains RTC, FRAM, environment, power,
  display, SHZK, and VibWire only
  (`../TunnelMonitor-node/include/TunnelMonitor/product/tunnelmonitor/TunnelMonitorBuildProfile.h:370-398`).
- `BoardPins.h` defines I2C pins but no E2 CLK/DATA pins
  (`../TunnelMonitor-node/include/TunnelMonitor/BoardPins.h:26-30`).
- `platformio.ini` has no EE871 dependency.
- `open_questions.md` explicitly leaves E2 for future CO2Control composition
  (`../TunnelMonitor-node/docs/guidelines/open_questions.md:97-110`).

Before implementation, the product owner must decide whether EE871 is:

1. a new device in the current TunnelMonitor profile; or
2. part of the separately named future `Co2Control` product profile.

That decision changes profile identity, board/pin policy, sample schema, storage,
Cloud, web, and validation scope.

## What Belongs in TunnelMonitor, Not EE871-E2

The following are required integration work but are not missing library
features.

| Firmware area | Required TunnelMonitor change |
| --- | --- |
| Product contracts | Append `ServiceId::E2Task`, `BusId::E2`, `DeviceKind::Ee871`, and activate an owned `E2Bus` resource without renumbering existing values. |
| Error/events | Allocate precise E2 error/event identities for timeout, NACK/absence, stuck bus, PEC, identity, recovery, sensor error, range, and persistent uncertainty. |
| Board/electrical | Select non-conflicting CLK/DATA pins; document external pull-ups, E2 high voltage, bidirectional open-drain level shifting, and cable assumptions. |
| Dependency | Pin an exact reviewed EE871 release/commit in firmware and native environments. |
| E2 owner | Add one static worker task, bounded command/result queues, immutable deadlines, cached status, events, recovery backoff, and no direct access from CLI/web/measurement. |
| Device module | Add a thin `Ee871Module` that privately owns one stable `EE871::EE871` instance and translates library results into project contracts. |
| Measurement runtime | Dispatch E2 profile rows alongside I2C and RS485, including submit/take/cancel/deadline handling. |
| Reading catalog | Add at least a `co2_ppm` reading descriptor with exact units and mask mapping. |
| Product schema | Extend the selected profile, sample field list, CSV, replay, Cloud projection, web/status, and golden tests. Version intentional durable/wire changes. |
| Health | Publish separate E2 worker/resource health and EE871 device presence/health. Keep sensor-domain errors separate from bus health. |
| Settings/maintenance | Keep cadence and device enablement explicit. Do not expose arbitrary custom writes. Gate address/filter/interval/calibration operations as maintenance. |
| Validation | Add fake worker/module tests and connected HIL for waveform, electrical interface, hotplug, faults, timing, and long-run behavior. |

The current fixed capacities appear large enough for one additional service and
device, but every exact count and static validity assertion must be updated
deliberately.

## Recommended E2 Worker Shape

TunnelMonitor's I2C owner is a useful structural pattern, but E2 must be a
separate owner because E2 is not hardware I2C.

### Ownership

```text
App / Measurement / CLI / Web
  -> fixed E2 commands
    -> E2Task static queue
      -> Ee871Module
        -> EE871::EE871
          -> E2 GPIO adapter
```

Only `E2Task` may call the EE871 instance or drive E2 lines.

### Module lifecycle

The existing generic measurement contract can be reused:

- `Probe`
- `Measure`
- `ReadLast`
- start result: accepted/busy/disabled/unsupported/rejected
- poll state: idle/running/complete/needs-bus-recovery
- one retained consume-once terminal result
- exact request ID, device ID, and immutable deadline

Relevant current contracts are:

- `../TunnelMonitor-node/include/TunnelMonitor/contracts/DeviceMeasurement.h:20-98`
- `../TunnelMonitor-node/include/TunnelMonitor/i2c/I2cDeviceBinding.h:13-45`
- `../TunnelMonitor-node/include/TunnelMonitor/i2c/I2cTask.h:57-108`

The E2 binding should be separate from `I2cDeviceBinding` but can use the same
small non-virtual function-table pattern.

### Required behavior

- `bind()` only attaches callbacks/configuration; it performs no bus I/O.
- Request admission performs no bus I/O.
- `begin()`/identity work runs only in the E2 owner context.
- An absent optional device produces an initialized offline module, not a
  failed owner task.
- Normal measurements fast-fail while offline.
- Only an explicit owner recovery command/procedure calls `recover()`.
- Recovery backoff and retry cadence belong to the worker.
- Checked sample results, not raw transport-success values, become valid
  `co2_ppm`.
- The worker publishes a 64-bit completion time and its own counters.
- Cancellation is honored before I/O and between bounded synchronous calls.
- Persistent writes use separate maintenance commands and are never automatic
  recovery or measurement side effects.

### Deadlines

Do not copy I2C's 20 ms transfer timeout or 100 ms recovery assumptions. E2
uses a 500-5,000 Hz clock, permits clock stretching, and has 150/300 ms write
behavior.

The E2 worker must define deadlines from measured and documented EE871 worst
cases. Before each synchronous library call, it must verify that sufficient
deadline budget remains. If the library call has started, it runs to its tested
bound; cancellation cannot preempt bit-level signaling.

## Measurement Policy for TunnelMonitor

The following policy remains in firmware.

### Warm-up

Do not mark CO2 valid during initial sensor warm-up. The current protocol notes
describe a first measurement after approximately 4.3 seconds and recommend
allowing roughly 5-10 seconds before relying on data
(`docs/EE871_E2_Protocol_and_Register_Map.md:307-319`).

The worker/module should publish initializing or stale/invalid state during
that interval. The library should not invent a power-on timestamp.

### Average versus fast

- MV4 averaged is the default low-noise field value.
- MV3 fast is useful only when the product explicitly wants faster, noisier
  response.
- If both are required, expose them as distinct reading IDs rather than hiding
  one behind a runtime switch.

### Status side effect

The production sample sequence is:

```text
read selected MV3/MV4 value
read status for that last value
optionally read error code
publish validity
```

Reading status can trigger a new measurement and reset interval timing. A
triggered result is typically ready 5-10 seconds later
(`docs/EE871_E2_Protocol_and_Register_Map.md:321-327`).

Therefore:

- do not poll status merely as a harmless health register;
- do not reuse TunnelMonitor's existing generic five-second live ENV refresh
  without a deliberate EE871 timing design;
- keep scheduled sample cadence, optional live refresh, warm-up, trigger
  readiness, and stale policy explicit.

The current TunnelMonitor default production measurement cadence is 900 seconds
(`../TunnelMonitor-node/include/TunnelMonitor/core/SettingsService.h:17`), much
longer than an EE871 transaction. That does not remove the need for a correct
status-trigger policy.

## Suggested Error Translation

Exact TunnelMonitor numeric values require an append-only contract decision.
The semantic mapping should preserve at least:

| EE871 outcome | E2 worker/device meaning |
| --- | --- |
| `OK` | Successful protocol operation |
| `NACK` during presence probe | No response on this attempt; preserve NACK and let bounded owner policy decide presence, because an EE871 may also NACK while measurement has priority |
| `NACK` during an online operation | Transfer failure; may contribute to offline transition |
| `TIMEOUT` | Clock-stretch or bounded operation timeout, retaining phase/detail |
| `BUS_STUCK` | Physical E2 bus fault requiring explicit recovery |
| `PEC_MISMATCH` | E2 protocol integrity failure |
| `NOT_SUPPORTED` identity/capability | Responding incompatible device |
| `BUSY` from offline guard | Driver offline; explicit recovery required |
| `CO2_SENSOR_ERROR` | Sensor-domain measurement failure, not bus failure |
| `OUT_OF_RANGE` from checked sample | Valid transport but invalid sensor-domain value |
| Write verify/dirty failure | Maintenance outcome uncertain; inspect/resync |

Do not collapse these outcomes into a generic device read failure in cached
status, events, or HIL output.

## Required Validation Before Production

### EE871 library native tests

At minimum add:

- `RequirePresent` absence failure;
- `AllowAbsent` accepted offline startup;
- incompatible group/subgroup/missing CO2 rejection;
- offline normal call returns without line activity;
- successful recovery with capability reload;
- failed recovery stays offline;
- checked MV4 and MV3 success with value-before-status ordering;
- status CO2 error with each documented error code;
- status error without error-code capability;
- error-code transfer failure;
- 50,000 and 50,001 ppm boundary cases;
- sensor/range errors do not increment transport failures;
- legal and excessive `0x10`/`0x50` stretch;
- pointer completion before `0x51`;
- delay/yield slicing and no yield in bit timing;
- persistent single-byte accepted-but-unverified state;
- SDA stuck before START;
- safe configuration/WCET boundaries.

### TunnelMonitor native tests

Mirror the existing I2C module/worker rigor:

- no I/O during bind or admission;
- static queue full and result retention;
- exact request/device/deadline identity;
- queued and active cancellation;
- optional absent startup;
- recovery backoff and bus invalidation;
- one active operation per driver instance;
- checked sample translation into valid/error masks;
- worker heartbeat remains bounded during legal E2 delays;
- 64-bit completion and stale policy;
- profile/catalog/schema/CSV/Cloud mapping;
- disabled device produces bus silence;
- no library type leaks into shared contracts.

### Hardware HIL

Run and capture:

- actual E2 CLK/DATA waveform at configured minimum/maximum clock rates;
- open-drain release-low behavior through the selected level shifter;
- pull-up voltage and rise time;
- real clock stretching;
- sensor absent at boot and later hotplug;
- unplug/replug while online;
- SCL and SDA stuck-low fixtures;
- PEC corruption/fault injection where feasible;
- normal, fast, status-error, and out-of-range simulations where feasible;
- warm-up and first-valid-sample timing;
- status-trigger-to-ready timing;
- long cable at the intended installation length;
- 150/300 ms write completion;
- persistent setting readback, power cycle, and restoration;
- owner queue/heartbeat/watchdog behavior during faults and maintenance;
- long-run soak with no unexpected reset, queue leak, or false-valid CO2.

No hardware result should be claimed until the exact firmware/library build,
board, fixture, and raw evidence are recorded.

## Recommended Implementation Order

1. Fix `0x10`/`0x50` completion/stretch handling and pointer sequencing.
2. Add `BeginPolicy::AllowAbsent`, startup diagnostics, and the offline no-I/O
   guard.
3. Centralize full identity/capability validation and atomic feature reload in
   begin/probe/recover.
4. Add checked average/fast sample helpers with sensor-domain and range
   semantics.
5. Add cooperative long-delay callbacks and document/enforce public-operation
   blocking bounds.
6. Complete persistent single-byte uncertainty and resync behavior.
7. Expand the EE871 fake and native field-facing matrix.
8. Release and exact-pin a new EE871 version; do not silently change v1.0.0.
9. Resolve whether EE871 belongs to TunnelMonitor or the future Co2Control
   product, including pins and electrical design.
10. Add the separate TunnelMonitor E2 worker, EE871 module, contracts, profile,
    sample/schema projections, health, events, and diagnostics.
11. Run native cross-layer tests, firmware builds, connected HIL, fault tests,
    and soak validation.

## Production Acceptance Gate

EE871 integration is ready for production consideration only when all of the
following are true:

- all P0 library findings are fixed;
- EE871 native fault tests cover the listed startup, offline, identity, sample,
  timing, and recovery cases;
- the consuming firmware exact-pins the fixed library release;
- the product/profile decision is explicit;
- E2 pins and the electrical interface are documented and reviewed;
- the dedicated E2 worker is the sole owner;
- worker deadlines cover proven library operation bounds;
- only checked CO2 results enter sample/storage/Cloud paths;
- raw reads remain diagnostic only;
- maintenance writes are explicit, authorized, verified, and observable;
- health keeps bus transport, device presence, and sensor-domain errors
  distinguishable;
- connected hardware HIL and fault/soak evidence pass on the intended board.

## Final Answer

The library is a **good architectural base** for TunnelMonitor because it
already has the right injected-HAL, no-bus-ownership, fixed-state, synchronous
shape.

It is **not functionally complete for TunnelMonitor field use** in v1.0.0. The
critical missing pieces are:

1. accepted absent-at-boot/offline lifecycle;
2. latched offline behavior with explicit recovery;
3. full EE871 CO2 identity and capability validation;
4. checked value/status/error/range sample helpers;
5. correct long write and custom-pointer completion timing;
6. cooperative, documented worker-safe blocking bounds.

TunnelMonitor must separately add the E2 owner, GPIO/electrical adapter,
deadlines/queues, product profile, CO2 schema, scheduling, cached status,
health/events, persistence/Cloud mapping, and validation. Those concerns should
not be moved into the EE871 protocol library.
