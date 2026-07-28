# EE871 E2 Operation Timing Bounds

Last reviewed: 2026-07-28.

## Purpose

`EE871::operationTimingBound()` provides a conservative admission bound for a
complete synchronous public operation. It performs no E2 I/O. An owner can use
the static overload before `begin()` to validate a proposed `Config`, or the
instance overload after `begin()` to query the normalized active configuration.

This is not a preemption guarantee. It assumes every transport callback remains
bounded and that delay callbacks wait at least the requested duration. Callback
runtime or scheduler oversleep beyond that contract can extend wall-clock time.
The bound includes the longest protocol branch for the selected operation; an
early validation, unsupported-capability, NACK, or offline return may be much
shorter.

## Common Terms

All intermediate arithmetic is checked `uint64_t` microseconds. The result is
ceiling-divided to milliseconds and published only when it fits `uint32_t`.

Let:

- `L = clockLowUs`
- `H = clockHighUs`
- `SH = startHoldUs`
- `PH = stopHoldUs`
- `BT = bitTimeoutUs`
- `BY = byteTimeoutUs`
- `SETUP = 10 us`
- `WD = normalized writeDelayMs`
- `ID = normalized intervalWriteDelayMs`

The shared terms are:

```text
START = BT + 2*SH + L
STOP = SETUP + BT + 2*PH
READ = START + 3*BY + STOP
NORMAL_WRITE = START + 4*BY + STOP
COMPLETION_WRITE(delayMs) = START + 4*BY + 1000*delayMs
RESET = 9*(L + BT + H) + L + SETUP + BT + 2*PH
```

For shorter formulas below:

```text
R = READ
P = COMPLETION_WRITE(WD)  // 0x50 pointer write
W = COMPLETION_WRITE(WD)  // effectful 0x10 byte write
N = NORMAL_WRITE
I = COMPLETION_WRITE(ID)  // committing interval high byte
```

`READ` contains the control/ACK, data/master-ACK, and PEC/master-NACK byte
deadlines. `NORMAL_WRITE` contains four ordinary byte deadlines and an ordinary
STOP. For completion writes, the completion window replaces separate final
ACK/STOP/quiet terms: it is one total budget beginning after the PEC byte.

## Operation Formulas

| `OperationKind` | `elementCount` | Formula in microseconds |
| --- | ---: | --- |
| `CONTROL_READ` | 1 | `R` |
| `CUSTOM_POINTER_WRITE` | 1 | `P` |
| `CUSTOM_BYTE_READ` | 1 | `P + R` |
| `CUSTOM_BLOCK_READ` | 1..256 | `P + elementCount*R` |
| `CUSTOM_BYTE_WRITE_VERIFY` | 1 | `W + P + R` |
| `INTERVAL_WRITE_VERIFY` | 1 | `N + I + P + 2*R` |
| `PART_NAME_WRITE_VERIFY` | 1 | `16*(W + P + R)` |
| `RAW_CO2_READ` | 1 | `2*R` |
| `BUS_RESET` | 1 | `RESET` |
| `BEGIN_REQUIRE_PRESENT` | 1 | `RESET + 4*R + P + 7*R` |
| `BEGIN_ALLOW_ABSENT` | 1 | `RESET + 4*R + P + 7*R` |
| `PROBE_IDENTITY` | 1 | `4*R` |
| `RECOVER_IDENTITY_AND_CAPABILITIES` | 1 | `RESET + 4*R + P + 7*R` |
| `CHECKED_CO2_AVERAGE` | 1 | `P + 4*R` |
| `CHECKED_CO2_FAST` | 1 | `P + 4*R` |
| `CUSTOM_BLOCK_WRITE_VERIFY` | 1..16 | `elementCount*(W + P + R)` |
| `RESYNC_PERSISTENT_CONFIG` | 1 | `9*P + 27*R` |
| `AUTO_ADJUST_MAINTENANCE` | 1 | `W + 2*P + 2*R` |
| `BUS_ADDRESS_CHANGE` | 1 | `W` |

The pointer component is intentionally included before every custom-memory
readback. A block read sets the pointer once and then performs
`elementCount` auto-incrementing `0x51` reads. Part-name and paired
offset/gain writes retain per-element immediate equality verification so
acknowledged, observed, and matched element counts remain exact.

The interval procedure is deliberately different: it stages the low byte,
writes the committing high byte with one pair-completion budget, then sets the
pointer once and verifies both bytes. It does not perform an invalid immediate
read between `0xC6` and `0xC7`.

The lifecycle formulas reserve four reads for group low/high, subgroup, and
available-measurements validation. Begin and recovery then reserve one pointer
write plus seven auto-incrementing reads for capability bytes `0x03..0x09`.
`BEGIN_ALLOW_ABSENT` uses the same conservative bound as strict begin even
though definite absence returns sooner. `PROBE_IDENTITY` does not load or
publish capabilities.

Each checked CO2 bound reserves two value reads, one side-effecting status read,
and the worst-case capability-gated error-code pointer plus data read. The
query includes that branch regardless of the current capability cache.

`RESYNC_PERSISTENT_CONFIG` deliberately publishes one fixed bound. The
worst-case no-target procedure groups nine target reads: 16-byte part name,
bus address, two-byte interval, factor, filter, mode, auto-adjust status,
two-byte offset, and two-byte gain. That is `9*P + 27*R`. Capability checks can
skip unsupported targets, and a target-specific unresolved resync is shorter,
but neither case changes the public bound. `elementCount` does not encode the
mutation target.

Auto-adjust includes a pre-write status observation, one effectful write, and a
post-write status observation. Address change includes only the effectful write:
it intentionally performs no unsafe old-address readback or hidden address
scan.

## Exhaustive Public Bus-Method Map

This table is checked against the `@note Timing contract` markers in
`include/EE871/EE871.h` by `tools/check_public_timing_contract.py`. One row
represents one public declaration; overloads therefore have separate rows.

<!-- PUBLIC_BUS_TIMING_MAP_BEGIN -->
| Method | `OperationKind` | Procedure/formula |
| --- | --- | --- |
| `begin(config)` | `BEGIN_REQUIRE_PRESENT` or `BEGIN_ALLOW_ABSENT` | Selected by `Config::beginPolicy`; `RESET + 11*R + P`. |
| `probe()` | `PROBE_IDENTITY` | Raw, health-neutral compatible-identity probe; `4*R`. |
| `recover()` | `RECOVER_IDENTITY_AND_CAPABILITIES` | Tracked reset, identity, and capability reload; `RESET + 11*R + P`. |
| `resyncPersistentConfig()` | `RESYNC_PERSISTENT_CONFIG` | Fixed capability-aware worst case; `9*P + 27*R`. |
| `readControlByte(mainCommand, data)` | `CONTROL_READ` | `R`. |
| `readU16(lowCommand, highCommand, value)` | `RAW_CO2_READ` | Two control reads; `2*R`. |
| `setCustomPointer(address)` | `CUSTOM_POINTER_WRITE` | `P`. |
| `customRead(address, data)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `customRead(address, buf, len)` | `CUSTOM_BLOCK_READ` | `P + len*R`, `len=1..256`. |
| `customWrite(address, value)` | `CUSTOM_BYTE_WRITE_VERIFY` or `AUTO_ADJUST_MAINTENANCE` or `BUS_ADDRESS_CHANGE` | Address-classified dispatch. Protected pair/calibration/read-only addresses reject before I/O; admitted targets use the corresponding bound. |
| `writeMeasurementInterval(interval)` | `INTERVAL_WRITE_VERIFY` | Deferred pair commit and pair readback; `N + I + P + 2*R`. |
| `readGroup(group)` | `RAW_CO2_READ` | Group low/high control reads; `2*R`. |
| `readSubgroup(subgroup)` | `CONTROL_READ` | `R`. |
| `readAvailableMeasurements(bits)` | `CONTROL_READ` | `R`. |
| `readFirmwareVersion(main, sub)` | `CUSTOM_BLOCK_READ` | Two-byte block read; `P + 2*R`. |
| `readE2SpecVersion(version)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `readOperatingFunctions(bits)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `readOperatingModeSupport(bits)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `readSpecialFeatures(bits)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `readSerialNumber(buf)` | `CUSTOM_BLOCK_READ` | 16-byte block read; `P + 16*R`. |
| `readPartName(buf)` | `CUSTOM_BLOCK_READ` | 16-byte block read; `P + 16*R`. |
| `writePartName(buf)` | `PART_NAME_WRITE_VERIFY` | Sixteen effectful writes with per-element verification; `16*(W + P + R)`. |
| `readBusAddress(address)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `writeBusAddress(address)` | `BUS_ADDRESS_CHANGE` | `W`; no in-session readback or address scan. |
| `readMeasurementInterval(interval)` | `CUSTOM_BLOCK_READ` | Two-byte block read; `P + 2*R`. |
| `readCo2IntervalFactor(factor)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `writeCo2IntervalFactor(factor)` | `CUSTOM_BYTE_WRITE_VERIFY` | `W + P + R`. |
| `readCo2Filter(filter)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `writeCo2Filter(filter)` | `CUSTOM_BYTE_WRITE_VERIFY` | `W + P + R`. |
| `readOperatingMode(mode)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `writeOperatingMode(mode)` | `CUSTOM_BYTE_WRITE_VERIFY` | `W + P + R`. |
| `readAutoAdjustStatus(running)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `startAutoAdjust()` | `AUTO_ADJUST_MAINTENANCE` | Pre-observe, write, post-observe; `W + 2*P + 2*R`. |
| `readCo2Offset(offset)` | `CUSTOM_BLOCK_READ` | Capability-gated two-byte block read; `P + 2*R`. |
| `writeCo2Offset(offset)` | `CUSTOM_BLOCK_WRITE_VERIFY` | Two per-element verified writes; `2*(W + P + R)`. |
| `readCo2Gain(gain)` | `CUSTOM_BLOCK_READ` | Capability-gated two-byte block read; `P + 2*R`. |
| `writeCo2Gain(gain)` | `CUSTOM_BLOCK_WRITE_VERIFY` | Two per-element verified writes; `2*(W + P + R)`. |
| `readCo2CalPoints(lower, upper)` | `CUSTOM_BLOCK_READ` | Capability-gated four-byte block read; `P + 4*R`. |
| `readStatus(status)` | `CONTROL_READ` | `R`; may trigger the next measurement under documented device conditions. |
| `readErrorCode(code)` | `CUSTOM_BYTE_READ` | `P + R`. |
| `readCo2Fast(ppm)` | `RAW_CO2_READ` | Raw MV3 low/high; `2*R`. |
| `readCo2Average(ppm)` | `RAW_CO2_READ` | Raw MV4 low/high; `2*R`. |
| `readCo2AverageSample(out)` | `CHECKED_CO2_AVERAGE` | Value, status, and worst-case error detail; `P + 4*R`. |
| `readCo2FastSample(out)` | `CHECKED_CO2_FAST` | Value, status, and worst-case error detail; `P + 4*R`. |
| `busReset()` | `BUS_RESET` | Nine recovery clocks and STOP; `RESET`. |
| `checkBusIdle()` | `BUS_RESET` | Line-read-only operation mapped conservatively to the existing reset class. |
<!-- PUBLIC_BUS_TIMING_MAP_END -->

`customWrite()` does not use one fabricated universal procedure. Its constexpr
address classifier routes admitted special addresses to the typed algorithm
shown above and rejects unsafe pair, calibration, and documented read-only
addresses before line I/O. All other admitted writable bytes use
`CUSTOM_BYTE_WRITE_VERIFY`.

## Explicit Non-I/O Public Methods

The public-header audit also requires `NO_E2_IO` markers. These cover:

- `tick()` and `end()`;
- driver state, configuration, identity, capability, health, and mutation
  getters;
- `getSettings()` and all cached `has*()` helpers;
- both `operationTimingBound()` overloads;
- `hasCo2Error()`;
- `acknowledgeAutoAdjustUncertainty()`.

They have no E2 blocking bound because they perform no E2 line access. In
particular, the auto-adjust acknowledgement only reconciles retained RAM
evidence; it does not write the sensor.

## Validation And Normalization

The static query uses the same centralized validation as `begin()`:

- all five required transport callbacks must exist;
- the address must be 0..7;
- clock low/high must each be at least 100 us;
- `clockLowUs + clockHighUs + 10 us` must be at most 2000 us;
- START/STOP holds must each be at least 4 us;
- `bitTimeoutUs` must be 1..25000;
- `byteTimeoutUs` must be between `bitTimeoutUs` and 35000;
- write and interval delays must not exceed 5000 ms;
- long-delay slice must not exceed 50 ms;
- begin policy must be `REQUIRE_PRESENT` or `ALLOW_ABSENT`.

Write delays below 150 ms normalize to 150 ms. Interval delays below 300 ms
normalize to 300 ms. A zero long-delay slice normalizes to 1 ms, and a zero
offline threshold normalizes to one. The query does not mutate the supplied
`Config` or a driver instance.

Fixed-size operations reject counts other than one. `CUSTOM_BLOCK_READ`
accepts 1..256, and `CUSTOM_BLOCK_WRITE_VERIFY` accepts 1..16. Rejected queries
do not modify the output object and do not touch E2 lines.

## Minimum-Hold Reference Example

For 100 us clock phases, 4 us START/STOP holds, 25/35 ms bit/byte deadlines,
and 150/300 ms completion windows, the published bounds are:

| Operation | Bound |
| --- | ---: |
| Control read | 156 ms |
| Pointer write | 316 ms |
| One custom byte read | 471 ms |
| Three-byte custom block read | 781 ms |
| Custom byte write plus verify | 786 ms |
| Interval pair write plus verify | 1281 ms |
| Complete 16-byte part-name write plus per-element verify | 12566 ms |
| Raw two-byte CO2 read | 311 ms |
| Bus reset | 252 ms |
| Strict or optional begin | 2274 ms |
| Full identity probe | 621 ms |
| Identity-and-capability recovery | 2274 ms |
| Checked average or fast CO2 sample | 936 ms |
| Two-byte persistent block write plus per-element verify | 1571 ms |
| Complete capability-aware persistent resync | 7025 ms |
| Auto-adjust maintenance procedure | 1256 ms |
| Bus-address write completion | 316 ms |

Multi-byte helpers can therefore have substantially larger admission bounds
than one E2 transaction. Persistent writes remain explicit maintenance
operations; this query does not make them suitable for latency-sensitive or
ISR contexts.

## Health And Timestamp Ownership

Health success/failure counters count tracked E2 transfers, not samples or
public API calls. One checked sample normally contributes three successful
transfers and can contribute five when detailed error-code acquisition runs.
Semantic sensor errors and checked-range failures do not invent transport
failures.

`tick(uint32_t nowMs)` only stores the caller-supplied timestamp. It performs no
hidden bus work, scheduling, retry, or timebase extension. An external owner
that wants current diagnostic timestamps should call `tick()` immediately
before each owned library operation. The core library does not provide a
64-bit scheduler, task clock, retry cadence, sample freshness policy, or
application measurement cadence.
