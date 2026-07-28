# EE871 E2 Operation Timing Bounds

Last reviewed: 2026-07-28.

## Purpose

`EE871::operationTimingBound()` provides a conservative admission bound for a
complete synchronous public operation. It performs no E2 I/O. An owner can use
the static overload before `begin()` to validate a proposed `Config`, or the
instance overload after `begin()` to query the normalized active config.

This is not a preemption guarantee. It assumes every transport callback remains
bounded and that delay callbacks wait at least the requested duration. Callback
runtime or scheduler oversleep beyond that contract can extend wall-clock time.

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

`READ` contains the control/ACK, data/master-ACK, and PEC/master-NACK byte
deadlines. `NORMAL_WRITE` contains four ordinary byte deadlines and an ordinary
STOP. For completion writes, the completion window replaces the separate final
ACK/STOP/quiet terms: it is one total budget beginning after the PEC byte.

## Operation Formulas

| `OperationKind` | `elementCount` | Formula in microseconds |
| --- | ---: | --- |
| `CONTROL_READ` | 1 | `READ` |
| `CUSTOM_POINTER_WRITE` | 1 | `COMPLETION_WRITE(WD)` |
| `CUSTOM_BYTE_READ` | 1 | `COMPLETION_WRITE(WD) + READ` |
| `CUSTOM_BLOCK_READ` | 1..256 | `COMPLETION_WRITE(WD) + elementCount*READ` |
| `CUSTOM_BYTE_WRITE_VERIFY` | 1 | `2*COMPLETION_WRITE(WD) + READ` |
| `INTERVAL_WRITE_VERIFY` | 1 | `NORMAL_WRITE + COMPLETION_WRITE(ID) + COMPLETION_WRITE(WD) + 2*READ` |
| `PART_NAME_WRITE_VERIFY` | 1 | `16*(2*COMPLETION_WRITE(WD) + READ)` |
| `RAW_CO2_READ` | 1 | `2*READ` |
| `BUS_RESET` | 1 | `RESET` |

The pointer component is intentionally included before each custom-memory
readback. A block read sets the pointer once and then performs
`elementCount` auto-incrementing `0x51` reads.

The interval formula reflects the real procedure: an ordinary staged low-byte
write, the high-byte interval commit, one completed pointer write, and two
auto-incrementing verification reads.

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
- long-delay slice must not exceed 50 ms.

Write delays below 150 ms normalize to 150 ms. Interval delays below 300 ms
normalize to 300 ms. A zero long-delay slice normalizes to 1 ms, and a zero
offline threshold normalizes to one. The query does not mutate the supplied
`Config` or a driver instance.

Fixed-size operations reject counts other than one. `CUSTOM_BLOCK_READ`
rejects zero and values above 256. Rejected queries do not modify the output
object and do not touch E2 lines.

## Default Example

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
| Complete 16-byte part-name write plus per-byte verify | 12566 ms |
| Raw two-byte CO2 read | 311 ms |
| Bus reset | 252 ms |

Multi-byte convenience helpers can therefore have substantially larger
admission bounds than one E2 transaction. Persistent writes remain explicit
maintenance operations; this query does not make them suitable for
latency-sensitive or ISR contexts.
