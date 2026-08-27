# EE871-E2 Code Audit — Open Findings and Proposals (2026-08-26)

Working document. Each finding below was verified against the vendor
documentation (`E2 interface specification v4.1`, `AN0105`, `AN1611-1`, the
`EE871 E2 addendum`) and the current source. Simple, unambiguous fixes were
applied directly during the audit (see `CHANGELOG.md [Unreleased]`); this file
lists only the findings that need a maintainer decision or a change worth
reviewing on hardware. Delete each section when resolved, and delete the file
when empty.

Severity: **A** = functional risk, fix soon; **B** = robustness/design gap;
**C** = cleanup/maintainability.

---

## 1. [B] `begin()` duplicates `busReset()` with weaker behavior

**Where:** `src/EE871.cpp` — `begin()` bus-recovery block vs `busReset()`.

**Issue:** `begin()` cannot call `busReset()` (it guards on `_initialized`), so
it carries an inline copy of the 9-clock + STOP reset. The copy silently
continues when SCL stays stuck during the reset clocks, while `busReset()`
returns `BUS_STUCK` at that point. Two implementations of the same waveform is
a maintenance hazard: a future timing fix in one will miss the other.

**Proposal:** Extract a private `Status _busResetRaw()` containing the current
`busReset()` body without the `_initialized` guard. `busReset()` becomes
guard + `_busResetRaw()`. In `begin()`, replace the inline block with:

```cpp
if (!readScl(_config) || !readSda(_config)) {
  Status st = _busResetRaw();
  if (!st.ok()) {
    _resetStoppedState();
    return st;   // BUS_STUCK with the precise reason
  }
}
```

Net effect: ~30 lines removed, one waveform definition, and `begin()` gains the
stricter stuck-SCL diagnostics. Behavior change is limited to error reporting
on already-failing hardware; re-run the stuck-line HIL scenario to confirm.

---

## 2. [B] `begin()` accepts clock timings the sensor and the driver's own timeouts cannot sustain

**Where:** `src/EE871.cpp` `begin()` validation; `include/EE871/Config.h`.

**Issue:** Validation enforces only lower bounds (`clockLowUs/clockHighUs >=
100`). Two upper-bound constraints are unchecked:

1. The E2 spec and the EE871 addendum specify fCLK = **500–5000 Hz**. A config
   with e.g. `clockLowUs = 2000` yields a ~240 Hz clock — outside the device's
   specified operating range.
2. The nominal (zero-stretch) byte time is `9 * (10 + clockHighUs +
   clockLowUs)` µs. With slow clocks and the default `byteTimeoutUs = 35000`,
   every transfer fails with a confusing `TIMEOUT` even though nothing is
   wrong on the bus.

**Proposal:** Add two checks to `begin()` next to the existing timing checks:

```cpp
if (config.clockLowUs > 1000 || config.clockHighUs > 1000) {
  return Status::Error(Err::INVALID_CONFIG, "Clock timing above spec (fCLK < 500 Hz)");
}
if (9U * (kDataSetupUs + config.clockHighUs + config.clockLowUs) >= config.byteTimeoutUs) {
  return Status::Error(Err::INVALID_CONFIG, "byteTimeoutUs below nominal byte time");
}
```

(`kDataSetupUs` lives in `EE871.cpp`, so the check belongs there; move the
constant above `begin()` if needed. Add matching notes to `Config.h` field
docs and a native test for each rejection.)

---

## 3. [B] A burst of reads during one measurement window can spuriously drive the driver OFFLINE

**Where:** `_updateHealth()` policy in `src/EE871.cpp`; default
`offlineThreshold = 5` in `Config.h`.

**Issue:** In the factory-default operating mode ("priority to measurement",
0xD8 bit1 = 0) the sensor **NACKs every transaction during the ~0.7 s
measurement** (E2 spec §operating mode; observed as real MV3 NACKs in the
8-hour soak). A typical sampling burst — MV3 lo+hi, MV4 lo+hi, status — is 5
tracked transactions issued back-to-back in well under 0.7 s. If the burst
lands in a measurement window, all 5 NACK and `consecutiveFailures` reaches
the default `offlineThreshold` of 5: the driver reports OFFLINE for a healthy
sensor. A NACK is protocol-indistinguishable from an absent device, so the
driver genuinely cannot classify it; the policy question is how many
consecutive NACKs mean "gone".

**Proposal (no code change in the transfer path):**

1. Raise the default `offlineThreshold` from 5 to **8** (one full burst plus
   margin) and document why next to the field.
2. Document the failure mode explicitly in the `EE871.h` health-tracking
   section and in the README checked-sampling recipe: on a control-byte NACK
   during scheduled sampling, wait >= 1 s and retry once before treating the
   cycle as failed (the soak harness already implements exactly this policy
   with its 1500 ms retry).

Alternative considered and rejected: treating `NACK` as a non-counted `BUSY` —
that would blind the health tracker to a genuinely unplugged sensor, whose
symptom is also NACK.

---

## 4. [B] 16-bit values are read in two bus transactions and can tear in E2-priority mode

**Where:** `readU16()` (`readCo2Fast`, `readCo2Average`, `readGroup`);
same pattern for custom-memory pairs.

**Issue:** Low and high bytes are read in separate START/STOP transactions with
no consistency check. In the default measurement-priority mode this cannot
produce a torn CO2 value (the sensor NACKs while it measures/updates, and the
vendor's own AN0105 reference reads exactly this way). But if an application
enables **E2-priority mode** (0xD8 bit1 = 1), communication is allowed during
measurement and a value crossing a 256-boundary between the two reads yields a
result up to ~255 ppm off.

**Proposal:** Document the constraint on `readCo2Fast()`/`readCo2Average()`
("in E2-priority mode, guard against torn reads"). If E2-priority operation
becomes a supported use case, add the standard mitigation inside `readU16()`:
read low, high, then low again; if the second low differs, repeat once. Cost
is one extra transaction only in the retry case; no API change.

---

## 5. [C] Dead status codes: `Err::BUSY`, `Err::IN_PROGRESS`

**Where:** `include/EE871/Status.h`; `Status::inProgress()`; the
`st.inProgress()` early-out in `_updateHealth()`; error-name switches in both
example CLIs.

**Issue:** Nothing in the driver ever returns `BUSY` or `IN_PROGRESS`; the
`inProgress()` check in `_updateHealth()` is unreachable. They are leftovers
from an abandoned asynchronous design (`tick()` is now diagnostics-only).

**Proposal:** Keep the enum values through 1.x (removing enumerators is a
source-breaking change for downstream `switch` statements) — their doc
comments now say "reserved". Remove both values, `Status::inProgress()`, the
`_updateHealth()` early-out, and the example switch cases together at the next
major version. Tracked here so the intent is not lost.

---

## 6. [B] IDF example CLI misses two hardening fixes the Arduino CLI received in 1.0.1

**Where:** `examples/idf/basic_bringup/main/main.cpp`.

**Issue:** Two CHANGELOG 1.0.1 fixes were applied only to the Arduino example:

- **Scanner:** the IDF `scan` reports `FOUND` on control-byte ACK alone (it
  even prints `PEC=MISMATCH` while still counting the device) and probes each
  address once. The Arduino scanner requires ACK **and** valid PEC over 5
  attempts and buckets invalid responses separately.
- **`libtest`:** the IDF version drives raw `sendStart`/`sendByteRaw`
  transactions, bypassing the clock-stretch-aware, PEC-validating driver path,
  while its help text claims it tests library commands. The Arduino version
  calls `device.readControlByte()`.

**Proposal:** Port the Arduino implementations: gate `FOUND` on ACK + PEC with
the same 5-attempt/invalid-response accounting, and route `libtest` through
`device.readControlByte()`. Both are contained, mechanical ports; validate on
hardware together with the GPIO-mode/pin fixes applied in this audit (the IDF
example has so far only ever been build-tested).

---

## 7. [C] Example divergences and diagnostics caveats

**Where:** `examples/common/`, `examples/idf/`.

- **Line-overflow behavior differs:** the Arduino `CliShell` returns a
  too-long-line marker (surfaces as "Unknown command"); the IDF `pollLine`
  silently discards the line. Unify on the Arduino behavior (an operator
  should see that input was dropped).
- **Sniffer perturbs timing:** the Arduino `snifferCallback` performs blocking
  `Serial.printf` inside transport callbacks, inserting up to several ms into
  bit timing mid-transaction. Functionally tolerated (master owns the clock)
  but any timing measurement taken while sniffing is invalid. Add one line to
  the `sniff` help text; optionally buffer and flush in `loop()` like `buslog`.
- **IDF sniffer poll tick is dead weight:** `diag::sniffer().tick()` samples
  once per 10 ms loop and can never decode 100 µs-scale traffic; only the
  trace-wrapper path produces decodes. Remove the poll or comment it.
- **Include style:** `BoardConfig.h`/`Log.h` include `"examples/common/X.h"`
  (works only via PlatformIO's `src_dir` include path) while `E2Diagnostics.h`
  uses plain relative includes. Standardize on relative includes.
- **IDF component name coupling:** `examples/idf/basic_bringup/main/CMakeLists.txt`
  `REQUIRES "EE871-E2"` only resolves if the repo directory is literally named
  `EE871-E2`. Add a note to the example README (or register the component with
  an explicit name).

---

## 8. [C] Tool/test cleanups

- `tools/check_core_timing_guard.py`: `ALLOWED_CALL_COUNTS` /
  `ALLOWED_INCLUDE_COUNTS` are permanently empty, so ~40 lines of
  reconciliation reduce to "any hit fails". Simplify to a plain
  forbidden-pattern scan.
- `tools/check_cli_contract.py`: the bare word-search `MANDATORY_COMMANDS`
  list is satisfiable by a help string alone; the dispatch regexes are the
  real check. Drop the list or convert entries to dispatch regexes.
- `tools/ee871_hil_runner.py` `compact_summary_row()`: the `attempt*` /
  `retry_of` keys are only ever produced by the soak runner, which has its own
  `compact_row`. Delete the dead keys or mark the schema as shared.
- `test/support/FakeE2Transport.h`: `setSdaStuckLow`, `setSdaStuckHigh`, and
  `delayCalls()` are unused. Preferred: add the missing stuck-SDA tests
  (`begin()` bus-reset path and `checkBusIdle()` both have SDA-stuck branches
  the suite never exercises). Otherwise delete the helpers.
- `examples/common/Log.h`: `LOGD`/`LOGT` are unused; keep or trim at will.

---

## Verified correct (do not re-audit)

Checked byte-for-byte against the vendor documents; no action needed:

- Control-byte layout, MSB-first order, ACK/NACK polarity, additive PEC
  (read: ctrl+data; write: ctrl+addr+data), read frame (data master-ACKed,
  PEC master-NACKed), write frame (four slave-ACKed bytes). Matches spec v4.1
  and the AN0105 reference.
- `0x50` pointer-set frame: address byte = pointer high (ignored by EE871),
  data byte = pointer low. Pointer auto-increment on `0x51` reads.
- Group `0x0367`, subgroup `0x09`, available-measurements CO2 bit `0x08`,
  feature-bit assignments in custom `0x07/0x08/0x09`, custom-memory addresses
  `0x58..0x5F`, `0xA0..0xBF`, `0xC0/0xC1`, `0xC6/0xC7`, `0xCB`, `0xD3`,
  `0xD8/0xD9`.
- Interval pair write: both bytes written back-to-back with **no** delay
  between them, then one <= 300 ms wait — exactly the AN1611-1 model (flash
  write starts only after both bytes). Single-byte `0x10` writes with the
  <= 150 ms wait likewise match.
- Default timeouts (25 ms/bit, 35 ms/byte) match the spec's clock-stretch
  limits; `waitSclHigh` handles stretching on every rising edge, which the
  vendor's own 8051 reference does not.
- `startAutoAdjust()` readback-verify is safe: 0xD9 bit0 reads 1 for the
  ~5 min adjustment, far longer than the 150 ms verify delay.
- Dirty-flag semantics for partial multi-byte persistent writes, including
  the `customWrite()` reroute of single `0xC6`/`0xC7` writes through the
  paired interval path (a lone half-write would leave the sensor waiting for
  its companion byte).
- Health tracking updates only through tracked wrappers; `probe()` and
  `busReset()` are correctly untracked.
