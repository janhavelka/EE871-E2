# EE871-E2 Code Audit Resolution Report (2026-08-31)

## Scope and source state

The requested `docs/CODE_AUDIT.md` was not present after fetching all remotes.
The only current audit source was `docs/AUDIT_FINDINGS_2026-08-26.md`, so this
review treated that file as the intended report.

The review started from a clean, synchronized `main` at commit `d0a251a`.
Every finding was checked against the current implementation, native fake,
example CLIs, contract tools, maintained hardware evidence, and the searchable
vendor extracts. The open-findings file was removed after all sections were
resolved; this report preserves the decisions and evidence.

## Finding dispositions

### 1. Duplicated `begin()` / `busReset()` waveform — valid and fixed

`begin()` and `busReset()` did contain separate reset waveforms. `begin()`
eventually failed on a permanently low SCL, but only after continuing through
all reset clocks and with the less precise final-idle diagnostic.

The waveform now lives in private `_busResetRaw()`. Public `busReset()` retains
its initialization guard and remains health-neutral; `begin()` can call the raw
helper before initialization and returns its precise `BUS_STUCK` result. The
helper also reuses the common bounded SCL-high wait. Poll steps are clipped to
the remaining deadline, avoiding the previous up-to-four-microsecond overshoot
for timeouts not divisible by five.

Native coverage now proves SCL-stuck fail-fast behavior, permanent SDA-low
failure, SDA release during recovery clocks, successful untracked reset, and
SDA-low idle diagnosis.

### 2. Missing timing upper validation — partly valid; proposal corrected

The missing minimum-frequency check was real. The proposed independent
`clockLowUs <= 1000` / `clockHighUs <= 1000` limits were not correct because the
vendor specifies a clock-frequency range plus individual minimum high/low
times, not individual maximums. Those limits would reject valid asymmetric
timing and still accept `1000 + 1000 us`, whose generated period is actually
2010 us after the driver's 10 us data setup (about 497.5 Hz).

`begin()` now validates the waveform it generates:

```text
10 us + clockLowUs + clockHighUs <= 2000 us
```

This accepts the inclusive 500 Hz boundary and rejects slower configurations.
It also requires `byteTimeoutUs` to be strictly greater than the nominal
nine-bit byte time. The audit's statement that such configurations always
timed out without stretching was false—the byte budget was only consulted
inside an SCL-low wait—but rejecting an incoherent budget is still the simplest
safe configuration contract.

Tests cover both 100 us minima, valid asymmetric 500 Hz timing, one microsecond
below the frequency boundary, the setup-time edge case, maximum `uint16_t`
inputs, and byte-budget equality/boundaries. Arithmetic is widened before the
sum; the present field ranges cannot overflow `uint32_t`.

### 3. Five-read burst spuriously reaches OFFLINE — invalid as reported

No default threshold change was made.

The claimed five consecutive failures do not occur for the stated
`readCo2Fast()` + `readCo2Average()` + `readStatus()` sequence. Each 16-bit read
returns immediately when its low-byte transaction NACKs, so a fully NACKed
sequence records three tracked failures, not five. A new native regression test
proves the count and the resulting `DEGRADED` state at threshold five.

The claimed cause of observed bench NACKs was also not established. Generic E2
mode documentation permits measurement-priority NACKs, but AN0105 explicitly
lists EE871 among the devices that can process enquiries while measuring. The
qualified unit advertised operating-mode support `0x00`, and the maintained
hardware ledger already classifies the phase-correlated NACK cause as unknown.

Raising the default from five to eight would therefore weaken unplug detection
on the basis of an incorrect transaction count and an unproven cause. API docs
now state explicitly that health counts tracked transfers, not sample cycles.
Application-owned retry policy remains unchanged and no NACK is reclassified.

### 4. Torn 16-bit reads in E2-priority mode — invalid

No extra low/high/low traffic was added. E2 v4.1 states that reading a measured
value's low byte captures its associated high byte in the slave, ensuring a
coherent pair. AN0105 requires the same low-then-high sequence for consistent
data and applies it to MV3/MV4. That contract is not conditional on operating
priority.

The current `readU16()` order is correct. Public docs now record the latch
contract so it is not replaced later by a redundant consistency read that
would add latency and new failure opportunities.

The audit also mentioned custom-memory pairs, but the measured-value latch
contract does not apply to `0x51` reads. Adjacent custom fields already use the
vendor pointer-auto-increment recipe; no source establishes an atomic snapshot
against independent sensor-side mutation, and the proposed `readU16()` change
would not affect `customRead()` anyway. No speculative extra traffic was added.

### 5. `BUSY` / `IN_PROGRESS` are not returned — current-path observation valid; removal rejected

The synchronous driver does not currently produce either status, and the
`IN_PROGRESS` health branch is unreachable from today's raw transfers.
Removing public enum members or `Status::inProgress()` would nevertheless
break downstream source and shift later implicit numeric enum values used by
diagnostics/telemetry.

Both values and the harmless neutral branch were retained for compatibility.
`BUSY` is now documented consistently as reserved and not returned. This is
simpler and safer than scheduling a breaking cleanup with no runtime benefit.

### 6. IDF scanner and `libtest` lagged Arduino hardening — valid and fixed

The IDF scanner now makes up to five attempts, counts a device only after ACK
and valid PEC, and reports PEC-invalid responses separately. IDF `libtest` now
accepts the initialized driver and calls `readControlByte()`, gaining the
production clock-stretch, PEC, status, and health path instead of issuing raw
transactions.

The IDF source contract now checks the PEC-gated scanner structure and rejects
raw calls inside `libtest`, preventing the old implementation from satisfying
the check accidentally. Hardware revalidation of these commands was not run in
this software-only audit.

### 7. Example divergences — valid and resolved

- Both bounded line readers now surface the same marker, and both command
  processors emit an explicit `Input line too long (maximum 127 characters)`
  warning instead of silently dropping input or reporting a misleading unknown
  command.
- Both `sniff` help/start paths warn that synchronous decoder output perturbs
  E2 timing. README guidance states that sniffer-enabled traffic is not timing
  or protocol-stability evidence.
- The ineffective 10 ms IDF sniffer polling method/call were removed. The
  synchronous trace-wrapper path remains the actual decoder input.
- Same-directory Arduino helper includes now use relative names.
- The IDF example README documents its checkout-directory/component-name
  coupling and the simple rename workaround.

### 8. Tool/test cleanups — valid items fixed; optional logging churn rejected

- The core timing guard is now a direct forbidden-pattern scan; empty allowance
  maps and reconciliation logic were removed without weakening diagnostics.
- Arduino CLI mandatory commands are now checked by dispatch regexes, not bare
  words that help text could satisfy.
- HIL-only compact summaries no longer list soak-only attempt/retry keys.
- The fake's unused delay-call counter was removed. SDA-low reset/idle tests
  were added. The previously unused SDA-high fault exposed a real precision
  gap, so START now verifies that SDA was actually pulled low and returns
  `BUS_STUCK` instead of a misleading later NACK; that path is tested.
- `LOGD` / `LOGT` were retained. `LOGE` is also currently unused, and the full
  conventional example logging surface has zero core or steady-state cost;
  trimming only two levels would be arbitrary churn.

## Validation performed

- `scripts/pio.cmd test -e native`: 43/43 tests passed.
- `python tools/check_core_timing_guard.py`: passed.
- `python tools/check_cli_contract.py`: passed.
- `python tools/check_idf_example_contract.py`: passed.
- `python -m unittest discover -s test -p "test_hil_runner_parser.py"`:
  40/40 tests passed.
- `doxygen Doxyfile`: passed with no warnings.
- `scripts/pio.cmd run -e ex_bringup_s3`: passed.
- `scripts/pio.cmd run -e ex_bringup_s2`: passed.
- `scripts/pio.cmd run -e compat_tunnelmonitor_s3`: source compilation was
  attempted twice but the environment did not complete. The shared local
  PlatformIO package directory lost upstream Arduino core headers and sources
  during compilation, including `Client.h`, `HWCDC.h`, and
  `esp32-hal-touch-ng.c`, even after the required wrapper reinstalled its
  pinned packages. This is recorded as a local package-cache failure, not a
  passing compatibility build or a source validation result.
- Physical EE871 hardware tests, electrical fault injection, soak, and native
  ESP-IDF builds were not run (`idf.py` was not available). No new hardware
  claim is made; native fake fault coverage is listed separately above.
