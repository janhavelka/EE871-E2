# EE871-E2 Python HIL Runner

Last updated: 2026-07-29

`tools/ee871_hil_runner.py` drives the Arduino or native ESP-IDF diagnostic
CLI and captures repeatable evidence. It does not flash firmware, operate
relays, prove electrical safety, or turn a dry run into hardware evidence.

A runner `PASS` applies only to the selected commands and parser assertions.
It does not by itself prove CO2 accuracy, calibration validity, electrical
compliance, long-soak stability, or production readiness.

## Safe Plans

The quick default plan remains non-persistent:

```powershell
python tools/ee871_hil_runner.py --port COM7 --output-dir hil_logs
```

It runs `version`, `help`, `probe`, a raw averaged read, `selftest`, health and
dirty-state checks, and `stress 50`.

Use the complete Prompt 04 safe plan before maintenance HIL:

```powershell
python tools/ee871_hil_runner.py --port COM7 --complete-safe
```

`--complete-safe`, `--include-extended`, and `--extended-safe` select the same
complete/extended plan. In addition to the quick plan it covers:

- `buscheck` and released-line `levels`;
- side-effecting `status`;
- raw `co2fast` and `co2avg`;
- semantically checked `samplefast` and `sampleavg`, with `drv` immediately
  before and after each procedure to compare transport-failure counters;
- `features`, cached `caps`, `fw`, and `e2spec`;
- `stress_mix 100`, `stress 500`, repeated reads and lifecycle cycles;
- explicit `recover`, full capability-aware `resync`, health, and dirty state.

Checked-sample validation parses every value/status/error attempt and validity
field, enforces FAST/MV3 and AVERAGE/MV4 identity, status-bit consistency, and
clean/error enum coherence. A coherent sensor-domain error remains
distinguishable from a transport failure, but it is not reported as a
healthy-bench PASS.

“Complete safe” means the complete non-destructive Prompt 04 command surface;
it does not include power-up warm-up or stale-measurement timing observation.
Those require controlled sensor power/timing evidence and remain separate HIL
rows rather than being silently simulated by the runner.

Extended controls:

- `--read-loop-count N`
- `--cycle-loop-count N`

## Baselines Are Recorded Values, Not Factory Defaults

Every destructive plan first records:

- exact build/commit and structured board, operator, sensor, fixture, electrical
  authority, and applicable power-procedure metadata;
- feature bytes and cached capabilities;
- a fresh complete mutation diagnostic;
- all 256 custom-memory bytes using `reg dump 0 256`;
- semantic reads for serial, exact part-name bytes, address, interval, factor,
  filter, mode, offset, gain, calibration points, and auto-adjust state.

The runner calls these values the **recorded baseline**. It never claims they
are factory defaults unless an independent factory record says so.

The 256-byte image is forensic evidence only. It contains read-only, volatile,
reserved, pointer, action, paired, address, and calibration registers. The
runner never replays the image and never uses `reg write` for restoration.
Only typed, allowlisted APIs may restore a recorded setting.

The baseline is written immediately to:

- `custom_memory_baseline.json`
- `custom_memory_baseline.hex`
- `checkpoint.json`

`checkpoint.json` is atomically replaced after every completed step. Before
each destructive command is transmitted, it is also atomically journaled as
`in_flight_destructive` with its exact resolved command and target group. The
entry is cleared only after the result is captured and checkpointed. If a
process, serial, or power failure leaves it in flight, treat the mutation
outcome as unknown: do not write or restore until read-only inspection and
explicit recovery establish the actual state.

An unexpected serial or parser exception produces a `FAIL` transcript,
summary, and checkpoint instead of escaping without final artifacts. Any
in-flight destructive entry remains intact and explicitly uncertain.

## Reversible Persistent Configuration

Persistent configuration runs are disabled by default and require two
confirmations:

```powershell
python tools/ee871_hil_runner.py --port COM7 `
  --include-persistent-writes `
  --confirm-persistent-writes `
  --board "ESP32-S3" `
  --target-name "ex_bringup_s3" `
  --operator "name" `
  --sensor-id "bench-ee871-01" `
  --fixture-id "isolated-e2-fixture-01" `
  --electrical-authority "approved-procedure-id"
```

The live runner additionally requires:

```text
RUN EE871 PERSISTENT WRITES
```

The interval test always runs. Without `--maintenance-interval`, the runner
chooses one adjacent valid decisecond value, verifies it, and restores the
recorded baseline. Optional explicit test values are:

- `--maintenance-interval 150..36000`
- `--write-interval-factor -128..127`
- `--write-co2-filter 0..255`
- `--write-operating-mode 0..3`
- `--write-part-name-hex <exactly 32 hex digits>`

For each selected target the sequence is:

1. require the full snapshot, a parsed target baseline, and a fresh clean
   mutation diagnostic; reject an explicit test value equal to that baseline;
2. issue one typed test write;
3. independently read back the test value;
4. require `VERIFIED`, `unresolved=false`, exact target and exact element
   counts and the target's exact register range;
5. capture a second complete 256-byte image before restoration and fail if
   any non-volatile byte outside the selected typed target changed;
6. issue a new typed write of the immutable recorded baseline;
7. read back the baseline and require another exact verified diagnostic;
8. capture the final 256-byte image and fail unexpected non-volatile changes.

An operating-mode test is admitted only when the recorded baseline is itself
inside the typed CLI restore range `0..3`. Historical or vendor-specific raw
mode values outside that range are recorded, but the runner refuses to change
them because it could not safely restore them through the same typed API.
The same preflight rule requires a recorded interval in `150..36000` and a
recorded address in `0..7` before their first mutation.

The mutation diagnostic and pre-restore image use these exact allowlists:

| Typed target | Custom-memory range |
| --- | --- |
| CO2 offset | `0x58..0x59` |
| CO2 gain | `0x5A..0x5B` |
| Part name | `0xB0..0xBF` |
| Bus address | `0xC0` |
| Global interval | `0xC6..0xC7` |
| CO2 interval factor | `0xCB` |
| CO2 filter | `0xD3` |
| Operating mode | `0xD8` |
| Auto-adjust | `0xD9` |

Only the selected range and documented volatile/read-dependent addresses
`0xC1`, `0xD9`, `0xFE`, and `0xFF` may differ in a pre-restore comparison.

If a destructive command, readback, or diagnostic is failed, missing, or
uncertain, the runner latches the failure and sends no later write—including
automatic restoration. Read-only evidence and an explicitly selected
target-specific resync remain possible. An operator must then inspect the
checkpoint, establish actual state, and authorize a separate recovery run.
This prevents a guessed “cleanup” write from compounding an unknown mutation.

## Calibration Writes

Offset/gain tests are separate from ordinary configuration and require all
persistent confirmations plus:

```powershell
--include-calibration-writes `
--confirm-calibration-writes `
--write-co2-offset <value>       # and/or
--write-co2-gain <value>
```

The live phrase is:

```text
RUN EE871 CALIBRATION WRITES
```

The typed test/readback/diagnostic/restore procedure is the same as above, but
restoring register values does not prove calibration accuracy. Run it only with
approved reference conditions and calibration authority. Calibration runs
reject interval, factor, filter, mode, and part-name test options; they do not
perform the default interval mutation.

## Bus-Address Candidate and Restoration

Address HIL is a dedicated run:

```powershell
python tools/ee871_hil_runner.py --port COM7 `
  --include-address-change `
  --candidate-address 1 `
  --confirm-address-change `
  --confirm-address-restore `
  --board "ESP32-S3" `
  --target-name "ex_bringup_s3" `
  --operator "name" `
  --sensor-id "bench-ee871-01" `
  --fixture-id "isolated-address-fixture-01" `
  --power-procedure "sensor-only-procedure-id" `
  --electrical-authority "approved-procedure-id"
```

It requires `RUN EE871 ADDRESS CHANGE` before the candidate request and the
separate in-sequence phrase `RESTORE THE RECORDED EE871 ADDRESS` before the
second address-change workflow.

The runner:

1. snapshots the original address and all custom memory;
2. issues one candidate write and requires
   `PERSISTENT_STATE_UNCERTAIN`, `BUS_ADDRESS`, `ACKNOWLEDGED`, and retained
   candidate evidence;
3. asks the operator to keep the controller/object powered and perform only the
   approved sensor address-activation procedure, then requires the exact
   in-sequence activation phrase;
4. uses `addr rebegin <candidate>` to call `end()`, explicitly configure the
   retained candidate, `begin()`, and `resyncPersistentConfig()` without a
   scan;
5. verifies the candidate and captures a complete pre-restore image that may
   differ only at address register `0xC0` and documented volatile bytes;
6. after independent restore authorization, repeats the full procedure back to
   the recorded address and compares the final snapshot.

Every address phase is admitted only after the preceding diagnostic, operator
confirmation, rebegin/resync, and readback passed. Any critical failure latches
the workflow and blocks later activation, rebegin, or restoration steps.

Never use `scan`, guess an address, or read through the old session after the
candidate request. A controller reboot loses RAM mutation intent and cannot
complete this same-object reconciliation proof.

## Auto-Adjust

Auto-adjust is non-replayable, non-cancellable, and non-restorable. It has a
separate exact opt-in:

```powershell
python tools/ee871_hil_runner.py --port COM7 `
  --include-auto-adjust `
  --confirm-auto-adjust `
  --board "ESP32-S3" `
  --target-name "ex_bringup_s3" `
  --operator "name" `
  --sensor-id "bench-ee871-01" `
  --fixture-id "calibration-fixture-01" `
  --electrical-authority "vendor-approved-calibration-procedure"
```

The live phrase is:

```text
RUN EE871 AUTO ADJUST ONCE
```

The plan requires an idle baseline, starts auto-adjust exactly once, records
pre/post mutation evidence, observes status, runs read-only resync, and captures
a final forensic image. It never retries, cancels, acknowledges uncertainty,
or claims to restore calibration. The one-shot command is sent only after the
operator enters the exact controlled-conditions phrase requested in sequence.

## Sensor Power-Cycle and Stuck-Line Plans

Run hazardous plans separately from all mutation plans.

Sensor-only lifecycle:

```powershell
python tools/ee871_hil_runner.py --port COM7 `
  --include-power-cycle --confirm-power-cycle `
  --board "ESP32-S3" --target-name "ex_bringup_s3" `
  --operator "name" --sensor-id "bench-ee871-01" `
  --fixture-id "sensor-power-fixture-01" `
  --power-procedure "sensor-only-procedure-id" `
  --electrical-authority "approved-procedure-id"
```

The controller and serial session stay powered. The operator cycles only the
named sensor rail using the recorded procedure, prevents back-powering through
E2 pull-ups/level shifting, then the runner records levels, explicit recovery,
health, and dirty state. Controller+sensor USB restart/reconnect automation is
not claimed by this plan.

SDA-low and SCL-low fault-jig coverage:

```powershell
python tools/ee871_hil_runner.py --port COM7 `
  --include-stuck-line --confirm-stuck-line `
  --board "ESP32-S3" --target-name "ex_bringup_s3" `
  --operator "name" --sensor-id "bench-ee871-01" `
  --fixture-id "open-drain-fault-jig-01" `
  --electrical-authority "approved-procedure-id"
```

The plan runs the two faults separately. Each phase records levels before,
during, and after the fault; checks bounded `buscheck`, tracked `status`, and
`libreset` results; releases the reviewed open-drain/current-limited jig; then
recovers and checks health. Never force an E2 line high. A logic analyzer or
oscilloscope remains required to prove the millisecond timing bound.

Operator steps remain `OPERATOR_REVIEW_REQUIRED`; parser success cannot replace
physical fixture and waveform evidence. Skipping or mistyping any required
operator transition aborts the run, so a later phase cannot proceed with a
fault jig or address/power state left unresolved.

Live fault runs also require the exact runtime phrases
`RUN EE871 SENSOR POWER CYCLE` or `RUN EE871 STUCK LINE FAULTS`.

## Safety and Plan Isolation

The runner rejects combinations of persistent, address, auto-adjust,
unplug/replug, stuck-line, and power-cycle plans. Run one hazardous group at a
time on dedicated hardware.

Live hazardous runs require non-placeholder:

- `--board`
- `--target-name`
- `--operator`
- `--sensor-id`
- `--fixture-id`
- `--electrical-authority`
- `--power-procedure` for address and sensor power-cycle plans

Use `--dry-run` to inspect a plan without serial I/O or hardware claims.

## Artifacts and Verdicts

Every run creates a timestamped directory containing:

- `serial_transcript.txt`
- `summary.json`
- `summary.md`
- `checkpoint.json`
- baseline JSON/hex files when a full baseline was captured

Verdicts:

- `PASS`: all selected automatic assertions passed.
- `FAIL`: a timeout or semantic assertion failed.
- `OPERATOR_REVIEW_REQUIRED`: physical/operator evidence is still required.
- `INCOMPLETE`: dry run, skipped step, or no complete result.

Exit codes are `0`, `1`, `2`, and `3`, respectively.

Parser and planner tests:

```powershell
python -m unittest discover -s test -p "*hil_runner_parser.py"
```

No hardware, calibration, power-cycle, stuck-line, or auto-adjust result should
be claimed unless the corresponding raw artifacts and physical evidence were
actually captured.
