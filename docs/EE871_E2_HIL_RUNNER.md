# EE871-E2 Python HIL Runner

Last updated: 2026-07-31

`tools/ee871_hil_runner.py` drives the EE871 example serial CLI and records
repeatable HIL evidence. The default plan is non-persistent. A runner `PASS`
means the selected serial CLI transcript matched parser expectations; it is not
a CO2 accuracy, calibration, long-soak, fault-tolerance, or production-readiness
claim without the matching bench record.

Recorded bench evidence is summarized in
[EE871_E2_HARDWARE_VALIDATION_MATRIX.md](EE871_E2_HARDWARE_VALIDATION_MATRIX.md).
The current example/HIL build pin is pioarduino `55.03.311`,
Arduino-ESP32 `3.3.11`, and ESP-IDF `5.5.5`. Its clean-commit COM20 run passed
184/184 safe, extended, and niche commands with final READY state, zero
transport failures, and clean persistent state. The prior `55.03.39`
firmware's targeted HIL passed 144/144, and a serial-only discriminator passed
10,000/10,000 identical 201-byte `dirty` replies. Because `dirty` performs no
E2 operation, that discriminator qualifies CLI framing only.

An accelerated 543.594-second scheduled-read regression then completed 108
sample cycles with 564 ordinary passes, two recorded first-attempt
`SCHEDULED_CONTROL_NACK_RECOVERED` outcomes, and zero hard failures, reviews,
skips, reconnects, or transport-counter regressions. This validates the narrow
harness policy over that interval; it is not a completed long soak.

The historical eight-hour soak used pioarduino `54.03.20` and
Arduino-ESP32 `3.2.0`. It remains a strict FAIL: 29 responses stalled
mid-line in HWCDC and 11 scheduled MV3 reads received a real NACK on the
`0xC1` control byte. The old runner incorrectly labeled those complete NACK
responses as review-required because it looked for the successful value line
before evaluating the parsed status. Arduino-ESP32
[PR #12606](https://github.com/espressif/arduino-esp32/pull/12606) documents
the matching HWCDC TX lost-wakeup/data-loss defect and fixes it in 3.3.9.
No completed long-soak result is claimed for the current platform.

## Default Safe Run

```powershell
python tools/ee871_hil_runner.py --port COM7 --output-dir hil_logs
```

Common serial arguments:

- `--port` serial port, for example `COM7` or `/dev/ttyUSB0`.
- `--baud` defaults to `115200`.
- `--timeout` defaults to `8` seconds for initial serial drain.
- `--command-timeout` defaults to `20` seconds for ordinary commands.
- `--idle` defaults to `0.35` seconds after a complete CLI prompt, allowing
  native USB CDC output to settle before the next command.
- `--output-dir` defaults to `hil_logs`.
- `--address` / `--device-address` records expected E2 address metadata only.
- `--dry-run` writes artifacts without opening serial.

The default safe sequence is:

```text
version
help
probe
read
selftest
drv
dirty
stress 50
drv
dirty
```

This sequence avoids persistent configuration writes. `probe` is diagnostic-only
by driver contract, while `read`, `selftest`, and `stress` are tracked operations
and can update driver health counters. `dirty` must remain clean for a normal
safe run.

Live command completion requires the CLI prompt and, for value reads, the
command-specific value line. If command framing times out, the runner stops the
remaining plan so later commands cannot be credited with shifted responses.
The serial port is opened with DTR and RTS already deasserted so attaching the
runner does not intentionally reset native-USB ESP32 targets.

Live serial runs require `pyserial`; install it in the active Python
environment:

```powershell
python -m pip install pyserial
```

Dry-run and parser tests use only the Python standard library.

## Extended Safe Plan

Append extended safe operations:

```powershell
python tools/ee871_hil_runner.py --port COM7 --include-extended
```

`--extended-safe` is accepted as an alias for `--include-extended`.

Extended options:

- `--read-loop-count N` controls repeated safe `read` commands.
- `--cycle-loop-count N` controls repeated `probe` / `read` / `selftest` cycles.

The extended plan also includes a bounded `stress 500`, `recover`, `drv`, and
`dirty`. These commands still avoid persistent writes.

## Niche Safe Plan

Append the fixed identity, guard, GPIO/E2 diagnostic, trace/sniffer, and mixed
stress plan:

```powershell
python tools/ee871_hil_runner.py --port COM20 --include-niche
```

This plan reads identity, firmware, capabilities, serial, part name, address,
interval, factor, filter, and operating mode; checks out-of-range address,
interval, and mode requests; runs idle-level, clock, address-scan, timing,
library-command, and full diagnostics; verifies the fixed trace buffer has no
drops; exercises the protocol sniffer; and finishes with
`stress_mix 500`, READY health, and clean persistent state.

The selected guard commands fail before a persistent write. The niche plan
does not write calibration, address, interval, filter, factor, part name, or
operating mode. Timing points below the documented 100 us half-cycle minimum
are explicitly reported as characterization, not supported settings.

## Serial-Only Framing Discriminator

`tools/ee871_serial_discriminator.py` repeats the state-only `dirty` command
without E2 traffic, verifies the exact runtime framework versions, and writes
only compact aggregate evidence:

```powershell
python tools/ee871_serial_discriminator.py --port COM20 `
  --count 10000 `
  --run-dir hil_logs/serial_discriminator_10000 `
  --expected-arduino-version 3.3.11 `
  --expected-idf-version v5.5.5 `
  --expected-library-version 1.0.1
```

It requires every round trip to parse as clean and records response-length and
SHA-256 histograms. This isolates native-USB/CLI framing; it is not sensor
transport or long-soak evidence.

## Read-Only Soak And Scheduled-NACK Retry

`tools/ee871_soak_runner.py` runs a checkpointed, non-persistent soak with
scheduled MV3/MV4/status samples, health/dirty checks, and periodic
`stress_mix` blocks:

```powershell
python tools/ee871_soak_runner.py --port COM20 `
  --duration-hours 8 `
  --board ESP32-S3-PSRAM `
  --target-name ex_bringup_s3
```

The bench EE871 returned complete control-byte NACK replies on a small number
of scheduled MV3 reads. Their phase clustering is consistent with an internal
sensor activity window, but a NACK does not expose its sensor-internal cause.
The generic E2 operating-mode specification allows measurement-priority NACKs,
while the EE871 application note lists EE871 as an exception that can process
enquiries while measuring. The core therefore returns the NACK without retry
or diagnosis; retry cadence belongs to the application.

The soak harness applies a narrower policy:

- Only a scheduled `co2fast` or `co2avg` sample is eligible.
- The first result must be a parsed `NACK` with the `Control byte NACK`
  diagnostic.
- The command is retried at most once after
  `--scheduled-nack-retry-ms`; the default is 1,500 ms and zero disables it.
- Both the original attempt and retry remain in the transcript.
- A successful retry records `SCHEDULED_CONTROL_NACK_RECOVERED`; a failed retry
  remains a hard failure.
- Preflight, status, health, dirty, maintenance, and `stress_mix` operations
  are never covered by this exception.

While a run is active, `checkpoint.json` is atomically replaced after every
record. On normal finalization, `summary.json` receives the same final payload
and the now-redundant checkpoint is removed.

This bounded harness policy does not alter the driver's per-attempt health
accounting; the original attempt and retry are tracked normally. It must not be
described as a core-library retry, and it does not turn a serial framing
timeout, another NACK context, or an exhausted retry into PASS.

## Persistent Writes

Persistent writes are disabled by default. They require both the persistent plan
flag and an explicit confirmation flag:

```powershell
python tools/ee871_hil_runner.py --port COM7 `
  --include-persistent-writes `
  --confirm-persistent-writes
```

For scripted wrappers, `--confirm-persistent-writes` may also be passed the exact
text `I UNDERSTAND EE871 PERSISTENT WRITES`. For live serial runs, the runner
also prompts for:

```text
RUN EE871 PERSISTENT WRITES
```

Persistent-write options:

- `--maintenance-interval <150..36000>` writes the measurement interval in
  deciseconds. If omitted, the plan reads and rewrites the parsed current value.
- `--write-co2-offset <-32768..32767>` writes persistent CO2 offset.
- `--write-co2-gain <0..65535>` writes persistent CO2 gain.

Only run these on a bench sensor where configuration changes are acceptable and
original values have been recorded. The plan records `dirty` and `resync` output
around persistent operations.

## Operator Fault Prompts

Operator fault flags insert prompt steps and therefore require human review:

```powershell
python tools/ee871_hil_runner.py --port COM7 --include-unplug-replug
python tools/ee871_hil_runner.py --port COM7 --include-stuck-line
python tools/ee871_hil_runner.py --port COM7 --include-power-cycle
```

At each prompt, type:

- `done` after applying or restoring the requested fault.
- `skip` to mark the step skipped.
- `abort` to stop the run.

Fault prompts do not induce hardware faults by themselves. The resulting verdict
is `OPERATOR_REVIEW_REQUIRED` unless a parser-detected failure takes precedence.

## Artifacts

Each invocation creates a timestamped directory under `--output-dir`, for
example `hil_logs/ee871_20260601T094218Z/`, containing:

- `serial_transcript.txt` - raw serial transcript plus per-command result lines.
- `summary.json` - metadata, parsed state, result counts, final verdict, and a
  compact command ledger. Ordinary PASS payloads are omitted; abnormal rows
  retain a bounded response excerpt.
- `summary.md` - operator-friendly summary and artifact index.

The JSON and markdown summaries include the claim boundary. Record board model,
target firmware, sensor serial/part, wiring, supply, pull-ups, level shifter,
ambient conditions, and operator notes alongside these artifacts when converting
a run into a formal validation record.

Keep failed or review-required runs only when they are useful evidence. For a
formal PASS record, retain `summary.md` plus compact `summary.json` when
machine-readable rows add value. The raw transcript may be removed after
checking that the summaries contain the full command ledger and final state.
Retain raw serial for unique negative evidence such as truncation, timeout,
NACK, protocol corruption, or timing evolution. Do not commit binary monitor
captures or firmware build artifacts.

## Verdicts

- `PASS` - every selected automated command passed parser expectations.
- `FAIL` - a command timed out or a parser detected a failure such as non-OK
  status, selftest failures, stress errors, offline health, or dirty persistent
  state where clean state was required.
- `OPERATOR_REVIEW_REQUIRED` - operator evidence is required or an expected token
  was missing even though serial output was captured.
- `INCOMPLETE` - dry run, skipped steps, no results, or mixed non-pass outcomes
  that did not produce a hard failure.

The process exits `0` only for `PASS`; `FAIL` exits `1`,
`OPERATOR_REVIEW_REQUIRED` exits `2`, and `INCOMPLETE` exits `3`.

## Parser Tests

Host-only parser tests live under `test/` and cover:

- ANSI-colored `selftest` output.
- `stress` and `stress_mix` summaries.
- `drv` health output.
- `dirty` persistent-configuration output.
- Dirty/stress validator failures.
- Fully terminated prompt framing, including split prompt/newline chunks.
- Parsed non-OK status taking precedence over a missing success-value token.
- The soak harness's narrowly scoped scheduled control-byte NACK retry result.
- Exact persistent-write confirmation parsing.

Run them with:

```powershell
python -m unittest discover -s test -p "*hil_runner_parser.py"
```
