# EE871-E2 Python HIL Runner

`tools/ee871_hil_runner.py` drives the EE871 example serial CLI and records
repeatable HIL evidence. The default plan is non-persistent. A runner `PASS`
means the selected serial CLI transcript matched parser expectations; it is not
a CO2 accuracy, calibration, long-soak, fault-tolerance, or production-readiness
claim without the matching bench record.

## Default Safe Run

```powershell
python tools/ee871_hil_runner.py --port COM7 --output-dir hil_logs
```

Common serial arguments:

- `--port` serial port, for example `COM7` or `/dev/ttyUSB0`.
- `--baud` defaults to `115200`.
- `--timeout` defaults to `8` seconds for initial serial drain.
- `--command-timeout` defaults to `20` seconds for ordinary commands.
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

Live serial runs currently use `pyserial` if it is not already present in the
active Python environment:

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
- `summary.json` - metadata, parsed state, result counts, final verdict, and
  command summaries without raw payload duplication.
- `summary.md` - operator-friendly summary and artifact index.

The JSON and markdown summaries include the claim boundary. Record board model,
target firmware, sensor serial/part, wiring, supply, pull-ups, level shifter,
ambient conditions, and operator notes alongside these artifacts when converting
a run into a formal validation record.

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
- Exact persistent-write confirmation parsing.

Run them with:

```powershell
python -m unittest discover -s test -p "*hil_runner_parser.py"
```
