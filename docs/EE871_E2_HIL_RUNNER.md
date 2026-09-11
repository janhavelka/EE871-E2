# EE871-E2 HIL and Soak Tools

Last updated: 2026-09-11

`tools/ee871_hil_runner.py` drives the example serial CLI and records
repeatable hardware-in-the-loop evidence. Its default plan avoids persistent
writes. A runner PASS means the selected CLI responses matched parser
expectations; CO2 accuracy, calibration, fault tolerance, and long-run claims
need the corresponding bench evidence.

Completed campaigns, tested versions, and outstanding hardware validation are
maintained in the
[hardware validation matrix](EE871_E2_HARDWARE_VALIDATION_MATRIX.md).

## Setup and Default Run

Build and flash the intended example before attaching the runner. On Windows,
use the repository PlatformIO wrapper, for example
`.\scripts\pio.cmd run -e ex_bringup_s3`; build environments and platform pins
are defined in [platformio.ini](../platformio.ini). Close other serial monitors
so the runner owns the port.

Live serial tools require `pyserial` in the active Python environment. Dry
runs and parser tests use the Python standard library.

```powershell
python -m pip install pyserial
python tools/ee871_hil_runner.py --port COM7 --output-dir hil_logs
```

The default sequence is:

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

`probe` is health-neutral. `read`, `selftest`, and `stress` contain tracked E2
operations and can change health counters. A normal safe run requires clean
persistent state.

| Argument | Purpose / default |
|---|---|
| `--port` | Serial port, such as `COM7` or `/dev/ttyUSB0`. |
| `--baud` | 115200. |
| `--timeout` | Initial CLI synchronization timeout, 8 s. |
| `--command-timeout` | Ordinary command timeout, 20 s; long built-in commands have their own bounds. |
| `--idle` | Settle time after a complete prompt, 0.35 s. |
| `--output-dir` | Parent artifact directory, `hil_logs`. |
| `--address` / `--device-address` | Expected E2 address metadata; does not retarget the firmware. |
| `--board`, `--target-name`, `--operator` | Bench metadata. |
| `--dry-run` | Writes the planned artifacts without opening serial; verdict is INCOMPLETE. |

Each attachment opens the port with DTR/RTS deasserted and synchronizes using
`\ndirty\n`, which terminates any partial command and requests a response
without E2 traffic. The runner requires the `persistentConfigDirty` marker
before accepting a prompt. Successful value reads must include their value
line; a fully parsed error takes precedence over a missing success value.
A framing timeout stops the remaining plan to prevent shifted responses.

## Extended and Niche Plans

```powershell
python tools/ee871_hil_runner.py --port COM7 --include-extended --include-niche
```

`--include-extended` (alias `--extended-safe`) adds `stress 500`, repeated reads,
probe/read/selftest cycles, and recovery/health/dirty checks. Use
`--read-loop-count N` and `--cycle-loop-count N` to control repetition.

`--include-niche` adds identity/configuration reads, parameter guards,
GPIO/E2 diagnostics, trace/sniffer checks, and `stress_mix 500`. Guard commands
fail before writing. Neither plan writes calibration, address, interval,
filter, factor, part name, or operating mode. Timing points below the supported
100 us half-cycle minimum are characterization only.

## Serial Framing Discriminator

`tools/ee871_serial_discriminator.py` repeats the state-only `dirty` command
without E2 traffic. It checks clean responses and records response-length and
SHA-256 histograms, isolating CLI/native-USB framing from sensor transport.

```powershell
python tools/ee871_serial_discriminator.py --port COM7 `
  --count 10000 --run-dir hil_logs/serial_discriminator_10000 `
  --expected-library-version 1.1.0
```

Set `--expected-library-version`, `--expected-arduino-version`, and
`--expected-idf-version` to the versions actually flashed when exact version
checks are required. This run does not qualify E2 signaling or sensor behavior.

## Soak and Scheduled NACK Policy

`tools/ee871_soak_runner.py` runs a non-persistent soak with scheduled
MV3/MV4/status reads, health/dirty checks, and periodic `stress_mix` blocks:

```powershell
python tools/ee871_soak_runner.py --port COM7 `
  --duration-hours 8 --board ESP32-S3-PSRAM --target-name ex_bringup_s3
```

Defaults are one sample cycle per 60 s and `stress_mix 500` every 30 minutes.
Use `--sample-interval-seconds`, `--stress-period-minutes`, and `--stress-count`
to change these. Status reads can trigger another measurement only when the
global interval is >15 s and the previous measurement is >10 s old.

The harness has a separate, narrowly scoped retry policy:

- Only scheduled `co2fast` or `co2avg` commands are eligible.
- The first result must be a complete parsed `NACK` with the `Control byte NACK`
  diagnostic. A NACK alone does not identify its sensor-internal cause.
- One extra command is allowed after `--scheduled-nack-retry-ms` (default
  1500 ms; zero disables it). Both attempts stay in the transcript.
- A successful retry records `SCHEDULED_CONTROL_NACK_RECOVERED`; failure
  remains a hard failure. Preflight, status, health, dirty, maintenance,
  `stress_mix`, and serial framing failures are outside this exception.

The driver also supports opt-in `Config::readNackRetries`: 0..3 additional
MV3/MV4/status frame attempts after a control-byte NACK, clean STOP/idle checks,
and a fixed 1 ms HAL pause. Its optional application guard can veto retries.
The driver counts each final frame result once in health and retains separate
NACK/retry diagnostics; see the
[protocol retry contract](EE871_E2_Protocol_and_Register_Map.md#134-bounded-read-retries-and-health).
Default example configurations leave driver retries disabled.

Record both policies with the firmware configuration when reporting results.
A harness retry sends another full CLI command and is tracked normally; a
recovered harness result does not demonstrate an internal driver retry. To
isolate driver retry behavior, disable the harness policy with
`--scheduled-nack-retry-ms 0` and capture the driver diagnostics in the target
application.

The soak atomically replaces `checkpoint.json` after every record. Normal
finalization writes `summary.json` and removes the redundant checkpoint.

## Persistent Maintenance

Persistent plans require explicit flags and a live confirmation:

```powershell
python tools/ee871_hil_runner.py --port COM7 `
  --include-persistent-writes --confirm-persistent-writes
```

The live prompt requires `RUN EE871 PERSISTENT WRITES`.
`--confirm-persistent-writes` also accepts the exact argument
`I UNDERSTAND EE871 PERSISTENT WRITES` for scripted wrappers; this does not
bypass the live prompt.

| Option | Persistent change |
|---|---|
| `--maintenance-interval <150..36000>` | Interval in deciseconds; omitted means read and rewrite the current value. |
| `--write-co2-offset <-32768..32767>` | CO2 offset. |
| `--write-co2-gain <0..65535>` | CO2 gain. |

Use a bench sensor with recorded original settings. The plan checks `dirty`
and `resync` around writes and blocks maintenance writes when clean state has
not been established.

## Operator Fault Steps

`--include-unplug-replug`, `--include-stuck-line`, and `--include-power-cycle`
insert operator prompts. At each prompt, enter `done` after applying/restoring
the requested fault, `skip` to skip, or `abort` to stop. The tools do not induce
faults themselves. Operator steps require review unless a detected failure
takes precedence.

## Artifacts and Verdicts

Each HIL invocation creates a timestamped directory containing:

- `serial_transcript.txt`: serial responses and command results.
- `summary.json`: metadata, parsed state, counts, verdict, and compact command
  ledger; abnormal rows retain bounded excerpts.
- `summary.md`: readable summary and artifact index.

Record the exact firmware/library commits, retry settings, board, sensor
serial/part, wiring, supply, pull-ups, level shifter, and conditions with the
artifacts. Retain concise evidence that supports the validation matrix; raw
serial is particularly useful for unique NACK, timeout, truncation, corruption,
or timing failures. Keep build binaries and redundant successful transcripts
out of maintained documentation.

| Verdict | Meaning | Exit code |
|---|---|---:|
| PASS | Every selected automated command met parser expectations. | 0 |
| FAIL | Timeout or detected status, selftest, stress, health, or dirty-state failure. | 1 |
| OPERATOR_REVIEW_REQUIRED | Operator evidence needed or expected token missing. | 2 |
| INCOMPLETE | Dry run, skipped steps, no results, or remaining incomplete outcomes. | 3 |

Host parser tests cover status precedence, split prompt framing, health/dirty
and stress parsing, scheduled control-NACK policy, and persistent confirmation:

```powershell
python -m unittest discover -s test -p "*hil_runner_parser.py"
```
