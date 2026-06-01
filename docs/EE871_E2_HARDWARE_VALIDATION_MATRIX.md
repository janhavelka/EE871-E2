# EE871-E2 Hardware Validation Matrix

Date: 2026-06-01
Branch: `hardening/ee871-e2-industry-readiness`

This matrix started as a hardware validation plan and now also records completed
bench evidence where available. Default status remains `NOT RUN` until a test is
executed and recorded with board, firmware, serial port, sensor, wiring, supply,
pull-ups, and observed output.

For repeatable evidence capture, use `tools/ee871_hil_runner.py` after flashing
the diagnostic CLI. The runner records a raw serial transcript, structured JSON,
and a Markdown summary. A runner `PASS` applies only to the selected automated
serial command groups; it does not prove CO2 accuracy, warm-up suitability,
fault tolerance, long-soak stability, calibration validity, or production
readiness.

Allowed statuses:

- `NOT RUN` - scenario has not been executed.
- `PASS` - scenario was executed and met expected behavior.
- `FAIL` - scenario was executed and did not meet expected behavior.
- `BLOCKED` - scenario could not be run because a prerequisite is missing.
- `NOT APPLICABLE` - scenario does not apply to the tested setup.

## Safe Bring-Up CLI Recipe

Use the Arduino PlatformIO CLI example or the native ESP-IDF diagnostic/basic
bring-up CLI. Both expose the same user-visible command surface.

Safe non-persistent sequence:

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

Manual expanded safe sequence:

```text
version
drv
dirty
buscheck
levels
probe
drv
status
read
co2fast
co2avg
features
caps
fw
e2spec
selftest
dirty
stress_mix 20
dirty
recover
resync
dirty
drv
```

Notes:

- This sequence avoids persistent configuration writes. It is not strictly
  read-only: tracked reads update driver health, and `recover` may issue bus
  recovery clocks before probing.
- `probe` is diagnostic-only and should not change health counters.
- `status` can trigger a new EE871 measurement when the previous sample is old.
- `read` reads the CO2 averaged value; `co2fast` reads MV3; `co2avg` reads MV4.
- `dirty` is state-only and should report `persistentConfigDirty: no` on a
  clean startup and after normal safe commands.
- `resync` calls `resyncPersistentConfig()`. It performs verified persistent
  configuration reads and must only clear dirty state when that API returns OK.
- Normal `probe`, `read`, `selftest`, `stress`, and `stress_mix` commands should
  not create persistent dirty state.
- Record raw command output and timestamps for each board/sensor combination.
- The automatic equivalent for the default safe sequence is:

  ```bash
  python tools/ee871_hil_runner.py --port COMx
  ```

- Extended safe repeatability can be captured with:

  ```bash
  python tools/ee871_hil_runner.py --port COMx --include-extended
  ```

Persistent-write commands are bench-only:

```text
interval
interval <150..36000>
offset
offset <signed_ppm>
gain
gain <0..65535>
partname
partname <text>
addr
addr <0-7>
factor
factor <value>
filter
filter <value>
mode
mode <0..3>
reg write <addr> <value>
autoadj start
```

Warnings before persistent writes:

- Persistent writes may change sensor configuration and may persist across power
  cycles.
- Persistent writes can have long delays and flash/endurance implications.
- Only run persistent-write tests on a bench sensor where configuration changes
  are acceptable and the original values have been recorded.
- `reg write <addr> <value>` can write arbitrary custom memory, including
  persistent/configuration addresses; treat it as a bench-only operation.
- After any failed multi-byte persistent write, run `dirty` before trusting
  persistent configuration. Use `resync` only to clear dirty state after the
  driver confirms persistent fields are readable and coherent.
- Induce or observe dirty state through the fake/native tests or dedicated test
  firmware unless deliberately running the bench persistent-write matrix below.
- The HIL runner requires both `--include-persistent-writes` and
  `--confirm-persistent-writes` before it sends persistent write commands. It
  rewrites the parsed current measurement interval by default, or writes
  `--maintenance-interval <deciseconds>` when provided. CO2 offset/gain writes
  require explicit values.

Example:

```bash
python tools/ee871_hil_runner.py --port COMx --include-persistent-writes --confirm-persistent-writes
```

Operator fault flows remain review-required evidence:

```bash
python tools/ee871_hil_runner.py --port COMx --include-unplug-replug
python tools/ee871_hil_runner.py --port COMx --include-stuck-line
python tools/ee871_hil_runner.py --port COMx --include-power-cycle
```

## Current Evidence Status

Safe EE871 HIL evidence was recorded on 2026-06-01 using the Arduino diagnostic
CLI on `COM17`.

- Board/target: ESP32-S3, PlatformIO `ex_bringup_s3`.
- Upload command: `python -m platformio run -e ex_bringup_s3 -j 1 -t upload --upload-port COM17`.
- Firmware/library: firmware build `Jun  1 2026 20:57:04`, EE871 library
  `0.3.0 (84a46b6, 2026-06-01 20:57:01, clean)`.
- Safe default HIL: PASS, 10 PASS / 0 FAIL / 0 SKIP / 0 review.
- Extended safe HIL: PASS, 33 PASS / 0 FAIL / 0 SKIP / 0 review.
- Manual resync check: PASS, `dirty`, `resync`, `dirty`.
- Safe `read`: OK, CO2 averaged value `567 ppm` in the safe-default run.
- `selftest`: PASS, `pass=27 fail=0 skip=0`.
- `drv`: READY, online yes, zero consecutive failures.
- `stress 50`: PASS, `50/50`, 0 errors.
- `stress 500`: PASS, `500/500`, 0 errors.
- Persistent dirty state stayed clean: `persistentConfigDirty: no`,
  `resyncNeeded: no` before/after safe stress and after manual `resync`.
- `rv` is not advertised by help; help advertises `version / ver`. No alias was
  added during this validation pass.

Artifacts:

- Safe default transcript:
  `hil_results/safe_default/ee871_20260601T185912Z/serial_transcript.txt`
- Safe default JSON/Markdown:
  `hil_results/safe_default/ee871_20260601T185912Z/summary.json`,
  `hil_results/safe_default/ee871_20260601T185912Z/summary.md`
- Extended safe transcript:
  `hil_results/extended_safe/ee871_20260601T185921Z/serial_transcript.txt`
- Extended safe JSON/Markdown:
  `hil_results/extended_safe/ee871_20260601T185921Z/summary.json`,
  `hil_results/extended_safe/ee871_20260601T185921Z/summary.md`
- Manual resync transcript:
  `hil_results/manual_resync/ee871_20260601T190024Z/serial_transcript.txt`
- Manual resync JSON/Markdown:
  `hil_results/manual_resync/ee871_20260601T190024Z/summary.json`,
  `hil_results/manual_resync/ee871_20260601T190024Z/summary.md`

Persistent configuration validation was recorded on 2026-06-01 after explicit
bench-unit approval. The bench run changed only the measurement interval, then
restored the baseline:

- Local time: 2026-06-01 21:35 CEST (`2026-06-01T19:35:00Z`).
- Serial port: `COM17`, baud `115200`.
- Board/target: ESP32-S3, PlatformIO `ex_bringup_s3`.
- Firmware/library: firmware build `Jun  1 2026 20:57:04`, EE871 library
  `0.3.0 (84a46b6, 2026-06-01 20:57:01, clean)`.
- Baseline persistent values: interval `150 ds` (`15.0 s`), CO2 interval
  factor `85`, operating mode `0x55` (low power, measurement priority), bus
  address `0`, part name `EE871`, serial `1920935602368A..`
  (`31 39 32 30 39 33 35 36 30 32 33 36 38 41 00 00`), CO2 offset `0 ppm`,
  CO2 gain `32768`.
- Baseline dirty state was clean through `drv`: `persistentConfigDirty: no`,
  `persistentConfigDirtyError: OK`, `resyncNeeded: no`.
- Commands run:
  `version`, `interval`, `factor`, `mode`, `addr`, `partname`, `serial`,
  `offset`, `gain`, `drv`, `interval 160`, `interval`, `dirty`, `resync`,
  `dirty`, `interval 150`, `interval`, `dirty`, `resync`, `dirty`, final
  `interval`, `factor`, `mode`, `addr`, `partname`, `serial`, `offset`, `gain`,
  `drv`.
- Results: all 31 captured steps passed; `interval 160` returned OK and read
  back `160 ds`; `interval 150` returned OK and restored/read back `150 ds`;
  `dirty` stayed clean after the test write, after `resync`, after restore, and
  at final `drv`.
- CO2 offset/gain were read only and not modified. Bus address was read only and
  not modified because no automated post-power-cycle retarget/recovery path was
  used during this run.
- Power-cycle persistence was not performed; no operator power-cycle step was
  executed during the automated run.
- Failed-write/operator recovery was not physically induced; native fake tests
  remain the evidence for partial persistent-write dirty/resync behavior.

Artifacts:

- Persistent validation transcript:
  `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/serial_transcript.txt`
- Persistent validation JSON/Markdown:
  `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/summary.json`,
  `hil_results/persistent_config_validation/ee871_20260601T193500Z_interval_restore/summary.md`

Physical fault/jig tests were not run.

## Board Matrix

| ID | Board | Framework/example | Target | Sensor | Pull-ups/level shift | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| B-S3-A | ESP32-S3 dev board | `examples/01_basic_bringup_cli` | `ex_bringup_s3` | EE871-E2 bench sensor | External pull-ups, level shifter as required | PASS | 2026-06-01 on `COM17`; safe default and extended safe HIL PASS; persistent interval write/readback/restore PASS. GPIOs from firmware: DATA=6, CLOCK=7. |
| B-S2-A | ESP32-S2 dev board | `examples/01_basic_bringup_cli` | `ex_bringup_s2` | EE871-E2 bench sensor | External pull-ups, level shifter as required | NOT RUN | Record GPIOs, supply, cable length. |
| B-S3-IDF | ESP32-S3 dev board | `examples/idf/basic_bringup` | `esp32s3` | EE871-E2 bench sensor | External pull-ups, level shifter as required | NOT RUN | Requires local or CI pure ESP-IDF build. |
| B-S2-IDF | ESP32-S2 dev board | `examples/idf/basic_bringup` | `esp32s2` | EE871-E2 bench sensor | External pull-ups, level shifter as required | NOT RUN | Requires local or CI pure ESP-IDF build. |

## Functional Matrix

| ID | Scenario | Board(s) | CLI/API sequence | Expected behavior | Status | Evidence to capture |
| --- | --- | --- | --- | --- | --- | --- |
| F-01 | Power-up `begin()` with sensor present | S2, S3 | Power cycle, open monitor, inspect boot output, `drv`, `dirty` | Device initializes or reports a precise non-OK `Status`; driver state is READY on success and persistent dirty state is clean. | PASS | Safe HIL boot/initial output plus `drv`/`dirty`: READY, online yes, dirty no. |
| F-02 | Probe no-health-side-effects | S2, S3 | `drv`, `probe`, `drv` | Successful or failed `probe` does not change health counters/state. | PASS | Safe and extended HIL `probe`: Status OK; health stayed READY with zero failures. |
| F-03 | Status read | S2, S3 | `status`, `drv` | Status byte read completes or returns bounded error; tracked success/failure updates health as documented. | NOT RUN | `status` output and health counters. |
| F-04 | CO2 averaged read | S2, S3 | `read`, `co2avg` | MV4 averaged value is reported, or a precise bounded error is returned. | PASS | Safe default `read`: Status OK, CO2 avg `567 ppm`. |
| F-05 | CO2 fast read | S2, S3 | `co2fast` | MV3 fast-response value is reported, or a precise bounded error is returned. | NOT RUN | CO2 ppm value and status. |
| F-06 | PEC success on normal reads | S2, S3 | `id`, `status`, `read`, `features` | Normal reads do not report `PEC_MISMATCH`. | NOT RUN | Command output. |
| F-07 | Feature/cache sanity | S2, S3 | `features`, `caps`, `cfg` | Capability output is internally consistent and guards unsupported writes. | NOT RUN | Feature bytes and booleans. |
| F-08 | Warm-up behavior | S2, S3 | Power cycle, run `status`, `read`, `co2avg` every 30 s during first 10 min | Operator/application validation treats readings as warm-up data until EE871 warm-up has elapsed. | NOT RUN | Timestamped ppm trend. |
| F-09 | Stale sample behavior | S2, S3 | Compare `status`, wait 5-10 s, `co2avg`, repeat after >10 s | Status-triggered measurement behavior and sample freshness are observable and documented. | NOT RUN | Timestamped status/CO2 output. |
| F-10 | Safe self-test | S2, S3 | `dirty`, `selftest`, `dirty` | Safe commands complete with expected pass/fail report; no persistent settings are changed and persistent dirty remains clean. | PASS | Safe and extended HIL: `selftest` pass=27 fail=0 skip=0; dirty stayed clean. |
| F-11 | Mixed read stress | S2, S3 | `dirty`, `stress_mix 100`, `dirty` | No hangs; failures, if any, are bounded and health counters match output; persistent dirty remains clean. | NOT RUN | Stress summary and dirty output. |
| F-12 | Repeated CO2 read stress | S2, S3 | `dirty`, `stress 100`, `dirty` | No hangs; CO2 read success rate and health counters are recorded; persistent dirty remains clean. | PASS | Safe HIL `stress 50`: 50/50, 0 errors; extended HIL `stress 500`: 500/500, 0 errors; dirty stayed clean. |
| F-13 | Dirty resync command on coherent config | S2, S3 | `dirty`, `resync`, `dirty` | `resync` returns precise status; if OK, dirty remains or becomes clean only through `resyncPersistentConfig()`. | PASS | Manual resync artifact: pre dirty clean, `resync` Status OK, post dirty clean. |

## Persistent Configuration Matrix

Run these only on a bench sensor after recording original values.

| ID | Scenario | Board(s) | CLI/API sequence | Expected behavior | Status | Evidence to capture |
| --- | --- | --- | --- | --- | --- | --- |
| P-01 | Measurement interval write/readback | S2, S3 | `interval`, `dirty`, record value, `interval <bench_value>`, `interval`, `dirty` | Write returns OK and readback matches; on failure, `dirty` reports whether persistent state may be partial. | PASS | 2026-06-01 on ESP32-S3 `COM17`: baseline `150 ds`, wrote `160 ds`, read back `160 ds`, dirty clean; restored `150 ds`, read back `150 ds`, dirty clean. |
| P-02 | Measurement interval power-cycle persistence | S2, S3 | Run P-01, power cycle sensor and MCU, `interval` | Value persists across power cycle or documented sensor behavior explains difference. | NOT RUN | No operator power-cycle step was executed during the automated persistent validation run. |
| P-03 | CO2 offset write/readback | S2, S3 | `offset`, `dirty`, record value, `offset <bench_value>`, `offset`, `dirty` | Write returns OK and readback matches; dirty diagnostics checked on failure. | NOT RUN | Read-only baseline/final value recorded as `0 ppm`; no calibration write was performed. |
| P-04 | CO2 gain write/readback | S2, S3 | `gain`, `dirty`, record value, `gain <bench_value>`, `gain`, `dirty` | Write returns OK and readback matches; dirty diagnostics checked on failure. | NOT RUN | Read-only baseline/final value recorded as `32768`; no calibration write was performed. |
| P-05 | Part name write/readback | S2, S3 | `partname`, `dirty`, record value, `partname <bench_text>`, `partname`, `dirty` | Write returns OK and readback matches; dirty diagnostics checked on failure. | NOT RUN | Before/write/after output plus dirty diagnostics. |
| P-06 | Bus address write | S2, S3 | `addr`, record value, `addr <bench_addr>`, power cycle, `scan`; then rebuild/reconfigure firmware for the new address or use a dedicated test wrapper | Address change behaves as documented and does not retarget the current session until power cycle. | NOT RUN | Address read/scan output, configured-address follow-up output. |

## Fault And Recovery Matrix

| ID | Scenario | Board(s) | CLI/API sequence | Expected behavior | Status | Evidence to capture |
| --- | --- | --- | --- | --- | --- | --- |
| R-01 | Wrong wiring or no sensor | S2, S3 | Disconnect sensor, boot, `probe`, `status`, `drv` | Initialization or reads fail with bounded non-OK status; no hang. | NOT RUN | Boot log, command output, health counters. |
| R-02 | Unplug/replug recovery | S2, S3 | Start connected, `read`, unplug, repeated `read`, replug, `recover`, `drv` | Tracked failures degrade/offline as configured; successful `recover` returns READY. | NOT RUN | Timestamped health transitions. |
| R-03 | SDA stuck low | S2, S3 | Use fault jig to pull SDA low, `buscheck`, `libreset`, `drv` | `BUS_STUCK` or precise bounded error; no unbounded wait. | NOT RUN | Jig setup and command output. |
| R-04 | SCL stuck low / clock stretch timeout | S2, S3 | Use fault jig to pull SCL low, `status`, `buscheck`, `libreset` | Timeout or `BUS_STUCK` within configured deadline; no hang. | NOT RUN | Timing notes and output. |
| R-05 | SDA forced high/no ACK | S2, S3 | Use fault jig/open line, `probe`, `status` | NACK/no-response error is bounded and health rules match `probe` vs tracked reads. | NOT RUN | Command output. |
| R-06 | Recovery clocks on stuck bus | S2, S3 | `busreset`, `libreset`, `buscheck` | Recovery clocks are issued and idle state is reported accurately. | NOT RUN | Output and line-level observation if available. |
| R-07 | Timing sweep | S2, S3 | `timing` | Supported timing range is identified without hangs; failures are bounded. | NOT RUN | Timing table output. |
| R-08 | Bus trace sanity | S2, S3 | `verbose 1`, `status`, `trace stats`, `verbose 0` | Trace captures bounded line activity and does not destabilize reads. | NOT RUN | Trace stats and sample trace. |

## Sign-Off Template

For each completed row, record:

- Date/time.
- Operator.
- Commit SHA and firmware build timestamp.
- Board model and target.
- Sensor part/serial number.
- GPIO pins, pull-up values, level shifter, supply voltage, cable length.
- CLI command transcript.
- Result status and notes.
