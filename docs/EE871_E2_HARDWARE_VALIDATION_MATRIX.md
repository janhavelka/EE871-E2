# EE871-E2 Hardware Validation Matrix

Created: 2026-06-01
Last updated: 2026-07-31
Branch: `main`

This matrix started as a hardware validation plan and now also records completed
bench evidence where available. Default status remains `NOT RUN` until a test is
executed and recorded with board, firmware, serial port, sensor, and observed
output.

For repeatable evidence capture, use `tools/ee871_hil_runner.py` after flashing
the diagnostic CLI. The runner records raw serial, compact structured JSON,
and a Markdown summary; curated PASS evidence may discard redundant raw
output after the ledger is verified. A runner `PASS` applies only to the selected automated
serial command groups.

Current evidence summary:

- Current build pin: pioarduino `platform-espressif32` `55.03.311`,
  Arduino-ESP32 `3.3.11`, ESP-IDF `5.5.5`; ESP32-S3 and ESP32-S2 builds pass.
- Current ESP32-S3 Arduino diagnostic CLI on `COM20`, pioarduino
  `55.03.311`: 184/184 safe/extended/niche commands PASS from clean firmware
  `3bce89e`; final READY, 3,109 tracked successes, zero failures, persistent
  state clean, `stress 500` 500/500, and `stress_mix 500` 500/500.
- Current native-USB reattachment regression: 100/100 separate process
  sessions PASS; a new process completed 10,000/10,000 identical state-only
  replies immediately after full HIL closed COM20.
- Native ESP-IDF v6.0.1 example builds pass in GitHub Actions for ESP32-S3 and
  ESP32-S2.
- Prior ESP32-S3 Arduino diagnostic CLI on `COM20`, pioarduino
  `platform-espressif32` `55.03.39`, Arduino-ESP32 `3.3.9`, ESP-IDF `5.5.4`:
  targeted HIL 144/144 PASS and serial-only `dirty` discriminator
  10,000/10,000 PASS with identical 201-byte replies.
- Historical ESP32-S3 `COM20` evidence on pioarduino `54.03.20`:
  automated safe/extended HIL 33/33 PASS;
  same-value interval and raw CO2 offset/gain register HIL 25/25 PASS;
  range/capability guards 11/11 PASS; trace/sniffer/mixed-stress 11/11 PASS; full bus
  diagnostics PASS.
- ESP32-S3 Arduino diagnostic CLI on `COM17`: safe default HIL PASS, extended
  safe HIL PASS, manual dirty/resync PASS, persistent interval
  write/readback/restore PASS.
- Historical COM17 physical unplug/replug recovery: PASS as an
  operator-confirmed manual test on 2026-06-02. No automated HIL transcript
  artifact is recorded for that historical step.
- Historical `54.03.20` COM20 operator-assisted absent-sensor boot, hot
  unplug/replug, SDA/SCL stuck-low, and a complete sensor/MCU power cycle with
  measurement-interval persistence: PASS.
- Historical `54.03.20` COM20 immediate warm-up capture: PASS. The operator
  confirmed a sensor/MCU power cycle; sampling began 0.250 s after COM20 reappeared.
  MV3/MV4 remained `0 ppm` with status `0x08` through 4 s, then became
  `678 ppm` with status `0x00` at 5 s. Sensor rail timing was not instrumented.
  Final health was READY, 65 transport successes/zero failures, dirty clean,
  interval `150 ds`.
- Historical `54.03.20` COM20 strict 10-minute post-power-cycle stability
  capture: FAIL because one of 63 scheduled CLI commands returned a bounded
  NACK at t=330 s.
  The other 62 succeeded. The first sample was approximately 66 seconds after
  MCU boot, so this row is separate from the later immediate warm-up PASS.
  Manually normalized interactive output from an immediate 2,000-operation
  stress follow-up recorded zero errors; no raw follow-up transcript was
  retained.
- Historical `54.03.20` eight-hour soak: strict FAIL with 2,376 PASS, 29
  Arduino-ESP32 3.2.0 HWCDC replies stalled mid-line, and 11 real MV3 `0xC1`
  control-byte NACKs. The NACKs clustered at the same measurement phase and
  mixed stress blocks otherwise passed. The old runner's 11 review labels are
  a known classification error; the NACKs remain failures.

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
- The core driver does not retry a NACK or another failed transfer. It returns
  the precise error so the application can own bounded retry cadence. A
  control-byte NACK alone does not disclose the sensor's internal reason.
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

- The separate read-only soak harness may retry only a scheduled MV3/MV4
  `Control byte NACK` once after `--scheduled-nack-retry-ms` (default
  1,500 ms). It records both attempts and reports
  `SCHEDULED_CONTROL_NACK_RECOVERED` only when the retry passes. This is
  harness policy, not a library retry, and does not infer the NACK's internal
  cause.

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

## Current Build Baseline (`55.03.311`)

- Platform: pioarduino `platform-espressif32` `55.03.311`.
- Framework: Arduino-ESP32 `3.3.11`.
- ESP-IDF libraries: `5.5.5`.
- ESP32-S3 and ESP32-S2 example builds: PASS.
- TunnelMonitor compatibility build on retained `54.03.20`: PASS without
  source shims.
- COM20 full HIL: PASS, 184 PASS / 0 FAIL / 0 SKIP / 0 review from clean
  firmware `3bce89e`.
- Runtime target: ESP32-S3 revision 0.2, 4 MB embedded flash, 2 MB embedded
  QSPI PSRAM.
- Final state: READY, online, 3,109 tracked successes, zero consecutive/total
  failures, persistent state clean; selftest 26/0/1, repeated stress 500/500,
  mixed stress 500/500.
- Evidence is condensed in `hil_results/README.md`.
- Two preceding review runs proved
  that the example scanner accepted ACK-only malformed frames and that
  `libtest` duplicated an incomplete raw bus path. The scanner now requires
  valid PEC, and `libtest` now uses the production driver.

## Prior Platform Evidence (`55.03.39`)

Prior targeted platform and HIL evidence was recorded on 2026-07-31 using the
Arduino diagnostic CLI on `COM20`.

- Board/target: ESP32-S3 revision 0.2, embedded 4 MB flash, embedded 2 MB QSPI
  PSRAM, PlatformIO `ex_bringup_s3`.
- Platform stack: pioarduino `platform-espressif32` `55.03.39`,
  Arduino-ESP32 `3.3.9`, ESP-IDF `5.5.4`.
- Firmware/library: firmware build `Jul 31 2026 09:28:51`, EE871 library
  `1.0.1 (2ee66cf, 2026-07-31 09:28:49, dirty)`. Keep that dirty source state
  attached to these artifacts when citing them.
- Runtime identity: `ESP32-S3 rev 2`, 4,194,304 bytes flash, PSRAM ready with
  2,097,152 bytes.
- Targeted HIL: PASS, 144 PASS / 0 FAIL / 0 SKIP / 0 review. Final parsed state
  was READY with zero consecutive/total transport failures, clean persistent
  state, `stress 500` at 500/500, and selftest at 26 PASS / 0 FAIL / 1
  unsupported-mode SKIP.
- Serial-only discriminator: PASS, 10,000/10,000 `dirty` command round trips in
  14.078 seconds; all replies were exactly 201 bytes. `dirty` performs no E2 bus
  operation, so this isolates Arduino native-USB CLI framing and does not
  qualify sensor transport, accuracy, or long-soak stability.
- Accelerated scheduled-read regression: PASS, 108 sample cycles in 543.594 s,
  with 564 ordinary command passes, two fully framed MV3 control-byte NACK
  attempts each recovered by one 1,500 ms application-level retry, and zero
  hard failures, reviews, skips, reconnects, or counter regressions. The NACKs
  were about 105 s apart at nearly identical phase modulo the 15 s configured
  interval. This records schedule-correlated recurrence at the E2 control-byte
  ACK boundary and distinguishes it from host USB reply truncation, but does
  not identify the electrical or sensor-internal cause. Final state was READY
  with clean persistent state.
- Unsupported operating-mode guard: PASS. `mode` returned `NOT_SUPPORTED`
  without decoding `0x55`; tracked counters remained 3,908 successes / 2
  failures across the command and the driver remained READY.
- The platform change addresses the HWCDC TX lost-wakeup/data-loss mechanism
  documented and fixed by
  [Arduino-ESP32 PR #12606](https://github.com/espressif/arduino-esp32/pull/12606).
  The discriminator demonstrates the fix on this board but is not a completed
  long-soak result.
- Wiring visible to firmware remained DATA=GPIO6 and CLOCK=GPIO7. Pull-ups,
  level shifter, supply voltage, ambient conditions, and cable length were not
  independently measured and must not be inferred from PASS.

Prior `55.03.39` COM20 evidence is condensed in `hil_results/README.md`.

## Historical TunnelMonitor-Parity Evidence (`54.03.20`)

The broader COM20 platform-compatibility and physical/niche evidence below was
recorded on 2026-07-30 using the Arduino diagnostic CLI and the then-current
TunnelMonitor-node parity stack.

- Board/target: ESP32-S3 revision 0.2, embedded 4 MB flash, embedded 2 MB QSPI
  PSRAM, PlatformIO `ex_bringup_s3`.
- Platform stack: pioarduino `platform-espressif32` `54.03.20`,
  Arduino-ESP32 `3.2.0`, ESP-IDF `5.4.1`, GCC `14.2.0`.
  PlatformIO labels the precompiled IDF library package
  `5.4.0+sha.2f7dcd862a`; its installed `esp_idf_version.h` reports 5.4.1.
- Firmware/library: firmware build `Jul 30 2026 12:57:22`, EE871 library
  `1.0.0 (1fbe7d8, 2026-07-30 12:57:20, dirty)`. The recorded dirty status
  consists of the HIL fixes and documentation described in this change.
- Evidence/source boundary: the retained COM20 artifacts apply to the exact
  firmware metadata above. Later example-only CLI input/sniffer cleanup was
  rebuilt for S2/S3 and covered by parser/native tests, but hardware was not
  rerun after that cleanup.
- Final runtime-memory smoke firmware build: `Jul 30 2026 13:06:41`.
  `version` reported `ESP32-S3 rev 2`, 4,194,304 bytes flash, and PSRAM ready
  with 2,097,152 bytes; its safe plan passed 10/10 with selftest 27/27,
  stress 50/50, READY state, zero failures, and clean persistent state.
- Wiring visible to firmware: DATA=GPIO6, CLOCK=GPIO7.
- Sensor identity: group `0x0367`, subgroup `0x09`, available measurements
  `0x08`, serial `1920935602368A`, part name `EE871`, firmware `1.4`, E2
  specification version `4`.
- Safe plus extended HIL: PASS, 33 PASS / 0 FAIL / 0 SKIP / 0 review.
  `selftest` passed 27/27, `stress 50` passed 50/50, `stress 500` passed
  500/500, final driver state was READY, transport failures were zero, and
  persistent dirty state remained clean.
- Same-value persistent-register HIL: PASS, 25 PASS / 0 FAIL / 0 SKIP /
  0 review. Interval `150 ds`, raw CO2 offset `0 ppm`, and raw CO2 gain `32768` were
  written back to their baseline values and verified; `resync` succeeded and
  dirty state remained clean. These are custom-memory command/readback tests;
  no calibration capability was discovered or validated, and this is not
  calibration-correctness or accuracy evidence.
- Guard validation: PASS, 11/11. Invalid address `8`, intervals `149` and
  `36001`, and mode `4` returned `OUT_OF_RANGE`; unsupported valid address,
  specific interval, filter, mode, and auto-adjust writes returned
  `NOT_SUPPORTED`; validation errors did not change health counters.
- Full diagnostics: PASS. Both lines idled high, pin control and clock pulses
  passed, address 0 was found, and all eight timing points from 500 Hz through
  10 kHz responded with valid PEC. Only 500-5000 Hz is the specified range;
  the out-of-spec responses are characterization, not a supported operating
  claim.
- Trace/sniffer/mixed-stress: PASS, 11/11. Trace pending/dropped counts were
  zero, a status transaction decoded, and `stress_mix 500` passed 500/500 with
  937 tracked successes and zero failures.
- The sensor does not advertise address configuration, specific interval,
  filter, operating-mode configuration, or auto-adjust. Successful writes for
  those features are therefore not applicable to this unit.
- Operator-assisted physical tests passed: absent-sensor boot; hot
  unplug with precise OFFLINE transition and replug recovery; SDA stuck-low;
  SCL stuck-low/clock timeout; and a complete sensor/MCU power cycle with
  measurement-interval persistence. The interval was changed from `150 ds` to
  `160 ds`, persisted across the cycle, and restored to `150 ds`.

Historical `54.03.20` COM20 evidence is condensed in
`hil_results/README.md`. Detailed generated artifacts remain recoverable from
Git commit `3687e49`.

Earlier COM17 evidence follows.

Safe EE871 HIL evidence was recorded on 2026-06-01 using the Arduino diagnostic
CLI on `COM17`.

- Board/target: ESP32-S3, PlatformIO `ex_bringup_s3`.
- Upload command: `python -m platformio run -e ex_bringup_s3 -j 1 -t upload --upload-port COM17`.
- Firmware/library: firmware build `Jun  1 2026 20:57:04`, EE871 library
  `0.3.0 (84a46b6, 2026-06-01 20:57:01, clean)`.
- Version note: these hardware artifacts were captured before the final
  `1.0.0` version metadata/docs polish. Keep the exact firmware/library version
  above when citing these transcripts; the final release-polish pass did not
  add a new hardware transcript.
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

Physical unplug/replug recovery was confirmed by the operator on 2026-06-02 as
an operator-confirmed manual test. This is recorded as PASS for the unplug/replug
scenario only. It is not automated HIL evidence, and no raw transcript artifact
exists for this manual physical recovery step.

Retained artifacts:

- Condensed command evidence: `hil_results/README.md`.

Persistent configuration validation was recorded on 2026-06-01 after explicit
bench-unit approval. The bench run changed only the measurement interval, then
restored the baseline:

- Local time: 2026-06-01 21:35 CEST (`2026-06-01T19:35:00Z`).
- Serial port: `COM17`, baud `115200`.
- Board/target: ESP32-S3, PlatformIO `ex_bringup_s3`.
- Firmware/library: firmware build `Jun  1 2026 20:57:04`, EE871 library
  `0.3.0 (84a46b6, 2026-06-01 20:57:01, clean)`.
- Baseline persistent values: interval `150 ds` (`15.0 s`), CO2 interval
  factor `85`, raw operating-mode memory `0x55` (capability not advertised;
  do not decode it as a valid mode), bus
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
- Power-cycle persistence was not performed in this historical COM17 run; the
  historical `54.03.20` COM20 power-cycle result is recorded separately above.
- Failed-write/operator recovery was not physically induced; native fake tests
  remain the evidence for partial persistent-write dirty/resync behavior.

Retained artifact:

- Condensed persistent-validation evidence: `hil_results/README.md`.

Stuck-line fault/jig tests were not run in this historical COM17 session; the
historical `54.03.20` COM20 results are recorded separately above.

## Board Matrix

| ID | Board | Framework/example | Target | Sensor | Adapter | Status | Notes |
| --- | --- | --- | --- | --- | --- | --- | --- |
| B-S3-A | ESP32-S3 dev board | `examples/01_basic_bringup_cli` | `ex_bringup_s3` | EE871-E2 bench sensor | Arduino open-drain GPIO callbacks | BUILD PASS; HIL PASS | Current `55.03.311` clean-commit HIL passed 184/184 on `COM20`; rev 0.2, 4 MB flash, 2 MB QSPI PSRAM, READY/clean, zero transport failures. Prior-platform results remain separately labeled. GPIOs: DATA=6, CLOCK=7. |
| B-S2-A | ESP32-S2 | `examples/01_basic_bringup_cli` | `ex_bringup_s2` | Build only | Arduino open-drain GPIO callbacks | CI BUILD PASS | GitHub Actions release-candidate build passes. |
| B-S3-IDF | ESP32-S3 | `examples/idf/basic_bringup` | `esp32s3` | Build only | Native IDF open-drain GPIO callbacks | CI BUILD PASS | GitHub Actions ESP-IDF v6.0.1 build passes. |
| B-S2-IDF | ESP32-S2 | `examples/idf/basic_bringup` | `esp32s2` | Build only | Native IDF open-drain GPIO callbacks | CI BUILD PASS | GitHub Actions ESP-IDF v6.0.1 build passes. |

## Functional Matrix

| ID | Scenario | Board(s) | CLI/API sequence | Expected behavior | Status | Evidence to capture |
| --- | --- | --- | --- | --- | --- | --- |
| F-01 | `begin()` after MCU reset/upload with sensor present | S3 | Reset/upload, open monitor, inspect boot output, `drv`, `dirty` | Device initializes or reports a precise non-OK `Status`; driver state is READY on success and persistent dirty state is clean. | PASS | COM20 safe HIL startup plus post-power-cycle `drv`/`dirty`: READY, online yes, dirty no. Complete sensor/MCU cycle also passed P-02. |
| F-02 | Probe no-health-side-effects | S3 | `drv`, `probe`, `drv` | Successful or failed `probe` does not change health counters/state. | PASS | Safe and extended HIL `probe`: Status OK; health stayed READY with zero failures. |
| F-03 | Status read | S3 | `status`, `drv` | Status byte read completes or returns bounded error; tracked success/failure updates health as documented. | PASS | COM20 `status`: OK, byte `0x00`; trace decoded the transaction; final health READY with zero failures. |
| F-04 | CO2 averaged read | S3 | `read`, `co2avg` | MV4 averaged value is reported, or a precise bounded error is returned. | PASS | COM20 reads returned OK; observed bench values varied with ambient conditions. |
| F-05 | CO2 fast read | S3 | `co2fast` | MV3 fast-response value is reported, or a precise bounded error is returned. | PASS | COM20 expanded diagnostics: MV3 read returned OK; observed bench values varied with ambient conditions. |
| F-06 | PEC success on normal reads | S3 | `id`, `status`, `read`, `features` | Normal reads do not report `PEC_MISMATCH`. | PASS | COM20 identity/status/measurement/features/library-command diagnostics completed with valid PEC and no transport failures. |
| F-07 | Feature/cache sanity | S3 | `features`, `caps`, `cfg` | Capability output is internally consistent and guards unsupported writes. | PASS | COM20 flags `0x93/0x00/0x00`; interval/part-name support and unsupported address/factor/filter/mode/auto-adjust guards matched cached capabilities. The flags do not establish calibration capability. |
| F-08 | Immediate warm-up behavior | S3 | Power cycle sensor and MCU; after COM returns, sample at 0, 1, 2, 3, 4, 5, 6, 8, 10, 12, and 15 s using `co2fast`, `co2avg`, then `status` | Immediate post-power-cycle value/status evolution is observed; every operation is bounded and any error is recorded precisely. | PASS | Operator confirmed both sensor and MCU were powered off for at least 5 s; raw evidence records COM20 absent for 8.766 s. Startup prompt arrived 0.250 s after COM reappeared; MV3/MV4 were `0 ppm` and status `0x08` through 4 s, then `678 ppm` and status `0x00` at 5 s through 15 s. These times are relative to COM reappearance because sensor rail timing was not instrumented. All 33 scheduled commands were correctly framed. Final READY, 65 transport successes/zero failures, dirty clean, interval `150 ds`. A separate delayed-start attempt recorded one bounded fast-read NACK at 20.094 s and recovered immediately; the attempt ledger retains it. This is warm-up behavior evidence, not CO2 accuracy validation. |
| F-09 | Stale sample behavior | S3 | `status`, wait 7 s, `co2avg`; wait >10 s, `co2avg`, `status`, wait 7 s, `co2avg` | Status-triggered timing and sample evolution are observable without claiming internal freshness. | PASS | COM20: status `0x00`; averaged reads at t=607/618/625 s were 634/634/626 ppm, all OK; this is observational and not an accuracy/freshness-internals claim. |
| F-10 | Safe self-test | S3 | `dirty`, `selftest`, `dirty` | Safe commands complete with expected pass/fail report; no persistent settings are changed and persistent dirty remains clean. | PASS | Prior `55.03.39` library-1.0.1 HIL: 26 PASS / 0 FAIL / 1 unsupported-mode SKIP, dirty clean. Historical pre-guard HIL reported 27/0/0 because it decoded unsupported `0x55`; that old count is retained only with its exact firmware context. |
| F-11 | Mixed read stress | S3 | `dirty`, `stress_mix 100`, `dirty` | No hangs; failures, if any, are bounded and health counters match output; persistent dirty remains clean. | PASS | COM20 `stress_mix 500`: 500/500, 0 errors, health success +937/failures +0, READY, dirty clean. |
| F-12 | Repeated CO2 read stress | S3 | `dirty`, `stress 100`, `dirty` | No hangs; CO2 read success rate and health counters are recorded; persistent dirty remains clean. | PASS | COM20 safe HIL `stress 50`: 50/50; extended HIL `stress 500`: 500/500; zero errors and dirty stayed clean. Historical COM17 results agree. |
| F-13 | Dirty resync command on coherent config | S3 | `dirty`, `resync`, `dirty` | `resync` returns precise status; if OK, dirty remains or becomes clean only through `resyncPersistentConfig()`. | PASS | COM20 persistent HIL: pre dirty clean, `resync` OK, post dirty clean. Historical COM17 manual evidence agrees. |
| F-14 | Historical `54.03.20` post-power-cycle 10-minute transport stability | S3 | After a complete power cycle, every 30 s for 10 min read `co2fast`, `co2avg`, then `status` | All 63 scheduled CLI commands return OK; final state is READY with zero consecutive failures and persistent state remains clean. | FAIL | COM20 strict capture began about 66 s after MCU boot: 62/63 scheduled CLI commands OK; one bounded fast-read NACK at t=330 s; final READY/zero consecutive failures/dirty clean. Manually normalized immediate follow-up recorded `stress_mix 1000` 1000/1000 and `stress 1000` 1000/1000; raw follow-up transcript was not retained. |
| F-15 | Historical `54.03.20` eight-hour soak | S3 | Run scheduled MV3/MV4/status/health/dirty samples each minute plus periodic mixed stress | Complete without serial framing stalls; preserve every transport failure and finish READY/clean. | FAIL | Completed 480 cycles: 2,376 PASS, 29 HWCDC mid-line reply stalls, and 11 real MV3 `0xC1` control-byte NACKs. The old runner mislabeled the NACK rows as review-required; they are failures. NACKs clustered at the same measurement phase, mixed stress otherwise passed, final READY/clean, and no transport counter regression occurred. Arduino-ESP32 PR #12606 fixes the separate HWCDC lost-wakeup path in 3.3.9. |
| F-16 | Prior `55.03.39` targeted regression HIL | S3 | Run the targeted safe/extended command plan after upgrading Arduino-ESP32 | Every selected command is framed and parsed; finish READY with no transport failures or dirty state. | PASS | COM20/library 1.0.1: 144/144 PASS, selftest 26 PASS / 0 FAIL / 1 unsupported-mode SKIP, stress 500/500, final READY, zero total/consecutive transport failures, persistent state clean. This is targeted regression evidence, not a long soak. |
| F-17 | Prior `55.03.39` HWCDC serial discriminator | S3 | Repeat state-only `dirty` 10,000 times and compare complete response lengths | No missing/truncated replies and every complete response has identical framing. | PASS | COM20: 10,000/10,000 in 14.078 s; all replies exactly 201 bytes. `dirty` performs no E2 operation, so this isolates CLI/HWCDC framing only. |
| F-18 | Prior `55.03.39` scheduled-NACK policy regression | S3 | Sample MV3/MV4/status every 5 s, periodically run mixed stress, and retry only a fully framed scheduled MV3/MV4 control-byte NACK once after 1,500 ms | Preserve both attempts; no hidden core retry; finish READY/clean without omitted commands. | PASS | 108 cycles / 543.594 s: 564 ordinary PASS, 2 `SCHEDULED_CONTROL_NACK_RECOVERED`, 0 FAIL/review/SKIP/reconnect/counter regression. Both failures were MV3 control-byte NACKs; both retries passed. NACKs were about 105 s apart at nearly identical 15 s phase. Internal sensor cause remains unknown. |
| F-19 | Unsupported operating-mode fail-closed guard | S3 | `drv`, `mode`, `drv` on a sensor advertising no mode capability | Return `NOT_SUPPORTED`, decode no mode value, perform no tracked E2 transfer, and preserve health. | PASS | Library 1.0.1 on COM20 returned `NOT_SUPPORTED` code 14; transport counters stayed 3,908 successes / 2 failures and state stayed READY with zero consecutive failures. Native fake coverage also proves read output preservation and no-I/O read/write guards. |
| F-20 | Current `55.03.311` full regression HIL | S3 | Run safe, extended, identity/capability, invalid-parameter, bus diagnostic, trace/sniffer, and stress plans | Every selected command is framed and classified; finish READY with no transport failure or persistent dirty state. | PASS | Clean firmware `3bce89e`: 184/184 PASS, selftest 26/0/1, `stress 500` 500/500, `stress_mix 500` 500/500, address 0 only with valid PEC, library control-byte test 9/9, final READY/clean with 3,109 successes and zero failures. |
| F-21 | Current `55.03.311` native-USB process reattachment | S3 | Close the HIL process, open COM20 from a new process, establish CLI framing, and repeat rapid process-level close/open cycles | Reattachment needs no MCU reset or cable replug; queued boot output or stale partial input cannot shift responses. | PASS | The former blank-line probe timeout was reproduced; the CLI deliberately ignores empty lines. Without any reset/replug, `version` returned normally. Explicit `\ndirty\n` synchronization then passed 100/100 separate process sessions and an immediate 184/184 full HIL rerun. After HIL closed COM20, a new process passed 10,000/10,000 state-only replies (all 201 bytes, one hash) in 10.094 s. |

## Persistent Configuration Matrix

Run these only on a bench sensor after recording original values.

| ID | Scenario | Board(s) | CLI/API sequence | Expected behavior | Status | Evidence to capture |
| --- | --- | --- | --- | --- | --- | --- |
| P-01 | Measurement interval write/readback | S3 | `interval`, `dirty`, record value, `interval <bench_value>`, `interval`, `dirty` | Write returns OK and readback matches; on failure, `dirty` reports whether persistent state may be partial. | PASS | COM20 baseline `150 ds`, rewrote/read back `150 ds`, resync OK, dirty clean. Historical COM17 changed to `160 ds` and restored `150 ds`. |
| P-02 | Measurement interval power-cycle persistence | S3 | Run P-01, power cycle sensor and MCU, `interval` | Value persists across power cycle or documented sensor behavior explains difference. | PASS | COM20: baseline `150 ds`, wrote/verified `160 ds`, fully power-cycled sensor and MCU, verified `160 ds`, restored/verified `150 ds`, resync OK, dirty clean. |
| P-03 | Raw CO2 offset register write/readback | S3 | `offset`, `dirty`, record value, `offset <bench_value>`, `offset`, `dirty` | Same-value write returns OK and readback matches; dirty diagnostics checked on failure. | PASS | COM20 same-value write `0 ppm` returned OK and read back `0 ppm`; dirty stayed clean. This validates only the custom-memory command/readback path. No calibration capability, correctness, or accuracy claim is made. |
| P-04 | Raw CO2 gain register write/readback | S3 | `gain`, `dirty`, record value, `gain <bench_value>`, `gain`, `dirty` | Same-value write returns OK and readback matches; dirty diagnostics checked on failure. | PASS | COM20 same-value write `32768` returned OK and read back `32768`; dirty stayed clean. This validates only the custom-memory command/readback path. No calibration capability, correctness, or accuracy claim is made. |
| P-05 | Part name write/readback | S3 | `partname`, `dirty`, record value, `partname <bench_text>`, `partname`, `dirty` | Write returns OK and readback matches; dirty diagnostics checked on failure. | PASS | COM20 same-value `EE871` write returned OK and read back `EE871`; the verified block and the broader session limitation are retained in `niche_diagnostics_20260730.md`. The ad-hoc session was not used as an overall PASS. |
| P-06 | Bus address write | S3 | `addr`, record value, `addr <bench_addr>`, power cycle, `scan`; then rebuild/reconfigure firmware for the new address or use a dedicated test wrapper | Address change behaves as documented and does not retarget the current session until power cycle. | NOT APPLICABLE | COM20 sensor flags do not advertise address configuration. Valid `addr 0` returned `NOT_SUPPORTED`; invalid `addr 8` returned `OUT_OF_RANGE` without bus/health side effects. |

## Fault And Recovery Matrix

| ID | Scenario | Board(s) | CLI/API sequence | Expected behavior | Status | Evidence to capture |
| --- | --- | --- | --- | --- | --- | --- |
| R-01 | Wrong wiring or no sensor | S3 | Disconnect sensor, boot, `drv`, `buscheck`, `probe`, `read`, `dirty` | Initialization or reads fail with bounded non-OK status; no hang. | PASS | COM20 absent-sensor boot: `begin()` returned bounded `NACK`, driver stayed UNINIT, both lines high, counters zero, dirty clean. |
| R-02 | Unplug/replug recovery | S3 | Start connected, `read`, unplug, repeated `read`, replug, `recover`, `drv` | Tracked failures degrade/offline as configured; successful `recover` returns READY. | PASS | Historical `54.03.20` COM20 run: raw probe NACK/health-neutral; five tracked NACK reads reached OFFLINE at threshold 5; replug preserved OFFLINE; explicit recover OK in 16 ms, READY, selftest 27/27, stress_mix 100/100. Historical COM17 evidence also retained. |
| R-03 | SDA stuck low | S3 | Use fault resistor to pull SDA low, `buscheck`, `libreset`, `drv` | `BUS_STUCK` or precise bounded error; no unbounded wait. | PASS | COM20 470-ohm fault: SDA low/SCL high; BUS_STUCK; raw probe PEC_MISMATCH health-neutral; tracked read PEC_MISMATCH in 16 ms; library reset BUS_STUCK; release/recover restored READY. |
| R-04 | SCL stuck low / clock stretch timeout | S3 | Pull SCL low, `buscheck`, `probe`, `read`, `busreset`, `libreset` | Timeout or `BUS_STUCK` within configured deadline; no hang. | PASS | COM20 470-ohm fault: SCL low/SDA high; raw probe TIMEOUT in 31 ms detail 25000 without health change; tracked read TIMEOUT in 32 ms; library reset BUS_STUCK in 31 ms; release/recover restored READY. |
| R-05 | Sensor absent/open line/no ACK | S3 | Unplug sensor, `probe`, tracked reads | NACK/no-response error is bounded and health rules match raw probe versus tracked reads. | PASS | COM20 sensor unplug: raw probe NACK in 15 ms without health change; tracked reads returned NACK and updated health through DEGRADED to OFFLINE. |
| R-06 | Recovery clocks on idle bus (smoke) | S3 | `busreset`, `buscheck` | Nine recovery clocks are issued and the idle state is reported accurately. | PASS | COM20 no-fault recovery issued nine clocks and ended SCL/SDA high; R-03/R-04 separately prove clocks cannot falsely clear physically held-low lines and explicit recovery succeeds after release. |
| R-07 | Timing sweep | S3 | `timing` | Supported timing range is identified without hangs; failures are bounded. | PASS | COM20 full diagnostics: six in-spec points from 500-5000 Hz and two characterization-only points at 6667/10000 Hz ACKed with valid PEC. |
| R-08 | Bus trace sanity | S3 | `verbose 1`, `status`, `trace stats`, `verbose 0` | Trace captures bounded line activity and does not destabilize reads. | PASS | COM20 trace captured a successful status transfer, pending=0, dropped=0; sniffer decoded the transaction; following mixed stress passed 500/500. |

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
