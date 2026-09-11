# EE871-E2 Hardware Validation Matrix

Last updated: 2026-09-11

This is the maintained hardware evidence ledger and repeatable scenario plan.
Results apply to the recorded firmware, fixture, configuration and exposure.
A historical PASS does not qualify later code changes. Build and native-test
results are separate from hardware results; see the [CI workflow](../.github/workflows/ci.yml)
for software validation gates and the [HIL runner guide](EE871_E2_HIL_RUNNER.md)
for capture commands.

## Software Validation

Software checks on 2026-09-11 validated implementation
[`3d32ac30cc4b228299d3f0feaffd538c031f5276`](https://github.com/janhavelka/EE871-E2/commit/3d32ac30cc4b228299d3f0feaffd538c031f5276),
including the calibration/capability guards, persistent-write uncertainty
tracking, and auto-adjust preflight. The earlier `a358f92` release-preparation
run passed 71 native tests; this implementation adds 22 regression cases.

| Check | Result |
| --- | --- |
| `.\scripts\pio.cmd test -e native` | 93/93 native cases passed. |
| `python -m unittest discover -s test -p "test_*.py"` | 58/58 Python cases passed. |
| Timing, Arduino CLI, and native IDF example contract checkers | All passed. |
| `python scripts/generate_version.py check` | Version.h, IDF manifest and Doxyfile match package version 1.1.0. |
| `doxygen Doxyfile` and local Markdown target check | Documentation generated without warnings; local links resolve. |
| `.\scripts\pio.cmd run -e ex_bringup_s3 -e ex_bringup_s2 -e compat_tunnelmonitor_s3` | All three Arduino builds passed; build-only evidence. |
| [CI run 34605633301](https://github.com/janhavelka/EE871-E2/actions/runs/34605633301) at `3d32ac3` | All six jobs passed: native tests, library validation, Arduino S2/S3, and native ESP-IDF 6.0.1 S2/S3. Build/test evidence only. |

The final release revision must pass the CI workflow, including native
ESP-IDF 6.0.1 builds for both ESP32-S3 and ESP32-S2. Each workflow run identifies
the tested commit. Local `idf.py` builds were not run; an earlier green run
does not establish success for a later candidate.

## September 11 Current-Main Targeted HIL

The COM11 campaign starting at **14:18:21 UTC** exercised exact library source
[`32dfb0665ac3628a25578ab7a03e1ebc285040d2`](https://github.com/janhavelka/EE871-E2/commit/32dfb0665ac3628a25578ab7a03e1ebc285040d2),
version `1.1.0`. The temporary Arduino harness copied the core and public
headers byte-for-byte from clean `main`; fault injection lived in its external
HAL. Its application image SHA-256 was
`10fdeeb0a2513285758c0239c285fa53214985ee8bb0de32109d86987d63a270`,
built September 11 at 16:17:15 local time (Europe/Prague). Platform versions
were pioarduino `55.03.311`, Arduino `3.3.11`, and IDF libraries `5.5.5`.

The fixture was CO2Control HW2.0.0, ESP32-S3 N16R8 (16 MB flash, 8 MB PSRAM),
COM11 USB `303a:1001`, serial `3C:0F:02:CD:6B:3C`, DATA GPIO4 / CLOCK GPIO5.
EE871 serial `1920935602368A` reported firmware `1.4`, E2 specification `4`,
metadata `0x00..0x09 = 01 04 04 08 08 01 08 93 00 00`, address `0`, and
interval `150 ds`. Calibration support bytes `0x03/0x04` both advertised CO2.
The fixture's existing level shifting, pull-ups, cable and supply were not
independently measured.

| Group | Recorded result | Scope |
| --- | --- | --- |
| Real sensor and API assertions | **407 PASS / 0 FAIL / 3 SKIP** | Identity, capabilities, typed reads, parameter guards, probe/reset/recovery, coherent resync, lifecycle and diagnostics. Offset `0`, gain `32768`, calibration points `0/50000` read successfully through the new support guards. Unsupported mode/auto-adjust and other unadvertised writes were rejected. |
| Address and timing sweep | PASS | Address 0 initialized; addresses 1..7 returned NACK during identity probing. Initialization/status passed with clock high/low phases `995`, `500`, `250`, `200`, `150`, and `100 us`, each with `10 us` setup. This tests the corrected library timing configurations, not standalone CLI decoding or external waveforms. |
| Repeated reads and acquisition | PASS | 100/100 alternating MV3/MV4 stress reads; then 61 MV3/MV4/status cycles in 60.939 s. Fast readings `572..588 ppm`, averaged readings `580..588 ppm`, all status bytes `0x00`. Both captured real-operation sessions had zero transport failures, eligible NACKs or retries and remained READY/clean. Repeated reads are not distinct sensor conversions. |
| Injected error assertions | **15 PASS / 1 FAIL** | Low/high-byte read-NACK recovery, three retries per exhausted frame, three exhausted frames latching OFFLINE, no traffic while OFFLINE, explicit recovery, callback veto, default retries disabled, PEC rejection without retry, SCL timeout without retry (32 ms), STOP failure blocking retry, stuck-SDA readback, and identity NACK exclusion passed. The custom-read NACK expectation failed with `PEC_MISMATCH` detail `255`; retained below. |
| Existing native fake suite on ESP32 | **93/93 PASS** | Emulated callbacks, separate from real sensor I/O. Includes uncertain persistent writes, reserved/absent capability data, auto-adjust preflight, dirty retention and resync. Running on the MCU does not turn these cases into physical sensor fault tests. |
| Configuration and cleanup | PASS | All 30 captured configuration/calibration/part-name bytes matched afterward; zero direct persistent-write attempts; E2 lines released. The original production application was restored and hash-verified, with healthy EE871, aggregate health `ok`, watchdog `quorum_ok`, and saved-settings metadata unchanged. |

The first harness total was **515 PASS / 1 FAIL / 3 SKIP**, so its overall
verdict remains **REVIEW_REQUIRED**. Its address-7 absence check used identity
control `0x1F`; redirecting a logical custom read `0x51` to physical `0x5F`
unexpectedly reached PEC verification instead of returning NACK. This alone
does not establish address aliasing or a library defect.

A targeted follow-up beginning **14:25:25 UTC** retained **81 PASS / 3 FAIL**
and `REVIEW_REQUIRED`. Its image SHA-256 was
`3810f1d9378fd1dd57017a951d1534ac1869218f846eeaa43bdb5587f1c1d686`.
Three custom-read address rewrites to `0x5F` and three full control rewrites
to `0x1F` all sampled ACK, data `0xFF`, and PEC `0xFF`. Each returned
`PEC_MISMATCH` in 7 ms, with one frame, one tracked failure and no retry.
The latter three cases failed their NACK expectation. Capture recorded the
commanded control bits and GPIO ACK/data/PEC samples; it was not an external
logic-analyzer trace. Every case immediately followed `setCustomPointer(0)`.
Normal reads and explicit recovery succeeded afterward, all 30 registers
remained unchanged, and healthy production restoration succeeded again.
The follow-up transcript SHA-256 is
`b4fe37615fb9c1ed5ea7227e588c3a241d04b9775847b936aa58c82a2defd21d`.

The final pointer-context comparison beginning **14:31:00 UTC** passed
**303/303 assertions**. It made 27 observations: three repetitions in each
of three pointer contexts, through each of three control paths. These paths
were logical custom read `0x51` rewritten to `0x5F`, rewritten to `0x1F`, and
an ordinary `begin()` at configured address 7 with **no control rewriting**.
The last path independently reproduced the same context dependence.

| Context before the wrong-address frame | Result across all three paths |
| --- | --- |
| Recovery completed; no pending pointer update | 9/9 NACK, 3 ms each. |
| Immediately after `setCustomPointer(0)` | 9/9 sampled ACK, data `0xFF`, PEC `0xFF`; `PEC_MISMATCH`, 7 ms each. |
| Pointer set, then consumed by one successful normal custom read | 9/9 NACK, 3 ms each. |

All 12 captured NACKs in the two logical-custom-read paths passed strict
no-retry and single-health-failure checks. No invalid frame was accepted as a
successful read. Every case recovered successfully; all 30 captured registers
again matched. The image SHA-256 was
`36389598bc60d9c8c8267986bc3e3ed309d6baf663de78df54aec237664cff73`;
the transcript SHA-256 is
`67d53f95ee2b3d47cf855dc8198c4ae6ec337918b80e2d558e130c21a70aa4bd`.

This isolates an observed response to a wrong-address frame while a pointer
read is pending on this fixture. It explains why the first two harnesses'
NACK expectations did not hold; their failed verdicts remain intact. It does
not establish a documented addressing exception, a physical cause, or behavior
of other sensor firmware. The driver already fails closed with the precise
observed status, so no library change was made. For future NACK injection,
complete any pending custom-pointer read before assuming identity-scan absence
predicts a NACK from another frame.

The harness held board outputs low during testing and prevented direct
persistent-write controls from reaching the sensor. Supported calibration
writes, physical partial-write interruption, unplug/power faults, extended
interval status-trigger timing, electrical waveform measurements and a long
soak were not run. Native ESP-IDF runtime and ESP32-S2 hardware remain untested.

The original application's SHA-256 was
`bae394d6f962e161cb5af292f1c5f356c81654588765976a90d19b0630c0c2a8`,
clean CO2Control `b77ecf02`, library
`1.0.1@9481b0f569eb4096c2204d30b79ac25f8d16898a`. Before testing, esptool
verified the retained bootloader, partition table and application against
flash. Only the application at `0x10000` was replaced and restored; the test
did not deploy the new library to production.

Local captures and temporary harness sources are retained under the ignored
`hil_logs/20260911T140235Z_co2_targeted/` directory. The first serial transcript
SHA-256 is `7c5abcc7105168a3879bb4615dcfa584ad3c0989435a0906020d334001a36722`.
This ledger preserves the compact result; raw artifacts are local, not a
published download.

## Earlier September Library Hardware Evidence

The September 11 CO2Control comparison exercised clean EE871-E2
[`a358f92a6882e00810c775a61f5499d5cff60885`](https://github.com/janhavelka/EE871-E2/commit/a358f92a6882e00810c775a61f5499d5cff60885),
source version `1.1.0`, including strict initialization, ordinary-STOP timing
and bounded read retries. The firmware was clean
`170bd52e4847377332f7c9bd4d912baf5da74123`, environment
`co2control_wifi_hil`, built 2026-09-11 09:55:08 UTC. It used pioarduino
`55.03.311`, Arduino-ESP32 `3.3.11`, and ESP-IDF libraries `5.5.5`.

The fixture was CO2Control HW2.0.0 / `co2control_s3_hw200`, ESP32-S3 on
COM11, USB `303a:1001`, USB serial `3C:0F:02:CD:6B:3C`. The sensor was
EE871 serial `1920935602368A`, firmware `1.4`, capability flags `93/00/00`,
measurement interval `150 ds`. CO2Control explicitly enabled three additional
read attempts; the library default remains zero retries.

| Observation | Recorded result | Qualification |
| --- | --- | --- |
| Ten-minute E2 stress | 562 successful owner operations, zero sensor errors or NACKs; all 522 admitted HIL jobs completed and drained | Completed; did not exercise retry recovery. Owner operations are not distinct physical conversions. |
| Thirty-minute normal acquisition | 120 successful readings; one control NACK, one retry, one recovered frame, zero exhausted frames or sensor errors | One real MV3 low-byte `0xC1` NACK recovered on its first retry. CO2 remained valid and qualified; the valve stayed On. |
| Recovered measurement timing | Whole owner measurement completed in 32 ms; its preceding clean neighbor took 28 ms | Includes owner processing; not individual frame or external waveform timing. |
| Normal-capture HTTP exception | One HTTP 403; polling resumed with no new sensor error | Capture exception retained; rejection cause unknown. It occurred separately from the recovered E2 event. |
| Five-minute targeted follow-up | Ten start/stop epochs; 257 admitted HIL jobs completed, none failed/lost/pending; 276 successful owner operations, zero sensor errors or NACKs | E2 assertions passed, but overall harness result is **INCOMPLETE**: stress and restored-smoke Web-session cleanup each returned HTTP 403. It adds no retry-recovery observation. |

The comparison and follow-up both restored the previous normal production
image, clean `b77ecf02`, library `1.0.1@9481b0f569eb4096c2204d30b79ac25f8d16898a`.
Application writes were hash-verified and live version/dependency checks
matched; no independent full-flash readback is claimed. The comparison's
restored boot had one startup sensor fault before its ten-minute observation
window, then added 40 successful readings with no new errors and regained
qualification. Saved settings matched across the arms and restoration.

Exact image hashes, retry-event fields, capture boundaries and restoration
evidence are preserved in the commit-pinned
[comparison report](https://github.com/janhavelka/TunnelMonitor-node/blob/7e808b7ef464352cf8759e229a7a222bf777b474/docs/reports/hil-testing/condensed/2026-09-11_ee871_read_retries_hil.md)
and [targeted follow-up](https://github.com/janhavelka/TunnelMonitor-node/blob/2e51078a07c156cdfbc3c801205419674757ca80/docs/reports/hil-testing/condensed/2026-09-11_ee871_targeted_stress.md).
The [implementation verification report](https://github.com/janhavelka/TunnelMonitor-node/blob/7e808b7ef464352cf8759e229a7a222bf777b474/docs/reports/ee871_read_retries_20260911.md)
records 71 library native tests and 58 Python tests at the tested revision.

Limits of this evidence:

- The later calibration/capability guards, auto-adjust preflight, and expanded
  write-uncertainty/resync behavior were not exercised by this earlier run. This
  campaign did not record calibration support bytes `0x03/0x04` and does not
  qualify those changes on hardware.
- One recovered ACK miss does not establish a fault-rate improvement, a
  long-term reliability rate, or the physical cause of the NACK.
- All three retries and exhaustion, callback vetoes and non-NACK fail-closed
  behavior have native fake coverage; they were not physically forced in this
  campaign. Partial persistent-write failure remains native-only evidence.
- No oscilloscope, logic analyzer or external sensor-supply instrument was
  connected. Cached whole-board INA228 readings do not measure the sensor rail
  during a retry or exclude short transients. Sensor accuracy was not tested.
- There is no completed long soak for this candidate or current platform.
  ESP32-S2, native ESP-IDF runtime, and TunnelMonitor hardware were not exercised
  in this campaign. The standalone diagnostic CLI was not requalified by the
  CO2Control integration test.

## Historical Fixtures And Results

These results predate the September strict-initialization, latched-OFFLINE,
ordinary-STOP and retry changes. Keep them attached to their recorded builds.
The COM20 board was ESP32-S3 revision 0.2, 4 MB embedded flash and 2 MB QSPI
PSRAM, using `ex_bringup_s3`, DATA GPIO6 and CLOCK GPIO7. Its EE871 sensor
identity was group `0x0367`, subgroup `0x09`, measurements `0x08`, serial
`1920935602368A`, part `EE871`, firmware `1.4`, E2 specification `4`.
Pull-ups, level shifter, supply voltage and cable length were not independently
measured; none should be inferred from PASS.

| Evidence | Exact firmware / platform | Results |
| --- | --- | --- |
| H-311, July 31, COM20 | Clean `3bce89eee9cd2145488c4e842f353a9ef8e4163e`; pioarduino `55.03.311`, Arduino `3.3.11`, IDF `5.5.5` | Safe/extended/niche HIL 184/184; selftest 26 PASS / 0 FAIL / 1 unsupported-mode SKIP; repeated and mixed stress each 500/500; final READY, persistent state clean, 3,109 successes and zero transport failures. |
| H-39, July 31, COM20 | Library `1.0.1`, `2ee66cf` **dirty**, firmware built 2026-07-31 09:28:51; pioarduino `55.03.39`, Arduino `3.3.9`, IDF `5.5.4` | Targeted HIL 144/144; selftest 26/0/1; stress 500/500; final READY/clean with zero failures. Scheduled regression: 108 cycles in 543.594 s, 564 ordinary passes and two MV3 NACKs recovered by one application retry after 1.5 s; no hard failure, reconnect or counter regression. |
| H-54, July 30, COM20 | Library `1.0.0`, `1fbe7d8` **dirty**, firmware built 2026-07-30 12:57:22; pioarduino `54.03.20`, Arduino `3.2.0`, IDF `5.4.1`, GCC `14.2.0` | Safe/extended 33/33; same-value persistent-register checks 25/25; range/capability guards 11/11; trace/sniffer/mixed-stress 11/11; physical fault and power-cycle checks below. Separate memory smoke built 13:06:41 passed 10/10, selftest 27/27, stress 50/50. |
| H-17, June 1, COM17 | ESP32-S3 `ex_bringup_s3`; library `0.3.0`, clean `84a46b694c8b5c8755b75890e4f95b0c44a0d7b8`, firmware built 2026-06-01 20:57:04; platform not recorded | Safe 10/10, extended 33/33, selftest 27/27, stress 50/50 and 500/500; READY/clean. Interval write/readback/restore: 31 captured steps passed. June 2 unplug/replug was operator-confirmed only, with no automated transcript. |

The dirty source states above are part of the provenance; those abbreviated
commits alone cannot reproduce the tested working trees. H-54's installed
IDF header reported `5.4.1` although PlatformIO's package label was
`5.4.0+sha.2f7dcd862a`. Its results do not revalidate later example-only cleanup.

The historical [matrix](https://github.com/janhavelka/EE871-E2/blob/6e6a77cada686808dae427c2a4f8624a61ca4304/docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md)
and [condensed ledger](https://github.com/janhavelka/EE871-E2/blob/6e6a77cada686808dae427c2a4f8624a61ca4304/hil_results/README.md)
preserve the earlier detail. Generated serial/JSON/Markdown artifacts removed
in the earlier cleanup remain recoverable from
[commit `3687e49`](https://github.com/janhavelka/EE871-E2/tree/3687e4930f63151eb7e3802425b56f15271d8314/hil_results).

### Negative Evidence And Scope Boundaries

| Historical scenario | Result and retained qualification |
| --- | --- |
| H-54 strict ten-minute post-power-cycle stability | **FAIL**: 62/63 scheduled commands succeeded; one bounded MV3 NACK at 330 s. Sampling started about 66 s after MCU boot. Final READY/clean. An immediate manually normalized 2,000-operation stress follow-up reported no errors, but no raw follow-up transcript was retained. |
| H-54 eight-hour soak | **FAIL**: 480 cycles, 2,376 successful commands, 29 native-USB replies stalled mid-line and 11 complete MV3 `0xC1` control-byte NACKs. The old runner incorrectly labeled those NACKs as review-required; they remain failures. Final READY/clean. |
| USB framing discrimination | H-39: 10,000/10,000 identical 201-byte state-only `dirty` replies in 14.078 s. H-311: 100/100 separate process sessions and 10,000/10,000 identical replies in 10.094 s after full HIL closed COM20, without reset/replug. Blank-line probes had been ignored by the CLI; explicit `\ndirty\n` synchronization worked. This qualifies host/CLI framing only. |
| NACK cause | Historical NACKs recurred near the same measurement phase; the H-39 application retries succeeded. This is an observed ACK-boundary symptom, not proof of sensor busy state, supply trouble or an electrical cause. The hardware reports distinguish it from the separate old HWCDC reply-truncation problem. |
| Scanner and library diagnostics | H-311 found only address 0 with valid PEC; `libtest` passed 9/9 using production driver reads. The later scanner's full `begin()` identity/capability/feature validation and status check still lack standalone CLI hardware revalidation. |
| Timing sweep | H-54's old candidate set responded at six nominal in-spec and two out-of-spec points (6667/10000 Hz). Its 1000/1000 us point omitted setup time from the 500 Hz label. The September 11 targeted harness passed all six corrected configurations, slowest 995/995 us plus 10 us setup; the standalone CLI remains unqualified. Out-of-spec historical responses are not supported operating claims. |

## Repeatable Scenario Matrix

Historical PASS entries below refer to H-311/H-39/H-54/H-17 above, not an
automatic PASS for the current candidate. Unless a recorded result is supplied,
a new firmware/fixture run is `NOT RUN`. Use `PASS`, `FAIL`, `BLOCKED` or
`NOT APPLICABLE` for completed classifications, and retain `INCOMPLETE` when a
capture or cleanup prevents an overall verdict.

### Functional And Scheduling Checks

| ID | Scenario and sequence | Expected behavior / retained hardware result |
| --- | --- | --- |
| F-01 | Sensor-present boot: `version`, `drv`, `dirty` | Successful initialization yields READY and clean persistent state; failures remain precise and bounded. Historical CLI PASS; current-main targeted harness initialization/capability reads PASS. |
| F-02 | Diagnostic probe: `drv`, `probe`, `drv` | Probe must not change health counters/state. Historical CLI PASS. |
| F-03 | Status: `status`, `drv` | Bounded result with tracked health; reading status can trigger a new measurement under the documented interval/age conditions. Historical `0x00` reads and trace PASS. |
| F-04/F-05/F-06 | MV4/MV3 and PEC: `read`, `co2avg`, `co2fast`, `id`, `features` | Low-before-high paired reads; valid PEC or precise bounded error. Historical CLI PASS; September normal acquisition had one recovered MV3 NACK and no public sensor error. |
| F-07/F-19 | Capabilities: `features`, `caps`, `cfg`, `drv`, `mode`, `drv` | Unadvertised mode returns `NOT_SUPPORTED` without I/O/health change. H-39 PASS, counters unchanged at 3,908 successes / 2 failures. Flags `93/00/00` do not establish calibration capability. |
| F-08 | Complete sensor/MCU power cycle, then MV3/MV4/status at 0, 1, 2, 3, 4, 5, 6, 8, 10, 12 and 15 s | H-54 PASS: `0 ppm`/status `0x08` through 4 s, then `678 ppm`/`0x00` at 5 s; final READY, 65 successes, zero failures, clean, interval `150 ds`. Capture started 0.250 s after COM reappeared; sensor rail timing was not instrumented. A separate delayed-start attempt had one bounded NACK at 20.094 s and recovered immediately. |
| F-09 | Status and sample evolution: `status`, wait 7 s, `co2avg`; wait >10 s, `co2avg`, `status`, wait 7 s, `co2avg` | H-54 observational PASS: values 634/634/626 ppm at recorded times 607/618/625 s. This does not prove internal freshness or status-trigger operation at the tested 15 s interval. |
| F-10/F-11/F-12 | `dirty`, `selftest`, `stress 500`, `stress_mix 500`, `dirty` | Bounded operations, accurate counters, no persistent writes. H-311 PASS with selftest 26/0/1 and both stress groups 500/500. Earlier 27/0/0 selftests predated the unsupported-mode guard. |
| F-13 | Coherent configuration: `dirty`, `resync`, `dirty` | Only successful verified resync clears dirty state. H-54/H-17 coherent-config PASS; induced partial-write recovery remains native-only. |
| F-14/F-15 | Ten-minute scheduled reads / eight-hour soak | Historical H-54 FAIL as recorded above; current long soak NOT RUN. |
| F-16/F-17/F-18 | Platform regression / state-only USB / scheduled application retry | H-39 results above are historical PASS within their selected commands and exposure. The 1.5 s application retry is separate from current library retries. |
| F-20/F-21 | Full standalone CLI / process reattachment | H-311 184/184 HIL and 100/100 process sessions PASS; current standalone CLI HIL NOT RUN. |
| F-22 | Explicit read retries during normal acquisition | September `a358f92` observation: one `0xC1` NACK recovered on first retry, complete measurement OK, qualification retained. |
| F-23 | Retry budget exhaustion, veto, STOP/idle failure and non-NACK exclusion | Current-main targeted HAL injections passed exhaustion, veto, PEC/timeout and cleanup exclusions; 12 captured custom-read NACKs also passed no-retry checks. Retain the pending-pointer response and initial failed expectations above. Electrical fault-jig testing NOT RUN. Preserve final transport status, health and retry diagnostics. |

### Persistent Configuration

Record original values and use a bench sensor approved for persistent changes.
Restore and verify every changed value. Check `dirty` immediately after a
failed write; clear it only through successful `resyncPersistentConfig()`.
Recovery restores communication but retains persistent uncertainty.

| ID | Scenario and sequence | Retained result |
| --- | --- | --- |
| P-01 | `interval`, record baseline, `interval <bench_value>`, `interval`, `dirty`, restore and verify | H-17 PASS: `150 -> 160 -> 150 ds`, all 31 captured steps passed, dirty clean. H-54 same-value `150 ds` write/readback PASS. |
| P-02 | Change interval, power cycle sensor and MCU, read interval, restore | H-54 PASS: `160 ds` persisted, restored to `150 ds`, resync OK/clean. This power-cycle test was not run in H-17. |
| P-03/P-04 | Record `offset`/`gain`, same-value writes, readback, `dirty` | H-54 PASS: `0 ppm` / `32768`, clean. These qualify custom-memory command/readback only; no calibration capability, accuracy or calibration correctness was validated. |
| P-05 | Record `partname`, same-value write/readback, `dirty` | H-54 ad-hoc `EE871` readback PASS; the whole ad-hoc session was not classified PASS. |
| P-06 | Supported address write, power cycle, retarget firmware, `scan`/`probe`, restore | NOT APPLICABLE to this sensor: address configuration unadvertised. Valid `addr 0` returned `NOT_SUPPORTED`; invalid `addr 8` returned `OUT_OF_RANGE` without bus/health effects. |
| P-07 | Induce partial or uncertain persistent write; inspect dirty diagnostics, recover communication if needed, then resync | Native fake coverage covers single/raw and multi-byte writes, interrupted PEC/final ACK/STOP/readback, multiple pending targets, lifecycle retention, and lost capabilities. Physical failure injection NOT RUN. |
| P-08 | Supported auto-adjust start/status; duplicate request; uncertain start followed by idle-status resync | Native fake coverage of support/reserved-bit guards, `BUSY` without another start, precise transport errors, and dirty retention until complete resync. Hardware calibration NOT RUN; resync does not prove adjustment success. |

H-54 invalid intervals `149`/`36001` and mode `4` also returned
`OUT_OF_RANGE`. Unsupported address, factor, filter, mode and auto-adjust writes
returned `NOT_SUPPORTED`; these guards passed without changing health counters.

### Physical Faults And Recovery

All physical results in this table are historical H-54; they have not been
repeated against the current retry candidate. Applications own fault injection,
retry settings, deadlines and recovery cadence.

| ID | Scenario and sequence | Retained result |
| --- | --- | --- |
| R-01/R-05 | Sensor absent at boot / unplugged: `drv`, `buscheck`, `probe`, tracked reads, `dirty` | PASS: absent boot returned bounded NACK, stayed UNINIT with zero counters and clean state. Raw probe NACK in 15 ms was health-neutral; tracked NACKs degraded/offlined the initialized session. |
| R-02 | Connected read, unplug, repeated reads, replug, `recover`, `drv` | PASS: five tracked NACKs reached OFFLINE at threshold 5; replug retained OFFLINE; explicit recovery succeeded in 16 ms, READY, selftest 27/27, mixed stress 100/100. H-17 also has operator-confirmed recovery without a transcript. |
| R-03 | Pull SDA low through 470 ohms: `buscheck`, `probe`, read, `libreset`; release and recover | PASS: bus check/reset `BUS_STUCK`, raw and tracked reads `PEC_MISMATCH`, tracked read bounded at 16 ms; recovery restored READY after release. These are observed historical statuses, not a contract requiring PEC mismatch for stuck SDA. |
| R-04 | Pull SCL low through 470 ohms: `buscheck`, `probe`, read, `libreset`; release and recover | PASS: raw probe TIMEOUT in 31 ms/detail 25000 without health change; tracked read TIMEOUT in 32 ms; reset `BUS_STUCK` in 31 ms; release/recovery restored READY. |
| R-06 | Idle-bus `busreset`, `buscheck` | PASS: nine recovery clocks, final SCL/SDA high. Physically held-low lines were not falsely reported clear. |
| R-07 | `timing` | All six corrected library timing configurations passed the current-main targeted harness. Standalone CLI and external waveform timing remain unqualified. |
| R-08 | `verbose 1`, `status`, `trace stats`, `verbose 0`, `stress_mix 500` | PASS: bounded status trace decoded; pending/dropped zero; following mixed stress 500/500. |

## Safe Bring-Up Recipe And Recording

Use the [runner guide](EE871_E2_HIL_RUNNER.md) for automatic plans and explicit
persistent-write/fault-test options. A compact non-persistent CLI sequence is:

```text
version
drv
dirty
buscheck
probe
co2fast
co2avg
status
selftest
stress_mix 50
drv
dirty
```

These commands update health and can have measurement side effects: `status`
can trigger a measurement when the interval is greater than 15 s and the last
measurement is older than 10 s. `probe` is health-neutral. `recover` may issue
bounded reset clocks; `resync` reads persistent configuration and only clears
dirty state after successful verification.

The library defaults to one frame attempt. An application can explicitly set
`readNackRetries` to 1..3 for MV3/MV4/status control-byte NACKs only, after
successful STOP and idle checks, with a fixed 1 ms HAL pause and optional
`allowReadRetry` veto. Identity/custom reads, writes, PEC failures and timeouts
are not retried. Record the actual retry configuration; the soak harness's
optional 1.5 s application retry is a separate policy.

For each new record, include date/time and operator, exact commit and dirty
state, firmware timestamp/hash, build platform, board/target, sensor identity,
port, GPIOs, pull-ups, level shifter, supply and cable details, retry/deadline
settings, commands and exposure, statuses/counters, cleanup/restoration and
limitations. Mark unmeasured details explicitly. Preserve a compact result with
durable provenance before discarding redundant captures.
