# HIL Evidence Summary

Last updated: 2026-09-09

2026-09-09 evidence qualification: all recorded hardware results below predate
the latched-OFFLINE, fail-closed-begin(), and rewritten-scanner changes. They
remain historical evidence until the updated firmware is re-run on hardware;
this software audit performed no hardware/HIL or soak re-validation.

This is the retained hardware evidence ledger. Generated command tables, JSON,
and raw serial captures were removed after review because they repeated the
results below. The last commit containing the detailed artifacts is
`3687e49`; Git history remains the recovery source if a low-level transcript is
needed.

## Bench

- Board: ESP32-S3 revision 0.2, 4 MB embedded flash, 2 MB embedded QSPI PSRAM.
- Sensor: EE871-E2 at address 0, DATA GPIO6, CLOCK GPIO7.

## Results

| Stack | Result | Important evidence |
| --- | --- | --- |
| pioarduino `55.03.311`, Arduino `3.3.11`, IDF `5.5.5` | PASS | Clean firmware `3bce89e`; full safe/extended/niche HIL 184/184; self-test 26/0/1 unsupported skip; repeated stress 500/500; mixed stress 500/500; final READY and persistent state clean with 3,109 successes and zero transport failures. |
| pioarduino `55.03.39`, Arduino `3.3.9`, IDF `5.5.4` | PASS | Targeted HIL 144/144; state-only USB discriminator 10,000/10,000; scheduled-read regression ran 108 cycles in 543.594 s with 564 ordinary passes and two MV3 NACKs, both recovered by one application retry after 1.5 s. |
| pioarduino `54.03.20`, Arduino `3.2.0`, IDF `5.4.1` | MIXED | Safe/extended 33/33, persistent same-value checks 25/25, niche diagnostics, PSRAM smoke, and operator-assisted physical faults passed. The strict 10-minute run and eight-hour soak failed as described below. |
| Historical COM17 | PASS | Safe, extended, resync, interval write/readback/restore, and manually confirmed unplug/replug recovery passed. |

## Findings And Fixes

- The address scanner accepted a sampled ACK as discovery even when the frame
  PEC was invalid. The recorded fix required ACK plus valid PEC; the current
  scanner requires full production begin() identity/capability/feature validation
  followed by a status/PEC read. Historical final scan: address 0 only.
- The example `libtest` command duplicated raw E2 signaling and intermittently
  produced invalid PEC data. It now uses the production driver's bounded,
  clock-stretch-aware, PEC-validating `readControlByte()` path. Final result:
  9/9.
- Unsupported operating-mode access now fails closed with `NOT_SUPPORTED`
  instead of decoding an unadvertised register.
- The reported USB reattachment failure was a host-tool framing error, not a
  stalled HWCDC link. The tool sent a blank line, which the CLI intentionally
  ignores. Explicit `\ndirty\n` synchronization passed 100/100 separate
  process sessions and the full 184/184 HIL again. After HIL closed COM20, a
  new process passed 10,000/10,000 identical state-only replies without a reset
  or cable replug.

These were robustness defects revealed by HIL. The framework-neutral core did
not require Arduino/ESP-IDF compatibility shims for the newer platform.

## Historical Negative Evidence

- Eight-hour `54.03.20` soak: 480 cycles, 2,376 successful commands, 29
  incomplete native-USB replies, and 11 complete MV3 control-byte NACKs. The
  incomplete replies match the Arduino 3.2.0 HWCDC lost-wakeup defect fixed
  upstream in Arduino 3.3.9. Final sensor state remained READY/clean.
- Strict 10-minute `54.03.20` run: 62/63 scheduled commands passed; one bounded
  MV3 NACK occurred at 330 s. The next operations succeeded.
- The NACKs are real sensor-facing failures at the control-byte ACK boundary.
  A retry 1.5 s later succeeded in the later scheduled regression, but a NACK
  alone does not reveal the sensor's internal cause. The core driver therefore
  returns it without hidden retry; retry policy remains application-owned.
- Physical fault HIL passed for sensor-absent startup, hot unplug to OFFLINE,
  explicit replug recovery, SDA stuck low, SCL stuck low/timeout, and complete
  power-cycle interval persistence/restoration.
