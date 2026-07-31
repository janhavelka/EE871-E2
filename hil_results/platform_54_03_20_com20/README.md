# COM20 ESP32-S3 HIL Evidence

Date: 2026-07-30

This directory contains the authoritative HIL evidence for the ESP32-S3 COM20
bench on pioarduino `platform-espressif32` `54.03.20`. Superseded runner
attempts and wrapper-only console logs were removed after their replacement
runs were verified.

## Automated Runner Evidence

- `safe_extended_settle_final/ee871_20260730T105954Z/` - safe plus extended
  plan, 33/33 PASS.
- `persistent_calibration_settle_final/ee871_20260730T105933Z/` - approved
  same-value interval/offset/gain maintenance plan, 25/25 PASS.
- `prompt_newline_postfix/ee871_20260730T152316Z/` - final prompt-framing
  regression run, 10/10 PASS.
- `runtime_psram_final/ee871_20260730T110655Z/` - runtime flash/PSRAM smoke,
  10/10 PASS.
- `overnight_preflight/ee871_20260730T172216Z/` - immediate pre-soak safe
  baseline, 10/10 PASS.
- `overnight_soak_20260730T173126Z/` - eight-hour soak, strict FAIL:
  2,376 PASS, 29 incomplete HWCDC replies, 11 real scheduled MV3 control-byte
  NACKs, and 41 planned command records absent after framing timeouts. The old
  runner emitted no explicit SKIP rows for those unsent commands.

For successful generated runs, the Markdown command ledger is authoritative;
duplicate raw serial and JSON outputs were removed. The failed overnight soak
retains its raw transcript plus compact JSON because its incomplete replies
and NACKs are unique negative evidence.

## Focused Diagnostic Evidence

- `niche_diagnostics_20260730.md` - curated identity/capability, guard,
  GPIO/E2 diagnostic, trace/sniffer, mixed-stress, and same-value part-name
  evidence condensed from five ad-hoc normalized console captures.

## Physical And Power-Cycle Evidence

- `operator_physical_faults_20260730.md`
- `post_power_cycle_stability_20260730.txt`
- `warmup_stale_followup_20260730.md`
- `immediate_warmup_final_20260730.txt`
- `immediate_warmup_delayed_start_20260730.txt`
- `immediate_warmup_attempts_20260730.md`

The immediate-warm-up final run is PASS. The 10-minute stability capture is
strictly FAIL because one of 63 scheduled CLI commands returned a bounded
NACK. The delayed-start warm-up attempt records a second bounded NACK and is
retained as negative transport evidence. Both failures recovered on the next
successful operation; neither is hidden or reclassified.

The eight-hour soak is also retained as negative evidence, not promoted to
PASS. Its 29 incomplete replies were reproduced interactively with a
serial-only command; the exact 64-byte-boundary truncation/queued-suffix pattern
matches the Arduino-ESP32 3.2.0 HWCDC lost-wakeup defect fixed upstream. The
serial-only raw stream was not retained, so the characterization note states
that limitation. Its 11 complete NACK replies originated at the sensor-facing
transaction boundary; their sensor-internal cause is not inferred from NACK
alone.

- `serial_only_hwcdc_characterization_20260731.md` - retrospective lab note for
  the pre-upgrade serial-only discriminator; exact counts and missing-raw
  boundary are stated explicitly.

The operator-assisted report explicitly identifies its normalized interactive
origin. The stability follow-up likewise identifies observations for which no
raw follow-up transcript was retained.

## Scope

These artifacts validate the recorded COM20 command paths, bounded failures,
recovery, persistent-state diagnostics, and runtime memory configuration. They
do not establish CO2 accuracy, calibration accuracy, ESP32-S2 hardware
behavior, pure ESP-IDF hardware behavior, or long-term field reliability.
