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

Each runner directory contains the raw serial transcript, structured JSON, and
Markdown summary produced by `tools/ee871_hil_runner.py`.

## Focused Diagnostic Evidence

- `full_diagnostics_final.txt`
- `range_capability_guards_final.txt`
- `trace_sniffer_mixed_stress_final.txt`
- `read_identity_diagnostics_final.txt`
- `part_name_write_readback_raw.txt`

`part_name_write_readback_raw.txt` is retained only for its clean same-value
part-name write/readback blocks. The broader ad-hoc session contains unrelated
harness expectation failures and is not an overall PASS artifact.

## Physical And Power-Cycle Evidence

- `operator_physical_faults_20260730.md`
- `operator_physical_faults_20260730_serial_transcript.txt`
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

The operator-assisted transcript is explicitly normalized, not byte-for-byte
raw serial. The stability follow-up report explicitly identifies manually
normalized interactive observations for which no raw follow-up transcript was
retained.

## Scope

These artifacts validate the recorded COM20 command paths, bounded failures,
recovery, persistent-state diagnostics, and runtime memory configuration. They
do not establish CO2 accuracy, calibration accuracy, ESP32-S2 hardware
behavior, pure ESP-IDF hardware behavior, or long-term field reliability.
