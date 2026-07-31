# EE871-E2 Documentation Index

Last updated: 2026-07-31

Use this index to choose the right document. The repository contains maintained
engineering docs, historical audit records, curated protocol notes, and raw
source-document extracts. Do not treat every file under `docs/` as current
release guidance.

## Current Maintained Docs

| Document | Purpose | Current status |
| --- | --- | --- |
| [EE871_E2_HARDWARE_VALIDATION_MATRIX.md](EE871_E2_HARDWARE_VALIDATION_MATRIX.md) | Hardware validation plan plus recorded bench evidence. | Current evidence ledger. |
| [EE871_E2_HIL_RUNNER.md](EE871_E2_HIL_RUNNER.md) | Operator guide for `tools/ee871_hil_runner.py`. | Current HIL runner usage. |
| [IDF_PORT.md](IDF_PORT.md) | ESP-IDF port architecture, constraints, and validation checklist. | Current guidance; ESP32-S3/S2 native IDF example builds pass in CI. |
| [IDF_PORT_IMPLEMENTATION.md](IDF_PORT_IMPLEMENTATION.md) | Short implementation note for files added by the IDF port. | Current implementation summary. |
| [EE871_E2_Protocol_and_Register_Map.md](EE871_E2_Protocol_and_Register_Map.md) | Curated E2 protocol and EE871 register reference. | Current implementation reference; verify exact vendor tables against PDFs when needed. |
| [prompts/README.md](prompts/README.md) | Ordered AI-coder prompt series that closes the general EE871 gaps first, then adds an E2 owner/module exclusively to the Co2Control product in consuming firmware. | Current implementation sequence; supersedes the older single TunnelMonitor-fit prompt and explicitly keeps all non-Co2Control production products E2-disabled. |

## Historical Docs

| Document | How to read it |
| --- | --- |
| [EE871_IDF_MERGED_INDUSTRY_READINESS_AUDIT.md](EE871_IDF_MERGED_INDUSTRY_READINESS_AUDIT.md) | Historical audit from 2026-05-29. Several findings have since been addressed; use the hardware matrix and current maintained docs for current status. |
| [EE871_E2_HARDENING_FINAL_REPORT.md](EE871_E2_HARDENING_FINAL_REPORT.md) | Historical June 2026 hardening snapshot. Use the hardware matrix for current COM20 validation evidence. |
| [EE871_E2_RELEASE_NOTES_1.0.0.md](EE871_E2_RELEASE_NOTES_1.0.0.md) | Historical 1.0.0 release snapshot. Its limitations describe evidence available at release time, not the current COM20 ledger. |
| [EE871_CO2_LIBRARY_PRODUCTION_AUDIT_2026-07-01.md](EE871_CO2_LIBRARY_PRODUCTION_AUDIT_2026-07-01.md) | Point-in-time audit before later hardening and COM20 qualification. Findings must be checked against current code and the hardware matrix. |
| [EE871_TUNNELMONITOR_NODE_SUITABILITY_AUDIT_2026-07-28.md](EE871_TUNNELMONITOR_NODE_SUITABILITY_AUDIT_2026-07-28.md) | Point-in-time cross-repository audit at commit `b5588be`. Later platform/HIL work does not by itself close its P0 API findings. |

## Source Material

Vendor PDFs are kept under `docs/` for traceability:

- [E2_interface_specification_v4_1.pdf](E2_interface_specification_v4_1.pdf)
- [E2_interface_utilising_AN0105.pdf](E2_interface_utilising_AN0105.pdf)
- [EE871_E2_CO2_interface_AN1611-1.pdf](EE871_E2_CO2_interface_AN1611-1.pdf)
- [EE871_E2_interface_addendum.pdf](EE871_E2_interface_addendum.pdf)
- [EE871_digital_interface_user_guide.pdf](EE871_digital_interface_user_guide.pdf)
- [EE871_EE240_wireless_user_guide.pdf](EE871_EE240_wireless_user_guide.pdf)

Searchable extracted text lives in:

- [extracted-md/](extracted-md/) - compact curated notes and source inventory.
- [pdf-extracted-md/](pdf-extracted-md/) - raw text extracted from the PDFs.

Raw extracted text is for search and audit support. When a value affects driver
behavior, prefer the curated protocol reference and verify against the source
PDF if there is any ambiguity.

## Validation Status Snapshot

- Current example/HIL build platform: pioarduino `55.03.311`,
  Arduino-ESP32 `3.3.11`, ESP-IDF `5.5.5`, 4 MB flash, and 2 MB QSPI PSRAM.
- Current `55.03.311` COM20 HIL: 184/184 PASS from clean firmware `3bce89e`;
  safe/extended/niche coverage finished READY with zero transport failures,
  clean persistent state, repeated stress 500/500, and mixed stress 500/500.
- Current native-USB process reattachment: 100/100 separate sessions PASS;
  after full HIL closed COM20, a new process completed 10,000/10,000 identical
  state-only replies without reset or cable replug.
- GitHub Actions native ESP-IDF v6.0.1 example builds pass for ESP32-S3 and
  ESP32-S2.
- Prior `55.03.39` targeted HIL: 144/144 PASS. Its serial-only discriminator:
  10,000/10,000 identical 201-byte `dirty` replies PASS; `dirty` performs no
  E2 operation, so this is CLI-framing evidence rather than a long-soak claim.
- Prior `55.03.39` accelerated scheduled-read regression: PASS over 108 cycles and
  543.594 s, with 564 ordinary passes, two recorded sensor NACK attempts each
  recovered by one 1,500 ms harness retry, and no hard failure, omission,
  reconnect, or counter regression. Final state was READY/clean. This is not a
  completed long soak.
- Prior `55.03.39` unsupported operating-mode HIL: `NOT_SUPPORTED`, no decoded stale
  value, unchanged tracked transport counters, and READY state.
- Historical `54.03.20` ESP32-S3 COM20 safe/extended, persistent-write,
  diagnostics, range/capability, trace/sniffer, stress, and runtime QSPI PSRAM
  HIL: PASS.
- Historical ESP32-S3 COM20 operator-assisted absent-sensor boot, hot
  unplug/OFFLINE/replug recovery, SDA/SCL stuck-low faults, and complete
  sensor/MCU power cycle with measurement-interval persistence: PASS.
- Historical immediate warm-up capture: PASS. Sampling began 0.250 s after COM20
  reappeared; values/status transitioned from `0 ppm`/`0x08` through 4 s to
  `678 ppm`/`0x00` at 5 s; final READY with zero transport failures.
- Historical strict 10-minute COM20 post-power-cycle stability capture: FAIL from
  one bounded NACK among 63 scheduled CLI commands at t=330 s. Manually
  normalized interactive output from the immediate 2,000-operation follow-up
  recorded zero errors; no raw follow-up transcript was retained.
- Historical eight-hour `54.03.20` soak: strict FAIL with 29 Arduino-ESP32
  3.2.0 HWCDC mid-line stalls and 11 real MV3 `0xC1` control-byte NACKs.
  Arduino-ESP32 PR #12606 fixes the HWCDC lost-wakeup path in 3.3.9. The
  current `55.03.311` source passed S2/S3 builds and 184/184 COM20 HIL; no
  completed long soak is recorded for it.
- Historical COM17 safe/persistent and manual unplug/replug evidence is
  retained in the hardware matrix.
