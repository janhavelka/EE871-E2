# EE871-E2 Documentation Index

Last updated: 2026-06-27

Use this index to choose the right document. The repository contains maintained
engineering docs, historical audit records, curated protocol notes, and raw
source-document extracts. Do not treat every file under `docs/` as current
release guidance.

## Current Maintained Docs

| Document | Purpose | Current status |
| --- | --- | --- |
| [EE871_E2_HARDENING_FINAL_REPORT.md](EE871_E2_HARDENING_FINAL_REPORT.md) | Running hardening, validation, and release-readiness record for the branch. | Current branch report. |
| [EE871_E2_HARDWARE_VALIDATION_MATRIX.md](EE871_E2_HARDWARE_VALIDATION_MATRIX.md) | Hardware validation plan plus recorded bench evidence. | Current evidence ledger. |
| [EE871_E2_HIL_RUNNER.md](EE871_E2_HIL_RUNNER.md) | Operator guide for `tools/ee871_hil_runner.py`. | Current HIL runner usage. |
| [EE871_E2_RELEASE_NOTES_1.1.0.md](EE871_E2_RELEASE_NOTES_1.1.0.md) | Release summary, validation status, limitations, and tagging checklist for `1.1.0`. | Current release notes. |
| [EE871_E2_RELEASE_NOTES_1.0.0.md](EE871_E2_RELEASE_NOTES_1.0.0.md) | Release summary, validation evidence, limitations, and tagging checklist for `1.0.0`. | Previous release notes. |
| [IDF_PORT.md](IDF_PORT.md) | ESP-IDF port architecture, constraints, and validation checklist. | Current guidance; pure IDF build success remains unproven until CI or local `idf.py` proves it. |
| [IDF_PORT_IMPLEMENTATION.md](IDF_PORT_IMPLEMENTATION.md) | Short implementation note for files added by the IDF port. | Current implementation summary. |
| [EE871_E2_Protocol_and_Register_Map.md](EE871_E2_Protocol_and_Register_Map.md) | Curated E2 protocol and EE871 register reference. | Current implementation reference; verify exact vendor tables against PDFs when needed. |

## Historical Docs

| Document | How to read it |
| --- | --- |
| [EE871_IDF_MERGED_INDUSTRY_READINESS_AUDIT.md](EE871_IDF_MERGED_INDUSTRY_READINESS_AUDIT.md) | Historical audit from 2026-05-29. Several findings have since been addressed on `hardening/ee871-e2-industry-readiness`; use the hardening report and hardware matrix for current status. |

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

- Arduino ESP32-S3 safe HIL: recorded PASS on `COM17`.
- Arduino ESP32-S3 persistent interval write/readback/restore: recorded PASS on
  `COM17`.
- Physical unplug/replug recovery: recorded PASS as an operator-confirmed
  manual test; no automated transcript artifact is recorded.
- ESP32-S2 hardware HIL: not recorded.
- Pure ESP-IDF build success: CI job is configured, but no local or GitHub
  Actions pass evidence is recorded in this workspace.
- 1.1.0 absent-startup, checked-sample, identity-validation, and
  cooperative-delay hardware HIL: not recorded.
- Stuck-line fault-jig validation: not recorded.
