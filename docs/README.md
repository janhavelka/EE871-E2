# EE871-E2 Documentation Index

Last updated: 2026-08-26

Use this index to choose the right document. The repository contains maintained
engineering docs, curated protocol notes, and raw source-document extracts.

## Current Maintained Docs

| Document | Purpose | Current status |
| --- | --- | --- |
| [EE871_E2_HARDWARE_VALIDATION_MATRIX.md](EE871_E2_HARDWARE_VALIDATION_MATRIX.md) | Hardware validation plan plus recorded bench evidence. | Current evidence ledger. |
| [EE871_E2_HIL_RUNNER.md](EE871_E2_HIL_RUNNER.md) | Operator guide for `tools/ee871_hil_runner.py`. | Current HIL runner usage. |
| [IDF_PORT.md](IDF_PORT.md) | ESP-IDF port architecture, constraints, and validation checklist. | Current guidance; ESP32-S3/S2 native IDF example builds pass in CI. |
| [IDF_PORT_IMPLEMENTATION.md](IDF_PORT_IMPLEMENTATION.md) | Short implementation note for files added by the IDF port. | Current implementation summary. |
| [EE871_E2_Protocol_and_Register_Map.md](EE871_E2_Protocol_and_Register_Map.md) | Curated E2 protocol and EE871 register reference. | Current implementation reference; verify exact vendor tables against PDFs when needed. |
| [AUDIT_FINDINGS_2026-08-26.md](AUDIT_FINDINGS_2026-08-26.md) | Open audit findings with concrete proposals. | Working list; delete sections as they are resolved. |

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

## Validation Status

Bench/HIL evidence is maintained in exactly two places; do not duplicate it here:

- [hil_results/README.md](../hil_results/README.md) - summary of the latest
  qualification runs.
- [EE871_E2_HARDWARE_VALIDATION_MATRIX.md](EE871_E2_HARDWARE_VALIDATION_MATRIX.md) -
  full plan and recorded evidence ledger.

Historical audits, prompt series, and release-note snapshots that used to live
under `docs/` were removed on 2026-08-26; recover them from git history if
needed.
