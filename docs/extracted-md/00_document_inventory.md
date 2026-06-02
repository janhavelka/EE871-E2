# EE871-E2 Document Inventory

Compact driver notes live in `docs/extracted-md/01_*.md` through `08_*.md`.
Raw text extracted from the PDFs lives in `docs/pdf-extracted-md/`; use it for
search, then verify exact tables against the PDFs under `docs/`.

| Source PDF / note | Pages | Use for driver work |
|---|---:|---|
| `E2_interface_specification_v4_1.pdf` | 20 | Generic E2 physical layer, byte framing, control byte, checksum, read/write commands, custom memory map. |
| `E2_interface_utilising_AN0105.pdf` | 18 | Bit-banged master implementation examples and timing guidance. |
| `EE871_E2_CO2_interface_AN1611-1.pdf` | 12 | EE871/EE892/EE893 CO2-specific E2 values, measurement timing, current profile, CO2 value selection. |
| `EE871_E2_interface_addendum.pdf` | 3 | Small addendum for EE871 E2 behavior. |
| `EE871_digital_interface_user_guide.pdf` | 2 | EE871 product identity, supply, range, connector, E2 cable length, Modbus/E2 ordering context. |
| `EE871_EE240_wireless_user_guide.pdf` | 2 | Wireless-network variant context; not the primary driver protocol source. |
| `EE871_E2_Protocol_and_Register_Map.md` | n/a | Curated implementation reference already cross-checked against the PDFs. |

Source priority: CO2-specific EE871 notes override generic E2 behavior when they differ. Do not use wireless-network documentation to change the wired E2 driver unless the same fact is present in the E2 sources.
