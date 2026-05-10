# EE871-E2 Variant Differences and Open Questions

## Variants / Context

| Topic | Notes |
|---|---|
| Wired EE871 digital-interface guide | Lists Modbus RTU or E2 options, supply 4.75 VDC to 7.5 VDC, E2 cable length up to 10 m, and CO2 range up to 5% / 50,000 ppm. |
| EE240 wireless user guide | Describes EE871 as a CO2 probe for the EE240 wireless sensor network and lists up to 10,000 ppm measured data range. Treat this as wireless-system context, not the wired E2 driver limit. |
| Generic E2 specification | Applies to E+E transmitters unless product-specific documentation says otherwise. |
| CO2 E2 addendum / AN1611-1 | Product-family source for EE871/EE892/EE893 CO2 measurements, current profile, intervals, and CO2 value selection. |

## Open Questions

- Exact connector pinout should be verified against the ordered probe drawing before committing board-level documentation.
- The E2 custom-memory map is generic; only use addresses confirmed by EE871/CO2 notes or by device probing.
- The complete error-code table is not cleanly represented in compact notes; use the source PDF before exposing public symbolic error codes.
- Wireless EE240 timing/range facts should not be merged into the wired E2 API unless the same fact is present in wired E2 sources.

Sources: EE871 digital interface user guide, pp. 1-2; EE871 EE240 wireless user guide, pp. 1-2; E2 specification v4.1; AN1611-1.
