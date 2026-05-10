# EE871-E2 Chip Overview

## Device Identity

| Item | Notes |
|---|---|
| Device | E+E Elektronik EE871 CO2 probe with E2 digital interface. |
| Measurement principle | Dual-wavelength NDIR CO2 sensing cell with factory CO2 and temperature adjustment. |
| Digital protocols | E2 or Modbus RTU depending on ordered interface variant. This library targets E2. |
| CO2 range | Digital-interface guide states up to 5% CO2, equivalent to 50,000 ppm. Wireless EE240 guide states up to 10,000 ppm for that system context. |
| E2 device group | EE871 group value is 0x0367 (decimal 871). |
| E2 subgroup | 0x09 for the CO2 module family. |

Sources: EE871 digital interface user guide, p. 1-2; EE871 E2 CO2 interface AN1611-1, pp. 2-3; E2 protocol/register reference.

## Driver Scope

- Implement an E2 master for a slave probe; the probe never initiates communication.
- Read CO2 through E2 measurement value 3 or 4.
- Support documented EE871 custom-memory fields: 0xC0 bus address (0..7, default 0), 0xC1 CO2 error code, 0xC6/0xC7 global measurement interval (15..3600 s, 0.1 s units), 0xD3 CO2 filter identifier, 0xD8 operating mode bits, and 0xD9 auto-adjustment bit.
- Keep Modbus RTU facts separate from the E2 transport; the same physical probe family can be ordered with either interface.
