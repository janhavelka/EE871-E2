# EE871-E2 Register Map

E2 custom memory is byte-addressed. Use control byte 0x50 to set the internal pointer and 0x51 to read sequential bytes. Use control byte 0x10 only for documented R/W custom-memory bytes.

## Generic E2 Custom Memory

| Address | Access | Meaning | Notes |
|---:|---|---|---|
| 0x00 | R | Firmware main version | 0x55/0x55 indicates no command support in generic spec wording. |
| 0x01 | R | Firmware sub-version | Generic E2 definition. |
| 0x02 | R | E2 spec version | Version used during product development. |
| 0x03..0x06 | R | Supported-function bit fields | Generic E2 feature-discovery bytes for adjustment/timestamp features. Bit value 1 means the function exists; 0 means it does not. |
| 0x07 | R | Operating-function support | bit0 serial number, bit1 part name, bit2 E2 bus address, bit3 reserved, bit4 global measurement interval, bit5 specific measurement interval, bit6 measurement-value filter, bit7 error code. |
| 0x08 | R | Operating-mode support | bit0 low-power mode, bit1 E2 priority, bits2..7 reserved. |
| 0x09 | R | Special-feature support | bit0 manual auto adjustment, bits1..7 reserved. |
| 0x58..0x59 | R/W | CO2 offset | Signed int ppm. |
| 0x5A..0x5B | R/W | CO2 gain | Gain value / 32768. |
| 0x5C..0x5F | R/W | CO2 lower/upper adjustment points | ppm values. |
| 0x8C..0x8E | R/W | Last CO2 custom adjustment date | Year/month/day. |
| 0xA0..0xAF | R | E+E serial number | Unique serial number. |
| 0xB0..0xBF | R/W | Part name | Filled with sensor type on delivery, e.g. EE871. |
| 0xC0 | R/W | E2 bus address | Generic spec range 0..7; delivery default 0. |
| 0xC1 | R/W | Error code | Read when status byte bit3 reports a CO2 error. EE871/CO2 codes: 1 supply voltage low, 200 sensor counts low, 201 sensor counts high, 202 supply-voltage breakdown at measurement current peak. |
| 0xC6 | R/W | Global measurement interval low byte | Unsigned 16-bit value in 0.1 s units, low byte first. Valid EE871/CO2 range is 150..36000, equal to 15..3600 s. |
| 0xC7 | R/W | Global measurement interval high byte | Write both 0xC6 and 0xC7; firmware starts the flash write after both bytes and can hold the bus up to 300 ms. |
| 0xCB | R/W | Specific interval CO2 | Generic E2 definition: positive value multiplies the global interval; negative value divides it. The EE871 CO2 notes do not define a reset value or exact signed encoding. |
| 0xD3 | R/W | CO2 filter | Generic E2 definition says the filter is product-specific; the EE871 CO2 notes identify the parameter but do not define numeric values. |
| 0xD8 | R/W | Operating mode | bit0 measure mode: 0 free-running/trigger mode, 1 low-power mode; bit1 E2 priority: 0 measurement priority/NACK during measurement, 1 E2 communication priority; bits2..7 reserved. |
| 0xD9 | R/W | Auto adjustment control/status | bit0 read 1 while auto adjustment is running, read 0 in normal operation; write 1 starts auto adjustment; write 0 cannot interrupt it; bits1..7 reserved. |

Source: E2 interface specification v4.1, pp. 12-14; EE871 E2 CO2 interface AN1611-1, pp. 2-8.

## Measurement Interval

AN1611-1 says to check global-interval support by reading custom address 0x07 and requiring bit4 = 1. Read 0xC6 first and then 0xC7 with 0x51; the pointer auto-increments after the low byte.

Formula: interval_seconds = uint16(0xC7:0xC6) / 10.

To write a new interval, compute raw = seconds * 10, write low byte to 0xC6 and high byte to 0xC7 with 0x10. Values below 150 (15 s) and above 36000 (3600 s) are ignored by the firmware.

Source: AN1611-1, pp. 8-9.
