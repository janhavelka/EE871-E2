# EE871-E2 Protocol Commands and Transactions

## Byte Framing

- All bytes transfer MSB first.
- The first byte is the E2 control byte: high nibble main command, bits 3..1 slave address, bit 0 read/write.
- ACK is data low on the ninth bit; NACK is data high.
- The packet error code (PEC) is the low byte of the sum of transmitted bytes.

Source: E2 interface specification v4.1, pp. 5-7.

## Read Commands

| Control-byte base | Purpose | Driver note |
|---|---|---|
| 0x11 | Sensor type low byte | EE871 returns 0x67 for group low. |
| 0x21 | Sensor subgroup | EE871 subgroup is 0x09. |
| 0x31 | Available physical measurements | EE871 CO2 bit is set. |
| 0x41 | Sensor type high byte | EE871 returns 0x03 for group high. |
| 0x51 | Read internal custom address | Reads current custom pointer and auto-increments. |
| 0x71 | Status byte | Also triggers a measurement under defined timing conditions. |
| 0x81..0xF1 | Measurement value bytes | Low/high byte pairs for measurement values 1..4. |

## Write Commands

| Control-byte base | Purpose | Driver note |
|---|---|---|
| 0x10 | Direct write to custom area | Address byte selects custom address; data byte is value. |
| 0x50 | Set internal custom pointer | Address byte = high pointer byte; data byte = low pointer byte. |

## EE871 CO2 Values

- Measurement value 3 is fast-response CO2, unsigned 16-bit ppm, not averaged.
- Measurement value 4 is the standard averaged CO2 value, unsigned 16-bit ppm, calculated from the last 11 measurements.
- At 15 s measuring interval, AN1611-1 gives about tau90 = 60 s for value 3 and tau90 = 105 s for value 4.

Sources: EE871 E2 CO2 interface AN1611-1, pp. 3, 9-10; E2 specification v4.1, pp. 7-11.
