# EE871-E2 Modes, Status, and Faults

## Status Byte

Read control byte 0x71 for status. For EE871 CO2, bit 3 corresponds to CO2 status:

| Bit | Meaning |
|---:|---|
| 0 | Humidity status in generic E2 table; not relevant for EE871 CO2. |
| 1 | Temperature status in generic E2 table. |
| 2 | Air velocity status in generic E2 table. |
| 3 | CO2 status: 0 = last CO2 measurement OK, 1 = error during last CO2 measurement. |

Source: E2 interface specification v4.1, p. 9; AN1611-1, p. 3.

## Measurement Trigger Behavior

- Reading the status byte starts a new measurement in the slave.
- For measuring intervals longer than 15 s, the master can trigger a measurement by reading status byte 0x71 only if the last measurement is older than 10 s.
- After a triggered measurement, the interval counter is set to 0.

Source: AN1611-1, pp. 5, 10.

## Current / Power Modes

AN1611-1 describes sleep, sensor-element stabilization, measurement, infrared-lamp charge/current peak, and communication states. The key driver consequence is to avoid polling loops that keep the module in communication mode or prevent sleep.

## Errors

When status bit3 is 1, read custom-memory address 0xC1 for the EE871/CO2 error code.

| Code | Meaning |
|---:|---|
| 1 | Supply voltage low detected. |
| 200 | Sensor counts low; possible electronics or sensor-cell damage. |
| 201 | Sensor counts high; possible electronics or sensor-cell damage. |
| 202 | Supply-voltage breakdown at the measurement current peak; source notes possible excessive internal resistance of the supply unit. |

Source: AN1611-1, pp. 3-9.
