# EE871-E2 Electrical and Timing Notes

## Supply and Current

| Parameter | Value | Source |
|---|---|---|
| Probe supply | 4.75 VDC to 7.5 VDC | EE871 digital interface user guide, p. 2 |
| Average current | 120 uA at 1 h interval to 4.3 mA at 15 s interval | EE871 digital interface user guide, p. 2 |
| Sleep mode current | 40 uA minimum | AN1611-1, p. 9 |
| Communication mode current | 3.4 mA | AN1611-1, p. 9 |
| Measurement peak | Up to about 1.2 A during infrared-lamp charge / measurement peak | AN1611-1, p. 9 |

## E2 Timing

| Timing item | Value / rule | Source |
|---|---|---|
| Clock frequency | 500 Hz to 5000 Hz | E2 specification v4.1, p. 4 |
| Clock high / low | At least 100 us high and 100 us low | E2 specification v4.1, p. 4 |
| START setup | Data falling while clock high, then at least 4 us before clock low | E2 specification v4.1, p. 5 |
| Slave clock stretching | Slave may hold clock low after bits/bytes; implementation reference uses 25 ms bit and 35 ms byte timeouts | E2 specification v4.1, p. 5; AN0105 |

## Implementation Consequences

- Do not rely on hardware I2C peripherals unless they can run slowly enough, expose open-drain clock readback, and tolerate the E2 addressing/checksum format.
- The master must read back released clock before sampling data so clock stretching works.
- Avoid unnecessary bus traffic; AN1611-1 notes that communication can prevent the CO2 module from returning to sleep.
