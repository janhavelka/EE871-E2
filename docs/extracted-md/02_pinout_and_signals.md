# EE871-E2 Pinout and Signals

## E2 Bus Signals

| Signal | Direction | Electrical behavior | Notes |
|---|---|---|---|
| Clock / SCL | Master-driven | Open-drain/open-collector with pull-up; idle high. | Master supplies the clock. Slave may stretch low. |
| Data / SDA | Bidirectional | Open-drain/open-collector with pull-up; idle high. | Data changes only while clock is low except START/STOP. |
| GND | Reference | Common ground/reference. | Required by the E2 physical layer. |

Source: E2 interface specification v4.1, pp. 3-6.

## EE871 Connector Notes

The EE871 digital-interface guide shows the E2 version on an M12x1 connector with supply, ground, data, and clock. The curated protocol reference records the mapping as pin 1 GND, pin 2 +UB, pin 3 DATA, pin 4 CLOCK. Verify against the ordered probe drawing before wiring production hardware.

Sources: EE871 digital interface user guide, p. 2; EE871 protocol/register reference.

## Bus Implementation Notes

- Use pull-ups on both clock and data; the pull-up rail defines bus-high voltage.
- The CO2 E2 note gives a likely bus-high range of 3.6 V to 5.2 V and recommends 4.5 V to 5.0 V for lower supply current.
- Keep E2 cable length to 10 m or less for the EE871 guide.
- If the host MCU is not 5 V tolerant, use an open-drain level shifter that preserves clock stretching.

Sources: EE871 digital interface user guide, p. 2; EE871 E2 CO2 interface AN1611-1, p. 8.
