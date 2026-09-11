# EE871-E2 Initialization and Operational Notes

## Suggested Bring-Up

1. Configure open-drain clock/data GPIOs and pull-ups for the chosen bus-high voltage.
2. Verify both lines idle high before issuing START.
3. Read sensor type low/high bytes with control bytes 0x11 and 0x41; expect group 0x0367 for EE871.
4. Read subgroup with 0x21; expect 0x09 for the CO2 module family.
5. Read available physical measurements with 0x31; verify CO2 support.
6. Allow the application's 5..10 s power-up warm-up policy before relying on readings. Read measurement value 4 for averaged CO2 or value 3 for fast response, low byte before high byte. For checked sampling, read status with 0x71 afterward to evaluate the last value; status can trigger another measurement only when the interval is >15 s and the previous measurement is >10 s old.

Sources: E2 specification v4.1, pp. 7-10; AN1611-1, pp. 2-5.

## Reset / Pointer State

- The internal custom-memory pointer defaults to 0x0000 after power-up.
- Set it explicitly with command 0x50 before sequential 0x51 reads.
- The pointer auto-increments after reads.
- `busReset()` is a bounded, health-neutral line reset. It does not clear OFFLINE; only successful explicit `recover()` revalidates identity/capabilities and restores ordinary operation.

Source: E2 specification v4.1, pp. 8, 10.

## Write Cautions

- A slave ACK only confirms the byte transfer, not necessarily that a later checksum-valid write has completed internally.
- AN1611-1 notes that flash writes interrupt E2 communication; attempts to communicate during a write can force clock-low extension until the write routine finishes.
- Always write both interval bytes 0xC6 and 0xC7 when changing measurement interval.
- A direct custom write may need 150 ms; the interval pair may need 300 ms. The volatile 0x50 pointer update has no flash delay and uses the ordinary STOP timeout.
- Persistent writes require bounded commit waits and readback verification. Partial or uncertain writes remain visible through dirty/resync diagnostics until a documented verification path succeeds.

Sources: E2 specification v4.1, pp. 10-11; AN1611-1, p. 8.
