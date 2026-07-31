# COM20 Operator-Assisted Physical HIL

Date: 2026-07-30

Claim boundary: these results cover the listed ESP32-S3/EE871 bench operations
only. They do not prove CO2 accuracy, warm-up policy, long-soak behavior,
calibration accuracy, ESP32-S2 hardware, or pure ESP-IDF hardware.

This report retains the command outcomes and measured durations transcribed
during the operator phases. The duplicate normalized console transcript was
removed during the 2026-07-31 evidence condensation. The source session was
not a byte-for-byte serial capture and had no independently available
wall-clock timestamps.

## Setup

- Port: `COM20`, 115200 baud.
- Board: ESP32-S3 revision 0.2, embedded 4 MB flash, embedded 2 MB QSPI PSRAM.
- Target: PlatformIO `ex_bringup_s3`.
- Platform: pioarduino `platform-espressif32` `54.03.20`,
  Arduino-ESP32 `3.2.0`, ESP-IDF headers `5.4.1`.
- Firmware build: `Jul 30 2026 13:06:41`.
- Library: `1.0.0 (1fbe7d8, 2026-07-30 13:06:39, dirty)`.
- E2 pins: DATA/SDA GPIO6, CLOCK/SCL GPIO7.
- Fault resistor: operator-confirmed 470 ohm from the selected MCU-side E2 line
  to common ground.
- Physical unplug, replug, resistor application/removal, and complete power
  cycle were confirmed by the user during the HIL session.
- Pull-up values, supply voltage, level-shifter model, cable length, and ambient
  conditions were not independently measured.

## Sensor-Absent Boot

- The EE871 was unplugged before an MCU reset.
- `begin()` failed closed with `NACK` (`Control byte NACK`) and no hang.
- Driver state remained `UNINIT`; health success/failure counters remained zero.
- SCL and SDA were both high and the pin-toggle diagnostic passed.
- `probe` and normal reads returned `NOT_INITIALIZED`.
- Persistent dirty state remained clean.

Result: PASS.

## Hot Unplug And Replug

- Baseline before unplug: READY, success 2, failures 0, dirty clean.
- Opening the corrected HIL serial connection produced no reset output and
  preserved the baseline driver state.
- With the sensor unplugged, `buscheck` reported both lines high.
- Raw `probe` returned `NACK` in 15 ms and did not change health counters.
- Five tracked `read` operations returned `NACK` and transitioned:
  READY -> DEGRADED (failures 1-4) -> OFFLINE (failure 5).
- An additional `status` operation while OFFLINE returned bounded `NACK`;
  consecutive and total failures became 6.
- Dirty state remained clean.
- After replug, state was still OFFLINE with the same counters.
- Explicit `recover` returned OK in 16 ms, reset consecutive failures to zero,
  and restored READY.
- Post-recovery CO2 read returned OK (`669 ppm` observed).
- Selftest passed 27/27.
- `stress_mix 100` passed 100/100 with zero errors and +187 tracked successes.

Result: PASS.

## SDA Stuck Low

- With 470 ohm from SDA/GPIO6 to ground, levels reported SCL high/SDA low.
- `buscheck` returned `BUS_STUCK` (`SDA stuck low`).
- Raw `probe` returned `PEC_MISMATCH` without changing health counters.
- A tracked read returned `PEC_MISMATCH` in 16 ms and transitioned READY to
  DEGRADED with one consecutive failure.
- Diagnostic recovery clocks ended with SDA still low.
- Library bus reset returned `BUS_STUCK` (`Bus stuck after reset`).
- Dirty state remained clean.
- After releasing SDA, both lines returned high.
- Explicit `recover` returned OK in 16 ms and restored READY.
- Post-recovery CO2 read returned OK (`683 ppm` observed).

Result: PASS.

## SCL Stuck Low

- With 470 ohm from SCL/GPIO7 to ground, levels reported SCL low/SDA high.
- `buscheck` returned `BUS_STUCK` (`SCL stuck low`).
- Raw `probe` returned `TIMEOUT` in 31 ms with detail `25000` us and did not
  change health counters.
- A tracked read returned `TIMEOUT` in 32 ms with detail `25000` us and
  transitioned READY to DEGRADED with one consecutive failure.
- Diagnostic recovery clocks could not raise SCL.
- Library bus reset returned `BUS_STUCK` (`SCL stuck during reset`) in 31 ms.
- Dirty state remained clean.
- After releasing SCL, both lines returned high.
- Explicit `recover` returned OK in 16 ms and restored READY.
- Post-recovery CO2 read returned OK (`689 ppm` observed).
- Selftest passed 27/27.
- `stress_mix 100` passed 100/100 with zero errors and +187 tracked successes.

Result: PASS.

## Complete Sensor/MCU Power Cycle With Measurement-Interval Persistence

- Recorded baseline interval: `150 ds`; dirty state clean.
- Wrote interval `160 ds` in 344 ms and verified `160 ds`; dirty state clean.
- The operator disconnected ESP32-S3 USB power and EE871 power, waited at least
  five seconds, then reconnected both.
- After the complete power cycle:
  - driver state was READY with zero failures;
  - runtime reported 4,194,304 bytes flash and PSRAM ready with 2,097,152 bytes;
  - interval read back as `160 ds`;
  - CO2 offset remained `0 ppm`;
  - CO2 gain remained `32768`;
  - part name remained `EE871`;
  - dirty state remained clean.
- Restored interval `150 ds` in 344 ms and verified `150 ds`.
- `resync` returned OK in 203 ms and dirty state remained clean.
- Final selftest passed 27/27.
- Final `stress_mix 100` passed 100/100 with zero errors.
- Final driver state was READY with 331 successes, zero failures, and clean
  persistent state.

Result: PASS. Baseline configuration was restored.

## Overall Result

PASS for sensor-absent boot, hot unplug/OFFLINE/replug recovery, SDA stuck-low,
SCL stuck-low/clock-timeout recovery, and a complete sensor/MCU power cycle with
measurement-interval persistence on the recorded `54.03.20` COM20 ESP32-S3
bench.
