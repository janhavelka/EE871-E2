# EE871 ESP-IDF Diagnostic Basic Bring-Up CLI

This example owns two GPIO lines and injects native ESP-IDF callbacks into the
framework-neutral driver. It uses GPIO-style E2 signaling, not hardware I2C.
Production applications must integrate that ownership into their bus manager
and serialize all access to the driver and shared lines.

## Wiring

- SCL: GPIO7; SDA: GPIO6; E2 device address: `0`.
- Change the GPIO constants in `main/main.cpp` for your board.
- External pull-ups are required; internal weak pull-ups are disabled by default.
- Use a bidirectional open-drain level shifter between 3.3 V GPIOs and the
  sensor-side bus. Pull-ups are 4.7 kOhm to 100 kOhm, with bus high at 3.6-5.2 V
  (recommended 4.5-5.0 V). The cable-length guideline is 10 m maximum.

Check the ordered probe's connector and supply requirements in the
[protocol reference](../../../docs/EE871_E2_Protocol_and_Register_Map.md).

## Build and run

From the repository root in an initialized ESP-IDF 6.0.1 or later environment:

```sh
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup flash monitor
```

Use `esp32s2` for the other target and `-p PORT` when selecting a serial port.
Keep the checkout directory named `EE871-E2`, or update the main component's
`REQUIRES "EE871-E2"` entry to match. The example locates the core through
`EXTRA_COMPONENT_DIRS`; the [integration guide](../../../docs/IDF_PORT.md)
explains component dependencies, GPIO callbacks and FreeRTOS wait behavior.

## Commands and limits

Type `help` for the commands, which match the Arduino bring-up CLI. The
[HIL/CLI guide](../../../docs/EE871_E2_HIL_RUNNER.md) describes the detailed command,
confirmation and evidence rules:

- `read`, `co2fast` and `co2avg` return raw values. `co2fastsample` and
  `co2avgsample` also read sensor status after the value; status can trigger
  the next measurement. Checked success does not establish freshness or warm-up.
- `operation` prints cached persistent-operation evidence without bus I/O.
  Maintenance admission blocks unresolved state. `resync` proves current
  coherence, not that the original requested write or calibration succeeded.
- `bounds` calculates requested-delay sums without bus I/O. It excludes
  callback and scheduler overhead. `timing` performs live bus tests.
- `sniff` prints synchronously from the transport path and perturbs timing;
  sniffer-enabled traffic is unsuitable for timing or stability qualification.

The command loop owns the driver, calls `tick()` and yields between commands.
Read retries default to zero. Native FreeRTOS callbacks provide cooperative
post-write quiet waits; bus operations remain synchronous. This example does
not implement production sampling or automatic recovery. Build and hardware
results are recorded separately in the
[validation matrix](../../../docs/EE871_E2_HARDWARE_VALIDATION_MATRIX.md).
