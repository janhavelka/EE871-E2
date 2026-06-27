# EE871 ESP-IDF Diagnostic Basic Bring-Up CLI

This is a diagnostic/basic bring-up example. It owns the E2 GPIO lines for
demonstration and injects callbacks into the framework-neutral EE871 driver.
Production applications should integrate the callbacks into their own GPIO or
bus manager instead of copying this example's bus ownership model directly.

EE871-E2 uses GPIO-style E2 signaling, not ESP-IDF `driver/i2c_master` or
hardware I2C. If multiple tasks can touch the same `EE871` instance or shared
E2 lines, serialize access outside the driver.

The CLI provides the same user-visible command set as the Arduino bring-up CLI:
help, version, scan, probe, recover, driver health, raw CO2 reads,
checked CO2 sample reads,
feature/capability inspection, configuration/calibration helpers, register/raw
access, diagnostics, bus reset, trace, self-test, stress, and mixed stress
workflows.

`co2fast` and `co2avg` read raw MV3/MV4 values. `samplefast` and `sampleavg`
use the checked value-then-status helpers and can trigger a new measurement
because they read the EE871 status byte.

- Default SCL: GPIO6
- Default SDA: GPIO7
- Default E2 address: `0`
- Pull-ups: external pull-ups are expected; internal weak pull-ups are disabled
  by default in `ee871_idf::init()`.

Change the GPIO constants in `main/main.cpp` for your board. Build and monitor
with ESP-IDF:

```bash
idf.py -C examples/idf/basic_bringup set-target esp32s3 build
idf.py -C examples/idf/basic_bringup flash monitor
```

Use `idf.py -C examples/idf/basic_bringup set-target esp32s2 build` for
ESP32-S2 validation. The driver core does not configure GPIO, own the bus, or
log.
