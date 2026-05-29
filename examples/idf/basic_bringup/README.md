# EE871 ESP-IDF Basic Bring-Up CLI

This example owns the E2 GPIO lines and injects callbacks into the
framework-neutral EE871 driver. It provides the same user-visible command set
as the Arduino bring-up CLI: help, version, scan, probe, recover, driver
health, CO2 reads, feature/capability inspection, configuration/calibration
helpers, register/raw access, diagnostics, bus reset, trace, self-test, stress,
and mixed stress workflows.

- Default SCL: GPIO6
- Default SDA: GPIO7
- Default E2 address: `0`
- Pull-ups: external pull-ups are expected; internal weak pull-ups are disabled
  by default in `ee871_idf::init()`.

Change the GPIO constants in `main/main.cpp` for your board. Build and monitor
with ESP-IDF:

```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Use `idf.py set-target esp32s2` for ESP32-S2 validation. The driver core does
not configure GPIO, own the bus, or log.
