# EE871 ESP-IDF Basic Bring-Up

This example owns the E2 GPIO lines and injects callbacks into the
framework-neutral EE871 driver.

- Default SCL: GPIO9
- Default SDA: GPIO8
- Default E2 address: `0`
- Pull-ups: external pull-ups are expected; internal weak pull-ups are disabled
  by default in `ee871_idf::init()`.

Change the GPIO constants in `main/main.cpp` for your board. The driver core
does not configure GPIO, own the bus, or log.
