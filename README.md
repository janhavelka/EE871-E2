# EE871 E2 Driver Library

Production-grade EE871 CO2 sensor driver for the E2 bus on ESP32 (Arduino/PlatformIO).

## Features

- **E2 bus HAL injection** - no Wire/I2C dependency in library code
- **Health monitoring** - READY/DEGRADED/OFFLINE tracking
- **Deterministic behavior** - bounded loops, explicit timeouts
- **Managed synchronous** - blocking transfers with spec-compliant limits
- **Feature guards** - optional EE871 registers are checked from cached capability flags

## Installation

### PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps =
  https://github.com/janhavelka/EE871-E2.git
```

### Manual

Copy `include/EE871/` and `src/` to your project.

## Quick Start

```cpp
#include <Arduino.h>
#include "EE871/EE871.h"

struct E2BusPins {
  int scl;
  int sda;
};

static E2BusPins bus{9, 8};

static void setScl(bool level, void* user) {
  auto* pins = static_cast<E2BusPins*>(user);
  digitalWrite(pins->scl, level ? HIGH : LOW);
}

static void setSda(bool level, void* user) {
  auto* pins = static_cast<E2BusPins*>(user);
  digitalWrite(pins->sda, level ? HIGH : LOW);
}

static bool readScl(void* user) {
  auto* pins = static_cast<E2BusPins*>(user);
  return digitalRead(pins->scl) != 0;
}

static bool readSda(void* user) {
  auto* pins = static_cast<E2BusPins*>(user);
  return digitalRead(pins->sda) != 0;
}

static void delayUs(uint32_t us, void* user) {
  (void)user;
  delayMicroseconds(us);
}

EE871::EE871 sensor;

void setup() {
  Serial.begin(115200);
  pinMode(bus.scl, OUTPUT_OPEN_DRAIN);
  pinMode(bus.sda, OUTPUT_OPEN_DRAIN);
  digitalWrite(bus.scl, HIGH);
  digitalWrite(bus.sda, HIGH);

  EE871::Config cfg;
  cfg.setScl = setScl;
  cfg.setSda = setSda;
  cfg.readScl = readScl;
  cfg.readSda = readSda;
  cfg.delayUs = delayUs;
  cfg.busUser = &bus;
  cfg.deviceAddress = 0;

  auto status = sensor.begin(cfg);
  if (!status.ok()) {
    Serial.printf("Init failed: %s\n", status.msg);
    return;
  }

  Serial.println("Device initialized");
}

void loop() {
  sensor.tick(millis());

  uint16_t ppm = 0;
  if (sensor.readCo2Average(ppm).ok()) {
    Serial.printf("CO2: %u ppm\n", ppm);
  }

  delay(1000);
}
```

## Health Monitoring

```cpp
if (sensor.state() == EE871::DriverState::OFFLINE) {
  sensor.recover();
}

Serial.printf("Failures: %u consecutive, %lu total\n",
              sensor.consecutiveFailures(),
              static_cast<unsigned long>(sensor.totalFailures()));
```

Validation and precondition errors return before E2 traffic and do not update health counters. `probe()` uses raw E2 reads and is diagnostic-only; normal reads/writes use tracked wrappers. `IN_PROGRESS` is treated as neutral for health if future scheduled operations use it.

## Timing And Blocking

The driver is managed synchronous: E2 transactions block for bounded protocol time, and `tick(nowMs)` only records the latest application timestamp for diagnostics. Clock stretching is bounded by `bitTimeoutUs` and `byteTimeoutUs`; flash writes are bounded by `writeDelayMs` or `intervalWriteDelayMs` with max 5000 ms validation.

The library never owns GPIO pins or an I2C/Wire instance. Applications provide `setScl`, `setSda`, `readScl`, `readSda`, and `delayUs` callbacks.

## Main API

- Lifecycle: `begin`, `tick`, `end`
- Diagnostics: `probe`, `recover`, `busReset`, `checkBusIdle`
- Identification: `readGroup`, `readSubgroup`, `readFirmwareVersion`, `readE2SpecVersion`
- Measurements: `readStatus`, `readCo2Fast`, `readCo2Average`, `readErrorCode`
- Custom memory/config: `customRead`, `customWrite`, `writeMeasurementInterval`, bus address, filter, operating mode, auto-adjust, calibration helpers

## Examples

- `examples/01_basic_bringup_cli/` - Interactive CLI for testing

## Documentation

- `CHANGELOG.md` - full release history
- `docs/IDF_PORT.md` - ESP-IDF portability guidance
- `docs/DOXYGEN.md` - how to build and browse API docs

## License

MIT License. See [LICENSE](LICENSE).
