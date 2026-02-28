# EE871 E2 Driver Library

Production-grade EE871 CO2 sensor driver for the E2 bus on ESP32 (Arduino/PlatformIO).

## Features

- **E2 bus HAL injection** - no Wire/I2C dependency in library code
- **Health monitoring** - READY/DEGRADED/OFFLINE tracking
- **Deterministic behavior** - bounded loops, explicit timeouts
- **Managed synchronous** - blocking transfers with spec-compliant limits

## Installation

### PlatformIO (recommended)

Add to `platformio.ini`:

```ini
lib_deps =
  https://github.com/your-username/ee871-e2.git
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

## Examples

- `examples/01_basic_bringup_cli/` - Interactive CLI for testing

## License

MIT License. See [LICENSE](LICENSE).

