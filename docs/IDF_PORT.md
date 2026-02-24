# EE871-E2 — ESP-IDF Migration Prompt

> **Library**: EE871-E2 (CO2 sensor, E2 bus protocol)
> **Current version**: 0.2.0 → **Target**: 2.0.0
> **Namespace**: `ee871` (lowercase)
> **Include path**: `#include "EE871/EE871.h"`
> **Difficulty**: Trivial — already pure C++, packaging only

---

## Pre-Migration

```bash
git tag v0.2.0   # freeze Arduino-era version
```

---

## Current State

Zero Arduino dependencies. All hardware interaction via injected callbacks:

```cpp
using E2SetLineFn  = void (*)(bool level, void* user);
using E2ReadLineFn = bool (*)(void* user);
using E2DelayUsFn  = void (*)(uint32_t us, void* user);
```

Time fully injected via `tick(uint32_t nowMs)`. Nothing to change in library code.

---

## Steps

### 1. Add `CMakeLists.txt` (library root)

```cmake
idf_component_register(
    SRCS "src/EE871.cpp"
    INCLUDE_DIRS "include"
)
```

### 2. Add `idf_component.yml` (library root)

```yaml
version: "2.0.0"
description: "EE871 CO2 sensor driver — E2 bus protocol"
targets:
  - esp32s2
  - esp32s3
dependencies:
  idf: ">=5.0"
```

### 3. Version bump

- `library.json` → `2.0.0`
- `Version.h` (if present) → `2.0.0`

### 4. Remove Arduino example

Delete or replace any `examples/` content that uses Arduino `setup()`/`loop()` with the ESP-IDF example below.

### 5. Add ESP-IDF example

Create `examples/espidf_basic/main/main.cpp`:

```cpp
#include <cstdio>
#include "EE871/EE871.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static constexpr gpio_num_t E2_PIN = GPIO_NUM_5;

static void setLine(bool level, void* /*user*/) {
    if (level) {
        gpio_set_direction(E2_PIN, GPIO_MODE_INPUT);
    } else {
        gpio_set_direction(E2_PIN, GPIO_MODE_OUTPUT);
        gpio_set_level(E2_PIN, 0);
    }
}

static bool readLine(void* /*user*/) {
    return gpio_get_level(E2_PIN) != 0;
}

static void delayUs(uint32_t us, void* /*user*/) {
    esp_rom_delay_us(us);
}

extern "C" void app_main() {
    gpio_config_t io{};
    io.pin_bit_mask = 1ULL << E2_PIN;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    ee871::Config cfg{};
    cfg.setLine = setLine;
    cfg.readLine = readLine;
    cfg.delayUs = delayUs;
    cfg.user = nullptr;

    ee871::Sensor sensor;
    ee871::Status st = sensor.begin(cfg);
    if (st.err != ee871::Err::Ok) {
        printf("begin() failed: %s\n", st.msg);
    }

    while (true) {
        uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000);
        sensor.tick(nowMs);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

Create `examples/espidf_basic/main/CMakeLists.txt`:

```cmake
idf_component_register(SRCS "main.cpp" INCLUDE_DIRS "." REQUIRES driver esp_timer)
```

Create `examples/espidf_basic/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS "../..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ee871_espidf_basic)
```

---

## Verification

```bash
cd examples/espidf_basic && idf.py set-target esp32s2 && idf.py build
```

- [ ] `idf.py build` succeeds
- [ ] Zero `#include <Arduino.h>` anywhere in library
- [ ] Version bumped to 2.0.0
- [ ] `git tag v2.0.0`
