from __future__ import annotations

import pathlib
import shutil
import subprocess
import tempfile
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[1]


class TransportWaitTests(unittest.TestCase):
    def test_framework_adapters_do_not_underwait_positive_slices(self) -> None:
        # Compile the shipped headers with recording framework shims. This
        # checks requested scheduler delays, not measured device latency.
        compiler = shutil.which("g++")
        self.assertIsNotNone(compiler, "Native adapter regression requires g++")
        stubs = {
            "Arduino.h": r'''
#pragma once
#include <cstdint>
static constexpr int HIGH = 1, LOW = 0, OUTPUT_OPEN_DRAIN = 1;
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 1; }
extern uint32_t microsecondCalls, arduinoYields;
inline void delayMicroseconds(uint32_t) { ++microsecondCalls; }
inline void yield() { ++arduinoYields; }
''',
            "driver/gpio.h": r'''
#pragma once
#include <cstdint>
#include "esp_err.h"
using gpio_num_t = int;
static constexpr int GPIO_NUM_NC = -1, GPIO_MODE_INPUT_OUTPUT_OD = 1;
static constexpr int GPIO_PULLUP_ENABLE = 1, GPIO_PULLUP_DISABLE = 0;
static constexpr int GPIO_PULLDOWN_DISABLE = 0, GPIO_INTR_DISABLE = 0;
struct gpio_config_t {
  uint64_t pin_bit_mask;
  int mode, pull_up_en, pull_down_en, intr_type;
};
inline esp_err_t gpio_config(const gpio_config_t*) { return ESP_OK; }
inline esp_err_t gpio_set_level(gpio_num_t, int) { return ESP_OK; }
inline int gpio_get_level(gpio_num_t) { return 1; }
''',
            "esp_err.h": r'''
#pragma once
using esp_err_t = int;
static constexpr esp_err_t ESP_OK = 0, ESP_ERR_INVALID_ARG = 1;
''',
            "esp_rom_sys.h": r'''
#pragma once
#include <cstdint>
extern uint32_t microsecondCalls;
inline void esp_rom_delay_us(uint32_t) { ++microsecondCalls; }
''',
            "freertos/FreeRTOS.h": r'''
#pragma once
#include <cstdint>
using TickType_t = uint32_t;
#define portTICK_PERIOD_MS (1000U / configTICK_RATE_HZ)
''',
            "freertos/task.h": r'''
#pragma once
#include <cassert>
#include "freertos/FreeRTOS.h"
extern TickType_t delayTicks[2];
extern uint32_t delayCalls, idfYields;
inline void vTaskDelay(TickType_t ticks) {
  assert(delayCalls < 2);
  delayTicks[delayCalls++] = ticks;
}
#define taskYIELD() (++idfYields)
''',
        }
        source = r'''
#include "freertos/FreeRTOS.h"
#include "common/E2Transport.h"
#include "idf/common/E2GpioTransport.h"
#include <cassert>
uint32_t microsecondCalls = 0, arduinoYields = 0;
TickType_t delayTicks[2] = {};
uint32_t delayCalls = 0, idfYields = 0;
int main() {
  const uint32_t waits[] = {0, 1, 9, 10, 11, 49, 50};
  for (uint32_t ms : waits) {
    delayCalls = 0;
    transport::delayMs(ms, nullptr);
    ee871_idf::delayMs(ms, nullptr);
    if (ms == 0) {
      assert(delayCalls == 0);
      continue;
    }
    // vTaskDelay(n) can return just after n-1 full tick periods. Both
    // adapters must cover the requested duration even at that phase.
    assert(delayCalls == 2);
    for (uint32_t ticks : delayTicks) {
      assert(ticks > 0);
      assert((ticks - 1ULL) * 1000 >= ms * configTICK_RATE_HZ);
      assert(ticks * 1000ULL <= ms * configTICK_RATE_HZ + 2000);
    }
  }
  assert(microsecondCalls == 0 && arduinoYields == 0 && idfYields == 0);
  transport::yieldTask(nullptr);
  ee871_idf::yieldTask(nullptr);
  assert(arduinoYields == 1 && idfYields == 1);
  ee871_idf::E2GpioBus bus;
  const EE871::Config cfg = ee871_idf::makeConfig(bus, 3);
  assert(cfg.delayMs == ee871_idf::delayMs && cfg.yield == ee871_idf::yieldTask);
  assert(cfg.delayUs == ee871_idf::delayUs && cfg.busUser == &bus);
  assert(cfg.deviceAddress == 3);
}
'''
        with tempfile.TemporaryDirectory(prefix="ee871-wait-adapters-") as directory:
            root = pathlib.Path(directory)
            for relative, content in stubs.items():
                path = root / relative
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content, encoding="utf-8")
            cpp = root / "adapter.cpp"
            cpp.write_text(source, encoding="utf-8")
            for tick_rate in (100, 128, 1000):
                with self.subTest(tick_rate=tick_rate):
                    executable = root / f"adapter-{tick_rate}.exe"
                    build = subprocess.run(
                        [compiler, "-std=c++17", f"-DconfigTICK_RATE_HZ={tick_rate}",
                         "-I", str(root), "-I", str(ROOT / "include"),
                         "-I", str(ROOT / "examples"), str(cpp), "-o", str(executable)],
                        capture_output=True, text=True, timeout=60,
                    )
                    self.assertEqual(0, build.returncode, build.stdout + build.stderr)
                    run = subprocess.run([str(executable)], capture_output=True, text=True, timeout=10)
                    self.assertEqual(0, run.returncode, run.stdout + run.stderr)


if __name__ == "__main__":
    unittest.main()
