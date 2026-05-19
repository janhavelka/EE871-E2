#include <cstdint>

#include "E2GpioTransport.h"
#include "EE871/EE871.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

static constexpr char TAG[] = "ee871_basic";
static constexpr gpio_num_t E2_SCL = GPIO_NUM_9;
static constexpr gpio_num_t E2_SDA = GPIO_NUM_8;
static constexpr uint8_t EE871_ADDRESS = 0;
static constexpr TickType_t SAMPLE_PERIOD_TICKS = pdMS_TO_TICKS(5000);

uint32_t nowMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

void logStatus(const char* op, const EE871::Status& st) {
  ESP_LOGE(TAG, "%s failed: code=%u detail=%ld msg=%s", op,
           static_cast<unsigned>(st.code), static_cast<long>(st.detail), st.msg);
}

}  // namespace

extern "C" void app_main(void) {
  ee871_idf::E2GpioBus bus;
  ESP_ERROR_CHECK(ee871_idf::init(bus, E2_SCL, E2_SDA, false));

  EE871::EE871 sensor;
  EE871::Config cfg = ee871_idf::makeConfig(bus, EE871_ADDRESS);
  EE871::Status st = sensor.begin(cfg);
  if (!st.ok()) {
    logStatus("begin", st);
    return;
  }

  sensor.tick(nowMs());

  uint16_t group = 0;
  st = sensor.readGroup(group);
  if (st.ok()) {
    ESP_LOGI(TAG, "Group=0x%04X", static_cast<unsigned>(group));
  } else {
    logStatus("readGroup", st);
  }

  uint8_t subgroup = 0;
  st = sensor.readSubgroup(subgroup);
  if (st.ok()) {
    ESP_LOGI(TAG, "Subgroup=0x%02X", static_cast<unsigned>(subgroup));
  } else {
    logStatus("readSubgroup", st);
  }

  while (true) {
    sensor.tick(nowMs());

    uint8_t statusByte = 0;
    st = sensor.readStatus(statusByte);
    if (st.ok()) {
      ESP_LOGI(TAG, "Status=0x%02X CO2-error=%s", static_cast<unsigned>(statusByte),
               EE871::EE871::hasCo2Error(statusByte) ? "yes" : "no");
    } else {
      logStatus("readStatus", st);
    }

    uint16_t ppm = 0;
    st = sensor.readCo2Average(ppm);
    if (st.ok()) {
      ESP_LOGI(TAG, "CO2 average=%u ppm", static_cast<unsigned>(ppm));
    } else {
      logStatus("readCo2Average", st);
    }

    vTaskDelay(SAMPLE_PERIOD_TICKS);
  }
}
