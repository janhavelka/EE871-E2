/// @file E2GpioTransport.h
/// @brief ESP-IDF GPIO adapter for the EE871 E2 callback contract.
#pragma once

#include <cstdint>

#include "EE871/EE871.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_rom_sys.h"

namespace ee871_idf {

struct E2GpioBus {
  gpio_num_t scl = GPIO_NUM_NC;
  gpio_num_t sda = GPIO_NUM_NC;
};

inline esp_err_t init(E2GpioBus& bus, gpio_num_t scl, gpio_num_t sda,
                      bool enableInternalPullups = false) {
  if (scl == GPIO_NUM_NC || sda == GPIO_NUM_NC || scl == sda) {
    return ESP_ERR_INVALID_ARG;
  }

  bus.scl = scl;
  bus.sda = sda;

  gpio_config_t cfg = {};
  cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(scl)) |
                     (1ULL << static_cast<uint32_t>(sda));
  cfg.mode = GPIO_MODE_OUTPUT_OD;
  cfg.pull_up_en = enableInternalPullups ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
  cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  cfg.intr_type = GPIO_INTR_DISABLE;

  esp_err_t err = gpio_config(&cfg);
  if (err != ESP_OK) {
    return err;
  }
  err = gpio_set_level(scl, 1);
  if (err != ESP_OK) {
    return err;
  }
  return gpio_set_level(sda, 1);
}

inline void setScl(bool level, void* user) {
  auto* bus = static_cast<E2GpioBus*>(user);
  if (bus != nullptr && bus->scl != GPIO_NUM_NC) {
    (void)gpio_set_level(bus->scl, level ? 1 : 0);
  }
}

inline void setSda(bool level, void* user) {
  auto* bus = static_cast<E2GpioBus*>(user);
  if (bus != nullptr && bus->sda != GPIO_NUM_NC) {
    (void)gpio_set_level(bus->sda, level ? 1 : 0);
  }
}

inline bool readScl(void* user) {
  auto* bus = static_cast<E2GpioBus*>(user);
  return (bus != nullptr && bus->scl != GPIO_NUM_NC) ? (gpio_get_level(bus->scl) != 0)
                                                     : false;
}

inline bool readSda(void* user) {
  auto* bus = static_cast<E2GpioBus*>(user);
  return (bus != nullptr && bus->sda != GPIO_NUM_NC) ? (gpio_get_level(bus->sda) != 0)
                                                     : false;
}

inline void delayUs(uint32_t us, void*) {
  esp_rom_delay_us(us);
}

inline EE871::Config makeConfig(E2GpioBus& bus, uint8_t deviceAddress = 0) {
  EE871::Config cfg;
  cfg.setScl = setScl;
  cfg.setSda = setSda;
  cfg.readScl = readScl;
  cfg.readSda = readSda;
  cfg.delayUs = delayUs;
  cfg.busUser = &bus;
  cfg.deviceAddress = deviceAddress;
  return cfg;
}

}  // namespace ee871_idf
