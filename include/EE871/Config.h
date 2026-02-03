/// @file Config.h
/// @brief Configuration structure for EE871 driver
#pragma once

#include <cstdint>
#include "EE871/Status.h"

namespace ee871 {

/// E2 set-line callback signature (open-drain).
/// @param level 1 = release line (HIGH), 0 = drive LOW
/// @param user User context pointer passed through from Config
using E2SetLineFn = void (*)(bool level, void* user);

/// E2 read-line callback signature.
/// @param user User context pointer passed through from Config
/// @return true if line is HIGH
using E2ReadLineFn = bool (*)(void* user);

/// E2 delay callback signature.
/// @param us Microseconds to delay
/// @param user User context pointer passed through from Config
using E2DelayUsFn = void (*)(uint32_t us, void* user);

/// Configuration for EE871 driver
struct Config {
  // === E2 Transport (required) ===
  E2SetLineFn setScl = nullptr;   ///< Set/release clock line
  E2SetLineFn setSda = nullptr;   ///< Set/release data line
  E2ReadLineFn readScl = nullptr; ///< Read clock line
  E2ReadLineFn readSda = nullptr; ///< Read data line
  E2DelayUsFn delayUs = nullptr;  ///< Delay for bit timing
  void* busUser = nullptr;        ///< User context for callbacks

  // === Device Settings ===
  uint8_t deviceAddress = 0;      ///< E2 device address (0-7)

  // === Timing (E2 spec) ===
  uint16_t clockLowUs = 100;      ///< Minimum CLK low time
  uint16_t clockHighUs = 100;     ///< Minimum CLK high time
  uint16_t startHoldUs = 100;     ///< START hold time (spec min 4us, use clockHighUs for margin)
  uint16_t stopHoldUs = 100;      ///< STOP hold time (spec min 4us, use clockHighUs for margin)

  uint32_t bitTimeoutUs = 25000;  ///< Clock-stretch timeout per bit
  uint32_t byteTimeoutUs = 35000; ///< Clock-stretch timeout per byte

  uint32_t writeDelayMs = 150;    ///< Flash write delay for 0x10/0x50
  uint32_t intervalWriteDelayMs = 300; ///< Flash delay for 0xC6/0xC7 pair

  // === Health Tracking ===
  uint8_t offlineThreshold = 5;   ///< Consecutive failures before OFFLINE state
};

} // namespace ee871
