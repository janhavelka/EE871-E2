/// @file Config.h
/// @brief Configuration structure for EE871 driver
#pragma once

#include <cstdint>
#include "EE871/Status.h"

namespace EE871 {

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

/// @brief Configuration for EE871 driver.
///
/// The transport callbacks implement GPIO-style open-drain E2 line control:
/// setting a line high releases it, and setting a line low drives it low.
/// Callbacks must be bounded and deterministic. They must not call back into
/// public methods on the same EE871 driver instance.
struct Config {
  // === E2 Transport (required) ===
  E2SetLineFn setScl = nullptr;   ///< Set/release clock line
  E2SetLineFn setSda = nullptr;   ///< Set/release data line
  E2ReadLineFn readScl = nullptr; ///< Read clock line
  E2ReadLineFn readSda = nullptr; ///< Read data line
  E2DelayUsFn delayUs = nullptr;  ///< Delay for bit timing
  void* busUser = nullptr;        ///< User context for callbacks

  // === Device Settings ===
  uint8_t deviceAddress = 0;      ///< E2 protocol device address (0-7), not a hardware I2C address.

  // === Timing (E2 spec) ===
  uint16_t clockLowUs = 100;      ///< Minimum CLK low time, must be >= 100 us.
  uint16_t clockHighUs = 100;     ///< Minimum CLK high time, must be >= 100 us.
  uint16_t startHoldUs = 100;     ///< START hold time, must be >= 4 us.
  uint16_t stopHoldUs = 100;      ///< STOP hold time, must be >= 4 us.

  uint32_t bitTimeoutUs = 25000;  ///< Clock-stretch timeout per bit, must be > 0.
  uint32_t byteTimeoutUs = 35000; ///< Clock-stretch timeout per byte, must be >= bitTimeoutUs.

  uint32_t writeDelayMs = 150;    ///< Flash write delay for 0x10/0x50, max 5000 ms.
  uint32_t intervalWriteDelayMs = 300; ///< Flash delay for 0xC6/0xC7 pair, max 5000 ms.

  // === Health Tracking ===
  uint8_t offlineThreshold = 5;   ///< Consecutive failures before OFFLINE; zero normalizes to 1 in begin().
};

} // namespace EE871
