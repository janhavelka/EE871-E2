/**
 * @file BoardConfig.h
 * @brief Example board configuration for ESP32-S2 / ESP32-S3 reference hardware.
 *
 * These are convenience defaults for reference designs only.
 * NOT part of the library API. Override for your hardware.
 *
 * @warning The library itself is board-agnostic. All pins are passed via Config.
 *          These defaults are provided for examples only.
 */

#pragma once

#include <stdint.h>

#include "examples/common/E2Transport.h"

namespace board {

// ====================================================================
// EXAMPLE DEFAULTS - ESP32-S2 / ESP32-S3 REFERENCE HARDWARE
// ====================================================================
// These values are NOT library defaults. They are example-only values.
// Override them for your board by creating your own BoardConfig.h or
// passing explicit values to Config structs in your application.
// ====================================================================

/// @brief E2 DATA pin. Example default for ESP32-S2/S3.
static constexpr int E2_DATA = 7;

/// @brief E2 CLOCK pin. Example default for ESP32-S2/S3.
static constexpr int E2_CLOCK = 6;

/// @brief E2 timing defaults (spec: >=100 us high/low).
static constexpr uint16_t E2_CLOCK_LOW_US = 100;
static constexpr uint16_t E2_CLOCK_HIGH_US = 100;
static constexpr uint32_t E2_BIT_TIMEOUT_US = 25000;
static constexpr uint32_t E2_BYTE_TIMEOUT_US = 35000;
static constexpr uint32_t E2_WRITE_DELAY_MS = 150;
static constexpr uint32_t E2_INTERVAL_WRITE_DELAY_MS = 300;

/// @brief LED pin. Example default for ESP32-S3 (RGB LED on GPIO48).
/// Set to -1 to disable.
static constexpr int LED = 10;

/// @brief Access to E2 pin storage.
inline transport::E2Pins& e2Pins() {
  static transport::E2Pins pins{E2_CLOCK, E2_DATA};
  return pins;
}

/// @brief Initialize E2 pins for examples using the default config.
inline bool initE2() {
  return transport::initE2(e2Pins(), E2_CLOCK, E2_DATA);
}

}  // namespace board
