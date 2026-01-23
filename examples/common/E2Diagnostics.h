/// @file E2Diagnostics.h
/// @brief Lightweight helpers for E2 status display (examples only)
/// @note NOT part of the library
#pragma once

#include <Arduino.h>
#include "EE871/CommandTable.h"
#include "Log.h"

namespace e2diag {

inline void printStatus(uint8_t status) {
  Serial.printf("Status: 0x%02X", status);
  if (status & ee871::cmd::STATUS_CO2_ERROR_MASK) {
    Serial.print(" (CO2 error)");
  }
  Serial.println();
}

} // namespace e2diag
