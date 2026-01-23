/// @file E2Transport.h
/// @brief Bit-banged E2 transport adapter for examples
/// @note NOT part of the library - examples only
#pragma once

#include <Arduino.h>

namespace transport {

struct E2Pins {
  int scl;
  int sda;
};

/// Initialize pins for open-drain E2 bus use.
inline bool initE2(E2Pins& pins, int sclPin, int sdaPin) {
  pins.scl = sclPin;
  pins.sda = sdaPin;

  pinMode(pins.scl, OUTPUT_OPEN_DRAIN);
  pinMode(pins.sda, OUTPUT_OPEN_DRAIN);

  digitalWrite(pins.scl, HIGH);
  digitalWrite(pins.sda, HIGH);

  return true;
}

inline void setScl(bool level, void* user) {
  auto* pins = static_cast<E2Pins*>(user);
  digitalWrite(pins->scl, level ? HIGH : LOW);
}

inline void setSda(bool level, void* user) {
  auto* pins = static_cast<E2Pins*>(user);
  digitalWrite(pins->sda, level ? HIGH : LOW);
}

inline bool readScl(void* user) {
  auto* pins = static_cast<E2Pins*>(user);
  return digitalRead(pins->scl) != 0;
}

inline bool readSda(void* user) {
  auto* pins = static_cast<E2Pins*>(user);
  return digitalRead(pins->sda) != 0;
}

inline void delayUs(uint32_t us, void* user) {
  (void)user;
  delayMicroseconds(us);
}

} // namespace transport
