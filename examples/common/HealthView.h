#pragma once

#include <Arduino.h>

#include "EE871/EE871.h"

inline void printHealthView(const EE871::EE871& driver) {
  Serial.printf("state=%d online=%s failures=%u totalFail=%lu totalOk=%lu\n",
                static_cast<int>(driver.state()), driver.isOnline() ? "true" : "false",
                static_cast<unsigned>(driver.consecutiveFailures()),
                static_cast<unsigned long>(driver.totalFailures()),
                static_cast<unsigned long>(driver.totalSuccess()));
}
