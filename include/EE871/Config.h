/// @file Config.h
/// @brief Configuration structure for EE871 driver
#pragma once

#include <cstdint>
#include "EE871/Status.h"

namespace EE871 {

/// @brief E2 open-drain set-line callback signature.
///
/// A high level means "release the line" so the external pull-up can drive it
/// high; a low level means actively pull the line low. The callback belongs to
/// the application or example adapter, not to the core driver. It must be
/// deterministic, bounded, and must not call back into public methods on the
/// same EE871 instance.
/// @param level 1 = release line (HIGH), 0 = drive LOW.
/// @param user User context pointer passed through from Config.
using E2SetLineFn = void (*)(bool level, void* user);

/// @brief E2 read-line callback signature.
///
/// Used for clock-stretch detection and data sampling. The callback must read
/// the physical open-drain line level and return quickly.
/// @param user User context pointer passed through from Config.
/// @return true if the line is HIGH/released, false if it is LOW.
using E2ReadLineFn = bool (*)(void* user);

/// @brief E2 microsecond delay callback signature.
///
/// The callback must honor microsecond delays as closely as the platform
/// reasonably allows. Public bus operations are synchronous and can call this
/// callback repeatedly for E2 bit timing and bounded write-delay waits.
/// @param us Microseconds to delay.
/// @param user User context pointer passed through from Config.
using E2DelayUsFn = void (*)(uint32_t us, void* user);

/// Optional bounded application veto for an otherwise safe read retry.
/// Called with busUser before and after the retry pause, and once more after
/// the final idle-line check, before another frame. Return false after
/// deadline expiry, cancellation or a latched callback error. No recursive
/// driver calls or bus I/O are allowed from this callback.
using E2AllowReadRetryFn = bool (*)(void* user);

/// @brief Configuration for EE871 driver.
///
/// The transport callbacks implement GPIO-style open-drain E2 line control.
/// EE871-E2 is not Arduino Wire, ESP-IDF `driver/i2c_master`, or any hardware
/// I2C peripheral. The driver does not own GPIO pins, bus objects, tasks, locks,
/// framework handles, or pull-up configuration.
///
/// Applications must keep callback state alive for the lifetime of the driver
/// session and externally serialize calls when multiple tasks or bus users share
/// an EE871 instance or the same physical E2 lines. Public bus operations are
/// blocking and are not ISR-safe.
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
  uint16_t clockLowUs = 100;      ///< >= 100 us; high + low + 10 us must be <= 2000 us.
  uint16_t clockHighUs = 100;     ///< >= 100 us; high + low + 10 us must be <= 2000 us.
  uint16_t startHoldUs = 100;     ///< START hold time, must be >= 4 us.
  uint16_t stopHoldUs = 100;      ///< STOP hold time, must be >= 4 us.

  uint32_t bitTimeoutUs = 25000;  ///< Per-bit stretch timeout: 1..25000 us.
  uint32_t byteTimeoutUs = 35000; ///< Byte deadline: <=35000 us, > nominal time, >= bit timeout.

  uint32_t writeDelayMs = 150;    ///< Flash write delay after a 0x10 custom write, max 5000 ms.
  uint32_t intervalWriteDelayMs = 300; ///< Flash delay for 0xC6/0xC7 pair, max 5000 ms.

  // === Health Tracking ===
  uint8_t offlineThreshold = 5;   ///< Consecutive tracked failures before OFFLINE; zero normalizes to 1.

  /// Per-SCL-release deadline for flash-write STOP and bus reset: 300000..5000000 us.
  /// AN1611-1 documents an EE871 deviation from generic E2's 25/35 ms transfer
  /// limits: flash commits can extend CLK low for 150 ms, or 300 ms for the
  /// interval pair. This separate budget includes 50 ms margin by default;
  /// Ordinary read and volatile pointer-write STOPs use bitTimeoutUs instead.
  /// This does not relax bitTimeoutUs/byteTimeoutUs or replace write delays.
  /// Appended to preserve existing positional aggregate initialization.
  uint32_t flashStretchTimeoutUs = 350000;

  /// Additional control-NACK attempts for MV3/MV4/status only: 0..3, default off.
  /// Each retry requires successful STOP/idle and a fixed 1000 us HAL pause.
  /// Appended fields preserve existing positional Config initializers.
  uint8_t readNackRetries = 0;
  E2AllowReadRetryFn allowReadRetry = nullptr; ///< Null permits eligible retries.
};

} // namespace EE871
