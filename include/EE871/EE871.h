/// @file EE871.h
/// @brief Main driver class for EE871 CO2 sensor (E2 bus)
#pragma once

#include <cstddef>
#include <cstdint>

#include "EE871/CommandTable.h"
#include "EE871/Config.h"
#include "EE871/Status.h"
#include "EE871/Version.h"

namespace ee871 {

/// Driver state for health monitoring
enum class DriverState : uint8_t {
  UNINIT,    ///< begin() not called or end() called
  READY,     ///< Operational, consecutiveFailures == 0
  DEGRADED,  ///< 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    ///< consecutiveFailures >= offlineThreshold
};

/// EE871 driver class
class EE871 {
public:
  // =========================================================================
  // Lifecycle
  // =========================================================================

  /// Initialize the driver with configuration
  /// @param config Configuration including E2 transport callbacks
  /// @return Status::Ok() on success, error otherwise
  Status begin(const Config& config);

  /// Process periodic tasks (call regularly from loop)
  /// @param nowMs Current timestamp in milliseconds
  void tick(uint32_t nowMs);

  /// Shutdown the driver and release resources
  void end();

  // =========================================================================
  // Diagnostics
  // =========================================================================

  /// Check if device is present on the bus (no health tracking)
  /// @return Status::Ok() if device responds, error otherwise
  Status probe();

  /// Attempt to recover from DEGRADED/OFFLINE state
  /// @return Status::Ok() if device now responsive, error otherwise
  Status recover();

  // =========================================================================
  // Driver State
  // =========================================================================

  /// Get current driver state
  DriverState state() const { return _driverState; }

  /// Check if driver is ready for operations
  bool isOnline() const {
    return _driverState == DriverState::READY ||
           _driverState == DriverState::DEGRADED;
  }

  // =========================================================================
  // Health Tracking
  // =========================================================================

  /// Timestamp of last successful E2 operation
  uint32_t lastOkMs() const { return _lastOkMs; }

  /// Timestamp of last failed E2 operation
  uint32_t lastErrorMs() const { return _lastErrorMs; }

  /// Most recent error status
  Status lastError() const { return _lastError; }

  /// Consecutive failures since last success
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }

  /// Total failure count (lifetime)
  uint32_t totalFailures() const { return _totalFailures; }

  /// Total success count (lifetime)
  uint32_t totalSuccess() const { return _totalSuccess; }

  // =========================================================================
  // E2 Protocol Helpers
  // =========================================================================

  /// Read a control-byte addressed value (main command nibble)
  Status readControlByte(uint8_t mainCommandNibble, uint8_t& data);

  /// Read a 16-bit value using low/high control bytes
  Status readU16(uint8_t mainCommandLow, uint8_t mainCommandHigh, uint16_t& value);

  /// Set internal custom pointer (0x50)
  Status setCustomPointer(uint16_t address);

  /// Read custom memory byte at address
  Status customRead(uint8_t address, uint8_t& data);

  /// Read custom memory block (auto-increment)
  Status customRead(uint8_t address, uint8_t* buf, size_t len);

  /// Write custom memory byte at address (0x10) and verify
  Status customWrite(uint8_t address, uint8_t value);

  /// Write global measurement interval (0xC6/0xC7) and verify
  /// @param intervalDeciSeconds Interval in 0.1 s units
  Status writeMeasurementInterval(uint16_t intervalDeciSeconds);

  // =========================================================================
  // EE871 Helpers
  // =========================================================================

  // =========================================================================
  // Identification
  // =========================================================================

  Status readGroup(uint16_t& group);
  Status readSubgroup(uint8_t& subgroup);
  Status readAvailableMeasurements(uint8_t& bits);

  // =========================================================================
  // Firmware / Spec Version
  // =========================================================================

  /// Read firmware version (main.sub)
  Status readFirmwareVersion(uint8_t& main, uint8_t& sub);

  /// Read E2 specification version implemented by device
  Status readE2SpecVersion(uint8_t& version);

  // =========================================================================
  // Feature Discovery
  // =========================================================================

  /// Read operating functions bitfield (0x07)
  /// @see cmd::FEATURE_* constants for bit meanings
  Status readOperatingFunctions(uint8_t& bits);

  /// Read operating mode support bitfield (0x08)
  /// @see cmd::MODE_SUPPORT_* constants
  Status readOperatingModeSupport(uint8_t& bits);

  /// Read special features bitfield (0x09)
  /// @see cmd::SPECIAL_FEATURE_* constants
  Status readSpecialFeatures(uint8_t& bits);

  // =========================================================================
  // Identity Strings
  // =========================================================================

  /// Read 16-byte serial number (0xA0-0xAF)
  /// @param buf Buffer of at least 16 bytes
  Status readSerialNumber(uint8_t* buf);

  /// Read 16-byte part name (0xB0-0xBF)
  /// @param buf Buffer of at least 16 bytes
  Status readPartName(uint8_t* buf);

  /// Write 16-byte part name (0xB0-0xBF)
  /// @param buf Buffer of exactly 16 bytes
  Status writePartName(const uint8_t* buf);

  // =========================================================================
  // Bus Address
  // =========================================================================

  /// Read current bus address (0xC0)
  Status readBusAddress(uint8_t& address);

  /// Write bus address (0xC0) - requires power cycle to take effect
  /// @param address New address (0-7)
  /// @return OUT_OF_RANGE if address > 7
  Status writeBusAddress(uint8_t address);

  // =========================================================================
  // Measurement Interval
  // =========================================================================

  /// Read global measurement interval
  /// @param intervalDeciSeconds Interval in 0.1 s units
  Status readMeasurementInterval(uint16_t& intervalDeciSeconds);

  /// Read CO2-specific interval factor (0xCB)
  /// Positive = multiplier, Negative = divider
  Status readCo2IntervalFactor(int8_t& factor);

  /// Write CO2-specific interval factor (0xCB)
  Status writeCo2IntervalFactor(int8_t factor);

  // =========================================================================
  // Filter / Operating Mode
  // =========================================================================

  /// Read CO2 filter setting (0xD3)
  Status readCo2Filter(uint8_t& filter);

  /// Write CO2 filter setting (0xD3)
  Status writeCo2Filter(uint8_t filter);

  /// Read operating mode (0xD8)
  /// @see cmd::OPERATING_MODE_* constants
  Status readOperatingMode(uint8_t& mode);

  /// Write operating mode (0xD8)
  /// bit0: 0=freerunning, 1=low power
  /// bit1: 0=measurement priority, 1=E2 priority
  Status writeOperatingMode(uint8_t mode);

  // =========================================================================
  // Auto Adjustment
  // =========================================================================

  /// Check if auto adjustment is running (0xD9 bit0)
  Status readAutoAdjustStatus(bool& running);

  /// Start auto adjustment (cannot be stopped once started)
  /// Device will return 0x55 during adjustment (~5 min)
  Status startAutoAdjust();

  // =========================================================================
  // Calibration (Advanced)
  // =========================================================================

  /// Read CO2 offset (signed, ppm)
  Status readCo2Offset(int16_t& offset);

  /// Write CO2 offset (signed, ppm)
  Status writeCo2Offset(int16_t offset);

  /// Read CO2 gain (gain = value / 32768)
  Status readCo2Gain(uint16_t& gain);

  /// Write CO2 gain (gain = value / 32768)
  Status writeCo2Gain(uint16_t gain);

  /// Read last calibration points (lower, upper in ppm)
  Status readCo2CalPoints(uint16_t& lower, uint16_t& upper);

  // =========================================================================
  // Status / Measurements
  // =========================================================================

  /// Read status byte (also triggers measurement if interval > 15s)
  Status readStatus(uint8_t& status);

  /// Read error code (0xC1) - valid when status bit3 is set
  Status readErrorCode(uint8_t& code);

  /// Read CO2 fast response (MV3, raw/unfiltered)
  Status readCo2Fast(uint16_t& ppm);

  /// Read CO2 averaged value (MV4, 11-sample moving average)
  Status readCo2Average(uint16_t& ppm);

  // =========================================================================
  // Bus Safety
  // =========================================================================

  /// Reset bus state by clocking with SDA high
  /// Use after timeout/stuck bus conditions
  /// @return Ok if bus lines are free after reset
  Status busReset();

  /// Check if bus lines are idle (both high)
  /// @return Ok if idle, BUS_STUCK if either line is low
  Status checkBusIdle();

private:
  // =========================================================================
  // Tracked/Raw Transport Wrappers
  // =========================================================================

  Status _readControlByteRaw(uint8_t controlByte, uint8_t& data);
  Status _readControlByteTracked(uint8_t controlByte, uint8_t& data);

  Status _writeCommandRaw(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte);
  Status _writeCommandTracked(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte);

  // =========================================================================
  // Health Management
  // =========================================================================

  /// Update health counters and state based on operation result
  /// Called ONLY from tracked transport wrappers
  Status _updateHealth(const Status& st);

  // =========================================================================
  // State
  // =========================================================================

  Config _config;
  bool _initialized = false;
  DriverState _driverState = DriverState::UNINIT;
  uint32_t _nowMs = 0;

  // Health counters
  uint32_t _lastOkMs = 0;
  uint32_t _lastErrorMs = 0;
  Status _lastError = Status::Ok();
  uint8_t _consecutiveFailures = 0;
  uint32_t _totalFailures = 0;
  uint32_t _totalSuccess = 0;
};

} // namespace ee871
