/// @file EE871.h
/// @brief Main driver class for EE871 CO2 sensor (E2 bus)
#pragma once

#include <cstddef>
#include <cstdint>

#include "EE871/CommandTable.h"
#include "EE871/Config.h"
#include "EE871/Status.h"
#include "EE871/Version.h"

namespace EE871 {

/// @brief Coarse driver health state.
enum class DriverState : uint8_t {
  UNINIT,    ///< begin() not called or end() called
  READY,     ///< Operational, consecutiveFailures == 0
  DEGRADED,  ///< 1 <= consecutiveFailures < offlineThreshold
  OFFLINE    ///< consecutiveFailures >= offlineThreshold
};

/// Snapshot of current configuration, cached feature flags, and driver health.
struct SettingsSnapshot {
  Config config;                  ///< Active normalized configuration.
  DriverState state = DriverState::UNINIT; ///< Current coarse health state.
  bool initialized = false;       ///< True after successful begin().
  uint32_t nowMs = 0;             ///< Last tick() timestamp seen by the driver.
  uint8_t operatingFunctions = 0; ///< Cached custom-memory 0x07 feature flags.
  uint8_t operatingModeSupport = 0; ///< Cached custom-memory 0x08 mode flags.
  uint8_t specialFeatures = 0;    ///< Cached custom-memory 0x09 feature flags.
  uint32_t lastOkMs = 0;          ///< Last tracked successful E2 operation.
  uint32_t lastErrorMs = 0;       ///< Last tracked failed E2 operation.
  Status lastError = Status::Ok(); ///< Last tracked error status.
  uint8_t consecutiveFailures = 0; ///< Current consecutive tracked failures.
  uint32_t totalFailures = 0;     ///< Total tracked failures.
  uint32_t totalSuccess = 0;      ///< Total tracked successes.
  bool persistentConfigDirty = false; ///< True when persistent config may be partially applied.
  Status persistentConfigDirtyError = Status::Ok(); ///< First error that marked persistent config dirty.
};

/// @brief Transport-agnostic EE871 CO2 sensor driver for the E2 bus.
///
/// EE871-E2 uses GPIO-style open-drain E2 signaling through injected line and
/// delay callbacks. It is not an Arduino Wire, ESP-IDF hardware I2C, or other
/// owned-bus driver.
///
/// Instances are not thread-safe. Use one owner task/context or externally
/// serialize all public calls, including state-only accessors and tick().
/// Shared users of the same GPIO/E2 bus must also serialize outside the
/// library. Public methods that touch the E2 bus are blocking and are not
/// ISR-safe because they may perform bus I/O and call the configured delay
/// callback. Transport callbacks must be bounded and deterministic, and must
/// not call public methods on the same EE871 instance recursively.
class EE871 {
public:
  EE871() = default;
  EE871(const EE871&) = delete;
  EE871& operator=(const EE871&) = delete;
  EE871(EE871&&) = delete;
  EE871& operator=(EE871&&) = delete;

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

  /// Shutdown the driver and release resources.
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

  /// Re-read persistent configuration and clear dirty diagnostics when coherent.
  /// @return Status::Ok() when persistent fields can be read and validated.
  Status resyncPersistentConfig();

  // =========================================================================
  // Driver State
  // =========================================================================

  /// Get current driver state.
  /// @return Current coarse health state.
  DriverState state() const { return _driverState; }

  /// Alias for state(), matching the shared driver-health naming.
  /// @return Current coarse health state.
  DriverState driverState() const { return _driverState; }

  /// Alias for driverState(), used by shared diagnostics.
  /// @return Current coarse health state.
  DriverState healthState() const { return _driverState; }

  /// Check whether begin() has completed successfully.
  /// @return true after successful begin() and before end().
  bool isInitialized() const { return _initialized; }

  /// Check if driver is ready for operations.
  /// @return true when the driver is READY or DEGRADED.
  bool isOnline() const {
    return _driverState == DriverState::READY ||
           _driverState == DriverState::DEGRADED;
  }

  /// Active normalized configuration.
  /// @return Current configuration copy stored by begin(), or defaults before begin().
  const Config& getConfig() const { return _config; }

  /// Copy current configuration, feature-cache, and health state.
  /// @param out Receives the current snapshot.
  /// @return Status::Ok(); snapshot access does not touch the E2 bus.
  Status getSettings(SettingsSnapshot& out) const;

  /// Return current configuration, feature-cache, and health state by value.
  /// @return Current settings snapshot.
  SettingsSnapshot getSettings() const;

  // =========================================================================
  // Health Tracking
  // =========================================================================

  /// Timestamp of last successful E2 operation.
  /// @return Millisecond timestamp supplied through tick().
  uint32_t lastOkMs() const { return _lastOkMs; }

  /// Timestamp of last failed E2 operation.
  /// @return Millisecond timestamp supplied through tick().
  uint32_t lastErrorMs() const { return _lastErrorMs; }

  /// Most recent error status.
  /// @return Last tracked failure status.
  Status lastError() const { return _lastError; }

  /// Consecutive failures since last success.
  /// @return Current consecutive tracked failure count.
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }

  /// Total failure count (lifetime).
  /// @return Lifetime tracked failure count.
  uint32_t totalFailures() const { return _totalFailures; }

  /// Total success count (lifetime).
  /// @return Lifetime tracked success count.
  uint32_t totalSuccess() const { return _totalSuccess; }

  /// Consecutive failures required before OFFLINE.
  /// @return Normalized threshold currently in use.
  uint8_t offlineThreshold() const { return _config.offlineThreshold; }

  /// Check if a multi-byte persistent write may have partially applied.
  /// @return true when persistent configuration needs explicit resync/inspection.
  bool persistentConfigDirty() const { return _persistentConfigDirty; }

  /// First error that marked persistent configuration dirty.
  /// @return Stored error status, or Status::Ok() when not dirty.
  Status persistentConfigDirtyError() const { return _persistentConfigDirtyError; }

  // =========================================================================
  // E2 Protocol Helpers
  // =========================================================================

  /// Read a control-byte addressed value.
  /// @param mainCommandNibble EE871-supported main-command nibble.
  /// @param[out] data Returned data byte.
  /// @return Status::Ok() on success; NOT_SUPPORTED for EE871-unsupported measurement commands.
  Status readControlByte(uint8_t mainCommandNibble, uint8_t& data);

  /// Read a 16-bit value using low/high control bytes.
  /// @param mainCommandLow Low-byte main-command nibble.
  /// @param mainCommandHigh High-byte main-command nibble.
  /// @param[out] value Little-endian assembled value.
  /// @return Status::Ok() when both byte reads succeed.
  Status readU16(uint8_t mainCommandLow, uint8_t mainCommandHigh, uint16_t& value);

  /// Set internal custom pointer using command 0x50.
  /// @param address Custom-memory address; only the low byte is sent for EE871.
  /// @return Status::Ok() after the pointer write and configured write delay.
  Status setCustomPointer(uint16_t address);

  /// Read one custom-memory byte.
  /// @param address Custom-memory address.
  /// @param[out] data Returned byte.
  /// @return Status::Ok() when pointer write and data read succeed.
  Status customRead(uint8_t address, uint8_t& data);

  /// Read a custom-memory block using pointer auto-increment.
  /// @param address First custom-memory address.
  /// @param[out] buf Destination buffer.
  /// @param len Number of bytes to read.
  /// @return Status::Ok() when all bytes are read.
  Status customRead(uint8_t address, uint8_t* buf, size_t len);

  /// Write one custom-memory byte with command 0x10 and verify by readback.
  /// @param address Custom-memory address.
  /// @param value Byte to write.
  /// @return Status::Ok() when the readback matches.
  Status customWrite(uint8_t address, uint8_t value);

  /// Write global measurement interval (0xC6/0xC7) and verify
  /// @param intervalDeciSeconds Interval in 0.1 s units
  /// @return Status::Ok() when both interval bytes verify. A failure after the
  /// first byte succeeds marks persistent configuration dirty.
  Status writeMeasurementInterval(uint16_t intervalDeciSeconds);

  // =========================================================================
  // EE871 Helpers
  // =========================================================================

  // =========================================================================
  // Identification
  // =========================================================================

  /// Read EE871 sensor group identifier.
  /// @param[out] group Expected value is cmd::SENSOR_GROUP_ID.
  /// @return Status::Ok() when both group bytes are read.
  Status readGroup(uint16_t& group);
  /// Read EE871 sensor subgroup identifier.
  /// @param[out] subgroup Expected value is cmd::SENSOR_SUBGROUP_ID.
  /// @return Status::Ok() when the subgroup byte is read.
  Status readSubgroup(uint8_t& subgroup);
  /// Read the available-measurements bitfield.
  /// @param[out] bits Measurement availability flags.
  /// @return Status::Ok() when the bitfield is read.
  Status readAvailableMeasurements(uint8_t& bits);

  // =========================================================================
  // Firmware / Spec Version
  // =========================================================================

  /// Read firmware version (main.sub)
  /// @param[out] main Firmware main version byte.
  /// @param[out] sub Firmware sub version byte.
  /// @return Status::Ok() when both bytes are read.
  Status readFirmwareVersion(uint8_t& main, uint8_t& sub);

  /// Read E2 specification version implemented by device
  /// @param[out] version E2 specification version byte.
  /// @return Status::Ok() when the byte is read.
  Status readE2SpecVersion(uint8_t& version);

  // =========================================================================
  // Feature Discovery
  // =========================================================================

  /// Read operating functions bitfield (0x07)
  /// @param[out] bits Feature flags from custom memory 0x07.
  /// @return Status::Ok() when the byte is read.
  /// @see cmd::FEATURE_* constants for bit meanings
  Status readOperatingFunctions(uint8_t& bits);

  /// Read operating mode support bitfield (0x08)
  /// @param[out] bits Operating-mode support flags from custom memory 0x08.
  /// @return Status::Ok() when the byte is read.
  /// @see cmd::MODE_SUPPORT_* constants
  Status readOperatingModeSupport(uint8_t& bits);

  /// Read special features bitfield (0x09)
  /// @param[out] bits Special-feature flags from custom memory 0x09.
  /// @return Status::Ok() when the byte is read.
  /// @see cmd::SPECIAL_FEATURE_* constants
  Status readSpecialFeatures(uint8_t& bits);

  // =========================================================================
  // Feature Support Queries (use cached values from begin())
  // =========================================================================

  /// Check if serial number is readable
  bool hasSerialNumber() const { return (_operatingFunctions & cmd::FEATURE_SERIAL_NUMBER) != 0; }

  /// Check if part name is readable/writable
  bool hasPartName() const { return (_operatingFunctions & cmd::FEATURE_PART_NAME) != 0; }

  /// Check if bus address is configurable
  bool hasAddressConfig() const { return (_operatingFunctions & cmd::FEATURE_ADDRESS_CONFIG) != 0; }

  /// Check if global measurement interval is configurable
  bool hasGlobalInterval() const { return (_operatingFunctions & cmd::FEATURE_GLOBAL_INTERVAL) != 0; }

  /// Check if specific (per-quantity) interval is configurable
  bool hasSpecificInterval() const { return (_operatingFunctions & cmd::FEATURE_SPECIFIC_INTERVAL) != 0; }

  /// Check if measurement filter is configurable
  bool hasFilterConfig() const { return (_operatingFunctions & cmd::FEATURE_FILTER_CONFIG) != 0; }

  /// Check if error code register exists
  bool hasErrorCode() const { return (_operatingFunctions & cmd::FEATURE_ERROR_CODE) != 0; }

  /// Check if low power mode is supported
  bool hasLowPowerMode() const { return (_operatingModeSupport & cmd::MODE_SUPPORT_LOW_POWER) != 0; }

  /// Check if E2 priority mode is supported
  bool hasE2Priority() const { return (_operatingModeSupport & cmd::MODE_SUPPORT_E2_PRIORITY) != 0; }

  /// Check if auto adjustment is supported
  bool hasAutoAdjust() const { return (_specialFeatures & cmd::SPECIAL_FEATURE_AUTO_ADJUST) != 0; }

  // =========================================================================
  // Identity Strings
  // =========================================================================

  /// Read 16-byte serial number (0xA0-0xAF)
  /// @param buf Buffer of at least 16 bytes
  /// @return Status::Ok() when all 16 bytes are read.
  Status readSerialNumber(uint8_t* buf);

  /// Read 16-byte part name (0xB0-0xBF)
  /// @param buf Buffer of at least 16 bytes
  /// @return Status::Ok() when all 16 bytes are read.
  Status readPartName(uint8_t* buf);

  /// Write 16-byte part name (0xB0-0xBF)
  /// @param buf Buffer of exactly 16 bytes
  /// @return Status::Ok() when all bytes verify. A failure after one byte
  /// succeeds marks persistent configuration dirty.
  Status writePartName(const uint8_t* buf);

  // =========================================================================
  // Bus Address
  // =========================================================================

  /// Read current bus address (0xC0).
  /// @param[out] address Current E2 device address.
  /// @return Status::Ok() when the byte is read.
  Status readBusAddress(uint8_t& address);

  /// Write bus address (0xC0) - requires power cycle to take effect.
  /// This updates persistent sensor configuration and does not retarget the
  /// current driver session.
  /// @param address New address (0-7)
  /// @return OUT_OF_RANGE if address > 7
  Status writeBusAddress(uint8_t address);

  // =========================================================================
  // Measurement Interval
  // =========================================================================

  /// Read global measurement interval
  /// @param intervalDeciSeconds Interval in 0.1 s units
  /// @return Status::Ok() when both bytes are read.
  Status readMeasurementInterval(uint16_t& intervalDeciSeconds);

  /// Read CO2-specific interval factor (0xCB)
  /// Positive = multiplier, Negative = divider
  /// @param[out] factor Signed interval factor.
  /// @return Status::Ok() when the byte is read.
  Status readCo2IntervalFactor(int8_t& factor);

  /// Write CO2-specific interval factor (0xCB)
  /// @param factor Signed interval factor.
  /// @return Status::Ok() when the byte verifies.
  Status writeCo2IntervalFactor(int8_t factor);

  // =========================================================================
  // Filter / Operating Mode
  // =========================================================================

  /// Read CO2 filter setting (0xD3).
  /// @param[out] filter Filter setting byte.
  /// @return Status::Ok() when the byte is read.
  Status readCo2Filter(uint8_t& filter);

  /// Write CO2 filter setting (0xD3).
  /// @param filter Filter setting byte.
  /// @return Status::Ok() when the byte verifies.
  Status writeCo2Filter(uint8_t filter);

  /// Read operating mode (0xD8)
  /// @param[out] mode Operating-mode byte.
  /// @return Status::Ok() when the byte is read.
  /// @see cmd::OPERATING_MODE_* constants
  Status readOperatingMode(uint8_t& mode);

  /// Write operating mode (0xD8)
  /// bit0: 0=freerunning, 1=low power
  /// bit1: 0=measurement priority, 1=E2 priority
  /// @param mode Operating-mode byte.
  /// @return Status::Ok() when the byte verifies.
  Status writeOperatingMode(uint8_t mode);

  // =========================================================================
  // Auto Adjustment
  // =========================================================================

  /// Check if auto adjustment is running (0xD9 bit0).
  /// @param[out] running true when auto adjustment is running.
  /// @return Status::Ok() when the byte is read.
  Status readAutoAdjustStatus(bool& running);

  /// Start auto adjustment (cannot be stopped once started)
  /// Device will return 0x55 during adjustment (~5 min)
  /// @return Status::Ok() when the control byte verifies.
  Status startAutoAdjust();

  // =========================================================================
  // Calibration (Advanced)
  // =========================================================================

  /// Read CO2 offset (signed, ppm).
  /// @param[out] offset Signed offset in ppm.
  /// @return Status::Ok() when both bytes are read.
  Status readCo2Offset(int16_t& offset);

  /// Write CO2 offset (signed, ppm).
  /// @param offset Signed offset in ppm.
  /// @return Status::Ok() when both bytes verify. A high-byte failure after the
  /// low byte succeeds marks persistent configuration dirty.
  Status writeCo2Offset(int16_t offset);

  /// Read CO2 gain (gain = value / 32768).
  /// @param[out] gain Raw gain value.
  /// @return Status::Ok() when both bytes are read.
  Status readCo2Gain(uint16_t& gain);

  /// Write CO2 gain (gain = value / 32768).
  /// @param gain Raw gain value.
  /// @return Status::Ok() when both bytes verify. A high-byte failure after the
  /// low byte succeeds marks persistent configuration dirty.
  Status writeCo2Gain(uint16_t gain);

  /// Read last calibration points.
  /// @param[out] lower Lower calibration point in ppm.
  /// @param[out] upper Upper calibration point in ppm.
  /// @return Status::Ok() when both 16-bit values are read.
  Status readCo2CalPoints(uint16_t& lower, uint16_t& upper);

  // =========================================================================
  // Status / Measurements
  // =========================================================================

  /// Read status byte; this can trigger a new measurement on EE871.
  /// @param[out] status Status byte.
  /// @return Status::Ok() when the status byte and PEC verify.
  Status readStatus(uint8_t& status);

  /// Check if CO2 error bit is set in a status byte
  /// @param statusByte Value previously read via readStatus()
  /// @return true if bit3 (CO2 error) is set
  static constexpr bool hasCo2Error(uint8_t statusByte) {
    return (statusByte & cmd::STATUS_CO2_ERROR_MASK) != 0;
  }

  /// Read CO2 error code from custom memory 0xC1.
  /// @param[out] code Error code, valid when status bit3 is set and the feature is supported.
  /// @return Status::Ok() when the byte is read.
  Status readErrorCode(uint8_t& code);

  /// Read CO2 fast response value from MV3.
  /// @param[out] ppm CO2 concentration in ppm.
  /// @return Status::Ok() when MV3 low/high reads succeed.
  Status readCo2Fast(uint16_t& ppm);

  /// Read CO2 averaged value from MV4.
  /// @param[out] ppm CO2 concentration in ppm.
  /// @return Status::Ok() when MV4 low/high reads succeed.
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

  Status _writeCommandRaw(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte,
                          bool* writeAccepted = nullptr);
  Status _writeCommandTracked(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte,
                              bool* writeAccepted = nullptr);
  Status _customWriteDirect(uint8_t address, uint8_t value, bool* writeAccepted = nullptr);

  // =========================================================================
  // Health Management
  // =========================================================================

  /// Update health counters and state based on operation result
  /// Called ONLY from tracked transport wrappers
  Status _updateHealth(const Status& st);

  void _resetStoppedState();
  void _markPersistentConfigDirty(const Status& st);
  void _clearPersistentConfigDirty();

  // =========================================================================
  // State
  // =========================================================================

  Config _config;
  bool _initialized = false;
  DriverState _driverState = DriverState::UNINIT;
  uint32_t _nowMs = 0;

  // Feature flags (cached during begin())
  uint8_t _operatingFunctions = 0;   ///< Cached 0x07
  uint8_t _operatingModeSupport = 0; ///< Cached 0x08
  uint8_t _specialFeatures = 0;      ///< Cached 0x09

  // Health counters
  uint32_t _lastOkMs = 0;
  uint32_t _lastErrorMs = 0;
  Status _lastError = Status::Ok();
  uint8_t _consecutiveFailures = 0;
  uint32_t _totalFailures = 0;
  uint32_t _totalSuccess = 0;
  bool _persistentConfigDirty = false;
  Status _persistentConfigDirtyError = Status::Ok();
};

} // namespace EE871
