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

/// @brief Public operation classes used for conservative blocking admission.
///
/// Values describe library/device operations only. They do not imply an RTOS,
/// queue, scheduler, or product command.
enum class OperationKind : uint8_t {
  CONTROL_READ = 0,             ///< One control-byte addressed read transaction.
  CUSTOM_POINTER_WRITE = 1,     ///< One 0x50 pointer write and completion window.
  CUSTOM_BYTE_READ = 2,         ///< Pointer write followed by one 0x51 read.
  CUSTOM_BLOCK_READ = 3,        ///< Pointer write followed by elementCount 0x51 reads.
  CUSTOM_BYTE_WRITE_VERIFY = 4, ///< 0x10 write, completion, pointer write, and readback.
  INTERVAL_WRITE_VERIFY = 5,    ///< Staged interval pair commit and two-byte readback.
  PART_NAME_WRITE_VERIFY = 6,   ///< Complete fixed 16-byte part-name write/readback sequence.
  RAW_CO2_READ = 7,             ///< Low-byte then high-byte raw MV3/MV4 read.
  BUS_RESET = 8,                ///< Nine reset clocks, bounded line waits, and STOP.
  BEGIN_REQUIRE_PRESENT = 9,    ///< Strict startup with reset, full identity, and capabilities.
  BEGIN_ALLOW_ABSENT = 10,      ///< Optional startup; conservative bound matches strict startup.
  PROBE_IDENTITY = 11,          ///< Raw diagnostic validation of full EE871 CO2 identity.
  RECOVER_IDENTITY_AND_CAPABILITIES = 12, ///< Tracked reset, identity, and capability reload.
  CHECKED_CO2_AVERAGE = 13,     ///< MV4, status, and worst-case error-code procedure.
  CHECKED_CO2_FAST = 14,        ///< MV3, status, and worst-case error-code procedure.
  CUSTOM_BLOCK_WRITE_VERIFY = 15, ///< One to sixteen persistent bytes with target-specific verification.
  RESYNC_PERSISTENT_CONFIG = 16, ///< Worst-case capability-aware persistent-state reconciliation.
  AUTO_ADJUST_MAINTENANCE = 17, ///< Auto-adjust pre-observation, write, and post-observation.
  BUS_ADDRESS_CHANGE = 18       ///< One address-register write without unsafe in-session readback.
};

/// @brief Conservative blocking-time result for one operation class.
struct OperationTimingBound {
  OperationKind kind{OperationKind::CONTROL_READ}; ///< Operation represented by this result.
  uint16_t elementCount{1}; ///< Block element count; one for fixed-size operations.
  uint32_t maxBlockingMs{0}; ///< Conservative maximum while callbacks honor their contract.
};

/// @brief Atomically cached EE871 identity and CO2 availability.
struct DeviceIdentity {
  uint16_t group{0};                 ///< Raw sensor group identifier.
  uint8_t subgroup{0};               ///< Raw sensor subgroup identifier.
  uint8_t availableMeasurements{0};  ///< Raw available-measurements bitfield.
  bool co2Available{false};          ///< True when the CO2 bit is advertised.
  bool valid{false};                 ///< True only after complete identity validation.
};

/// @brief Atomically cached custom-memory capability bytes 0x03..0x09.
struct CapabilitySnapshot {
  uint8_t customAdjustmentSupport{0};      ///< Custom byte 0x03.
  uint8_t adjustmentPointSupport{0};       ///< Custom byte 0x04.
  uint8_t adjustmentTimeGeneralSupport{0}; ///< Custom byte 0x05.
  uint8_t adjustmentTimeSupport{0};        ///< Custom byte 0x06.
  uint8_t operatingFunctions{0};           ///< Custom byte 0x07.
  uint8_t operatingModeSupport{0};         ///< Custom byte 0x08.
  uint8_t specialFeatures{0};              ///< Custom byte 0x09.
  bool valid{false};                       ///< True only after all seven bytes load and validate.
};

/// @brief EE871 CO2 measured-value source selected by a checked read.
enum class Co2ValueKind : uint8_t {
  FAST = 0,    ///< Fast, unaveraged MV3 value.
  AVERAGE = 1, ///< Averaged MV4 value.
};

/// @brief Normalized EE871 CO2 sensor-domain error.
enum class Co2SensorError : uint8_t {
  NONE = 0,                              ///< Status reports no CO2 error.
  SUPPLY_VOLTAGE_LOW = 1,                ///< Error code 1.
  SENSOR_COUNTS_LOW = 200,               ///< Error code 200.
  SENSOR_COUNTS_HIGH = 201,              ///< Error code 201.
  SUPPLY_VOLTAGE_BREAKDOWN_AT_PEAK = 202, ///< Error code 202.
  UNKNOWN = 255,                         ///< Status error with no recognized detail.
};

/// @brief Per-step evidence from a checked CO2 value/status procedure.
///
/// Attempted flags distinguish an unattempted step from a successful step:
/// default Status::Ok() alone does not prove that a bus operation ran.
struct Co2ReadResult {
  Co2ValueKind kind{Co2ValueKind::AVERAGE}; ///< Requested MV3/MV4 value kind.

  uint16_t ppm{0};       ///< Raw ppm retained even when checked validity fails.
  bool ppmValid{false};  ///< True only after clean status and range validation.

  uint8_t statusByte{0};   ///< Raw status byte when statusValid is true.
  bool statusValid{false}; ///< True after a successful status transfer.
  bool co2Error{false};    ///< CO2 status bit derived from a valid status byte.

  uint8_t errorCode{0};      ///< Raw custom-memory 0xC1 code when valid.
  bool errorCodeValid{false}; ///< True after a successful supported code read.
  Co2SensorError sensorError{Co2SensorError::NONE}; ///< NONE for clean status; UNKNOWN for unclassified status error.

  bool valueReadAttempted{false};     ///< True once the MV3/MV4 read is started.
  bool statusReadAttempted{false};    ///< True once the status read is started.
  bool errorCodeReadAttempted{false}; ///< True once a supported code read starts.

  Status valueReadStatus{Status::Ok()};     ///< Exact raw value-read result.
  Status statusReadStatus{Status::Ok()};    ///< Exact status-read result.
  Status errorCodeReadStatus{Status::Ok()}; ///< Exact error-code-read result.
};

/// @brief Persistent configuration or explicit maintenance target.
enum class MutationTarget : uint8_t {
  NONE = 0,                ///< No mutation has been admitted.
  RAW_CUSTOM_BYTE = 1,     ///< Ordinary writable custom-memory byte.
  PART_NAME = 2,           ///< Fixed 16-byte part name.
  BUS_ADDRESS = 3,         ///< Persistent E2 device address.
  GLOBAL_INTERVAL = 4,     ///< Deferred-commit 0xC6/0xC7 interval pair.
  CO2_INTERVAL_FACTOR = 5, ///< CO2-specific interval factor.
  CO2_FILTER = 6,          ///< CO2 filter setting.
  OPERATING_MODE = 7,      ///< Operating-mode setting.
  AUTO_ADJUST = 8,         ///< Irreversible auto-adjust maintenance action.
  CO2_OFFSET = 9,          ///< Paired CO2 offset setting.
  CO2_GAIN = 10,           ///< Paired CO2 gain setting.
};

/// @brief Best evidence retained for the most recently admitted mutation.
enum class MutationEffect : uint8_t {
  NONE = 0,                  ///< No effectful request was admitted.
  NO_EFFECT = 1,             ///< The request was definitely rejected or never completed.
  ACKNOWLEDGED = 2,          ///< Accepted, but final target state was not proved.
  INDETERMINATE = 3,         ///< Complete PEC sent; acceptance was ambiguous.
  VERIFIED = 4,              ///< Target-specific observation matched the request.
  RESYNCHRONIZED = 5,        ///< Coherent actual state was observed but did not match.
  OPERATOR_ACKNOWLEDGED = 6, ///< Auto-adjust historical ambiguity was explicitly accepted.
};

/// @brief Fixed-size evidence for one persistent or maintenance mutation.
///
/// For a multi-byte target, attemptedValue is the most recently attempted
/// element and observedValue is the most recently observed element. Historical
/// cause and observation evidence may remain after reconciliation; unresolved
/// alone controls mutation admission and the legacy dirty mirror.
struct MutationDiagnostic {
  bool unresolved{false}; ///< True until explicit target reconciliation.
  MutationTarget target{MutationTarget::NONE}; ///< Most recently admitted target.
  MutationEffect effect{MutationEffect::NONE}; ///< Strongest retained effect evidence.
  uint8_t firstAddress{0}; ///< First custom-memory address in the target.
  uint8_t lastAddress{0};  ///< Last custom-memory address in the target.
  uint16_t elementsRequested{0}; ///< Elements in the admitted intent.
  uint16_t elementsAcknowledged{0}; ///< Requests definitely ACKed.
  uint16_t elementsObserved{0}; ///< Target elements read successfully.
  uint16_t elementsMatched{0};  ///< Observed elements equal to retained intent.
  uint8_t attemptedValue{0}; ///< Most recently attempted target byte.
  uint8_t preObservedValue{0}; ///< Auto-adjust pre-write observation.
  bool preObservedValueValid{false}; ///< True after successful pre-observation.
  uint8_t observedValue{0}; ///< Most recently observed target byte.
  bool observedValueValid{false}; ///< True after at least one observation.
  Status cause{Status::Ok()}; ///< First unresolved cause or settled history.
};

/// @brief Snapshot of current configuration, cached feature flags, and driver health.
///
/// Snapshot access does not touch the E2 bus. The legacy persistent dirty
/// fields mirror the appended mutation diagnostic so diagnostics can inspect
/// unresolved persistent or explicit maintenance state without bus traffic.
struct SettingsSnapshot {
  Config config;                  ///< Active normalized configuration copied from begin().
  DriverState state = DriverState::UNINIT; ///< Current coarse health state.
  bool initialized = false;       ///< True after successful begin().
  uint32_t nowMs = 0;             ///< Last tick() timestamp seen by the driver.
  uint8_t operatingFunctions = 0; ///< Cached custom-memory 0x07 feature flags.
  uint8_t operatingModeSupport = 0; ///< Cached custom-memory 0x08 mode flags.
  uint8_t specialFeatures = 0;    ///< Cached custom-memory 0x09 feature flags.
  uint32_t lastOkMs = 0;          ///< Last tracked successful E2 operation.
  uint32_t lastErrorMs = 0;       ///< Last tracked failed E2 operation.
  Status lastError = Status::Ok(); ///< Last tracked error, or semantic recovery incompatibility.
  uint8_t consecutiveFailures = 0; ///< Tracked streak, or normalized semantic OFFLINE latch.
  uint32_t totalFailures = 0;     ///< Total tracked failures.
  uint32_t totalSuccess = 0;      ///< Total tracked successes.
  bool persistentConfigDirty = false; ///< Legacy mirror of mutation.unresolved.
  Status persistentConfigDirtyError = Status::Ok(); ///< Legacy unresolved cause, otherwise OK.
  BeginPolicy beginPolicy{BeginPolicy::REQUIRE_PRESENT}; ///< Active startup policy.
  Status beginProbeStatus{Status::Ok()}; ///< Accepted startup absence or OK.
  DeviceIdentity identity{};      ///< Atomically cached validated identity.
  CapabilitySnapshot capabilities{}; ///< Atomically cached capability bytes.
  MutationDiagnostic mutation{}; ///< Cache-only persistent/maintenance evidence.
};

/// @brief Transport-agnostic EE871 CO2 sensor driver for the E2 bus.
///
/// EE871-E2 uses GPIO-style open-drain E2 signaling through injected line and
/// delay callbacks. It is not an Arduino Wire, ESP-IDF hardware I2C, or other
/// owned-bus driver.
///
/// The driver is non-copyable and non-movable so callback-owned state, cached
/// health, and mutation diagnostics stay associated with one stable
/// instance.
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
  /// @brief Construct an uninitialized driver instance.
  EE871() = default;

  /// @brief Copying is disabled; keep driver instances in stable storage.
  EE871(const EE871&) = delete;
  /// @brief Copy assignment is disabled; pass references or pointers instead.
  EE871& operator=(const EE871&) = delete;
  /// @brief Moving is disabled; callbacks and diagnostics are instance-bound.
  EE871(EE871&&) = delete;
  /// @brief Move assignment is disabled.
  EE871& operator=(EE871&&) = delete;

  // =========================================================================
  // Lifecycle
  // =========================================================================

  /// Initialize the driver with configuration.
  ///
  /// begin() validates timing and callbacks, normalizes configuration, validates
  /// the complete EE871 CO2 identity, and atomically caches custom-memory
  /// capabilities 0x03..0x09. REQUIRE_PRESENT fails closed on any discovery
  /// error. ALLOW_ABSENT accepts only DEVICE_NOT_FOUND produced by an
  /// authoritative presence mechanism. The GPIO E2 transport cannot make that
  /// distinction, so its NACK remains NACK and leaves the driver uninitialized.
  /// Responding incompatible devices and partial capability reads also fail.
  ///
  /// The driver does not configure GPIO, pins, pull-ups, tasks, locks, or
  /// framework handles.
  /// @param config Configuration including E2 transport callbacks.
  /// @return Status::Ok() on success, error otherwise.
  /// @note Timing contract: BUS BEGIN_REQUIRE_PRESENT or BEGIN_ALLOW_ABSENT.
  Status begin(const Config& config);

  /// Record the latest application timestamp for diagnostics.
  ///
  /// The current driver performs synchronous bus operations in public API calls;
  /// tick() does not advance hidden asynchronous E2 transfers.
  /// @param nowMs Current timestamp in milliseconds.
  /// @note Timing contract: NO_E2_IO.
  void tick(uint32_t nowMs);

  /// End the driver session and clear runtime/device caches.
  ///
  /// The core driver owns no GPIO or framework resources, so application-owned
  /// callback state remains the caller's responsibility. Retained mutation
  /// evidence and intent survive on this object; destroying the object loses
  /// that RAM evidence. Applications requiring restart/power-loss continuity
  /// must persist maintenance workflow state outside this library.
  /// @note Timing contract: NO_E2_IO.
  void end();

  // =========================================================================
  // Diagnostics
  // =========================================================================

  /// Check if device is present on the bus.
  ///
  /// probe() uses raw diagnostic transfers and validates group, subgroup, and
  /// advertised CO2 support. It is callable while OFFLINE and does not update
  /// health, state, begin diagnostics, identity, or capability caches.
  /// @return Status::Ok() if a compatible EE871 CO2 device responds, preserving
  /// the original precise transport or semantic error otherwise.
  /// @note Timing contract: BUS PROBE_IDENTITY.
  Status probe();

  /// Attempt to recover from DEGRADED/OFFLINE state.
  ///
  /// Recovery performs a tracked bounded bus reset, validates full identity,
  /// reloads all seven capabilities into local candidates, and publishes both
  /// only after complete success. It is the only operation that can restore a
  /// latched OFFLINE driver. Retry cadence remains application-owned.
  /// @return Status::Ok() after entering READY with fresh caches; otherwise the
  /// original precise failure.
  /// @note Timing contract: BUS RECOVER_IDENTITY_AND_CAPABILITIES.
  Status recover();

  /// Reconcile retained mutation intent or read all supported persistent state.
  ///
  /// With an unresolved mutation, this reads only the retained target and
  /// reports VERIFIED when it matches or RESYNCHRONIZED when a coherent
  /// ordinary target does not match. The bus-address target requires the
  /// caller to end the old session, follow the authorized power procedure,
  /// explicitly configure the retained candidate address, and begin there
  /// before calling this method. The driver never scans addresses.
  ///
  /// Without unresolved intent, this performs one capability-aware coherence
  /// pass and skips unsupported optional settings. Applications must compare
  /// observed configuration with their own intended baseline.
  ///
  /// AUTO_ADJUST remains unresolved after a later not-running observation
  /// because completed and never-started history cannot be distinguished.
  /// Only acknowledgeAutoAdjustUncertainty() can accept that narrow case.
  ///
  /// This API touches the E2 bus, is blocking within configured timing/write
  /// delay bounds, is not ISR-safe, and uses tracked operations that can update
  /// health on transfer failure.
  /// @return Status::Ok() after coherent reconciliation, a precise read or
  /// semantic failure, or PERSISTENT_STATE_UNCERTAIN for unresolved
  /// auto-adjust/address preconditions.
  /// @note Timing contract: BUS RESYNC_PERSISTENT_CONFIG.
  Status resyncPersistentConfig();

  // =========================================================================
  // Driver State
  // =========================================================================

  /// Get current driver state.
  /// @return Current coarse health state.
  /// @note Timing contract: NO_E2_IO.
  DriverState state() const { return _driverState; }

  /// Alias for state(), matching the shared driver-health naming.
  /// @return Current coarse health state.
  /// @note Timing contract: NO_E2_IO.
  DriverState driverState() const { return _driverState; }

  /// Alias for driverState(), used by shared diagnostics.
  /// @return Current coarse health state.
  /// @note Timing contract: NO_E2_IO.
  DriverState healthState() const { return _driverState; }

  /// Check whether begin() has completed successfully.
  /// @return true after successful begin(), including any authoritative
  /// optional-absence result, and before end().
  /// @note Timing contract: NO_E2_IO.
  bool isInitialized() const { return _initialized; }

  /// Check if driver is ready for operations.
  /// @return true when the driver is READY or DEGRADED. Any authoritative
  /// accepted absence is initialized but returns false because it is OFFLINE.
  /// @note Timing contract: NO_E2_IO.
  bool isOnline() const {
    return _driverState == DriverState::READY ||
           _driverState == DriverState::DEGRADED;
  }

  /// Active normalized configuration.
  /// @return Current configuration copy stored by begin(), or defaults before begin().
  /// @note Timing contract: NO_E2_IO.
  const Config& getConfig() const { return _config; }

  /// Copy current configuration, feature-cache, and health state.
  /// @param out Receives the current snapshot.
  /// @return Status::Ok(); snapshot access does not touch the E2 bus.
  /// @note Timing contract: NO_E2_IO.
  Status getSettings(SettingsSnapshot& out) const;

  /// Return current configuration, feature-cache, and health state by value.
  /// @return Current settings snapshot.
  /// @note Timing contract: NO_E2_IO.
  SettingsSnapshot getSettings() const;

  /// Return the atomically cached device identity.
  /// @return Copy of the current identity snapshot; performs no E2 I/O.
  /// @note Timing contract: NO_E2_IO.
  DeviceIdentity identity() const { return _identity; }

  /// Return the atomically cached capability bytes.
  /// @return Copy of the current capability snapshot; performs no E2 I/O.
  /// @note Timing contract: NO_E2_IO.
  CapabilitySnapshot capabilities() const { return _capabilities; }

  /// Calculate a conservative blocking bound from the active normalized config.
  ///
  /// This query is cache-only and performs no E2 line reads or writes.
  /// @param kind Operation class to calculate.
  /// @param elementCount One for fixed-size operations; 1..256 for
  /// CUSTOM_BLOCK_READ; 1..16 for CUSTOM_BLOCK_WRITE_VERIFY.
  /// @param[out] out Published only on success.
  /// @return NOT_INITIALIZED before begin(), INVALID_PARAM for an invalid count,
  /// or Status::Ok() with a conservative bound.
  /// @note Timing contract: NO_E2_IO.
  Status operationTimingBound(
      OperationKind kind,
      uint16_t elementCount,
      OperationTimingBound& out) const;

  /// Calculate a conservative blocking bound from a supplied configuration.
  ///
  /// The supplied configuration is validated and normalized exactly as for
  /// begin(). This pure query performs no E2 I/O and does not mutate a driver.
  /// Callback runtime beyond the requested delay is outside the calculated
  /// bound; callbacks must remain bounded and honor requested minimum delays.
  /// @param config Configuration to validate and normalize.
  /// @param kind Operation class to calculate.
  /// @param elementCount One for fixed-size operations; 1..256 for
  /// CUSTOM_BLOCK_READ; 1..16 for CUSTOM_BLOCK_WRITE_VERIFY.
  /// @param[out] out Published only on success.
  /// @return INVALID_CONFIG, INVALID_PARAM, OUT_OF_RANGE, or Status::Ok().
  /// @note Timing contract: NO_E2_IO.
  static Status operationTimingBound(
      const Config& config,
      OperationKind kind,
      uint16_t elementCount,
      OperationTimingBound& out);

  // =========================================================================
  // Health Tracking
  // =========================================================================

  /// Timestamp of last successful E2 operation.
  /// @return Millisecond timestamp supplied through tick().
  /// @note Timing contract: NO_E2_IO.
  uint32_t lastOkMs() const { return _lastOkMs; }

  /// Timestamp of last failed E2 operation.
  /// @return Millisecond timestamp supplied through tick().
  /// @note Timing contract: NO_E2_IO.
  uint32_t lastErrorMs() const { return _lastErrorMs; }

  /// Most recent error status.
  ///
  /// A responding incompatible identity during recover() records semantic
  /// NOT_SUPPORTED here without incrementing transport counters or changing
  /// lastErrorMs(). Otherwise this is the last tracked transfer failure.
  /// @return Last tracked transfer failure or semantic recovery incompatibility.
  /// @note Timing contract: NO_E2_IO.
  Status lastError() const { return _lastError; }

  /// Consecutive failures since last success.
  ///
  /// Authoritative accepted absence and semantic recovery incompatibility
  /// normalize this to offlineThreshold() as a state latch without inventing
  /// transport failures.
  /// @return Current tracked failure streak or normalized OFFLINE latch.
  /// @note Timing contract: NO_E2_IO.
  uint8_t consecutiveFailures() const { return _consecutiveFailures; }

  /// Total failure count (lifetime).
  /// @return Lifetime tracked failure count.
  /// @note Timing contract: NO_E2_IO.
  uint32_t totalFailures() const { return _totalFailures; }

  /// Total success count (lifetime).
  /// @return Lifetime tracked success count.
  /// @note Timing contract: NO_E2_IO.
  uint32_t totalSuccess() const { return _totalSuccess; }

  /// Consecutive failures required before OFFLINE.
  /// @return Normalized threshold currently in use.
  /// @note Timing contract: NO_E2_IO.
  uint8_t offlineThreshold() const { return _config.offlineThreshold; }

  /// Check if persistent or explicit maintenance state is unresolved.
  ///
  /// This legacy source-compatible name mirrors mutationDiagnostic().unresolved,
  /// including an ambiguous single-byte write or auto-adjust maintenance action.
  /// Unrelated reads, begin(), recover(), and health success do not clear it.
  /// @return true when target-specific reconciliation is required.
  /// @note Timing contract: NO_E2_IO.
  bool persistentConfigDirty() const { return _mutationDiagnostic.unresolved; }

  /// First cause retained for unresolved persistent/maintenance state.
  ///
  /// The original failing status is preserved so diagnostics can report the
  /// cause that created the unresolved condition.
  /// @return Stored cause while unresolved, otherwise Status::Ok() even when
  /// historical diagnostic evidence remains.
  /// @note Timing contract: NO_E2_IO.
  Status persistentConfigDirtyError() const {
    return _mutationDiagnostic.unresolved
               ? _mutationDiagnostic.cause
               : Status::Ok();
  }

  /// Return retained persistent/maintenance mutation evidence.
  ///
  /// This cache-only query performs no E2 I/O. Evidence on the same driver
  /// object survives end(), failed begin(), and a later successful begin()
  /// until target-specific reconciliation succeeds.
  /// @return Copy of the current mutation diagnostic.
  /// @note Timing contract: NO_E2_IO.
  MutationDiagnostic mutationDiagnostic() const {
    return _mutationDiagnostic;
  }

  /// Resolve only an observed not-running auto-adjust ambiguity.
  ///
  /// This cache-only operator action is intentionally narrow. It succeeds only
  /// after resyncPersistentConfig() successfully observed 0xD9 not running for
  /// an unresolved AUTO_ADJUST request.
  /// @return Status::Ok() on acknowledgement, otherwise a precise precondition
  /// error.
  /// @note Timing contract: NO_E2_IO.
  Status acknowledgeAutoAdjustUncertainty();

  // =========================================================================
  // E2 Protocol Helpers
  // =========================================================================

  /// Read a control-byte addressed value.
  /// @param mainCommandNibble EE871-supported main-command nibble.
  /// @param[out] data Returned data byte.
  /// @return Status::Ok() on success; NOT_SUPPORTED for EE871-unsupported measurement commands.
  /// @note Timing contract: BUS CONTROL_READ.
  Status readControlByte(uint8_t mainCommandNibble, uint8_t& data);

  /// Read a 16-bit value using low/high control bytes.
  /// @param mainCommandLow Low-byte main-command nibble.
  /// @param mainCommandHigh High-byte main-command nibble.
  /// @param[out] value Little-endian assembled value.
  /// @return Status::Ok() when both byte reads succeed.
  /// @note Timing contract: BUS RAW_CO2_READ.
  Status readU16(uint8_t mainCommandLow, uint8_t mainCommandHigh, uint16_t& value);

  /// Set internal custom pointer using command 0x50.
  /// @param address Custom-memory address; only the low byte is sent for EE871.
  /// @return Status::Ok() after the pointer write and configured write delay.
  /// @note Timing contract: BUS CUSTOM_POINTER_WRITE.
  Status setCustomPointer(uint16_t address);

  /// Read one custom-memory byte.
  /// @param address Custom-memory address.
  /// @param[out] data Returned byte.
  /// @return Status::Ok() when pointer write and data read succeed.
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status customRead(uint8_t address, uint8_t& data);

  /// Read a custom-memory block using pointer auto-increment.
  /// @param address First custom-memory address.
  /// @param[out] buf Destination buffer; must be non-null when len > 0.
  /// @param len Number of bytes to read; zero is rejected as INVALID_PARAM.
  /// @return Status::Ok() when all bytes are read, INVALID_PARAM for invalid buffer/length.
  /// @note Timing contract: BUS CUSTOM_BLOCK_READ.
  Status customRead(uint8_t address, uint8_t* buf, size_t len);

  /// Write a safely routed custom-memory target.
  ///
  /// Known typed targets use their capability and semantic checks. Unsafe
  /// interval/calibration halves and documented read-only addresses are
  /// rejected before line I/O. Remaining writable bytes use immediate equality
  /// verification as RAW_CUSTOM_BYTE. That fallback is an expert maintenance
  /// operation: derive address semantics and restoration from authoritative
  /// vendor documentation, and never restore by replaying a raw memory dump.
  /// @param address Custom-memory address.
  /// @param value Byte to write.
  /// @return Status::Ok() when target-specific verification succeeds, or a
  /// precise rejection/uncertainty status.
  /// @note Timing contract: BUS CUSTOM_BYTE_WRITE_VERIFY or AUTO_ADJUST_MAINTENANCE or BUS_ADDRESS_CHANGE.
  Status customWrite(uint8_t address, uint8_t value);

  /// Write global measurement interval (0xC6/0xC7) and verify
  /// @param intervalDeciSeconds Interval in 0.1 s units
  /// @return Status::Ok() when both interval bytes verify. Accepted partial
  /// pair progress remains observable in mutationDiagnostic();
  /// NOT_SUPPORTED is returned before I/O without cached support.
  /// @note Timing contract: BUS INTERVAL_WRITE_VERIFY.
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
  /// @note Timing contract: BUS RAW_CO2_READ.
  Status readGroup(uint16_t& group);
  /// Read EE871 sensor subgroup identifier.
  /// @param[out] subgroup Expected value is cmd::SENSOR_SUBGROUP_ID.
  /// @return Status::Ok() when the subgroup byte is read.
  /// @note Timing contract: BUS CONTROL_READ.
  Status readSubgroup(uint8_t& subgroup);
  /// Read the available-measurements bitfield.
  /// @param[out] bits Measurement availability flags.
  /// @return Status::Ok() when the bitfield is read.
  /// @note Timing contract: BUS CONTROL_READ.
  Status readAvailableMeasurements(uint8_t& bits);

  // =========================================================================
  // Firmware / Spec Version
  // =========================================================================

  /// Read firmware version (main.sub)
  /// @param[out] main Firmware main version byte.
  /// @param[out] sub Firmware sub version byte.
  /// @return Status::Ok() when both bytes are read.
  /// @note Timing contract: BUS CUSTOM_BLOCK_READ.
  Status readFirmwareVersion(uint8_t& main, uint8_t& sub);

  /// Read the diagnostic E2 specification version recorded by the device.
  ///
  /// This evidence does not gate begin(), probe(), or recover(); the available
  /// documentation defines no safe compatibility table for that policy.
  /// @param[out] version E2 specification version byte.
  /// @return Status::Ok() when the byte is read.
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readE2SpecVersion(uint8_t& version);

  // =========================================================================
  // Feature Discovery
  // =========================================================================

  /// Read operating functions bitfield (0x07)
  /// @param[out] bits Feature flags from custom memory 0x07.
  /// @return Status::Ok() when the byte validates; NOT_SUPPORTED with packed
  /// address/raw detail when a reserved bit is set. Output is unchanged on
  /// failure.
  /// @see cmd::FEATURE_* constants for bit meanings
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readOperatingFunctions(uint8_t& bits);

  /// Read operating mode support bitfield (0x08)
  /// @param[out] bits Operating-mode support flags from custom memory 0x08.
  /// @return Status::Ok() when the byte validates; NOT_SUPPORTED with packed
  /// address/raw detail when a reserved bit is set. Output is unchanged on
  /// failure.
  /// @see cmd::MODE_SUPPORT_* constants
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readOperatingModeSupport(uint8_t& bits);

  /// Read special features bitfield (0x09)
  /// @param[out] bits Special-feature flags from custom memory 0x09.
  /// @return Status::Ok() when the byte validates; NOT_SUPPORTED with packed
  /// address/raw detail when a reserved bit is set. Output is unchanged on
  /// failure.
  /// @see cmd::SPECIAL_FEATURE_* constants
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readSpecialFeatures(uint8_t& bits);

  // =========================================================================
  // Feature Support Queries (use cached values from begin())
  // =========================================================================

  /// Check if serial number is readable.
  /// @return true when cached feature flags advertise serial number support.
  /// @note Timing contract: NO_E2_IO.
  bool hasSerialNumber() const { return (_operatingFunctions & cmd::FEATURE_SERIAL_NUMBER) != 0; }

  /// Check if part name is readable/writable.
  /// @return true when cached feature flags advertise part-name support.
  /// @note Timing contract: NO_E2_IO.
  bool hasPartName() const { return (_operatingFunctions & cmd::FEATURE_PART_NAME) != 0; }

  /// Check if bus address is configurable.
  /// @return true when cached feature flags advertise address configuration.
  /// @note Timing contract: NO_E2_IO.
  bool hasAddressConfig() const { return (_operatingFunctions & cmd::FEATURE_ADDRESS_CONFIG) != 0; }

  /// Check if global measurement interval is configurable.
  /// @return true when cached feature flags advertise global interval support.
  /// @note Timing contract: NO_E2_IO.
  bool hasGlobalInterval() const { return (_operatingFunctions & cmd::FEATURE_GLOBAL_INTERVAL) != 0; }

  /// Check if specific (per-quantity) interval is configurable.
  /// @return true when cached feature flags advertise specific interval support.
  /// @note Timing contract: NO_E2_IO.
  bool hasSpecificInterval() const { return (_operatingFunctions & cmd::FEATURE_SPECIFIC_INTERVAL) != 0; }

  /// Check if measurement filter is configurable.
  /// @return true when cached feature flags advertise filter configuration.
  /// @note Timing contract: NO_E2_IO.
  bool hasFilterConfig() const { return (_operatingFunctions & cmd::FEATURE_FILTER_CONFIG) != 0; }

  /// Check if error code register exists.
  /// @return true when cached feature flags advertise error-code support.
  /// @note Timing contract: NO_E2_IO.
  bool hasErrorCode() const { return (_operatingFunctions & cmd::FEATURE_ERROR_CODE) != 0; }

  /// Check if CO2 offset/gain adjustment is supported.
  /// @return true when cached custom byte 0x03 advertises CO2 adjustment;
  /// performs no E2 I/O.
  /// @note Timing contract: NO_E2_IO.
  bool hasCo2OffsetGain() const {
    return (_capabilities.customAdjustmentSupport &
            cmd::FEATURE_CO2_CUSTOM_ADJUSTMENT) != 0;
  }

  /// Check if CO2 adjustment points are supported.
  /// @return true when cached custom byte 0x04 advertises CO2 points;
  /// performs no E2 I/O.
  /// @note Timing contract: NO_E2_IO.
  bool hasCo2AdjustmentPoints() const {
    return (_capabilities.adjustmentPointSupport &
            cmd::FEATURE_CO2_ADJUSTMENT_POINT) != 0;
  }

  /// Check if low power mode is supported.
  /// @return true when cached mode flags advertise low-power mode.
  /// @note Timing contract: NO_E2_IO.
  bool hasLowPowerMode() const { return (_operatingModeSupport & cmd::MODE_SUPPORT_LOW_POWER) != 0; }

  /// Check if E2 priority mode is supported.
  /// @return true when cached mode flags advertise E2 priority mode.
  /// @note Timing contract: NO_E2_IO.
  bool hasE2Priority() const { return (_operatingModeSupport & cmd::MODE_SUPPORT_E2_PRIORITY) != 0; }

  /// Check if auto adjustment is supported.
  /// @return true when cached special-feature flags advertise auto adjustment.
  /// @note Timing contract: NO_E2_IO.
  bool hasAutoAdjust() const { return (_specialFeatures & cmd::SPECIAL_FEATURE_AUTO_ADJUST) != 0; }

  // =========================================================================
  // Identity Strings
  // =========================================================================

  /// Read 16-byte serial number (0xA0-0xAF).
  /// @param[out] buf Buffer of at least cmd::CUSTOM_SERIAL_LEN bytes; not NUL-terminated by the driver.
  /// @return Status::Ok() when all 16 bytes are read, INVALID_PARAM for null
  /// buffer, or NOT_SUPPORTED before I/O without cached support.
  /// @note Timing contract: BUS CUSTOM_BLOCK_READ.
  Status readSerialNumber(uint8_t* buf);

  /// Read 16-byte part name (0xB0-0xBF).
  /// @param[out] buf Buffer of at least cmd::CUSTOM_PART_NAME_LEN bytes; not NUL-terminated by the driver.
  /// @return Status::Ok() when all 16 bytes are read, INVALID_PARAM for null
  /// buffer, or NOT_SUPPORTED before I/O without cached support.
  /// @note Timing contract: BUS CUSTOM_BLOCK_READ.
  Status readPartName(uint8_t* buf);

  /// Write 16-byte part name (0xB0-0xBF).
  /// @param buf Buffer of exactly cmd::CUSTOM_PART_NAME_LEN bytes; embedded NUL bytes are written as data.
  /// @return Status::Ok() when all bytes verify. Accepted partial progress
  /// remains observable in mutationDiagnostic(); null returns INVALID_PARAM
  /// and missing cached support returns NOT_SUPPORTED before I/O.
  /// @note Timing contract: BUS PART_NAME_WRITE_VERIFY.
  Status writePartName(const uint8_t* buf);

  // =========================================================================
  // Bus Address
  // =========================================================================

  /// Read current bus address (0xC0).
  /// @param[out] address Current E2 device address.
  /// @return Status::Ok() for 0..7; OUT_OF_RANGE with raw detail for an invalid
  /// persisted byte; NOT_SUPPORTED before I/O when cached capabilities do not
  /// advertise address configuration. Output is unchanged on failure.
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readBusAddress(uint8_t& address);

  /// Request a persistent bus-address change through the current session.
  ///
  /// Activation timing is not guessed. After a clean acknowledgement this
  /// returns PERSISTENT_STATE_UNCERTAIN and retains the candidate. The
  /// application explicitly calls end(), follows its authorized vendor/product
  /// power procedure if required, supplies Config::deviceAddress=candidate to
  /// begin(), then calls resyncPersistentConfig(). The driver neither reads
  /// back through the old address nor scans alternate addresses.
  /// @param address New address (0-7)
  /// @return OUT_OF_RANGE if address > 7, NOT_SUPPORTED without capability,
  /// a precise transfer failure, or PERSISTENT_STATE_UNCERTAIN after ACK.
  /// @note Timing contract: BUS BUS_ADDRESS_CHANGE.
  Status writeBusAddress(uint8_t address);

  // =========================================================================
  // Measurement Interval
  // =========================================================================

  /// Read global measurement interval
  /// @param intervalDeciSeconds Interval in 0.1 s units
  /// @return Status::Ok() for 150..36000; OUT_OF_RANGE with assembled raw
  /// detail otherwise; NOT_SUPPORTED before I/O when cached capabilities do
  /// not advertise a global interval. Output is unchanged on failure.
  /// @note Timing contract: BUS CUSTOM_BLOCK_READ.
  Status readMeasurementInterval(uint16_t& intervalDeciSeconds);

  /// Read CO2-specific interval factor (0xCB)
  /// Positive = multiplier, Negative = divider
  /// @param[out] factor Signed interval factor.
  /// @return Status::Ok() for any nonzero signed factor; OUT_OF_RANGE for zero;
  /// NOT_SUPPORTED before I/O when cached capabilities do not advertise a
  /// specific interval. Output is unchanged on failure.
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readCo2IntervalFactor(int8_t& factor);

  /// Write CO2-specific interval factor (0xCB).
  /// @param factor Signed interval factor.
  /// @return Status::Ok() when a nonzero byte verifies; OUT_OF_RANGE for zero;
  /// NOT_SUPPORTED before I/O without cached support. This is a persistent
  /// single-byte write.
  /// @note Timing contract: BUS CUSTOM_BYTE_WRITE_VERIFY.
  Status writeCo2IntervalFactor(int8_t factor);

  // =========================================================================
  // Filter / Operating Mode
  // =========================================================================

  /// Read opaque vendor-defined CO2 filter setting (0xD3).
  /// @param[out] filter Raw product-specific filter byte.
  /// @return Status::Ok() when the byte is read; NOT_SUPPORTED before I/O
  /// when cached capabilities do not advertise filter configuration. Success
  /// does not semantically qualify the product-specific value.
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readCo2Filter(uint8_t& filter);

  /// Write opaque vendor-defined CO2 filter setting (0xD3).
  /// @param filter Raw product-specific filter byte from authoritative device
  /// documentation.
  /// @return Status::Ok() when the byte verifies; NOT_SUPPORTED before I/O
  /// without cached support. This is a persistent single-byte write; equality
  /// verification does not semantically qualify the product-specific value.
  /// @note Timing contract: BUS CUSTOM_BYTE_WRITE_VERIFY.
  Status writeCo2Filter(uint8_t filter);

  /// Read operating mode (0xD8)
  /// @param[out] mode Operating-mode byte.
  /// @return Status::Ok() when reserved bits are clear and every set mode bit
  /// is advertised; OUT_OF_RANGE for reserved bits; NOT_SUPPORTED for an
  /// unadvertised defined bit or when the entire feature is absent. Output is
  /// unchanged on failure.
  /// @see cmd::OPERATING_MODE_* constants
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readOperatingMode(uint8_t& mode);

  /// Write operating mode (0xD8).
  ///
  /// bit0: 0=freerunning, 1=low power. bit1: 0=measurement priority,
  /// 1=E2 priority.
  /// @param mode Operating-mode byte.
  /// @return Status::Ok() when the validated byte verifies; OUT_OF_RANGE for
  /// reserved bits; NOT_SUPPORTED for an unadvertised defined bit or when the
  /// entire feature is absent.
  /// @note Timing contract: BUS CUSTOM_BYTE_WRITE_VERIFY.
  Status writeOperatingMode(uint8_t mode);

  // =========================================================================
  // Auto Adjustment
  // =========================================================================

  /// Check if auto adjustment is running (0xD9 bit0).
  /// @param[out] running true when auto adjustment is running.
  /// @return Status::Ok() when reserved bits are clear; OUT_OF_RANGE with raw
  /// detail otherwise; NOT_SUPPORTED before I/O when cached capabilities do
  /// not advertise auto-adjust. Output is unchanged on failure.
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readAutoAdjustStatus(bool& running);

  /// Start auto adjustment (cannot be stopped once started).
  ///
  /// This explicit maintenance action first reads 0xD9 and returns BUSY without
  /// writing if already running. With external serialization, an observed
  /// not-running pre-state followed by an acknowledged write and immediate
  /// running post-state verifies this request started. The action cannot be
  /// cancelled through E2, measured values may remain held during adjustment,
  /// and an accepted request is never replayed automatically.
  ///
  /// A clean not-running post-observation is historically ambiguous and
  /// returns PERSISTENT_STATE_UNCERTAIN. A later running resync verifies the
  /// request; a later clear observation still requires the narrow cache-only
  /// acknowledgeAutoAdjustUncertainty() operator decision.
  /// @return Status::Ok() only when running is observed, NOT_SUPPORTED before
  /// I/O without cached support, BUSY when already running, or a precise
  /// transport/uncertainty status.
  /// @note Timing contract: BUS AUTO_ADJUST_MAINTENANCE.
  Status startAutoAdjust();

  // =========================================================================
  // Calibration (Advanced)
  // =========================================================================

  /// Read CO2 offset (signed, ppm).
  /// @param[out] offset Signed offset in ppm.
  /// @return Status::Ok() when both bytes are read; NOT_SUPPORTED before I/O
  /// when cached capabilities do not advertise CO2 offset/gain.
  /// @note Timing contract: BUS CUSTOM_BLOCK_READ.
  Status readCo2Offset(int16_t& offset);

  /// Write CO2 offset (signed, ppm).
  /// @param offset Signed offset in ppm.
  /// @return Status::Ok() when both bytes verify; accepted partial progress
  /// remains observable in mutationDiagnostic(); NOT_SUPPORTED is returned
  /// before I/O without cached support.
  /// @note Timing contract: BUS CUSTOM_BLOCK_WRITE_VERIFY.
  Status writeCo2Offset(int16_t offset);

  /// Read CO2 gain (gain = value / 32768).
  /// @param[out] gain Raw gain value.
  /// @return Status::Ok() when both bytes are read; NOT_SUPPORTED before I/O
  /// when cached capabilities do not advertise CO2 offset/gain.
  /// @note Timing contract: BUS CUSTOM_BLOCK_READ.
  Status readCo2Gain(uint16_t& gain);

  /// Write CO2 gain (gain = value / 32768).
  /// @param gain Raw gain value.
  /// @return Status::Ok() when both bytes verify; accepted partial progress
  /// remains observable in mutationDiagnostic(); NOT_SUPPORTED is returned
  /// before I/O without cached support.
  /// @note Timing contract: BUS CUSTOM_BLOCK_WRITE_VERIFY.
  Status writeCo2Gain(uint16_t gain);

  /// Read last calibration points.
  /// @param[out] lower Lower calibration point in ppm.
  /// @param[out] upper Upper calibration point in ppm.
  /// @return Status::Ok() when both 16-bit values are read; NOT_SUPPORTED
  /// before I/O when cached capabilities do not advertise adjustment points.
  /// @note Timing contract: BUS CUSTOM_BLOCK_READ.
  Status readCo2CalPoints(uint16_t& lower, uint16_t& upper);

  // =========================================================================
  // Status / Measurements
  // =========================================================================

  /// Read the status for the last measured values.
  ///
  /// Under documented device conditions, reading status can start/trigger the
  /// next measurement and reset interval timing. Applications own warm-up,
  /// trigger readiness, freshness, and cadence policy.
  /// @param[out] status Status byte.
  /// @return Status::Ok() when the status byte and PEC verify.
  /// @note Timing contract: BUS CONTROL_READ.
  Status readStatus(uint8_t& status);

  /// Check if CO2 error bit is set in a status byte
  /// @param statusByte Value previously read via readStatus()
  /// @return true if bit3 (CO2 error) is set
  /// @note Timing contract: NO_E2_IO.
  static constexpr bool hasCo2Error(uint8_t statusByte) {
    return (statusByte & cmd::STATUS_CO2_ERROR_MASK) != 0;
  }

  /// Read CO2 error code from custom memory 0xC1.
  /// @param[out] code Error code, valid when status bit3 is set and the feature is supported.
  /// @return Status::Ok() when the byte is read.
  /// @note Timing contract: BUS CUSTOM_BYTE_READ.
  Status readErrorCode(uint8_t& code);

  /// Read the raw CO2 fast-response value from MV3.
  ///
  /// This API applies no status or range policy and does not read status.
  /// @param[out] ppm CO2 concentration in ppm.
  /// @return Status::Ok() when MV3 low/high reads succeed.
  /// @note Timing contract: BUS RAW_CO2_READ.
  Status readCo2Fast(uint16_t& ppm);

  /// Read the raw CO2 averaged value from MV4.
  ///
  /// This API applies no status or range policy and does not read status.
  /// @param[out] ppm CO2 concentration in ppm.
  /// @return Status::Ok() when MV4 low/high reads succeed.
  /// @note Timing contract: BUS RAW_CO2_READ.
  Status readCo2Average(uint16_t& ppm);

  /// Read and validate an averaged MV4 CO2 sample.
  ///
  /// Reads MV4 first and status second so status applies to that last value.
  /// The status read can start/trigger the next measurement and reset interval
  /// timing under documented conditions. A reported CO2 error optionally
  /// reads custom error code 0xC1 when cached capabilities advertise it.
  /// Applications retain ownership of warm-up, freshness, trigger readiness,
  /// and cadence policy.
  /// @param[out] out Replaced with complete per-step evidence.
  /// @return Exact transport/protocol failure, CO2_SENSOR_ERROR,
  /// OUT_OF_RANGE, or Status::Ok().
  /// @note Timing contract: BUS CHECKED_CO2_AVERAGE.
  Status readCo2AverageSample(Co2ReadResult& out);

  /// Read and validate a fast MV3 CO2 sample.
  ///
  /// Reads MV3 first and status second so status applies to that last value.
  /// The status read can start/trigger the next measurement and reset interval
  /// timing under documented conditions. A reported CO2 error optionally
  /// reads custom error code 0xC1 when cached capabilities advertise it.
  /// Applications retain ownership of warm-up, freshness, trigger readiness,
  /// and cadence policy.
  /// @param[out] out Replaced with complete per-step evidence.
  /// @return Exact transport/protocol failure, CO2_SENSOR_ERROR,
  /// OUT_OF_RANGE, or Status::Ok().
  /// @note Timing contract: BUS CHECKED_CO2_FAST.
  Status readCo2FastSample(Co2ReadResult& out);

  // =========================================================================
  // Bus Safety
  // =========================================================================

  /// Reset bus state by clocking with SDA high.
  ///
  /// Use after timeout/stuck bus conditions. This touches E2 lines, is blocking
  /// within configured timing bounds, and is not ISR-safe. This diagnostic
  /// operation is health-neutral; use recover() for an explicit tracked
  /// recovery attempt.
  /// @return Ok if bus lines are free after reset.
  /// @note Timing contract: BUS BUS_RESET.
  Status busReset();

  /// Check if bus lines are idle (both high).
  ///
  /// This reads the configured line callbacks and does not issue an E2 transfer.
  /// It is still not ISR-safe unless the application proves its callbacks are
  /// ISR-safe.
  /// @return Ok if idle, BUS_STUCK if either line is low.
  /// @note Timing contract: BUS BUS_RESET.
  Status checkBusIdle();

private:
  enum class ClockWaitClass : uint8_t {
    NORMAL_BIT = 0,
    WRITE_COMPLETION = 1,
    INTERVAL_COMMIT = 2,
  };

  struct ByteDeadline {
    uint32_t elapsedUs{0};
    uint32_t limitUs{0};
  };

  struct CompletionBudget {
    uint32_t consumedUs{0};
    uint32_t limitUs{0};
  };

  struct MutationProgress {
    bool pecTransferred{false};
    bool finalAckObserved{false};
    bool requestAcknowledged{false};
    bool stopCompleted{false};
    uint32_t completionElapsedUs{0};
  };

  struct MutationIntent {
    MutationTarget target{MutationTarget::NONE};
    uint8_t firstAddress{0};
    uint8_t elementCount{0};
    uint8_t values[16]{};
    bool autoAdjustNotRunningObservedAfterFailure{false};
  };

  // =========================================================================
  // Tracked/Raw Transport Wrappers
  // =========================================================================

  static Status _validateConfig(const Config& input, Config& normalized);
  static Status _calculateOperationTimingBound(
      const Config& normalized,
      OperationKind kind,
      uint16_t elementCount,
      OperationTimingBound& out);
  static void _delayUs(
      const Config& config, uint32_t us, ByteDeadline* deadline = nullptr);
  static Status _delayWithinDeadline(
      const Config& config,
      uint32_t us,
      ByteDeadline& deadline);
  static void _delayLongMs(const Config& config, uint32_t totalMs);
  static void _finishCompletionBudget(
      const Config& config, CompletionBudget& budget);
  static Status _waitSclHigh(
      const Config& config,
      ByteDeadline& deadline);
  static Status _waitSclHighCompletion(
      const Config& config,
      ClockWaitClass waitClass,
      CompletionBudget& completionBudget);
  static Status _e2Start(const Config& config);
  static Status _e2Stop(
      const Config& config,
      ClockWaitClass waitClass,
      CompletionBudget* completionBudget = nullptr);
  static Status _writeBit(
      const Config& config, bool bit, ByteDeadline& deadline);
  static Status _readBit(
      const Config& config, bool& bit, ByteDeadline& deadline);
  static Status _writeByte(
      const Config& config, uint8_t value, ByteDeadline& deadline);
  static Status _readByte(
      const Config& config, uint8_t& value, ByteDeadline& deadline);
  static Status _readAck(
      const Config& config,
      bool& acked,
      ClockWaitClass waitClass,
      ByteDeadline& deadline,
      bool* observed = nullptr,
      CompletionBudget* completionBudget = nullptr);
  static Status _sendAck(
      const Config& config, bool ack, ByteDeadline& deadline);

  Status _busResetRaw();
  Status _busResetTracked();
  Status _setCustomPointerRaw(uint8_t address);
  Status _setCustomPointerTracked(uint8_t address);

  Status _readControlByteRaw(
      uint8_t controlByte,
      uint8_t& data);
  Status _readControlByteTracked(uint8_t controlByte, uint8_t& data);

  Status _writeCommandRaw(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte,
                          ClockWaitClass completionClass,
                          MutationProgress* progress = nullptr);
  Status _writeCommandTracked(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte,
                              ClockWaitClass completionClass,
                              MutationProgress* progress = nullptr);
  Status _mutationAdmissionGuard() const;
  Status _beginMutation(
      MutationTarget target,
      uint8_t firstAddress,
      const uint8_t* values,
      uint8_t elementCount);
  Status _writeCustomByteEffectful(
      uint8_t address,
      uint8_t value,
      MutationTarget target,
      ClockWaitClass completionClass,
      MutationProgress& progress);
  void _classifyMutationEffect(
      const Status& status,
      const MutationProgress& progress);
  Status _observeMutationBytes(
      uint8_t firstAddress,
      const uint8_t* expected,
      uint8_t elementCount,
      bool resolveOnSuccess);
  void _resolveMutation(MutationEffect effect);
  Status _writeVerifiedBytes(
      MutationTarget target,
      uint8_t firstAddress,
      const uint8_t* values,
      uint8_t elementCount);
  Status _writeBusAddressDirect(uint8_t address);
  Status _writeMeasurementIntervalDirect(uint16_t intervalDeciSeconds);
  Status _writeCo2IntervalFactorDirect(int8_t factor);
  Status _writeCo2FilterDirect(uint8_t filter);
  Status _writeOperatingModeDirect(uint8_t mode);
  Status _startAutoAdjustDirect();
  Status _writeCo2PairDirect(
      MutationTarget target,
      uint8_t firstAddress,
      uint16_t value);
  Status _resyncUnresolvedMutation();
  Status _resyncAllSupportedPersistentConfig();
  static Status _validateCapabilityValue(
      uint8_t address, uint8_t value);
  static Status _validateBusAddressValue(uint8_t address);
  static Status _validateIntervalValue(uint16_t intervalDeciSeconds);
  static Status _validateIntervalFactorValue(int8_t factor);
  Status _validateOperatingModeValue(uint8_t mode) const;
  static Status _validateAutoAdjustRaw(uint8_t raw);
  Status _validateMutationObservation(
      MutationTarget target,
      const uint8_t* values,
      uint8_t elementCount) const;
  Status _readValidatedCapability(uint8_t address, uint8_t& out);
  Status _readAndValidateIdentityRaw(DeviceIdentity& out);
  Status _readCapabilitiesRaw(CapabilitySnapshot& out);
  Status _readAndValidateIdentityTracked(DeviceIdentity& out);
  Status _readCapabilitiesTracked(CapabilitySnapshot& out);
  Status _readAndValidateIdentity(
      DeviceIdentity& out,
      bool tracked);
  Status _readCapabilities(
      CapabilitySnapshot& out, bool tracked);
  void _publishIdentityAndCapabilities(
      const DeviceIdentity& identity,
      const CapabilitySnapshot& capabilities);
  void _clearIdentityAndCapabilities();
  void _latchSemanticOffline(const Status& cause);
  bool _normalOperationAllowed(Status& status) const;
  Status _readCo2Sample(Co2ValueKind kind, Co2ReadResult& out);

  // =========================================================================
  // Health Management
  // =========================================================================

  /// Update health counters and state based on operation result
  /// Called ONLY from tracked transport wrappers
  Status _updateHealth(const Status& st);

  void _resetStoppedState();

  // =========================================================================
  // State
  // =========================================================================

  Config _config;
  bool _initialized = false;
  DriverState _driverState = DriverState::UNINIT;
  uint32_t _nowMs = 0;

  DeviceIdentity _identity;
  CapabilitySnapshot _capabilities;
  Status _beginProbeStatus = Status::Ok();
  bool _recoveryBypass = false;

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
  MutationDiagnostic _mutationDiagnostic{};
  MutationIntent _mutationIntent{};
};

} // namespace EE871
