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
  OFFLINE    ///< Latched; ordinary operations fail fast without bus traffic. Only recover() restores READY.
};

/// Existing operation paths supported by the pure requested-delay calculator.
/// A count denotes bytes only for CUSTOM_READ; every other kind is one operation.
enum class OperationKind : uint8_t {
  CONTROL_READ,        ///< readControlByte(target): target is the supported main-command nibble.
  CO2_PAIR_READ,       ///< readCo2Fast()/readCo2Average(), including per-byte eligible retries.
  CUSTOM_POINTER,      ///< setCustomPointer(target), with ordinary STOP allowance.
  CUSTOM_READ,         ///< customRead(target, buffer, count); no crossing address 0xFF.
  CUSTOM_WRITE,        ///< customWrite(target, value); C6/C7 include companion read and interval pair.
  INTERVAL_WRITE,      ///< writeMeasurementInterval(): two frames, one quiet wait, pair readback.
  CO2_ADJUSTMENT_READ, ///< readCo2Offset()/readCo2Gain(), including on-demand support read.
  CO2_CAL_POINTS_READ, ///< readCo2CalPoints(), including on-demand support read.
  CO2_ADJUSTMENT_WRITE,///< writeCo2Offset()/writeCo2Gain(), support plus two verified byte writes.
  PART_NAME_WRITE,     ///< writePartName(): all 16 verified bytes.
  AUTO_ADJUST_START,   ///< startAutoAdjust(): status preflight plus verified request byte.
  BEGIN,              ///< Strict begin(), reserving possible reset, identity and capability loads.
  PROBE,              ///< probe(): identity only, without reset or capability load.
  RECOVER,            ///< recover(): reset, identity and capability loads.
  BUS_RESET,          ///< busReset(): nine releases plus STOP, allowing pending flash.
  PERSISTENT_RESYNC,   ///< resyncPersistentConfig(): all 256 pending targets and supported branches.
  CO2_CHECKED_SAMPLE   ///< Checked fast/average sample, including optional error-code read.
};

/// Sum of maximum requested protocol waits/delays under the configured callback contract.
/// This is a conservative budget, not measured elapsed time or a hard wall-time bound.
struct OperationTimingBound {
  uint64_t requestedDelayUs = 0; ///< Microseconds, exact integer sum with no intermediate rounding.
};

/// Session counters for eligible tracked MV3/MV4/status control-byte NACKs.
/// Counters saturate; end()/begin() reset them, recover() preserves them.
/// Last-event fields describe the latest eligible frame that encountered a
/// NACK and remain unchanged by ordinary successes or ineligible commands.
struct ReadRetryDiagnostics {
  uint32_t controlNacks = 0; ///< Includes disabled retries and failed cleanup.
  uint32_t retries = 0; ///< Additional frames actually attempted.
  uint32_t recovered = 0; ///< Frames succeeding after at least one retry.
  uint32_t exhausted = 0; ///< Clean final NACK after all configured nonzero retries.
  uint8_t lastControlByte = 0; ///< Latest eligible NACK-bearing frame; zero before any event.
  uint8_t lastRetriesUsed = 0; ///< Additional attempts used by that frame.
  Status lastError = Status::Ok(); ///< Final frame result; OK after recovery.
  Status lastCleanupError = Status::Ok(); ///< STOP/idle failure blocking retry.
  bool cleanupBlocked = false; ///< STOP or idle check prevented another attempt.
  bool retryVetoed = false; ///< Application guard denied another attempt.
  bool lastRecovered = false; ///< Latest NACK-bearing frame succeeded after a retry.
};

/// Outer public persistent mutation; nested byte writes keep this identity.
enum class PersistentOperation : uint8_t {
  NONE, CUSTOM_WRITE, MEASUREMENT_INTERVAL, PART_NAME, BUS_ADDRESS,
  CO2_INTERVAL_FACTOR, CO2_FILTER, OPERATING_MODE, AUTO_ADJUST, CO2_OFFSET, CO2_GAIN
};

/// Aggregate evidence about the requested register bytes, not physical outcomes.
enum class PersistentEffect : uint8_t {
  NONE,         ///< No possible effect: no attempt or definite pre-acceptance rejection.
  POSSIBLE,     ///< Possible effect or only partial target progress.
  ACKNOWLEDGED, ///< Every target byte received a final PEC ACK; readback is incomplete/different.
  VERIFIED      ///< Every target byte was successfully read back matching its request.
};

/// Latest outer persistent operation, with fixed capacity for the 16-byte part name.
///
/// Masks are relative to startAddress; only bits below length are meaningful.
/// Possible effects include acknowledged and matched bytes. A sampled PEC ACK
/// survives a later clock-low/STOP error; a definite PEC NACK proves no effect
/// for that byte even if cleanup fails. Observations require a complete successful
/// read frame (PEC and STOP included). result preserves the original code/detail.
/// VERIFIED certifies only bytes read: C0 does not establish active-address timing,
/// and D9 does not establish that calibration ran or completed successfully.
///
/// Only the next public mutation replaces this record, including rejected calls.
/// Reads, recover(), resyncPersistentConfig(), and end()/begin() retain it. Resync
/// proves current coherence, never retrospective intent. The independent pending
/// bitmap/first dirty error remains the sole owner of unresolved configuration.
/// Applications retain baseline/intent across operations and own maintenance
/// admission, candidate address sessions, and reconciliation; no automatic replay.
struct PersistentOperationResult {
  static constexpr size_t CAPACITY = 16; ///< Largest supported typed target.
  PersistentOperation operation = PersistentOperation::NONE; ///< Outer API identity.
  uint8_t startAddress = 0; ///< First target register, including paired raw interval writes.
  uint8_t length = 0; ///< Target span; zero before any mutation call.
  uint8_t requested[CAPACITY] = {}; ///< Requested bytes, valid only under requestedValidMask.
  uint8_t observed[CAPACITY] = {}; ///< Readback bytes, valid only under observedValidMask.
  uint16_t requestedValidMask = 0; ///< Raw interval complement becomes valid after its read.
  uint16_t attemptedMask = 0; ///< Direct write procedure attempted; includes START failure, excludes local/OFFLINE rejection.
  uint16_t possibleEffectMask = 0; ///< PEC started without a definite final PEC NACK.
  uint16_t acknowledgedMask = 0; ///< Final PEC ACK was actually sampled.
  uint16_t observedValidMask = 0; ///< Successfully validated readback bytes.
  uint16_t matchedMask = 0; ///< Valid readback equals the valid requested byte.
  Status result = Status::Ok(); ///< Exact outer result; default OK is not an operation.

  /// Summarize the masks; partial target progress remains POSSIBLE.
  /// @return Strongest evidence established for the whole target span.
  PersistentEffect effect() const {
    if (possibleEffectMask == 0) return PersistentEffect::NONE;
    if (length == 0 || length > CAPACITY) return PersistentEffect::POSSIBLE;
    const uint16_t all = static_cast<uint16_t>((uint32_t{1} << length) - 1U);
    if (matchedMask == all) return PersistentEffect::VERIFIED;
    if (acknowledgedMask == all) return PersistentEffect::ACKNOWLEDGED;
    return PersistentEffect::POSSIBLE;
  }
};

/// @brief Fixed-size evidence from an explicit checked MV3/MV4 sample read.
///
/// A step is attempted when its existing typed API is invoked, even if a
/// precondition prevents bus traffic. An unattempted step's default OK Status
/// does not indicate success. Valid fields retain earlier evidence if a later
/// step fails. Every sample call replaces this entire result, including on
/// precondition failure. No field certifies warm-up, freshness, accuracy, or
/// application/product range; the application owns those policies.
struct Co2ReadResult {
  uint16_t ppm = 0;          ///< Raw word; use as evidence only when valueValid.
  bool valueValid = false;  ///< Both value-byte transfers succeeded, including PEC and STOP.
  bool ppmValid = false;    ///< Complete checked procedure succeeded with clear CO2 status.
  uint8_t statusByte = 0;   ///< Raw status byte; meaningful only when statusValid.
  bool statusValid = false; ///< Status transfer succeeded, including PEC and STOP.
  bool co2Error = false;    ///< Status bit3; meaningful only when statusValid.
  uint8_t errorCode = 0;    ///< Raw 0xC1 code, including unknown codes; use only when errorCodeValid.
  bool errorCodeValid = false; ///< Supported error-detail read succeeded.
  bool valueReadAttempted = false; ///< Requested raw MV3/MV4 API was invoked.
  bool statusReadAttempted = false; ///< Side-effecting status API was invoked after valid value.
  bool errorCodeReadAttempted = false; ///< Error-detail API was invoked for an advertised status error.
  Status valueReadStatus = Status::Ok(); ///< Exact value-read result, if attempted.
  Status statusReadStatus = Status::Ok(); ///< Exact status-read result, if attempted.
  Status errorCodeReadStatus = Status::Ok(); ///< Exact error-detail result, if attempted.
};

/// @brief Snapshot of current configuration, cached feature flags, and driver health.
///
/// Snapshot access does not touch the E2 bus. The persistent dirty fields mirror
/// persistentConfigDirty() and persistentConfigDirtyError() so diagnostics can
/// inspect possible partial persistent writes without issuing new bus traffic.
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
  Status lastError = Status::Ok(); ///< Last tracked error status.
  uint8_t consecutiveFailures = 0; ///< Current consecutive tracked failures.
  uint32_t totalFailures = 0;     ///< Total tracked failures.
  uint32_t totalSuccess = 0;      ///< Total tracked successes.
  bool persistentConfigDirty = false; ///< True when persistent config may be partially applied.
  Status persistentConfigDirtyError = Status::Ok(); ///< First error that marked persistent config dirty.
  ReadRetryDiagnostics readRetry; ///< Cached diagnostics; no bus access.
};

/// @brief Transport-agnostic EE871 CO2 sensor driver for the E2 bus.
///
/// EE871-E2 uses GPIO-style open-drain E2 signaling through injected line and
/// delay callbacks. It is not an Arduino Wire, ESP-IDF hardware I2C, or other
/// owned-bus driver.
///
/// The driver is non-copyable and non-movable so callback-owned state, cached
/// health, and persistent dirty diagnostics stay associated with one stable
/// instance.
///
/// Instances are not thread-safe. Use one owner task/context or externally
/// serialize all public calls, including state-only accessors and tick().
/// Shared users of the same GPIO/E2 bus must also serialize outside the
/// library. Public methods that touch the E2 bus are blocking and are not
/// ISR-safe because they may perform bus I/O and call the configured delay
/// callback. Transport callbacks must be bounded and deterministic, and must
/// not call public methods on the same EE871 instance recursively.
///
/// Retries default to disabled. Config::readNackRetries opts into at most three
/// additional attempts for MV3/MV4/status control-byte NACKs before any data,
/// after successful STOP and idle checks, with a fixed 1 ms pause. Other
/// failures, identity/custom reads, and all writes are never retried. The
/// optional allowReadRetry callback can veto for application deadlines or HAL
/// errors. Health counts each tracked frame's final result once; separate
/// diagnostics retain NACK/retry counts. A NACK alone does not identify its
/// physical cause. The application still owns sampling cadence and recovery.
/// A 16-bit read stops after a failed low-byte frame; a retried high byte does
/// not re-read the low byte or disturb its sensor-side latch.
///
/// Ordinary read outputs are valid only when the returned Status is OK. Checked
/// sample results instead retain per-step evidence with explicit validity flags.
/// Raw byte and
/// buffer reads may change their outputs before a later PEC, STOP, or transfer
/// failure; methods that preserve outputs on failure document that separately.
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

  /// Calculate a conservative requested-delay budget without invoking callbacks.
  ///
  /// Pure and usable before begin(), even with null transport callbacks. Uses the
  /// same numeric configuration validation as begin(); begin additionally requires
  /// transport callbacks. Zero post-write waits and offlineThreshold=0 stay valid.
  /// No GPIO, delay, retry, reset, health, capability or evidence state is touched.
  /// Unknown capabilities/state reserve every supported branch; early rejection
  /// and failed transfers consume no more than the returned budget. Full byte
  /// deadlines include nominal phases, and failure cleanup includes STOP.
  ///
  /// CONTROL_READ classifies retries from the actual main command. CUSTOM_WRITE
  /// also covers typed address/mode/factor/filter writes at their register address;
  /// an address change budget covers the stored byte only, not a candidate session.
  /// Simple typed custom reads map to CUSTOM_READ with their address/count;
  /// readU16() is the sum of its two CONTROL_READ bounds. Cache-only diagnostics,
  /// tick() and end() request zero delay and are intentionally excluded. checkBusIdle()
  /// also requests zero delay but reads GPIO and is not a cache-only diagnostic.
  ///
  /// Quiet-wait slices sum to the configured milliseconds. GPIO execution time,
  /// delay overshoot, optional callback/guard/yield overhead, scheduler preemption
  /// and latency are excluded. The application owns any additional wall-time
  /// bounds and admission policy. This calculator neither performs an operation
  /// nor certifies persistent effects, calibration success or physical timing.
  /// @param config Explicit timing/retry configuration; callbacks need not be installed.
  /// @param operation Existing operation class to budget.
  /// @param[out] out Published only on success; unchanged on any error.
  /// @param target Main-command nibble for CONTROL_READ; custom address 0..255 for
  /// CUSTOM_POINTER/READ/WRITE; must be zero for every other operation.
  /// @param count CUSTOM_READ byte count 1..256-target; must be one otherwise.
  /// @return OK, INVALID_CONFIG for numeric configuration errors, INVALID_PARAM
  /// for unknown selectors/unsupported arguments, NOT_SUPPORTED for an unsupported
  /// control command, or OUT_OF_RANGE for address/span/arithmetic overflow.
  static Status operationTimingBound(const Config& config, OperationKind operation,
                                      OperationTimingBound& out, uint16_t target = 0,
                                      size_t count = 1);

  // =========================================================================
  // Lifecycle
  // =========================================================================

  /// Initialize the driver with configuration.
  ///
  /// begin() validates timing and callbacks, normalizes configuration, verifies
  /// the EE871 group/subgroup and CO2 capability, and atomically caches valid
  /// feature flags from 0x07..0x09. Reserved feature bits return NOT_SUPPORTED;
  /// identity, capability, or transfer failures retain their precise status.
  /// Failed initialization from UNINIT leaves an empty feature cache; calling
  /// begin() on an active session returns ALREADY_INITIALIZED without changes.
  /// Adjustment support at 0x03/0x04 is checked by the typed calibration APIs.
  /// Neither successful nor failed initialization clears persistent dirty state.
  /// The driver does not configure GPIO, pins, pull-ups, tasks, locks, or
  /// framework handles.
  /// @param config Configuration including E2 transport callbacks.
  /// @return Status::Ok() on success, error otherwise.
  Status begin(const Config& config);

  /// Record the latest application timestamp for diagnostics.
  ///
  /// The current driver performs synchronous bus operations in public API calls;
  /// tick() does not advance hidden asynchronous E2 transfers.
  /// @param nowMs Current timestamp in milliseconds.
  void tick(uint32_t nowMs);

  /// End the driver session and clear runtime/cache state.
  ///
  /// The core driver owns no GPIO or framework resources, so application-owned
  /// callback state remains the caller's responsibility. Persistent dirty
  /// diagnostics and their pending readback targets survive end(), subsequent
  /// begin() attempts, and recover() until resyncPersistentConfig() succeeds.
  void end();

  // =========================================================================
  // Diagnostics
  // =========================================================================

  /// Check if device is present on the bus.
  ///
  /// probe() validates the group, subgroup, and CO2 capability through raw
  /// diagnostic transfers. It does not update health counters or driver state.
  /// @return Status::Ok() if a compatible EE871 responds, error otherwise.
  Status probe();

  /// Attempt to recover from DEGRADED/OFFLINE state.
  ///
  /// Recovery performs a bounded raw bus reset, complete raw identity check,
  /// and atomic feature-cache refresh with reserved-bit validation, then
  /// commits that attempt to health once. A failed recovery clears all cached
  /// capabilities and latches OFFLINE
  /// even from READY/DEGRADED; only a fully compatible response restores READY.
  /// Recovery preserves persistent dirty diagnostics and pending readback
  /// targets; responsive transport does not verify uncertain writes.
  /// @return Status::Ok() if a compatible device is responsive, error otherwise.
  Status recover();

  /// Re-read persistent configuration and clear dirty diagnostics when coherent.
  ///
  /// This proves the persistent fields are readable and coherent by the
  /// driver's rules: a valid global interval, valid support byte 0x03,
  /// advertised CO2 offset/gain and part name, plus every register implicated
  /// in an uncertain write. Pending bus-address (0xC0), mode (0xD8), and
  /// auto-adjust status (0xD9) values receive their domain checks. Pending
  /// calibration or part-name targets require their advertised support.
  /// An uncertain auto-adjust request requires idle status before calibration
  /// readback and again when its pending target is visited; running returns BUSY.
  /// This cannot prove the requested values were applied or that an uncertain
  /// calibration request ran or succeeded; compare with an application-owned
  /// baseline. Dirty state clears only after the entire readback succeeds.
  /// Pending targets add at most 256 pointer/read pairs, up to four support
  /// byte 0x04 pointer/read pairs, and an initial auto-adjust status pair.
  /// Allow a maintenance-time budget using the configured frame bounds.
  ///
  /// This API touches the E2 bus, is blocking within configured timing/write
  /// delay bounds, is not ISR-safe, and uses tracked operations that can update
  /// health on transfer failure.
  /// While OFFLINE, returns the latched failure immediately; call recover() first.
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

  /// Total tracked failure count in the current session.
  /// @return Count since begin(); end() resets it, failed recover() adds one.
  uint32_t totalFailures() const { return _totalFailures; }

  /// Total tracked success count in the current session.
  /// @return Count since begin(); end() resets it, successful recover() adds one.
  uint32_t totalSuccess() const { return _totalSuccess; }

  /// Copy cached session retry counters and the latest NACK-bearing frame.
  /// @return Retry diagnostics snapshot; no bus access.
  ReadRetryDiagnostics readRetryDiagnostics() const { return _readRetry; }

  /// Consecutive failures required before OFFLINE.
  /// @return Normalized threshold currently in use.
  uint8_t offlineThreshold() const { return _config.offlineThreshold; }

  /// Check if a persistent write may have applied without verified completion.
  ///
  /// Single-byte writes can fail after acceptance; multi-byte writes are not
  /// bus-atomic. Once PEC transmission starts, interrupted signaling is
  /// conservatively uncertain unless a definite PEC NACK was sampled. Earlier
  /// rejection does not dirty that byte, but cannot undo earlier bytes in a
  /// multi-byte operation. Dirty state and all pending targets survive
  /// unrelated successes, end()/begin(), and recover(); only a successful
  /// resyncPersistentConfig() clears them.
  /// @return true when persistent configuration needs explicit resync/inspection.
  bool persistentConfigDirty() const { return _persistentConfigDirty; }

  /// First error that marked persistent configuration dirty.
  ///
  /// The original failing status is preserved so diagnostics can report the
  /// failure that created the dirty condition.
  /// @return Stored error status, or Status::Ok() when not dirty.
  Status persistentConfigDirtyError() const { return _persistentConfigDirtyError; }

  /// Copy the latest persistent operation without bus I/O or diagnostic/state changes.
  /// Core writes remain permitted while dirty for compatibility; applications own
  /// admission. This bounded record never replaces all-target dirty tracking.
  /// @return Latest outer mutation evidence, or operation NONE before any call.
  PersistentOperationResult getLastPersistentOperation() const { return _lastPersistentOperation; }

  // =========================================================================
  // E2 Protocol Helpers
  // =========================================================================

  /// Read a control-byte addressed value.
  /// Output may change on failure and must only be used after success.
  /// @param mainCommandNibble EE871-supported main-command nibble.
  /// @param[out] data Returned data byte.
  /// @return Status::Ok() on success; NOT_SUPPORTED for EE871-unsupported
  /// measurement commands. Only MV3/MV4/status control-byte NACKs are eligible
  /// for the explicitly configured read retries; defaults perform one attempt.
  Status readControlByte(uint8_t mainCommandNibble, uint8_t& data);

  /// Read a 16-bit value using low/high control bytes.
  ///
  /// For measurement command pairs, E2 v4.1 specifies that the low-byte read
  /// captures the associated high byte; this method preserves that order.
  /// @param mainCommandLow Low-byte main-command nibble.
  /// @param mainCommandHigh High-byte main-command nibble.
  /// @param[out] value Little-endian assembled value; unchanged on failure.
  /// @return Status::Ok() when both byte reads succeed.
  Status readU16(uint8_t mainCommandLow, uint8_t mainCommandHigh, uint16_t& value);

  /// Set internal custom pointer using command 0x50.
  ///
  /// This is a volatile pointer update, not a flash write; no write delay is
  /// applied and its STOP uses the ordinary bitTimeoutUs budget.
  /// @param address Custom-memory address, 0x00..0xFF; the transmitted high byte is zero.
  /// @return Status::Ok() after acknowledged pointer write and successful STOP;
  /// OUT_OF_RANGE for an address above 0xFF, before bus traffic.
  Status setCustomPointer(uint16_t address);

  /// Read one custom-memory byte.
  /// Raw access does not validate register support or value semantics; output
  /// may change on failure and must only be used after success.
  /// @param address Custom-memory address.
  /// @param[out] data Returned byte.
  /// @return Status::Ok() when pointer write and data read succeed.
  Status customRead(uint8_t address, uint8_t& data);

  /// Read a custom-memory block using pointer auto-increment.
  /// Raw access does not validate register support or value semantics. The
  /// buffer can be partially updated on failure; discard it unless all reads succeed.
  /// @param address First custom-memory address.
  /// @param[out] buf Destination buffer; must be non-null when len > 0.
  /// @param len Number of bytes to read; zero is rejected as INVALID_PARAM.
  /// @return Status::Ok() when all bytes are read, INVALID_PARAM for invalid
  /// buffer/zero length, or OUT_OF_RANGE if the read extends beyond 0xFF.
  Status customRead(uint8_t address, uint8_t* buf, size_t len);

  /// Write one custom-memory byte with command 0x10 and verify by readback.
  ///
  /// Addresses 0xC6 and 0xC7 are special: the driver reads the companion byte,
  /// assembles the complete interval, and routes the operation through
  /// writeMeasurementInterval(). This adds read traffic and uses the paired
  /// persistent-write timing/dirty-state rules.
  /// Other addresses are raw maintenance access: the caller owns register
  /// support and value validation. Any uncertain completion marks that address
  /// dirty; unrelated operations cannot clear it. Successful resync proves raw
  /// target readability, with domain checks only for registers documented there.
  /// @param address Custom-memory address.
  /// @param value Byte to write.
  /// @return Status::Ok() when readback matches, or a precise validation,
  /// capability, or transport status.
  Status customWrite(uint8_t address, uint8_t value);

  /// Write global measurement interval (0xC6/0xC7) and verify.
  /// @param intervalDeciSeconds Interval in 0.1 s units, from 150 through
  /// 36000 (15 through 3600 seconds).
  /// @return Status::Ok() when both interval bytes verify. Uncertain first-byte
  /// completion or any subsequent write/readback failure marks both bytes dirty;
  /// OUT_OF_RANGE is returned before capability checks or bus traffic.
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

  /// Read operating functions bitfield (0x07).
  /// Raw diagnostic read: does not validate reserved bits or refresh the cache
  /// used by has*() queries. begin()/recover() validate and install the cache.
  /// @param[out] bits Feature flags from custom memory 0x07.
  /// @return Status::Ok() when the byte is read.
  /// @see cmd::FEATURE_* constants for bit meanings
  Status readOperatingFunctions(uint8_t& bits);

  /// Read operating mode support bitfield (0x08).
  /// Raw diagnostic read: does not validate reserved bits or refresh the cache
  /// used by has*() queries. begin()/recover() validate and install the cache.
  /// @param[out] bits Operating-mode support flags from custom memory 0x08.
  /// @return Status::Ok() when the byte is read.
  /// @see cmd::MODE_SUPPORT_* constants
  Status readOperatingModeSupport(uint8_t& bits);

  /// Read special features bitfield (0x09).
  /// Raw diagnostic read: does not validate reserved bits or refresh the cache
  /// used by has*() queries. begin()/recover() validate and install the cache.
  /// @param[out] bits Special-feature flags from custom memory 0x09.
  /// @return Status::Ok() when the byte is read.
  /// @see cmd::SPECIAL_FEATURE_* constants
  Status readSpecialFeatures(uint8_t& bits);

  // =========================================================================
  // Feature Support Queries (use validated cache from begin()/recover())
  // =========================================================================
  //
  // A false result means "not present in the current cache." Before successful
  // begin() or after failed recover(), false does not prove the physical device
  // lacks that feature. Raw feature reads do not refresh this cache.

  /// Check if serial number is readable.
  /// @return true when cached feature flags advertise serial number support.
  bool hasSerialNumber() const { return (_operatingFunctions & cmd::FEATURE_SERIAL_NUMBER) != 0; }

  /// Check if part name is readable/writable.
  /// @return true when cached feature flags advertise part-name support.
  bool hasPartName() const { return (_operatingFunctions & cmd::FEATURE_PART_NAME) != 0; }

  /// Check if bus address is configurable.
  /// @return true when cached feature flags advertise address configuration.
  bool hasAddressConfig() const { return (_operatingFunctions & cmd::FEATURE_ADDRESS_CONFIG) != 0; }

  /// Check if global measurement interval is configurable.
  /// @return true when cached feature flags advertise global interval support.
  bool hasGlobalInterval() const { return (_operatingFunctions & cmd::FEATURE_GLOBAL_INTERVAL) != 0; }

  /// Check if specific (per-quantity) interval is configurable.
  /// @return true when cached feature flags advertise specific interval support.
  bool hasSpecificInterval() const { return (_operatingFunctions & cmd::FEATURE_SPECIFIC_INTERVAL) != 0; }

  /// Check if measurement filter is configurable.
  /// @return true when cached feature flags advertise filter configuration.
  bool hasFilterConfig() const { return (_operatingFunctions & cmd::FEATURE_FILTER_CONFIG) != 0; }

  /// Check if error code register exists.
  /// @return true when cached feature flags advertise error-code support.
  bool hasErrorCode() const { return (_operatingFunctions & cmd::FEATURE_ERROR_CODE) != 0; }

  /// Check if low power mode is supported.
  /// @return true when cached mode flags advertise low-power mode.
  bool hasLowPowerMode() const { return (_operatingModeSupport & cmd::MODE_SUPPORT_LOW_POWER) != 0; }

  /// Check if E2 priority mode is supported.
  /// @return true when cached mode flags advertise E2 priority mode.
  bool hasE2Priority() const { return (_operatingModeSupport & cmd::MODE_SUPPORT_E2_PRIORITY) != 0; }

  /// Check if auto adjustment is supported.
  /// @return true when cached special-feature flags advertise auto adjustment.
  bool hasAutoAdjust() const { return (_specialFeatures & cmd::SPECIAL_FEATURE_AUTO_ADJUST) != 0; }

  // =========================================================================
  // Identity Strings
  // =========================================================================

  /// Read 16-byte serial number (0xA0-0xAF).
  /// The buffer may be partially updated on failure; use it only after success.
  /// @param[out] buf Buffer of at least cmd::CUSTOM_SERIAL_LEN bytes; not NUL-terminated by the driver.
  /// @return Status::Ok() when all 16 bytes are read, INVALID_PARAM for null buffer.
  Status readSerialNumber(uint8_t* buf);

  /// Read 16-byte part name (0xB0-0xBF).
  /// The buffer may be partially updated on failure; use it only after success.
  /// @param[out] buf Buffer of at least cmd::CUSTOM_PART_NAME_LEN bytes; not NUL-terminated by the driver.
  /// @return Status::Ok() when all 16 bytes are read, INVALID_PARAM for null buffer.
  Status readPartName(uint8_t* buf);

  /// Write 16-byte part name (0xB0-0xBF).
  /// @param buf Buffer of exactly cmd::CUSTOM_PART_NAME_LEN bytes; embedded NUL bytes are written as data.
  /// @return Status::Ok() when all bytes verify. Uncertain first-byte completion
  /// or a later failure marks the whole part name dirty; null buffer returns INVALID_PARAM.
  Status writePartName(const uint8_t* buf);

  // =========================================================================
  // Bus Address
  // =========================================================================

  /// Read the stored bus address (0xC0) without validating its range.
  /// This does not change Config::deviceAddress or establish when a pending
  /// address change becomes active on the sensor.
  /// @param[out] address Raw stored E2 device address; valid only after success.
  /// @return Status::Ok() when the byte is read.
  Status readBusAddress(uint8_t& address);

  /// Write bus address (0xC0).
  /// This updates persistent sensor configuration and does not retarget the
  /// current driver session. Do not assume when the device activates the new
  /// address; a failed verification may require explicit retargeting and resync.
  /// @param address New address (0-7)
  /// @return OUT_OF_RANGE before capability checks or bus traffic if address
  /// is greater than 7; otherwise a capability or write/readback status.
  Status writeBusAddress(uint8_t address);

  // =========================================================================
  // Measurement Interval
  // =========================================================================

  /// Read global measurement interval without enforcing its configured range.
  /// Fixed intervals remain readable without advertised configuration support.
  /// @param[out] intervalDeciSeconds Interval in 0.1 s units; unchanged on failure.
  /// @return Status::Ok() when both bytes are read.
  Status readMeasurementInterval(uint16_t& intervalDeciSeconds);

  /// Read CO2-specific interval factor (0xCB)
  /// Positive = multiplier, Negative = divider
  /// @param[out] factor Signed interval factor; unchanged on failure.
  /// @return Status::Ok() when the byte is read.
  Status readCo2IntervalFactor(int8_t& factor);

  /// Write CO2-specific interval factor (0xCB).
  /// @param factor Signed interval factor.
  /// @return Status::Ok() when the byte verifies. This is a persistent single-byte write.
  Status writeCo2IntervalFactor(int8_t factor);

  // =========================================================================
  // Filter / Operating Mode
  // =========================================================================

  /// Read CO2 filter setting (0xD3).
  /// Fixed filter settings remain readable without advertised configuration support.
  /// @param[out] filter Filter setting byte.
  /// @return Status::Ok() when the byte is read.
  Status readCo2Filter(uint8_t& filter);

  /// Write CO2 filter setting (0xD3).
  /// @param filter Filter setting byte.
  /// @return Status::Ok() when the byte verifies. This is a persistent single-byte write.
  Status writeCo2Filter(uint8_t filter);

  /// Read operating mode (0xD8).
  ///
  /// Fails closed with NOT_SUPPORTED when neither operating-mode capability is
  /// advertised. Reserved bits in a returned value produce OUT_OF_RANGE;
  /// an active mode bit whose capability is absent produces NOT_SUPPORTED.
  /// @param[out] mode Operating-mode byte; unchanged on failure.
  /// @return Status::Ok() only when a supported, valid mode byte is read.
  /// @see cmd::OPERATING_MODE_* constants
  Status readOperatingMode(uint8_t& mode);

  /// Write operating mode (0xD8).
  ///
  /// bit0: 0=freerunning, 1=low power. bit1: 0=measurement priority,
  /// 1=E2 priority.
  /// Fails closed with NOT_SUPPORTED when neither operating-mode capability is
  /// advertised, including for mode 0.
  /// Each active bit also requires its corresponding advertised capability.
  /// @param mode Operating-mode byte.
  /// @return Status::Ok() when the byte verifies. This is a persistent single-byte write.
  Status writeOperatingMode(uint8_t mode);

  // =========================================================================
  // Auto Adjustment
  // =========================================================================

  /// Check if auto adjustment is running (0xD9 bit0).
  /// Requires advertised auto-adjust support; reserved result bits return
  /// OUT_OF_RANGE. The output is unchanged on failure.
  /// @param[out] running true when auto adjustment is running.
  /// @return Status::Ok() when the byte is read.
  Status readAutoAdjustStatus(bool& running);

  /// Start auto adjustment (cannot be stopped once started).
  ///
  /// Reads and validates status first; BUSY prevents retriggering a running
  /// adjustment. The sensor retains previous measured values during adjustment.
  /// This is a bench/maintenance operation unless the application explicitly
  /// owns the calibration workflow. A failed request may still have started it;
  /// inspect dirty diagnostics and status before deciding what to do next.
  /// @return Status::Ok() when the start register verifies, not when calibration completes.
  Status startAutoAdjust();

  // =========================================================================
  // Calibration (Advanced)
  // =========================================================================

  /// Read CO2 offset (signed, ppm).
  /// Every call first reads support byte 0x03 with a pointer update and one
  /// byte read. Missing CO2 bit3 or nonzero reserved high bits return
  /// NOT_SUPPORTED; support-read transfer failures retain their precise status.
  /// @param[out] offset Signed offset in ppm; unchanged on failure.
  /// @return Status::Ok() when both bytes are read.
  Status readCo2Offset(int16_t& offset);

  /// Write CO2 offset (signed, ppm).
  /// Maintenance operation. Every call first reads support byte 0x03 with a
  /// pointer update and one byte read; unsupported or malformed flags return
  /// NOT_SUPPORTED before any persistent write. Transfer errors are preserved.
  /// @param offset Signed offset in ppm.
  /// @return Status::Ok() when both bytes verify. Uncertain low-byte completion
  /// or a later write/readback failure marks both offset bytes dirty.
  Status writeCo2Offset(int16_t offset);

  /// Read CO2 gain (gain = value / 32768).
  /// Every call first reads support byte 0x03 with a pointer update and one
  /// byte read. Missing CO2 bit3 or nonzero reserved high bits return
  /// NOT_SUPPORTED; support-read transfer failures retain their precise status.
  /// @param[out] gain Raw gain value; unchanged on failure.
  /// @return Status::Ok() when both bytes are read.
  Status readCo2Gain(uint16_t& gain);

  /// Write CO2 gain (gain = value / 32768).
  /// Maintenance operation. Every call first reads support byte 0x03 with a
  /// pointer update and one byte read; unsupported or malformed flags return
  /// NOT_SUPPORTED before any persistent write. Transfer errors are preserved.
  /// @param gain Raw gain value.
  /// @return Status::Ok() when both bytes verify. Uncertain low-byte completion
  /// or a later write/readback failure marks both gain bytes dirty.
  Status writeCo2Gain(uint16_t gain);

  /// Read last calibration points.
  /// Every call first reads support byte 0x04 with a pointer update and one
  /// byte read. Missing CO2 bit3 or nonzero reserved high bits return
  /// NOT_SUPPORTED; support-read transfer failures retain their precise status.
  /// @param[out] lower Lower calibration point in ppm; unchanged on failure.
  /// @param[out] upper Upper calibration point in ppm; unchanged on failure.
  /// @return Status::Ok() when both 16-bit values are read.
  Status readCo2CalPoints(uint16_t& lower, uint16_t& upper);

  // =========================================================================
  // Status / Measurements
  // =========================================================================

  /// Read status byte.
  ///
  /// This operation can trigger a measurement and reset its interval counter
  /// when the global interval is >15 s and the prior value is >10 s old
  /// (AN1611-1 sections 4 and 10). For checked sampling, read MV3/MV4 first
  /// and status second so status describes the last measured value.
  /// @param[out] status Raw status byte; valid only after success.
  /// @return Status::Ok() when the status byte and PEC verify. Control-byte
  /// NACK retries follow Config::readNackRetries (disabled by default).
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
  ///
  /// This is a raw value API. It does not read status, reject the CO2 error
  /// bit, validate warm-up/freshness, or enforce a product-specific ppm range.
  /// Reading MV3 low first captures the associated high byte in the sensor.
  /// Control-byte NACK retries follow Config::readNackRetries (default off),
  /// without inferring the sensor's internal reason.
  /// @param[out] ppm CO2 concentration in ppm; unchanged on failure.
  /// @return Status::Ok() when MV3 low/high reads succeed.
  Status readCo2Fast(uint16_t& ppm);

  /// Read CO2 averaged value from MV4.
  ///
  /// This is a raw value API. It does not read status, reject the CO2 error
  /// bit, validate warm-up/freshness, or enforce a product-specific ppm range.
  /// Reading MV4 low first captures the associated high byte in the sensor.
  /// Control-byte NACK retries follow Config::readNackRetries (default off),
  /// without inferring the sensor's internal reason.
  /// @param[out] ppm CO2 concentration in ppm; unchanged on failure.
  /// @return Status::Ok() when MV4 low/high reads succeed.
  Status readCo2Average(uint16_t& ppm);

  /// Read a checked fast-response MV3 sample, explicitly including sensor status.
  ///
  /// Reads the raw value low byte then high byte through readCo2Fast(), followed
  /// by readStatus(). A failed value read short-circuits without reading status.
  /// When status bit3 is set and hasErrorCode() is true, reads error detail via
  /// readErrorCode(); unknown codes remain visible and are still sensor errors.
  /// The status read can trigger the next measurement and reset its interval
  /// counter only when the global interval is >15 s and the previous value is
  /// >10 s old (AN1611-1 sections 4 and 10). Value-before-status ordering does
  /// not prove a fresh conversion. No warm-up, freshness, accuracy or ppm range
  /// validation is performed; applications own cadence and validity policy.
  ///
  /// Uses existing bounded tracked transfers and their configured narrow NACK
  /// retries; adds no aggregate health event or helper-level replay. A sensor
  /// error alone is independent of transport health. Blocking, task-context
  /// only; externally serialize access as for all other E2 operations.
  /// @param[out] out Replaced with per-step evidence, including on failure.
  /// @return Exact first failing transfer/precondition Status, or
  /// CO2_SENSOR_ERROR for status bit3 after successful requested transfers
  /// (detail is the error code when valid, otherwise the status byte), or OK
  /// with ppmValid true. Error-detail failure takes return precedence while
  /// out.co2Error retains the sensor failure separately.
  Status readCo2FastSample(Co2ReadResult& out);

  /// Read a checked averaged MV4 sample, explicitly including sensor status.
  ///
  /// Uses readCo2Average() for the raw value, then the same procedure, evidence,
  /// conditional status-trigger side effect, validity limits and error/health
  /// semantics as readCo2FastSample(). A clean result accepts the full uint16_t
  /// word range; product-specific range checks belong to the caller.
  /// @param[out] out Replaced with per-step evidence, including on failure.
  /// @return Exact first failing transfer/precondition Status, CO2_SENSOR_ERROR,
  /// or OK with ppmValid true. See readCo2FastSample() for detail semantics.
  Status readCo2AverageSample(Co2ReadResult& out);

  // =========================================================================
  // Bus Safety
  // =========================================================================

  /// Reset bus state by clocking with SDA high.
  ///
  /// Use after timeout/stuck bus conditions. This touches E2 lines, is blocking
  /// within configured timing bounds, and is not ISR-safe.
  /// Health-neutral; does not clear a latched OFFLINE state.
  /// @return Ok if bus lines are free after reset.
  Status busReset();

  /// Check if bus lines are idle (both high).
  ///
  /// This reads the configured line callbacks and does not issue an E2 transfer.
  /// It is still not ISR-safe unless the application proves its callbacks are
  /// ISR-safe.
  /// @return Ok if idle, BUS_STUCK if either line is low.
  Status checkBusIdle();

private:
  Status _readCo2Sample(bool fast, Co2ReadResult& out);

  // =========================================================================
  // Tracked/Raw Transport Wrappers
  // =========================================================================

  struct ReadAttemptInfo {
    bool controlNack = false;
    Status stopStatus = Status::Ok();
  };
  Status _readControlByteRaw(uint8_t controlByte, uint8_t& data,
                             ReadAttemptInfo* attempt = nullptr);
  Status _readControlByteTracked(uint8_t controlByte, uint8_t& data);

  Status _writeCommandRaw(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte,
                          PersistentEffect* effect = nullptr);
  Status _writeCommandTracked(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte);
  Status _customWriteDirect(uint8_t address, uint8_t value);
  Status _writeMeasurementInterval(uint16_t intervalDeciSeconds);
  Status _writeCo2AdjustmentPair(uint8_t lowAddress, uint16_t value);
  Status _readPersistentBack(uint8_t address, uint8_t* data, uint8_t length);
  void _beginPersistentOperation(PersistentOperation operation, uint8_t address,
                                  const uint8_t* requested, uint8_t length);
  Status _finishPersistentOperation(const Status& st);
  Status _busResetRaw();
  Status _validateIdentityRaw();
  Status _requireCo2AdjustmentSupport(uint8_t address);
  Status _readFeatureFlagsRaw(uint8_t& operatingFunctions,
                              uint8_t& operatingModeSupport,
                              uint8_t& specialFeatures);
  Status _recoverTracked();
  Status _offlineStatus() const;

  // =========================================================================
  // Health Management
  // =========================================================================

  /// Update health counters and state based on operation result
  /// Called only from the two tracked transfer wrappers and _recoverTracked().
  Status _updateHealth(const Status& st);

  void _resetStoppedState();
  void _markPersistentConfigDirty(const Status& st, uint8_t address, uint8_t length = 1);
  void _clearPersistentConfigDirty();

  // =========================================================================
  // State
  // =========================================================================

  Config _config;
  bool _initialized = false;
  DriverState _driverState = DriverState::UNINIT;
  uint32_t _nowMs = 0;

  // Feature flags (validated and cached during begin()/recover())
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
  ReadRetryDiagnostics _readRetry;
  PersistentOperationResult _lastPersistentOperation;
  bool _persistentConfigDirty = false;
  Status _persistentConfigDirtyError = Status::Ok();
  // One bit per custom address; retained with dirty diagnostics across sessions.
  uint8_t _pendingPersistentReads[cmd::CUSTOM_MEMORY_SIZE / 8] = {};
};

} // namespace EE871
