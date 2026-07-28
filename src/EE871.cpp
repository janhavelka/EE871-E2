/// @file EE871.cpp
/// @brief Implementation of the EE871 E2 driver

#include "EE871/EE871.h"

#include <limits>

namespace EE871 {
namespace {

static constexpr uint32_t kPollStepUs = 5;
static constexpr uint32_t kDataSetupUs = 10;

inline void setScl(const Config& cfg, bool level) {
  cfg.setScl(level, cfg.busUser);
}

inline void setSda(const Config& cfg, bool level) {
  cfg.setSda(level, cfg.busUser);
}

inline bool readScl(const Config& cfg) {
  return cfg.readScl(cfg.busUser);
}

inline bool readSda(const Config& cfg) {
  return cfg.readSda(cfg.busUser);
}

uint32_t saturatingAddU32(uint32_t lhs, uint32_t rhs) {
  const uint32_t room = std::numeric_limits<uint32_t>::max() - lhs;
  return (rhs > room) ? std::numeric_limits<uint32_t>::max() : (lhs + rhs);
}

bool checkedAddU64(uint64_t& value, uint64_t addend) {
  if (addend > std::numeric_limits<uint64_t>::max() - value) {
    return false;
  }
  value += addend;
  return true;
}

Status addScaledU64(uint64_t& value, uint64_t factor, uint64_t count) {
  if (factor != 0 && count > std::numeric_limits<uint64_t>::max() / factor) {
    return Status::Error(Err::OUT_OF_RANGE, "Timing bound overflow");
  }
  if (!checkedAddU64(value, factor * count)) {
    return Status::Error(Err::OUT_OF_RANGE, "Timing bound overflow");
  }
  return Status::Ok();
}

uint64_t startBoundUs(const Config& cfg) {
  return static_cast<uint64_t>(cfg.bitTimeoutUs) +
         (2ULL * cfg.startHoldUs) + cfg.clockLowUs;
}

uint64_t stopBoundUs(const Config& cfg) {
  return static_cast<uint64_t>(kDataSetupUs) + cfg.bitTimeoutUs +
         (2ULL * cfg.stopHoldUs);
}

uint64_t readTransactionBoundUs(const Config& cfg) {
  return startBoundUs(cfg) + (3ULL * cfg.byteTimeoutUs) + stopBoundUs(cfg);
}

uint64_t normalWriteTransactionBoundUs(const Config& cfg) {
  return startBoundUs(cfg) + (4ULL * cfg.byteTimeoutUs) + stopBoundUs(cfg);
}

uint64_t completionWriteTransactionBoundUs(
    const Config& cfg, uint32_t completionMs) {
  return startBoundUs(cfg) + (4ULL * cfg.byteTimeoutUs) +
         (static_cast<uint64_t>(completionMs) * 1000ULL);
}

uint64_t busResetBoundUs(const Config& cfg) {
  const uint64_t pulse =
      static_cast<uint64_t>(cfg.clockLowUs) + cfg.bitTimeoutUs + cfg.clockHighUs;
  return (static_cast<uint64_t>(cmd::BUS_RESET_CLOCKS) * pulse) +
         cfg.clockLowUs + kDataSetupUs + cfg.bitTimeoutUs +
         (2ULL * cfg.stopHoldUs);
}

static uint8_t calcPecRead(uint8_t controlByte, uint8_t dataByte) {
  return static_cast<uint8_t>((controlByte + dataByte) & 0xFF);
}

static uint8_t calcPecWrite(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte) {
  return static_cast<uint8_t>((controlByte + addressByte + dataByte) & 0xFF);
}

constexpr Co2SensorError co2SensorErrorFromCode(uint8_t code) {
  switch (code) {
    case cmd::CO2_ERROR_SUPPLY_VOLTAGE_LOW:
      return Co2SensorError::SUPPLY_VOLTAGE_LOW;
    case cmd::CO2_ERROR_SENSOR_COUNTS_LOW:
      return Co2SensorError::SENSOR_COUNTS_LOW;
    case cmd::CO2_ERROR_SENSOR_COUNTS_HIGH:
      return Co2SensorError::SENSOR_COUNTS_HIGH;
    case cmd::CO2_ERROR_SUPPLY_VOLTAGE_BREAKDOWN:
      return Co2SensorError::SUPPLY_VOLTAGE_BREAKDOWN_AT_PEAK;
    default:
      return Co2SensorError::UNKNOWN;
  }
}

} // namespace

void EE871::_delayUs(
    const Config& config, uint32_t us, ByteDeadline* deadline) {
  config.delayUs(us, config.busUser);
  if (deadline != nullptr) {
    deadline->elapsedUs = saturatingAddU32(deadline->elapsedUs, us);
  }
}

Status EE871::_delayWithinDeadline(
    const Config& config,
    uint32_t us,
    ClockWaitClass waitClass,
    ByteDeadline& deadline) {
  if (deadline.elapsedUs > deadline.limitUs ||
      us > deadline.limitUs - deadline.elapsedUs) {
    return Status::Error(
        Err::TIMEOUT,
        waitClass == ClockWaitClass::NORMAL_BIT
            ? "Byte timeout"
            : (waitClass == ClockWaitClass::INTERVAL_COMMIT
                   ? "Interval commit timeout"
                   : "Write completion timeout"),
        static_cast<int32_t>(deadline.elapsedUs));
  }
  _delayUs(config, us, &deadline);
  return Status::Ok();
}

void EE871::_delayLongMs(const Config& config, uint32_t totalMs) {
  uint32_t remainingMs = totalMs;
  while (remainingMs != 0U) {
    const uint32_t slice =
        (remainingMs > config.longDelaySliceMs)
            ? config.longDelaySliceMs
            : remainingMs;
    if (config.delayMs != nullptr) {
      config.delayMs(slice, config.busUser);
    } else {
      config.delayUs(slice * 1000U, config.busUser);
    }
    if (config.yield != nullptr) {
      (config.yield)(config.busUser);
    }
    remainingMs -= slice;
  }
}

Status EE871::_waitSclHigh(
    const Config& config,
    ClockWaitClass waitClass,
    ByteDeadline& deadline) {
  uint32_t waitedUs = 0;
  const uint32_t bitLimit =
      (waitClass == ClockWaitClass::NORMAL_BIT)
          ? config.bitTimeoutUs
          : deadline.limitUs;

  while (!readScl(config)) {
    if (waitedUs > bitLimit || kPollStepUs > bitLimit - waitedUs) {
      return Status::Error(
          Err::TIMEOUT,
          waitClass == ClockWaitClass::NORMAL_BIT
              ? "Clock stretch timeout"
              : (waitClass == ClockWaitClass::INTERVAL_COMMIT
                     ? "Interval commit timeout"
                     : "Write completion timeout"),
          static_cast<int32_t>(waitedUs));
    }
    if (deadline.elapsedUs > deadline.limitUs ||
        kPollStepUs > deadline.limitUs - deadline.elapsedUs) {
      return Status::Error(
          Err::TIMEOUT,
          waitClass == ClockWaitClass::NORMAL_BIT
              ? "Byte timeout"
              : (waitClass == ClockWaitClass::INTERVAL_COMMIT
                     ? "Interval commit timeout"
                     : "Write completion timeout"),
          static_cast<int32_t>(deadline.elapsedUs));
    }
    _delayUs(config, kPollStepUs, &deadline);
    waitedUs += kPollStepUs;
  }
  return Status::Ok();
}

Status EE871::_e2Start(const Config& config) {
  setSda(config, true);
  setScl(config, true);
  ByteDeadline idleWait{0, config.bitTimeoutUs};
  Status st = _waitSclHigh(
      config, ClockWaitClass::NORMAL_BIT, idleWait);
  if (!st.ok()) {
    return Status::Error(
        Err::BUS_STUCK, "SCL low before START", st.detail);
  }
  if (!readSda(config)) {
    return Status::Error(Err::BUS_STUCK, "SDA low before START");
  }
  _delayUs(config, config.startHoldUs);
  setSda(config, false);
  _delayUs(config, config.startHoldUs);
  setScl(config, false);
  _delayUs(config, config.clockLowUs);
  return Status::Ok();
}

Status EE871::_e2Stop(
    const Config& config,
    ClockWaitClass waitClass,
    ByteDeadline* suppliedDeadline) {
  if (waitClass == ClockWaitClass::NORMAL_BIT) {
    setSda(config, false);
    _delayUs(config, kDataSetupUs);
    setScl(config, true);
    ByteDeadline sclDeadline{0, config.bitTimeoutUs};
    Status st = _waitSclHigh(
        config, ClockWaitClass::NORMAL_BIT, sclDeadline);
    if (!st.ok()) {
      return st;
    }
    _delayUs(config, config.stopHoldUs);
    setSda(config, true);
    _delayUs(config, config.stopHoldUs);
    return Status::Ok();
  }

  ByteDeadline normalDeadline{0, config.bitTimeoutUs};
  ByteDeadline& deadline =
      suppliedDeadline != nullptr ? *suppliedDeadline : normalDeadline;

  setSda(config, false);
  Status st = _delayWithinDeadline(
      config, kDataSetupUs, waitClass, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = _waitSclHigh(config, waitClass, deadline);
  if (!st.ok()) {
    return st;
  }
  st = _delayWithinDeadline(
      config, config.stopHoldUs, waitClass, deadline);
  if (!st.ok()) {
    return st;
  }
  setSda(config, true);
  return _delayWithinDeadline(
      config, config.stopHoldUs, waitClass, deadline);
}

Status EE871::_writeBit(
    const Config& config, bool bit, ByteDeadline& deadline) {
  setSda(config, bit);
  Status st = _delayWithinDeadline(
      config, kDataSetupUs, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = _waitSclHigh(
      config, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  st = _delayWithinDeadline(
      config, config.clockHighUs, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, false);
  return _delayWithinDeadline(
      config, config.clockLowUs, ClockWaitClass::NORMAL_BIT, deadline);
}

Status EE871::_readBit(
    const Config& config, bool& bit, ByteDeadline& deadline) {
  setSda(config, true);
  Status st = _delayWithinDeadline(
      config, kDataSetupUs, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = _waitSclHigh(
      config, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  const uint32_t sampleDelay = config.clockHighUs / 2U;
  st = _delayWithinDeadline(
      config, sampleDelay, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  bit = readSda(config);
  st = _delayWithinDeadline(
      config,
      config.clockHighUs - sampleDelay,
      ClockWaitClass::NORMAL_BIT,
      deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, false);
  return _delayWithinDeadline(
      config, config.clockLowUs, ClockWaitClass::NORMAL_BIT, deadline);
}

Status EE871::_writeByte(
    const Config& config, uint8_t value, ByteDeadline& deadline) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    Status st = _writeBit(config, (value & mask) != 0, deadline);
    if (!st.ok()) {
      return st;
    }
  }
  return Status::Ok();
}

Status EE871::_readByte(
    const Config& config, uint8_t& value, ByteDeadline& deadline) {
  value = 0;
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    bool bit = false;
    Status st = _readBit(config, bit, deadline);
    if (!st.ok()) {
      return st;
    }
    if (bit) {
      value |= mask;
    }
  }
  return Status::Ok();
}

Status EE871::_readAck(
    const Config& config,
    bool& acked,
    ClockWaitClass waitClass,
    ByteDeadline& deadline) {
  setSda(config, true);
  Status st = _delayWithinDeadline(
      config, kDataSetupUs, waitClass, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = _waitSclHigh(config, waitClass, deadline);
  if (!st.ok()) {
    return st;
  }
  const uint32_t sampleDelay = config.clockHighUs / 2U;
  st = _delayWithinDeadline(config, sampleDelay, waitClass, deadline);
  if (!st.ok()) {
    return st;
  }
  acked = !readSda(config);
  st = _delayWithinDeadline(
      config, config.clockHighUs - sampleDelay, waitClass, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, false);
  return _delayWithinDeadline(
      config, config.clockLowUs, waitClass, deadline);
}

Status EE871::_sendAck(
    const Config& config, bool ack, ByteDeadline& deadline) {
  setSda(config, !ack);
  Status st = _delayWithinDeadline(
      config, kDataSetupUs, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = _waitSclHigh(
      config, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  st = _delayWithinDeadline(
      config, config.clockHighUs, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, false);
  st = _delayWithinDeadline(
      config, config.clockLowUs, ClockWaitClass::NORMAL_BIT, deadline);
  if (!st.ok()) {
    return st;
  }
  setSda(config, true);
  return Status::Ok();
}

Status EE871::_validateConfig(const Config& input, Config& normalized) {
  if (input.setScl == nullptr || input.setSda == nullptr ||
      input.readScl == nullptr || input.readSda == nullptr ||
      input.delayUs == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "Missing E2 callbacks");
  }
  if (input.deviceAddress > cmd::DEVICE_ADDRESS_MAX) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid device address");
  }
  if (input.clockLowUs < cmd::CLOCK_LOW_MIN_US ||
      input.clockHighUs < cmd::CLOCK_HIGH_MIN_US) {
    return Status::Error(Err::INVALID_CONFIG, "Clock timing below spec");
  }
  const uint64_t effectivePeriodUs =
      static_cast<uint64_t>(input.clockLowUs) + input.clockHighUs +
      kDataSetupUs;
  if (effectivePeriodUs > cmd::CLOCK_PERIOD_MAX_US) {
    return Status::Error(
        Err::INVALID_CONFIG, "Effective clock period exceeds 2000 us");
  }
  if (input.startHoldUs < 4U || input.stopHoldUs < 4U) {
    return Status::Error(Err::INVALID_CONFIG, "Start/stop hold below spec");
  }
  if (input.bitTimeoutUs == 0U ||
      input.bitTimeoutUs > cmd::BIT_TIMEOUT_MAX_US) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid bit timeout");
  }
  if (input.byteTimeoutUs < input.bitTimeoutUs ||
      input.byteTimeoutUs > cmd::BYTE_TIMEOUT_MAX_US) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid byte timeout");
  }
  if (input.writeDelayMs > cmd::WRITE_DELAY_MAX_MS) {
    return Status::Error(Err::INVALID_CONFIG, "writeDelayMs exceeds safe limit");
  }
  if (input.intervalWriteDelayMs > cmd::INTERVAL_WRITE_DELAY_MAX_MS) {
    return Status::Error(
        Err::INVALID_CONFIG, "intervalWriteDelayMs exceeds safe limit");
  }
  if (input.longDelaySliceMs > cmd::LONG_DELAY_SLICE_MAX_MS) {
    return Status::Error(
        Err::INVALID_CONFIG, "longDelaySliceMs exceeds safe limit");
  }
  const uint8_t beginPolicy = static_cast<uint8_t>(input.beginPolicy);
  if (beginPolicy >
      static_cast<uint8_t>(BeginPolicy::ALLOW_ABSENT)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid begin policy");
  }

  normalized = input;
  if (normalized.writeDelayMs < cmd::WRITE_DELAY_PROTOCOL_MIN_MS) {
    normalized.writeDelayMs = cmd::WRITE_DELAY_PROTOCOL_MIN_MS;
  }
  if (normalized.intervalWriteDelayMs <
      cmd::INTERVAL_WRITE_DELAY_PROTOCOL_MIN_MS) {
    normalized.intervalWriteDelayMs =
        cmd::INTERVAL_WRITE_DELAY_PROTOCOL_MIN_MS;
  }
  if (normalized.longDelaySliceMs == 0U) {
    normalized.longDelaySliceMs = 1U;
  }
  if (normalized.offlineThreshold == 0U) {
    normalized.offlineThreshold = 1U;
  }
  return Status::Ok();
}

Status EE871::_calculateOperationTimingBound(
    const Config& normalized,
    OperationKind kind,
    uint16_t elementCount,
    OperationTimingBound& out) {
  const bool blockRead = kind == OperationKind::CUSTOM_BLOCK_READ;
  if ((!blockRead && elementCount != 1U) ||
      (blockRead && (elementCount == 0U || elementCount > 256U))) {
    return Status::Error(
        Err::INVALID_PARAM, "Invalid timing-bound element count", elementCount);
  }

  const uint64_t readUs = readTransactionBoundUs(normalized);
  const uint64_t pointerUs = completionWriteTransactionBoundUs(
      normalized, normalized.writeDelayMs);
  const uint64_t customWriteUs = completionWriteTransactionBoundUs(
      normalized, normalized.writeDelayMs);
  uint64_t totalUs = 0;
  Status st = Status::Ok();

  switch (kind) {
    case OperationKind::CONTROL_READ:
      totalUs = readUs;
      break;
    case OperationKind::CUSTOM_POINTER_WRITE:
      totalUs = pointerUs;
      break;
    case OperationKind::CUSTOM_BYTE_READ:
      totalUs = pointerUs;
      st = addScaledU64(totalUs, readUs, 1U);
      break;
    case OperationKind::CUSTOM_BLOCK_READ:
      totalUs = pointerUs;
      st = addScaledU64(totalUs, readUs, elementCount);
      break;
    case OperationKind::CUSTOM_BYTE_WRITE_VERIFY:
      totalUs = customWriteUs;
      st = addScaledU64(totalUs, pointerUs, 1U);
      if (st.ok()) {
        st = addScaledU64(totalUs, readUs, 1U);
      }
      break;
    case OperationKind::INTERVAL_WRITE_VERIFY:
      totalUs = normalWriteTransactionBoundUs(normalized);
      st = addScaledU64(
          totalUs,
          completionWriteTransactionBoundUs(
              normalized, normalized.intervalWriteDelayMs),
          1U);
      if (st.ok()) {
        st = addScaledU64(totalUs, pointerUs, 1U);
      }
      if (st.ok()) {
        st = addScaledU64(totalUs, readUs, 2U);
      }
      break;
    case OperationKind::PART_NAME_WRITE_VERIFY: {
      uint64_t oneElementUs = customWriteUs;
      st = addScaledU64(oneElementUs, pointerUs, 1U);
      if (st.ok()) {
        st = addScaledU64(oneElementUs, readUs, 1U);
      }
      if (st.ok()) {
        st = addScaledU64(
            totalUs, oneElementUs, cmd::CUSTOM_PART_NAME_LEN);
      }
      break;
    }
    case OperationKind::RAW_CO2_READ:
      st = addScaledU64(totalUs, readUs, 2U);
      break;
    case OperationKind::BUS_RESET:
      totalUs = busResetBoundUs(normalized);
      break;
    case OperationKind::BEGIN_REQUIRE_PRESENT:
    case OperationKind::BEGIN_ALLOW_ABSENT:
    case OperationKind::RECOVER_IDENTITY_AND_CAPABILITIES:
      totalUs = busResetBoundUs(normalized);
      st = addScaledU64(totalUs, readUs, 4U);
      if (st.ok()) {
        st = addScaledU64(totalUs, pointerUs, 1U);
      }
      if (st.ok()) {
        st = addScaledU64(totalUs, readUs, 7U);
      }
      break;
    case OperationKind::PROBE_IDENTITY:
      st = addScaledU64(totalUs, readUs, 4U);
      break;
    case OperationKind::CHECKED_CO2_AVERAGE:
    case OperationKind::CHECKED_CO2_FAST:
      totalUs = pointerUs;
      st = addScaledU64(totalUs, readUs, 4U);
      break;
    default:
      return Status::Error(
          Err::INVALID_PARAM, "Invalid operation kind");
  }
  if (!st.ok()) {
    return st;
  }

  if (!checkedAddU64(totalUs, 999ULL)) {
    return Status::Error(Err::OUT_OF_RANGE, "Timing bound overflow");
  }
  const uint64_t maxBlockingMs = totalUs / 1000ULL;
  if (maxBlockingMs > std::numeric_limits<uint32_t>::max()) {
    return Status::Error(Err::OUT_OF_RANGE, "Timing bound exceeds uint32_t");
  }

  OperationTimingBound candidate;
  candidate.kind = kind;
  candidate.elementCount = elementCount;
  candidate.maxBlockingMs = static_cast<uint32_t>(maxBlockingMs);
  out = candidate;
  return Status::Ok();
}

Status EE871::operationTimingBound(
    const Config& config,
    OperationKind kind,
    uint16_t elementCount,
    OperationTimingBound& out) {
  Config normalized;
  Status st = _validateConfig(config, normalized);
  if (!st.ok()) {
    return st;
  }
  return _calculateOperationTimingBound(
      normalized, kind, elementCount, out);
}

Status EE871::operationTimingBound(
    OperationKind kind,
    uint16_t elementCount,
    OperationTimingBound& out) const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  return _calculateOperationTimingBound(
      _config, kind, elementCount, out);
}

Status EE871::begin(const Config& config) {
  // Prevent double-init without explicit end()
  if (_initialized) {
    return Status::Error(Err::ALREADY_INITIALIZED, "Call end() first");
  }

  _resetStoppedState();

  Config normalized;
  Status st = _validateConfig(config, normalized);
  if (!st.ok()) {
    return st;
  }
  _config = normalized;

  // Require an idle bus or one successful bounded raw reset.
  if (!readScl(_config) || !readSda(_config)) {
    st = _busResetRaw();
    if (!st.ok()) {
      _resetStoppedState();
      return st;
    }
  }

  DeviceIdentity identityCandidate;
  bool identityNackTerminatedCleanly = false;
  st = _readAndValidateIdentityRaw(
      identityCandidate, &identityNackTerminatedCleanly);
  if (!st.ok()) {
    const bool acceptedAbsence =
        _config.beginPolicy == BeginPolicy::ALLOW_ABSENT &&
        ((st.code == Err::NACK && identityNackTerminatedCleanly) ||
         st.code == Err::DEVICE_NOT_FOUND);
    if (acceptedAbsence) {
      _initialized = true;
      _beginProbeStatus = st;
      _latchSemanticOffline(st);
      return Status::Ok();
    }
    _resetStoppedState();
    return st;
  }

  CapabilitySnapshot capabilityCandidate;
  st = _readCapabilitiesRaw(capabilityCandidate);
  if (!st.ok()) {
    _resetStoppedState();
    return st;
  }

  _publishIdentityAndCapabilities(
      identityCandidate, capabilityCandidate);
  _initialized = true;
  _driverState = DriverState::READY;
  _beginProbeStatus = Status::Ok();
  return Status::Ok();
}

void EE871::tick(uint32_t nowMs) {
  _nowMs = nowMs;
}

void EE871::end() {
  _resetStoppedState();
}

Status EE871::getSettings(SettingsSnapshot& out) const {
  out.config = _config;
  out.state = _driverState;
  out.initialized = _initialized;
  out.nowMs = _nowMs;
  out.operatingFunctions = _operatingFunctions;
  out.operatingModeSupport = _operatingModeSupport;
  out.specialFeatures = _specialFeatures;
  out.lastOkMs = _lastOkMs;
  out.lastErrorMs = _lastErrorMs;
  out.lastError = _lastError;
  out.consecutiveFailures = _consecutiveFailures;
  out.totalFailures = _totalFailures;
  out.totalSuccess = _totalSuccess;
  out.persistentConfigDirty = _persistentConfigDirty;
  out.persistentConfigDirtyError = _persistentConfigDirtyError;
  out.beginPolicy = _config.beginPolicy;
  out.beginProbeStatus = _beginProbeStatus;
  out.identity = _identity;
  out.capabilities = _capabilities;
  return Status::Ok();
}

SettingsSnapshot EE871::getSettings() const {
  SettingsSnapshot out;
  (void)getSettings(out);
  return out;
}

void EE871::_resetStoppedState() {
  _config = Config{};
  _initialized = false;
  _driverState = DriverState::UNINIT;
  _nowMs = 0;
  _clearIdentityAndCapabilities();
  _beginProbeStatus = Status::Ok();
  _recoveryBypass = false;
  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;
  _lastWriteProgress = WriteProgress{};
}

void EE871::_publishIdentityAndCapabilities(
    const DeviceIdentity& identity,
    const CapabilitySnapshot& capabilities) {
  _identity = identity;
  _capabilities = capabilities;
  _operatingFunctions = capabilities.operatingFunctions;
  _operatingModeSupport = capabilities.operatingModeSupport;
  _specialFeatures = capabilities.specialFeatures;
}

void EE871::_clearIdentityAndCapabilities() {
  _identity = DeviceIdentity{};
  _capabilities = CapabilitySnapshot{};
  _operatingFunctions = 0;
  _operatingModeSupport = 0;
  _specialFeatures = 0;
}

void EE871::_latchSemanticOffline(const Status& cause) {
  _clearIdentityAndCapabilities();
  _driverState = DriverState::OFFLINE;
  if (_consecutiveFailures < _config.offlineThreshold) {
    _consecutiveFailures = _config.offlineThreshold;
  }
  if (cause.code == Err::NOT_SUPPORTED) {
    _lastError = cause;
  }
}

bool EE871::_normalOperationAllowed(Status& status) const {
  if (_initialized &&
      _driverState == DriverState::OFFLINE &&
      !_recoveryBypass) {
    status = Status::Error(
        Err::OFFLINE, "Driver is offline; call recover()");
    return false;
  }
  status = Status::Ok();
  return true;
}

void EE871::_markPersistentConfigDirty(const Status& st) {
  if (!_persistentConfigDirty) {
    _persistentConfigDirty = true;
    _persistentConfigDirtyError = st;
  }
}

void EE871::_clearPersistentConfigDirty() {
  _persistentConfigDirty = false;
  _persistentConfigDirtyError = Status::Ok();
}

Status EE871::probe() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  DeviceIdentity candidate;
  return _readAndValidateIdentityRaw(candidate);
}

Status EE871::recover() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  const bool enteredOffline =
      _driverState == DriverState::OFFLINE;
  struct RecoveryBypassScope {
    explicit RecoveryBypassScope(bool& bypassIn) : bypass(bypassIn) {
      bypass = true;
    }
    ~RecoveryBypassScope() { bypass = false; }
    bool& bypass;
  } bypassScope(_recoveryBypass);

  Status st = _busResetTracked();
  if (!st.ok()) {
    if (enteredOffline) {
      _latchSemanticOffline(st);
    }
    return st;
  }

  DeviceIdentity identityCandidate;
  st = _readAndValidateIdentityTracked(identityCandidate);
  if (!st.ok()) {
    if (enteredOffline || st.code == Err::NOT_SUPPORTED) {
      _latchSemanticOffline(st);
    }
    return st;
  }

  CapabilitySnapshot capabilityCandidate;
  st = _readCapabilitiesTracked(capabilityCandidate);
  if (!st.ok()) {
    if (enteredOffline) {
      _latchSemanticOffline(st);
    }
    return st;
  }

  _publishIdentityAndCapabilities(
      identityCandidate, capabilityCandidate);
  _beginProbeStatus = Status::Ok();
  _driverState = DriverState::READY;
  _consecutiveFailures = 0;
  return Status::Ok();
}

Status EE871::_readAndValidateIdentityRaw(
    DeviceIdentity& out,
    bool* nackTerminatedCleanly) {
  return _readAndValidateIdentity(
      out, false, nackTerminatedCleanly);
}

Status EE871::_readAndValidateIdentityTracked(DeviceIdentity& out) {
  return _readAndValidateIdentity(out, true, nullptr);
}

Status EE871::_readAndValidateIdentity(
    DeviceIdentity& out,
    bool tracked,
    bool* nackTerminatedCleanly) {
  out = DeviceIdentity{};
  if (nackTerminatedCleanly != nullptr) {
    *nackTerminatedCleanly = false;
  }
  auto readMain =
      [this, tracked, nackTerminatedCleanly](
          uint8_t mainCommand, uint8_t& value) {
        const uint8_t control =
            cmd::makeControlRead(mainCommand, _config.deviceAddress);
        if (tracked) {
          return _readControlByteTracked(control, value);
        }
        bool transactionTerminatedCleanly = false;
        const Status readStatus = _readControlByteRaw(
            control, value, &transactionTerminatedCleanly);
        if (nackTerminatedCleanly != nullptr &&
            readStatus.code == Err::NACK) {
          *nackTerminatedCleanly = transactionTerminatedCleanly;
        }
        return readStatus;
      };

  uint8_t groupLow = 0;
  uint8_t groupHigh = 0;
  uint8_t subgroup = 0;
  uint8_t availableMeasurements = 0;
  Status st = readMain(cmd::MAIN_TYPE_LO, groupLow);
  if (!st.ok()) {
    return st;
  }
  st = readMain(cmd::MAIN_TYPE_HI, groupHigh);
  if (!st.ok()) {
    return st;
  }

  const uint16_t group =
      static_cast<uint16_t>(groupLow) |
      (static_cast<uint16_t>(groupHigh) << 8);
  if (group != cmd::SENSOR_GROUP_ID) {
    return Status::Error(
        Err::NOT_SUPPORTED, "Unexpected group id", group);
  }

  st = readMain(cmd::MAIN_TYPE_SUB, subgroup);
  if (!st.ok()) {
    return st;
  }
  if (subgroup != cmd::SENSOR_SUBGROUP_ID) {
    return Status::Error(
        Err::NOT_SUPPORTED, "Unexpected subgroup id", subgroup);
  }

  st = readMain(cmd::MAIN_AVAIL_MEAS, availableMeasurements);
  if (!st.ok()) {
    return st;
  }
  if ((availableMeasurements & cmd::AVAILABLE_MEAS_MASK) == 0U) {
    return Status::Error(
        Err::NOT_SUPPORTED,
        "CO2 measurement not advertised",
        availableMeasurements);
  }

  DeviceIdentity candidate;
  candidate.group = group;
  candidate.subgroup = subgroup;
  candidate.availableMeasurements = availableMeasurements;
  candidate.co2Available = true;
  candidate.valid = true;
  out = candidate;
  return Status::Ok();
}

Status EE871::_readCapabilitiesRaw(CapabilitySnapshot& out) {
  return _readCapabilities(out, false);
}

Status EE871::_readCapabilitiesTracked(CapabilitySnapshot& out) {
  return _readCapabilities(out, true);
}

Status EE871::_readCapabilities(
    CapabilitySnapshot& out, bool tracked) {
  out = CapabilitySnapshot{};
  Status st = tracked
                  ? _setCustomPointerTracked(
                        cmd::CUSTOM_ADJUSTMENT_SUPPORT)
                  : _setCustomPointerRaw(
                        cmd::CUSTOM_ADJUSTMENT_SUPPORT);
  if (!st.ok()) {
    return st;
  }

  uint8_t values[7] = {};
  const uint8_t control =
      cmd::makeControlRead(
          cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
  for (uint8_t i = 0; i < 7U; ++i) {
    st = tracked
             ? _readControlByteTracked(control, values[i])
             : _readControlByteRaw(control, values[i]);
    if (!st.ok()) {
      return st;
    }
  }

  CapabilitySnapshot candidate;
  candidate.customAdjustmentSupport = values[0];
  candidate.adjustmentPointSupport = values[1];
  candidate.adjustmentTimeGeneralSupport = values[2];
  candidate.adjustmentTimeSupport = values[3];
  candidate.operatingFunctions = values[4];
  candidate.operatingModeSupport = values[5];
  candidate.specialFeatures = values[6];
  candidate.valid = true;
  out = candidate;
  return Status::Ok();
}

Status EE871::resyncPersistentConfig() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  uint16_t interval = 0;
  Status st = readMeasurementInterval(interval);
  if (!st.ok()) {
    return st;
  }
  if (interval < cmd::INTERVAL_MIN_DECISEC ||
      interval > cmd::INTERVAL_MAX_DECISEC) {
    return Status::Error(Err::OUT_OF_RANGE, "Interval out of range", interval);
  }

  int16_t offset = 0;
  st = readCo2Offset(offset);
  if (!st.ok()) {
    return st;
  }

  uint16_t gain = 0;
  st = readCo2Gain(gain);
  if (!st.ok()) {
    return st;
  }

  if (hasPartName()) {
    uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
    st = readPartName(partName);
    if (!st.ok()) {
      return st;
    }
  }

  _clearPersistentConfigDirty();
  return Status::Ok();
}

Status EE871::readControlByte(uint8_t mainCommandNibble, uint8_t& data) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  if (mainCommandNibble > 0x0F) {
    return Status::Error(Err::INVALID_PARAM, "Invalid main command");
  }
  if (!cmd::isReadMainCommandSupported(mainCommandNibble)) {
    return Status::Error(Err::NOT_SUPPORTED, "Unsupported EE871 main command",
                         mainCommandNibble);
  }
  const uint8_t control = cmd::makeControlRead(mainCommandNibble, _config.deviceAddress);
  return _readControlByteTracked(control, data);
}

Status EE871::readU16(uint8_t mainCommandLow, uint8_t mainCommandHigh, uint16_t& value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  if (mainCommandLow > 0x0F || mainCommandHigh > 0x0F) {
    return Status::Error(Err::INVALID_PARAM, "Invalid main command");
  }
  if (!cmd::isReadMainCommandSupported(mainCommandLow) ||
      !cmd::isReadMainCommandSupported(mainCommandHigh)) {
    return Status::Error(Err::NOT_SUPPORTED, "Unsupported EE871 main command");
  }

  uint8_t low = 0;
  uint8_t high = 0;
  Status st = readControlByte(mainCommandLow, low);
  if (!st.ok()) {
    return st;
  }
  st = readControlByte(mainCommandHigh, high);
  if (!st.ok()) {
    return st;
  }
  value = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  return Status::Ok();
}

Status EE871::setCustomPointer(uint16_t address) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  if (address > 0xFF) {
    return Status::Error(Err::OUT_OF_RANGE, "Custom pointer > 0xFF",
                         static_cast<int32_t>(address));
  }
  return _setCustomPointerTracked(static_cast<uint8_t>(address));
}

Status EE871::customRead(uint8_t address, uint8_t& data) {
  return customRead(address, &data, 1);
}

Status EE871::customRead(uint8_t address, uint8_t* buf, size_t len) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  if (buf == nullptr || len == 0) {
    return Status::Error(Err::INVALID_PARAM, "Invalid buffer");
  }
  const size_t maxLen = static_cast<size_t>(cmd::CUSTOM_MEMORY_SIZE) -
                        static_cast<size_t>(address);
  if (len > maxLen) {
    return Status::Error(Err::OUT_OF_RANGE, "Read exceeds custom memory map");
  }
  Status st = setCustomPointer(address);
  if (!st.ok()) {
    return st;
  }

  for (size_t i = 0; i < len; ++i) {
    st = readControlByte(cmd::MAIN_CUSTOM_PTR, buf[i]);
    if (!st.ok()) {
      return st;
    }
  }
  return Status::Ok();
}

Status EE871::customWrite(uint8_t address, uint8_t value) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  if (address == cmd::CUSTOM_INTERVAL_L || address == cmd::CUSTOM_INTERVAL_H) {
    uint8_t other = 0;
    const uint8_t otherAddr = (address == cmd::CUSTOM_INTERVAL_L)
                                  ? cmd::CUSTOM_INTERVAL_H
                                  : cmd::CUSTOM_INTERVAL_L;
    Status st = customRead(otherAddr, other);
    if (!st.ok()) {
      return st;
    }
    const uint16_t interval = (address == cmd::CUSTOM_INTERVAL_L)
                                  ? static_cast<uint16_t>(value) |
                                        (static_cast<uint16_t>(other) << 8)
                                  : static_cast<uint16_t>(other) |
                                        (static_cast<uint16_t>(value) << 8);
    return writeMeasurementInterval(interval);
  }

  return _customWriteDirect(address, value);
}

Status EE871::_setCustomPointerRaw(uint8_t address) {
  const uint8_t control =
      cmd::makeControlWrite(cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
  return _writeCommandRaw(
      control,
      0x00,
      address,
      ClockWaitClass::WRITE_COMPLETION,
      nullptr);
}

Status EE871::_setCustomPointerTracked(uint8_t address) {
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  return _updateHealth(_setCustomPointerRaw(address));
}

Status EE871::_customWriteDirect(
    uint8_t address, uint8_t value, bool* writeMayHaveEffect) {
  if (writeMayHaveEffect != nullptr) {
    *writeMayHaveEffect = false;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  const uint8_t control = cmd::makeControlWrite(cmd::MAIN_CUSTOM_WRITE, _config.deviceAddress);
  WriteProgress progress;
  Status st = _writeCommandTracked(
      control,
      address,
      value,
      ClockWaitClass::WRITE_COMPLETION,
      &progress);
  if (writeMayHaveEffect != nullptr) {
    *writeMayHaveEffect =
        progress.effect == WriteEffect::INDETERMINATE ||
        progress.effect == WriteEffect::ACKNOWLEDGED ||
        progress.effect == WriteEffect::VERIFIED;
  }
  if (!st.ok()) {
    return st;
  }

  uint8_t verify = 0;
  st = customRead(address, verify);
  if (!st.ok()) {
    return st;
  }
  if (verify != value) {
    return Status::Error(
        Err::VERIFY_MISMATCH, "Write verification mismatch", verify);
  }
  _lastWriteProgress.effect = WriteEffect::VERIFIED;
  return Status::Ok();
}

Status EE871::writeMeasurementInterval(uint16_t intervalDeciSeconds) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasGlobalInterval()) {
    return Status::Error(Err::NOT_SUPPORTED, "Global interval not supported");
  }

  // Validate range: 15.0s - 3600.0s (150 - 36000 deciseconds)
  if (intervalDeciSeconds < cmd::INTERVAL_MIN_DECISEC ||
      intervalDeciSeconds > cmd::INTERVAL_MAX_DECISEC) {
    return Status::Error(Err::OUT_OF_RANGE, "Interval must be 150-36000 (15-3600s)",
                         intervalDeciSeconds);
  }

  const uint8_t control = cmd::makeControlWrite(cmd::MAIN_CUSTOM_WRITE, _config.deviceAddress);
  const uint8_t low = static_cast<uint8_t>(intervalDeciSeconds & 0xFF);
  const uint8_t high = static_cast<uint8_t>(intervalDeciSeconds >> 8);

  WriteProgress lowProgress;
  Status st = _writeCommandTracked(
      control,
      cmd::CUSTOM_INTERVAL_L,
      low,
      ClockWaitClass::NORMAL_BIT,
      &lowProgress);
  if (!st.ok()) {
    if (lowProgress.effect == WriteEffect::INDETERMINATE ||
        lowProgress.effect == WriteEffect::ACKNOWLEDGED) {
      _markPersistentConfigDirty(st);
    }
    return st;
  }
  st = _writeCommandTracked(
      control,
      cmd::CUSTOM_INTERVAL_H,
      high,
      ClockWaitClass::INTERVAL_COMMIT,
      nullptr);
  if (!st.ok()) {
    _markPersistentConfigDirty(st);
    return st;
  }

  uint8_t verifyBytes[2] = {};
  st = customRead(cmd::CUSTOM_INTERVAL_L, verifyBytes, 2);
  if (!st.ok()) {
    _markPersistentConfigDirty(st);
    return st;
  }
  const uint16_t verify = static_cast<uint16_t>(verifyBytes[0]) |
                          (static_cast<uint16_t>(verifyBytes[1]) << 8);
  if (verify != intervalDeciSeconds) {
    Status err = Status::Error(
        Err::VERIFY_MISMATCH, "Write verification mismatch", verify);
    _markPersistentConfigDirty(err);
    return err;
  }
  _lastWriteProgress.effect = WriteEffect::VERIFIED;
  return Status::Ok();
}

Status EE871::readGroup(uint16_t& group) {
  Status st = readU16(cmd::MAIN_TYPE_LO, cmd::MAIN_TYPE_HI, group);
  if (!st.ok()) {
    return st;
  }
  if (group != cmd::SENSOR_GROUP_ID) {
    return Status::Error(Err::NOT_SUPPORTED, "Unexpected group id", group);
  }
  return Status::Ok();
}

Status EE871::readSubgroup(uint8_t& subgroup) {
  Status st = readControlByte(cmd::MAIN_TYPE_SUB, subgroup);
  if (!st.ok()) {
    return st;
  }
  if (subgroup != cmd::SENSOR_SUBGROUP_ID) {
    return Status::Error(Err::NOT_SUPPORTED, "Unexpected subgroup id", subgroup);
  }
  return Status::Ok();
}

Status EE871::readAvailableMeasurements(uint8_t& bits) {
  return readControlByte(cmd::MAIN_AVAIL_MEAS, bits);
}

Status EE871::readStatus(uint8_t& status) {
  return readControlByte(cmd::MAIN_STATUS, status);
}

Status EE871::readErrorCode(uint8_t& code) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasErrorCode()) {
    return Status::Error(Err::NOT_SUPPORTED, "Error code not supported");
  }
  return customRead(cmd::CUSTOM_ERROR_CODE, code);
}

Status EE871::readCo2Fast(uint16_t& ppm) {
  return readU16(cmd::MAIN_MV3_LO, cmd::MAIN_MV3_HI, ppm);
}

Status EE871::readCo2Average(uint16_t& ppm) {
  return readU16(cmd::MAIN_MV4_LO, cmd::MAIN_MV4_HI, ppm);
}

Status EE871::readCo2AverageSample(Co2ReadResult& out) {
  return _readCo2Sample(Co2ValueKind::AVERAGE, out);
}

Status EE871::readCo2FastSample(Co2ReadResult& out) {
  return _readCo2Sample(Co2ValueKind::FAST, out);
}

Status EE871::_readCo2Sample(
    Co2ValueKind kind, Co2ReadResult& out) {
  out = Co2ReadResult{};
  out.kind = kind;

  uint16_t ppm = 0;
  out.valueReadAttempted = true;
  out.valueReadStatus =
      kind == Co2ValueKind::AVERAGE
          ? readCo2Average(ppm)
          : readCo2Fast(ppm);
  if (!out.valueReadStatus.ok()) {
    return out.valueReadStatus;
  }
  out.ppm = ppm;

  uint8_t status = 0;
  out.statusReadAttempted = true;
  out.statusReadStatus = readStatus(status);
  if (!out.statusReadStatus.ok()) {
    return out.statusReadStatus;
  }
  out.statusByte = status;
  out.statusValid = true;
  out.co2Error =
      (status & cmd::STATUS_CO2_ERROR_MASK) != 0U;

  if (out.co2Error) {
    out.sensorError = Co2SensorError::UNKNOWN;
    if (hasErrorCode()) {
      uint8_t code = 0;
      out.errorCodeReadAttempted = true;
      out.errorCodeReadStatus = readErrorCode(code);
      if (!out.errorCodeReadStatus.ok()) {
        return out.errorCodeReadStatus;
      }
      out.errorCode = code;
      out.errorCodeValid = true;
      out.sensorError = co2SensorErrorFromCode(code);
    }
    return Status::Error(
        Err::CO2_SENSOR_ERROR,
        "CO2 sensor status error",
        out.errorCodeValid ? out.errorCode : out.statusByte);
  }

  if (ppm > cmd::CO2_PPM_MAX) {
    return Status::Error(
        Err::OUT_OF_RANGE, "CO2 ppm out of range", ppm);
  }

  out.ppmValid = true;
  return Status::Ok();
}

// ============================================================================
// Firmware / Spec Version
// ============================================================================

Status EE871::readFirmwareVersion(uint8_t& main, uint8_t& sub) {
  Status st = customRead(cmd::CUSTOM_FW_VERSION_MAIN, main);
  if (!st.ok()) {
    return st;
  }
  return customRead(cmd::CUSTOM_FW_VERSION_SUB, sub);
}

Status EE871::readE2SpecVersion(uint8_t& version) {
  return customRead(cmd::CUSTOM_E2_SPEC_VERSION, version);
}

// ============================================================================
// Feature Discovery
// ============================================================================

Status EE871::readOperatingFunctions(uint8_t& bits) {
  return customRead(cmd::CUSTOM_OPERATING_FUNCTIONS, bits);
}

Status EE871::readOperatingModeSupport(uint8_t& bits) {
  return customRead(cmd::CUSTOM_OPERATING_MODE_SUPPORT, bits);
}

Status EE871::readSpecialFeatures(uint8_t& bits) {
  return customRead(cmd::CUSTOM_SPECIAL_FEATURES, bits);
}

// ============================================================================
// Identity Strings
// ============================================================================

Status EE871::readSerialNumber(uint8_t* buf) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (buf == nullptr) {
    return Status::Error(Err::INVALID_PARAM, "Null buffer");
  }
  if (!hasSerialNumber()) {
    return Status::Error(Err::NOT_SUPPORTED, "Serial number not supported");
  }
  return customRead(cmd::CUSTOM_SERIAL_START, buf, cmd::CUSTOM_SERIAL_LEN);
}

Status EE871::readPartName(uint8_t* buf) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (buf == nullptr) {
    return Status::Error(Err::INVALID_PARAM, "Null buffer");
  }
  if (!hasPartName()) {
    return Status::Error(Err::NOT_SUPPORTED, "Part name not supported");
  }
  return customRead(cmd::CUSTOM_PART_NAME_START, buf, cmd::CUSTOM_PART_NAME_LEN);
}

Status EE871::writePartName(const uint8_t* buf) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (buf == nullptr) {
    return Status::Error(Err::INVALID_PARAM, "Null buffer");
  }
  if (!hasPartName()) {
    return Status::Error(Err::NOT_SUPPORTED, "Part name not supported");
  }
  for (uint8_t i = 0; i < cmd::CUSTOM_PART_NAME_LEN; ++i) {
    bool accepted = false;
    Status st = _customWriteDirect(cmd::CUSTOM_PART_NAME_START + i, buf[i], &accepted);
    if (!st.ok()) {
      if (i > 0 || accepted) {
        _markPersistentConfigDirty(st);
      }
      return st;
    }
  }
  return Status::Ok();
}

// ============================================================================
// Bus Address
// ============================================================================

Status EE871::readBusAddress(uint8_t& address) {
  // Address can always be read, guard only applies to write
  return customRead(cmd::CUSTOM_BUS_ADDRESS, address);
}

Status EE871::writeBusAddress(uint8_t address) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasAddressConfig()) {
    return Status::Error(Err::NOT_SUPPORTED, "Address config not supported");
  }
  if (address > cmd::BUS_ADDRESS_MAX) {
    return Status::Error(Err::OUT_OF_RANGE, "Address must be 0-7", address);
  }
  return customWrite(cmd::CUSTOM_BUS_ADDRESS, address);
}

// ============================================================================
// Measurement Interval
// ============================================================================

Status EE871::readMeasurementInterval(uint16_t& intervalDeciSeconds) {
  // Interval can always be read, guard only applies to write
  uint8_t low = 0;
  uint8_t high = 0;
  Status st = customRead(cmd::CUSTOM_INTERVAL_L, low);
  if (!st.ok()) {
    return st;
  }
  st = customRead(cmd::CUSTOM_INTERVAL_H, high);
  if (!st.ok()) {
    return st;
  }
  intervalDeciSeconds = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  return Status::Ok();
}

Status EE871::readCo2IntervalFactor(int8_t& factor) {
  // Factor can always be read, guard only applies to write
  uint8_t raw = 0;
  Status st = customRead(cmd::CUSTOM_CO2_INTERVAL_FACTOR, raw);
  if (!st.ok()) {
    return st;
  }
  factor = static_cast<int8_t>(raw);
  return Status::Ok();
}

Status EE871::writeCo2IntervalFactor(int8_t factor) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasSpecificInterval()) {
    return Status::Error(Err::NOT_SUPPORTED, "Specific interval not supported");
  }
  return customWrite(cmd::CUSTOM_CO2_INTERVAL_FACTOR, static_cast<uint8_t>(factor));
}

// ============================================================================
// Filter / Operating Mode
// ============================================================================

Status EE871::readCo2Filter(uint8_t& filter) {
  // Filter can always be read, guard only applies to write
  return customRead(cmd::CUSTOM_FILTER_CO2, filter);
}

Status EE871::writeCo2Filter(uint8_t filter) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasFilterConfig()) {
    return Status::Error(Err::NOT_SUPPORTED, "Filter config not supported");
  }
  return customWrite(cmd::CUSTOM_FILTER_CO2, filter);
}

Status EE871::readOperatingMode(uint8_t& mode) {
  // Mode can always be read, guard only applies to write
  return customRead(cmd::CUSTOM_OPERATING_MODE, mode);
}

Status EE871::writeOperatingMode(uint8_t mode) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  // Only bits 0 and 1 are valid.
  if (mode > 0x03) {
    return Status::Error(Err::OUT_OF_RANGE, "Invalid mode bits", mode);
  }
  // Check if requested mode bits are supported
  if ((mode & cmd::OPERATING_MODE_MEASUREMODE_MASK) && !hasLowPowerMode()) {
    return Status::Error(Err::NOT_SUPPORTED, "Low power mode not supported");
  }
  if ((mode & cmd::OPERATING_MODE_E2_PRIORITY_MASK) && !hasE2Priority()) {
    return Status::Error(Err::NOT_SUPPORTED, "E2 priority not supported");
  }
  return customWrite(cmd::CUSTOM_OPERATING_MODE, mode);
}

// ============================================================================
// Auto Adjustment
// ============================================================================

Status EE871::readAutoAdjustStatus(bool& running) {
  // Status can always be read, guard only applies to start
  uint8_t raw = 0;
  Status st = customRead(cmd::CUSTOM_AUTO_ADJUST, raw);
  if (!st.ok()) {
    return st;
  }
  running = (raw & cmd::AUTO_ADJUST_RUNNING_MASK) != 0;
  return Status::Ok();
}

Status EE871::startAutoAdjust() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasAutoAdjust()) {
    return Status::Error(Err::NOT_SUPPORTED, "Auto adjust not supported");
  }
  // Writing 1 starts auto adjustment (cannot be stopped)
  return customWrite(cmd::CUSTOM_AUTO_ADJUST, 0x01);
}

// ============================================================================
// Calibration
// ============================================================================

Status EE871::readCo2Offset(int16_t& offset) {
  uint8_t low = 0;
  uint8_t high = 0;
  Status st = customRead(cmd::CUSTOM_CO2_OFFSET_L, low);
  if (!st.ok()) {
    return st;
  }
  st = customRead(cmd::CUSTOM_CO2_OFFSET_H, high);
  if (!st.ok()) {
    return st;
  }
  offset = static_cast<int16_t>(static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8));
  return Status::Ok();
}

Status EE871::writeCo2Offset(int16_t offset) {
  const uint16_t raw = static_cast<uint16_t>(offset);
  bool lowAccepted = false;
  Status st = _customWriteDirect(cmd::CUSTOM_CO2_OFFSET_L,
                                 static_cast<uint8_t>(raw & 0xFF),
                                 &lowAccepted);
  if (!st.ok()) {
    if (lowAccepted) {
      _markPersistentConfigDirty(st);
    }
    return st;
  }
  st = _customWriteDirect(cmd::CUSTOM_CO2_OFFSET_H, static_cast<uint8_t>(raw >> 8));
  if (!st.ok()) {
    _markPersistentConfigDirty(st);
  }
  return st;
}

Status EE871::readCo2Gain(uint16_t& gain) {
  uint8_t low = 0;
  uint8_t high = 0;
  Status st = customRead(cmd::CUSTOM_CO2_GAIN_L, low);
  if (!st.ok()) {
    return st;
  }
  st = customRead(cmd::CUSTOM_CO2_GAIN_H, high);
  if (!st.ok()) {
    return st;
  }
  gain = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  return Status::Ok();
}

Status EE871::writeCo2Gain(uint16_t gain) {
  bool lowAccepted = false;
  Status st = _customWriteDirect(cmd::CUSTOM_CO2_GAIN_L,
                                 static_cast<uint8_t>(gain & 0xFF),
                                 &lowAccepted);
  if (!st.ok()) {
    if (lowAccepted) {
      _markPersistentConfigDirty(st);
    }
    return st;
  }
  st = _customWriteDirect(cmd::CUSTOM_CO2_GAIN_H, static_cast<uint8_t>(gain >> 8));
  if (!st.ok()) {
    _markPersistentConfigDirty(st);
  }
  return st;
}

Status EE871::readCo2CalPoints(uint16_t& lower, uint16_t& upper) {
  uint8_t buf[4] = {0};
  Status st = customRead(cmd::CUSTOM_CO2_POINT_L_L, buf, 4);
  if (!st.ok()) {
    return st;
  }
  lower = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
  upper = static_cast<uint16_t>(buf[2]) | (static_cast<uint16_t>(buf[3]) << 8);
  return Status::Ok();
}

// ============================================================================
// Bus Safety
// ============================================================================

Status EE871::busReset() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  return _busResetRaw();
}

Status EE871::_busResetRaw() {
  setSda(_config, true);
  for (uint8_t i = 0; i < cmd::BUS_RESET_CLOCKS; ++i) {
    setScl(_config, false);
    _delayUs(_config, _config.clockLowUs);
    setScl(_config, true);
    ByteDeadline pulseWait{0, _config.bitTimeoutUs};
    Status st = _waitSclHigh(
        _config, ClockWaitClass::NORMAL_BIT, pulseWait);
    if (!st.ok()) {
      return Status::Error(
          Err::BUS_STUCK, "SCL stuck during reset", st.detail);
    }
    _delayUs(_config, _config.clockHighUs);
  }

  setScl(_config, false);
  _delayUs(_config, _config.clockLowUs);
  Status st = _e2Stop(
      _config, ClockWaitClass::NORMAL_BIT, nullptr);
  if (!st.ok()) {
    return Status::Error(
        Err::BUS_STUCK, "SCL stuck during reset STOP", st.detail);
  }

  if (!readScl(_config) || !readSda(_config)) {
    return Status::Error(Err::BUS_STUCK, "Bus stuck after reset");
  }

  return Status::Ok();
}

Status EE871::_busResetTracked() {
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  return _updateHealth(_busResetRaw());
}

Status EE871::checkBusIdle() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  const bool sclHigh = readScl(_config);
  const bool sdaHigh = readSda(_config);

  if (!sclHigh && !sdaHigh) {
    return Status::Error(Err::BUS_STUCK, "Both SCL and SDA stuck low");
  }
  if (!sclHigh) {
    return Status::Error(Err::BUS_STUCK, "SCL stuck low");
  }
  if (!sdaHigh) {
    return Status::Error(Err::BUS_STUCK, "SDA stuck low");
  }

  return Status::Ok();
}

Status EE871::_readControlByteRaw(
    uint8_t controlByte,
    uint8_t& data,
    bool* transactionTerminatedCleanly) {
  if (transactionTerminatedCleanly != nullptr) {
    *transactionTerminatedCleanly = false;
  }
  Status st = _e2Start(_config);
  if (!st.ok()) {
    return st;
  }

  auto cleanup = [this, transactionTerminatedCleanly](
                     const Status& primary) {
    const Status cleanupStatus =
        _e2Stop(_config, ClockWaitClass::NORMAL_BIT, nullptr);
    if (transactionTerminatedCleanly != nullptr) {
      *transactionTerminatedCleanly = cleanupStatus.ok();
    }
    return primary.ok() ? cleanupStatus : primary;
  };

  ByteDeadline controlDeadline{0, _config.byteTimeoutUs};
  st = _writeByte(_config, controlByte, controlDeadline);
  if (!st.ok()) {
    return cleanup(st);
  }

  bool acked = false;
  st = _readAck(
      _config, acked, ClockWaitClass::NORMAL_BIT, controlDeadline);
  if (!st.ok()) {
    return cleanup(st);
  }
  if (!acked) {
    return cleanup(Status::Error(Err::NACK, "Control byte NACK"));
  }

  ByteDeadline dataDeadline{0, _config.byteTimeoutUs};
  st = _readByte(_config, data, dataDeadline);
  if (!st.ok()) {
    return cleanup(st);
  }
  st = _sendAck(_config, true, dataDeadline);
  if (!st.ok()) {
    return cleanup(st);
  }

  uint8_t pec = 0;
  ByteDeadline pecDeadline{0, _config.byteTimeoutUs};
  st = _readByte(_config, pec, pecDeadline);
  if (!st.ok()) {
    return cleanup(st);
  }
  st = _sendAck(_config, false, pecDeadline);
  if (!st.ok()) {
    return cleanup(st);
  }

  const uint8_t expected = calcPecRead(controlByte, data);
  const Status pecStatus =
      pec == expected
          ? Status::Ok()
          : Status::Error(Err::PEC_MISMATCH, "PEC mismatch", pec);
  const Status stopStatus =
      _e2Stop(_config, ClockWaitClass::NORMAL_BIT, nullptr);
  if (transactionTerminatedCleanly != nullptr) {
    *transactionTerminatedCleanly = stopStatus.ok();
  }
  if (!pecStatus.ok()) {
    return pecStatus;
  }
  return stopStatus;
}

Status EE871::_readControlByteTracked(uint8_t controlByte, uint8_t& data) {
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  Status st = _readControlByteRaw(controlByte, data);
  return _updateHealth(st);
}

Status EE871::_writeCommandRaw(
    uint8_t controlByte,
    uint8_t addressByte,
    uint8_t dataByte,
    ClockWaitClass completionClass,
    WriteProgress* progressOut) {
  WriteProgress progress;
  _lastWriteProgress = progress;

  Status st = _e2Start(_config);
  if (!st.ok()) {
    if (progressOut != nullptr) {
      *progressOut = progress;
    }
    return st;
  }

  auto publishProgress = [this, progressOut](const WriteProgress& value) {
    _lastWriteProgress = value;
    if (progressOut != nullptr) {
      *progressOut = value;
    }
  };
  auto cleanupNormal = [this](const Status& primary) {
    const Status cleanupStatus =
        _e2Stop(_config, ClockWaitClass::NORMAL_BIT, nullptr);
    return primary.ok() ? cleanupStatus : primary;
  };
  auto sendPayloadByte =
      [this, &cleanupNormal](uint8_t value, const char* nackMessage) {
        ByteDeadline deadline{0, _config.byteTimeoutUs};
        Status byteStatus = _writeByte(_config, value, deadline);
        if (!byteStatus.ok()) {
          return cleanupNormal(byteStatus);
        }
        bool acked = false;
        byteStatus = _readAck(
            _config, acked, ClockWaitClass::NORMAL_BIT, deadline);
        if (!byteStatus.ok()) {
          return cleanupNormal(byteStatus);
        }
        if (!acked) {
          return cleanupNormal(Status::Error(Err::NACK, nackMessage));
        }
        return Status::Ok();
      };

  st = sendPayloadByte(controlByte, "Control byte NACK");
  if (!st.ok()) {
    publishProgress(progress);
    return st;
  }
  st = sendPayloadByte(addressByte, "Address byte NACK");
  if (!st.ok()) {
    publishProgress(progress);
    return st;
  }
  st = sendPayloadByte(dataByte, "Data byte NACK");
  if (!st.ok()) {
    publishProgress(progress);
    return st;
  }

  const uint8_t pec = calcPecWrite(controlByte, addressByte, dataByte);
  ByteDeadline pecDeadline{0, _config.byteTimeoutUs};
  st = _writeByte(_config, pec, pecDeadline);
  if (!st.ok()) {
    st = cleanupNormal(st);
    publishProgress(progress);
    return st;
  }

  progress.pecTransferred = true;
  progress.effect = WriteEffect::INDETERMINATE;

  const bool hasLongCompletion =
      completionClass != ClockWaitClass::NORMAL_BIT;
  uint32_t completionLimitUs = _config.byteTimeoutUs;
  if (hasLongCompletion) {
    const uint32_t completionMs =
        completionClass == ClockWaitClass::INTERVAL_COMMIT
            ? _config.intervalWriteDelayMs
            : _config.writeDelayMs;
    if (completionMs > std::numeric_limits<uint32_t>::max() / 1000U) {
      st = Status::Error(
          Err::OUT_OF_RANGE, "Completion timeout conversion overflow");
      publishProgress(progress);
      return st;
    }
    completionLimitUs = completionMs * 1000U;
  }

  ByteDeadline completionDeadline{
      hasLongCompletion ? 0U : pecDeadline.elapsedUs,
      completionLimitUs};
  bool acked = false;
  st = _readAck(
      _config, acked, completionClass, completionDeadline);
  if (!st.ok()) {
    const Status cleanupStatus =
        _e2Stop(_config, completionClass, &completionDeadline);
    (void)cleanupStatus;
    progress.completionElapsedUs = completionDeadline.elapsedUs;
    publishProgress(progress);
    return st;
  }
  if (!acked) {
    progress.effect = WriteEffect::NO_EFFECT;
    const Status cleanupStatus = hasLongCompletion
        ? _e2Stop(_config, completionClass, &completionDeadline)
        : _e2Stop(_config, ClockWaitClass::NORMAL_BIT, nullptr);
    progress.completionElapsedUs = completionDeadline.elapsedUs;
    progress.stopCompleted = cleanupStatus.ok();
    publishProgress(progress);
    return Status::Error(Err::NACK, "PEC NACK");
  }

  progress.requestAcknowledged = true;
  progress.effect = WriteEffect::ACKNOWLEDGED;
  st = _e2Stop(
      _config, completionClass, &completionDeadline);
  progress.completionElapsedUs = completionDeadline.elapsedUs;
  if (!st.ok()) {
    publishProgress(progress);
    return st;
  }
  progress.stopCompleted = true;

  if (hasLongCompletion &&
      completionDeadline.elapsedUs < completionDeadline.limitUs) {
    const uint32_t remainingUs =
        completionDeadline.limitUs - completionDeadline.elapsedUs;
    _delayLongMs(_config, remainingUs / 1000U);
    const uint32_t remainderUs = remainingUs % 1000U;
    if (remainderUs != 0U) {
      _delayUs(_config, remainderUs);
    }
  }

  publishProgress(progress);
  return Status::Ok();
}

Status EE871::_writeCommandTracked(
    uint8_t controlByte,
    uint8_t addressByte,
    uint8_t dataByte,
    ClockWaitClass completionClass,
    WriteProgress* progress) {
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    if (progress != nullptr) {
      *progress = WriteProgress{};
    }
    return guard;
  }
  Status st = _writeCommandRaw(
      controlByte,
      addressByte,
      dataByte,
      completionClass,
      progress);
  return _updateHealth(st);
}

Status EE871::_updateHealth(const Status& st) {
  if (!_initialized) {
    return st;
  }
  if (st.inProgress()) {
    return st;
  }

  if (st.ok()) {
    _lastOkMs = _nowMs;
    _consecutiveFailures = 0;
    if (_totalSuccess != std::numeric_limits<uint32_t>::max()) {
      _totalSuccess++;
    }
    _driverState = DriverState::READY;
  } else {
    _lastErrorMs = _nowMs;
    _lastError = st;
    if (_totalFailures != std::numeric_limits<uint32_t>::max()) {
      _totalFailures++;
    }
    if (_consecutiveFailures != std::numeric_limits<uint8_t>::max()) {
      _consecutiveFailures++;
    }
    if (_consecutiveFailures >= _config.offlineThreshold) {
      _driverState = DriverState::OFFLINE;
    } else {
      _driverState = DriverState::DEGRADED;
    }
  }

  return st;
}

} // namespace EE871
