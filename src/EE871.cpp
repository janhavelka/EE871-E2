/// @file EE871.cpp
/// @brief Implementation of the EE871 E2 driver

#include "EE871/EE871.h"

#include <limits>

namespace EE871 {
namespace {

static constexpr uint32_t kPollStepUs = 5;

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

inline void delayUs(const Config& cfg, uint32_t us, uint32_t* elapsedUs) {
  cfg.delayUs(us, cfg.busUser);
  if (elapsedUs != nullptr) {
    const uint32_t room = std::numeric_limits<uint32_t>::max() - *elapsedUs;
    *elapsedUs = (us > room) ? std::numeric_limits<uint32_t>::max() : (*elapsedUs + us);
  }
}

static Status waitSclHigh(const Config& cfg, uint32_t* elapsedUs) {
  uint32_t waitedUs = 0;
  while (!readScl(cfg)) {
    if (waitedUs >= cfg.bitTimeoutUs) {
      return Status::Error(Err::TIMEOUT, "Clock stretch timeout", static_cast<int32_t>(waitedUs));
    }
    if (elapsedUs != nullptr) {
      const uint32_t remaining =
          (*elapsedUs < cfg.byteTimeoutUs) ? (cfg.byteTimeoutUs - *elapsedUs) : 0U;
      if (remaining < kPollStepUs) {
        return Status::Error(Err::TIMEOUT, "Byte timeout", static_cast<int32_t>(*elapsedUs));
      }
    }
    delayUs(cfg, kPollStepUs, elapsedUs);
    waitedUs += kPollStepUs;
  }
  return Status::Ok();
}

// Data setup time before SCL rises (minimum per E2 spec)
static constexpr uint32_t kDataSetupUs = 10;

static Status e2Start(const Config& cfg) {
  setSda(cfg, true);
  setScl(cfg, true);
  Status st = waitSclHigh(cfg, nullptr);
  if (!st.ok()) {
    return st;
  }
  delayUs(cfg, cfg.startHoldUs, nullptr);
  setSda(cfg, false);
  delayUs(cfg, cfg.startHoldUs, nullptr);
  setScl(cfg, false);
  delayUs(cfg, cfg.clockLowUs, nullptr);
  return Status::Ok();
}

static Status e2Stop(const Config& cfg) {
  // SCL is already low with proper low time from last bit
  setSda(cfg, false);  // Ensure SDA low before releasing SCL
  delayUs(cfg, kDataSetupUs, nullptr);
  setScl(cfg, true);
  Status st = waitSclHigh(cfg, nullptr);
  if (!st.ok()) {
    return st;
  }
  delayUs(cfg, cfg.stopHoldUs, nullptr);
  setSda(cfg, true);
  delayUs(cfg, cfg.stopHoldUs, nullptr);
  return Status::Ok();
}

static Status writeBit(const Config& cfg, bool bit, uint32_t* elapsedUs) {
  // SCL is already low from previous bit or START
  setSda(cfg, bit);
  delayUs(cfg, kDataSetupUs, elapsedUs);  // Data setup time
  setScl(cfg, true);
  Status st = waitSclHigh(cfg, elapsedUs);
  if (!st.ok()) {
    return st;
  }
  delayUs(cfg, cfg.clockHighUs, elapsedUs);
  setScl(cfg, false);
  delayUs(cfg, cfg.clockLowUs, elapsedUs);  // Clock low time AFTER pulling low
  return Status::Ok();
}

static Status readBit(const Config& cfg, bool& bit, uint32_t* elapsedUs) {
  // SCL is already low from previous bit
  setSda(cfg, true);  // Release SDA for slave to drive
  delayUs(cfg, kDataSetupUs, elapsedUs);  // Setup time
  setScl(cfg, true);
  Status st = waitSclHigh(cfg, elapsedUs);
  if (!st.ok()) {
    return st;
  }
  const uint32_t sampleDelay = cfg.clockHighUs / 2;
  delayUs(cfg, sampleDelay, elapsedUs);
  bit = readSda(cfg);
  delayUs(cfg, cfg.clockHighUs - sampleDelay, elapsedUs);
  setScl(cfg, false);
  delayUs(cfg, cfg.clockLowUs, elapsedUs);  // Clock low time AFTER pulling low
  return Status::Ok();
}

static Status writeByte(const Config& cfg, uint8_t value, uint32_t* elapsedUs) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    Status st = writeBit(cfg, (value & mask) != 0, elapsedUs);
    if (!st.ok()) {
      return st;
    }
  }
  return Status::Ok();
}

static Status readByte(const Config& cfg, uint8_t& value, uint32_t* elapsedUs) {
  value = 0;
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    bool bit = false;
    Status st = readBit(cfg, bit, elapsedUs);
    if (!st.ok()) {
      return st;
    }
    if (bit) {
      value |= mask;
    }
  }
  return Status::Ok();
}

static Status readAck(const Config& cfg, bool& acked, uint32_t* elapsedUs) {
  // SCL is already low from last data bit
  setSda(cfg, true);  // Release SDA for slave to drive ACK
  delayUs(cfg, kDataSetupUs, elapsedUs);
  setScl(cfg, true);
  Status st = waitSclHigh(cfg, elapsedUs);
  if (!st.ok()) {
    return st;
  }
  const uint32_t sampleDelay = cfg.clockHighUs / 2;
  delayUs(cfg, sampleDelay, elapsedUs);
  acked = !readSda(cfg);  // ACK = SDA low
  delayUs(cfg, cfg.clockHighUs - sampleDelay, elapsedUs);
  setScl(cfg, false);
  delayUs(cfg, cfg.clockLowUs, elapsedUs);  // Low time for next phase
  return Status::Ok();
}

static Status sendAck(const Config& cfg, bool ack, uint32_t* elapsedUs) {
  // SCL is already low from last data bit
  setSda(cfg, !ack);  // ACK = SDA low, NACK = SDA high
  delayUs(cfg, kDataSetupUs, elapsedUs);
  setScl(cfg, true);
  Status st = waitSclHigh(cfg, elapsedUs);
  if (!st.ok()) {
    return st;
  }
  delayUs(cfg, cfg.clockHighUs, elapsedUs);
  setScl(cfg, false);
  delayUs(cfg, cfg.clockLowUs, elapsedUs);  // Low time for next phase
  setSda(cfg, true);  // Release SDA
  return Status::Ok();
}

static uint8_t calcPecRead(uint8_t controlByte, uint8_t dataByte) {
  return static_cast<uint8_t>((controlByte + dataByte) & 0xFF);
}

static uint8_t calcPecWrite(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte) {
  return static_cast<uint8_t>((controlByte + addressByte + dataByte) & 0xFF);
}

static void delayLongMs(const Config& cfg, uint32_t delayMs) {
  uint32_t remaining = delayMs;
  while (remaining > 0) {
    const uint32_t slice = (remaining < cfg.longDelaySliceMs)
                               ? remaining
                               : static_cast<uint32_t>(cfg.longDelaySliceMs);
    if (cfg.delayMs != nullptr) {
      cfg.delayMs(slice, cfg.busUser);
    } else {
      cfg.delayUs(slice * 1000U, cfg.busUser);
    }
    if (cfg.yield != nullptr) {
      cfg.yield(cfg.busUser);
    }
    remaining -= slice;
  }
}

static bool isPresenceOrTransportFailure(const Status& st) {
  switch (st.code) {
    case Err::NACK:
    case Err::TIMEOUT:
    case Err::BUS_STUCK:
    case Err::E2_ERROR:
    case Err::PEC_MISMATCH:
    case Err::DEVICE_NOT_FOUND:
      return true;
    default:
      return false;
  }
}

} // namespace

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

  // Check bus is idle before probing
  if (!readScl(_config) || !readSda(_config)) {
    st = _busResetRaw();
    if (!st.ok()) {
      _resetStoppedState();
      return st;
    }
  }

  IdentitySnapshot identity;
  st = _readIdentityRaw(identity);
  if (!st.ok()) {
    if (_config.beginPolicy == BeginPolicy::AllowAbsent &&
        isPresenceOrTransportFailure(st)) {
      _initialized = true;
      _driverState = DriverState::OFFLINE;
      _consecutiveFailures = _config.offlineThreshold;
      _beginProbeStatus = st;
      return Status::Ok();
    }
    _resetStoppedState();
    return st;
  }

  (void)_readFeatureFlagsRaw();
  _beginProbeStatus = Status::Ok();

  _initialized = true;
  _driverState = DriverState::READY;
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
  out.hasDelayMs = _config.delayMs != nullptr;
  out.hasYield = _config.yield != nullptr;
  out.longDelaySliceMs = _config.longDelaySliceMs;
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
  _allowOfflineTransfer = false;
  _nowMs = 0;
  _beginProbeStatus = Status::Ok();
  _operatingFunctions = 0;
  _operatingModeSupport = 0;
  _specialFeatures = 0;
  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;
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

Status EE871::_validateConfig(const Config& input, Config& normalized) const {
  if (input.setScl == nullptr || input.setSda == nullptr ||
      input.readScl == nullptr || input.readSda == nullptr ||
      input.delayUs == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "Missing E2 callbacks");
  }
  if (input.deviceAddress > cmd::DEVICE_ADDRESS_MAX) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid device address");
  }
  if (static_cast<uint8_t>(input.beginPolicy) >
      static_cast<uint8_t>(BeginPolicy::AllowAbsent)) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid begin policy");
  }
  if (input.clockLowUs < 100 || input.clockHighUs < 100) {
    return Status::Error(Err::INVALID_CONFIG, "Clock timing below spec");
  }
  if (input.startHoldUs < 4 || input.stopHoldUs < 4) {
    return Status::Error(Err::INVALID_CONFIG, "Start/stop hold below spec");
  }
  if (input.bitTimeoutUs == 0 || input.byteTimeoutUs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "Timeouts must be non-zero");
  }
  if (input.byteTimeoutUs < input.bitTimeoutUs) {
    return Status::Error(Err::INVALID_CONFIG, "byteTimeoutUs must be >= bitTimeoutUs");
  }
  if (input.writeDelayMs > cmd::WRITE_DELAY_MAX_MS) {
    return Status::Error(Err::INVALID_CONFIG, "writeDelayMs exceeds safe limit");
  }
  if (input.intervalWriteDelayMs > cmd::INTERVAL_WRITE_DELAY_MAX_MS) {
    return Status::Error(Err::INVALID_CONFIG, "intervalWriteDelayMs exceeds safe limit");
  }
  if (input.longDelaySliceMs > 50) {
    return Status::Error(Err::INVALID_CONFIG, "longDelaySliceMs exceeds safe limit");
  }

  normalized = input;
  if (normalized.offlineThreshold == 0) {
    normalized.offlineThreshold = 1;
  }
  if (normalized.longDelaySliceMs == 0) {
    normalized.longDelaySliceMs = 1;
  }
  return Status::Ok();
}

Status EE871::_busResetRaw() {
  // Clock out 9+ pulses with SDA high to reset slave state machine.
  setSda(_config, true);
  for (uint8_t i = 0; i < cmd::BUS_RESET_CLOCKS; ++i) {
    setScl(_config, false);
    delayUs(_config, _config.clockLowUs, nullptr);
    setScl(_config, true);

    uint32_t waited = 0;
    while (!readScl(_config) && waited < _config.bitTimeoutUs) {
      delayUs(_config, kPollStepUs, nullptr);
      waited += kPollStepUs;
    }
    if (waited >= _config.bitTimeoutUs) {
      return Status::Error(Err::BUS_STUCK, "SCL stuck during reset");
    }
    delayUs(_config, _config.clockHighUs, nullptr);
  }

  setScl(_config, false);
  delayUs(_config, _config.clockLowUs, nullptr);
  setSda(_config, false);
  delayUs(_config, kDataSetupUs, nullptr);
  setScl(_config, true);
  delayUs(_config, _config.stopHoldUs, nullptr);
  setSda(_config, true);
  delayUs(_config, _config.stopHoldUs, nullptr);

  if (!readScl(_config) || !readSda(_config)) {
    return Status::Error(Err::BUS_STUCK, "Bus stuck after reset");
  }
  return Status::Ok();
}

Status EE871::_readIdentityRaw(IdentitySnapshot& out) {
  uint8_t low = 0;
  uint8_t high = 0;
  Status st = _readControlByteRaw(cmd::makeControlRead(cmd::MAIN_TYPE_LO, _config.deviceAddress),
                                  low);
  if (!st.ok()) {
    return st;
  }
  st = _readControlByteRaw(cmd::makeControlRead(cmd::MAIN_TYPE_HI, _config.deviceAddress),
                           high);
  if (!st.ok()) {
    return st;
  }
  out.group = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  if (out.group != cmd::SENSOR_GROUP_ID) {
    return Status::Error(Err::NOT_SUPPORTED, "Unexpected group id", out.group);
  }

  st = _readControlByteRaw(cmd::makeControlRead(cmd::MAIN_TYPE_SUB, _config.deviceAddress),
                           out.subgroup);
  if (!st.ok()) {
    return st;
  }
  if (out.subgroup != cmd::SENSOR_SUBGROUP_ID) {
    return Status::Error(Err::NOT_SUPPORTED, "Unexpected subgroup id", out.subgroup);
  }

  st = _readControlByteRaw(cmd::makeControlRead(cmd::MAIN_AVAIL_MEAS, _config.deviceAddress),
                           out.availableMeasurements);
  if (!st.ok()) {
    return st;
  }
  if ((out.availableMeasurements & cmd::AVAILABLE_MEAS_MASK) == 0) {
    return Status::Error(Err::NOT_SUPPORTED, "CO2 measurement not advertised",
                         out.availableMeasurements);
  }
  return Status::Ok();
}

Status EE871::_readIdentityTracked(IdentitySnapshot& out) {
  uint8_t low = 0;
  uint8_t high = 0;
  Status st = _readControlByteTracked(
      cmd::makeControlRead(cmd::MAIN_TYPE_LO, _config.deviceAddress), low);
  if (!st.ok()) {
    return st;
  }
  st = _readControlByteTracked(
      cmd::makeControlRead(cmd::MAIN_TYPE_HI, _config.deviceAddress), high);
  if (!st.ok()) {
    return st;
  }
  out.group = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  if (out.group != cmd::SENSOR_GROUP_ID) {
    return Status::Error(Err::NOT_SUPPORTED, "Unexpected group id", out.group);
  }

  st = _readControlByteTracked(
      cmd::makeControlRead(cmd::MAIN_TYPE_SUB, _config.deviceAddress), out.subgroup);
  if (!st.ok()) {
    return st;
  }
  if (out.subgroup != cmd::SENSOR_SUBGROUP_ID) {
    return Status::Error(Err::NOT_SUPPORTED, "Unexpected subgroup id", out.subgroup);
  }

  st = _readControlByteTracked(
      cmd::makeControlRead(cmd::MAIN_AVAIL_MEAS, _config.deviceAddress),
      out.availableMeasurements);
  if (!st.ok()) {
    return st;
  }
  if ((out.availableMeasurements & cmd::AVAILABLE_MEAS_MASK) == 0) {
    return Status::Error(Err::NOT_SUPPORTED, "CO2 measurement not advertised",
                         out.availableMeasurements);
  }
  return Status::Ok();
}

Status EE871::_readFeatureFlagsRaw() {
  _operatingFunctions = 0;
  _operatingModeSupport = 0;
  _specialFeatures = 0;

  const uint8_t ptrControl = cmd::makeControlWrite(cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
  Status st = _writeCommandRaw(ptrControl, 0x00, cmd::CUSTOM_OPERATING_FUNCTIONS);
  if (!st.ok()) {
    return st;
  }

  uint8_t operatingFunctions = 0;
  uint8_t operatingModeSupport = 0;
  uint8_t specialFeatures = 0;
  const uint8_t readControl = cmd::makeControlRead(cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
  st = _readControlByteRaw(readControl, operatingFunctions);
  if (!st.ok()) {
    return st;
  }
  st = _readControlByteRaw(readControl, operatingModeSupport);
  if (!st.ok()) {
    return st;
  }
  st = _readControlByteRaw(readControl, specialFeatures);
  if (!st.ok()) {
    return st;
  }

  _operatingFunctions = operatingFunctions;
  _operatingModeSupport = operatingModeSupport;
  _specialFeatures = specialFeatures;
  return Status::Ok();
}

Status EE871::probe() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  IdentitySnapshot identity;
  return _readIdentityRaw(identity);
}

Status EE871::recover() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  const bool startedOffline = _driverState == DriverState::OFFLINE;
  (void)_busResetRaw();

  IdentitySnapshot identity;
  _allowOfflineTransfer = true;
  Status st = _readIdentityTracked(identity);
  _allowOfflineTransfer = false;
  if (!st.ok()) {
    if (startedOffline) {
      _driverState = DriverState::OFFLINE;
      if (_consecutiveFailures < _config.offlineThreshold) {
        _consecutiveFailures = _config.offlineThreshold;
      }
    }
    return st;
  }

  (void)_readFeatureFlagsRaw();
  _beginProbeStatus = Status::Ok();
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
  const uint8_t control = cmd::makeControlWrite(cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
  const uint8_t addrHigh = static_cast<uint8_t>(address >> 8);
  const uint8_t addrLow = static_cast<uint8_t>(address & 0xFF);
  return _writeCommandTracked(control, addrHigh, addrLow);
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

Status EE871::_customWriteDirect(uint8_t address, uint8_t value, bool* writeAccepted) {
  if (writeAccepted != nullptr) {
    *writeAccepted = false;
  }
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  const uint8_t control = cmd::makeControlWrite(cmd::MAIN_CUSTOM_WRITE, _config.deviceAddress);
  Status st = _writeCommandTracked(control, address, value, writeAccepted);
  if (!st.ok()) {
    return st;
  }

  delayLongMs(_config, _config.writeDelayMs);

  uint8_t verify = 0;
  st = customRead(address, verify);
  if (!st.ok()) {
    return st;
  }
  if (verify != value) {
    return Status::Error(Err::E2_ERROR, "Write verify failed", verify);
  }
  return Status::Ok();
}

Status EE871::writeMeasurementInterval(uint16_t intervalDeciSeconds) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
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

  bool lowAccepted = false;
  Status st = _writeCommandTracked(control, cmd::CUSTOM_INTERVAL_L, low, &lowAccepted);
  if (!st.ok()) {
    if (lowAccepted) {
      _markPersistentConfigDirty(st);
    }
    return st;
  }
  st = _writeCommandTracked(control, cmd::CUSTOM_INTERVAL_H, high);
  if (!st.ok()) {
    _markPersistentConfigDirty(st);
    return st;
  }

  delayLongMs(_config, _config.intervalWriteDelayMs);

  uint8_t verifyLow = 0;
  uint8_t verifyHigh = 0;
  st = customRead(cmd::CUSTOM_INTERVAL_L, verifyLow);
  if (!st.ok()) {
    _markPersistentConfigDirty(st);
    return st;
  }
  st = customRead(cmd::CUSTOM_INTERVAL_H, verifyHigh);
  if (!st.ok()) {
    _markPersistentConfigDirty(st);
    return st;
  }
  const uint16_t verify = static_cast<uint16_t>(verifyLow) |
                          (static_cast<uint16_t>(verifyHigh) << 8);
  if (verify != intervalDeciSeconds) {
    Status err = Status::Error(Err::E2_ERROR, "Interval verify failed", verify);
    _markPersistentConfigDirty(err);
    return err;
  }
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

Status EE871::readCo2FastSample(Co2ReadResult& out) {
  return _readCo2Sample(Co2ValueKind::Fast, out);
}

Status EE871::readCo2AverageSample(Co2ReadResult& out) {
  return _readCo2Sample(Co2ValueKind::Average, out);
}

Status EE871::_readCo2Sample(Co2ValueKind kind, Co2ReadResult& out) {
  out = Co2ReadResult{};
  out.kind = kind;

  uint16_t ppm = 0;
  Status st = (kind == Co2ValueKind::Fast) ? readCo2Fast(ppm) : readCo2Average(ppm);
  out.ppm = ppm;
  out.valueReadStatus = st;
  if (!st.ok()) {
    return st;
  }

  uint8_t statusByte = 0;
  st = readStatus(statusByte);
  out.statusReadStatus = st;
  if (!st.ok()) {
    return st;
  }
  out.statusByte = statusByte;
  out.statusValid = true;

  if (hasCo2Error(statusByte)) {
    out.co2Error = true;
    if (hasErrorCode()) {
      uint8_t errorCode = 0;
      st = readErrorCode(errorCode);
      out.errorCodeReadStatus = st;
      if (!st.ok()) {
        return st;
      }
      out.errorCode = errorCode;
      out.errorCodeValid = true;
      return Status::Error(Err::CO2_SENSOR_ERROR, "CO2 sensor status error", errorCode);
    }
    return Status::Error(Err::CO2_SENSOR_ERROR, "CO2 sensor status error", statusByte);
  }

  if (ppm > cmd::CO2_PPM_MAX) {
    return Status::Error(Err::OUT_OF_RANGE, "CO2 ppm out of range", ppm);
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

Status EE871::_readControlByteRaw(uint8_t controlByte, uint8_t& data) {
  Status st = e2Start(_config);
  if (!st.ok()) {
    return st;
  }

  uint32_t elapsedUs = 0;
  st = writeByte(_config, controlByte, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }

  bool acked = false;
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  if (!acked) {
    e2Stop(_config);
    return Status::Error(Err::NACK, "Control byte NACK");
  }

  elapsedUs = 0;
  st = readByte(_config, data, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  st = sendAck(_config, true, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }

  uint8_t pec = 0;
  elapsedUs = 0;
  st = readByte(_config, pec, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  st = sendAck(_config, false, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }

  st = e2Stop(_config);
  if (!st.ok()) {
    return st;
  }

  const uint8_t expected = calcPecRead(controlByte, data);
  if (pec != expected) {
    return Status::Error(Err::PEC_MISMATCH, "PEC mismatch", pec);
  }
  return Status::Ok();
}

Status EE871::_readControlByteTracked(uint8_t controlByte, uint8_t& data) {
  if (_initialized && _driverState == DriverState::OFFLINE && !_allowOfflineTransfer) {
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }
  Status st = _readControlByteRaw(controlByte, data);
  return _updateHealth(st);
}

Status EE871::_writeCommandRaw(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte,
                               bool* writeAccepted) {
  if (writeAccepted != nullptr) {
    *writeAccepted = false;
  }

  Status st = e2Start(_config);
  if (!st.ok()) {
    return st;
  }

  uint32_t elapsedUs = 0;
  st = writeByte(_config, controlByte, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  bool acked = false;
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  if (!acked) {
    e2Stop(_config);
    return Status::Error(Err::NACK, "Control byte NACK");
  }

  elapsedUs = 0;
  st = writeByte(_config, addressByte, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  if (!acked) {
    e2Stop(_config);
    return Status::Error(Err::NACK, "Address byte NACK");
  }

  elapsedUs = 0;
  st = writeByte(_config, dataByte, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  if (!acked) {
    e2Stop(_config);
    return Status::Error(Err::NACK, "Data byte NACK");
  }

  const uint8_t pec = calcPecWrite(controlByte, addressByte, dataByte);
  elapsedUs = 0;
  st = writeByte(_config, pec, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    e2Stop(_config);
    return st;
  }
  if (!acked) {
    e2Stop(_config);
    return Status::Error(Err::NACK, "PEC NACK");
  }

  if (writeAccepted != nullptr) {
    *writeAccepted = true;
  }

  st = e2Stop(_config);
  if (!st.ok()) {
    return st;
  }
  return st;
}

Status EE871::_writeCommandTracked(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte,
                                   bool* writeAccepted) {
  if (_initialized && _driverState == DriverState::OFFLINE && !_allowOfflineTransfer) {
    return Status::Error(Err::BUSY, "Driver is offline; call recover()");
  }
  Status st = _writeCommandRaw(controlByte, addressByte, dataByte, writeAccepted);
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
