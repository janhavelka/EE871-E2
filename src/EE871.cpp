/// @file EE871.cpp
/// @brief Implementation of the EE871 E2 driver

#include "EE871/EE871.h"

#include <limits>

namespace EE871 {
namespace {

static constexpr uint32_t kPollStepUs = 5;
static constexpr uint32_t kDataSetupUs = 10;
static constexpr uint32_t kMinClockFrequencyHz = 500;
static constexpr uint32_t kMaxNominalBitTimeUs = 1000000U / kMinClockFrequencyHz;
static constexpr uint32_t kBitsPerByte = 9;
static constexpr uint32_t kMaxBitTimeoutUs = 25000;
static constexpr uint32_t kMaxByteTimeoutUs = 35000;

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

inline void releaseBusLines(const Config& cfg) {
  setSda(cfg, true);
  setScl(cfg, true);
}

inline uint32_t nominalBitTimeUs(const Config& cfg) {
  return kDataSetupUs + static_cast<uint32_t>(cfg.clockHighUs) + cfg.clockLowUs;
}

inline void delayUs(const Config& cfg, uint32_t us, uint32_t* elapsedUs) {
  cfg.delayUs(us, cfg.busUser);
  if (elapsedUs != nullptr) {
    const uint32_t room = std::numeric_limits<uint32_t>::max() - *elapsedUs;
    *elapsedUs = (us > room) ? std::numeric_limits<uint32_t>::max() : (*elapsedUs + us);
  }
}

static Status requireByteBudget(const Config& cfg, uint32_t requiredUs,
                                const uint32_t* elapsedUs) {
  if (elapsedUs == nullptr) {
    return Status::Ok();
  }
  const uint32_t remaining =
      (*elapsedUs < cfg.byteTimeoutUs) ? (cfg.byteTimeoutUs - *elapsedUs) : 0U;
  if (requiredUs > remaining) {
    return Status::Error(Err::TIMEOUT, "Byte timeout", static_cast<int32_t>(*elapsedUs));
  }
  return Status::Ok();
}

static Status waitSclHigh(const Config& cfg, uint32_t timeoutUs, uint32_t* elapsedUs,
                          uint32_t reservedAfterWaitUs = 0) {
  uint32_t waitedUs = 0;
  while (!readScl(cfg)) {
    if (waitedUs >= timeoutUs) {
      return Status::Error(Err::TIMEOUT, "Clock stretch timeout", static_cast<int32_t>(waitedUs));
    }
    uint32_t stepUs = kPollStepUs;
    const uint32_t bitRemainingUs = timeoutUs - waitedUs;
    if (stepUs > bitRemainingUs) {
      stepUs = bitRemainingUs;
    }
    if (elapsedUs != nullptr) {
      const uint32_t remaining =
          (*elapsedUs < cfg.byteTimeoutUs) ? (cfg.byteTimeoutUs - *elapsedUs) : 0U;
      if (remaining <= reservedAfterWaitUs) {
        return Status::Error(Err::TIMEOUT, "Byte timeout", static_cast<int32_t>(*elapsedUs));
      }
      const uint32_t waitRemainingUs = remaining - reservedAfterWaitUs;
      if (stepUs > waitRemainingUs) {
        stepUs = waitRemainingUs;
      }
    }
    delayUs(cfg, stepUs, elapsedUs);
    waitedUs += stepUs;
  }
  return Status::Ok();
}

static Status e2Start(const Config& cfg) {
  releaseBusLines(cfg);
  Status st = waitSclHigh(cfg, cfg.bitTimeoutUs, nullptr);
  if (!st.ok()) {
    releaseBusLines(cfg);
    return st;
  }
  delayUs(cfg, cfg.startHoldUs, nullptr);
  if (!readSda(cfg)) {
    releaseBusLines(cfg);
    return Status::Error(Err::BUS_STUCK, "SDA stuck low before START");
  }
  setSda(cfg, false);
  delayUs(cfg, cfg.startHoldUs, nullptr);
  if (readSda(cfg)) {
    releaseBusLines(cfg);
    return Status::Error(Err::BUS_STUCK, "SDA did not go low for START");
  }
  setScl(cfg, false);
  delayUs(cfg, cfg.clockLowUs, nullptr);
  if (readScl(cfg)) {
    releaseBusLines(cfg);
    return Status::Error(Err::BUS_STUCK, "SCL did not go low for START");
  }
  return Status::Ok();
}

static Status e2Stop(const Config& cfg, uint32_t stretchTimeoutUs) {
  // Establish a complete low phase so cleanup is safe from any transfer stage.
  setScl(cfg, false);
  delayUs(cfg, cfg.clockLowUs, nullptr);
  if (readScl(cfg)) {
    releaseBusLines(cfg);
    return Status::Error(Err::BUS_STUCK, "SCL did not go low for STOP");
  }
  setSda(cfg, false);
  delayUs(cfg, kDataSetupUs, nullptr);
  if (readSda(cfg)) {
    releaseBusLines(cfg);
    return Status::Error(Err::BUS_STUCK, "SDA did not go low for STOP");
  }
  setScl(cfg, true);
  Status st = waitSclHigh(cfg, stretchTimeoutUs, nullptr);
  if (!st.ok()) {
    releaseBusLines(cfg);
    return st;
  }
  delayUs(cfg, cfg.stopHoldUs, nullptr);
  setSda(cfg, true);
  delayUs(cfg, cfg.stopHoldUs, nullptr);
  if (!readSda(cfg)) {
    releaseBusLines(cfg);
    return Status::Error(Err::BUS_STUCK, "SDA did not release after STOP");
  }
  return Status::Ok();
}

static Status finishWithStop(const Config& cfg, const Status& transferStatus,
                             uint8_t controlByte, Status* cleanupStatus = nullptr) {
  // AN1611-1 section 5 permits flash stretching for direct custom writes
  // (0x10 at address 0). Reads and volatile 0x50 pointer updates keep the
  // ordinary timeout, including cleanup after a transfer failure.
  const uint32_t stretchTimeoutUs =
      controlByte == cmd::makeControlWrite(cmd::MAIN_CUSTOM_WRITE, cfg.deviceAddress)
          ? cfg.flashStretchTimeoutUs : cfg.bitTimeoutUs;
  const Status stopStatus = e2Stop(cfg, stretchTimeoutUs);
  if (cleanupStatus != nullptr) *cleanupStatus = stopStatus;
  return transferStatus.ok() ? stopStatus : transferStatus;
}

static bool permitsReadNackRetry(uint8_t controlByte) {
  if ((controlByte & cmd::RW_READ) == 0U) return false;
  const uint8_t mainCommand = controlByte >> cmd::MAIN_SHIFT;
  return mainCommand == cmd::MAIN_STATUS || mainCommand == cmd::MAIN_MV3_LO ||
         mainCommand == cmd::MAIN_MV3_HI || mainCommand == cmd::MAIN_MV4_LO ||
         mainCommand == cmd::MAIN_MV4_HI;
}

static void incrementSaturated(uint32_t& value) {
  if (value != std::numeric_limits<uint32_t>::max()) ++value;
}

static Status finishClockLow(const Config& cfg, uint32_t* elapsedUs) {
  setScl(cfg, false);
  delayUs(cfg, cfg.clockLowUs, elapsedUs);
  if (readScl(cfg)) {
    releaseBusLines(cfg);
    return Status::Error(Err::BUS_STUCK, "SCL did not go low");
  }
  return Status::Ok();
}

static Status writeBit(const Config& cfg, bool bit, uint32_t* elapsedUs) {
  Status st = requireByteBudget(cfg, nominalBitTimeUs(cfg), elapsedUs);
  if (!st.ok()) {
    return st;
  }
  // SCL is already low from previous bit or START
  setSda(cfg, bit);
  delayUs(cfg, kDataSetupUs, elapsedUs);
  setScl(cfg, true);
  st = waitSclHigh(cfg, cfg.bitTimeoutUs, elapsedUs,
                   static_cast<uint32_t>(cfg.clockHighUs) + cfg.clockLowUs);
  if (!st.ok()) {
    return st;
  }
  delayUs(cfg, cfg.clockHighUs, elapsedUs);
  return finishClockLow(cfg, elapsedUs);
}

static Status readBit(const Config& cfg, bool& bit, uint32_t* elapsedUs) {
  Status st = requireByteBudget(cfg, nominalBitTimeUs(cfg), elapsedUs);
  if (!st.ok()) {
    return st;
  }
  // SCL is already low from previous bit
  setSda(cfg, true);  // Release SDA for slave to drive
  delayUs(cfg, kDataSetupUs, elapsedUs);
  setScl(cfg, true);
  st = waitSclHigh(cfg, cfg.bitTimeoutUs, elapsedUs,
                   static_cast<uint32_t>(cfg.clockHighUs) + cfg.clockLowUs);
  if (!st.ok()) {
    return st;
  }
  const uint32_t sampleDelay = cfg.clockHighUs / 2;
  delayUs(cfg, sampleDelay, elapsedUs);
  bit = readSda(cfg);
  delayUs(cfg, cfg.clockHighUs - sampleDelay, elapsedUs);
  return finishClockLow(cfg, elapsedUs);
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
  Status st = requireByteBudget(cfg, nominalBitTimeUs(cfg), elapsedUs);
  if (!st.ok()) {
    return st;
  }
  // SCL is already low from last data bit
  setSda(cfg, true);  // Release SDA for slave to drive ACK
  delayUs(cfg, kDataSetupUs, elapsedUs);
  setScl(cfg, true);
  st = waitSclHigh(cfg, cfg.bitTimeoutUs, elapsedUs,
                   static_cast<uint32_t>(cfg.clockHighUs) + cfg.clockLowUs);
  if (!st.ok()) {
    return st;
  }
  const uint32_t sampleDelay = cfg.clockHighUs / 2;
  delayUs(cfg, sampleDelay, elapsedUs);
  acked = !readSda(cfg);  // ACK = SDA low
  delayUs(cfg, cfg.clockHighUs - sampleDelay, elapsedUs);
  return finishClockLow(cfg, elapsedUs);
}

static Status sendAck(const Config& cfg, bool ack, uint32_t* elapsedUs) {
  Status st = requireByteBudget(cfg, nominalBitTimeUs(cfg), elapsedUs);
  if (!st.ok()) {
    return st;
  }
  // SCL is already low from last data bit
  setSda(cfg, !ack);  // ACK = SDA low, NACK = SDA high
  delayUs(cfg, kDataSetupUs, elapsedUs);
  setScl(cfg, true);
  st = waitSclHigh(cfg, cfg.bitTimeoutUs, elapsedUs,
                   static_cast<uint32_t>(cfg.clockHighUs) + cfg.clockLowUs);
  if (!st.ok()) {
    return st;
  }
  delayUs(cfg, cfg.clockHighUs, elapsedUs);
  st = finishClockLow(cfg, elapsedUs);
  setSda(cfg, true);  // Release SDA
  return st;
}

static uint8_t calcPecRead(uint8_t controlByte, uint8_t dataByte) {
  return static_cast<uint8_t>((controlByte + dataByte) & 0xFF);
}

static uint8_t calcPecWrite(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte) {
  return static_cast<uint8_t>((controlByte + addressByte + dataByte) & 0xFF);
}

static void sleepMs(const Config& cfg, uint32_t delayMs) {
  for (uint32_t i = 0; i < delayMs; ++i) {
    cfg.delayUs(1000, cfg.busUser);
  }
}

} // namespace

Status EE871::begin(const Config& config) {
  // Prevent double-init without explicit end()
  if (_initialized) {
    return Status::Error(Err::ALREADY_INITIALIZED, "Call end() first");
  }

  _resetStoppedState();

  if (config.readNackRetries > cmd::READ_NACK_RETRIES_MAX) {
    return Status::Error(Err::INVALID_CONFIG, "Read NACK retries must be <=3");
  }

  if (config.setScl == nullptr || config.setSda == nullptr ||
      config.readScl == nullptr || config.readSda == nullptr ||
      config.delayUs == nullptr) {
    return Status::Error(Err::INVALID_CONFIG, "Missing E2 callbacks");
  }
  if (config.deviceAddress > cmd::DEVICE_ADDRESS_MAX) {
    return Status::Error(Err::INVALID_CONFIG, "Invalid device address");
  }
  if (config.clockLowUs < 100 || config.clockHighUs < 100) {
    return Status::Error(Err::INVALID_CONFIG, "Clock timing below spec");
  }
  const uint32_t configuredBitTimeUs = nominalBitTimeUs(config);
  if (configuredBitTimeUs > kMaxNominalBitTimeUs) {
    return Status::Error(Err::INVALID_CONFIG, "Clock frequency below spec");
  }
  if (config.startHoldUs < 4 || config.stopHoldUs < 4) {
    return Status::Error(Err::INVALID_CONFIG, "Start/stop hold below spec");
  }
  if (config.bitTimeoutUs == 0 || config.byteTimeoutUs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "Timeouts must be non-zero");
  }
  if (config.bitTimeoutUs > kMaxBitTimeoutUs ||
      config.byteTimeoutUs > kMaxByteTimeoutUs) {
    return Status::Error(Err::INVALID_CONFIG, "Timeout exceeds E2 specification");
  }
  if (config.byteTimeoutUs < config.bitTimeoutUs) {
    return Status::Error(Err::INVALID_CONFIG, "byteTimeoutUs must be >= bitTimeoutUs");
  }
  const uint32_t nominalByteTimeUs = kBitsPerByte * configuredBitTimeUs;
  if (nominalByteTimeUs >= config.byteTimeoutUs) {
    return Status::Error(Err::INVALID_CONFIG, "byteTimeoutUs must exceed nominal byte time");
  }
  if (config.writeDelayMs > cmd::WRITE_DELAY_MAX_MS) {
    return Status::Error(Err::INVALID_CONFIG, "writeDelayMs exceeds safe limit");
  }
  if (config.intervalWriteDelayMs > cmd::INTERVAL_WRITE_DELAY_MAX_MS) {
    return Status::Error(Err::INVALID_CONFIG, "intervalWriteDelayMs exceeds safe limit");
  }
  // AN1611-1 permits up to 300 ms of clock extension during interval commits.
  if (config.flashStretchTimeoutUs < 300000U ||
      config.flashStretchTimeoutUs > cmd::WRITE_DELAY_MAX_MS * 1000U ||
      config.flashStretchTimeoutUs > cmd::INTERVAL_WRITE_DELAY_MAX_MS * 1000U) {
    return Status::Error(Err::INVALID_CONFIG, "Flash stretch timeout outside safe limits");
  }

  Config normalized = config;
  if (normalized.offlineThreshold == 0) {
    normalized.offlineThreshold = 1;
  }
  _config = normalized;

  // Check bus is idle before probing
  if (!readScl(_config) || !readSda(_config)) {
    Status st = _busResetRaw();
    if (!st.ok()) {
      _resetStoppedState();
      return st;
    }
  }

  Status st = _validateIdentityRaw();
  if (!st.ok()) {
    _resetStoppedState();
    return st;
  }

  // Read feature flags into locals so a partial read cannot update the cache.
  uint8_t operatingFunctions = 0;
  uint8_t operatingModeSupport = 0;
  uint8_t specialFeatures = 0;
  st = _readFeatureFlagsRaw(operatingFunctions, operatingModeSupport,
                            specialFeatures);
  if (!st.ok()) {
    _resetStoppedState();
    return st;
  }
  _operatingFunctions = operatingFunctions;
  _operatingModeSupport = operatingModeSupport;
  _specialFeatures = specialFeatures;

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
  out.readRetry = _readRetry;
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
  _operatingFunctions = 0;
  _operatingModeSupport = 0;
  _specialFeatures = 0;
  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;
  _readRetry = ReadRetryDiagnostics{};
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

  return _validateIdentityRaw();
}

Status EE871::recover() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  return _recoverTracked();
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

  sleepMs(_config, _config.writeDelayMs);

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

  // Validate range: 15.0s - 3600.0s (150 - 36000 deciseconds)
  if (intervalDeciSeconds < cmd::INTERVAL_MIN_DECISEC ||
      intervalDeciSeconds > cmd::INTERVAL_MAX_DECISEC) {
    return Status::Error(Err::OUT_OF_RANGE, "Interval must be 150-36000 (15-3600s)",
                         intervalDeciSeconds);
  }
  if (!hasGlobalInterval()) {
    return Status::Error(Err::NOT_SUPPORTED, "Global interval not supported");
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

  sleepMs(_config, _config.intervalWriteDelayMs);

  uint8_t verifyBuf[2] = {0};
  st = customRead(cmd::CUSTOM_INTERVAL_L, verifyBuf, 2);
  if (!st.ok()) {
    _markPersistentConfigDirty(st);
    return st;
  }
  const uint16_t verify = static_cast<uint16_t>(verifyBuf[0]) |
                          (static_cast<uint16_t>(verifyBuf[1]) << 8);
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

// ============================================================================
// Firmware / Spec Version
// ============================================================================

Status EE871::readFirmwareVersion(uint8_t& main, uint8_t& sub) {
  uint8_t buf[2] = {0};
  Status st = customRead(cmd::CUSTOM_FW_VERSION_MAIN, buf, 2);
  if (!st.ok()) {
    return st;
  }
  main = buf[0];
  sub = buf[1];
  return Status::Ok();
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
  if (address > cmd::BUS_ADDRESS_MAX) {
    return Status::Error(Err::OUT_OF_RANGE, "Address must be 0-7", address);
  }
  if (!hasAddressConfig()) {
    return Status::Error(Err::NOT_SUPPORTED, "Address config not supported");
  }
  return customWrite(cmd::CUSTOM_BUS_ADDRESS, address);
}

// ============================================================================
// Measurement Interval
// ============================================================================

Status EE871::readMeasurementInterval(uint16_t& intervalDeciSeconds) {
  // Interval can always be read, guard only applies to write
  uint8_t buf[2] = {0};
  Status st = customRead(cmd::CUSTOM_INTERVAL_L, buf, 2);
  if (!st.ok()) {
    return st;
  }
  intervalDeciSeconds = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
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
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  if (!hasLowPowerMode() && !hasE2Priority()) {
    return Status::Error(Err::NOT_SUPPORTED, "Operating mode not supported");
  }

  uint8_t value = 0;
  Status st = customRead(cmd::CUSTOM_OPERATING_MODE, value);
  if (!st.ok()) {
    return st;
  }
  if ((value & static_cast<uint8_t>(~0x03U)) != 0U) {
    return Status::Error(Err::OUT_OF_RANGE, "Invalid operating mode bits", value);
  }
  mode = value;
  return Status::Ok();
}

Status EE871::writeOperatingMode(uint8_t mode) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  // Only bits 0 and 1 are valid.
  if (mode > 0x03) {
    return Status::Error(Err::OUT_OF_RANGE, "Invalid mode bits", mode);
  }
  if (!hasLowPowerMode() && !hasE2Priority()) {
    return Status::Error(Err::NOT_SUPPORTED, "Operating mode not supported");
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
  uint8_t buf[2] = {0};
  Status st = customRead(cmd::CUSTOM_CO2_OFFSET_L, buf, 2);
  if (!st.ok()) {
    return st;
  }
  offset = static_cast<int16_t>(static_cast<uint16_t>(buf[0]) |
                                (static_cast<uint16_t>(buf[1]) << 8));
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
  uint8_t buf[2] = {0};
  Status st = customRead(cmd::CUSTOM_CO2_GAIN_L, buf, 2);
  if (!st.ok()) {
    return st;
  }
  gain = static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
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
  // Clock out 9+ pulses with SDA high to reset slave state machine
  setSda(_config, true);
  for (uint8_t i = 0; i < cmd::BUS_RESET_CLOCKS; ++i) {
    setScl(_config, false);
    delayUs(_config, _config.clockLowUs, nullptr);
    if (readScl(_config)) {
      releaseBusLines(_config);
      return Status::Error(Err::BUS_STUCK, "SCL did not go low during reset");
    }
    setScl(_config, true);
    Status st = waitSclHigh(_config, _config.flashStretchTimeoutUs, nullptr);
    if (!st.ok()) {
      releaseBusLines(_config);
      return Status::Error(Err::BUS_STUCK, "SCL stuck during reset");
    }
    delayUs(_config, _config.clockHighUs, nullptr);
  }

  // Generate a stretch-aware STOP; e2Stop establishes its own full low phase.
  Status stopStatus = e2Stop(_config, _config.flashStretchTimeoutUs);
  if (!stopStatus.ok()) {
    if (stopStatus.code == Err::TIMEOUT) {
      return Status::Error(Err::BUS_STUCK, "SCL stuck during reset STOP");
    }
    return stopStatus;
  }

  // Verify bus is now idle
  if (!readScl(_config) || !readSda(_config)) {
    return Status::Error(Err::BUS_STUCK, "Bus stuck after reset");
  }

  return Status::Ok();
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

Status EE871::_validateIdentityRaw() {
  uint8_t groupLow = 0;
  uint8_t groupHigh = 0;
  uint8_t subgroup = 0;
  uint8_t availableMeasurements = 0;

  Status st = _readControlByteRaw(
      cmd::makeControlRead(cmd::MAIN_TYPE_LO, _config.deviceAddress), groupLow);
  if (!st.ok()) {
    return st;
  }
  st = _readControlByteRaw(
      cmd::makeControlRead(cmd::MAIN_TYPE_HI, _config.deviceAddress), groupHigh);
  if (!st.ok()) {
    return st;
  }
  const uint16_t group =
      static_cast<uint16_t>(groupLow) | (static_cast<uint16_t>(groupHigh) << 8);
  if (group != cmd::SENSOR_GROUP_ID) {
    return Status::Error(Err::NOT_SUPPORTED, "Unexpected group id", group);
  }

  st = _readControlByteRaw(
      cmd::makeControlRead(cmd::MAIN_TYPE_SUB, _config.deviceAddress), subgroup);
  if (!st.ok()) {
    return st;
  }
  if (subgroup != cmd::SENSOR_SUBGROUP_ID) {
    return Status::Error(Err::NOT_SUPPORTED, "Unexpected subgroup id", subgroup);
  }

  st = _readControlByteRaw(
      cmd::makeControlRead(cmd::MAIN_AVAIL_MEAS, _config.deviceAddress),
      availableMeasurements);
  if (!st.ok()) {
    return st;
  }
  if ((availableMeasurements & cmd::AVAILABLE_MEAS_MASK) == 0U) {
    return Status::Error(Err::NOT_SUPPORTED, "CO2 measurement not available",
                         availableMeasurements);
  }
  return Status::Ok();
}

Status EE871::_readFeatureFlagsRaw(uint8_t& operatingFunctions,
                                   uint8_t& operatingModeSupport,
                                   uint8_t& specialFeatures) {
  const uint8_t pointerControl =
      cmd::makeControlWrite(cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
  Status st = _writeCommandRaw(pointerControl, 0x00,
                               cmd::CUSTOM_OPERATING_FUNCTIONS);
  if (!st.ok()) {
    return st;
  }

  const uint8_t readControl =
      cmd::makeControlRead(cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
  st = _readControlByteRaw(readControl, operatingFunctions);
  if (st.ok()) {
    st = _readControlByteRaw(readControl, operatingModeSupport);
  }
  if (st.ok()) {
    st = _readControlByteRaw(readControl, specialFeatures);
  }
  return st;
}

Status EE871::_recoverTracked() {
  Status st = _busResetRaw();
  if (st.ok()) {
    st = _validateIdentityRaw();
  }
  uint8_t operatingFunctions = 0;
  uint8_t operatingModeSupport = 0;
  uint8_t specialFeatures = 0;
  if (st.ok()) {
    st = _readFeatureFlagsRaw(operatingFunctions, operatingModeSupport,
                              specialFeatures);
  }
  if (st.ok()) {
    _operatingFunctions = operatingFunctions;
    _operatingModeSupport = operatingModeSupport;
    _specialFeatures = specialFeatures;
  } else {
    _operatingFunctions = 0;
    _operatingModeSupport = 0;
    _specialFeatures = 0;
    // A failed compatibility check must require another explicit recovery.
    if (_consecutiveFailures < _config.offlineThreshold) {
      _consecutiveFailures = _config.offlineThreshold;
    }
  }
  return _updateHealth(st);
}

Status EE871::_offlineStatus() const {
  return Status::Error(_lastError.ok() ? Err::E2_ERROR : _lastError.code,
                       "Driver offline; call recover()", _lastError.detail);
}

Status EE871::_readControlByteRaw(uint8_t controlByte, uint8_t& data,
                                  ReadAttemptInfo* attempt) {
  if (attempt != nullptr) *attempt = ReadAttemptInfo{};
  Status st = e2Start(_config);
  if (!st.ok()) {
    return st;
  }

  uint32_t elapsedUs = 0;
  st = writeByte(_config, controlByte, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }

  bool acked = false;
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  if (!acked) {
    if (attempt != nullptr) attempt->controlNack = true;
    return finishWithStop(_config, Status::Error(Err::NACK, "Control byte NACK"),
                          controlByte, attempt != nullptr ? &attempt->stopStatus : nullptr);
  }

  elapsedUs = 0;
  st = readByte(_config, data, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  st = sendAck(_config, true, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }

  uint8_t pec = 0;
  elapsedUs = 0;
  st = readByte(_config, pec, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  st = sendAck(_config, false, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }

  const uint8_t expected = calcPecRead(controlByte, data);
  return finishWithStop(_config, pec == expected
      ? Status::Ok() : Status::Error(Err::PEC_MISMATCH, "PEC mismatch", pec),
      controlByte);
}

Status EE871::_readControlByteTracked(uint8_t controlByte, uint8_t& data) {
  if (_driverState == DriverState::OFFLINE) {
    return _offlineStatus();
  }
  if (!permitsReadNackRetry(controlByte)) {
    return _updateHealth(_readControlByteRaw(controlByte, data));
  }

  Status st = Status::Ok();
  bool sawNack = false;
  for (uint8_t retry = 0; retry <= _config.readNackRetries; ++retry) {
    if (retry != 0U) incrementSaturated(_readRetry.retries);
    ReadAttemptInfo attempt;
    st = _readControlByteRaw(controlByte, data, &attempt);
    if (attempt.controlNack) {
      incrementSaturated(_readRetry.controlNacks);
      if (!sawNack) {
        _readRetry.lastControlByte = controlByte;
        _readRetry.lastCleanupError = Status::Ok();
        _readRetry.cleanupBlocked = false;
        _readRetry.retryVetoed = false;
        _readRetry.lastRecovered = false;
        sawNack = true;
      }
    }
    if (sawNack) {
      _readRetry.lastRetriesUsed = retry;
      _readRetry.lastError = st;
    }
    if (st.ok()) {
      if (retry != 0U) {
        incrementSaturated(_readRetry.recovered);
        _readRetry.lastRecovered = true;
      }
      break;
    }
    if (!attempt.controlNack) break;
    if (!attempt.stopStatus.ok()) {
      _readRetry.cleanupBlocked = true;
      _readRetry.lastCleanupError = attempt.stopStatus;
      break;
    }
    if (retry == _config.readNackRetries) {
      if (retry != 0U) incrementSaturated(_readRetry.exhausted);
      break;
    }

    // HAL failures/deadlines are application-owned; the optional guard lets
    // that owner veto further bus traffic without replacing the original NACK.
    if (_config.allowReadRetry != nullptr && !_config.allowReadRetry(_config.busUser)) {
      _readRetry.retryVetoed = true;
      break;
    }
    Status idle = checkBusIdle();
    if (!idle.ok()) {
      _readRetry.cleanupBlocked = true;
      _readRetry.lastCleanupError = idle;
      break;
    }
    delayUs(_config, cmd::READ_NACK_RETRY_DELAY_US, nullptr);
    if (_config.allowReadRetry != nullptr && !_config.allowReadRetry(_config.busUser)) {
      _readRetry.retryVetoed = true;
      break;
    }
    idle = checkBusIdle();
    if (!idle.ok()) {
      _readRetry.cleanupBlocked = true;
      _readRetry.lastCleanupError = idle;
      break;
    }
    // The idle read callbacks can themselves latch an application HAL error.
    if (_config.allowReadRetry != nullptr && !_config.allowReadRetry(_config.busUser)) {
      _readRetry.retryVetoed = true;
      break;
    }
  }
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
    return finishWithStop(_config, st, controlByte);
  }
  bool acked = false;
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  if (!acked) {
    return finishWithStop(_config, Status::Error(Err::NACK, "Control byte NACK"),
                          controlByte);
  }

  elapsedUs = 0;
  st = writeByte(_config, addressByte, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  if (!acked) {
    return finishWithStop(_config, Status::Error(Err::NACK, "Address byte NACK"),
                          controlByte);
  }

  elapsedUs = 0;
  st = writeByte(_config, dataByte, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  if (!acked) {
    return finishWithStop(_config, Status::Error(Err::NACK, "Data byte NACK"),
                          controlByte);
  }

  const uint8_t pec = calcPecWrite(controlByte, addressByte, dataByte);
  elapsedUs = 0;
  st = writeByte(_config, pec, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  st = readAck(_config, acked, &elapsedUs);
  if (!st.ok()) {
    return finishWithStop(_config, st, controlByte);
  }
  if (!acked) {
    return finishWithStop(_config, Status::Error(Err::NACK, "PEC NACK"), controlByte);
  }

  if (writeAccepted != nullptr) {
    *writeAccepted = true;
  }

  return finishWithStop(_config, Status::Ok(), controlByte);
}

Status EE871::_writeCommandTracked(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte,
                                   bool* writeAccepted) {
  if (_driverState == DriverState::OFFLINE) {
    if (writeAccepted != nullptr) {
      *writeAccepted = false;
    }
    return _offlineStatus();
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
