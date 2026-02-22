/// @file EE871.cpp
/// @brief Implementation of the EE871 E2 driver

#include "EE871/EE871.h"

#include <limits>

namespace ee871 {
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
    *elapsedUs += us;
  }
}

static Status waitSclHigh(const Config& cfg, uint32_t* elapsedUs) {
  uint32_t waitedUs = 0;
  while (!readScl(cfg)) {
    if (waitedUs >= cfg.bitTimeoutUs) {
      return Status::Error(Err::TIMEOUT, "Clock stretch timeout", static_cast<int32_t>(waitedUs));
    }
    if (elapsedUs != nullptr && (*elapsedUs + kPollStepUs) > cfg.byteTimeoutUs) {
      return Status::Error(Err::TIMEOUT, "Byte timeout", static_cast<int32_t>(*elapsedUs));
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
  if (config.startHoldUs < 4 || config.stopHoldUs < 4) {
    return Status::Error(Err::INVALID_CONFIG, "Start/stop hold below spec");
  }
  if (config.bitTimeoutUs == 0 || config.byteTimeoutUs == 0) {
    return Status::Error(Err::INVALID_CONFIG, "Timeouts must be non-zero");
  }
  if (config.byteTimeoutUs < config.bitTimeoutUs) {
    return Status::Error(Err::INVALID_CONFIG, "byteTimeoutUs must be >= bitTimeoutUs");
  }
  if (config.offlineThreshold == 0) {
    return Status::Error(Err::INVALID_CONFIG, "offlineThreshold must be > 0");
  }
  if (config.writeDelayMs > cmd::WRITE_DELAY_MAX_MS) {
    return Status::Error(Err::INVALID_CONFIG, "writeDelayMs exceeds safe limit");
  }
  if (config.intervalWriteDelayMs > cmd::INTERVAL_WRITE_DELAY_MAX_MS) {
    return Status::Error(Err::INVALID_CONFIG, "intervalWriteDelayMs exceeds safe limit");
  }

  _config = config;
  _initialized = false;
  _driverState = DriverState::UNINIT;
  _nowMs = 0;
  _lastOkMs = 0;
  _lastErrorMs = 0;
  _lastError = Status::Ok();
  _consecutiveFailures = 0;
  _totalFailures = 0;
  _totalSuccess = 0;

  // Check bus is idle before probing
  if (!readScl(_config) || !readSda(_config)) {
    // Attempt bus reset - clock out pulses with SDA high
    setSda(_config, true);
    for (uint8_t i = 0; i < cmd::BUS_RESET_CLOCKS; ++i) {
      setScl(_config, false);
      delayUs(_config, _config.clockLowUs, nullptr);
      setScl(_config, true);
      // Wait for SCL to actually rise (handle clock stretching)
      uint32_t waited = 0;
      while (!readScl(_config) && waited < _config.bitTimeoutUs) {
        delayUs(_config, kPollStepUs, nullptr);
        waited += kPollStepUs;
      }
      delayUs(_config, _config.clockHighUs, nullptr);
    }
    // Generate STOP condition to leave bus in known state
    setScl(_config, false);
    delayUs(_config, _config.clockLowUs, nullptr);
    setSda(_config, false);
    delayUs(_config, kDataSetupUs, nullptr);
    setScl(_config, true);
    delayUs(_config, _config.stopHoldUs, nullptr);
    setSda(_config, true);
    delayUs(_config, _config.stopHoldUs, nullptr);
    // Check again
    if (!readScl(_config) || !readSda(_config)) {
      return Status::Error(Err::BUS_STUCK, "Bus stuck after reset");
    }
  }

  uint8_t low = 0;
  uint8_t high = 0;
  const uint8_t controlLow = cmd::makeControlRead(cmd::MAIN_TYPE_LO, _config.deviceAddress);
  const uint8_t controlHigh = cmd::makeControlRead(cmd::MAIN_TYPE_HI, _config.deviceAddress);

  Status st = _readControlByteRaw(controlLow, low);
  if (!st.ok()) {
    return st;
  }
  st = _readControlByteRaw(controlHigh, high);
  if (!st.ok()) {
    return st;
  }

  const uint16_t group = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  if (group != cmd::SENSOR_GROUP_ID) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Unexpected group id", group);
  }

  // Cache feature flags for guards
  // Use raw reads since we're not fully initialized yet
  _operatingFunctions = 0;
  _operatingModeSupport = 0;
  _specialFeatures = 0;

  // Set pointer to 0x07
  const uint8_t ptrControl = cmd::makeControlWrite(cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
  st = _writeCommandRaw(ptrControl, 0x00, cmd::CUSTOM_OPERATING_FUNCTIONS);
  if (st.ok()) {
    const uint8_t readControl = cmd::makeControlRead(cmd::MAIN_CUSTOM_PTR, _config.deviceAddress);
    // Read 0x07, 0x08, 0x09 in sequence (auto-increment)
    st = _readControlByteRaw(readControl, _operatingFunctions);
    if (st.ok()) {
      st = _readControlByteRaw(readControl, _operatingModeSupport);
    }
    if (st.ok()) {
      st = _readControlByteRaw(readControl, _specialFeatures);
    }
  }
  // If feature read fails, continue with defaults (all features disabled)
  // This is non-fatal - the device still works, just with guards active

  _initialized = true;
  _driverState = DriverState::READY;
  return Status::Ok();
}

void EE871::tick(uint32_t nowMs) {
  _nowMs = nowMs;
}

void EE871::end() {
  _initialized = false;
  _driverState = DriverState::UNINIT;
}

Status EE871::probe() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  uint8_t low = 0;
  uint8_t high = 0;
  const uint8_t controlLow = cmd::makeControlRead(cmd::MAIN_TYPE_LO, _config.deviceAddress);
  const uint8_t controlHigh = cmd::makeControlRead(cmd::MAIN_TYPE_HI, _config.deviceAddress);

  Status st = _readControlByteRaw(controlLow, low);
  if (!st.ok()) {
    return st;
  }
  st = _readControlByteRaw(controlHigh, high);
  if (!st.ok()) {
    return st;
  }

  const uint16_t group = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  if (group != cmd::SENSOR_GROUP_ID) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Unexpected group id", group);
  }
  return Status::Ok();
}

Status EE871::recover() {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }

  // Attempt bus reset first to clear any stuck state (no health tracking)
  // Ignore result - still try to probe even if bus reset reports stuck
  busReset();

  // Probe device (tracked - updates health state)
  uint16_t group = 0;
  Status st = readGroup(group);
  if (st.ok()) {
    return Status::Ok();
  }
  return st;
}

Status EE871::readControlByte(uint8_t mainCommandNibble, uint8_t& data) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  if (mainCommandNibble > 0x0F) {
    return Status::Error(Err::INVALID_PARAM, "Invalid main command");
  }
  const uint8_t control = cmd::makeControlRead(mainCommandNibble, _config.deviceAddress);
  return _readControlByteTracked(control, data);
}

Status EE871::readU16(uint8_t mainCommandLow, uint8_t mainCommandHigh, uint16_t& value) {
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

  const uint8_t control = cmd::makeControlWrite(cmd::MAIN_CUSTOM_WRITE, _config.deviceAddress);
  Status st = _writeCommandTracked(control, address, value);
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

  Status st = _writeCommandTracked(control, cmd::CUSTOM_INTERVAL_L, low);
  if (!st.ok()) {
    return st;
  }
  st = _writeCommandTracked(control, cmd::CUSTOM_INTERVAL_H, high);
  if (!st.ok()) {
    return st;
  }

  sleepMs(_config, _config.intervalWriteDelayMs);

  uint8_t verifyLow = 0;
  uint8_t verifyHigh = 0;
  st = customRead(cmd::CUSTOM_INTERVAL_L, verifyLow);
  if (!st.ok()) {
    return st;
  }
  st = customRead(cmd::CUSTOM_INTERVAL_H, verifyHigh);
  if (!st.ok()) {
    return st;
  }
  const uint16_t verify = static_cast<uint16_t>(verifyLow) |
                          (static_cast<uint16_t>(verifyHigh) << 8);
  if (verify != intervalDeciSeconds) {
    return Status::Error(Err::E2_ERROR, "Interval verify failed", verify);
  }
  return Status::Ok();
}

Status EE871::readGroup(uint16_t& group) {
  Status st = readU16(cmd::MAIN_TYPE_LO, cmd::MAIN_TYPE_HI, group);
  if (!st.ok()) {
    return st;
  }
  if (group != cmd::SENSOR_GROUP_ID) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Unexpected group id", group);
  }
  return Status::Ok();
}

Status EE871::readSubgroup(uint8_t& subgroup) {
  Status st = readControlByte(cmd::MAIN_TYPE_SUB, subgroup);
  if (!st.ok()) {
    return st;
  }
  if (subgroup != cmd::SENSOR_SUBGROUP_ID) {
    return Status::Error(Err::DEVICE_NOT_FOUND, "Unexpected subgroup id", subgroup);
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
  if (buf == nullptr) {
    return Status::Error(Err::INVALID_PARAM, "Null buffer");
  }
  if (!hasSerialNumber()) {
    return Status::Error(Err::NOT_SUPPORTED, "Serial number not supported");
  }
  return customRead(cmd::CUSTOM_SERIAL_START, buf, cmd::CUSTOM_SERIAL_LEN);
}

Status EE871::readPartName(uint8_t* buf) {
  if (buf == nullptr) {
    return Status::Error(Err::INVALID_PARAM, "Null buffer");
  }
  if (!hasPartName()) {
    return Status::Error(Err::NOT_SUPPORTED, "Part name not supported");
  }
  return customRead(cmd::CUSTOM_PART_NAME_START, buf, cmd::CUSTOM_PART_NAME_LEN);
}

Status EE871::writePartName(const uint8_t* buf) {
  if (buf == nullptr) {
    return Status::Error(Err::INVALID_PARAM, "Null buffer");
  }
  if (!hasPartName()) {
    return Status::Error(Err::NOT_SUPPORTED, "Part name not supported");
  }
  for (uint8_t i = 0; i < cmd::CUSTOM_PART_NAME_LEN; ++i) {
    Status st = customWrite(cmd::CUSTOM_PART_NAME_START + i, buf[i]);
    if (!st.ok()) {
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
  // Check if requested mode bits are supported
  if ((mode & cmd::OPERATING_MODE_MEASUREMODE_MASK) && !hasLowPowerMode()) {
    return Status::Error(Err::NOT_SUPPORTED, "Low power mode not supported");
  }
  if ((mode & cmd::OPERATING_MODE_E2_PRIORITY_MASK) && !hasE2Priority()) {
    return Status::Error(Err::NOT_SUPPORTED, "E2 priority not supported");
  }
  // Only bits 0 and 1 are valid
  if (mode > 0x03) {
    return Status::Error(Err::OUT_OF_RANGE, "Invalid mode bits", mode);
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
  Status st = customWrite(cmd::CUSTOM_CO2_OFFSET_L, static_cast<uint8_t>(raw & 0xFF));
  if (!st.ok()) {
    return st;
  }
  return customWrite(cmd::CUSTOM_CO2_OFFSET_H, static_cast<uint8_t>(raw >> 8));
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
  Status st = customWrite(cmd::CUSTOM_CO2_GAIN_L, static_cast<uint8_t>(gain & 0xFF));
  if (!st.ok()) {
    return st;
  }
  return customWrite(cmd::CUSTOM_CO2_GAIN_H, static_cast<uint8_t>(gain >> 8));
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

  // Clock out 9+ pulses with SDA high to reset slave state machine
  setSda(_config, true);
  for (uint8_t i = 0; i < cmd::BUS_RESET_CLOCKS; ++i) {
    setScl(_config, false);
    delayUs(_config, _config.clockLowUs, nullptr);
    setScl(_config, true);
    // Wait for clock to rise (slave might stretch)
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

  // Generate STOP condition
  setScl(_config, false);
  delayUs(_config, _config.clockLowUs, nullptr);
  setSda(_config, false);
  delayUs(_config, kDataSetupUs, nullptr);
  setScl(_config, true);
  delayUs(_config, _config.stopHoldUs, nullptr);
  setSda(_config, true);
  delayUs(_config, _config.stopHoldUs, nullptr);

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
  Status st = _readControlByteRaw(controlByte, data);
  return _updateHealth(st);
}

Status EE871::_writeCommandRaw(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte) {
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

  st = e2Stop(_config);
  if (!st.ok()) {
    return st;
  }
  return st;
}

Status EE871::_writeCommandTracked(uint8_t controlByte, uint8_t addressByte, uint8_t dataByte) {
  Status st = _writeCommandRaw(controlByte, addressByte, dataByte);
  return _updateHealth(st);
}

Status EE871::_updateHealth(const Status& st) {
  if (!_initialized) {
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

} // namespace ee871
