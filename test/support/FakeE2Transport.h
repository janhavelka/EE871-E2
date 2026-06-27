/// @file FakeE2Transport.h
/// @brief Deterministic native E2 callback-boundary fake for runtime fault tests.
#pragma once

#include <cstddef>
#include <cstdint>

#include "EE871/CommandTable.h"
#include "EE871/Config.h"

namespace EE871Test {

class FakeE2Transport {
public:
  FakeE2Transport() { reset(); }

  void reset() {
    _masterSclReleased = true;
    _masterSdaReleased = true;
    _phase = Phase::IDLE;
    _bitCount = 0;
    _byte = 0;
    _control = 0;
    _address = 0;
    _data = 0;
    _pec = 0;
    _responseData = 0;
    _responsePec = 0;
    _slaveSda = true;
    _skipNextFalling = false;
    _customPointer = 0;
    _elapsedUs = 0;
    _delayCalls = 0;
    _delayMsCalls = 0;
    _delayMsTotal = 0;
    _maxDelayMsSlice = 0;
    _yieldCalls = 0;
    _totalBusTransactions = 0;
    _controlReadTraceLen = 0;
    for (size_t i = 0; i < 16; ++i) {
      _controlReadCounts[i] = 0;
    }
    for (size_t i = 0; i < kControlReadTraceCapacity; ++i) {
      _controlReadTrace[i] = 0;
    }
    _devicePresent = true;
    _groupId = EE871::cmd::SENSOR_GROUP_ID;
    _subgroup = EE871::cmd::SENSOR_SUBGROUP_ID;
    _availableMeasurements = EE871::cmd::AVAILABLE_MEAS_MASK;
    _holdSclLow = false;
    _sdaStuckLow = false;
    _sdaStuckHigh = false;
    _corruptReadPec = false;
    _failNextReadMainEnabled = false;
    _failNextReadMain = 0;
    _corruptNextReadPecMainEnabled = false;
    _corruptNextReadPecMain = 0;
    _failNextCustomReadAddressEnabled = false;
    _failNextCustomReadAddress = 0;
    _failNextWriteEnabled = false;
    _failNextWriteAddress = 0;
    _dropWriteEnabled = false;
    _dropWriteAddress = 0;
    _dropNextWriteEnabled = false;
    _dropNextWriteAddress = 0;
    _statusByte = 0;
    _mv3 = 600;
    _mv4 = 650;

    for (size_t i = 0; i < EE871::cmd::CUSTOM_MEMORY_SIZE; ++i) {
      _memory[i] = 0;
    }
    _memory[EE871::cmd::CUSTOM_OPERATING_FUNCTIONS] =
        EE871::cmd::FEATURE_SERIAL_NUMBER |
        EE871::cmd::FEATURE_PART_NAME |
        EE871::cmd::FEATURE_ADDRESS_CONFIG |
        EE871::cmd::FEATURE_GLOBAL_INTERVAL |
        EE871::cmd::FEATURE_SPECIFIC_INTERVAL |
        EE871::cmd::FEATURE_FILTER_CONFIG |
        EE871::cmd::FEATURE_ERROR_CODE;
    _memory[EE871::cmd::CUSTOM_OPERATING_MODE_SUPPORT] =
        EE871::cmd::MODE_SUPPORT_LOW_POWER |
        EE871::cmd::MODE_SUPPORT_E2_PRIORITY;
    _memory[EE871::cmd::CUSTOM_SPECIAL_FEATURES] =
        EE871::cmd::SPECIAL_FEATURE_AUTO_ADJUST;
    _memory[EE871::cmd::CUSTOM_INTERVAL_L] =
        static_cast<uint8_t>(EE871::cmd::INTERVAL_MIN_DECISEC & 0xFF);
    _memory[EE871::cmd::CUSTOM_INTERVAL_H] =
        static_cast<uint8_t>(EE871::cmd::INTERVAL_MIN_DECISEC >> 8);
  }

  EE871::Config makeConfig(uint8_t offlineThreshold = 5) {
    EE871::Config cfg;
    cfg.setScl = &FakeE2Transport::setSclThunk;
    cfg.setSda = &FakeE2Transport::setSdaThunk;
    cfg.readScl = &FakeE2Transport::readSclThunk;
    cfg.readSda = &FakeE2Transport::readSdaThunk;
    cfg.delayUs = &FakeE2Transport::delayUsThunk;
    cfg.busUser = this;
    cfg.clockLowUs = 100;
    cfg.clockHighUs = 100;
    cfg.startHoldUs = 4;
    cfg.stopHoldUs = 4;
    cfg.bitTimeoutUs = 25;
    cfg.byteTimeoutUs = 25;
    cfg.writeDelayMs = 0;
    cfg.intervalWriteDelayMs = 0;
    cfg.offlineThreshold = offlineThreshold;
    return cfg;
  }

  void resetElapsed() {
    _elapsedUs = 0;
    _delayCalls = 0;
  }

  void resetBusStats() {
    _delayMsCalls = 0;
    _delayMsTotal = 0;
    _maxDelayMsSlice = 0;
    _yieldCalls = 0;
    _totalBusTransactions = 0;
    _controlReadTraceLen = 0;
    for (size_t i = 0; i < 16; ++i) {
      _controlReadCounts[i] = 0;
    }
    for (size_t i = 0; i < kControlReadTraceCapacity; ++i) {
      _controlReadTrace[i] = 0;
    }
  }

  uint32_t elapsedUs() const { return _elapsedUs; }
  uint32_t delayCalls() const { return _delayCalls; }
  uint32_t delayMsCalls() const { return _delayMsCalls; }
  uint32_t delayMsTotal() const { return _delayMsTotal; }
  uint32_t maxDelayMsSlice() const { return _maxDelayMsSlice; }
  uint32_t yieldCalls() const { return _yieldCalls; }
  uint32_t totalBusTransactions() const { return _totalBusTransactions; }
  uint32_t controlReadCount(uint8_t mainCommandNibble) const {
    return (mainCommandNibble < 16) ? _controlReadCounts[mainCommandNibble] : 0;
  }
  uint32_t controlReadTraceLength() const { return _controlReadTraceLen; }
  uint8_t controlReadAt(uint32_t index) const {
    return (index < _controlReadTraceLen) ? _controlReadTrace[index] : 0xFF;
  }

  void setPresent(bool present) { setDevicePresent(present); }
  void setDevicePresent(bool present) { _devicePresent = present; }
  void setGroupId(uint16_t groupId) { _groupId = groupId; }
  void setSubgroup(uint8_t subgroup) { _subgroup = subgroup; }
  void setAvailableMeasurements(uint8_t bits) { _availableMeasurements = bits; }
  void setHoldSclLow(bool hold) { _holdSclLow = hold; }
  void setSdaStuckLow(bool stuck) { _sdaStuckLow = stuck; }
  void setSdaStuckHigh(bool stuck) { _sdaStuckHigh = stuck; }
  void setCorruptReadPec(bool corrupt) { _corruptReadPec = corrupt; }
  void setStatusByte(uint8_t status) { _statusByte = status; }
  void setCo2FastPpm(uint16_t ppm) { _mv3 = ppm; }
  void setCo2AveragePpm(uint16_t ppm) { _mv4 = ppm; }
  void setErrorCode(uint8_t code) { _memory[EE871::cmd::CUSTOM_ERROR_CODE] = code; }
  void setFeatureFlags(uint8_t operatingFunctions,
                       uint8_t operatingModeSupport,
                       uint8_t specialFeatures) {
    _memory[EE871::cmd::CUSTOM_OPERATING_FUNCTIONS] = operatingFunctions;
    _memory[EE871::cmd::CUSTOM_OPERATING_MODE_SUPPORT] = operatingModeSupport;
    _memory[EE871::cmd::CUSTOM_SPECIAL_FEATURES] = specialFeatures;
  }

  void failNextReadMain(uint8_t mainCommandNibble) {
    _failNextReadMain = mainCommandNibble;
    _failNextReadMainEnabled = true;
  }

  void corruptNextReadPecMain(uint8_t mainCommandNibble) {
    _corruptNextReadPecMain = mainCommandNibble;
    _corruptNextReadPecMainEnabled = true;
  }

  void failNextCustomReadAddress(uint8_t address) {
    _failNextCustomReadAddress = address;
    _failNextCustomReadAddressEnabled = true;
  }

  void attachLongDelayCallbacks(EE871::Config& cfg) {
    cfg.delayMs = &FakeE2Transport::delayMsThunk;
    cfg.yield = &FakeE2Transport::yieldThunk;
  }

  void setMemory(uint8_t address, uint8_t value) { _memory[address] = value; }
  uint8_t memory(uint8_t address) const { return _memory[address]; }

  void failNextWriteToAddress(uint8_t address) {
    _failNextWriteAddress = address;
    _failNextWriteEnabled = true;
  }

  void dropWritesToAddress(uint8_t address, bool enabled) {
    _dropWriteAddress = address;
    _dropWriteEnabled = enabled;
  }

  void dropNextWriteCommitToAddress(uint8_t address) {
    _dropNextWriteAddress = address;
    _dropNextWriteEnabled = true;
  }

private:
  enum class Phase : uint8_t {
    IDLE,
    WRITE_CONTROL,
    ACK_CONTROL,
    WRITE_ADDRESS,
    ACK_ADDRESS,
    WRITE_DATA,
    ACK_DATA,
    WRITE_PEC,
    ACK_PEC,
    READ_DATA,
    MASTER_ACK_DATA,
    READ_PEC,
    MASTER_ACK_PEC
  };

  static void setSclThunk(bool level, void* user) {
    static_cast<FakeE2Transport*>(user)->setScl(level);
  }

  static void setSdaThunk(bool level, void* user) {
    static_cast<FakeE2Transport*>(user)->setSda(level);
  }

  static bool readSclThunk(void* user) {
    return static_cast<FakeE2Transport*>(user)->readScl();
  }

  static bool readSdaThunk(void* user) {
    return static_cast<FakeE2Transport*>(user)->readSda();
  }

  static void delayUsThunk(uint32_t us, void* user) {
    static_cast<FakeE2Transport*>(user)->delayUs(us);
  }

  static void delayMsThunk(uint32_t ms, void* user) {
    static_cast<FakeE2Transport*>(user)->delayMs(ms);
  }

  static void yieldThunk(void* user) {
    static_cast<FakeE2Transport*>(user)->yield();
  }

  void setScl(bool level) {
    const bool wasHigh = readScl();
    _masterSclReleased = level;
    const bool isHigh = readScl();
    if (!wasHigh && isHigh) {
      onSclRising();
    } else if (wasHigh && !isHigh) {
      onSclFalling();
    }
  }

  void setSda(bool level) {
    const bool wasReleased = _masterSdaReleased;
    const bool sclHigh = readScl();
    _masterSdaReleased = level;

    if (sclHigh && wasReleased && !level) {
      beginTransaction();
    } else if (sclHigh && !wasReleased && level) {
      _phase = Phase::IDLE;
    }
  }

  bool readScl() const {
    return _masterSclReleased && !_holdSclLow;
  }

  bool readSda() const {
    if (_sdaStuckLow) {
      return false;
    }
    if (_sdaStuckHigh) {
      return true;
    }
    if (slaveDrivingPhase() && _masterSdaReleased) {
      return _slaveSda;
    }
    return _masterSdaReleased;
  }

  void delayUs(uint32_t us) {
    _elapsedUs += us;
    ++_delayCalls;
  }

  void delayMs(uint32_t ms) {
    _delayMsTotal += ms;
    if (ms > _maxDelayMsSlice) {
      _maxDelayMsSlice = ms;
    }
    ++_delayMsCalls;
  }

  void yield() {
    ++_yieldCalls;
  }

  void beginTransaction() {
    ++_totalBusTransactions;
    _phase = Phase::WRITE_CONTROL;
    _bitCount = 0;
    _byte = 0;
    _control = 0;
    _address = 0;
    _data = 0;
    _pec = 0;
    _responseData = 0;
    _responsePec = 0;
    _slaveSda = true;
  }

  void onSclRising() {
    switch (_phase) {
      case Phase::WRITE_CONTROL:
      case Phase::WRITE_ADDRESS:
      case Phase::WRITE_DATA:
      case Phase::WRITE_PEC:
        captureMasterBit();
        break;
      case Phase::ACK_CONTROL:
      case Phase::ACK_ADDRESS:
      case Phase::ACK_DATA:
      case Phase::ACK_PEC:
        _slaveSda = !ackForCurrentPhase();
        break;
      case Phase::READ_DATA:
        _slaveSda = readBitFromByte(_responseData);
        break;
      case Phase::READ_PEC:
        _slaveSda = readBitFromByte(_responsePec);
        break;
      case Phase::IDLE:
      case Phase::MASTER_ACK_DATA:
      case Phase::MASTER_ACK_PEC:
        break;
    }
  }

  void onSclFalling() {
    if (_skipNextFalling) {
      _skipNextFalling = false;
      return;
    }

    switch (_phase) {
      case Phase::ACK_CONTROL:
        if (!_devicePresent) {
          _phase = Phase::IDLE;
        } else if (controlIsRead()) {
          prepareReadResponse();
          _phase = Phase::READ_DATA;
          _bitCount = 0;
        } else {
          _phase = Phase::WRITE_ADDRESS;
          _bitCount = 0;
          _byte = 0;
        }
        break;
      case Phase::ACK_ADDRESS:
        _phase = Phase::WRITE_DATA;
        _bitCount = 0;
        _byte = 0;
        break;
      case Phase::ACK_DATA:
        _phase = Phase::WRITE_PEC;
        _bitCount = 0;
        _byte = 0;
        break;
      case Phase::ACK_PEC:
        applyWriteIfValid();
        _phase = Phase::IDLE;
        break;
      case Phase::READ_DATA:
        advanceReadBit(Phase::MASTER_ACK_DATA);
        break;
      case Phase::MASTER_ACK_DATA:
        _phase = Phase::READ_PEC;
        _bitCount = 0;
        break;
      case Phase::READ_PEC:
        advanceReadBit(Phase::MASTER_ACK_PEC);
        break;
      case Phase::MASTER_ACK_PEC:
        _phase = Phase::IDLE;
        break;
      case Phase::IDLE:
      case Phase::WRITE_CONTROL:
      case Phase::WRITE_ADDRESS:
      case Phase::WRITE_DATA:
      case Phase::WRITE_PEC:
        break;
    }
  }

  void captureMasterBit() {
    _byte = static_cast<uint8_t>((_byte << 1) | (_masterSdaReleased ? 1U : 0U));
    ++_bitCount;
    if (_bitCount < 8) {
      return;
    }

    switch (_phase) {
      case Phase::WRITE_CONTROL:
        _control = _byte;
        _phase = Phase::ACK_CONTROL;
        _skipNextFalling = true;
        break;
      case Phase::WRITE_ADDRESS:
        _address = _byte;
        _phase = Phase::ACK_ADDRESS;
        _skipNextFalling = true;
        break;
      case Phase::WRITE_DATA:
        _data = _byte;
        _phase = Phase::ACK_DATA;
        _skipNextFalling = true;
        break;
      case Phase::WRITE_PEC:
        _pec = _byte;
        _phase = Phase::ACK_PEC;
        _skipNextFalling = true;
        break;
      default:
        break;
    }
    _bitCount = 0;
    _byte = 0;
  }

  bool ackForCurrentPhase() {
    if (!_devicePresent) {
      return false;
    }
    if (_phase == Phase::ACK_CONTROL && controlIsRead() &&
        _failNextReadMainEnabled && mainCommand() == _failNextReadMain) {
      _failNextReadMainEnabled = false;
      return false;
    }
    if (_phase == Phase::ACK_CONTROL && controlIsRead() &&
        mainCommand() == EE871::cmd::MAIN_CUSTOM_PTR &&
        _failNextCustomReadAddressEnabled &&
        _customPointer == _failNextCustomReadAddress) {
      _failNextCustomReadAddressEnabled = false;
      return false;
    }
    if (_phase == Phase::ACK_ADDRESS &&
        mainCommand() == EE871::cmd::MAIN_CUSTOM_WRITE &&
        _failNextWriteEnabled &&
        _address == _failNextWriteAddress) {
      _failNextWriteEnabled = false;
      return false;
    }
    return _phase == Phase::ACK_CONTROL ||
           _phase == Phase::ACK_ADDRESS ||
           _phase == Phase::ACK_DATA ||
           _phase == Phase::ACK_PEC;
  }

  bool controlIsRead() const {
    return (_control & EE871::cmd::RW_READ) != 0;
  }

  uint8_t mainCommand() const {
    return static_cast<uint8_t>(_control >> EE871::cmd::MAIN_SHIFT);
  }

  void prepareReadResponse() {
    noteControlRead(mainCommand());
    _responseData = readValueForControl();
    _responsePec = static_cast<uint8_t>((_control + _responseData) & 0xFF);
    if (_corruptReadPec) {
      _responsePec = static_cast<uint8_t>(_responsePec ^ 0x01);
    }
    if (_corruptNextReadPecMainEnabled && mainCommand() == _corruptNextReadPecMain) {
      _corruptNextReadPecMainEnabled = false;
      _responsePec = static_cast<uint8_t>(_responsePec ^ 0x01);
    }
  }

  void noteControlRead(uint8_t mainCommandNibble) {
    if (mainCommandNibble < 16) {
      ++_controlReadCounts[mainCommandNibble];
    }
    if (_controlReadTraceLen < kControlReadTraceCapacity) {
      _controlReadTrace[_controlReadTraceLen] = mainCommandNibble;
      ++_controlReadTraceLen;
    }
  }

  uint8_t readValueForControl() {
    const uint8_t main = mainCommand();
    switch (main) {
      case EE871::cmd::MAIN_TYPE_LO:
        return static_cast<uint8_t>(_groupId & 0xFF);
      case EE871::cmd::MAIN_TYPE_HI:
        return static_cast<uint8_t>(_groupId >> 8);
      case EE871::cmd::MAIN_TYPE_SUB:
        return _subgroup;
      case EE871::cmd::MAIN_AVAIL_MEAS:
        return _availableMeasurements;
      case EE871::cmd::MAIN_STATUS:
        return _statusByte;
      case EE871::cmd::MAIN_MV3_LO:
        return static_cast<uint8_t>(_mv3 & 0xFF);
      case EE871::cmd::MAIN_MV3_HI:
        return static_cast<uint8_t>(_mv3 >> 8);
      case EE871::cmd::MAIN_MV4_LO:
        return static_cast<uint8_t>(_mv4 & 0xFF);
      case EE871::cmd::MAIN_MV4_HI:
        return static_cast<uint8_t>(_mv4 >> 8);
      case EE871::cmd::MAIN_CUSTOM_PTR: {
        const uint8_t value = _memory[_customPointer];
        ++_customPointer;
        return value;
      }
      default:
        return 0xFF;
    }
  }

  bool readBitFromByte(uint8_t value) const {
    return (value & static_cast<uint8_t>(0x80U >> _bitCount)) != 0;
  }

  void advanceReadBit(Phase nextPhase) {
    ++_bitCount;
    if (_bitCount >= 8) {
      _phase = nextPhase;
      _bitCount = 0;
    }
  }

  void applyWriteIfValid() {
    if (!_devicePresent) {
      return;
    }
    const uint8_t expected =
        static_cast<uint8_t>((_control + _address + _data) & 0xFF);
    if (_pec != expected) {
      return;
    }

    const uint8_t main = mainCommand();
    if (main == EE871::cmd::MAIN_CUSTOM_PTR) {
      _customPointer = _data;
    } else if (main == EE871::cmd::MAIN_CUSTOM_WRITE) {
      if (_dropNextWriteEnabled && _address == _dropNextWriteAddress) {
        _dropNextWriteEnabled = false;
        return;
      }
      if (!(_dropWriteEnabled && _address == _dropWriteAddress)) {
        _memory[_address] = _data;
      }
    }
  }

  bool slaveDrivingPhase() const {
    return _phase == Phase::ACK_CONTROL ||
           _phase == Phase::ACK_ADDRESS ||
           _phase == Phase::ACK_DATA ||
           _phase == Phase::ACK_PEC ||
           _phase == Phase::READ_DATA ||
           _phase == Phase::READ_PEC;
  }

  bool _masterSclReleased = true;
  bool _masterSdaReleased = true;
  static constexpr uint32_t kControlReadTraceCapacity = 128;
  Phase _phase = Phase::IDLE;
  uint8_t _bitCount = 0;
  uint8_t _byte = 0;
  uint8_t _control = 0;
  uint8_t _address = 0;
  uint8_t _data = 0;
  uint8_t _pec = 0;
  uint8_t _responseData = 0;
  uint8_t _responsePec = 0;
  bool _slaveSda = true;
  bool _skipNextFalling = false;
  uint8_t _customPointer = 0;
  uint32_t _elapsedUs = 0;
  uint32_t _delayCalls = 0;
  uint32_t _delayMsCalls = 0;
  uint32_t _delayMsTotal = 0;
  uint32_t _maxDelayMsSlice = 0;
  uint32_t _yieldCalls = 0;
  uint32_t _totalBusTransactions = 0;
  uint32_t _controlReadCounts[16] = {};
  uint8_t _controlReadTrace[kControlReadTraceCapacity] = {};
  uint32_t _controlReadTraceLen = 0;
  bool _devicePresent = true;
  uint16_t _groupId = EE871::cmd::SENSOR_GROUP_ID;
  uint8_t _subgroup = EE871::cmd::SENSOR_SUBGROUP_ID;
  uint8_t _availableMeasurements = EE871::cmd::AVAILABLE_MEAS_MASK;
  bool _holdSclLow = false;
  bool _sdaStuckLow = false;
  bool _sdaStuckHigh = false;
  bool _corruptReadPec = false;
  bool _failNextReadMainEnabled = false;
  uint8_t _failNextReadMain = 0;
  bool _corruptNextReadPecMainEnabled = false;
  uint8_t _corruptNextReadPecMain = 0;
  bool _failNextCustomReadAddressEnabled = false;
  uint8_t _failNextCustomReadAddress = 0;
  bool _failNextWriteEnabled = false;
  uint8_t _failNextWriteAddress = 0;
  bool _dropWriteEnabled = false;
  uint8_t _dropWriteAddress = 0;
  bool _dropNextWriteEnabled = false;
  uint8_t _dropNextWriteAddress = 0;
  uint8_t _statusByte = 0;
  uint16_t _mv3 = 0;
  uint16_t _mv4 = 0;
  uint8_t _memory[EE871::cmd::CUSTOM_MEMORY_SIZE] = {};
};

} // namespace EE871Test
