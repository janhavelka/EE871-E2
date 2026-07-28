/// @file FakeE2Transport.h
/// @brief Deterministic native E2 callback-boundary fake for runtime fault tests.
#pragma once

#include <cstddef>
#include <cstdint>

#include "EE871/CommandTable.h"
#include "EE871/Config.h"

namespace EE871Test {

enum class StretchPhase : uint8_t {
  NONE = 0,
  DATA_BIT,
  ACK_BIT,
  FINAL_ACK,
  STOP,
  NEXT_START,
};

class FakeE2Transport {
public:
  static constexpr size_t MAX_RECORDED_TRANSACTIONS = 128;

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
    _delayUsCalls = 0;
    _lastDelayUs = 0;
    _longDelayUsCalls = 0;
    _longDelayUsTotalUs = 0;
    _longDelaySlices = 0;
    _delayMsTotalMs = 0;
    _maxDelayMsSliceMs = 0;
    _yieldCount = 0;
    _lineWrites = 0;
    _lineReads = 0;
    _transactionCount = 0;
    _currentTransferIndex = 0;
    _devicePresent = true;
    _group = EE871::cmd::SENSOR_GROUP_ID;
    _subgroup = EE871::cmd::SENSOR_SUBGROUP_ID;
    _availableMeasurements = EE871::cmd::AVAILABLE_MEAS_MASK;
    _failAtTransferEnabled = false;
    _failAtTransferIndex = 0;
    _holdSclLow = false;
    _sdaStuckLow = false;
    _sdaStuckHigh = false;
    _corruptReadPec = false;
    _nackNextFinalAck = false;
    _finalAckAccepted = true;
    _failNextWriteEnabled = false;
    _failNextWriteAddress = 0;
    _dropWriteEnabled = false;
    _dropWriteAddress = 0;
    _dropNextWriteEnabled = false;
    _dropNextWriteAddress = 0;
    _statusByte = 0;
    _mv3 = 600;
    _mv4 = 650;
    _stretchPhase = StretchPhase::NONE;
    _stretchDurationUs = 0;
    _stretchMatchesToSkip = 0;
    _stretchOccurrences = 0;
    _stretchSequenceCount = 0;
    _stretchSequenceIndex = 0;
    _activeStretchRemainingUs = 0;
    _pointerCompletionDelayUs =
        EE871::cmd::WRITE_DELAY_PROTOCOL_MIN_MS * 1000U;
    _pointerCompletionRemainingUs = 0;
    _pointerCompletionDelaysUntilStart = 0;
    _transactionStartedDuringPointerCompletion = false;
    _pointerReadStartedEarly = false;
    _intervalCompletionDelayUs =
        EE871::cmd::INTERVAL_WRITE_DELAY_PROTOCOL_MIN_MS * 1000U;
    _intervalCompletionRemainingUs = 0;
    _intervalCompletionDelaysUntilStart = 0;
    _intervalTransactionStartedEarly = false;

    for (size_t i = 0; i < MAX_RECORDED_TRANSACTIONS; ++i) {
      _transactionMain[i] = 0;
      _transactionAddress[i] = 0;
      _transactionIsRead[i] = false;
      _transactionHasAddress[i] = false;
    }
    for (size_t i = 0; i < MAX_RECORDED_LONG_DELAYS; ++i) {
      _longDelayUsDurations[i] = 0;
    }
    for (size_t i = 0; i < EE871::cmd::CUSTOM_MEMORY_SIZE; ++i) {
      _memory[i] = 0;
    }
    _memory[EE871::cmd::CUSTOM_ADJUSTMENT_SUPPORT] =
        EE871::cmd::FEATURE_CO2_CUSTOM_ADJUSTMENT;
    _memory[EE871::cmd::CUSTOM_ADJUSTMENT_POINT_SUPPORT] =
        EE871::cmd::FEATURE_CO2_ADJUSTMENT_POINT;
    _memory[EE871::cmd::CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT] =
        EE871::cmd::FEATURE_CUSTOM_ADJUSTMENT_TIME_GENERAL;
    _memory[EE871::cmd::CUSTOM_ADJUSTMENT_TIME_SUPPORT] =
        EE871::cmd::FEATURE_CO2_ADJUSTMENT_TIME;
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
    EE871::Config cfg = makeDefaultTimingConfig();
    cfg.startHoldUs = 4;
    cfg.stopHoldUs = 4;
    cfg.offlineThreshold = offlineThreshold;
    cfg.longDelaySliceMs = 1;
    return cfg;
  }

  EE871::Config makeDefaultTimingConfig() {
    EE871::Config cfg;
    cfg.setScl = &FakeE2Transport::setSclThunk;
    cfg.setSda = &FakeE2Transport::setSdaThunk;
    cfg.readScl = &FakeE2Transport::readSclThunk;
    cfg.readSda = &FakeE2Transport::readSdaThunk;
    cfg.delayUs = &FakeE2Transport::delayUsThunk;
    cfg.busUser = this;
    cfg.delayMs = &FakeE2Transport::delayMsThunk;
    cfg.yield = &FakeE2Transport::yieldThunk;
    return cfg;
  }

  void resetElapsed() {
    _elapsedUs = 0;
    _delayUsCalls = 0;
    _lastDelayUs = 0;
    _longDelayUsCalls = 0;
    _longDelayUsTotalUs = 0;
    _longDelaySlices = 0;
    _delayMsTotalMs = 0;
    _maxDelayMsSliceMs = 0;
    _yieldCount = 0;
    for (size_t i = 0; i < MAX_RECORDED_LONG_DELAYS; ++i) {
      _longDelayUsDurations[i] = 0;
    }
  }

  void resetActivityCounters() {
    _lineWrites = 0;
    _lineReads = 0;
    _transactionCount = 0;
    for (size_t i = 0; i < MAX_RECORDED_TRANSACTIONS; ++i) {
      _transactionMain[i] = 0;
      _transactionAddress[i] = 0;
      _transactionIsRead[i] = false;
      _transactionHasAddress[i] = false;
    }
  }

  uint64_t elapsedUs() const { return _elapsedUs; }
  uint32_t delayCalls() const { return _delayUsCalls; }
  uint32_t lastDelayUs() const { return _lastDelayUs; }
  uint32_t longDelayUsCalls() const { return _longDelayUsCalls; }
  uint64_t longDelayUsTotalUs() const { return _longDelayUsTotalUs; }
  uint32_t longDelayUsDuration(size_t index) const {
    return index < MAX_RECORDED_LONG_DELAYS
               ? _longDelayUsDurations[index]
               : 0U;
  }
  uint32_t longDelaySlices() const { return _longDelaySlices; }
  uint32_t delayMsTotalMs() const { return _delayMsTotalMs; }
  uint32_t maxDelayMsSliceMs() const { return _maxDelayMsSliceMs; }
  uint32_t yieldCount() const { return _yieldCount; }
  uint32_t lineWrites() const { return _lineWrites; }
  uint32_t lineReads() const { return _lineReads; }
  uint32_t transactionCount() const { return _transactionCount; }
  bool busLinesIdle() const {
    return physicalSclHigh() && _masterSdaReleased && !_sdaStuckLow;
  }

  uint8_t transactionMain(size_t index) const {
    return index < MAX_RECORDED_TRANSACTIONS ? _transactionMain[index] : 0;
  }
  uint8_t transactionAddress(size_t index) const {
    return index < MAX_RECORDED_TRANSACTIONS ? _transactionAddress[index] : 0;
  }
  bool transactionIsRead(size_t index) const {
    return index < MAX_RECORDED_TRANSACTIONS && _transactionIsRead[index];
  }
  bool transactionHasAddress(size_t index) const {
    return index < MAX_RECORDED_TRANSACTIONS && _transactionHasAddress[index];
  }

  uint32_t countTransactions(uint8_t mainCommand, bool read) const {
    const size_t count =
        _transactionCount < MAX_RECORDED_TRANSACTIONS
            ? _transactionCount
            : MAX_RECORDED_TRANSACTIONS;
    uint32_t matches = 0;
    for (size_t i = 0; i < count; ++i) {
      if (_transactionMain[i] == mainCommand &&
          _transactionIsRead[i] == read) {
        ++matches;
      }
    }
    return matches;
  }

  void setDevicePresent(bool present) { _devicePresent = present; }
  void setIdentity(
      uint16_t group,
      uint8_t subgroup,
      uint8_t availableMeasurements) {
    _group = group;
    _subgroup = subgroup;
    _availableMeasurements = availableMeasurements;
  }
  void setGroup(uint16_t group) { _group = group; }
  void setSubgroup(uint8_t subgroup) { _subgroup = subgroup; }
  void setAvailableMeasurements(uint8_t bits) {
    _availableMeasurements = bits;
  }
  void setCapabilities(
      uint8_t customAdjustmentSupport,
      uint8_t adjustmentPointSupport,
      uint8_t adjustmentTimeGeneralSupport,
      uint8_t adjustmentTimeSupport,
      uint8_t operatingFunctions,
      uint8_t operatingModeSupport,
      uint8_t specialFeatures) {
    _memory[EE871::cmd::CUSTOM_ADJUSTMENT_SUPPORT] =
        customAdjustmentSupport;
    _memory[EE871::cmd::CUSTOM_ADJUSTMENT_POINT_SUPPORT] =
        adjustmentPointSupport;
    _memory[EE871::cmd::CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT] =
        adjustmentTimeGeneralSupport;
    _memory[EE871::cmd::CUSTOM_ADJUSTMENT_TIME_SUPPORT] =
        adjustmentTimeSupport;
    _memory[EE871::cmd::CUSTOM_OPERATING_FUNCTIONS] =
        operatingFunctions;
    _memory[EE871::cmd::CUSTOM_OPERATING_MODE_SUPPORT] =
        operatingModeSupport;
    _memory[EE871::cmd::CUSTOM_SPECIAL_FEATURES] =
        specialFeatures;
  }
  void failAtTransferIndex(uint32_t index) {
    _failAtTransferIndex = index;
    _failAtTransferEnabled = true;
  }

  void setHoldSclLow(bool hold) {
    const bool wasHigh = physicalSclHigh();
    _holdSclLow = hold;
    const bool isHigh = physicalSclHigh();
    if (!wasHigh && isHigh) {
      onSclRising();
    } else if (wasHigh && !isHigh) {
      onSclFalling();
    }
  }

  void setSdaStuckLow(bool stuck) { _sdaStuckLow = stuck; }
  void setSdaStuckHigh(bool stuck) { _sdaStuckHigh = stuck; }
  void setCorruptReadPec(bool corrupt) { _corruptReadPec = corrupt; }
  void nackNextFinalAck() { _nackNextFinalAck = true; }

  void setStretch(
      StretchPhase phase,
      uint32_t durationUs,
      uint16_t occurrences = 1,
      uint16_t matchesToSkip = 0) {
    _stretchPhase = phase;
    _stretchDurationUs = durationUs;
    _stretchMatchesToSkip = matchesToSkip;
    _stretchOccurrences = occurrences;
    _stretchSequenceCount = 0;
    _stretchSequenceIndex = 0;
    _activeStretchRemainingUs = 0;
    if (phase == StretchPhase::NEXT_START &&
        _phase == Phase::IDLE &&
        _masterSclReleased &&
        _masterSdaReleased &&
        occurrences != 0U &&
        matchesToSkip == 0U) {
      _activeStretchRemainingUs = durationUs;
      --_stretchOccurrences;
    }
  }

  void setStretchSequence(
      StretchPhase phase,
      const uint32_t* durationsUs,
      uint8_t count) {
    _stretchPhase = phase;
    _stretchDurationUs = 0;
    _stretchMatchesToSkip = 0;
    _stretchOccurrences = 0;
    _stretchSequenceCount = count > MAX_STRETCH_SEQUENCE
                                ? MAX_STRETCH_SEQUENCE
                                : count;
    _stretchSequenceIndex = 0;
    _activeStretchRemainingUs = 0;
    for (uint8_t i = 0; i < _stretchSequenceCount; ++i) {
      _stretchSequenceUs[i] = durationsUs[i];
    }
  }

  void clearStretch() {
    _stretchPhase = StretchPhase::NONE;
    _stretchDurationUs = 0;
    _stretchMatchesToSkip = 0;
    _stretchOccurrences = 0;
    _stretchSequenceCount = 0;
    _stretchSequenceIndex = 0;
    _activeStretchRemainingUs = 0;
  }

  void setPointerCompletionDelayUs(uint32_t durationUs) {
    _pointerCompletionDelayUs = durationUs;
  }
  bool pointerReadStartedEarly() const { return _pointerReadStartedEarly; }
  bool intervalTransactionStartedEarly() const {
    return _intervalTransactionStartedEarly;
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
    static_cast<FakeE2Transport*>(user)->onYield();
  }

  bool physicalSclHigh() const {
    return _masterSclReleased &&
           !_holdSclLow &&
           _activeStretchRemainingUs == 0U;
  }

  StretchPhase currentStretchPhase() const {
    switch (_phase) {
      case Phase::ACK_PEC:
        return StretchPhase::FINAL_ACK;
      case Phase::ACK_CONTROL:
      case Phase::ACK_ADDRESS:
      case Phase::ACK_DATA:
      case Phase::MASTER_ACK_DATA:
      case Phase::MASTER_ACK_PEC:
        return StretchPhase::ACK_BIT;
      case Phase::WRITE_CONTROL:
      case Phase::WRITE_ADDRESS:
      case Phase::WRITE_DATA:
      case Phase::WRITE_PEC:
      case Phase::READ_DATA:
      case Phase::READ_PEC:
        return StretchPhase::DATA_BIT;
      case Phase::IDLE:
        return _masterSdaReleased
                   ? StretchPhase::NEXT_START
                   : StretchPhase::STOP;
    }
    return StretchPhase::NONE;
  }

  bool activateConfiguredStretch() {
    if (_stretchPhase == StretchPhase::NONE ||
        currentStretchPhase() != _stretchPhase) {
      return false;
    }
    if (_stretchMatchesToSkip != 0U) {
      --_stretchMatchesToSkip;
      return false;
    }
    if (_stretchSequenceIndex < _stretchSequenceCount) {
      _activeStretchRemainingUs =
          _stretchSequenceUs[_stretchSequenceIndex++];
      return _activeStretchRemainingUs != 0U;
    }
    if (_stretchOccurrences == 0U) {
      return false;
    }
    --_stretchOccurrences;
    _activeStretchRemainingUs = _stretchDurationUs;
    return _activeStretchRemainingUs != 0U;
  }

  void setScl(bool level) {
    ++_lineWrites;
    const bool wasHigh = physicalSclHigh();
    _masterSclReleased = level;
    if (level && !wasHigh && _activeStretchRemainingUs == 0U) {
      (void)activateConfiguredStretch();
    }
    const bool isHigh = physicalSclHigh();
    if (!wasHigh && isHigh) {
      onSclRising();
    } else if (wasHigh && !isHigh) {
      onSclFalling();
    }
  }

  void setSda(bool level) {
    ++_lineWrites;
    const bool wasReleased = _masterSdaReleased;
    const bool sclHigh = physicalSclHigh();
    _masterSdaReleased = level;

    if (sclHigh && wasReleased && !level) {
      beginTransaction();
    } else if (sclHigh && !wasReleased && level) {
      _phase = Phase::IDLE;
    }
  }

  bool readScl() {
    ++_lineReads;
    return physicalSclHigh();
  }

  bool readSda() {
    ++_lineReads;
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

  void consumeSimulatedTime(uint32_t us) {
    _elapsedUs += us;
    if (_pointerCompletionRemainingUs != 0U) {
      _pointerCompletionRemainingUs =
          us >= _pointerCompletionRemainingUs
              ? 0U
              : _pointerCompletionRemainingUs - us;
    }
    if (_intervalCompletionRemainingUs != 0U) {
      _intervalCompletionRemainingUs =
          us >= _intervalCompletionRemainingUs
              ? 0U
              : _intervalCompletionRemainingUs - us;
    }

    if (_activeStretchRemainingUs == 0U) {
      return;
    }
    const bool wasHigh = physicalSclHigh();
    _activeStretchRemainingUs =
        us >= _activeStretchRemainingUs
            ? 0U
            : _activeStretchRemainingUs - us;
    const bool isHigh = physicalSclHigh();
    if (!wasHigh && isHigh) {
      onSclRising();
    }
  }

  void delayUs(uint32_t us) {
    ++_delayUsCalls;
    _lastDelayUs = us;
    const bool startPointerCompletion =
        _pointerCompletionDelaysUntilStart != 0U &&
        --_pointerCompletionDelaysUntilStart == 0U;
    const bool startIntervalCompletion =
        _intervalCompletionDelaysUntilStart != 0U &&
        --_intervalCompletionDelaysUntilStart == 0U;
    consumeSimulatedTime(us);
    if (startPointerCompletion) {
      _pointerCompletionRemainingUs = _pointerCompletionDelayUs;
    }
    if (startIntervalCompletion) {
      _intervalCompletionRemainingUs = _intervalCompletionDelayUs;
    }
    if (us >= 1000U) {
      const size_t index = _longDelayUsCalls;
      if (index < MAX_RECORDED_LONG_DELAYS) {
        _longDelayUsDurations[index] = us;
      }
      ++_longDelayUsCalls;
      _longDelayUsTotalUs += us;
    }
  }

  void delayMs(uint32_t ms) {
    ++_longDelaySlices;
    _delayMsTotalMs += ms;
    if (ms > _maxDelayMsSliceMs) {
      _maxDelayMsSliceMs = ms;
    }
    consumeSimulatedTime(ms * 1000U);
  }

  void onYield() { ++_yieldCount; }

  void beginTransaction() {
    _transactionStartedDuringPointerCompletion =
        _pointerCompletionRemainingUs != 0U;
    if (_intervalCompletionRemainingUs != 0U) {
      _intervalTransactionStartedEarly = true;
    }
    const size_t index = _transactionCount;
    _currentTransferIndex = static_cast<uint32_t>(index);
    ++_transactionCount;
    if (index < MAX_RECORDED_TRANSACTIONS) {
      _transactionMain[index] = 0;
      _transactionAddress[index] = 0;
      _transactionIsRead[index] = false;
      _transactionHasAddress[index] = false;
    }

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
    _finalAckAccepted = true;
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
    _byte = static_cast<uint8_t>((_byte << 1) |
                                 (_masterSdaReleased ? 1U : 0U));
    ++_bitCount;
    if (_bitCount < 8) {
      return;
    }

    switch (_phase) {
      case Phase::WRITE_CONTROL: {
        _control = _byte;
        const size_t index =
            _transactionCount == 0U ? 0U : _transactionCount - 1U;
        if (index < MAX_RECORDED_TRANSACTIONS) {
          _transactionMain[index] = mainCommand();
          _transactionIsRead[index] = controlIsRead();
        }
        if (_transactionStartedDuringPointerCompletion &&
            controlIsRead() &&
            mainCommand() == EE871::cmd::MAIN_CUSTOM_PTR) {
          _pointerReadStartedEarly = true;
        }
        if (controlIsRead() &&
            mainCommand() == EE871::cmd::MAIN_CUSTOM_PTR) {
          const size_t index =
              _transactionCount == 0U ? 0U : _transactionCount - 1U;
          if (index < MAX_RECORDED_TRANSACTIONS) {
            _transactionAddress[index] = _customPointer;
            _transactionHasAddress[index] = true;
          }
        }
        _phase = Phase::ACK_CONTROL;
        _skipNextFalling = true;
        break;
      }
      case Phase::WRITE_ADDRESS: {
        _address = _byte;
        const size_t index =
            _transactionCount == 0U ? 0U : _transactionCount - 1U;
        if (index < MAX_RECORDED_TRANSACTIONS) {
          _transactionAddress[index] = _address;
          _transactionHasAddress[index] = true;
        }
        _phase = Phase::ACK_ADDRESS;
        _skipNextFalling = true;
        break;
      }
      case Phase::WRITE_DATA: {
        _data = _byte;
        if (mainCommand() == EE871::cmd::MAIN_CUSTOM_PTR) {
          const size_t index =
              _transactionCount == 0U ? 0U : _transactionCount - 1U;
          if (index < MAX_RECORDED_TRANSACTIONS) {
            _transactionAddress[index] = _data;
            _transactionHasAddress[index] = true;
          }
        }
        _phase = Phase::ACK_DATA;
        _skipNextFalling = true;
        break;
      }
      case Phase::WRITE_PEC:
        _pec = _byte;
        if (mainCommand() == EE871::cmd::MAIN_CUSTOM_PTR) {
          _pointerCompletionDelaysUntilStart = 2;
        } else if (
            mainCommand() == EE871::cmd::MAIN_CUSTOM_WRITE &&
            _address == EE871::cmd::CUSTOM_INTERVAL_H) {
          _intervalCompletionDelaysUntilStart = 2;
        }
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
    if (_phase == Phase::ACK_CONTROL &&
        _failAtTransferEnabled &&
        _currentTransferIndex == _failAtTransferIndex) {
      _failAtTransferEnabled = false;
      return false;
    }
    if (_phase == Phase::ACK_PEC && _nackNextFinalAck) {
      _nackNextFinalAck = false;
      _finalAckAccepted = false;
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
    _responseData = readValueForControl();
    _responsePec =
        static_cast<uint8_t>((_control + _responseData) & 0xFF);
    if (_corruptReadPec) {
      _responsePec = static_cast<uint8_t>(_responsePec ^ 0x01);
    }
  }

  uint8_t readValueForControl() {
    const uint8_t main = mainCommand();
    switch (main) {
      case EE871::cmd::MAIN_TYPE_LO:
        return static_cast<uint8_t>(_group & 0xFF);
      case EE871::cmd::MAIN_TYPE_HI:
        return static_cast<uint8_t>(_group >> 8);
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
    if (!_devicePresent || !_finalAckAccepted) {
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
  uint64_t _elapsedUs = 0;
  uint32_t _delayUsCalls = 0;
  uint32_t _lastDelayUs = 0;
  static constexpr size_t MAX_RECORDED_LONG_DELAYS = 16;
  uint32_t _longDelayUsDurations[MAX_RECORDED_LONG_DELAYS] = {};
  uint32_t _longDelayUsCalls = 0;
  uint64_t _longDelayUsTotalUs = 0;
  uint32_t _longDelaySlices = 0;
  uint32_t _delayMsTotalMs = 0;
  uint32_t _maxDelayMsSliceMs = 0;
  uint32_t _yieldCount = 0;
  uint32_t _lineWrites = 0;
  uint32_t _lineReads = 0;
  uint32_t _transactionCount = 0;
  uint32_t _currentTransferIndex = 0;
  bool _devicePresent = true;
  uint16_t _group = 0;
  uint8_t _subgroup = 0;
  uint8_t _availableMeasurements = 0;
  bool _failAtTransferEnabled = false;
  uint32_t _failAtTransferIndex = 0;
  bool _holdSclLow = false;
  bool _sdaStuckLow = false;
  bool _sdaStuckHigh = false;
  bool _corruptReadPec = false;
  bool _nackNextFinalAck = false;
  bool _finalAckAccepted = true;
  bool _failNextWriteEnabled = false;
  uint8_t _failNextWriteAddress = 0;
  bool _dropWriteEnabled = false;
  uint8_t _dropWriteAddress = 0;
  bool _dropNextWriteEnabled = false;
  uint8_t _dropNextWriteAddress = 0;
  uint8_t _statusByte = 0;
  uint16_t _mv3 = 0;
  uint16_t _mv4 = 0;
  StretchPhase _stretchPhase = StretchPhase::NONE;
  uint32_t _stretchDurationUs = 0;
  uint16_t _stretchMatchesToSkip = 0;
  uint16_t _stretchOccurrences = 0;
  static constexpr uint8_t MAX_STRETCH_SEQUENCE = 16;
  uint32_t _stretchSequenceUs[MAX_STRETCH_SEQUENCE] = {};
  uint8_t _stretchSequenceCount = 0;
  uint8_t _stretchSequenceIndex = 0;
  uint32_t _activeStretchRemainingUs = 0;
  uint32_t _pointerCompletionDelayUs = 0;
  uint32_t _pointerCompletionRemainingUs = 0;
  uint8_t _pointerCompletionDelaysUntilStart = 0;
  bool _transactionStartedDuringPointerCompletion = false;
  bool _pointerReadStartedEarly = false;
  uint32_t _intervalCompletionDelayUs = 0;
  uint32_t _intervalCompletionRemainingUs = 0;
  uint8_t _intervalCompletionDelaysUntilStart = 0;
  bool _intervalTransactionStartedEarly = false;
  uint8_t _transactionMain[MAX_RECORDED_TRANSACTIONS] = {};
  uint8_t _transactionAddress[MAX_RECORDED_TRANSACTIONS] = {};
  bool _transactionIsRead[MAX_RECORDED_TRANSACTIONS] = {};
  bool _transactionHasAddress[MAX_RECORDED_TRANSACTIONS] = {};
  uint8_t _memory[EE871::cmd::CUSTOM_MEMORY_SIZE] = {};
};

} // namespace EE871Test
