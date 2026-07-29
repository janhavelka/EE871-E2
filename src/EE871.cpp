/// @file EE871.cpp
/// @brief Implementation of the EE871 E2 driver

#include "EE871/EE871.h"

#include <limits>

namespace EE871 {
namespace {

static constexpr uint32_t kPollStepUs = 5;
static constexpr uint32_t kDataSetupUs = 10;

constexpr int8_t signedByteFromRaw(uint8_t raw) {
  const int16_t decoded =
      raw <= 0x7FU
          ? static_cast<int16_t>(raw)
          : static_cast<int16_t>(raw) - 0x100;
  return static_cast<int8_t>(decoded);
}

constexpr int16_t signedWordFromRaw(uint16_t raw) {
  const int32_t decoded =
      raw <= 0x7FFFU
          ? static_cast<int32_t>(raw)
          : static_cast<int32_t>(raw) - 0x10000L;
  return static_cast<int16_t>(decoded);
}

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

uint64_t completionAckTailBoundUs(const Config& cfg) {
  return static_cast<uint64_t>(kDataSetupUs) +
         cfg.clockHighUs + cfg.clockLowUs;
}

uint64_t completionStopTailBoundUs(const Config& cfg) {
  return static_cast<uint64_t>(kDataSetupUs) +
         (2ULL * cfg.stopHoldUs);
}

uint64_t completionWriteTransactionBoundUs(
    const Config& cfg, uint32_t completionMs) {
  return startBoundUs(cfg) + (4ULL * cfg.byteTimeoutUs) +
         (static_cast<uint64_t>(completionMs) * 1000ULL) +
         completionAckTailBoundUs(cfg) +
         completionStopTailBoundUs(cfg);
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

enum class CustomWriteRoute : uint8_t {
  RAW,
  READ_ONLY,
  BUS_ADDRESS,
  INTERVAL_PAIR,
  INTERVAL_FACTOR,
  FILTER,
  OPERATING_MODE,
  AUTO_ADJUST,
  CO2_PAIR,
};

constexpr CustomWriteRoute classifyCustomWriteAddress(uint8_t address) {
  if (address <= cmd::CUSTOM_SPECIAL_FEATURES ||
      (address >= cmd::CUSTOM_CO2_POINT_L_L &&
       address <= cmd::CUSTOM_CO2_POINT_U_H) ||
      (address >= cmd::CUSTOM_SERIAL_START &&
       address < cmd::CUSTOM_SERIAL_START + cmd::CUSTOM_SERIAL_LEN) ||
      address == cmd::CUSTOM_ERROR_CODE ||
      address == cmd::CUSTOM_POINTER_LOW ||
      address == cmd::CUSTOM_POINTER_HIGH) {
    return CustomWriteRoute::READ_ONLY;
  }
  if (address >= cmd::CUSTOM_CO2_OFFSET_L &&
      address <= cmd::CUSTOM_CO2_GAIN_H) {
    return CustomWriteRoute::CO2_PAIR;
  }
  switch (address) {
    case cmd::CUSTOM_BUS_ADDRESS:
      return CustomWriteRoute::BUS_ADDRESS;
    case cmd::CUSTOM_INTERVAL_L:
    case cmd::CUSTOM_INTERVAL_H:
      return CustomWriteRoute::INTERVAL_PAIR;
    case cmd::CUSTOM_CO2_INTERVAL_FACTOR:
      return CustomWriteRoute::INTERVAL_FACTOR;
    case cmd::CUSTOM_FILTER_CO2:
      return CustomWriteRoute::FILTER;
    case cmd::CUSTOM_OPERATING_MODE:
      return CustomWriteRoute::OPERATING_MODE;
    case cmd::CUSTOM_AUTO_ADJUST:
      return CustomWriteRoute::AUTO_ADJUST;
    default:
      return CustomWriteRoute::RAW;
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
    ByteDeadline& deadline) {
  if (deadline.elapsedUs > deadline.limitUs ||
      us > deadline.limitUs - deadline.elapsedUs) {
    return Status::Error(
        Err::TIMEOUT,
        "Byte timeout",
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

void EE871::_finishCompletionBudget(
    const Config& config, CompletionBudget& budget) {
  if (budget.consumedUs >= budget.limitUs) {
    return;
  }
  const uint32_t remainingUs =
      budget.limitUs - budget.consumedUs;
  _delayLongMs(config, remainingUs / 1000U);
  const uint32_t remainderUs = remainingUs % 1000U;
  if (remainderUs != 0U) {
    _delayUs(config, remainderUs);
  }
  budget.consumedUs = budget.limitUs;
}

Status EE871::_waitSclHigh(
    const Config& config,
    ByteDeadline& deadline) {
  uint32_t waitedUs = 0;
  const uint32_t bitLimit = config.bitTimeoutUs;

  while (!readScl(config)) {
    if (waitedUs > bitLimit || kPollStepUs > bitLimit - waitedUs) {
      return Status::Error(
          Err::TIMEOUT,
          "Clock stretch timeout",
          static_cast<int32_t>(waitedUs));
    }
    if (deadline.elapsedUs > deadline.limitUs ||
        kPollStepUs > deadline.limitUs - deadline.elapsedUs) {
      return Status::Error(
          Err::TIMEOUT,
          "Byte timeout",
          static_cast<int32_t>(deadline.elapsedUs));
    }
    _delayUs(config, kPollStepUs, &deadline);
    waitedUs += kPollStepUs;
  }
  return Status::Ok();
}

Status EE871::_waitSclHighCompletion(
    const Config& config,
    ClockWaitClass waitClass,
    CompletionBudget& completionBudget) {
  // Only device-held-low polling spends the cumulative completion allowance.
  // The caller accounts for the bounded master waveform separately.
  while (!readScl(config)) {
    if (completionBudget.consumedUs > completionBudget.limitUs ||
        kPollStepUs >
            completionBudget.limitUs - completionBudget.consumedUs) {
      return Status::Error(
          Err::TIMEOUT,
          waitClass == ClockWaitClass::INTERVAL_COMMIT
              ? "Interval commit timeout"
              : "Write completion timeout",
          static_cast<int32_t>(completionBudget.consumedUs));
    }
    _delayUs(config, kPollStepUs);
    completionBudget.consumedUs = saturatingAddU32(
        completionBudget.consumedUs, kPollStepUs);
  }
  return Status::Ok();
}

Status EE871::_e2Start(const Config& config) {
  setSda(config, true);
  setScl(config, true);
  ByteDeadline idleWait{0, config.bitTimeoutUs};
  Status st = _waitSclHigh(config, idleWait);
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
    CompletionBudget* completionBudget) {
  if (waitClass == ClockWaitClass::NORMAL_BIT) {
    setSda(config, false);
    _delayUs(config, kDataSetupUs);
    setScl(config, true);
    ByteDeadline sclDeadline{0, config.bitTimeoutUs};
    Status st = _waitSclHigh(config, sclDeadline);
    if (!st.ok()) {
      return st;
    }
    _delayUs(config, config.stopHoldUs);
    setSda(config, true);
    _delayUs(config, config.stopHoldUs);
    return Status::Ok();
  }

  if (completionBudget == nullptr) {
    return Status::Error(
        Err::INVALID_PARAM, "Missing completion budget");
  }

  setSda(config, false);
  // Deterministic STOP waveform timing is outside the sensor's completion
  // allowance; only the SCL-high wait below consumes that allowance.
  _delayUs(config, kDataSetupUs);
  setScl(config, true);
  Status st = _waitSclHighCompletion(
      config, waitClass, *completionBudget);
  if (!st.ok()) {
    return st;
  }
  _delayUs(config, config.stopHoldUs);
  setSda(config, true);
  _delayUs(config, config.stopHoldUs);
  return Status::Ok();
}

Status EE871::_writeBit(
    const Config& config, bool bit, ByteDeadline& deadline) {
  setSda(config, bit);
  Status st = _delayWithinDeadline(
      config, kDataSetupUs, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = _waitSclHigh(config, deadline);
  if (!st.ok()) {
    return st;
  }
  st = _delayWithinDeadline(
      config, config.clockHighUs, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, false);
  return _delayWithinDeadline(
      config, config.clockLowUs, deadline);
}

Status EE871::_readBit(
    const Config& config, bool& bit, ByteDeadline& deadline) {
  setSda(config, true);
  Status st = _delayWithinDeadline(
      config, kDataSetupUs, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = _waitSclHigh(config, deadline);
  if (!st.ok()) {
    return st;
  }
  const uint32_t sampleDelay = config.clockHighUs / 2U;
  st = _delayWithinDeadline(
      config, sampleDelay, deadline);
  if (!st.ok()) {
    return st;
  }
  bit = readSda(config);
  st = _delayWithinDeadline(
      config,
      config.clockHighUs - sampleDelay,
      deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, false);
  return _delayWithinDeadline(
      config, config.clockLowUs, deadline);
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
    ByteDeadline& deadline,
    bool* observed,
    CompletionBudget* completionBudget) {
  if (observed != nullptr) {
    *observed = false;
  }
  if (waitClass != ClockWaitClass::NORMAL_BIT &&
      completionBudget == nullptr) {
    return Status::Error(
        Err::INVALID_PARAM, "Missing completion budget");
  }
  auto delayAckPhase =
      [&config, waitClass, &deadline](uint32_t us) {
        if (waitClass == ClockWaitClass::NORMAL_BIT) {
          return _delayWithinDeadline(
              config, us, deadline);
        }
        // Long completion mode budgets device-held-low polling only.
        _delayUs(config, us);
        return Status::Ok();
      };

  setSda(config, true);
  Status st = delayAckPhase(kDataSetupUs);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = waitClass == ClockWaitClass::NORMAL_BIT
      ? _waitSclHigh(config, deadline)
      : _waitSclHighCompletion(
            config, waitClass, *completionBudget);
  if (!st.ok()) {
    return st;
  }
  const uint32_t sampleDelay = config.clockHighUs / 2U;
  st = delayAckPhase(sampleDelay);
  if (!st.ok()) {
    return st;
  }
  acked = !readSda(config);
  if (observed != nullptr) {
    *observed = true;
  }
  st = delayAckPhase(config.clockHighUs - sampleDelay);
  if (!st.ok()) {
    return st;
  }
  setScl(config, false);
  return delayAckPhase(config.clockLowUs);
}

Status EE871::_sendAck(
    const Config& config, bool ack, ByteDeadline& deadline) {
  setSda(config, !ack);
  Status st = _delayWithinDeadline(
      config, kDataSetupUs, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, true);
  st = _waitSclHigh(config, deadline);
  if (!st.ok()) {
    return st;
  }
  st = _delayWithinDeadline(
      config, config.clockHighUs, deadline);
  if (!st.ok()) {
    return st;
  }
  setScl(config, false);
  st = _delayWithinDeadline(
      config, config.clockLowUs, deadline);
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
  const bool blockWrite =
      kind == OperationKind::CUSTOM_BLOCK_WRITE_VERIFY;
  if ((!blockRead && !blockWrite && elementCount != 1U) ||
      (blockRead && (elementCount == 0U || elementCount > 256U)) ||
      (blockWrite && (elementCount == 0U || elementCount > 16U))) {
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
    case OperationKind::CUSTOM_BLOCK_WRITE_VERIFY: {
      uint64_t oneElementUs = customWriteUs;
      st = addScaledU64(oneElementUs, pointerUs, 1U);
      if (st.ok()) {
        st = addScaledU64(oneElementUs, readUs, 1U);
      }
      if (st.ok()) {
        st = addScaledU64(totalUs, oneElementUs, elementCount);
      }
      break;
    }
    case OperationKind::RESYNC_PERSISTENT_CONFIG:
      st = addScaledU64(totalUs, pointerUs, 9U);
      if (st.ok()) {
        st = addScaledU64(totalUs, readUs, 27U);
      }
      break;
    case OperationKind::AUTO_ADJUST_MAINTENANCE:
      totalUs = customWriteUs;
      st = addScaledU64(totalUs, pointerUs, 2U);
      if (st.ok()) {
        st = addScaledU64(totalUs, readUs, 2U);
      }
      break;
    case OperationKind::BUS_ADDRESS_CHANGE:
      totalUs = customWriteUs;
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
  st = _readAndValidateIdentityRaw(identityCandidate);
  if (!st.ok()) {
    const bool acceptedAbsence =
        _config.beginPolicy == BeginPolicy::ALLOW_ABSENT &&
        st.code == Err::DEVICE_NOT_FOUND;
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
  out.operatingFunctions = _capabilities.operatingFunctions;
  out.operatingModeSupport = _capabilities.operatingModeSupport;
  out.specialFeatures = _capabilities.specialFeatures;
  out.lastOkMs = _lastOkMs;
  out.lastErrorMs = _lastErrorMs;
  out.lastError = _lastError;
  out.consecutiveFailures = _consecutiveFailures;
  out.totalFailures = _totalFailures;
  out.totalSuccess = _totalSuccess;
  out.persistentConfigDirty = _mutationDiagnostic.unresolved;
  out.persistentConfigDirtyError =
      _mutationDiagnostic.unresolved
          ? _mutationDiagnostic.cause
          : Status::Ok();
  out.mutation = _mutationDiagnostic;
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
}

void EE871::_publishIdentityAndCapabilities(
    const DeviceIdentity& identity,
    const CapabilitySnapshot& capabilities) {
  _identity = identity;
  _capabilities = capabilities;
}

void EE871::_clearIdentityAndCapabilities() {
  _identity = DeviceIdentity{};
  _capabilities = CapabilitySnapshot{};
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

Status EE871::_mutationAdmissionGuard() const {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  if (_mutationDiagnostic.unresolved) {
    return Status::Error(
        Err::PERSISTENT_STATE_UNCERTAIN,
        "Persistent state unresolved; call resyncPersistentConfig()");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  return Status::Ok();
}

Status EE871::_beginMutation(
    MutationTarget target,
    uint8_t firstAddress,
    const uint8_t* values,
    uint8_t elementCount) {
  Status st = _mutationAdmissionGuard();
  if (!st.ok()) {
    return st;
  }
  if (target == MutationTarget::NONE || values == nullptr ||
      elementCount == 0U || elementCount > 16U ||
      static_cast<uint16_t>(firstAddress) + elementCount > 256U) {
    return Status::Error(
        Err::INVALID_PARAM, "Invalid mutation intent", elementCount);
  }

  _mutationDiagnostic = MutationDiagnostic{};
  _mutationDiagnostic.target = target;
  _mutationDiagnostic.firstAddress = firstAddress;
  _mutationDiagnostic.lastAddress =
      static_cast<uint8_t>(firstAddress + elementCount - 1U);
  _mutationDiagnostic.elementsRequested = elementCount;
  _mutationDiagnostic.attemptedValue = values[elementCount - 1U];

  _mutationIntent = MutationIntent{};
  _mutationIntent.target = target;
  _mutationIntent.firstAddress = firstAddress;
  _mutationIntent.elementCount = elementCount;
  for (uint8_t i = 0; i < elementCount; ++i) {
    _mutationIntent.values[i] = values[i];
  }
  return Status::Ok();
}

void EE871::_classifyMutationEffect(
    const Status& status,
    const MutationProgress& progress) {
  if (status.ok()) {
    _mutationDiagnostic.effect = MutationEffect::ACKNOWLEDGED;
    _mutationDiagnostic.unresolved = true;
    return;
  }

  if (progress.requestAcknowledged ||
      _mutationDiagnostic.elementsAcknowledged != 0U) {
    _mutationDiagnostic.effect = MutationEffect::ACKNOWLEDGED;
    _mutationDiagnostic.unresolved = true;
  } else if (progress.finalAckObserved) {
    _mutationDiagnostic.effect = MutationEffect::NO_EFFECT;
    _mutationDiagnostic.unresolved = false;
  } else if (progress.pecTransferred && status.code != Err::NACK) {
    _mutationDiagnostic.effect = MutationEffect::INDETERMINATE;
    _mutationDiagnostic.unresolved = true;
  } else {
    _mutationDiagnostic.effect = MutationEffect::NO_EFFECT;
    _mutationDiagnostic.unresolved = false;
  }

  if (_mutationDiagnostic.cause.ok()) {
    _mutationDiagnostic.cause = status;
  }
  if (!_mutationDiagnostic.unresolved) {
    _mutationIntent = MutationIntent{};
  }
}

Status EE871::_writeCustomByteEffectful(
    uint8_t address,
    uint8_t value,
    MutationTarget target,
    ClockWaitClass completionClass,
    MutationProgress& progress) {
  progress = MutationProgress{};
  if (_mutationIntent.target != target ||
      _mutationDiagnostic.target != target) {
    return Status::Error(
        Err::INVALID_PARAM, "Mutation target does not match admitted intent");
  }

  _mutationDiagnostic.attemptedValue = value;
  const uint8_t control =
      cmd::makeControlWrite(
          cmd::MAIN_CUSTOM_WRITE, _config.deviceAddress);
  Status st = _writeCommandTracked(
      control, address, value, completionClass, &progress);
  if (progress.requestAcknowledged) {
    ++_mutationDiagnostic.elementsAcknowledged;
  }
  _classifyMutationEffect(st, progress);
  return st;
}

void EE871::_resolveMutation(MutationEffect effect) {
  _mutationDiagnostic.effect = effect;
  _mutationDiagnostic.unresolved = false;
  _mutationIntent = MutationIntent{};
}

Status EE871::_observeMutationBytes(
    uint8_t firstAddress,
    const uint8_t* expected,
    uint8_t elementCount,
    bool resolveOnSuccess) {
  if (expected == nullptr || elementCount == 0U ||
      static_cast<uint16_t>(firstAddress) + elementCount > 256U) {
    return Status::Error(Err::INVALID_PARAM, "Invalid mutation observation");
  }

  Status st = setCustomPointer(firstAddress);
  if (!st.ok()) {
    if (_mutationDiagnostic.cause.ok()) {
      _mutationDiagnostic.cause = st;
    }
    _mutationDiagnostic.effect =
        _mutationDiagnostic.elementsAcknowledged != 0U
            ? MutationEffect::ACKNOWLEDGED
            : MutationEffect::INDETERMINATE;
    _mutationDiagnostic.unresolved = true;
    return st;
  }

  uint8_t observedValues[16] = {};
  for (uint8_t i = 0; i < elementCount; ++i) {
    st = readControlByte(
        cmd::MAIN_CUSTOM_PTR, observedValues[i]);
    if (!st.ok()) {
      if (_mutationDiagnostic.cause.ok()) {
        _mutationDiagnostic.cause = st;
      }
      _mutationDiagnostic.effect =
          _mutationDiagnostic.elementsAcknowledged != 0U
              ? MutationEffect::ACKNOWLEDGED
              : MutationEffect::INDETERMINATE;
      _mutationDiagnostic.unresolved = true;
      return st;
    }
    ++_mutationDiagnostic.elementsObserved;
    _mutationDiagnostic.observedValue = observedValues[i];
    _mutationDiagnostic.observedValueValid = true;
  }

  st = _validateMutationObservation(
      _mutationDiagnostic.target,
      observedValues,
      elementCount);
  if (!st.ok()) {
    if (_mutationDiagnostic.cause.ok()) {
      _mutationDiagnostic.cause = st;
    }
    _mutationDiagnostic.effect =
        _mutationDiagnostic.elementsAcknowledged != 0U
            ? MutationEffect::ACKNOWLEDGED
            : MutationEffect::INDETERMINATE;
    _mutationDiagnostic.unresolved = true;
    return st;
  }

  Status firstMismatch = Status::Ok();
  for (uint8_t i = 0; i < elementCount; ++i) {
    if (observedValues[i] != expected[i]) {
      if (firstMismatch.ok()) {
        firstMismatch = Status::Error(
            Err::VERIFY_MISMATCH,
            "Write verification mismatch",
            observedValues[i]);
      }
      if (_mutationDiagnostic.cause.ok()) {
        _mutationDiagnostic.cause = firstMismatch;
      }
    } else {
      ++_mutationDiagnostic.elementsMatched;
    }
  }

  if (!firstMismatch.ok()) {
    _mutationDiagnostic.effect = MutationEffect::ACKNOWLEDGED;
    _mutationDiagnostic.unresolved = true;
    return firstMismatch;
  }

  if (resolveOnSuccess) {
    _resolveMutation(MutationEffect::VERIFIED);
  }
  return Status::Ok();
}

Status EE871::_writeVerifiedBytes(
    MutationTarget target,
    uint8_t firstAddress,
    const uint8_t* values,
    uint8_t elementCount) {
  Status st = _beginMutation(
      target, firstAddress, values, elementCount);
  if (!st.ok()) {
    return st;
  }

  for (uint8_t i = 0; i < elementCount; ++i) {
    MutationProgress progress;
    st = _writeCustomByteEffectful(
        static_cast<uint8_t>(firstAddress + i),
        values[i],
        target,
        ClockWaitClass::WRITE_COMPLETION,
        progress);
    if (!st.ok()) {
      return st;
    }
    st = _observeMutationBytes(
        static_cast<uint8_t>(firstAddress + i),
        &values[i],
        1U,
        i + 1U == elementCount);
    if (!st.ok()) {
      return st;
    }
  }
  return Status::Ok();
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
    if (enteredOffline || st.code == Err::NOT_SUPPORTED) {
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

Status EE871::_readAndValidateIdentityRaw(DeviceIdentity& out) {
  return _readAndValidateIdentity(out, false);
}

Status EE871::_readAndValidateIdentityTracked(DeviceIdentity& out) {
  return _readAndValidateIdentity(out, true);
}

Status EE871::_readAndValidateIdentity(
    DeviceIdentity& out,
    bool tracked) {
  out = DeviceIdentity{};
  auto readMain =
      [this, tracked](uint8_t mainCommand, uint8_t& value) {
        const uint8_t control =
            cmd::makeControlRead(mainCommand, _config.deviceAddress);
        if (tracked) {
          return _readControlByteTracked(control, value);
        }
        return _readControlByteRaw(control, value);
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

Status EE871::_validateCapabilityValue(
    uint8_t address, uint8_t value) {
  uint8_t reservedMask = 0;
  switch (address) {
    case cmd::CUSTOM_ADJUSTMENT_SUPPORT:
      reservedMask = cmd::CUSTOM_ADJUSTMENT_SUPPORT_RESERVED_MASK;
      break;
    case cmd::CUSTOM_ADJUSTMENT_POINT_SUPPORT:
      reservedMask =
          cmd::CUSTOM_ADJUSTMENT_POINT_SUPPORT_RESERVED_MASK;
      break;
    case cmd::CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT:
      reservedMask =
          cmd::CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT_RESERVED_MASK;
      break;
    case cmd::CUSTOM_ADJUSTMENT_TIME_SUPPORT:
      reservedMask =
          cmd::CUSTOM_ADJUSTMENT_TIME_SUPPORT_RESERVED_MASK;
      break;
    case cmd::CUSTOM_OPERATING_FUNCTIONS:
      reservedMask = cmd::OPERATING_FUNCTIONS_RESERVED_MASK;
      break;
    case cmd::CUSTOM_OPERATING_MODE_SUPPORT:
      reservedMask = cmd::OPERATING_MODE_SUPPORT_RESERVED_MASK;
      break;
    case cmd::CUSTOM_SPECIAL_FEATURES:
      reservedMask = cmd::SPECIAL_FEATURES_RESERVED_MASK;
      break;
    default:
      return Status::Error(
          Err::INVALID_PARAM, "Not a capability address", address);
  }

  if ((value & reservedMask) != 0U) {
    return Status::Error(
        Err::NOT_SUPPORTED,
        "Capability reserved bit set",
        cmd::makeCapabilityValidationDetail(address, value));
  }
  return Status::Ok();
}

Status EE871::_validateBusAddressValue(uint8_t address) {
  if (address > cmd::BUS_ADDRESS_MAX) {
    return Status::Error(
        Err::OUT_OF_RANGE, "Bus address out of range", address);
  }
  return Status::Ok();
}

Status EE871::_validateIntervalValue(uint16_t intervalDeciSeconds) {
  if (intervalDeciSeconds < cmd::INTERVAL_MIN_DECISEC ||
      intervalDeciSeconds > cmd::INTERVAL_MAX_DECISEC) {
    return Status::Error(
        Err::OUT_OF_RANGE,
        "Interval must be 150-36000 (15-3600s)",
        intervalDeciSeconds);
  }
  return Status::Ok();
}

Status EE871::_validateIntervalFactorValue(int8_t factor) {
  if (factor == 0) {
    return Status::Error(
        Err::OUT_OF_RANGE, "Interval factor must be nonzero", 0);
  }
  return Status::Ok();
}

Status EE871::_validateOperatingModeValue(uint8_t mode) const {
  if ((mode & cmd::OPERATING_MODE_RESERVED_MASK) != 0U) {
    return Status::Error(
        Err::OUT_OF_RANGE, "Operating mode reserved bit set", mode);
  }
  if ((mode & cmd::OPERATING_MODE_MEASUREMODE_MASK) != 0U &&
      !hasLowPowerMode()) {
    return Status::Error(
        Err::NOT_SUPPORTED, "Low power mode not supported", mode);
  }
  if ((mode & cmd::OPERATING_MODE_E2_PRIORITY_MASK) != 0U &&
      !hasE2Priority()) {
    return Status::Error(
        Err::NOT_SUPPORTED, "E2 priority mode not supported", mode);
  }
  return Status::Ok();
}

Status EE871::_validateAutoAdjustRaw(uint8_t raw) {
  if ((raw & cmd::AUTO_ADJUST_RESERVED_MASK) != 0U) {
    return Status::Error(
        Err::OUT_OF_RANGE, "Auto-adjust reserved bit set", raw);
  }
  return Status::Ok();
}

Status EE871::_validateMutationObservation(
    MutationTarget target,
    const uint8_t* values,
    uint8_t elementCount) const {
  if (values == nullptr || elementCount == 0U) {
    return Status::Error(
        Err::INVALID_PARAM, "Invalid mutation observation");
  }
  switch (target) {
    case MutationTarget::BUS_ADDRESS:
      return _validateBusAddressValue(values[0]);
    case MutationTarget::GLOBAL_INTERVAL:
      if (elementCount < 2U) {
        return Status::Error(
            Err::INVALID_PARAM, "Interval observation is incomplete");
      }
      return _validateIntervalValue(
          static_cast<uint16_t>(values[0]) |
          (static_cast<uint16_t>(values[1]) << 8));
    case MutationTarget::CO2_INTERVAL_FACTOR:
      return _validateIntervalFactorValue(
          signedByteFromRaw(values[0]));
    case MutationTarget::OPERATING_MODE:
      return _validateOperatingModeValue(values[0]);
    case MutationTarget::AUTO_ADJUST:
      return _validateAutoAdjustRaw(values[0]);
    case MutationTarget::NONE:
    case MutationTarget::RAW_CUSTOM_BYTE:
    case MutationTarget::PART_NAME:
    case MutationTarget::CO2_FILTER:
    case MutationTarget::CO2_OFFSET:
    case MutationTarget::CO2_GAIN:
      return Status::Ok();
  }
  return Status::Error(
      Err::INVALID_PARAM, "Invalid mutation target");
}

Status EE871::_readValidatedCapability(
    uint8_t address, uint8_t& out) {
  uint8_t candidate = 0;
  Status st = customRead(address, candidate);
  if (!st.ok()) {
    return st;
  }
  st = _validateCapabilityValue(address, candidate);
  if (!st.ok()) {
    return st;
  }
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

  for (uint8_t i = 0; i < 7U; ++i) {
    st = _validateCapabilityValue(
        static_cast<uint8_t>(
            cmd::CUSTOM_ADJUSTMENT_SUPPORT + i),
        values[i]);
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
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  return _mutationDiagnostic.unresolved
             ? _resyncUnresolvedMutation()
             : _resyncAllSupportedPersistentConfig();
}

Status EE871::_resyncUnresolvedMutation() {
  if (_mutationIntent.target != _mutationDiagnostic.target ||
      _mutationIntent.elementCount == 0U) {
    return Status::Error(
        Err::PERSISTENT_STATE_UNCERTAIN,
        "Persistent mutation intent unavailable");
  }

  if (_mutationIntent.target == MutationTarget::BUS_ADDRESS &&
      _config.deviceAddress != _mutationIntent.values[0]) {
    return Status::Error(
        Err::PERSISTENT_STATE_UNCERTAIN,
        "Configure the retained candidate address before resync",
        _mutationIntent.values[0]);
  }

  switch (_mutationIntent.target) {
    case MutationTarget::PART_NAME:
      if (!hasPartName()) {
        return Status::Error(
            Err::NOT_SUPPORTED, "Part name not supported");
      }
      break;
    case MutationTarget::BUS_ADDRESS:
      if (!hasAddressConfig()) {
        return Status::Error(
            Err::NOT_SUPPORTED, "Address config not supported");
      }
      break;
    case MutationTarget::GLOBAL_INTERVAL:
      if (!hasGlobalInterval()) {
        return Status::Error(
            Err::NOT_SUPPORTED, "Global interval not supported");
      }
      break;
    case MutationTarget::CO2_INTERVAL_FACTOR:
      if (!hasSpecificInterval()) {
        return Status::Error(
            Err::NOT_SUPPORTED, "Specific interval not supported");
      }
      break;
    case MutationTarget::CO2_FILTER:
      if (!hasFilterConfig()) {
        return Status::Error(
            Err::NOT_SUPPORTED, "Filter config not supported");
      }
      break;
    case MutationTarget::AUTO_ADJUST:
      if (!hasAutoAdjust()) {
        return Status::Error(
            Err::NOT_SUPPORTED, "Auto adjust not supported");
      }
      break;
    case MutationTarget::CO2_OFFSET:
    case MutationTarget::CO2_GAIN:
      if (!hasCo2OffsetGain()) {
        return Status::Error(
            Err::NOT_SUPPORTED, "CO2 offset/gain not supported");
      }
      break;
    case MutationTarget::OPERATING_MODE:
      if (!hasLowPowerMode() && !hasE2Priority()) {
        return Status::Error(
            Err::NOT_SUPPORTED, "Operating mode not supported");
      }
      break;
    case MutationTarget::NONE:
      return Status::Error(
          Err::PERSISTENT_STATE_UNCERTAIN,
          "Persistent mutation target unavailable");
    case MutationTarget::RAW_CUSTOM_BYTE:
      break;
  }

  _mutationDiagnostic.elementsObserved = 0;
  _mutationDiagnostic.elementsMatched = 0;
  _mutationDiagnostic.observedValue = 0;
  _mutationDiagnostic.observedValueValid = false;

  if (_mutationIntent.target == MutationTarget::AUTO_ADJUST) {
    uint8_t observed = 0;
    Status st = customRead(cmd::CUSTOM_AUTO_ADJUST, observed);
    if (!st.ok()) {
      return st;
    }
    _mutationDiagnostic.elementsObserved = 1;
    _mutationDiagnostic.observedValue = observed;
    _mutationDiagnostic.observedValueValid = true;
    st = _validateAutoAdjustRaw(observed);
    if (!st.ok()) {
      return st;
    }
    const bool running =
        (observed & cmd::AUTO_ADJUST_RUNNING_MASK) != 0U;
    if (running &&
        _mutationDiagnostic.preObservedValueValid &&
        (_mutationDiagnostic.preObservedValue &
         cmd::AUTO_ADJUST_RUNNING_MASK) == 0U) {
      _mutationDiagnostic.elementsMatched = 1;
      _resolveMutation(MutationEffect::VERIFIED);
      return Status::Ok();
    }
    _mutationIntent.autoAdjustNotRunningObservedAfterFailure = true;
    return Status::Error(
        Err::PERSISTENT_STATE_UNCERTAIN,
        "Auto-adjust history remains ambiguous");
  }

  uint8_t observed[16] = {};
  Status st = setCustomPointer(_mutationIntent.firstAddress);
  if (!st.ok()) {
    return st;
  }
  for (uint8_t i = 0; i < _mutationIntent.elementCount; ++i) {
    st = readControlByte(cmd::MAIN_CUSTOM_PTR, observed[i]);
    if (!st.ok()) {
      return st;
    }
    ++_mutationDiagnostic.elementsObserved;
    _mutationDiagnostic.observedValue = observed[i];
    _mutationDiagnostic.observedValueValid = true;
  }

  st = _validateMutationObservation(
      _mutationIntent.target,
      observed,
      _mutationIntent.elementCount);
  if (!st.ok()) {
    return st;
  }

  for (uint8_t i = 0; i < _mutationIntent.elementCount; ++i) {
    if (observed[i] == _mutationIntent.values[i]) {
      ++_mutationDiagnostic.elementsMatched;
    }
  }

  _resolveMutation(
      _mutationDiagnostic.elementsMatched ==
              _mutationDiagnostic.elementsRequested
          ? MutationEffect::VERIFIED
          : MutationEffect::RESYNCHRONIZED);
  return Status::Ok();
}

Status EE871::_resyncAllSupportedPersistentConfig() {
  Status st;

  if (hasPartName()) {
    uint8_t bytes[cmd::CUSTOM_PART_NAME_LEN] = {};
    st = readPartName(bytes);
    if (!st.ok()) {
      return st;
    }
  }
  if (hasAddressConfig()) {
    uint8_t address = 0;
    st = readBusAddress(address);
    if (!st.ok()) {
      return st;
    }
  }
  if (hasGlobalInterval()) {
    uint16_t interval = 0;
    st = readMeasurementInterval(interval);
    if (!st.ok()) {
      return st;
    }
  }
  if (hasSpecificInterval()) {
    int8_t factor = 0;
    st = readCo2IntervalFactor(factor);
    if (!st.ok()) {
      return st;
    }
  }
  if (hasFilterConfig()) {
    uint8_t filter = 0;
    st = readCo2Filter(filter);
    if (!st.ok()) {
      return st;
    }
  }
  if (hasLowPowerMode() || hasE2Priority()) {
    uint8_t mode = 0;
    st = readOperatingMode(mode);
    if (!st.ok()) {
      return st;
    }
  }
  if (hasAutoAdjust()) {
    bool running = false;
    st = readAutoAdjustStatus(running);
    if (!st.ok()) {
      return st;
    }
  }
  if (hasCo2OffsetGain()) {
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
  }
  return Status::Ok();
}

Status EE871::acknowledgeAutoAdjustUncertainty() {
  if (!_mutationDiagnostic.unresolved ||
      _mutationDiagnostic.target != MutationTarget::AUTO_ADJUST ||
      _mutationIntent.target != MutationTarget::AUTO_ADJUST ||
      !_mutationIntent.autoAdjustNotRunningObservedAfterFailure ||
      !_mutationDiagnostic.observedValueValid ||
      (_mutationDiagnostic.observedValue &
       cmd::AUTO_ADJUST_RUNNING_MASK) != 0U) {
    return Status::Error(
        Err::INVALID_PARAM,
        "No observed not-running auto-adjust ambiguity");
  }
  _resolveMutation(MutationEffect::OPERATOR_ACKNOWLEDGED);
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
  Status st = _mutationAdmissionGuard();
  if (!st.ok()) {
    return st;
  }

  switch (classifyCustomWriteAddress(address)) {
    case CustomWriteRoute::READ_ONLY:
      return Status::Error(
          Err::NOT_SUPPORTED, "Custom address is read-only", address);
    case CustomWriteRoute::BUS_ADDRESS:
      return _writeBusAddressDirect(value);
    case CustomWriteRoute::INTERVAL_PAIR:
      return Status::Error(
          Err::NOT_SUPPORTED,
          "Use writeMeasurementInterval() for the interval pair",
          address);
    case CustomWriteRoute::INTERVAL_FACTOR:
      return _writeCo2IntervalFactorDirect(
          signedByteFromRaw(value));
    case CustomWriteRoute::FILTER:
      return _writeCo2FilterDirect(value);
    case CustomWriteRoute::OPERATING_MODE:
      return _writeOperatingModeDirect(value);
    case CustomWriteRoute::AUTO_ADJUST:
      return value == 1U
                 ? _startAutoAdjustDirect()
                 : Status::Error(
                       Err::NOT_SUPPORTED,
                       "Only value 1 can start auto-adjust",
                       value);
    case CustomWriteRoute::CO2_PAIR:
      return Status::Error(
          Err::NOT_SUPPORTED,
          "Use the paired CO2 offset/gain API",
          address);
    case CustomWriteRoute::RAW:
      return _writeVerifiedBytes(
          MutationTarget::RAW_CUSTOM_BYTE,
          address,
          &value,
          1U);
  }
  return Status::Error(Err::INVALID_PARAM, "Invalid custom-write route");
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

Status EE871::writeMeasurementInterval(uint16_t intervalDeciSeconds) {
  return _writeMeasurementIntervalDirect(intervalDeciSeconds);
}

Status EE871::_writeMeasurementIntervalDirect(
    uint16_t intervalDeciSeconds) {
  Status guard = _mutationAdmissionGuard();
  if (!guard.ok()) {
    return guard;
  }
  Status st = _validateIntervalValue(intervalDeciSeconds);
  if (!st.ok()) {
    return st;
  }
  if (!hasGlobalInterval()) {
    return Status::Error(Err::NOT_SUPPORTED, "Global interval not supported");
  }

  const uint8_t values[2] = {
      static_cast<uint8_t>(intervalDeciSeconds & 0xFFU),
      static_cast<uint8_t>(intervalDeciSeconds >> 8)};
  st = _beginMutation(
      MutationTarget::GLOBAL_INTERVAL,
      cmd::CUSTOM_INTERVAL_L,
      values,
      2U);
  if (!st.ok()) {
    return st;
  }

  MutationProgress progress;
  st = _writeCustomByteEffectful(
      cmd::CUSTOM_INTERVAL_L,
      values[0],
      MutationTarget::GLOBAL_INTERVAL,
      ClockWaitClass::NORMAL_BIT,
      progress);
  if (!st.ok()) {
    return st;
  }
  st = _writeCustomByteEffectful(
      cmd::CUSTOM_INTERVAL_H,
      values[1],
      MutationTarget::GLOBAL_INTERVAL,
      ClockWaitClass::INTERVAL_COMMIT,
      progress);
  if (!st.ok()) {
    return st;
  }
  return _observeMutationBytes(
      cmd::CUSTOM_INTERVAL_L, values, 2U, true);
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
  uint8_t values[2] = {};
  Status st = customRead(cmd::CUSTOM_FW_VERSION_MAIN, values, 2U);
  if (st.ok()) {
    main = values[0];
    sub = values[1];
  }
  return st;
}

Status EE871::readE2SpecVersion(uint8_t& version) {
  return customRead(cmd::CUSTOM_E2_SPEC_VERSION, version);
}

// ============================================================================
// Feature Discovery
// ============================================================================

Status EE871::readOperatingFunctions(uint8_t& bits) {
  return _readValidatedCapability(
      cmd::CUSTOM_OPERATING_FUNCTIONS, bits);
}

Status EE871::readOperatingModeSupport(uint8_t& bits) {
  return _readValidatedCapability(
      cmd::CUSTOM_OPERATING_MODE_SUPPORT, bits);
}

Status EE871::readSpecialFeatures(uint8_t& bits) {
  return _readValidatedCapability(
      cmd::CUSTOM_SPECIAL_FEATURES, bits);
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
  Status guard = _mutationAdmissionGuard();
  if (!guard.ok()) {
    return guard;
  }
  if (buf == nullptr) {
    return Status::Error(Err::INVALID_PARAM, "Null buffer");
  }
  if (!hasPartName()) {
    return Status::Error(Err::NOT_SUPPORTED, "Part name not supported");
  }
  return _writeVerifiedBytes(
      MutationTarget::PART_NAME,
      cmd::CUSTOM_PART_NAME_START,
      buf,
      cmd::CUSTOM_PART_NAME_LEN);
}

// ============================================================================
// Bus Address
// ============================================================================

Status EE871::readBusAddress(uint8_t& address) {
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
  uint8_t candidate = 0;
  Status st = customRead(cmd::CUSTOM_BUS_ADDRESS, candidate);
  if (!st.ok()) {
    return st;
  }
  st = _validateBusAddressValue(candidate);
  if (!st.ok()) {
    return st;
  }
  address = candidate;
  return Status::Ok();
}

Status EE871::writeBusAddress(uint8_t address) {
  return _writeBusAddressDirect(address);
}

Status EE871::_writeBusAddressDirect(uint8_t address) {
  Status guard = _mutationAdmissionGuard();
  if (!guard.ok()) {
    return guard;
  }
  Status st = _validateBusAddressValue(address);
  if (!st.ok()) {
    return st;
  }
  if (!hasAddressConfig()) {
    return Status::Error(Err::NOT_SUPPORTED, "Address config not supported");
  }
  st = _beginMutation(
      MutationTarget::BUS_ADDRESS,
      cmd::CUSTOM_BUS_ADDRESS,
      &address,
      1U);
  if (!st.ok()) {
    return st;
  }
  MutationProgress progress;
  st = _writeCustomByteEffectful(
      cmd::CUSTOM_BUS_ADDRESS,
      address,
      MutationTarget::BUS_ADDRESS,
      ClockWaitClass::WRITE_COMPLETION,
      progress);
  if (!st.ok()) {
    return st;
  }
  Status uncertain = Status::Error(
      Err::PERSISTENT_STATE_UNCERTAIN,
      "Bus-address activation requires explicit candidate-session resync",
      address);
  if (_mutationDiagnostic.cause.ok()) {
    _mutationDiagnostic.cause = uncertain;
  }
  return uncertain;
}

// ============================================================================
// Measurement Interval
// ============================================================================

Status EE871::readMeasurementInterval(uint16_t& intervalDeciSeconds) {
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
  uint8_t values[2] = {};
  Status st = customRead(cmd::CUSTOM_INTERVAL_L, values, 2U);
  if (!st.ok()) {
    return st;
  }
  const uint16_t candidate =
      static_cast<uint16_t>(values[0]) |
      (static_cast<uint16_t>(values[1]) << 8);
  st = _validateIntervalValue(candidate);
  if (!st.ok()) {
    return st;
  }
  intervalDeciSeconds = candidate;
  return Status::Ok();
}

Status EE871::readCo2IntervalFactor(int8_t& factor) {
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
  uint8_t raw = 0;
  Status st = customRead(cmd::CUSTOM_CO2_INTERVAL_FACTOR, raw);
  if (!st.ok()) {
    return st;
  }
  const int8_t candidate = signedByteFromRaw(raw);
  st = _validateIntervalFactorValue(candidate);
  if (!st.ok()) {
    return st;
  }
  factor = candidate;
  return Status::Ok();
}

Status EE871::writeCo2IntervalFactor(int8_t factor) {
  return _writeCo2IntervalFactorDirect(factor);
}

Status EE871::_writeCo2IntervalFactorDirect(int8_t factor) {
  Status guard = _mutationAdmissionGuard();
  if (!guard.ok()) {
    return guard;
  }
  Status st = _validateIntervalFactorValue(factor);
  if (!st.ok()) {
    return st;
  }
  if (!hasSpecificInterval()) {
    return Status::Error(Err::NOT_SUPPORTED, "Specific interval not supported");
  }
  const uint8_t value = static_cast<uint8_t>(factor);
  return _writeVerifiedBytes(
      MutationTarget::CO2_INTERVAL_FACTOR,
      cmd::CUSTOM_CO2_INTERVAL_FACTOR,
      &value,
      1U);
}

// ============================================================================
// Filter / Operating Mode
// ============================================================================

Status EE871::readCo2Filter(uint8_t& filter) {
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
  return customRead(cmd::CUSTOM_FILTER_CO2, filter);
}

Status EE871::writeCo2Filter(uint8_t filter) {
  return _writeCo2FilterDirect(filter);
}

Status EE871::_writeCo2FilterDirect(uint8_t filter) {
  Status guard = _mutationAdmissionGuard();
  if (!guard.ok()) {
    return guard;
  }
  if (!hasFilterConfig()) {
    return Status::Error(Err::NOT_SUPPORTED, "Filter config not supported");
  }
  return _writeVerifiedBytes(
      MutationTarget::CO2_FILTER,
      cmd::CUSTOM_FILTER_CO2,
      &filter,
      1U);
}

Status EE871::readOperatingMode(uint8_t& mode) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasLowPowerMode() && !hasE2Priority()) {
    return Status::Error(Err::NOT_SUPPORTED, "Operating mode not supported");
  }
  uint8_t candidate = 0;
  Status st = customRead(cmd::CUSTOM_OPERATING_MODE, candidate);
  if (!st.ok()) {
    return st;
  }
  st = _validateOperatingModeValue(candidate);
  if (!st.ok()) {
    return st;
  }
  mode = candidate;
  return Status::Ok();
}

Status EE871::writeOperatingMode(uint8_t mode) {
  return _writeOperatingModeDirect(mode);
}

Status EE871::_writeOperatingModeDirect(uint8_t mode) {
  Status guard = _mutationAdmissionGuard();
  if (!guard.ok()) {
    return guard;
  }
  Status st = _validateOperatingModeValue(mode);
  if (!st.ok()) {
    return st;
  }
  if (!hasLowPowerMode() && !hasE2Priority()) {
    return Status::Error(Err::NOT_SUPPORTED, "Operating mode not supported");
  }
  return _writeVerifiedBytes(
      MutationTarget::OPERATING_MODE,
      cmd::CUSTOM_OPERATING_MODE,
      &mode,
      1U);
}

// ============================================================================
// Auto Adjustment
// ============================================================================

Status EE871::readAutoAdjustStatus(bool& running) {
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
  uint8_t raw = 0;
  Status st = customRead(cmd::CUSTOM_AUTO_ADJUST, raw);
  if (!st.ok()) {
    return st;
  }
  st = _validateAutoAdjustRaw(raw);
  if (!st.ok()) {
    return st;
  }
  running = (raw & cmd::AUTO_ADJUST_RUNNING_MASK) != 0;
  return Status::Ok();
}

Status EE871::startAutoAdjust() {
  return _startAutoAdjustDirect();
}

Status EE871::_startAutoAdjustDirect() {
  Status guard = _mutationAdmissionGuard();
  if (!guard.ok()) {
    return guard;
  }
  if (!hasAutoAdjust()) {
    return Status::Error(Err::NOT_SUPPORTED, "Auto adjust not supported");
  }

  uint8_t preObserved = 0;
  Status st = customRead(cmd::CUSTOM_AUTO_ADJUST, preObserved);
  if (!st.ok()) {
    return st;
  }
  st = _validateAutoAdjustRaw(preObserved);
  if (!st.ok()) {
    return st;
  }
  if ((preObserved & cmd::AUTO_ADJUST_RUNNING_MASK) != 0U) {
    return Status::Error(Err::BUSY, "Auto-adjust already running");
  }

  const uint8_t requested = 1U;
  st = _beginMutation(
      MutationTarget::AUTO_ADJUST,
      cmd::CUSTOM_AUTO_ADJUST,
      &requested,
      1U);
  if (!st.ok()) {
    return st;
  }
  _mutationDiagnostic.preObservedValue = preObserved;
  _mutationDiagnostic.preObservedValueValid = true;

  MutationProgress progress;
  st = _writeCustomByteEffectful(
      cmd::CUSTOM_AUTO_ADJUST,
      requested,
      MutationTarget::AUTO_ADJUST,
      ClockWaitClass::WRITE_COMPLETION,
      progress);
  if (!st.ok()) {
    return st;
  }

  uint8_t observed = 0;
  st = customRead(cmd::CUSTOM_AUTO_ADJUST, observed);
  if (!st.ok()) {
    if (_mutationDiagnostic.cause.ok()) {
      _mutationDiagnostic.cause = st;
    }
    _mutationDiagnostic.effect = MutationEffect::ACKNOWLEDGED;
    _mutationDiagnostic.unresolved = true;
    return st;
  }
  _mutationDiagnostic.elementsObserved = 1;
  _mutationDiagnostic.observedValue = observed;
  _mutationDiagnostic.observedValueValid = true;
  st = _validateAutoAdjustRaw(observed);
  if (!st.ok()) {
    if (_mutationDiagnostic.cause.ok()) {
      _mutationDiagnostic.cause = st;
    }
    _mutationDiagnostic.effect = MutationEffect::ACKNOWLEDGED;
    _mutationDiagnostic.unresolved = true;
    return st;
  }
  if ((observed & cmd::AUTO_ADJUST_RUNNING_MASK) != 0U) {
    _mutationDiagnostic.elementsMatched = 1;
    _resolveMutation(MutationEffect::VERIFIED);
    return Status::Ok();
  }

  Status uncertain = Status::Error(
      Err::PERSISTENT_STATE_UNCERTAIN,
      "Auto-adjust request acknowledged but not observed running");
  if (_mutationDiagnostic.cause.ok()) {
    _mutationDiagnostic.cause = uncertain;
  }
  _mutationDiagnostic.effect = MutationEffect::ACKNOWLEDGED;
  _mutationDiagnostic.unresolved = true;
  return uncertain;
}

// ============================================================================
// Calibration
// ============================================================================

Status EE871::readCo2Offset(int16_t& offset) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasCo2OffsetGain()) {
    return Status::Error(
        Err::NOT_SUPPORTED, "CO2 offset/gain not supported");
  }
  uint8_t values[2] = {};
  Status st = customRead(cmd::CUSTOM_CO2_OFFSET_L, values, 2U);
  if (!st.ok()) {
    return st;
  }
  const uint16_t raw =
      static_cast<uint16_t>(values[0]) |
      (static_cast<uint16_t>(values[1]) << 8);
  offset = signedWordFromRaw(raw);
  return Status::Ok();
}

Status EE871::writeCo2Offset(int16_t offset) {
  return _writeCo2PairDirect(
      MutationTarget::CO2_OFFSET,
      cmd::CUSTOM_CO2_OFFSET_L,
      static_cast<uint16_t>(offset));
}

Status EE871::readCo2Gain(uint16_t& gain) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasCo2OffsetGain()) {
    return Status::Error(
        Err::NOT_SUPPORTED, "CO2 offset/gain not supported");
  }
  uint8_t values[2] = {};
  Status st = customRead(cmd::CUSTOM_CO2_GAIN_L, values, 2U);
  if (!st.ok()) {
    return st;
  }
  gain = static_cast<uint16_t>(values[0]) |
         (static_cast<uint16_t>(values[1]) << 8);
  return Status::Ok();
}

Status EE871::writeCo2Gain(uint16_t gain) {
  return _writeCo2PairDirect(
      MutationTarget::CO2_GAIN,
      cmd::CUSTOM_CO2_GAIN_L,
      gain);
}

Status EE871::_writeCo2PairDirect(
    MutationTarget target,
    uint8_t firstAddress,
    uint16_t value) {
  Status guard = _mutationAdmissionGuard();
  if (!guard.ok()) {
    return guard;
  }
  if (!hasCo2OffsetGain()) {
    return Status::Error(
        Err::NOT_SUPPORTED, "CO2 offset/gain not supported");
  }
  const uint8_t values[2] = {
      static_cast<uint8_t>(value & 0xFFU),
      static_cast<uint8_t>(value >> 8)};
  return _writeVerifiedBytes(
      target, firstAddress, values, 2U);
}

Status EE871::readCo2CalPoints(uint16_t& lower, uint16_t& upper) {
  if (!_initialized) {
    return Status::Error(Err::NOT_INITIALIZED, "Driver not initialized");
  }
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    return guard;
  }
  if (!hasCo2AdjustmentPoints()) {
    return Status::Error(
        Err::NOT_SUPPORTED, "CO2 adjustment points not supported");
  }
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
    Status st = _waitSclHigh(_config, pulseWait);
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
    uint8_t& data) {
  Status st = _e2Start(_config);
  if (!st.ok()) {
    return st;
  }

  auto cleanup = [this](const Status& primary) {
    const Status cleanupStatus =
        _e2Stop(_config, ClockWaitClass::NORMAL_BIT, nullptr);
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
    MutationProgress* progressOut) {
  MutationProgress progress;

  Status st = _e2Start(_config);
  if (!st.ok()) {
    if (progressOut != nullptr) {
      *progressOut = progress;
    }
    return st;
  }

  auto publishProgress = [progressOut](const MutationProgress& value) {
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
      hasLongCompletion ? _config.byteTimeoutUs : completionLimitUs};
  CompletionBudget completionBudget{
      0U, hasLongCompletion ? completionLimitUs : 0U};
  bool acked = false;
  bool finalAckObserved = false;
  st = _readAck(
      _config,
      acked,
      completionClass,
      completionDeadline,
      &finalAckObserved,
      hasLongCompletion ? &completionBudget : nullptr);
  progress.finalAckObserved = finalAckObserved;
  progress.requestAcknowledged = finalAckObserved && acked;
  if (!st.ok()) {
    const Status cleanupStatus =
        _e2Stop(
            _config,
            completionClass,
            hasLongCompletion ? &completionBudget : nullptr);
    (void)cleanupStatus;
    progress.completionElapsedUs =
        hasLongCompletion
            ? completionBudget.consumedUs
            : completionDeadline.elapsedUs;
    publishProgress(progress);
    return st;
  }
  if (!acked) {
    const Status cleanupStatus = hasLongCompletion
        ? _e2Stop(_config, completionClass, &completionBudget)
        : _e2Stop(_config, ClockWaitClass::NORMAL_BIT, nullptr);
    progress.completionElapsedUs =
        hasLongCompletion
            ? completionBudget.consumedUs
            : completionDeadline.elapsedUs;
    progress.stopCompleted = cleanupStatus.ok();
    publishProgress(progress);
    return Status::Error(Err::NACK, "PEC NACK");
  }

  st = _e2Stop(
      _config,
      completionClass,
      hasLongCompletion ? &completionBudget : nullptr);
  progress.completionElapsedUs =
      hasLongCompletion
          ? completionBudget.consumedUs
          : completionDeadline.elapsedUs;
  if (!st.ok()) {
    publishProgress(progress);
    return st;
  }
  progress.stopCompleted = true;

  if (hasLongCompletion) {
    _finishCompletionBudget(_config, completionBudget);
    progress.completionElapsedUs = completionBudget.consumedUs;
  }

  publishProgress(progress);
  return Status::Ok();
}

Status EE871::_writeCommandTracked(
    uint8_t controlByte,
    uint8_t addressByte,
    uint8_t dataByte,
    ClockWaitClass completionClass,
    MutationProgress* progress) {
  Status guard;
  if (!_normalOperationAllowed(guard)) {
    if (progress != nullptr) {
      *progress = MutationProgress{};
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
