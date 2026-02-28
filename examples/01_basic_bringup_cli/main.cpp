/// @file main.cpp
/// @brief Basic bringup example for EE871 (E2 bus)
/// @note This is an EXAMPLE, not part of the library

#include <Arduino.h>
#include <stdlib.h>
#include "common/Log.h"
#include "common/BoardConfig.h"
#include "common/E2Transport.h"
#include "common/E2Diagnostics.h"

#include "EE871/EE871.h"

// ============================================================================
// Globals
// ============================================================================

EE871::EE871 device;
EE871::Config deviceCfg;  // Stored for diagnostics
bool verboseMode = false;

// ============================================================================
// Bus Trace (example-only)
// ============================================================================

namespace buslog {

enum class EventType : uint8_t {
  SET_SCL = 0,
  SET_SDA = 1,
  READ_SCL = 2,
  READ_SDA = 3,
  DELAY_US = 4
};

struct Event {
  uint32_t tsUs;
  uint16_t data;
  uint8_t type;
  uint8_t value;
};

static constexpr size_t TRACE_CAPACITY = 512;
static constexpr size_t TRACE_MAX_FLUSH_PER_LOOP = 24;
static constexpr size_t TRACE_LINE_MAX = 40;

static Event traceBuffer[TRACE_CAPACITY];
static size_t traceHead = 0;
static size_t traceTail = 0;
static size_t traceCount = 0;
static uint32_t traceDropped = 0;
static bool traceEnabled = false;

inline void clear() {
  traceHead = 0;
  traceTail = 0;
  traceCount = 0;
  traceDropped = 0;
}

inline void setEnabled(bool enabled) {
  traceEnabled = enabled;
}

inline void push(EventType type, uint8_t value, uint16_t data) {
  if (!traceEnabled) {
    return;
  }
  if (traceCount >= TRACE_CAPACITY) {
    traceDropped++;
    return;
  }
  traceBuffer[traceHead] = { micros(), data, static_cast<uint8_t>(type), value };
  traceHead = (traceHead + 1) % TRACE_CAPACITY;
  traceCount++;
}

inline size_t boundedLen(int len, size_t cap) {
  if (len <= 0) {
    return 0;
  }
  const size_t ulen = static_cast<size_t>(len);
  return (ulen >= cap) ? (cap - 1U) : ulen;
}

inline size_t formatEvent(const Event& ev, char* out, size_t cap) {
  const unsigned long ts = static_cast<unsigned long>(ev.tsUs);
  switch (static_cast<EventType>(ev.type)) {
    case EventType::SET_SCL:
      return boundedLen(snprintf(out, cap, "[BUS] %10lu us SCL=%u\n", ts, ev.value), cap);
    case EventType::SET_SDA:
      return boundedLen(snprintf(out, cap, "[BUS] %10lu us SDA=%u\n", ts, ev.value), cap);
    case EventType::READ_SCL:
      return boundedLen(snprintf(out, cap, "[BUS] %10lu us SCL?=%u\n", ts, ev.value), cap);
    case EventType::READ_SDA:
      return boundedLen(snprintf(out, cap, "[BUS] %10lu us SDA?=%u\n", ts, ev.value), cap);
    case EventType::DELAY_US:
      return boundedLen(snprintf(out, cap, "[BUS] %10lu us delay %u us\n", ts, ev.data), cap);
    default:
      return boundedLen(snprintf(out, cap, "[BUS] %10lu us ???\n", ts), cap);
  }
}

inline void flush() {
  if (!traceEnabled) {
    return;
  }
  size_t emitted = 0;
  while (emitted < TRACE_MAX_FLUSH_PER_LOOP && traceCount > 0) {
    if (Serial.availableForWrite() < static_cast<int>(TRACE_LINE_MAX)) {
      break;
    }
    const Event ev = traceBuffer[traceTail];
    traceTail = (traceTail + 1) % TRACE_CAPACITY;
    traceCount--;

    char line[TRACE_LINE_MAX];
    const size_t len = formatEvent(ev, line, sizeof(line));
    if (len > 0) {
      Serial.write(reinterpret_cast<const uint8_t*>(line), len);
    }
    emitted++;
  }
}

inline void printStats() {
  Serial.println("=== Bus Trace ===");
  Serial.printf("  Enabled: %s\n", traceEnabled ? "yes" : "no");
  Serial.printf("  Pending: %u\n", static_cast<unsigned>(traceCount));
  Serial.printf("  Dropped: %lu\n", static_cast<unsigned long>(traceDropped));
  Serial.printf("  Capacity: %u\n", static_cast<unsigned>(TRACE_CAPACITY));
}

} // namespace buslog

namespace trace {

inline void setScl(bool level, void* user) {
  transport::setScl(level, user);
  buslog::push(buslog::EventType::SET_SCL, level ? 1U : 0U, 0);
}

inline void setSda(bool level, void* user) {
  transport::setSda(level, user);
  buslog::push(buslog::EventType::SET_SDA, level ? 1U : 0U, 0);
}

inline bool readScl(void* user) {
  const bool level = transport::readScl(user);
  buslog::push(buslog::EventType::READ_SCL, level ? 1U : 0U, 0);
  return level;
}

inline bool readSda(void* user) {
  const bool level = transport::readSda(user);
  buslog::push(buslog::EventType::READ_SDA, level ? 1U : 0U, 0);
  return level;
}

inline void delayUs(uint32_t us, void* user) {
  const uint16_t clipped = (us > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(us);
  buslog::push(buslog::EventType::DELAY_US, 0, clipped);
  transport::delayUs(us, user);
}

} // namespace trace

// ============================================================================
// Helper Functions
// ============================================================================

static constexpr uint16_t CUSTOM_MEM_SIZE = 0x100;
static constexpr size_t REG_DUMP_CHUNK_LEN = 16;

bool splitToken(const String& in, String& head, String& tail) {
  const int idx = in.indexOf(' ');
  if (idx < 0) {
    head = in;
    head.trim();
    tail = "";
    return head.length() > 0;
  }
  head = in.substring(0, idx);
  tail = in.substring(idx + 1);
  head.trim();
  tail.trim();
  return head.length() > 0;
}

bool parseU8Token(const String& token, uint8_t& out) {
  if (token.isEmpty()) {
    return false;
  }
  char* end = nullptr;
  const unsigned long value = strtoul(token.c_str(), &end, 0);
  if (end == token.c_str() || *end != '\0' || value > 0xFFUL) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

bool parseU16Token(const String& token, uint16_t& out) {
  if (token.isEmpty()) {
    return false;
  }
  char* end = nullptr;
  const unsigned long value = strtoul(token.c_str(), &end, 0);
  if (end == token.c_str() || *end != '\0' || value > 0xFFFFUL) {
    return false;
  }
  out = static_cast<uint16_t>(value);
  return true;
}

/// Convert error code to string
const char* errToStr(EE871::Err err) {
  using EE871::Err;
  switch (err) {
    case Err::OK:                  return "OK";
    case Err::NOT_INITIALIZED:     return "NOT_INITIALIZED";
    case Err::INVALID_CONFIG:      return "INVALID_CONFIG";
    case Err::E2_ERROR:            return "E2_ERROR";
    case Err::TIMEOUT:             return "TIMEOUT";
    case Err::INVALID_PARAM:       return "INVALID_PARAM";
    case Err::DEVICE_NOT_FOUND:    return "DEVICE_NOT_FOUND";
    case Err::PEC_MISMATCH:        return "PEC_MISMATCH";
    case Err::NACK:                return "NACK";
    case Err::BUSY:                return "BUSY";
    case Err::IN_PROGRESS:         return "IN_PROGRESS";
    case Err::BUS_STUCK:           return "BUS_STUCK";
    case Err::ALREADY_INITIALIZED: return "ALREADY_INITIALIZED";
    case Err::OUT_OF_RANGE:        return "OUT_OF_RANGE";
    case Err::NOT_SUPPORTED:       return "NOT_SUPPORTED";
    default:                       return "UNKNOWN";
  }
}

/// Convert driver state to string
const char* stateToStr(EE871::DriverState st) {
  using EE871::DriverState;
  switch (st) {
    case DriverState::UNINIT:   return "UNINIT";
    case DriverState::READY:    return "READY";
    case DriverState::DEGRADED: return "DEGRADED";
    case DriverState::OFFLINE:  return "OFFLINE";
    default:                    return "UNKNOWN";
  }
}

const char* stateColor(EE871::DriverState st, bool online, uint8_t consecutiveFailures) {
  if (st == EE871::DriverState::UNINIT) {
    return LOG_COLOR_RESET;
  }
  return LOG_COLOR_STATE(online, consecutiveFailures);
}

const char* goodIfZeroColor(uint32_t value) {
  return (value == 0U) ? LOG_COLOR_GREEN : LOG_COLOR_RED;
}

const char* goodIfNonZeroColor(uint32_t value) {
  return (value > 0U) ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW;
}

const char* onOffColor(bool enabled) {
  return enabled ? LOG_COLOR_GREEN : LOG_COLOR_RESET;
}

const char* skipCountColor(uint32_t value) {
  return (value > 0U) ? LOG_COLOR_YELLOW : LOG_COLOR_RESET;
}

const char* successRateColor(float pct) {
  if (pct >= 99.9f) return LOG_COLOR_GREEN;
  if (pct >= 80.0f) return LOG_COLOR_YELLOW;
  return LOG_COLOR_RED;
}

/// Print status details
void printStatus(const EE871::Status& st) {
  Serial.printf("  Status: %s%s%s (code=%u, detail=%ld)\n",
                LOG_COLOR_RESULT(st.ok()),
                errToStr(st.code),
                LOG_COLOR_RESET,
                static_cast<unsigned>(st.code),
                static_cast<long>(st.detail));
  if (st.msg && st.msg[0]) {
    Serial.printf("  Message: %s%s%s\n", LOG_COLOR_YELLOW, st.msg, LOG_COLOR_RESET);
  }
}

bool ensureProbeOk() {
  auto st = device.probe();
  if (!st.ok()) {
    LOGW("Probe failed");
    printStatus(st);
    return false;
  }
  return true;
}

/// Print driver health information
void printDriverHealth() {
  const uint32_t now = millis();
  const uint32_t totalOk = device.totalSuccess();
  const uint32_t totalFail = device.totalFailures();
  const uint32_t total = totalOk + totalFail;
  const float successRate = (total > 0U)
                                ? (100.0f * static_cast<float>(totalOk) / static_cast<float>(total))
                                : 0.0f;
  const EE871::Status lastErr = device.lastError();
  const EE871::DriverState st = device.state();
  const bool online = device.isOnline();

  Serial.println("=== Driver Health ===");
  Serial.printf("  State: %s%s%s\n",
                stateColor(st, online, device.consecutiveFailures()),
                stateToStr(st),
                LOG_COLOR_RESET);
  Serial.printf("  Online: %s%s%s\n",
                online ? LOG_COLOR_GREEN : LOG_COLOR_RED,
                log_bool_str(online),
                LOG_COLOR_RESET);
  Serial.printf("  Consecutive failures: %s%u%s\n",
                goodIfZeroColor(device.consecutiveFailures()),
                device.consecutiveFailures(),
                LOG_COLOR_RESET);
  Serial.printf("  Total success: %s%lu%s\n",
                goodIfNonZeroColor(totalOk),
                static_cast<unsigned long>(totalOk),
                LOG_COLOR_RESET);
  Serial.printf("  Total failures: %s%lu%s\n",
                goodIfZeroColor(totalFail),
                static_cast<unsigned long>(totalFail),
                LOG_COLOR_RESET);
  Serial.printf("  Success rate: %s%.1f%%%s\n",
                successRateColor(successRate),
                successRate,
                LOG_COLOR_RESET);

  const uint32_t lastOkMs = device.lastOkMs();
  if (lastOkMs > 0U) {
    Serial.printf("  Last OK: %lu ms ago (at %lu ms)\n",
                  static_cast<unsigned long>(now - lastOkMs),
                  static_cast<unsigned long>(lastOkMs));
  } else {
    Serial.println("  Last OK: never");
  }

  const uint32_t lastErrorMs = device.lastErrorMs();
  if (lastErrorMs > 0U) {
    Serial.printf("  Last error: %lu ms ago (at %lu ms)\n",
                  static_cast<unsigned long>(now - lastErrorMs),
                  static_cast<unsigned long>(lastErrorMs));
  } else {
    Serial.println("  Last error: never");
  }

  if (!lastErr.ok()) {
    Serial.printf("  Error code: %s%s%s\n",
                  LOG_COLOR_RED,
                  errToStr(lastErr.code),
                  LOG_COLOR_RESET);
    Serial.printf("  Error detail: %ld\n", static_cast<long>(lastErr.detail));
    if (lastErr.msg && lastErr.msg[0]) {
      Serial.printf("  Error msg: %s\n", lastErr.msg);
    }
  }
}

/// Print help
void printHelp() {
  auto helpSection = [](const char* title) {
    Serial.printf("\n%s[%s]%s\n", LOG_COLOR_GREEN, title, LOG_COLOR_RESET);
  };
  auto helpItem = [](const char* cmd, const char* desc) {
    Serial.printf("  %s%-32s%s - %s\n", LOG_COLOR_CYAN, cmd, LOG_COLOR_RESET, desc);
  };

  Serial.println();
  Serial.printf("%s=== EE871-E2 CLI Help ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);

  helpSection("Common");
  helpItem("help / ?", "Show this help");
  helpItem("scan", "Scan all 8 E2 addresses");
  helpItem("probe", "Probe device (no health tracking)");
  helpItem("recover", "Attempt recovery");
  helpItem("drv", "Show driver state and health");
  helpItem("read", "Read CO2 average");
  helpItem("cfg / settings", "Show driver state and feature flags");
  helpItem("verbose [0|1]", "Toggle bus trace output (no args = show)");
  helpItem("stress [N]", "Repeated CO2 average reads");
  helpItem("stress_mix [N]", "Mixed safe read operations");
  helpItem("selftest", "Safe command self-test with report");

  helpSection("Device Commands");
  helpItem("id", "Read group/subgroup/available bits");
  helpItem("status", "Read status byte (starts measurement)");
  helpItem("co2fast", "Read MV3 (fast response)");
  helpItem("co2avg", "Read MV4 (averaged)");
  helpItem("error", "Read error code (if status indicates error)");
  helpItem("reg read <addr>", "Read custom register (0x00..0xFF)");
  helpItem("reg write <addr> <value>", "Write custom register and verify");
  helpItem("reg dump [start] [len]", "Dump custom registers (default all)");
  helpItem("ctrl <main_nibble>", "Raw readControlByte(main_nibble)");
  helpItem("u16 <main_lo> <main_hi>", "Raw readU16(main_lo, main_hi)");
  helpItem("ptr <addr16>", "Set custom pointer");

  helpSection("Device Info");
  helpItem("fw", "Read firmware version");
  helpItem("e2spec", "Read E2 spec version");
  helpItem("features", "Read feature support flags");
  helpItem("serial", "Read serial number");
  helpItem("partname", "Read part name");
  helpItem("partname <text>", "Write part name (16 bytes max)");

  helpSection("Configuration");
  helpItem("addr", "Read current bus address");
  helpItem("addr <0-7>", "Write bus address (needs power cycle)");
  helpItem("interval", "Read measurement interval");
  helpItem("interval <dec>", "Write interval (150..36000 deciseconds)");
  helpItem("factor", "Read CO2 interval factor");
  helpItem("factor <val>", "Write CO2 interval factor");
  helpItem("filter", "Read CO2 filter setting");
  helpItem("filter <val>", "Write CO2 filter");
  helpItem("mode", "Read operating mode");
  helpItem("mode <val>", "Write operating mode (0..3)");

  helpSection("Calibration");
  helpItem("offset", "Read CO2 offset (ppm)");
  helpItem("offset <val>", "Write CO2 offset (signed)");
  helpItem("gain", "Read CO2 gain");
  helpItem("gain <val>", "Write CO2 gain");
  helpItem("calpoints", "Read last calibration points");
  helpItem("autoadj", "Read auto-adjust status");
  helpItem("autoadj start", "Start auto-adjustment (~5 min)");

  helpSection("Bus Safety");
  helpItem("buscheck", "Check if bus is idle");
  helpItem("libreset", "Bus reset via library");

  helpSection("Diagnostics");
  helpItem("diag", "Run full diagnostic suite");
  helpItem("levels", "Read current bus levels");
  helpItem("pintest", "Test pin toggle (MCU bus control)");
  helpItem("clocktest", "Generate clock pulses and verify");
  helpItem("sniff", "Toggle sniffer on/off");
  helpItem("timing", "Try different clock frequencies");
  helpItem("busreset", "Send 9 clocks to recover stuck bus");
  helpItem("tx <hex>", "Test transaction with control byte");
  helpItem("libtest", "Test all library commands (begin uses)");
  helpItem("caps", "Print feature capability booleans");
  helpItem("trace stats", "Show bus trace buffer stats");
  helpItem("trace clear", "Clear buffered trace events");
}

void runStressMix(int count) {
  struct OpStats {
    const char* name;
    uint32_t ok;
    uint32_t fail;
  };
  OpStats stats[] = {
      {"readStatus", 0, 0},
      {"readCo2Fast", 0, 0},
      {"readCo2Avg", 0, 0},
      {"readGroup", 0, 0},
      {"readSubgroup", 0, 0},
      {"readAvail", 0, 0},
      {"readFw", 0, 0},
      {"readFeatures", 0, 0},
  };
  const int opCount = static_cast<int>(sizeof(stats) / sizeof(stats[0]));

  const uint32_t succBefore = device.totalSuccess();
  const uint32_t failBefore = device.totalFailures();
  const uint32_t startMs = millis();

  for (int i = 0; i < count; ++i) {
    const int op = i % opCount;
    EE871::Status st = EE871::Status::Ok();

    switch (op) {
      case 0: {
        uint8_t status = 0;
        st = device.readStatus(status);
        break;
      }
      case 1: {
        uint16_t ppm = 0;
        st = device.readCo2Fast(ppm);
        break;
      }
      case 2: {
        uint16_t ppm = 0;
        st = device.readCo2Average(ppm);
        break;
      }
      case 3: {
        uint16_t group = 0;
        st = device.readGroup(group);
        if (st.ok() && group != EE871::cmd::SENSOR_GROUP_ID) {
          st = EE871::Status::Error(EE871::Err::DEVICE_NOT_FOUND, "unexpected group", group);
        }
        break;
      }
      case 4: {
        uint8_t subgroup = 0;
        st = device.readSubgroup(subgroup);
        if (st.ok() && subgroup != EE871::cmd::SENSOR_SUBGROUP_ID) {
          st = EE871::Status::Error(EE871::Err::DEVICE_NOT_FOUND, "unexpected subgroup", subgroup);
        }
        break;
      }
      case 5: {
        uint8_t avail = 0;
        st = device.readAvailableMeasurements(avail);
        break;
      }
      case 6: {
        uint8_t main = 0, sub = 0;
        st = device.readFirmwareVersion(main, sub);
        break;
      }
      case 7: {
        uint8_t ops = 0;
        st = device.readOperatingFunctions(ops);
        break;
      }
      default:
        break;
    }

    if (st.ok()) {
      stats[op].ok++;
    } else {
      stats[op].fail++;
      if (verboseMode) {
        Serial.printf("  [%d] %s failed: %s\n", i, stats[op].name, errToStr(st.code));
      }
    }
  }

  const uint32_t elapsed = millis() - startMs;
  uint32_t okTotal = 0;
  uint32_t failTotal = 0;
  for (int i = 0; i < opCount; ++i) {
    okTotal += stats[i].ok;
    failTotal += stats[i].fail;
  }

  Serial.println("=== stress_mix summary ===");
  const float successPct =
      (count > 0) ? (100.0f * static_cast<float>(okTotal) / static_cast<float>(count)) : 0.0f;
  Serial.printf("  Total: %sok=%lu%s %sfail=%lu%s (%s%.2f%%%s)\n",
                goodIfNonZeroColor(okTotal),
                static_cast<unsigned long>(okTotal),
                LOG_COLOR_RESET,
                goodIfZeroColor(failTotal),
                static_cast<unsigned long>(failTotal),
                LOG_COLOR_RESET,
                successRateColor(successPct),
                successPct,
                LOG_COLOR_RESET);
  Serial.printf("  Duration: %lu ms\n", static_cast<unsigned long>(elapsed));
  if (elapsed > 0) {
    Serial.printf("  Rate: %.2f ops/s\n", (1000.0f * static_cast<float>(count)) / elapsed);
  }
  for (int i = 0; i < opCount; ++i) {
    Serial.printf("  %-11s %sok=%lu%s %sfail=%lu%s\n",
                  stats[i].name,
                  goodIfNonZeroColor(stats[i].ok),
                  static_cast<unsigned long>(stats[i].ok),
                  LOG_COLOR_RESET,
                  goodIfZeroColor(stats[i].fail),
                  static_cast<unsigned long>(stats[i].fail),
                  LOG_COLOR_RESET);
  }
  const uint32_t successDelta = device.totalSuccess() - succBefore;
  const uint32_t failDelta = device.totalFailures() - failBefore;
  Serial.printf("  Health delta: %ssuccess +%lu%s, %sfailures +%lu%s\n",
                goodIfNonZeroColor(successDelta),
                static_cast<unsigned long>(successDelta),
                LOG_COLOR_RESET,
                goodIfZeroColor(failDelta),
                static_cast<unsigned long>(failDelta),
                LOG_COLOR_RESET);
}

void runSelfTest() {
  struct Result {
    uint32_t pass = 0;
    uint32_t fail = 0;
    uint32_t skip = 0;
  } result;

  enum class SelftestOutcome : uint8_t { PASS, FAIL, SKIP };
  auto report = [&](const char* name, SelftestOutcome outcome, const char* note) {
    const bool ok = (outcome == SelftestOutcome::PASS);
    const bool skip = (outcome == SelftestOutcome::SKIP);
    const char* color = skip ? LOG_COLOR_YELLOW : LOG_COLOR_RESULT(ok);
    const char* tag = skip ? "SKIP" : (ok ? "PASS" : "FAIL");
    Serial.printf("  [%s%s%s] %s", color, tag, LOG_COLOR_RESET, name);
    if (note && note[0]) {
      Serial.printf(" - %s", note);
    }
    Serial.println();
    if (skip) {
      result.skip++;
    } else if (ok) {
      result.pass++;
    } else {
      result.fail++;
    }
  };
  auto reportCheck = [&](const char* name, bool ok, const char* note) {
    report(name, ok ? SelftestOutcome::PASS : SelftestOutcome::FAIL, note);
  };
  auto reportSkip = [&](const char* name, const char* note) {
    report(name, SelftestOutcome::SKIP, note);
  };

  Serial.println("=== EE871 selftest (safe commands) ===");

  const uint32_t succBefore = device.totalSuccess();
  const uint32_t failBefore = device.totalFailures();
  const uint8_t consBefore = device.consecutiveFailures();

  EE871::Status st = device.probe();
  if (st.code == EE871::Err::NOT_INITIALIZED) {
    reportSkip("probe responds", "driver not initialized");
    reportSkip("remaining checks", "selftest aborted");
    Serial.printf("Selftest result: pass=%s%lu%s fail=%s%lu%s skip=%s%lu%s\n",
                  goodIfNonZeroColor(result.pass), static_cast<unsigned long>(result.pass), LOG_COLOR_RESET,
                  goodIfZeroColor(result.fail), static_cast<unsigned long>(result.fail), LOG_COLOR_RESET,
                  skipCountColor(result.skip), static_cast<unsigned long>(result.skip), LOG_COLOR_RESET);
    return;
  }
  reportCheck("probe responds", st.ok(), st.ok() ? "" : errToStr(st.code));
  const bool probeNoTrack = device.totalSuccess() == succBefore &&
                            device.totalFailures() == failBefore &&
                            device.consecutiveFailures() == consBefore;
  reportCheck("probe no-health-side-effects", probeNoTrack, "");

  uint16_t group = 0;
  st = device.readGroup(group);
  reportCheck("readGroup", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("group matches", st.ok() && group == EE871::cmd::SENSOR_GROUP_ID, "");

  uint8_t subgroup = 0;
  st = device.readSubgroup(subgroup);
  reportCheck("readSubgroup", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("subgroup matches", st.ok() && subgroup == EE871::cmd::SENSOR_SUBGROUP_ID, "");

  uint8_t avail = 0;
  st = device.readAvailableMeasurements(avail);
  reportCheck("readAvailableMeasurements", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("CO2 measurement bit present", st.ok() && (avail & EE871::cmd::AVAILABLE_MEAS_MASK) != 0, "");

  uint8_t fwMain = 0, fwSub = 0;
  st = device.readFirmwareVersion(fwMain, fwSub);
  reportCheck("readFirmwareVersion", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint8_t e2spec = 0;
  st = device.readE2SpecVersion(e2spec);
  reportCheck("readE2SpecVersion", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint8_t ops = 0, modes = 0, special = 0;
  st = device.readOperatingFunctions(ops);
  reportCheck("readOperatingFunctions", st.ok(), st.ok() ? "" : errToStr(st.code));
  if (st.ok()) st = device.readOperatingModeSupport(modes);
  reportCheck("readOperatingModeSupport", st.ok(), st.ok() ? "" : errToStr(st.code));
  if (st.ok()) st = device.readSpecialFeatures(special);
  reportCheck("readSpecialFeatures", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint8_t status = 0;
  st = device.readStatus(status);
  reportCheck("readStatus", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint16_t fast = 0, avg = 0;
  st = device.readCo2Fast(fast);
  reportCheck("readCo2Fast", st.ok(), st.ok() ? "" : errToStr(st.code));
  st = device.readCo2Average(avg);
  reportCheck("readCo2Average", st.ok(), st.ok() ? "" : errToStr(st.code));

  if (device.hasErrorCode()) {
    uint8_t code = 0;
    st = device.readErrorCode(code);
    reportCheck("readErrorCode", st.ok(), st.ok() ? "" : errToStr(st.code));
  } else {
    reportSkip("readErrorCode", "not supported");
  }

  if (device.hasSerialNumber()) {
    uint8_t sn[16] = {0};
    st = device.readSerialNumber(sn);
    reportCheck("readSerialNumber", st.ok(), st.ok() ? "" : errToStr(st.code));
  } else {
    reportSkip("readSerialNumber", "not supported");
  }

  if (device.hasPartName()) {
    uint8_t name[16] = {0};
    st = device.readPartName(name);
    reportCheck("readPartName", st.ok(), st.ok() ? "" : errToStr(st.code));
  } else {
    reportSkip("readPartName", "not supported");
  }

  uint8_t addr = 0;
  st = device.readBusAddress(addr);
  reportCheck("readBusAddress", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint16_t interval = 0;
  st = device.readMeasurementInterval(interval);
  reportCheck("readMeasurementInterval", st.ok(), st.ok() ? "" : errToStr(st.code));

  int8_t factor = 0;
  st = device.readCo2IntervalFactor(factor);
  reportCheck("readCo2IntervalFactor", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint8_t mode = 0;
  st = device.readOperatingMode(mode);
  reportCheck("readOperatingMode", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint8_t ctrl = 0;
  st = device.readControlByte(EE871::cmd::MAIN_STATUS, ctrl);
  reportCheck("readControlByte(MAIN_STATUS)", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint16_t rawU16 = 0;
  st = device.readU16(EE871::cmd::MAIN_MV4_LO, EE871::cmd::MAIN_MV4_HI, rawU16);
  reportCheck("readU16(MV4)", st.ok(), st.ok() ? "" : errToStr(st.code));

  st = device.recover();
  reportCheck("recover", st.ok(), st.ok() ? "" : errToStr(st.code));
  reportCheck("isOnline", device.isOnline(), "");

  Serial.printf("Selftest result: pass=%s%lu%s fail=%s%lu%s skip=%s%lu%s\n",
                goodIfNonZeroColor(result.pass), static_cast<unsigned long>(result.pass), LOG_COLOR_RESET,
                goodIfZeroColor(result.fail), static_cast<unsigned long>(result.fail), LOG_COLOR_RESET,
                skipCountColor(result.skip), static_cast<unsigned long>(result.skip), LOG_COLOR_RESET);
}

// ============================================================================
// Command Processing
// ============================================================================

void processCommand(const String& cmd) {
  String trimmed = cmd;
  trimmed.trim();

  if (trimmed == "help" || trimmed == "?") {
    printHelp();
  } else if (trimmed == "read") {
    uint16_t ppm = 0;
    auto st = device.readCo2Average(ppm);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  CO2 avg: %u ppm\n", ppm);
    }
  } else if (trimmed == "cfg" || trimmed == "settings") {
    printDriverHealth();
    uint8_t ops = 0;
    uint8_t modes = 0;
    uint8_t special = 0;
    auto st = device.readOperatingFunctions(ops);
    if (st.ok()) st = device.readOperatingModeSupport(modes);
    if (st.ok()) st = device.readSpecialFeatures(special);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  Features: ops=0x%02X modes=0x%02X special=0x%02X\n", ops, modes, special);
    }
  } else if (trimmed == "probe") {
    LOGI("Probing device (no health tracking)...");
    auto st = device.probe();
    printStatus(st);
  } else if (trimmed == "id") {
    uint16_t group = 0;
    uint8_t subgroup = 0;
    uint8_t avail = 0;
    auto st = device.readGroup(group);
    printStatus(st);
    if (st.ok()) {
      st = device.readSubgroup(subgroup);
      printStatus(st);
    }
    if (st.ok()) {
      st = device.readAvailableMeasurements(avail);
      printStatus(st);
    }
    if (st.ok()) {
      Serial.printf("  Group=0x%04X, Subgroup=0x%02X, Available=0x%02X\n",
                    group, subgroup, avail);
    }
  } else if (trimmed == "status") {
    uint8_t status = 0;
    auto st = device.readStatus(status);
    printStatus(st);
    if (st.ok()) {
      e2diag::printStatus(status);
      Serial.printf("  hasCo2Error(): %s\n", EE871::EE871::hasCo2Error(status) ? "YES" : "NO");
    }
  } else if (trimmed == "co2fast") {
    uint16_t ppm = 0;
    auto st = device.readCo2Fast(ppm);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  CO2 fast: %u ppm\n", ppm);
    }
  } else if (trimmed == "co2avg") {
    uint16_t ppm = 0;
    auto st = device.readCo2Average(ppm);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  CO2 avg: %u ppm\n", ppm);
    }
  } else if (trimmed == "error") {
    uint8_t code = 0;
    auto st = device.readErrorCode(code);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  Error code: %u\n", code);
    }
  } else if (trimmed.startsWith("reg ")) {
    String args = trimmed.substring(4);
    args.trim();

    String subcmd;
    String rest;
    if (!splitToken(args, subcmd, rest)) {
      LOGW("Usage: reg read|write|dump");
      return;
    }

    if (subcmd == "read") {
      String addrToken;
      String extra;
      if (!splitToken(rest, addrToken, extra) || extra.length() > 0) {
        LOGW("Usage: reg read <addr>");
        return;
      }
      uint8_t addr = 0;
      if (!parseU8Token(addrToken, addr)) {
        LOGW("Invalid address");
        return;
      }
      if (!ensureProbeOk()) {
        return;
      }
      uint8_t value = 0;
      auto st = device.customRead(addr, value);
      printStatus(st);
      if (st.ok()) {
        Serial.printf("  Reg[0x%02X] = 0x%02X (%u)\n",
                      static_cast<unsigned>(addr),
                      static_cast<unsigned>(value),
                      static_cast<unsigned>(value));
      }
    } else if (subcmd == "write") {
      String addrToken;
      String restAfterAddr;
      if (!splitToken(rest, addrToken, restAfterAddr)) {
        LOGW("Usage: reg write <addr> <value>");
        return;
      }
      String valueToken;
      String extra;
      if (!splitToken(restAfterAddr, valueToken, extra) || extra.length() > 0) {
        LOGW("Usage: reg write <addr> <value>");
        return;
      }
      uint8_t addr = 0;
      uint8_t value = 0;
      if (!parseU8Token(addrToken, addr) || !parseU8Token(valueToken, value)) {
        LOGW("Invalid address/value");
        return;
      }
      if (!ensureProbeOk()) {
        return;
      }
      auto st = device.customWrite(addr, value);
      printStatus(st);
      if (st.ok()) {
        Serial.printf("  Reg[0x%02X] <= 0x%02X\n",
                      static_cast<unsigned>(addr),
                      static_cast<unsigned>(value));
      }
    } else if (subcmd == "dump") {
      uint8_t start = 0;
      uint16_t len = CUSTOM_MEM_SIZE;
      if (rest.length() > 0) {
        String startToken;
        String restAfterStart;
        if (!splitToken(rest, startToken, restAfterStart)) {
          LOGW("Usage: reg dump [start] [len]");
          return;
        }
        if (!parseU8Token(startToken, start)) {
          LOGW("Invalid start");
          return;
        }
        if (restAfterStart.length() > 0) {
          String lenToken;
          String extra;
          if (!splitToken(restAfterStart, lenToken, extra) || extra.length() > 0) {
            LOGW("Usage: reg dump [start] [len]");
            return;
          }
          if (!parseU16Token(lenToken, len)) {
            LOGW("Invalid length");
            return;
          }
        } else {
          len = static_cast<uint16_t>(CUSTOM_MEM_SIZE - start);
        }
      }
      if (len == 0 || (static_cast<uint16_t>(start) + len) > CUSTOM_MEM_SIZE) {
        LOGW("Range out of bounds");
        return;
      }
      if (!ensureProbeOk()) {
        return;
      }

      uint16_t remaining = len;
      uint16_t offset = start;
      Serial.println("=== Custom Register Dump ===");
      while (remaining > 0) {
        const uint16_t chunk =
            (remaining > REG_DUMP_CHUNK_LEN) ? REG_DUMP_CHUNK_LEN : remaining;
        uint8_t buf[REG_DUMP_CHUNK_LEN] = {};
        auto st = device.customRead(static_cast<uint8_t>(offset), buf, chunk);
        if (!st.ok()) {
          printStatus(st);
          return;
        }
        Serial.printf("  0x%02X:", static_cast<unsigned>(offset & 0xFF));
        for (uint16_t i = 0; i < chunk; ++i) {
          Serial.printf(" %02X", static_cast<unsigned>(buf[i]));
        }
        Serial.println();
        offset = static_cast<uint16_t>(offset + chunk);
        remaining = static_cast<uint16_t>(remaining - chunk);
      }
    } else {
      LOGW("Unknown reg subcommand: %s", subcmd.c_str());
    }
  } else if (trimmed.startsWith("ctrl ")) {
    String token = trimmed.substring(5);
    token.trim();
    uint8_t mainNibble = 0;
    if (!parseU8Token(token, mainNibble)) {
      LOGW("Usage: ctrl <main_nibble>");
      return;
    }
    uint8_t value = 0;
    auto st = device.readControlByte(mainNibble, value);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  ctrl(0x%02X) -> 0x%02X (%u)\n",
                    static_cast<unsigned>(mainNibble),
                    static_cast<unsigned>(value),
                    static_cast<unsigned>(value));
    }
  } else if (trimmed.startsWith("u16 ")) {
    String args = trimmed.substring(4);
    args.trim();
    String loTok;
    String hiTok;
    if (!splitToken(args, loTok, hiTok) || hiTok.length() == 0) {
      LOGW("Usage: u16 <main_lo> <main_hi>");
      return;
    }
    uint8_t lo = 0;
    uint8_t hi = 0;
    if (!parseU8Token(loTok, lo) || !parseU8Token(hiTok, hi)) {
      LOGW("Invalid main_lo/main_hi");
      return;
    }
    uint16_t value = 0;
    auto st = device.readU16(lo, hi, value);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  u16(0x%02X,0x%02X) -> 0x%04X (%u)\n",
                    static_cast<unsigned>(lo),
                    static_cast<unsigned>(hi),
                    static_cast<unsigned>(value),
                    static_cast<unsigned>(value));
    }
  } else if (trimmed.startsWith("ptr ")) {
    String token = trimmed.substring(4);
    token.trim();
    uint16_t ptr = 0;
    if (!parseU16Token(token, ptr)) {
      LOGW("Usage: ptr <addr16>");
      return;
    }
    auto st = device.setCustomPointer(ptr);
    printStatus(st);
  } else if (trimmed == "drv") {
    printDriverHealth();
  } else if (trimmed == "recover") {
    LOGI("Attempting recovery...");
    auto st = device.recover();
    printStatus(st);
    printDriverHealth();
  
  // === Device Info Commands ===
  } else if (trimmed == "fw") {
    uint8_t main = 0, sub = 0;
    auto st = device.readFirmwareVersion(main, sub);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  Firmware: %u.%u\n", main, sub);
    }
  } else if (trimmed == "e2spec") {
    uint8_t ver = 0;
    auto st = device.readE2SpecVersion(ver);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  E2 spec version: %u\n", ver);
    }
  } else if (trimmed == "features") {
    uint8_t ops = 0, modes = 0, special = 0;
    auto st = device.readOperatingFunctions(ops);
    printStatus(st);
    if (st.ok()) st = device.readOperatingModeSupport(modes);
    if (st.ok()) st = device.readSpecialFeatures(special);
    if (st.ok()) {
      Serial.printf("  Operating functions (0x07): 0x%02X\n", ops);
      Serial.printf("    Serial number: %s\n", device.hasSerialNumber() ? "yes" : "no");
      Serial.printf("    Part name: %s\n", device.hasPartName() ? "yes" : "no");
      Serial.printf("    Address config: %s\n", device.hasAddressConfig() ? "yes" : "no");
      Serial.printf("    Global interval: %s\n", device.hasGlobalInterval() ? "yes" : "no");
      Serial.printf("    Specific interval: %s\n", device.hasSpecificInterval() ? "yes" : "no");
      Serial.printf("    Filter config: %s\n", device.hasFilterConfig() ? "yes" : "no");
      Serial.printf("    Error code: %s\n", device.hasErrorCode() ? "yes" : "no");
      Serial.printf("  Mode support (0x08): 0x%02X\n", modes);
      Serial.printf("    Low power: %s\n", device.hasLowPowerMode() ? "yes" : "no");
      Serial.printf("    E2 priority: %s\n", device.hasE2Priority() ? "yes" : "no");
      Serial.printf("  Special features (0x09): 0x%02X\n", special);
      Serial.printf("    Auto adjust: %s\n", device.hasAutoAdjust() ? "yes" : "no");
    }
  } else if (trimmed == "caps") {
    Serial.println("=== Capabilities ===");
    Serial.printf("  hasSerialNumber: %s\n", device.hasSerialNumber() ? "true" : "false");
    Serial.printf("  hasPartName: %s\n", device.hasPartName() ? "true" : "false");
    Serial.printf("  hasAddressConfig: %s\n", device.hasAddressConfig() ? "true" : "false");
    Serial.printf("  hasGlobalInterval: %s\n", device.hasGlobalInterval() ? "true" : "false");
    Serial.printf("  hasSpecificInterval: %s\n", device.hasSpecificInterval() ? "true" : "false");
    Serial.printf("  hasFilterConfig: %s\n", device.hasFilterConfig() ? "true" : "false");
    Serial.printf("  hasErrorCode: %s\n", device.hasErrorCode() ? "true" : "false");
    Serial.printf("  hasLowPowerMode: %s\n", device.hasLowPowerMode() ? "true" : "false");
    Serial.printf("  hasE2Priority: %s\n", device.hasE2Priority() ? "true" : "false");
    Serial.printf("  hasAutoAdjust: %s\n", device.hasAutoAdjust() ? "true" : "false");
  } else if (trimmed == "serial") {
    uint8_t sn[16] = {0};
    auto st = device.readSerialNumber(sn);
    printStatus(st);
    if (st.ok()) {
      Serial.print("  Serial: ");
      for (int i = 0; i < 16; i++) {
        if (sn[i] >= 0x20 && sn[i] < 0x7F) Serial.print((char)sn[i]);
        else Serial.print('.');
      }
      Serial.println();
      Serial.print("  Hex: ");
      for (int i = 0; i < 16; i++) Serial.printf("%02X ", sn[i]);
      Serial.println();
    }
  } else if (trimmed == "partname") {
    uint8_t name[16] = {0};
    auto st = device.readPartName(name);
    printStatus(st);
    if (st.ok()) {
      Serial.print("  Part name: ");
      for (int i = 0; i < 16; i++) {
        if (name[i] >= 0x20 && name[i] < 0x7F) Serial.print((char)name[i]);
        else if (name[i] == 0) break;
        else Serial.print('.');
      }
      Serial.println();
    }
  } else if (trimmed.startsWith("partname ")) {
    String text = trimmed.substring(9);
    text.trim();
    if (text.length() == 0) {
      LOGW("Usage: partname <text>");
      return;
    }
    uint8_t out[16] = {0};
    const size_t maxLen = (text.length() > 16) ? 16 : static_cast<size_t>(text.length());
    for (size_t i = 0; i < maxLen; ++i) {
      out[i] = static_cast<uint8_t>(text[i]);
    }
    auto st = device.writePartName(out);
    printStatus(st);
    if (st.ok()) {
      uint8_t verify[16] = {0};
      st = device.readPartName(verify);
      printStatus(st);
      if (st.ok()) {
        Serial.print("  Part name: ");
        for (int i = 0; i < 16; i++) {
          if (verify[i] >= 0x20 && verify[i] < 0x7F) Serial.print((char)verify[i]);
          else if (verify[i] == 0) break;
          else Serial.print('.');
        }
        Serial.println();
      }
    }
  
  // === Configuration Commands ===
  } else if (trimmed == "addr") {
    uint8_t addr = 0;
    auto st = device.readBusAddress(addr);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  Bus address: %u\n", addr);
    }
  } else if (trimmed.startsWith("addr ")) {
    int val = trimmed.substring(5).toInt();
    LOGI("Writing bus address %d (power cycle required)...", val);
    auto st = device.writeBusAddress(static_cast<uint8_t>(val));
    printStatus(st);
  } else if (trimmed == "interval") {
    uint16_t interval = 0;
    auto st = device.readMeasurementInterval(interval);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  Interval: %u deciseconds (%.1f s)\n", interval, interval / 10.0f);
    }
  } else if (trimmed.startsWith("interval ")) {
    int val = trimmed.substring(9).toInt();
    LOGI("Writing interval %d deciseconds...", val);
    auto st = device.writeMeasurementInterval(static_cast<uint16_t>(val));
    printStatus(st);
  } else if (trimmed == "factor") {
    int8_t factor = 0;
    auto st = device.readCo2IntervalFactor(factor);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  CO2 interval factor: %d\n", static_cast<int>(factor));
    }
  } else if (trimmed.startsWith("factor ")) {
    const int val = trimmed.substring(7).toInt();
    if (val < -128 || val > 127) {
      LOGW("factor must be -128..127");
      return;
    }
    auto st = device.writeCo2IntervalFactor(static_cast<int8_t>(val));
    printStatus(st);
  } else if (trimmed == "filter") {
    uint8_t filter = 0;
    auto st = device.readCo2Filter(filter);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  CO2 filter: %u\n", filter);
    }
  } else if (trimmed.startsWith("filter ")) {
    int val = trimmed.substring(7).toInt();
    auto st = device.writeCo2Filter(static_cast<uint8_t>(val));
    printStatus(st);
  } else if (trimmed == "mode") {
    uint8_t mode = 0;
    auto st = device.readOperatingMode(mode);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  Operating mode: 0x%02X\n", mode);
      Serial.printf("    Measure mode: %s\n", (mode & 0x01) ? "low power" : "freerunning");
      Serial.printf("    Priority: %s\n", (mode & 0x02) ? "E2 comm" : "measurement");
    }
  } else if (trimmed.startsWith("mode ")) {
    int val = trimmed.substring(5).toInt();
    auto st = device.writeOperatingMode(static_cast<uint8_t>(val));
    printStatus(st);
  
  // === Calibration Commands ===
  } else if (trimmed == "offset") {
    int16_t offset = 0;
    auto st = device.readCo2Offset(offset);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  CO2 offset: %d ppm\n", offset);
    }
  } else if (trimmed.startsWith("offset ")) {
    int val = trimmed.substring(7).toInt();
    LOGI("Writing CO2 offset %d...", val);
    auto st = device.writeCo2Offset(static_cast<int16_t>(val));
    printStatus(st);
  } else if (trimmed == "gain") {
    uint16_t gain = 0;
    auto st = device.readCo2Gain(gain);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  CO2 gain: %u (factor=%.4f)\n", gain, gain / 32768.0f);
    }
  } else if (trimmed.startsWith("gain ")) {
    int val = trimmed.substring(5).toInt();
    if (val < 0 || val > 65535) {
      LOGW("gain must be 0..65535");
      return;
    }
    auto st = device.writeCo2Gain(static_cast<uint16_t>(val));
    printStatus(st);
  } else if (trimmed == "calpoints") {
    uint16_t lower = 0, upper = 0;
    auto st = device.readCo2CalPoints(lower, upper);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  Cal points: lower=%u ppm, upper=%u ppm\n", lower, upper);
    }
  } else if (trimmed == "autoadj") {
    bool running = false;
    auto st = device.readAutoAdjustStatus(running);
    printStatus(st);
    if (st.ok()) {
      Serial.printf("  Auto adjustment: %s\n", running ? "RUNNING" : "idle");
    }
  } else if (trimmed == "autoadj start") {
    LOGI("Starting auto adjustment (takes ~5 minutes)...");
    auto st = device.startAutoAdjust();
    printStatus(st);
  
  // === Bus Safety Commands ===
  } else if (trimmed == "buscheck") {
    auto st = device.checkBusIdle();
    printStatus(st);
    if (st.ok()) {
      Serial.println("  Bus is idle (both lines high)");
    }
  } else if (trimmed == "libreset") {
    LOGI("Performing library bus reset...");
    auto st = device.busReset();
    printStatus(st);
  
  } else if (trimmed == "verbose") {
    LOGI("Verbose mode: %s%s%s", onOffColor(verboseMode), verboseMode ? "ON" : "OFF", LOG_COLOR_RESET);
  
  } else if (trimmed.startsWith("verbose ")) {
    int val = trimmed.substring(8).toInt();
    verboseMode = (val != 0);
    if (verboseMode) {
      buslog::clear();
    }
    buslog::setEnabled(verboseMode);
    LOGI("Verbose mode: %s%s%s", onOffColor(verboseMode), verboseMode ? "ON" : "OFF", LOG_COLOR_RESET);
  } else if (trimmed == "trace stats") {
    buslog::printStats();
  } else if (trimmed == "trace clear") {
    buslog::clear();
    LOGI("Bus trace cleared");
  
  // === Diagnostic Commands ===
  } else if (trimmed == "diag") {
    e2diag::runFullDiagnostics(deviceCfg);
  } else if (trimmed == "levels") {
    e2diag::printBusLevels(deviceCfg);
  } else if (trimmed == "pintest") {
    e2diag::testPinToggle(deviceCfg);
  } else if (trimmed == "clocktest") {
    e2diag::testClockPulses(deviceCfg, 10);
  } else if (trimmed == "sniff") {
    // Toggle
    if (e2diag::sniffer().isActive()) {
      e2diag::sniffer().stop();
    } else {
      e2diag::sniffer().start(&deviceCfg);
    }
  } else if (trimmed == "scan") {
    e2diag::scanAddresses(deviceCfg);
  } else if (trimmed == "timing") {
    e2diag::discoverTiming(deviceCfg);
  } else if (trimmed == "busreset") {
    e2diag::sendRecoveryClocks(deviceCfg);
  } else if (trimmed.startsWith("tx ")) {
    String hexStr = trimmed.substring(3);
    hexStr.trim();
    uint8_t ctrlByte = (uint8_t)strtol(hexStr.c_str(), nullptr, 16);
    e2diag::testTransaction(deviceCfg, ctrlByte);
  } else if (trimmed == "libtest") {
    e2diag::testLibraryCommands(deviceCfg);
  } else if (trimmed == "selftest") {
    runSelfTest();
  } else if (trimmed == "stress_mix") {
    runStressMix(100);
  } else if (trimmed.startsWith("stress_mix ")) {
    int count = trimmed.substring(11).toInt();
    if (count <= 0) count = 100;
    runStressMix(count);
  } else if (trimmed.startsWith("stress")) {
    int count = 100;
    if (trimmed.length() > 6) {
      count = trimmed.substring(6).toInt();
      if (count <= 0) count = 100;
    }
    int ok = 0;
    int fail = 0;
    bool hasFailure = false;
    EE871::Status firstFailure = EE871::Status::Ok();
    EE871::Status lastFailure = EE871::Status::Ok();
    for (int i = 0; i < count; ++i) {
      uint16_t ppm = 0;
      auto st = device.readCo2Average(ppm);
      if (st.ok()) {
        ++ok;
      } else {
        ++fail;
        if (!hasFailure) {
          firstFailure = st;
          hasFailure = true;
        }
        lastFailure = st;
      }
      device.tick(millis());
    }
    const float successPct = (count > 0) ? (100.0f * static_cast<float>(ok) / static_cast<float>(count)) : 0.0f;
    Serial.printf("Stress: %sok=%d%s %sfail=%d%s total=%d (%s%.2f%%%s)\n",
                  goodIfNonZeroColor(static_cast<uint32_t>(ok)),
                  ok,
                  LOG_COLOR_RESET,
                  goodIfZeroColor(static_cast<uint32_t>(fail)),
                  fail,
                  LOG_COLOR_RESET,
                  count,
                  successRateColor(successPct),
                  successPct,
                  LOG_COLOR_RESET);
    if (hasFailure) {
      Serial.println("Failure details:");
      Serial.println("  First failure:");
      printStatus(firstFailure);
      if (fail > 1) {
        Serial.println("  Last failure:");
        printStatus(lastFailure);
      }
    }

  } else {
    LOGW("Unknown command: %s", trimmed.c_str());
  }
}

// ============================================================================
// Setup and Loop
// ============================================================================

void setup() {
  log_begin(115200);
  delay(200);
  
  Serial.println();
  Serial.println("=== EE871 Bringup Example ===");

  if (!board::initE2()) {
    Serial.println("[E] Failed to initialize E2 pins");
    return;
  }

  Serial.printf("[I] E2 initialized (DATA=%d, CLOCK=%d)\n", board::E2_DATA, board::E2_CLOCK);

  // Configure and store for diagnostics (use trace wrappers for optional bus logging)
  deviceCfg.setScl = trace::setScl;
  deviceCfg.setSda = trace::setSda;
  deviceCfg.readScl = trace::readScl;
  deviceCfg.readSda = trace::readSda;
  deviceCfg.delayUs = trace::delayUs;
  deviceCfg.busUser = &board::e2Pins();
  deviceCfg.deviceAddress = EE871::cmd::DEFAULT_DEVICE_ADDRESS;
  deviceCfg.clockLowUs = board::E2_CLOCK_LOW_US;
  deviceCfg.clockHighUs = board::E2_CLOCK_HIGH_US;
  deviceCfg.bitTimeoutUs = board::E2_BIT_TIMEOUT_US;
  deviceCfg.byteTimeoutUs = board::E2_BYTE_TIMEOUT_US;
  deviceCfg.writeDelayMs = board::E2_WRITE_DELAY_MS;
  deviceCfg.intervalWriteDelayMs = board::E2_INTERVAL_WRITE_DELAY_MS;
  deviceCfg.offlineThreshold = 5;

  auto st = device.begin(deviceCfg);
  if (!st.ok()) {
    Serial.println("[E] Failed to initialize device");
    printStatus(st);
    Serial.println("\n[I] Running basic bus diagnostics...\n");
    e2diag::printBusLevels(deviceCfg);
    Serial.println();
    e2diag::testPinToggle(deviceCfg);
    Serial.println("\n[I] Type 'diag' for full diagnostics, 'help' for commands\n");
  } else {
    Serial.println("[I] Device initialized successfully");
    printDriverHealth();
  }

  Serial.println("\nType 'help' for commands");
  Serial.print("> ");
}

void loop() {
  device.tick(millis());
  e2diag::sniffer().tick();  // Background sniffer (if active)

  static String inputBuffer;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
        Serial.print("> ");
      }
    } else {
      inputBuffer += c;
    }
  }

  buslog::flush();
}

