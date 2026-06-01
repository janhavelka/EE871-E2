/// @file main.cpp
/// @brief ESP-IDF bringup CLI for EE871 (E2 bus)
/// @note This is an EXAMPLE, not part of the library.

#include <cerrno>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>

#include "E2GpioTransport.h"
#include "EE871/EE871.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================================
// CLI styling and example defaults
// ============================================================================

static constexpr const char* LOG_COLOR_RESET = "\033[0m";
static constexpr const char* LOG_COLOR_RED = "\033[31m";
static constexpr const char* LOG_COLOR_GREEN = "\033[32m";
static constexpr const char* LOG_COLOR_YELLOW = "\033[33m";
static constexpr const char* LOG_COLOR_CYAN = "\033[36m";
static constexpr const char* LOG_COLOR_GRAY = "\033[90m";

static constexpr int LOG_LEVEL = 2;
static constexpr size_t HELP_COMMAND_WIDTH = 32U;
static constexpr size_t MAX_LINE_LENGTH = 127U;
static constexpr uint16_t CUSTOM_MEM_SIZE = 0x100;
static constexpr size_t REG_DUMP_CHUNK_LEN = 16;
static constexpr uint32_t STRESS_PROGRESS_UPDATES = 10U;

static constexpr gpio_num_t E2_SCL = GPIO_NUM_6;
static constexpr gpio_num_t E2_SDA = GPIO_NUM_7;
static constexpr uint8_t EE871_ADDRESS = EE871::cmd::DEFAULT_DEVICE_ADDRESS;
static constexpr uint16_t E2_CLOCK_LOW_US = 100;
static constexpr uint16_t E2_CLOCK_HIGH_US = 100;
static constexpr uint32_t E2_BIT_TIMEOUT_US = 25000;
static constexpr uint32_t E2_BYTE_TIMEOUT_US = 35000;
static constexpr uint32_t E2_WRITE_DELAY_MS = 150;
static constexpr uint32_t E2_INTERVAL_WRITE_DELAY_MS = 300;

EE871::EE871 device;
EE871::Config deviceCfg;
ee871_idf::E2GpioBus e2Bus;
bool verboseMode = false;

uint32_t nowMs() {
  return static_cast<uint32_t>(esp_timer_get_time() / 1000LL);
}

uint32_t nowUs() {
  return static_cast<uint32_t>(esp_timer_get_time());
}

void delayMs(uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms));
}

const char* resultColor(bool ok) {
  return ok ? LOG_COLOR_GREEN : LOG_COLOR_RED;
}

const char* stateColor(bool online, uint8_t failures) {
  return online ? ((failures > 0U) ? LOG_COLOR_YELLOW : LOG_COLOR_GREEN) : LOG_COLOR_RED;
}

const char* boolStr(bool value) {
  return value ? "yes" : "no";
}

void logInfo(const char* fmt, ...) {
  if (LOG_LEVEL < 2) {
    return;
  }
  std::printf("%s[I]%s ", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  va_list ap;
  va_start(ap, fmt);
  std::vprintf(fmt, ap);
  va_end(ap);
  std::printf("\n");
}

void logWarn(const char* fmt, ...) {
  if (LOG_LEVEL < 2) {
    return;
  }
  std::printf("%s[W]%s ", LOG_COLOR_YELLOW, LOG_COLOR_RESET);
  va_list ap;
  va_start(ap, fmt);
  std::vprintf(fmt, ap);
  va_end(ap);
  std::printf("\n");
}

void printPrompt() {
  std::printf("> ");
  std::fflush(stdout);
}

// ============================================================================
// Bus trace and sniffer hooks
// ============================================================================

namespace diag {
void onLineSample(bool scl, bool sda);
}

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

Event traceBuffer[TRACE_CAPACITY];
size_t traceHead = 0;
size_t traceTail = 0;
size_t traceCount = 0;
uint32_t traceDropped = 0;
bool traceEnabled = false;

void clear() {
  traceHead = 0;
  traceTail = 0;
  traceCount = 0;
  traceDropped = 0;
}

void setEnabled(bool enabled) {
  traceEnabled = enabled;
}

void push(EventType type, uint8_t value, uint16_t data) {
  if (!traceEnabled) {
    return;
  }
  if (traceCount >= TRACE_CAPACITY) {
    traceDropped++;
    return;
  }
  traceBuffer[traceHead] = {nowUs(), data, static_cast<uint8_t>(type), value};
  traceHead = (traceHead + 1U) % TRACE_CAPACITY;
  traceCount++;
}

void flush() {
  if (!traceEnabled) {
    return;
  }
  size_t emitted = 0;
  while (emitted < TRACE_MAX_FLUSH_PER_LOOP && traceCount > 0U) {
    const Event ev = traceBuffer[traceTail];
    traceTail = (traceTail + 1U) % TRACE_CAPACITY;
    traceCount--;
    const unsigned long ts = static_cast<unsigned long>(ev.tsUs);
    switch (static_cast<EventType>(ev.type)) {
      case EventType::SET_SCL:
        std::printf("[BUS] %10lu us SCL=%u\n", ts, ev.value);
        break;
      case EventType::SET_SDA:
        std::printf("[BUS] %10lu us SDA=%u\n", ts, ev.value);
        break;
      case EventType::READ_SCL:
        std::printf("[BUS] %10lu us SCL?=%u\n", ts, ev.value);
        break;
      case EventType::READ_SDA:
        std::printf("[BUS] %10lu us SDA?=%u\n", ts, ev.value);
        break;
      case EventType::DELAY_US:
        std::printf("[BUS] %10lu us delay %u us\n", ts, ev.data);
        break;
      default:
        std::printf("[BUS] %10lu us ???\n", ts);
        break;
    }
    emitted++;
  }
}

void printStats() {
  std::printf("=== Bus Trace ===\n");
  std::printf("  Enabled: %s\n", traceEnabled ? "yes" : "no");
  std::printf("  Pending: %u\n", static_cast<unsigned>(traceCount));
  std::printf("  Dropped: %lu\n", static_cast<unsigned long>(traceDropped));
  std::printf("  Capacity: %u\n", static_cast<unsigned>(TRACE_CAPACITY));
}

}  // namespace buslog

namespace trace {

void sampleLines(void* user) {
  const bool scl = ee871_idf::readScl(user);
  const bool sda = ee871_idf::readSda(user);
  diag::onLineSample(scl, sda);
}

void setScl(bool level, void* user) {
  ee871_idf::setScl(level, user);
  buslog::push(buslog::EventType::SET_SCL, level ? 1U : 0U, 0);
  sampleLines(user);
}

void setSda(bool level, void* user) {
  ee871_idf::setSda(level, user);
  buslog::push(buslog::EventType::SET_SDA, level ? 1U : 0U, 0);
  sampleLines(user);
}

bool readScl(void* user) {
  const bool level = ee871_idf::readScl(user);
  buslog::push(buslog::EventType::READ_SCL, level ? 1U : 0U, 0);
  return level;
}

bool readSda(void* user) {
  const bool level = ee871_idf::readSda(user);
  buslog::push(buslog::EventType::READ_SDA, level ? 1U : 0U, 0);
  return level;
}

void delayUs(uint32_t us, void* user) {
  (void)user;
  const uint16_t clipped = (us > 0xFFFFu) ? 0xFFFFu : static_cast<uint16_t>(us);
  buslog::push(buslog::EventType::DELAY_US, 0, clipped);
  ee871_idf::delayUs(us, nullptr);
}

}  // namespace trace

// ============================================================================
// Small parsing helpers
// ============================================================================

char* trimInPlace(char* s) {
  while (*s == ' ' || *s == '\t') {
    ++s;
  }
  char* end = s + std::strlen(s);
  while (end > s && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
    --end;
  }
  *end = '\0';
  return s;
}

bool startsWith(const char* s, const char* prefix) {
  return std::strncmp(s, prefix, std::strlen(prefix)) == 0;
}

struct Tokens {
  char storage[MAX_LINE_LENGTH + 1U];
  char* argv[5];
  int argc = 0;
};

void splitTokens(const char* input, Tokens& out) {
  std::strncpy(out.storage, input, sizeof(out.storage) - 1U);
  out.storage[sizeof(out.storage) - 1U] = '\0';
  out.argc = 0;
  char* ctx = nullptr;
  for (char* tok = strtok_r(out.storage, " \t", &ctx);
       tok != nullptr && out.argc < static_cast<int>(sizeof(out.argv) / sizeof(out.argv[0]));
       tok = strtok_r(nullptr, " \t", &ctx)) {
    out.argv[out.argc++] = tok;
  }
}

bool parseU8Token(const char* token, uint8_t& out) {
  if (token == nullptr || token[0] == '\0') {
    return false;
  }
  char* end = nullptr;
  const unsigned long value = std::strtoul(token, &end, 0);
  if (end == token || *end != '\0' || value > 0xFFUL) {
    return false;
  }
  out = static_cast<uint8_t>(value);
  return true;
}

bool parseU16Token(const char* token, uint16_t& out) {
  if (token == nullptr || token[0] == '\0') {
    return false;
  }
  char* end = nullptr;
  const unsigned long value = std::strtoul(token, &end, 0);
  if (end == token || *end != '\0' || value > 0xFFFFUL) {
    return false;
  }
  out = static_cast<uint16_t>(value);
  return true;
}

bool parseIntToken(const char* token, long& out) {
  if (token == nullptr || token[0] == '\0') {
    return false;
  }
  char* end = nullptr;
  const long value = std::strtol(token, &end, 0);
  if (end == token || *end != '\0') {
    return false;
  }
  out = value;
  return true;
}

// ============================================================================
// Driver status formatting
// ============================================================================

const char* errToStr(EE871::Err err) {
  using EE871::Err;
  switch (err) {
    case Err::OK: return "OK";
    case Err::NOT_INITIALIZED: return "NOT_INITIALIZED";
    case Err::INVALID_CONFIG: return "INVALID_CONFIG";
    case Err::E2_ERROR: return "E2_ERROR";
    case Err::TIMEOUT: return "TIMEOUT";
    case Err::INVALID_PARAM: return "INVALID_PARAM";
    case Err::DEVICE_NOT_FOUND: return "DEVICE_NOT_FOUND";
    case Err::PEC_MISMATCH: return "PEC_MISMATCH";
    case Err::NACK: return "NACK";
    case Err::BUSY: return "BUSY";
    case Err::IN_PROGRESS: return "IN_PROGRESS";
    case Err::BUS_STUCK: return "BUS_STUCK";
    case Err::ALREADY_INITIALIZED: return "ALREADY_INITIALIZED";
    case Err::OUT_OF_RANGE: return "OUT_OF_RANGE";
    case Err::NOT_SUPPORTED: return "NOT_SUPPORTED";
    default: return "UNKNOWN";
  }
}

const char* stateToStr(EE871::DriverState st) {
  using EE871::DriverState;
  switch (st) {
    case DriverState::UNINIT: return "UNINIT";
    case DriverState::READY: return "READY";
    case DriverState::DEGRADED: return "DEGRADED";
    case DriverState::OFFLINE: return "OFFLINE";
    default: return "UNKNOWN";
  }
}

const char* driverStateColor(EE871::DriverState st, bool online, uint8_t consecutiveFailures) {
  if (st == EE871::DriverState::UNINIT) {
    return LOG_COLOR_RESET;
  }
  return stateColor(online, consecutiveFailures);
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

uint32_t stressProgressStep(uint32_t total) {
  if (total == 0U) {
    return 0U;
  }
  const uint32_t step = total / STRESS_PROGRESS_UPDATES;
  return (step == 0U) ? 1U : step;
}

void printStressProgress(uint32_t completed, uint32_t total, uint32_t okCount, uint32_t failCount) {
  if (completed == 0U || total == 0U) {
    return;
  }
  const uint32_t step = stressProgressStep(total);
  if (step == 0U || (completed != total && (completed % step) != 0U)) {
    return;
  }
  const float pct = (100.0f * static_cast<float>(completed)) / static_cast<float>(total);
  std::printf("  Progress: %lu/%lu (%s%.0f%%%s, ok=%s%lu%s, fail=%s%lu%s)\n",
              static_cast<unsigned long>(completed),
              static_cast<unsigned long>(total),
              successRateColor(pct),
              static_cast<double>(pct),
              LOG_COLOR_RESET,
              goodIfNonZeroColor(okCount),
              static_cast<unsigned long>(okCount),
              LOG_COLOR_RESET,
              goodIfZeroColor(failCount),
              static_cast<unsigned long>(failCount),
              LOG_COLOR_RESET);
}

void printStatus(const EE871::Status& st) {
  std::printf("  Status: %s%s%s (code=%u, detail=%ld)\n",
              resultColor(st.ok()),
              errToStr(st.code),
              LOG_COLOR_RESET,
              static_cast<unsigned>(st.code),
              static_cast<long>(st.detail));
  if (st.msg != nullptr && st.msg[0] != '\0') {
    std::printf("  Message: %s%s%s\n", LOG_COLOR_YELLOW, st.msg, LOG_COLOR_RESET);
  }
}

void printPersistentDirtyFields(const EE871::SettingsSnapshot& settings) {
  const bool dirty = settings.persistentConfigDirty;
  const EE871::Status dirtyErr = settings.persistentConfigDirtyError;
  std::printf("  persistentConfigDirty: %s%s%s\n",
              dirty ? LOG_COLOR_RED : LOG_COLOR_GREEN,
              dirty ? "yes" : "no",
              LOG_COLOR_RESET);
  std::printf("  persistentConfigDirtyError: %s (code=%u, detail=%ld)\n",
              errToStr(dirtyErr.code),
              static_cast<unsigned>(dirtyErr.code),
              static_cast<long>(dirtyErr.detail));
  std::printf("  persistentConfigDirtyError message: %s\n",
              (dirtyErr.msg != nullptr && dirtyErr.msg[0] != '\0') ? dirtyErr.msg : "<none>");
  std::printf("  resyncNeeded: %s%s%s\n",
              dirty ? LOG_COLOR_RED : LOG_COLOR_GREEN,
              dirty ? "yes" : "no",
              LOG_COLOR_RESET);
}

void printPersistentDirtyState(const char* title) {
  if (title != nullptr && title[0] != '\0') {
    std::printf("%s\n", title);
  }
  EE871::SettingsSnapshot settings;
  const EE871::Status snapshotStatus = device.getSettings(settings);
  if (!snapshotStatus.ok()) {
    printStatus(snapshotStatus);
    return;
  }
  printPersistentDirtyFields(settings);
}

void printPersistentDirtySummaryIfDirty() {
  EE871::SettingsSnapshot settings;
  const EE871::Status snapshotStatus = device.getSettings(settings);
  if (!snapshotStatus.ok()) {
    printStatus(snapshotStatus);
    return;
  }
  if (settings.persistentConfigDirty) {
    printPersistentDirtyFields(settings);
  }
}

bool ensureProbeOk() {
  auto st = device.probe();
  if (!st.ok()) {
    logWarn("Probe failed");
    printStatus(st);
    return false;
  }
  return true;
}

void printDriverHealth() {
  const uint32_t now = nowMs();
  EE871::SettingsSnapshot settings;
  const EE871::Status snapshotStatus = device.getSettings(settings);
  if (!snapshotStatus.ok()) {
    printStatus(snapshotStatus);
    return;
  }
  const uint32_t totalOk = settings.totalSuccess;
  const uint32_t totalFail = settings.totalFailures;
  const uint32_t total = totalOk + totalFail;
  const float successRate =
      (total > 0U) ? (100.0f * static_cast<float>(totalOk) / static_cast<float>(total)) : 0.0f;
  const EE871::Status lastErr = settings.lastError;
  const EE871::DriverState st = settings.state;
  const bool online = device.isOnline();

  std::printf("=== Driver Health ===\n");
  std::printf("  State: %s%s%s\n",
              driverStateColor(st, online, settings.consecutiveFailures),
              stateToStr(st),
              LOG_COLOR_RESET);
  std::printf("  Online: %s%s%s\n",
              online ? LOG_COLOR_GREEN : LOG_COLOR_RED,
              boolStr(online),
              LOG_COLOR_RESET);
  std::printf("  Consecutive failures: %s%u%s\n",
              goodIfZeroColor(settings.consecutiveFailures),
              static_cast<unsigned>(settings.consecutiveFailures),
              LOG_COLOR_RESET);
  std::printf("  Total success: %s%lu%s\n",
              goodIfNonZeroColor(totalOk),
              static_cast<unsigned long>(totalOk),
              LOG_COLOR_RESET);
  std::printf("  Total failures: %s%lu%s\n",
              goodIfZeroColor(totalFail),
              static_cast<unsigned long>(totalFail),
              LOG_COLOR_RESET);
  std::printf("  Success rate: %s%.1f%%%s\n",
              successRateColor(successRate),
              static_cast<double>(successRate),
              LOG_COLOR_RESET);

  if (settings.lastOkMs > 0U) {
    std::printf("  Last OK: %lu ms ago (at %lu ms)\n",
                static_cast<unsigned long>(now - settings.lastOkMs),
                static_cast<unsigned long>(settings.lastOkMs));
  } else {
    std::printf("  Last OK: never\n");
  }

  if (settings.lastErrorMs > 0U) {
    std::printf("  Last error: %lu ms ago (at %lu ms)\n",
                static_cast<unsigned long>(now - settings.lastErrorMs),
                static_cast<unsigned long>(settings.lastErrorMs));
  } else {
    std::printf("  Last error: never\n");
  }

  if (!lastErr.ok()) {
    std::printf("  Error code: %s%s%s\n", LOG_COLOR_RED, errToStr(lastErr.code), LOG_COLOR_RESET);
    std::printf("  Error detail: %ld\n", static_cast<long>(lastErr.detail));
    if (lastErr.msg != nullptr && lastErr.msg[0] != '\0') {
      std::printf("  Error msg: %s\n", lastErr.msg);
    }
  }
  printPersistentDirtyFields(settings);
}

void printHelpSection(const char* title) {
  std::printf("\n%s[%s]%s\n", LOG_COLOR_GREEN, title, LOG_COLOR_RESET);
}

void printHelpItem(const char* command, const char* description) {
  std::printf("  %s%-*s%s - %s\n",
              LOG_COLOR_CYAN,
              static_cast<int>(HELP_COMMAND_WIDTH),
              command,
              LOG_COLOR_RESET,
              description);
}

void printHelp() {
  std::printf("\n");
  std::printf("%s=== EE871-E2 CLI Help ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);

  printHelpSection("Common");
  printHelpItem("help / ?", "Show this help");
  printHelpItem("version / ver", "Print firmware and library version info");
  printHelpItem("scan", "Scan all 8 E2 addresses");
  printHelpItem("probe", "Probe device (no health tracking)");
  printHelpItem("recover", "Attempt recovery");
  printHelpItem("drv", "Show driver state and health");
  printHelpItem("dirty", "Show persistent dirty-state diagnostics");
  printHelpItem("resync", "Verify persistent config; clear dirty on success");
  printHelpItem("read", "Read CO2 average");
  printHelpItem("cfg / settings", "Show driver state and feature flags");
  printHelpItem("verbose [0|1]", "Toggle bus trace output (no args = show)");
  printHelpItem("stress [N]", "Repeated CO2 average reads");
  printHelpItem("stress_mix [N]", "Mixed safe read operations");
  printHelpItem("selftest", "Safe command self-test with report");

  printHelpSection("Device Commands");
  printHelpItem("id", "Read group/subgroup/available bits");
  printHelpItem("status", "Read status byte (starts measurement)");
  printHelpItem("co2fast", "Read MV3 (fast response)");
  printHelpItem("co2avg", "Read MV4 (averaged)");
  printHelpItem("error", "Read error code (if status indicates error)");
  printHelpItem("reg read <addr>", "Read custom register (0x00..0xFF)");
  printHelpItem("reg write <addr> <value>", "Write persistent custom register (bench only)");
  printHelpItem("reg dump [start] [len]", "Dump custom registers (default all)");
  printHelpItem("ctrl <main_nibble>", "Raw readControlByte(main_nibble)");
  printHelpItem("u16 <main_lo> <main_hi>", "Raw readU16(main_lo, main_hi)");
  printHelpItem("ptr <addr16>", "Set custom pointer");

  printHelpSection("Device Info");
  printHelpItem("fw", "Read firmware version");
  printHelpItem("e2spec", "Read E2 spec version");
  printHelpItem("features", "Read feature support flags");
  printHelpItem("serial", "Read serial number");
  printHelpItem("partname", "Read part name");
  printHelpItem("partname <text>", "Write persistent part name (16 bytes max)");

  printHelpSection("Configuration");
  printHelpItem("addr", "Read current bus address");
  printHelpItem("addr <0-7>", "Write persistent bus address (power cycle)");
  printHelpItem("interval", "Read measurement interval");
  printHelpItem("interval <dec>", "Write persistent interval (150..36000 ds)");
  printHelpItem("factor", "Read CO2 interval factor");
  printHelpItem("factor <val>", "Write persistent CO2 interval factor");
  printHelpItem("filter", "Read CO2 filter setting");
  printHelpItem("filter <val>", "Write persistent CO2 filter");
  printHelpItem("mode", "Read operating mode");
  printHelpItem("mode <val>", "Write persistent operating mode (0..3)");

  printHelpSection("Calibration");
  printHelpItem("offset", "Read CO2 offset (ppm)");
  printHelpItem("offset <val>", "Write persistent CO2 offset (signed)");
  printHelpItem("gain", "Read CO2 gain");
  printHelpItem("gain <val>", "Write persistent CO2 gain");
  printHelpItem("calpoints", "Read last calibration points");
  printHelpItem("autoadj", "Read auto-adjust status");
  printHelpItem("autoadj start", "Start config-changing auto-adjustment (~5 min)");

  printHelpSection("Bus Safety");
  printHelpItem("buscheck", "Check if bus is idle");
  printHelpItem("libreset", "Bus reset via library");

  printHelpSection("Diagnostics");
  printHelpItem("diag", "Run full diagnostic suite");
  printHelpItem("levels", "Read current bus levels");
  printHelpItem("pintest", "Test pin toggle (MCU bus control)");
  printHelpItem("clocktest", "Generate clock pulses and verify");
  printHelpItem("sniff", "Toggle sniffer on/off");
  printHelpItem("timing", "Try different clock frequencies");
  printHelpItem("busreset", "Send 9 clocks to recover stuck bus");
  printHelpItem("tx <hex>", "Test transaction with control byte");
  printHelpItem("libtest", "Test all library commands (begin uses)");
  printHelpItem("caps", "Print feature capability booleans");
  printHelpItem("trace stats", "Show bus trace buffer stats");
  printHelpItem("trace clear", "Clear buffered trace events");
}

void printVersionInfo() {
  std::printf("=== Version Info ===\n");
  std::printf("  Example firmware build: %s %s\n", __DATE__, __TIME__);
  std::printf("  EE871 library version: %s\n", EE871::VERSION);
  std::printf("  EE871 library full: %s\n", EE871::VERSION_FULL);
  std::printf("  EE871 library build: %s\n", EE871::BUILD_TIMESTAMP);
  std::printf("  EE871 library commit: %s (%s)\n", EE871::GIT_COMMIT, EE871::GIT_STATUS);
  std::printf("  Sensor firmware: use command 'fw'\n");
}

// ============================================================================
// Diagnostics
// ============================================================================

namespace diag {

const char* okColor(bool ok) {
  return ok ? LOG_COLOR_GREEN : LOG_COLOR_RED;
}

const char* warnColor() {
  return LOG_COLOR_YELLOW;
}

const char* neutralColor() {
  return LOG_COLOR_GRAY;
}

void printStatusByte(uint8_t status) {
  std::printf("Status: %s0x%02X%s", LOG_COLOR_CYAN, static_cast<unsigned>(status), LOG_COLOR_RESET);
  if ((status & EE871::cmd::STATUS_CO2_ERROR_MASK) != 0U) {
    std::printf(" (%sCO2 error%s)", LOG_COLOR_RED, LOG_COLOR_RESET);
  }
  std::printf("\n");
}

struct BusLevels {
  bool scl;
  bool sda;
};

BusLevels readBusLevels(const EE871::Config& cfg) {
  BusLevels lvl;
  lvl.scl = cfg.readScl(cfg.busUser);
  lvl.sda = cfg.readSda(cfg.busUser);
  return lvl;
}

void printBusLevels(const EE871::Config& cfg) {
  const auto lvl = readBusLevels(cfg);
  std::printf("%s=== Bus Levels ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  std::printf("  SCL: %s%s%s\n",
              okColor(lvl.scl),
              lvl.scl ? "HIGH (idle)" : "LOW (held)",
              LOG_COLOR_RESET);
  std::printf("  SDA: %s%s%s\n",
              okColor(lvl.sda),
              lvl.sda ? "HIGH (idle)" : "LOW (held)",
              LOG_COLOR_RESET);

  if (!lvl.scl && !lvl.sda) {
    std::printf("  %sWARNING%s: Both lines LOW - bus stuck or no pull-ups!\n",
                warnColor(), LOG_COLOR_RESET);
  } else if (!lvl.scl) {
    std::printf("  %sWARNING%s: SCL held LOW - clock stretching or stuck!\n",
                warnColor(), LOG_COLOR_RESET);
  } else if (!lvl.sda) {
    std::printf("  %sWARNING%s: SDA held LOW - slave holding or stuck!\n",
                warnColor(), LOG_COLOR_RESET);
  } else {
    std::printf("  %sOK%s: Bus idle (both HIGH)\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  }
}

void testPinToggle(const EE871::Config& cfg) {
  std::printf("%s=== Pin Toggle Test ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  std::printf("Testing if MCU can control and read bus lines...\n\n");

  std::printf("SCL pin test:\n");
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  const bool sclHigh = cfg.readScl(cfg.busUser);
  std::printf("  Set HIGH -> Read: %s%s%s\n",
              okColor(sclHigh),
              sclHigh ? "HIGH (OK)" : "LOW (FAIL - stuck or no pull-up)",
              LOG_COLOR_RESET);

  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  const bool sclLow = cfg.readScl(cfg.busUser);
  std::printf("  Set LOW  -> Read: %s%s%s\n",
              okColor(!sclLow),
              sclLow ? "HIGH (FAIL - can't pull low)" : "LOW (OK)",
              LOG_COLOR_RESET);

  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);

  std::printf("\nSDA pin test:\n");
  cfg.setSda(true, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  const bool sdaHigh = cfg.readSda(cfg.busUser);
  std::printf("  Set HIGH -> Read: %s%s%s\n",
              okColor(sdaHigh),
              sdaHigh ? "HIGH (OK)" : "LOW (FAIL - stuck or no pull-up)",
              LOG_COLOR_RESET);

  cfg.setSda(false, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  const bool sdaLow = cfg.readSda(cfg.busUser);
  std::printf("  Set LOW  -> Read: %s%s%s\n",
              okColor(!sdaLow),
              sdaLow ? "HIGH (FAIL - can't pull low)" : "LOW (OK)",
              LOG_COLOR_RESET);

  cfg.setSda(true, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);

  std::printf("\nSummary:\n");
  const bool sclOk = sclHigh && !sclLow;
  const bool sdaOk = sdaHigh && !sdaLow;
  if (sclOk && sdaOk) {
    std::printf("  %sPASS%s: Both pins working correctly\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  } else {
    if (!sclHigh) {
      std::printf("  %sFAIL%s: SCL has no pull-up or is stuck LOW\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    if (sclLow) {
      std::printf("  %sFAIL%s: SCL cannot be pulled LOW by MCU\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    if (!sdaHigh) {
      std::printf("  %sFAIL%s: SDA has no pull-up or is stuck LOW\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    if (sdaLow) {
      std::printf("  %sFAIL%s: SDA cannot be pulled LOW by MCU\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
  }
}

struct SniffResult {
  uint32_t sclTransitions;
  uint32_t sdaTransitions;
  uint32_t durationMs;
  bool sclStuckLow;
  bool sdaStuckLow;
};

SniffResult sniffBus(const EE871::Config& cfg, uint32_t durationMs) {
  SniffResult result = {0, 0, durationMs, false, false};
  const uint32_t startMs = nowMs();
  bool lastScl = cfg.readScl(cfg.busUser);
  bool lastSda = cfg.readSda(cfg.busUser);
  uint32_t sclLowStart = lastScl ? 0U : startMs;
  uint32_t sdaLowStart = lastSda ? 0U : startMs;

  while ((nowMs() - startMs) < durationMs) {
    const bool scl = cfg.readScl(cfg.busUser);
    const bool sda = cfg.readSda(cfg.busUser);
    if (scl != lastScl) {
      result.sclTransitions++;
      sclLowStart = scl ? 0U : nowMs();
      lastScl = scl;
    }
    if (sda != lastSda) {
      result.sdaTransitions++;
      sdaLowStart = sda ? 0U : nowMs();
      lastSda = sda;
    }
    const uint32_t now = nowMs();
    if (sclLowStart != 0U && (now - sclLowStart > 100U)) result.sclStuckLow = true;
    if (sdaLowStart != 0U && (now - sdaLowStart > 100U)) result.sdaStuckLow = true;
    cfg.delayUs(10, cfg.busUser);
  }
  return result;
}

void sniffAndPrint(const EE871::Config& cfg, uint32_t durationMs = 2000) {
  std::printf("%s=== Bus Sniffer (%lu ms) ===%s\n",
              LOG_COLOR_CYAN,
              static_cast<unsigned long>(durationMs),
              LOG_COLOR_RESET);
  std::printf("Monitoring bus activity...\n");
  const auto result = sniffBus(cfg, durationMs);
  std::printf("\nResults over %lu ms:\n", static_cast<unsigned long>(result.durationMs));
  std::printf("  SCL transitions: %lu\n", static_cast<unsigned long>(result.sclTransitions));
  std::printf("  SDA transitions: %lu\n", static_cast<unsigned long>(result.sdaTransitions));
  if (result.sclTransitions > 0U) {
    const float sclFreq =
        (static_cast<float>(result.sclTransitions) / 2.0f) /
        (static_cast<float>(result.durationMs) / 1000.0f);
    std::printf("  SCL approx freq: %.1f Hz\n", static_cast<double>(sclFreq));
  }
  if (result.sclStuckLow) {
    std::printf("  %sWARNING%s: SCL was stuck LOW for >100ms\n", warnColor(), LOG_COLOR_RESET);
  }
  if (result.sdaStuckLow) {
    std::printf("  %sWARNING%s: SDA was stuck LOW for >100ms\n", warnColor(), LOG_COLOR_RESET);
  }
  if (result.sclTransitions == 0U && result.sdaTransitions == 0U) {
    std::printf("\n  No bus activity detected - bus is quiet\n");
  }
}

void testClockPulses(const EE871::Config& cfg, int numPulses = 10) {
  std::printf("%s=== Clock Pulse Test (%d pulses) ===%s\n",
              LOG_COLOR_CYAN,
              numPulses,
              LOG_COLOR_RESET);
  int successHigh = 0;
  int successLow = 0;
  for (int i = 0; i < numPulses; ++i) {
    cfg.setScl(false, cfg.busUser);
    cfg.delayUs(cfg.clockLowUs, cfg.busUser);
    const bool readLow = cfg.readScl(cfg.busUser);
    if (!readLow) successLow++;

    cfg.setScl(true, cfg.busUser);
    cfg.delayUs(cfg.clockHighUs, cfg.busUser);
    const bool readHigh = cfg.readScl(cfg.busUser);
    if (readHigh) successHigh++;

    std::printf("  Pulse %2d: LOW=%s%s%s HIGH=%s%s%s\n",
                i + 1,
                okColor(!readLow),
                readLow ? "FAIL" : "ok",
                LOG_COLOR_RESET,
                okColor(readHigh),
                readHigh ? "ok" : "FAIL(stretched?)",
                LOG_COLOR_RESET);
  }
  std::printf("\nResults: %d/%d LOW ok, %d/%d HIGH ok\n",
              successLow,
              numPulses,
              successHigh,
              numPulses);
  if (successHigh < numPulses) {
    std::printf("  %sNOTE%s: HIGH failures may indicate clock stretching by slave\n",
                LOG_COLOR_YELLOW,
                LOG_COLOR_RESET);
  }
}

void sendStart(const EE871::Config& cfg) {
  cfg.setSda(true, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
  cfg.setSda(false, cfg.busUser);
  cfg.delayUs(10, cfg.busUser);
  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(cfg.clockLowUs, cfg.busUser);
}

void sendStop(const EE871::Config& cfg) {
  cfg.setSda(false, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
  cfg.setSda(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
}

bool sendByteRaw(const EE871::Config& cfg, uint8_t data, bool verbose = false) {
  for (int i = 7; i >= 0; --i) {
    const bool bit = ((data >> i) & 1U) != 0U;
    cfg.setSda(bit, cfg.busUser);
    cfg.delayUs(10, cfg.busUser);
    cfg.setScl(true, cfg.busUser);
    cfg.delayUs(cfg.clockHighUs, cfg.busUser);
    cfg.setScl(false, cfg.busUser);
    cfg.delayUs(cfg.clockLowUs, cfg.busUser);
  }

  cfg.setSda(true, cfg.busUser);
  cfg.delayUs(10, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs / 2U, cfg.busUser);
  const bool ack = !cfg.readSda(cfg.busUser);
  cfg.delayUs(cfg.clockHighUs / 2U, cfg.busUser);
  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(cfg.clockLowUs, cfg.busUser);

  if (verbose) {
    std::printf("  Sent 0x%02X -> %s%s%s\n",
                static_cast<unsigned>(data),
                okColor(ack),
                ack ? "ACK" : "NACK",
                LOG_COLOR_RESET);
  }
  return ack;
}

uint8_t readByteRaw(const EE871::Config& cfg, bool sendAck, bool verbose = false) {
  uint8_t data = 0;
  cfg.setSda(true, cfg.busUser);
  for (int i = 7; i >= 0; --i) {
    cfg.setScl(true, cfg.busUser);
    cfg.delayUs(cfg.clockHighUs / 2U, cfg.busUser);
    const bool bit = cfg.readSda(cfg.busUser);
    if (bit) data = static_cast<uint8_t>(data | (1U << i));
    cfg.delayUs(cfg.clockHighUs / 2U, cfg.busUser);
    cfg.setScl(false, cfg.busUser);
    cfg.delayUs(cfg.clockLowUs, cfg.busUser);
  }

  cfg.setSda(!sendAck, cfg.busUser);
  cfg.delayUs(10, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(cfg.clockLowUs, cfg.busUser);
  cfg.setSda(true, cfg.busUser);

  if (verbose) {
    std::printf("  Read 0x%02X, sent %s%s%s\n",
                static_cast<unsigned>(data),
                sendAck ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW,
                sendAck ? "ACK" : "NACK",
                LOG_COLOR_RESET);
  }
  return data;
}

void scanAddresses(const EE871::Config& cfg) {
  std::printf("%s=== E2 Address Scanner ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  std::printf("Scanning addresses 0-7 with status read (0x7x)...\n\n");
  int found = 0;
  for (uint8_t addr = 0; addr < 8U; ++addr) {
    const uint8_t ctrlByte = EE871::cmd::makeControlRead(EE871::cmd::MAIN_STATUS, addr);
    sendStart(cfg);
    const bool ack = sendByteRaw(cfg, ctrlByte, false);
    if (ack) {
      const uint8_t data = readByteRaw(cfg, true, false);
      const uint8_t pec = readByteRaw(cfg, false, false);
      sendStop(cfg);
      const uint8_t expectedPec = static_cast<uint8_t>((ctrlByte + data) & 0xFFU);
      const bool pecOk = (pec == expectedPec);
      std::printf("  Address %u: %sFOUND%s Status=0x%02X, PEC=%s%s%s\n",
                  static_cast<unsigned>(addr),
                  LOG_COLOR_GREEN,
                  LOG_COLOR_RESET,
                  static_cast<unsigned>(data),
                  okColor(pecOk),
                  pecOk ? "OK" : "MISMATCH",
                  LOG_COLOR_RESET);
      found++;
    } else {
      sendStop(cfg);
      std::printf("  Address %u: %sNo response (NACK)%s\n",
                  static_cast<unsigned>(addr),
                  neutralColor(),
                  LOG_COLOR_RESET);
    }
    delayMs(10);
  }
  std::printf("\nFound %s%d%s device(s)\n", okColor(found > 0), found, LOG_COLOR_RESET);
}

struct TimingResult {
  uint16_t clockUs;
  bool gotAck;
  uint8_t data;
  bool pecOk;
};

TimingResult tryTiming(const EE871::Config& cfg, uint16_t clockUs) {
  TimingResult result = {clockUs, false, 0, false};
  EE871::Config testCfg = cfg;
  testCfg.clockLowUs = clockUs;
  testCfg.clockHighUs = clockUs;
  const uint8_t ctrlByte = EE871::cmd::makeControlRead(EE871::cmd::MAIN_STATUS, 0);
  sendStart(testCfg);
  result.gotAck = sendByteRaw(testCfg, ctrlByte, false);
  if (result.gotAck) {
    result.data = readByteRaw(testCfg, true, false);
    const uint8_t pec = readByteRaw(testCfg, false, false);
    const uint8_t expectedPec = static_cast<uint8_t>((ctrlByte + result.data) & 0xFFU);
    result.pecOk = (pec == expectedPec);
  }
  sendStop(testCfg);
  return result;
}

void discoverTiming(const EE871::Config& cfg) {
  std::printf("%s=== Timing Discovery ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  std::printf("Testing different clock periods...\n");
  std::printf("E2 spec: 100-1000us (500-5000 Hz)\n\n");
  const uint16_t timings[] = {1000, 500, 250, 200, 150, 100, 75, 50};
  int foundCount = 0;
  for (uint16_t clockUs : timings) {
    const float freqHz = 1000000.0f / (2.0f * static_cast<float>(clockUs));
    const auto result = tryTiming(cfg, clockUs);
    std::printf("  %4u us (%5.0f Hz): ",
                static_cast<unsigned>(clockUs),
                static_cast<double>(freqHz));
    if (result.gotAck) {
      std::printf("%sACK%s, data=0x%02X, PEC=%s%s%s\n",
                  LOG_COLOR_GREEN,
                  LOG_COLOR_RESET,
                  static_cast<unsigned>(result.data),
                  okColor(result.pecOk),
                  result.pecOk ? "OK" : "BAD",
                  LOG_COLOR_RESET);
      foundCount++;
    } else {
      std::printf("%sNACK%s\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    delayMs(50);
  }
  std::printf("\n%s%d%s timing(s) worked\n", okColor(foundCount > 0), foundCount, LOG_COLOR_RESET);
  if (foundCount == 0) {
    std::printf("\n%sNo timing worked%s. Possible issues:\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    std::printf("  - Device not connected or powered\n");
    std::printf("  - Wrong pins configured\n");
    std::printf("  - Missing/wrong pull-ups\n");
    std::printf("  - Bus voltage mismatch (need level shifter?)\n");
    std::printf("  - Device address not 0\n");
  }
}

void sendRecoveryClocks(const EE871::Config& cfg) {
  std::printf("%s=== Bus Recovery (9 clocks) ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  cfg.setSda(true, cfg.busUser);
  for (int i = 0; i < 9; ++i) {
    cfg.setScl(false, cfg.busUser);
    cfg.delayUs(cfg.clockLowUs, cfg.busUser);
    cfg.setScl(true, cfg.busUser);
    cfg.delayUs(cfg.clockHighUs, cfg.busUser);
    if (cfg.readSda(cfg.busUser)) {
      std::printf("  SDA released after %d clock(s)\n", i + 1);
    }
  }
  sendStop(cfg);
  const auto lvl = readBusLevels(cfg);
  std::printf("  Final: SCL=%s%s%s SDA=%s%s%s\n",
              okColor(lvl.scl),
              lvl.scl ? "HIGH" : "LOW",
              LOG_COLOR_RESET,
              okColor(lvl.sda),
              lvl.sda ? "HIGH" : "LOW",
              LOG_COLOR_RESET);
}

void testTransaction(const EE871::Config& cfg, uint8_t ctrlByte) {
  std::printf("%s=== Transaction Test ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  std::printf("Control byte: 0x%02X\n", static_cast<unsigned>(ctrlByte));
  std::printf("  MainCmd: 0x%X, Addr: %u, R/W: %s\n",
              static_cast<unsigned>((ctrlByte >> 4U) & 0x0FU),
              static_cast<unsigned>((ctrlByte >> 1U) & 0x07U),
              (ctrlByte & 1U) ? "READ" : "WRITE");
  std::printf("\n");
  const auto lvlBefore = readBusLevels(cfg);
  std::printf("Bus before: SCL=%s SDA=%s\n", lvlBefore.scl ? "H" : "L", lvlBefore.sda ? "H" : "L");
  if (!lvlBefore.scl || !lvlBefore.sda) {
    std::printf("%sERROR%s: Bus not idle, aborting\n\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    return;
  }
  std::printf("\n1. Sending START...\n");
  sendStart(cfg);
  const auto lvlAfterStart = readBusLevels(cfg);
  std::printf("   After START: SCL=%s SDA=%s\n",
              lvlAfterStart.scl ? "H" : "L",
              lvlAfterStart.sda ? "H" : "L");
  std::printf("\n2. Sending control byte 0x%02X...\n", static_cast<unsigned>(ctrlByte));
  const bool ack = sendByteRaw(cfg, ctrlByte, true);
  if (!ack) {
    std::printf("   %sNACK%s received - device not responding\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    sendStop(cfg);
    std::printf("\n3. Sent STOP\n");
    return;
  }
  if ((ctrlByte & 0x01U) != 0U) {
    std::printf("\n3. Reading data byte...\n");
    const uint8_t data = readByteRaw(cfg, true, true);
    std::printf("\n4. Reading PEC...\n");
    const uint8_t pec = readByteRaw(cfg, false, true);
    const uint8_t expectedPec = static_cast<uint8_t>((ctrlByte + data) & 0xFFU);
    const bool pecOk = (pec == expectedPec);
    std::printf("\n5. PEC check: received=0x%02X, expected=0x%02X -> %s%s%s\n",
                static_cast<unsigned>(pec),
                static_cast<unsigned>(expectedPec),
                okColor(pecOk),
                pecOk ? "OK" : "MISMATCH",
                LOG_COLOR_RESET);
  }
  sendStop(cfg);
  std::printf("\n6. Sent STOP\n");
  const auto lvlAfter = readBusLevels(cfg);
  std::printf("Bus after: SCL=%s SDA=%s\n", lvlAfter.scl ? "H" : "L", lvlAfter.sda ? "H" : "L");
}

void testLibraryCommands(const EE871::Config& cfg) {
  std::printf("%s=== Library Command Test ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  std::printf("Testing exact commands used by begin()...\n\n");
  struct CmdTest {
    uint8_t mainCmd;
    const char* name;
  };
  const CmdTest tests[] = {
      {EE871::cmd::MAIN_TYPE_LO, "TYPE_LO (0x11)"},
      {EE871::cmd::MAIN_TYPE_HI, "TYPE_HI (0x41)"},
      {EE871::cmd::MAIN_TYPE_SUB, "TYPE_SUB (0x21)"},
      {EE871::cmd::MAIN_AVAIL_MEAS, "AVAIL (0x31)"},
      {EE871::cmd::MAIN_STATUS, "STATUS (0x71)"},
      {EE871::cmd::MAIN_MV3_LO, "MV3_LO (0xC1)"},
      {EE871::cmd::MAIN_MV3_HI, "MV3_HI (0xD1)"},
      {EE871::cmd::MAIN_MV4_LO, "MV4_LO (0xE1)"},
      {EE871::cmd::MAIN_MV4_HI, "MV4_HI (0xF1)"},
  };
  int passed = 0;
  const int numTests = static_cast<int>(sizeof(tests) / sizeof(tests[0]));
  for (int i = 0; i < numTests; ++i) {
    const uint8_t ctrlByte = EE871::cmd::makeControlRead(tests[i].mainCmd, cfg.deviceAddress);
    std::printf("%-18s [0x%02X]: ", tests[i].name, static_cast<unsigned>(ctrlByte));
    sendStart(cfg);
    const bool ack = sendByteRaw(cfg, ctrlByte, false);
    if (ack) {
      const uint8_t data = readByteRaw(cfg, true, false);
      const uint8_t pec = readByteRaw(cfg, false, false);
      sendStop(cfg);
      const uint8_t expectedPec = static_cast<uint8_t>((ctrlByte + data) & 0xFFU);
      const bool pecOk = (pec == expectedPec);
      std::printf("%sACK%s data=0x%02X PEC=%s%s%s\n",
                  LOG_COLOR_GREEN,
                  LOG_COLOR_RESET,
                  static_cast<unsigned>(data),
                  okColor(pecOk),
                  pecOk ? "OK" : "BAD",
                  LOG_COLOR_RESET);
      if (pecOk) passed++;
    } else {
      sendStop(cfg);
      std::printf("%sNACK%s\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    delayMs(20);
  }
  std::printf("\nPassed: %s%d%s/%d\n", okColor(passed == numTests), passed, LOG_COLOR_RESET, numTests);
  if (passed == 0) {
    std::printf("\n%sAll commands failed!%s Check:\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    std::printf("  - Is device address 0? (default)\n");
    std::printf("  - Datasheet command compatibility\n");
  } else if (passed < numTests) {
    std::printf("\n%sSome commands failed%s - may be normal depending on device state\n",
                LOG_COLOR_YELLOW,
                LOG_COLOR_RESET);
  }
}

void runFullDiagnostics(const EE871::Config& cfg) {
  std::printf("\n\n");
  std::printf("%s=== FULL E2 BUS DIAGNOSTICS ===%s\n\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  auto* bus = static_cast<ee871_idf::E2GpioBus*>(cfg.busUser);
  const int sda = (bus != nullptr) ? static_cast<int>(bus->sda) : -1;
  const int scl = (bus != nullptr) ? static_cast<int>(bus->scl) : -1;
  std::printf("Config: DATA=GPIO%d, CLOCK=GPIO%d\n", sda, scl);
  std::printf("Timing: LOW=%u us, HIGH=%u us (%.0f Hz)\n\n",
              static_cast<unsigned>(cfg.clockLowUs),
              static_cast<unsigned>(cfg.clockHighUs),
              static_cast<double>(1000000.0f / (cfg.clockLowUs + cfg.clockHighUs)));
  std::printf("%s[Step 1] Bus Levels%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printBusLevels(cfg);
  std::printf("\n");
  std::printf("%s[Step 2] Pin Toggle Test%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  testPinToggle(cfg);
  std::printf("\n");
  std::printf("%s[Step 3] Clock Pulse Test%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  testClockPulses(cfg, 5);
  std::printf("\n");
  std::printf("%s[Step 4] Bus Sniff (1s)%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  sniffAndPrint(cfg, 1000);
  std::printf("\n");
  std::printf("%s[Step 5] Address Scan%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  scanAddresses(cfg);
  std::printf("\n");
  std::printf("%s[Step 6] Timing Discovery%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  discoverTiming(cfg);
  std::printf("\n");
  std::printf("%s=== DIAGNOSTICS COMPLETE ===%s\n\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
}

struct SnifferState {
  bool active = false;
  bool lastScl = true;
  bool lastSda = true;

  enum class State : uint8_t { IDLE, RECEIVING_BYTE, WAITING_ACK } state = State::IDLE;
  uint8_t currentByte = 0;
  uint8_t bitCount = 0;
  bool isFirstByte = true;
  bool isReadMode = false;
  uint8_t byteIndex = 0;
  uint8_t lastMainCmd = 0;
  uint8_t pendingLowByte = 0;
  bool haveLowByte = false;
  uint8_t lowByteCmd = 0;
  uint32_t transitions = 0;
  uint32_t startMs = 0;
};

SnifferState& snifferState() {
  static SnifferState state;
  return state;
}

const char* getCmdName(uint8_t mainCmd, bool read) {
  switch (mainCmd) {
    case 0x1: return read ? "TYPE_LO" : "CUST_WR";
    case 0x2: return "SUBGRP";
    case 0x3: return "AVAIL";
    case 0x4: return "TYPE_HI";
    case 0x5: return read ? "CUST_RD" : "CUST_PTR";
    case 0x7: return "STATUS";
    case 0xC: return "CO2fast_L";
    case 0xD: return "CO2fast_H";
    case 0xE: return "CO2avg_L";
    case 0xF: return "CO2avg_H";
    default: return "???";
  }
}

bool isLowByteCmd(uint8_t mainCmd) {
  return mainCmd == 0xC || mainCmd == 0xE || mainCmd == 0x1;
}

bool isMatchingHighByte(uint8_t lowCmd, uint8_t highCmd) {
  if (lowCmd == 0xC && highCmd == 0xD) return true;
  if (lowCmd == 0xE && highCmd == 0xF) return true;
  if (lowCmd == 0x1 && highCmd == 0x4) return true;
  return false;
}

void onLineSample(bool scl, bool sda) {
  auto& s = snifferState();
  if (!s.active) {
    s.lastScl = scl;
    s.lastSda = sda;
    return;
  }

  if (s.lastScl && scl && s.lastSda && !sda) {
    std::printf("\n>START ");
    s.state = SnifferState::State::RECEIVING_BYTE;
    s.currentByte = 0;
    s.bitCount = 0;
    s.isFirstByte = true;
    s.byteIndex = 0;
    s.transitions++;
  } else if (s.lastScl && scl && !s.lastSda && sda) {
    std::printf(" STOP\n");
    s.state = SnifferState::State::IDLE;
    s.transitions++;
  } else if (!s.lastScl && scl) {
    s.transitions++;
    if (s.state == SnifferState::State::RECEIVING_BYTE) {
      s.currentByte = static_cast<uint8_t>((s.currentByte << 1U) | (sda ? 1U : 0U));
      s.bitCount++;
      if (s.bitCount == 8U) {
        s.state = SnifferState::State::WAITING_ACK;
      }
    } else if (s.state == SnifferState::State::WAITING_ACK) {
      const bool ack = !sda;
      if (s.isFirstByte) {
        s.isReadMode = (s.currentByte & 0x01U) != 0U;
        s.lastMainCmd = static_cast<uint8_t>((s.currentByte >> 4U) & 0x0FU);
        const uint8_t addr = static_cast<uint8_t>((s.currentByte >> 1U) & 0x07U);
        std::printf("[0x%02X %s a%u %s]",
                    static_cast<unsigned>(s.currentByte),
                    getCmdName(s.lastMainCmd, s.isReadMode),
                    static_cast<unsigned>(addr),
                    ack ? "ACK" : "NAK");
        s.isFirstByte = false;
      } else {
        if (s.byteIndex == 1U) {
          std::printf(" data=0x%02X(%u)",
                      static_cast<unsigned>(s.currentByte),
                      static_cast<unsigned>(s.currentByte));
          if (s.isReadMode && isLowByteCmd(s.lastMainCmd)) {
            s.pendingLowByte = s.currentByte;
            s.haveLowByte = true;
            s.lowByteCmd = s.lastMainCmd;
          } else if (s.isReadMode && s.haveLowByte &&
                     isMatchingHighByte(s.lowByteCmd, s.lastMainCmd)) {
            const uint16_t value =
                static_cast<uint16_t>((static_cast<uint16_t>(s.currentByte) << 8U) |
                                      s.pendingLowByte);
            std::printf(" => %u", static_cast<unsigned>(value));
            if (s.lowByteCmd == 0xC || s.lowByteCmd == 0xE) {
              std::printf(" ppm");
            }
            s.haveLowByte = false;
          }
        } else if (s.byteIndex == 2U) {
          std::printf(" pec=0x%02X", static_cast<unsigned>(s.currentByte));
        }
      }
      s.byteIndex++;
      s.currentByte = 0;
      s.bitCount = 0;
      s.state = SnifferState::State::RECEIVING_BYTE;
    }
  }

  s.lastScl = scl;
  s.lastSda = sda;
}

class BusSniffer {
public:
  void start(const EE871::Config* cfg) {
    auto& s = snifferState();
    s.lastScl = cfg->readScl(cfg->busUser);
    s.lastSda = cfg->readSda(cfg->busUser);
    s.transitions = 0;
    s.startMs = nowMs();
    s.state = SnifferState::State::IDLE;
    s.currentByte = 0;
    s.bitCount = 0;
    s.isFirstByte = true;
    s.haveLowByte = false;
    s.active = true;
    std::printf("[SNIFF] ON - 'sniff 0' to stop\n");
  }

  void stop() {
    auto& s = snifferState();
    if (s.active) {
      s.active = false;
      const uint32_t elapsed = nowMs() - s.startMs;
      std::printf("\n[SNIFF] OFF (%lu ms, %lu edges)\n",
                  static_cast<unsigned long>(elapsed),
                  static_cast<unsigned long>(s.transitions));
    }
  }

  bool isActive() const {
    return snifferState().active;
  }

  void tick(const EE871::Config& cfg) {
    if (!isActive()) {
      return;
    }
    onLineSample(ee871_idf::readScl(cfg.busUser), ee871_idf::readSda(cfg.busUser));
  }
};

BusSniffer& sniffer() {
  static BusSniffer instance;
  return instance;
}

}  // namespace diag

// ============================================================================
// Stress and self-test workflows
// ============================================================================

void runStress(int count) {
  const uint32_t successBefore = device.totalSuccess();
  const uint32_t failBefore = device.totalFailures();
  const uint32_t startMs = nowMs();
  int ok = 0;
  int fail = 0;
  bool hasFailure = false;
  EE871::Status firstFailure = EE871::Status::Ok();
  EE871::Status lastFailure = EE871::Status::Ok();

  for (int i = 0; i < count; ++i) {
    uint16_t ppm = 0;
    EE871::Status st = device.readCo2Average(ppm);
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
    printStressProgress(static_cast<uint32_t>(i + 1),
                        static_cast<uint32_t>(count),
                        static_cast<uint32_t>(ok),
                        static_cast<uint32_t>(fail));
    device.tick(nowMs());
  }

  const uint32_t elapsed = nowMs() - startMs;
  const float successPct =
      (count > 0) ? (100.0f * static_cast<float>(ok) / static_cast<float>(count)) : 0.0f;
  const uint32_t successDelta = device.totalSuccess() - successBefore;
  const uint32_t failDelta = device.totalFailures() - failBefore;

  std::printf("=== Stress Summary ===\n");
  std::printf("  Total: %d\n", count);
  std::printf("  Success: %s%d%s\n", goodIfNonZeroColor(static_cast<uint32_t>(ok)), ok, LOG_COLOR_RESET);
  std::printf("  Errors: %s%d%s\n", goodIfZeroColor(static_cast<uint32_t>(fail)), fail, LOG_COLOR_RESET);
  std::printf("  Success rate: %s%.2f%%%s\n",
              successRateColor(successPct),
              static_cast<double>(successPct),
              LOG_COLOR_RESET);
  std::printf("  Duration: %lu ms\n", static_cast<unsigned long>(elapsed));
  if (elapsed > 0U) {
    std::printf("  Rate: %.2f ops/s\n",
                static_cast<double>((1000.0f * static_cast<float>(count)) /
                                    static_cast<float>(elapsed)));
  }
  std::printf("  Health delta: %ssuccess +%lu%s, %sfailures +%lu%s\n",
              goodIfNonZeroColor(successDelta),
              static_cast<unsigned long>(successDelta),
              LOG_COLOR_RESET,
              goodIfZeroColor(failDelta),
              static_cast<unsigned long>(failDelta),
              LOG_COLOR_RESET);
  if (hasFailure) {
    std::printf("  Failure details:\n");
    std::printf("  First failure:\n");
    printStatus(firstFailure);
    if (fail > 1) {
      std::printf("  Last failure:\n");
      printStatus(lastFailure);
    }
  }
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
  const uint32_t startMs = nowMs();
  uint32_t okTotal = 0;
  uint32_t failTotal = 0;

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
        uint8_t main = 0;
        uint8_t sub = 0;
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
      okTotal++;
    } else {
      stats[op].fail++;
      failTotal++;
      if (verboseMode) {
        std::printf("  [%d] %s failed: %s\n", i, stats[op].name, errToStr(st.code));
      }
    }

    printStressProgress(static_cast<uint32_t>(i + 1),
                        static_cast<uint32_t>(count),
                        okTotal,
                        failTotal);
  }

  const uint32_t elapsed = nowMs() - startMs;
  std::printf("=== stress_mix summary ===\n");
  const float successPct =
      (count > 0) ? (100.0f * static_cast<float>(okTotal) / static_cast<float>(count)) : 0.0f;
  std::printf("  Total: %sok=%lu%s %sfail=%lu%s (%s%.2f%%%s)\n",
              goodIfNonZeroColor(okTotal),
              static_cast<unsigned long>(okTotal),
              LOG_COLOR_RESET,
              goodIfZeroColor(failTotal),
              static_cast<unsigned long>(failTotal),
              LOG_COLOR_RESET,
              successRateColor(successPct),
              static_cast<double>(successPct),
              LOG_COLOR_RESET);
  std::printf("  Duration: %lu ms\n", static_cast<unsigned long>(elapsed));
  if (elapsed > 0U) {
    std::printf("  Rate: %.2f ops/s\n",
                static_cast<double>((1000.0f * static_cast<float>(count)) /
                                    static_cast<float>(elapsed)));
  }
  for (int i = 0; i < opCount; ++i) {
    std::printf("  %-11s %sok=%lu%s %sfail=%lu%s\n",
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
  std::printf("  Health delta: %ssuccess +%lu%s, %sfailures +%lu%s\n",
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
    const char* color = skip ? LOG_COLOR_YELLOW : resultColor(ok);
    const char* tag = skip ? "SKIP" : (ok ? "PASS" : "FAIL");
    std::printf("  [%s%s%s] %s", color, tag, LOG_COLOR_RESET, name);
    if (note != nullptr && note[0] != '\0') {
      std::printf(" - %s", note);
    }
    std::printf("\n");
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

  std::printf("=== EE871 selftest (safe commands) ===\n");
  const uint32_t succBefore = device.totalSuccess();
  const uint32_t failBefore = device.totalFailures();
  const uint8_t consBefore = device.consecutiveFailures();

  EE871::Status st = device.probe();
  if (st.code == EE871::Err::NOT_INITIALIZED) {
    reportSkip("probe responds", "driver not initialized");
    reportSkip("remaining checks", "selftest aborted");
    std::printf("Selftest result: pass=%s%lu%s fail=%s%lu%s skip=%s%lu%s\n",
                goodIfNonZeroColor(result.pass),
                static_cast<unsigned long>(result.pass),
                LOG_COLOR_RESET,
                goodIfZeroColor(result.fail),
                static_cast<unsigned long>(result.fail),
                LOG_COLOR_RESET,
                skipCountColor(result.skip),
                static_cast<unsigned long>(result.skip),
                LOG_COLOR_RESET);
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
  reportCheck("CO2 measurement bit present", st.ok() && (avail & EE871::cmd::AVAILABLE_MEAS_MASK) != 0U, "");

  uint8_t fwMain = 0;
  uint8_t fwSub = 0;
  st = device.readFirmwareVersion(fwMain, fwSub);
  reportCheck("readFirmwareVersion", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint8_t e2spec = 0;
  st = device.readE2SpecVersion(e2spec);
  reportCheck("readE2SpecVersion", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint8_t ops = 0;
  uint8_t modes = 0;
  uint8_t special = 0;
  st = device.readOperatingFunctions(ops);
  reportCheck("readOperatingFunctions", st.ok(), st.ok() ? "" : errToStr(st.code));
  if (st.ok()) st = device.readOperatingModeSupport(modes);
  reportCheck("readOperatingModeSupport", st.ok(), st.ok() ? "" : errToStr(st.code));
  if (st.ok()) st = device.readSpecialFeatures(special);
  reportCheck("readSpecialFeatures", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint8_t status = 0;
  st = device.readStatus(status);
  reportCheck("readStatus", st.ok(), st.ok() ? "" : errToStr(st.code));

  uint16_t fast = 0;
  uint16_t avg = 0;
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

  std::printf("Selftest result: pass=%s%lu%s fail=%s%lu%s skip=%s%lu%s\n",
              goodIfNonZeroColor(result.pass),
              static_cast<unsigned long>(result.pass),
              LOG_COLOR_RESET,
              goodIfZeroColor(result.fail),
              static_cast<unsigned long>(result.fail),
              LOG_COLOR_RESET,
              skipCountColor(result.skip),
              static_cast<unsigned long>(result.skip),
              LOG_COLOR_RESET);
}

// ============================================================================
// Command processing
// ============================================================================

void printAsciiField(const char* label, const uint8_t* data, size_t len, bool stopAtNul) {
  std::printf("  %s: ", label);
  for (size_t i = 0; i < len; ++i) {
    if (stopAtNul && data[i] == 0U) {
      break;
    }
    if (data[i] >= 0x20U && data[i] < 0x7FU) {
      std::printf("%c", static_cast<char>(data[i]));
    } else {
      std::printf(".");
    }
  }
  std::printf("\n");
}

void processCommand(const char* input) {
  char line[MAX_LINE_LENGTH + 1U];
  std::strncpy(line, input, sizeof(line) - 1U);
  line[sizeof(line) - 1U] = '\0';
  char* trimmed = trimInPlace(line);
  if (trimmed[0] == '\0') {
    return;
  }

  if (std::strcmp(trimmed, "help") == 0 || std::strcmp(trimmed, "?") == 0) {
    printHelp();
  } else if (std::strcmp(trimmed, "version") == 0 || std::strcmp(trimmed, "ver") == 0) {
    printVersionInfo();
  } else if (std::strcmp(trimmed, "read") == 0) {
    uint16_t ppm = 0;
    auto st = device.readCo2Average(ppm);
    printStatus(st);
    if (st.ok()) {
      std::printf("  CO2 avg: %u ppm\n", static_cast<unsigned>(ppm));
    }
  } else if (std::strcmp(trimmed, "cfg") == 0 || std::strcmp(trimmed, "settings") == 0) {
    printDriverHealth();
    uint8_t ops = 0;
    uint8_t modes = 0;
    uint8_t special = 0;
    auto st = device.readOperatingFunctions(ops);
    if (st.ok()) st = device.readOperatingModeSupport(modes);
    if (st.ok()) st = device.readSpecialFeatures(special);
    printStatus(st);
    if (st.ok()) {
      std::printf("  Features: ops=0x%02X modes=0x%02X special=0x%02X\n",
                  static_cast<unsigned>(ops),
                  static_cast<unsigned>(modes),
                  static_cast<unsigned>(special));
    }
  } else if (std::strcmp(trimmed, "probe") == 0) {
    logInfo("Probing device (no health tracking)...");
    auto st = device.probe();
    printStatus(st);
  } else if (std::strcmp(trimmed, "dirty") == 0) {
    printPersistentDirtyState("=== Persistent Config Dirty State ===");
  } else if (std::strcmp(trimmed, "resync") == 0) {
    std::printf("=== Persistent Config Resync ===\n");
    std::printf("Before:\n");
    printPersistentDirtyState(nullptr);
    auto st = device.resyncPersistentConfig();
    printStatus(st);
    std::printf("After:\n");
    printPersistentDirtyState(nullptr);
  } else if (std::strcmp(trimmed, "id") == 0) {
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
      std::printf("  Group=0x%04X, Subgroup=0x%02X, Available=0x%02X\n",
                  static_cast<unsigned>(group),
                  static_cast<unsigned>(subgroup),
                  static_cast<unsigned>(avail));
    }
  } else if (std::strcmp(trimmed, "status") == 0) {
    uint8_t status = 0;
    auto st = device.readStatus(status);
    printStatus(st);
    if (st.ok()) {
      diag::printStatusByte(status);
      std::printf("  hasCo2Error(): %s\n", EE871::EE871::hasCo2Error(status) ? "YES" : "NO");
      if (EE871::EE871::hasCo2Error(status) && device.hasErrorCode()) {
        uint8_t code = 0;
        auto errSt = device.readErrorCode(code);
        printStatus(errSt);
        if (errSt.ok()) {
          std::printf("  CO2 error: %u (%s)\n",
                      static_cast<unsigned>(code),
                      EE871::cmd::co2ErrorCodeName(code));
        }
      }
      printPersistentDirtySummaryIfDirty();
    }
  } else if (std::strcmp(trimmed, "co2fast") == 0) {
    uint16_t ppm = 0;
    auto st = device.readCo2Fast(ppm);
    printStatus(st);
    if (st.ok()) {
      std::printf("  CO2 fast: %u ppm\n", static_cast<unsigned>(ppm));
    }
  } else if (std::strcmp(trimmed, "co2avg") == 0) {
    uint16_t ppm = 0;
    auto st = device.readCo2Average(ppm);
    printStatus(st);
    if (st.ok()) {
      std::printf("  CO2 avg: %u ppm\n", static_cast<unsigned>(ppm));
    }
  } else if (std::strcmp(trimmed, "error") == 0) {
    uint8_t code = 0;
    auto st = device.readErrorCode(code);
    printStatus(st);
    if (st.ok()) {
      std::printf("  Error code: %u (%s)\n",
                  static_cast<unsigned>(code),
                  EE871::cmd::co2ErrorCodeName(code));
    }
  } else if (startsWith(trimmed, "reg ")) {
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2) {
      logWarn("Usage: reg read|write|dump");
      return;
    }
    if (std::strcmp(tok.argv[1], "read") == 0) {
      if (tok.argc != 3) {
        logWarn("Usage: reg read <addr>");
        return;
      }
      uint8_t addr = 0;
      if (!parseU8Token(tok.argv[2], addr)) {
        logWarn("Invalid address");
        return;
      }
      if (!ensureProbeOk()) {
        return;
      }
      uint8_t value = 0;
      auto st = device.customRead(addr, value);
      printStatus(st);
      if (st.ok()) {
        std::printf("  Reg[0x%02X] = 0x%02X (%u)\n",
                    static_cast<unsigned>(addr),
                    static_cast<unsigned>(value),
                    static_cast<unsigned>(value));
      }
    } else if (std::strcmp(tok.argv[1], "write") == 0) {
      if (tok.argc != 4) {
        logWarn("Usage: reg write <addr> <value>");
        return;
      }
      uint8_t addr = 0;
      uint8_t value = 0;
      if (!parseU8Token(tok.argv[2], addr) || !parseU8Token(tok.argv[3], value)) {
        logWarn("Invalid address/value");
        return;
      }
      if (!ensureProbeOk()) {
        return;
      }
      auto st = device.customWrite(addr, value);
      printStatus(st);
      if (st.ok()) {
        std::printf("  Reg[0x%02X] <= 0x%02X\n",
                    static_cast<unsigned>(addr),
                    static_cast<unsigned>(value));
      }
    } else if (std::strcmp(tok.argv[1], "dump") == 0) {
      uint8_t start = 0;
      uint16_t len = CUSTOM_MEM_SIZE;
      if (tok.argc > 4) {
        logWarn("Usage: reg dump [start] [len]");
        return;
      }
      if (tok.argc >= 3) {
        if (!parseU8Token(tok.argv[2], start)) {
          logWarn("Invalid start");
          return;
        }
        len = static_cast<uint16_t>(CUSTOM_MEM_SIZE - start);
      }
      if (tok.argc == 4) {
        if (!parseU16Token(tok.argv[3], len)) {
          logWarn("Invalid length");
          return;
        }
      }
      if (len == 0U || (static_cast<uint16_t>(start) + len) > CUSTOM_MEM_SIZE) {
        logWarn("Range out of bounds");
        return;
      }
      if (!ensureProbeOk()) {
        return;
      }
      uint16_t remaining = len;
      uint16_t offset = start;
      std::printf("=== Custom Register Dump ===\n");
      while (remaining > 0U) {
        const uint16_t chunk = (remaining > REG_DUMP_CHUNK_LEN) ? REG_DUMP_CHUNK_LEN : remaining;
        uint8_t buf[REG_DUMP_CHUNK_LEN] = {};
        auto st = device.customRead(static_cast<uint8_t>(offset), buf, chunk);
        if (!st.ok()) {
          printStatus(st);
          return;
        }
        std::printf("  0x%02X:", static_cast<unsigned>(offset & 0xFFU));
        for (uint16_t i = 0; i < chunk; ++i) {
          std::printf(" %02X", static_cast<unsigned>(buf[i]));
        }
        std::printf("\n");
        offset = static_cast<uint16_t>(offset + chunk);
        remaining = static_cast<uint16_t>(remaining - chunk);
      }
    } else {
      logWarn("Unknown reg subcommand: %s", tok.argv[1]);
    }
  } else if (startsWith(trimmed, "ctrl ")) {
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc != 2) {
      logWarn("Usage: ctrl <main_nibble>");
      return;
    }
    uint8_t mainNibble = 0;
    if (!parseU8Token(tok.argv[1], mainNibble)) {
      logWarn("Usage: ctrl <main_nibble>");
      return;
    }
    uint8_t value = 0;
    auto st = device.readControlByte(mainNibble, value);
    printStatus(st);
    if (st.ok()) {
      std::printf("  ctrl(0x%02X) -> 0x%02X (%u)\n",
                  static_cast<unsigned>(mainNibble),
                  static_cast<unsigned>(value),
                  static_cast<unsigned>(value));
    }
  } else if (startsWith(trimmed, "u16 ")) {
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc != 3) {
      logWarn("Usage: u16 <main_lo> <main_hi>");
      return;
    }
    uint8_t lo = 0;
    uint8_t hi = 0;
    if (!parseU8Token(tok.argv[1], lo) || !parseU8Token(tok.argv[2], hi)) {
      logWarn("Invalid main_lo/main_hi");
      return;
    }
    uint16_t value = 0;
    auto st = device.readU16(lo, hi, value);
    printStatus(st);
    if (st.ok()) {
      std::printf("  u16(0x%02X,0x%02X) -> 0x%04X (%u)\n",
                  static_cast<unsigned>(lo),
                  static_cast<unsigned>(hi),
                  static_cast<unsigned>(value),
                  static_cast<unsigned>(value));
    }
  } else if (startsWith(trimmed, "ptr ")) {
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc != 2) {
      logWarn("Usage: ptr <addr16>");
      return;
    }
    uint16_t ptr = 0;
    if (!parseU16Token(tok.argv[1], ptr)) {
      logWarn("Usage: ptr <addr16>");
      return;
    }
    auto st = device.setCustomPointer(ptr);
    printStatus(st);
  } else if (std::strcmp(trimmed, "drv") == 0) {
    printDriverHealth();
  } else if (std::strcmp(trimmed, "recover") == 0) {
    logInfo("Attempting recovery...");
    auto st = device.recover();
    printStatus(st);
    printDriverHealth();
  } else if (std::strcmp(trimmed, "fw") == 0) {
    uint8_t main = 0;
    uint8_t sub = 0;
    auto st = device.readFirmwareVersion(main, sub);
    printStatus(st);
    if (st.ok()) {
      std::printf("  Firmware: %u.%u\n", static_cast<unsigned>(main), static_cast<unsigned>(sub));
    }
  } else if (std::strcmp(trimmed, "e2spec") == 0) {
    uint8_t ver = 0;
    auto st = device.readE2SpecVersion(ver);
    printStatus(st);
    if (st.ok()) {
      std::printf("  E2 spec version: %u\n", static_cast<unsigned>(ver));
    }
  } else if (std::strcmp(trimmed, "features") == 0) {
    uint8_t ops = 0;
    uint8_t modes = 0;
    uint8_t special = 0;
    auto st = device.readOperatingFunctions(ops);
    printStatus(st);
    if (st.ok()) st = device.readOperatingModeSupport(modes);
    if (st.ok()) st = device.readSpecialFeatures(special);
    if (st.ok()) {
      std::printf("  Operating functions (0x07): 0x%02X\n", static_cast<unsigned>(ops));
      std::printf("    Serial number: %s\n", device.hasSerialNumber() ? "yes" : "no");
      std::printf("    Part name: %s\n", device.hasPartName() ? "yes" : "no");
      std::printf("    Address config: %s\n", device.hasAddressConfig() ? "yes" : "no");
      std::printf("    Global interval: %s\n", device.hasGlobalInterval() ? "yes" : "no");
      std::printf("    Specific interval: %s\n", device.hasSpecificInterval() ? "yes" : "no");
      std::printf("    Filter config: %s\n", device.hasFilterConfig() ? "yes" : "no");
      std::printf("    Error code: %s\n", device.hasErrorCode() ? "yes" : "no");
      std::printf("  Mode support (0x08): 0x%02X\n", static_cast<unsigned>(modes));
      std::printf("    Low power: %s\n", device.hasLowPowerMode() ? "yes" : "no");
      std::printf("    E2 priority: %s\n", device.hasE2Priority() ? "yes" : "no");
      std::printf("  Special features (0x09): 0x%02X\n", static_cast<unsigned>(special));
      std::printf("    Auto adjust: %s\n", device.hasAutoAdjust() ? "yes" : "no");
    }
  } else if (std::strcmp(trimmed, "caps") == 0) {
    std::printf("=== Capabilities ===\n");
    std::printf("  hasSerialNumber: %s\n", device.hasSerialNumber() ? "true" : "false");
    std::printf("  hasPartName: %s\n", device.hasPartName() ? "true" : "false");
    std::printf("  hasAddressConfig: %s\n", device.hasAddressConfig() ? "true" : "false");
    std::printf("  hasGlobalInterval: %s\n", device.hasGlobalInterval() ? "true" : "false");
    std::printf("  hasSpecificInterval: %s\n", device.hasSpecificInterval() ? "true" : "false");
    std::printf("  hasFilterConfig: %s\n", device.hasFilterConfig() ? "true" : "false");
    std::printf("  hasErrorCode: %s\n", device.hasErrorCode() ? "true" : "false");
    std::printf("  hasLowPowerMode: %s\n", device.hasLowPowerMode() ? "true" : "false");
    std::printf("  hasE2Priority: %s\n", device.hasE2Priority() ? "true" : "false");
    std::printf("  hasAutoAdjust: %s\n", device.hasAutoAdjust() ? "true" : "false");
  } else if (std::strcmp(trimmed, "serial") == 0) {
    uint8_t sn[16] = {0};
    auto st = device.readSerialNumber(sn);
    printStatus(st);
    if (st.ok()) {
      printAsciiField("Serial", sn, sizeof(sn), false);
      std::printf("  Hex: ");
      for (uint8_t v : sn) {
        std::printf("%02X ", static_cast<unsigned>(v));
      }
      std::printf("\n");
    }
  } else if (std::strcmp(trimmed, "partname") == 0) {
    uint8_t name[16] = {0};
    auto st = device.readPartName(name);
    printStatus(st);
    if (st.ok()) {
      printAsciiField("Part name", name, sizeof(name), true);
    }
  } else if (startsWith(trimmed, "partname ")) {
    char* text = trimInPlace(trimmed + 9);
    if (text[0] == '\0') {
      logWarn("Usage: partname <text>");
      return;
    }
    uint8_t out[16] = {0};
    const size_t textLen = std::strlen(text);
    const size_t maxLen = (textLen > sizeof(out)) ? sizeof(out) : textLen;
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
        printAsciiField("Part name", verify, sizeof(verify), true);
      }
    }
  } else if (std::strcmp(trimmed, "addr") == 0) {
    uint8_t addr = 0;
    auto st = device.readBusAddress(addr);
    printStatus(st);
    if (st.ok()) {
      std::printf("  Bus address: %u\n", static_cast<unsigned>(addr));
    }
  } else if (startsWith(trimmed, "addr ")) {
    long val = 0;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2 || !parseIntToken(tok.argv[1], val)) {
      val = 0;
    }
    logInfo("Writing bus address %ld (power cycle required)...", val);
    auto st = device.writeBusAddress(static_cast<uint8_t>(val));
    printStatus(st);
  } else if (std::strcmp(trimmed, "interval") == 0) {
    uint16_t interval = 0;
    auto st = device.readMeasurementInterval(interval);
    printStatus(st);
    if (st.ok()) {
      std::printf("  Interval: %u deciseconds (%.1f s)\n",
                  static_cast<unsigned>(interval),
                  static_cast<double>(interval / 10.0f));
    }
  } else if (startsWith(trimmed, "interval ")) {
    long val = 0;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2 || !parseIntToken(tok.argv[1], val)) {
      val = 0;
    }
    logInfo("Writing interval %ld deciseconds...", val);
    auto st = device.writeMeasurementInterval(static_cast<uint16_t>(val));
    printStatus(st);
  } else if (std::strcmp(trimmed, "factor") == 0) {
    int8_t factor = 0;
    auto st = device.readCo2IntervalFactor(factor);
    printStatus(st);
    if (st.ok()) {
      std::printf("  CO2 interval factor: %d\n", static_cast<int>(factor));
    }
  } else if (startsWith(trimmed, "factor ")) {
    long val = 0;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2 || !parseIntToken(tok.argv[1], val) || val < -128L || val > 127L) {
      logWarn("factor must be -128..127");
      return;
    }
    auto st = device.writeCo2IntervalFactor(static_cast<int8_t>(val));
    printStatus(st);
  } else if (std::strcmp(trimmed, "filter") == 0) {
    uint8_t filter = 0;
    auto st = device.readCo2Filter(filter);
    printStatus(st);
    if (st.ok()) {
      std::printf("  CO2 filter: %u\n", static_cast<unsigned>(filter));
    }
  } else if (startsWith(trimmed, "filter ")) {
    long val = 0;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2 || !parseIntToken(tok.argv[1], val)) {
      val = 0;
    }
    auto st = device.writeCo2Filter(static_cast<uint8_t>(val));
    printStatus(st);
  } else if (std::strcmp(trimmed, "mode") == 0) {
    uint8_t mode = 0;
    auto st = device.readOperatingMode(mode);
    printStatus(st);
    if (st.ok()) {
      std::printf("  Operating mode: 0x%02X\n", static_cast<unsigned>(mode));
      std::printf("    Measure mode: %s\n", (mode & 0x01U) ? "low power" : "freerunning");
      std::printf("    Priority: %s\n", (mode & 0x02U) ? "E2 comm" : "measurement");
    }
  } else if (startsWith(trimmed, "mode ")) {
    long val = 0;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2 || !parseIntToken(tok.argv[1], val)) {
      val = 0;
    }
    auto st = device.writeOperatingMode(static_cast<uint8_t>(val));
    printStatus(st);
  } else if (std::strcmp(trimmed, "offset") == 0) {
    int16_t offset = 0;
    auto st = device.readCo2Offset(offset);
    printStatus(st);
    if (st.ok()) {
      std::printf("  CO2 offset: %d ppm\n", static_cast<int>(offset));
    }
  } else if (startsWith(trimmed, "offset ")) {
    long val = 0;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2 || !parseIntToken(tok.argv[1], val)) {
      val = 0;
    }
    logInfo("Writing CO2 offset %ld...", val);
    auto st = device.writeCo2Offset(static_cast<int16_t>(val));
    printStatus(st);
  } else if (std::strcmp(trimmed, "gain") == 0) {
    uint16_t gain = 0;
    auto st = device.readCo2Gain(gain);
    printStatus(st);
    if (st.ok()) {
      std::printf("  CO2 gain: %u (factor=%.4f)\n",
                  static_cast<unsigned>(gain),
                  static_cast<double>(gain / 32768.0f));
    }
  } else if (startsWith(trimmed, "gain ")) {
    long val = 0;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2 || !parseIntToken(tok.argv[1], val) || val < 0L || val > 65535L) {
      logWarn("gain must be 0..65535");
      return;
    }
    auto st = device.writeCo2Gain(static_cast<uint16_t>(val));
    printStatus(st);
  } else if (std::strcmp(trimmed, "calpoints") == 0) {
    uint16_t lower = 0;
    uint16_t upper = 0;
    auto st = device.readCo2CalPoints(lower, upper);
    printStatus(st);
    if (st.ok()) {
      std::printf("  Cal points: lower=%u ppm, upper=%u ppm\n",
                  static_cast<unsigned>(lower),
                  static_cast<unsigned>(upper));
    }
  } else if (std::strcmp(trimmed, "autoadj") == 0) {
    bool running = false;
    auto st = device.readAutoAdjustStatus(running);
    printStatus(st);
    if (st.ok()) {
      std::printf("  Auto adjustment: %s\n", running ? "RUNNING" : "idle");
    }
  } else if (std::strcmp(trimmed, "autoadj start") == 0) {
    logInfo("Starting auto adjustment (takes ~5 minutes)...");
    auto st = device.startAutoAdjust();
    printStatus(st);
  } else if (std::strcmp(trimmed, "buscheck") == 0) {
    auto st = device.checkBusIdle();
    printStatus(st);
    if (st.ok()) {
      std::printf("  Bus is idle (both lines high)\n");
    }
  } else if (std::strcmp(trimmed, "libreset") == 0) {
    logInfo("Performing library bus reset...");
    auto st = device.busReset();
    printStatus(st);
  } else if (std::strcmp(trimmed, "verbose") == 0) {
    logInfo("Verbose mode: %s%s%s", onOffColor(verboseMode), verboseMode ? "ON" : "OFF", LOG_COLOR_RESET);
  } else if (startsWith(trimmed, "verbose ")) {
    long val = 0;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc < 2 || !parseIntToken(tok.argv[1], val)) {
      val = 0;
    }
    verboseMode = (val != 0L);
    if (verboseMode) {
      buslog::clear();
    }
    buslog::setEnabled(verboseMode);
    logInfo("Verbose mode: %s%s%s", onOffColor(verboseMode), verboseMode ? "ON" : "OFF", LOG_COLOR_RESET);
  } else if (std::strcmp(trimmed, "trace stats") == 0) {
    buslog::printStats();
  } else if (std::strcmp(trimmed, "trace clear") == 0) {
    buslog::clear();
    logInfo("Bus trace cleared");
  } else if (std::strcmp(trimmed, "diag") == 0) {
    diag::runFullDiagnostics(deviceCfg);
  } else if (std::strcmp(trimmed, "levels") == 0) {
    diag::printBusLevels(deviceCfg);
  } else if (std::strcmp(trimmed, "pintest") == 0) {
    diag::testPinToggle(deviceCfg);
  } else if (std::strcmp(trimmed, "clocktest") == 0) {
    diag::testClockPulses(deviceCfg, 10);
  } else if (std::strcmp(trimmed, "sniff") == 0) {
    if (diag::sniffer().isActive()) {
      diag::sniffer().stop();
    } else {
      diag::sniffer().start(&deviceCfg);
    }
  } else if (std::strcmp(trimmed, "scan") == 0) {
    diag::scanAddresses(deviceCfg);
  } else if (std::strcmp(trimmed, "timing") == 0) {
    diag::discoverTiming(deviceCfg);
  } else if (std::strcmp(trimmed, "busreset") == 0) {
    diag::sendRecoveryClocks(deviceCfg);
  } else if (startsWith(trimmed, "tx ")) {
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc != 2) {
      logWarn("Usage: tx <hex>");
      return;
    }
    char* end = nullptr;
    const unsigned long value = std::strtoul(tok.argv[1], &end, 16);
    if (end == tok.argv[1] || *end != '\0' || value > 0xFFUL) {
      logWarn("Usage: tx <hex>");
      return;
    }
    diag::testTransaction(deviceCfg, static_cast<uint8_t>(value));
  } else if (std::strcmp(trimmed, "libtest") == 0) {
    diag::testLibraryCommands(deviceCfg);
  } else if (std::strcmp(trimmed, "selftest") == 0) {
    runSelfTest();
  } else if (std::strcmp(trimmed, "stress_mix") == 0) {
    runStressMix(100);
  } else if (startsWith(trimmed, "stress_mix ")) {
    long count = 100;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc >= 2) {
      (void)parseIntToken(tok.argv[1], count);
    }
    if (count <= 0L) count = 100;
    runStressMix(static_cast<int>(count));
  } else if (startsWith(trimmed, "stress")) {
    long count = 100;
    Tokens tok;
    splitTokens(trimmed, tok);
    if (tok.argc >= 2) {
      (void)parseIntToken(tok.argv[1], count);
    }
    if (count <= 0L) count = 100;
    runStress(static_cast<int>(count));
  } else {
    logWarn("Unknown command: %s", trimmed);
  }
}

// ============================================================================
// Console input and app_main
// ============================================================================

void configureConsoleInput() {
  setvbuf(stdin, nullptr, _IONBF, 0);
  const int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
  if (flags >= 0) {
    (void)fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
  }
}

bool pollLine(char* out, size_t outCap) {
  static char buffer[MAX_LINE_LENGTH + 1U] = {};
  static size_t len = 0;
  static bool overflowed = false;

  while (true) {
    errno = 0;
    const int ch = getchar();
    if (ch == EOF) {
      break;
    }
    const char c = static_cast<char>(ch);
    if (c == '\b' || c == 0x7F) {
      if (!overflowed && len > 0U) {
        len--;
        buffer[len] = '\0';
      }
      continue;
    }
    if (c == '\r' || c == '\n') {
      if (overflowed) {
        len = 0;
        buffer[0] = '\0';
        overflowed = false;
        continue;
      }
      if (len == 0U) {
        continue;
      }
      buffer[len] = '\0';
      std::strncpy(out, buffer, outCap - 1U);
      out[outCap - 1U] = '\0';
      len = 0;
      buffer[0] = '\0';
      char* trimmed = trimInPlace(out);
      if (trimmed != out) {
        std::memmove(out, trimmed, std::strlen(trimmed) + 1U);
      }
      return out[0] != '\0';
    }
    if (overflowed) {
      continue;
    }
    if (len < MAX_LINE_LENGTH) {
      buffer[len++] = c;
      buffer[len] = '\0';
    } else {
      len = 0;
      buffer[0] = '\0';
      overflowed = true;
    }
  }

  return false;
}

void configureDevice() {
  deviceCfg.setScl = trace::setScl;
  deviceCfg.setSda = trace::setSda;
  deviceCfg.readScl = trace::readScl;
  deviceCfg.readSda = trace::readSda;
  deviceCfg.delayUs = trace::delayUs;
  deviceCfg.busUser = &e2Bus;
  deviceCfg.deviceAddress = EE871_ADDRESS;
  deviceCfg.clockLowUs = E2_CLOCK_LOW_US;
  deviceCfg.clockHighUs = E2_CLOCK_HIGH_US;
  deviceCfg.bitTimeoutUs = E2_BIT_TIMEOUT_US;
  deviceCfg.byteTimeoutUs = E2_BYTE_TIMEOUT_US;
  deviceCfg.writeDelayMs = E2_WRITE_DELAY_MS;
  deviceCfg.intervalWriteDelayMs = E2_INTERVAL_WRITE_DELAY_MS;
  deviceCfg.offlineThreshold = 5;
}

extern "C" void app_main(void) {
  configureConsoleInput();

  std::printf("\n");
  std::printf("=== EE871 Bringup Example ===\n");

  const esp_err_t initErr = ee871_idf::init(e2Bus, E2_SCL, E2_SDA, false);
  if (initErr != ESP_OK) {
    std::printf("[E] Failed to initialize E2 pins: %s\n", esp_err_to_name(initErr));
    return;
  }

  std::printf("[I] E2 initialized (DATA=%d, CLOCK=%d)\n",
              static_cast<int>(E2_SDA),
              static_cast<int>(E2_SCL));

  configureDevice();

  auto st = device.begin(deviceCfg);
  if (!st.ok()) {
    std::printf("[E] Failed to initialize device\n");
    printStatus(st);
    std::printf("\n[I] Running basic bus diagnostics...\n\n");
    diag::printBusLevels(deviceCfg);
    std::printf("\n");
    diag::testPinToggle(deviceCfg);
    std::printf("\n[I] Type 'diag' for full diagnostics, 'help' for commands\n\n");
  } else {
    std::printf("[I] Device initialized successfully\n");
    printDriverHealth();
  }

  std::printf("\nType 'help' for commands\n");
  printPrompt();

  char line[MAX_LINE_LENGTH + 1U] = {};
  while (true) {
    device.tick(nowMs());
    diag::sniffer().tick(deviceCfg);
    if (pollLine(line, sizeof(line))) {
      processCommand(line);
      buslog::flush();
      printPrompt();
    }
    buslog::flush();
    delayMs(10);
  }
}
