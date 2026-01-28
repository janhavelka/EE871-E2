/// @file main.cpp
/// @brief Basic bringup example for EE871 (E2 bus)
/// @note This is an EXAMPLE, not part of the library

#include <Arduino.h>
#include "common/Log.h"
#include "common/BoardConfig.h"
#include "common/E2Transport.h"
#include "common/E2Diagnostics.h"

#include "EE871/EE871.h"

// ============================================================================
// Globals
// ============================================================================

ee871::EE871 device;
ee871::Config deviceCfg;  // Stored for diagnostics
bool verboseMode = false;

// ============================================================================
// Helper Functions
// ============================================================================

/// Convert error code to string
const char* errToStr(ee871::Err err) {
  using ee871::Err;
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
    default:                       return "UNKNOWN";
  }
}

/// Convert driver state to string
const char* stateToStr(ee871::DriverState st) {
  using ee871::DriverState;
  switch (st) {
    case DriverState::UNINIT:   return "UNINIT";
    case DriverState::READY:    return "READY";
    case DriverState::DEGRADED: return "DEGRADED";
    case DriverState::OFFLINE:  return "OFFLINE";
    default:                    return "UNKNOWN";
  }
}

/// Print status details
void printStatus(const ee871::Status& st) {
  Serial.printf("  Status: %s (code=%u, detail=%ld)\n",
                errToStr(st.code),
                static_cast<unsigned>(st.code),
                static_cast<long>(st.detail));
  if (st.msg && st.msg[0]) {
    Serial.printf("  Message: %s\n", st.msg);
  }
}

/// Print driver health information
void printDriverHealth() {
  Serial.println("=== Driver State ===");
  Serial.printf("  State: %s\n", stateToStr(device.state()));
  Serial.printf("  Consecutive failures: %u\n", device.consecutiveFailures());
  Serial.printf("  Total failures: %lu\n", static_cast<unsigned long>(device.totalFailures()));
  Serial.printf("  Total success: %lu\n", static_cast<unsigned long>(device.totalSuccess()));
  Serial.printf("  Last OK at: %lu ms\n", static_cast<unsigned long>(device.lastOkMs()));
  Serial.printf("  Last error at: %lu ms\n", static_cast<unsigned long>(device.lastErrorMs()));
  if (device.lastError().code != ee871::Err::OK) {
    Serial.printf("  Last error: %s\n", errToStr(device.lastError().code));
  }
}

/// Print help
void printHelp() {
  Serial.println("=== Device Commands ===");
  Serial.println("  probe             - Probe device (no health tracking)");
  Serial.println("  id                - Read group/subgroup/available bits");
  Serial.println("  status            - Read status byte (starts measurement)");
  Serial.println("  co2fast           - Read MV3 (fast response)");
  Serial.println("  co2avg            - Read MV4 (averaged)");
  Serial.println("  error             - Read error code (if status indicates error)");
  Serial.println("  drv               - Show driver state and health");
  Serial.println("  recover           - Attempt recovery");
  Serial.println();
  Serial.println("=== Device Info ===");
  Serial.println("  fw                - Read firmware version");
  Serial.println("  e2spec            - Read E2 spec version");
  Serial.println("  features          - Read feature support flags");
  Serial.println("  serial            - Read serial number");
  Serial.println("  partname          - Read part name");
  Serial.println();
  Serial.println("=== Configuration ===");
  Serial.println("  addr              - Read current bus address");
  Serial.println("  addr <0-7>        - Write bus address (needs power cycle)");
  Serial.println("  interval          - Read measurement interval");
  Serial.println("  interval <dec>    - Write interval (150-36000 deciseconds)");
  Serial.println("  filter            - Read CO2 filter setting");
  Serial.println("  filter <val>      - Write CO2 filter");
  Serial.println("  mode              - Read operating mode");
  Serial.println("  mode <val>        - Write operating mode (0-3)");
  Serial.println();
  Serial.println("=== Calibration (Advanced) ===");
  Serial.println("  offset            - Read CO2 offset (ppm)");
  Serial.println("  offset <val>      - Write CO2 offset (signed)");
  Serial.println("  gain              - Read CO2 gain");
  Serial.println("  calpoints         - Read last cal points");
  Serial.println("  autoadj           - Read auto-adjust status");
  Serial.println("  autoadj start     - Start auto-adjustment (~5 min)");
  Serial.println();
  Serial.println("=== Bus Safety ===");
  Serial.println("  buscheck          - Check if bus is idle");
  Serial.println("  libreset          - Bus reset via library");
  Serial.println();
  Serial.println("=== Diagnostics ===");
  Serial.println("  diag              - Run FULL diagnostic suite");
  Serial.println("  levels            - Read current bus levels");
  Serial.println("  pintest           - Test pin toggle (can MCU control bus?)");
  Serial.println("  clocktest         - Generate clock pulses and verify");
  Serial.println("  sniff 1           - Start background sniffer (non-blocking)");
  Serial.println("  sniff 0           - Stop background sniffer");
  Serial.println("  sniff             - Toggle sniffer on/off");
  Serial.println("  scan              - Scan all 8 E2 addresses");
  Serial.println("  timing            - Try different clock frequencies");
  Serial.println("  busreset          - Send 9 clocks to recover stuck bus");
  Serial.println("  tx <hex>          - Test transaction with control byte");
  Serial.println("  libtest           - Test all library commands (begin uses)");
  Serial.println();
  Serial.println("=== Other ===");
  Serial.println("  help              - Show this help");
  Serial.println("  verbose 0|1       - Set verbose mode");
}

// ============================================================================
// Command Processing
// ============================================================================

void processCommand(const String& cmd) {
  String trimmed = cmd;
  trimmed.trim();

  if (trimmed == "help" || trimmed == "?") {
    printHelp();
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
      Serial.printf("    Serial number: %s\n", (ops & 0x01) ? "yes" : "no");
      Serial.printf("    Part name: %s\n", (ops & 0x02) ? "yes" : "no");
      Serial.printf("    Address config: %s\n", (ops & 0x04) ? "yes" : "no");
      Serial.printf("    Global interval: %s\n", (ops & 0x10) ? "yes" : "no");
      Serial.printf("    Specific interval: %s\n", (ops & 0x20) ? "yes" : "no");
      Serial.printf("    Filter config: %s\n", (ops & 0x40) ? "yes" : "no");
      Serial.printf("    Error code: %s\n", (ops & 0x80) ? "yes" : "no");
      Serial.printf("  Mode support (0x08): 0x%02X\n", modes);
      Serial.printf("    Low power: %s\n", (modes & 0x01) ? "yes" : "no");
      Serial.printf("    E2 priority: %s\n", (modes & 0x02) ? "yes" : "no");
      Serial.printf("  Special features (0x09): 0x%02X\n", special);
      Serial.printf("    Auto adjust: %s\n", (special & 0x01) ? "yes" : "no");
    }
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
  
  } else if (trimmed.startsWith("verbose ")) {
    int val = trimmed.substring(8).toInt();
    verboseMode = (val != 0);
    LOGI("Verbose mode: %s", verboseMode ? "ON" : "OFF");
  
  // === Diagnostic Commands ===
  } else if (trimmed == "diag") {
    e2diag::runFullDiagnostics(deviceCfg);
  } else if (trimmed == "levels") {
    e2diag::printBusLevels(deviceCfg);
  } else if (trimmed == "pintest") {
    e2diag::testPinToggle(deviceCfg);
  } else if (trimmed == "clocktest") {
    e2diag::testClockPulses(deviceCfg, 10);
  } else if (trimmed == "sniff 1") {
    e2diag::sniffer().start(&deviceCfg);
  } else if (trimmed == "sniff 0") {
    e2diag::sniffer().stop();
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
  
  } else {
    LOGW("Unknown command: %s", trimmed.c_str());
  }
}

// ============================================================================
// Setup and Loop
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);  // Give USB CDC time to enumerate
  
  Serial.println();
  Serial.println("=== EE871 Bringup Example ===");

  if (!board::initE2()) {
    Serial.println("[E] Failed to initialize E2 pins");
    return;
  }

  Serial.printf("[I] E2 initialized (DATA=%d, CLOCK=%d)\n", board::E2_DATA, board::E2_CLOCK);

  // Configure and store for diagnostics
  deviceCfg.setScl = transport::setScl;
  deviceCfg.setSda = transport::setSda;
  deviceCfg.readScl = transport::readScl;
  deviceCfg.readSda = transport::readSda;
  deviceCfg.delayUs = transport::delayUs;
  deviceCfg.busUser = &board::e2Pins();
  deviceCfg.deviceAddress = ee871::cmd::DEFAULT_DEVICE_ADDRESS;
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
}
