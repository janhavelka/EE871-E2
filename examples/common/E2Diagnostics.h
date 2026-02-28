/// @file E2Diagnostics.h
/// @brief Deep E2 bus diagnostics for bringup and debugging (examples only)
/// @note NOT part of the library
#pragma once

#include <Arduino.h>
#include "EE871/CommandTable.h"
#include "EE871/Config.h"
#include "E2Transport.h"
#include "Log.h"

namespace e2diag {

inline const char* okColor(bool ok) {
  return ok ? LOG_COLOR_GREEN : LOG_COLOR_RED;
}

inline const char* warnColor() {
  return LOG_COLOR_YELLOW;
}

inline const char* neutralColor() {
  return LOG_COLOR_GRAY;
}

// ============================================================================
// Status Display
// ============================================================================

inline void printStatus(uint8_t status) {
  Serial.printf("Status: %s0x%02X%s", LOG_COLOR_CYAN, status, LOG_COLOR_RESET);
  if (status & EE871::cmd::STATUS_CO2_ERROR_MASK) {
    Serial.printf(" (%sCO2 error%s)", LOG_COLOR_RED, LOG_COLOR_RESET);
  }
  Serial.println();
}

// ============================================================================
// Bus Level Check
// ============================================================================

struct BusLevels {
  bool scl;
  bool sda;
};

/// Read current bus levels
inline BusLevels readBusLevels(const EE871::Config& cfg) {
  BusLevels lvl;
  lvl.scl = cfg.readScl(cfg.busUser);
  lvl.sda = cfg.readSda(cfg.busUser);
  return lvl;
}

/// Print current bus levels
inline void printBusLevels(const EE871::Config& cfg) {
  auto lvl = readBusLevels(cfg);
  Serial.printf("%s=== Bus Levels ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  Serial.printf("  SCL: %s%s%s\n",
                okColor(lvl.scl),
                lvl.scl ? "HIGH (idle)" : "LOW (held)",
                LOG_COLOR_RESET);
  Serial.printf("  SDA: %s%s%s\n",
                okColor(lvl.sda),
                lvl.sda ? "HIGH (idle)" : "LOW (held)",
                LOG_COLOR_RESET);
  
  if (!lvl.scl && !lvl.sda) {
    Serial.printf("  %sWARNING%s: Both lines LOW - bus stuck or no pull-ups!\n",
                  warnColor(), LOG_COLOR_RESET);
  } else if (!lvl.scl) {
    Serial.printf("  %sWARNING%s: SCL held LOW - clock stretching or stuck!\n",
                  warnColor(), LOG_COLOR_RESET);
  } else if (!lvl.sda) {
    Serial.printf("  %sWARNING%s: SDA held LOW - slave holding or stuck!\n",
                  warnColor(), LOG_COLOR_RESET);
  } else {
    Serial.printf("  %sOK%s: Bus idle (both HIGH)\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  }
}

// ============================================================================
// Pin Toggle Test
// ============================================================================

/// Test if we can toggle pins and read them back
inline void testPinToggle(const EE871::Config& cfg) {
  Serial.printf("%s=== Pin Toggle Test ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  Serial.println("Testing if MCU can control and read bus lines...\n");

  // Test SCL
  Serial.println("SCL pin test:");
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  bool sclHigh = cfg.readScl(cfg.busUser);
  Serial.printf("  Set HIGH -> Read: %s%s%s\n",
                okColor(sclHigh),
                sclHigh ? "HIGH (OK)" : "LOW (FAIL - stuck or no pull-up)",
                LOG_COLOR_RESET);

  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  bool sclLow = cfg.readScl(cfg.busUser);
  Serial.printf("  Set LOW  -> Read: %s%s%s\n",
                okColor(!sclLow),
                sclLow ? "HIGH (FAIL - can't pull low)" : "LOW (OK)",
                LOG_COLOR_RESET);

  cfg.setScl(true, cfg.busUser);  // Release
  cfg.delayUs(100, cfg.busUser);

  // Test SDA
  Serial.println("\nSDA pin test:");
  cfg.setSda(true, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  bool sdaHigh = cfg.readSda(cfg.busUser);
  Serial.printf("  Set HIGH -> Read: %s%s%s\n",
                okColor(sdaHigh),
                sdaHigh ? "HIGH (OK)" : "LOW (FAIL - stuck or no pull-up)",
                LOG_COLOR_RESET);

  cfg.setSda(false, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  bool sdaLow = cfg.readSda(cfg.busUser);
  Serial.printf("  Set LOW  -> Read: %s%s%s\n",
                okColor(!sdaLow),
                sdaLow ? "HIGH (FAIL - can't pull low)" : "LOW (OK)",
                LOG_COLOR_RESET);

  cfg.setSda(true, cfg.busUser);  // Release
  cfg.delayUs(100, cfg.busUser);

  // Summary
  Serial.println("\nSummary:");
  bool sclOk = sclHigh && !sclLow;
  bool sdaOk = sdaHigh && !sdaLow;
  
  if (sclOk && sdaOk) {
    Serial.printf("  %sPASS%s: Both pins working correctly\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  } else {
    if (!sclHigh) {
      Serial.printf("  %sFAIL%s: SCL has no pull-up or is stuck LOW\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    if (sclLow) {
      Serial.printf("  %sFAIL%s: SCL cannot be pulled LOW by MCU\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    if (!sdaHigh) {
      Serial.printf("  %sFAIL%s: SDA has no pull-up or is stuck LOW\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    if (sdaLow) {
      Serial.printf("  %sFAIL%s: SDA cannot be pulled LOW by MCU\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
  }
}

// ============================================================================
// Bus Sniffer
// ============================================================================

struct SniffResult {
  uint32_t sclTransitions;
  uint32_t sdaTransitions;
  uint32_t durationMs;
  bool sclStuckLow;
  bool sdaStuckLow;
};

/// Sniff bus activity for a given duration
inline SniffResult sniffBus(const EE871::Config& cfg, uint32_t durationMs) {
  SniffResult result = {0, 0, durationMs, false, false};
  
  uint32_t startMs = millis();
  bool lastScl = cfg.readScl(cfg.busUser);
  bool lastSda = cfg.readSda(cfg.busUser);
  
  uint32_t sclLowStart = lastScl ? 0 : startMs;
  uint32_t sdaLowStart = lastSda ? 0 : startMs;
  
  while ((millis() - startMs) < durationMs) {
    bool scl = cfg.readScl(cfg.busUser);
    bool sda = cfg.readSda(cfg.busUser);
    
    if (scl != lastScl) {
      result.sclTransitions++;
      if (!scl) sclLowStart = millis();
      else sclLowStart = 0;
      lastScl = scl;
    }
    
    if (sda != lastSda) {
      result.sdaTransitions++;
      if (!sda) sdaLowStart = millis();
      else sdaLowStart = 0;
      lastSda = sda;
    }
    
    // Check for stuck condition (>100ms low)
    uint32_t now = millis();
    if (sclLowStart && (now - sclLowStart > 100)) result.sclStuckLow = true;
    if (sdaLowStart && (now - sdaLowStart > 100)) result.sdaStuckLow = true;
    
    delayMicroseconds(10);  // ~100kHz sample rate
  }
  
  return result;
}

/// Sniff and print bus activity
inline void sniffAndPrint(const EE871::Config& cfg, uint32_t durationMs = 2000) {
  Serial.printf("%s=== Bus Sniffer (%lu ms) ===%s\n", LOG_COLOR_CYAN, static_cast<unsigned long>(durationMs), LOG_COLOR_RESET);
  Serial.println("Monitoring bus activity...");
  
  auto result = sniffBus(cfg, durationMs);
  
  Serial.printf("\nResults over %lu ms:\n", static_cast<unsigned long>(result.durationMs));
  Serial.printf("  SCL transitions: %lu\n", static_cast<unsigned long>(result.sclTransitions));
  Serial.printf("  SDA transitions: %lu\n", static_cast<unsigned long>(result.sdaTransitions));
  
  if (result.sclTransitions > 0) {
    float sclFreq = (result.sclTransitions / 2.0f) / (result.durationMs / 1000.0f);
    Serial.printf("  SCL approx freq: %.1f Hz\n", sclFreq);
  }
  
  if (result.sclStuckLow) {
    Serial.printf("  %sWARNING%s: SCL was stuck LOW for >100ms\n", warnColor(), LOG_COLOR_RESET);
  }
  if (result.sdaStuckLow) {
    Serial.printf("  %sWARNING%s: SDA was stuck LOW for >100ms\n", warnColor(), LOG_COLOR_RESET);
  }
  
  if (result.sclTransitions == 0 && result.sdaTransitions == 0) {
    Serial.println("\n  No bus activity detected - bus is quiet");
  }
}

// ============================================================================
// Clock Pulse Test
// ============================================================================

/// Generate clock pulses and verify
inline void testClockPulses(const EE871::Config& cfg, int numPulses = 10) {
  Serial.printf("%s=== Clock Pulse Test (%d pulses) ===%s\n", LOG_COLOR_CYAN, numPulses, LOG_COLOR_RESET);
  
  int successHigh = 0, successLow = 0;
  
  for (int i = 0; i < numPulses; i++) {
    // Pull low
    cfg.setScl(false, cfg.busUser);
    cfg.delayUs(cfg.clockLowUs, cfg.busUser);
    bool readLow = cfg.readScl(cfg.busUser);
    if (!readLow) successLow++;
    
    // Release high
    cfg.setScl(true, cfg.busUser);
    cfg.delayUs(cfg.clockHighUs, cfg.busUser);
    bool readHigh = cfg.readScl(cfg.busUser);
    if (readHigh) successHigh++;
    
    Serial.printf("  Pulse %2d: LOW=%s%s%s HIGH=%s%s%s\n", 
                  i + 1,
                  okColor(!readLow),
                  readLow ? "FAIL" : "ok",
                  LOG_COLOR_RESET,
                  okColor(readHigh),
                  readHigh ? "ok" : "FAIL(stretched?)",
                  LOG_COLOR_RESET);
  }
  
  Serial.printf("\nResults: %d/%d LOW ok, %d/%d HIGH ok\n", 
                successLow, numPulses, successHigh, numPulses);
  
  if (successHigh < numPulses) {
    Serial.printf("  %sNOTE%s: HIGH failures may indicate clock stretching by slave\n",
                  LOG_COLOR_YELLOW, LOG_COLOR_RESET);
  }
}

// ============================================================================
// Raw Bit-Bang Functions
// ============================================================================

/// Send START condition
inline void sendStart(const EE871::Config& cfg) {
  cfg.setSda(true, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
  cfg.setSda(false, cfg.busUser);  // SDA high->low while SCL high
  cfg.delayUs(10, cfg.busUser);
  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(cfg.clockLowUs, cfg.busUser);
}

/// Send STOP condition
inline void sendStop(const EE871::Config& cfg) {
  cfg.setSda(false, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
  cfg.setSda(true, cfg.busUser);  // SDA low->high while SCL high
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
}

/// Send a byte, return true if ACK received
inline bool sendByteRaw(const EE871::Config& cfg, uint8_t data, bool verbose = false) {
  // Send 8 bits MSB first
  for (int i = 7; i >= 0; i--) {
    bool bit = (data >> i) & 1;
    cfg.setSda(bit, cfg.busUser);
    cfg.delayUs(10, cfg.busUser);
    cfg.setScl(true, cfg.busUser);
    cfg.delayUs(cfg.clockHighUs, cfg.busUser);
    cfg.setScl(false, cfg.busUser);
    cfg.delayUs(cfg.clockLowUs, cfg.busUser);
  }
  
  // Release SDA for ACK
  cfg.setSda(true, cfg.busUser);
  cfg.delayUs(10, cfg.busUser);
  
  // Clock for ACK bit
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs / 2, cfg.busUser);
  bool ack = !cfg.readSda(cfg.busUser);  // ACK = SDA low
  cfg.delayUs(cfg.clockHighUs / 2, cfg.busUser);
  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(cfg.clockLowUs, cfg.busUser);
  
  if (verbose) {
    Serial.printf("  Sent 0x%02X -> %s%s%s\n",
                  data,
                  okColor(ack),
                  ack ? "ACK" : "NACK",
                  LOG_COLOR_RESET);
  }
  
  return ack;
}

/// Read a byte, send ACK/NACK
inline uint8_t readByteRaw(const EE871::Config& cfg, bool sendAck, bool verbose = false) {
  uint8_t data = 0;
  
  cfg.setSda(true, cfg.busUser);  // Release SDA
  
  // Read 8 bits MSB first
  for (int i = 7; i >= 0; i--) {
    cfg.setScl(true, cfg.busUser);
    cfg.delayUs(cfg.clockHighUs / 2, cfg.busUser);
    bool bit = cfg.readSda(cfg.busUser);
    if (bit) data |= (1 << i);
    cfg.delayUs(cfg.clockHighUs / 2, cfg.busUser);
    cfg.setScl(false, cfg.busUser);
    cfg.delayUs(cfg.clockLowUs, cfg.busUser);
  }
  
  // Send ACK/NACK
  cfg.setSda(!sendAck, cfg.busUser);  // ACK = SDA low
  cfg.delayUs(10, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(cfg.clockLowUs, cfg.busUser);
  cfg.setSda(true, cfg.busUser);  // Release
  
  if (verbose) {
    Serial.printf("  Read 0x%02X, sent %s%s%s\n",
                  data,
                  sendAck ? LOG_COLOR_GREEN : LOG_COLOR_YELLOW,
                  sendAck ? "ACK" : "NACK",
                  LOG_COLOR_RESET);
  }
  
  return data;
}

// ============================================================================
// Address Scanner
// ============================================================================

/// Scan all 8 possible E2 device addresses
inline void scanAddresses(const EE871::Config& cfg) {
  Serial.printf("%s=== E2 Address Scanner ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  Serial.println("Scanning addresses 0-7 with status read (0x7x)...\n");
  
  int found = 0;
  
  for (uint8_t addr = 0; addr < 8; addr++) {
    // Status read: main=0x7, addr, read=1
    uint8_t ctrlByte = EE871::cmd::makeControlRead(EE871::cmd::MAIN_STATUS, addr);
    
    sendStart(cfg);
    bool ack = sendByteRaw(cfg, ctrlByte, false);
    
    if (ack) {
      // Read data byte
      uint8_t data = readByteRaw(cfg, true, false);  // ACK
      uint8_t pec = readByteRaw(cfg, false, false);  // NACK
      sendStop(cfg);
      
      uint8_t expectedPec = (ctrlByte + data) & 0xFF;
      bool pecOk = (pec == expectedPec);
      
      Serial.printf("  Address %d: %sFOUND%s Status=0x%02X, PEC=%s%s%s\n", 
                    addr,
                    LOG_COLOR_GREEN,
                    LOG_COLOR_RESET,
                    data,
                    okColor(pecOk),
                    pecOk ? "OK" : "MISMATCH",
                    LOG_COLOR_RESET);
      found++;
    } else {
      sendStop(cfg);
      Serial.printf("  Address %d: %sNo response (NACK)%s\n", addr, neutralColor(), LOG_COLOR_RESET);
    }
    
    delay(10);  // Small gap between attempts
  }
  
  Serial.printf("\nFound %s%d%s device(s)\n", okColor(found > 0), found, LOG_COLOR_RESET);
}

// ============================================================================
// Timing/Frequency Discovery
// ============================================================================

struct TimingResult {
  uint16_t clockUs;
  bool gotAck;
  uint8_t data;
  bool pecOk;
};

/// Try a single timing and return result
inline TimingResult tryTiming(const EE871::Config& cfg, uint16_t clockUs) {
  TimingResult result = {clockUs, false, 0, false};
  
  // Create modified config with different timing
  EE871::Config testCfg = cfg;
  testCfg.clockLowUs = clockUs;
  testCfg.clockHighUs = clockUs;
  
  // Status read at address 0
  uint8_t ctrlByte = EE871::cmd::makeControlRead(EE871::cmd::MAIN_STATUS, 0);
  
  sendStart(testCfg);
  result.gotAck = sendByteRaw(testCfg, ctrlByte, false);
  
  if (result.gotAck) {
    result.data = readByteRaw(testCfg, true, false);
    uint8_t pec = readByteRaw(testCfg, false, false);
    uint8_t expectedPec = (ctrlByte + result.data) & 0xFF;
    result.pecOk = (pec == expectedPec);
  }
  
  sendStop(testCfg);
  return result;
}

/// Discover working timing/frequency
inline void discoverTiming(const EE871::Config& cfg) {
  Serial.printf("%s=== Timing Discovery ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  Serial.println("Testing different clock periods...\n");
  Serial.println("E2 spec: 100-1000us (500-5000 Hz)\n");
  
  // Test various timings from slow to fast
  uint16_t timings[] = {1000, 500, 250, 200, 150, 100, 75, 50};
  int numTimings = sizeof(timings) / sizeof(timings[0]);
  
  int foundCount = 0;
  
  for (int i = 0; i < numTimings; i++) {
    uint16_t clockUs = timings[i];
    float freqHz = 1000000.0f / (2 * clockUs);
    
    auto result = tryTiming(cfg, clockUs);
    
    Serial.printf("  %4u us (%5.0f Hz): ", clockUs, freqHz);
    
    if (result.gotAck) {
      Serial.printf("%sACK%s, data=0x%02X, PEC=%s%s%s\n",
                    LOG_COLOR_GREEN,
                    LOG_COLOR_RESET,
                    result.data,
                    okColor(result.pecOk),
                    result.pecOk ? "OK" : "BAD",
                    LOG_COLOR_RESET);
      foundCount++;
    } else {
      Serial.printf("%sNACK%s\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    
    delay(50);  // Gap between tests
  }
  
  Serial.printf("\n%s%d%s timing(s) worked\n", okColor(foundCount > 0), foundCount, LOG_COLOR_RESET);
  
  if (foundCount == 0) {
    Serial.printf("\n%sNo timing worked%s. Possible issues:\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    Serial.println("  - Device not connected or powered");
    Serial.println("  - Wrong pins configured");
    Serial.println("  - Missing/wrong pull-ups");
    Serial.println("  - Bus voltage mismatch (need level shifter?)");
    Serial.println("  - Device address not 0");
  }
}

// ============================================================================
// Generate 9 Clock Pulses (Bus Recovery)
// ============================================================================

/// Send 9 clock pulses to recover stuck bus
inline void sendRecoveryClocks(const EE871::Config& cfg) {
  Serial.printf("%s=== Bus Recovery (9 clocks) ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  
  cfg.setSda(true, cfg.busUser);  // Release SDA
  
  for (int i = 0; i < 9; i++) {
    cfg.setScl(false, cfg.busUser);
    cfg.delayUs(cfg.clockLowUs, cfg.busUser);
    cfg.setScl(true, cfg.busUser);
    cfg.delayUs(cfg.clockHighUs, cfg.busUser);
    
    if (cfg.readSda(cfg.busUser)) {
      Serial.printf("  SDA released after %d clock(s)\n", i + 1);
    }
  }
  
  // Send STOP
  sendStop(cfg);
  
  auto lvl = readBusLevels(cfg);
  Serial.printf("  Final: SCL=%s%s%s SDA=%s%s%s\n",
                okColor(lvl.scl),
                lvl.scl ? "HIGH" : "LOW",
                LOG_COLOR_RESET,
                okColor(lvl.sda),
                lvl.sda ? "HIGH" : "LOW",
                LOG_COLOR_RESET);
}

// ============================================================================
// Deep Protocol Test
// ============================================================================

/// Verbose single transaction test
inline void testTransaction(const EE871::Config& cfg, uint8_t ctrlByte) {
  Serial.printf("%s=== Transaction Test ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  Serial.printf("Control byte: 0x%02X\n", ctrlByte);
  Serial.printf("  MainCmd: 0x%X, Addr: %d, R/W: %s\n",
                (ctrlByte >> 4) & 0x0F,
                (ctrlByte >> 1) & 0x07,
                (ctrlByte & 1) ? "READ" : "WRITE");
  Serial.println();
  
  // Check bus before
  auto lvlBefore = readBusLevels(cfg);
  Serial.printf("Bus before: SCL=%s SDA=%s\n", 
                lvlBefore.scl ? "H" : "L", 
                lvlBefore.sda ? "H" : "L");
  
  if (!lvlBefore.scl || !lvlBefore.sda) {
    Serial.printf("%sERROR%s: Bus not idle, aborting\n\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    return;
  }
  
  // START
  Serial.println("\n1. Sending START...");
  sendStart(cfg);
  auto lvlAfterStart = readBusLevels(cfg);
  Serial.printf("   After START: SCL=%s SDA=%s\n",
                lvlAfterStart.scl ? "H" : "L",
                lvlAfterStart.sda ? "H" : "L");
  
  // Control byte
  Serial.printf("\n2. Sending control byte 0x%02X...\n", ctrlByte);
  bool ack = sendByteRaw(cfg, ctrlByte, true);
  
  if (!ack) {
    Serial.printf("   %sNACK%s received - device not responding\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    sendStop(cfg);
    Serial.println("\n3. Sent STOP\n");
    return;
  }
  
  // For read commands, read data + PEC
  if (ctrlByte & 0x01) {
    Serial.println("\n3. Reading data byte...");
    uint8_t data = readByteRaw(cfg, true, true);
    
    Serial.println("\n4. Reading PEC...");
    uint8_t pec = readByteRaw(cfg, false, true);
    
    uint8_t expectedPec = (ctrlByte + data) & 0xFF;
    const bool pecOk = (pec == expectedPec);
    Serial.printf("\n5. PEC check: received=0x%02X, expected=0x%02X -> %s%s%s\n",
                  pec,
                  expectedPec,
                  okColor(pecOk),
                  pecOk ? "OK" : "MISMATCH",
                  LOG_COLOR_RESET);
  }
  
  sendStop(cfg);
  Serial.println("\n6. Sent STOP\n");
  
  auto lvlAfter = readBusLevels(cfg);
  Serial.printf("Bus after: SCL=%s SDA=%s\n", 
                lvlAfter.scl ? "H" : "L", 
                lvlAfter.sda ? "H" : "L");
}

// ============================================================================
// Test All Library Commands
// ============================================================================

/// Test the exact commands that begin() uses
inline void testLibraryCommands(const EE871::Config& cfg) {
  Serial.printf("%s=== Library Command Test ===%s\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  Serial.println("Testing exact commands used by begin()...\n");
  
  struct CmdTest {
    uint8_t mainCmd;
    const char* name;
  };
  
  CmdTest tests[] = {
    {EE871::cmd::MAIN_TYPE_LO, "TYPE_LO (0x11)"},    // Group low
    {EE871::cmd::MAIN_TYPE_HI, "TYPE_HI (0x41)"},    // Group high
    {EE871::cmd::MAIN_TYPE_SUB, "TYPE_SUB (0x21)"},  // Subgroup
    {EE871::cmd::MAIN_AVAIL_MEAS, "AVAIL (0x31)"},   // Available measurements
    {EE871::cmd::MAIN_STATUS, "STATUS (0x71)"},     // Status
    {EE871::cmd::MAIN_MV3_LO, "MV3_LO (0xC1)"},     // CO2 fast low
    {EE871::cmd::MAIN_MV3_HI, "MV3_HI (0xD1)"},     // CO2 fast high
    {EE871::cmd::MAIN_MV4_LO, "MV4_LO (0xE1)"},     // CO2 avg low
    {EE871::cmd::MAIN_MV4_HI, "MV4_HI (0xF1)"},     // CO2 avg high
  };
  
  int numTests = sizeof(tests) / sizeof(tests[0]);
  int passed = 0;
  
  for (int i = 0; i < numTests; i++) {
    uint8_t ctrlByte = EE871::cmd::makeControlRead(tests[i].mainCmd, cfg.deviceAddress);
    
    Serial.printf("%-18s [0x%02X]: ", tests[i].name, ctrlByte);
    
    sendStart(cfg);
    bool ack = sendByteRaw(cfg, ctrlByte, false);
    
    if (ack) {
      uint8_t data = readByteRaw(cfg, true, false);
      uint8_t pec = readByteRaw(cfg, false, false);
      sendStop(cfg);
      
      uint8_t expectedPec = (ctrlByte + data) & 0xFF;
      bool pecOk = (pec == expectedPec);
      
      Serial.printf("%sACK%s data=0x%02X PEC=%s%s%s\n",
                    LOG_COLOR_GREEN,
                    LOG_COLOR_RESET,
                    data,
                    okColor(pecOk),
                    pecOk ? "OK" : "BAD",
                    LOG_COLOR_RESET);
      if (pecOk) passed++;
    } else {
      sendStop(cfg);
      Serial.printf("%sNACK%s\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    }
    
    delay(20);
  }
  
  Serial.printf("\nPassed: %s%d%s/%d\n",
                okColor(passed == numTests),
                passed,
                LOG_COLOR_RESET,
                numTests);
  
  if (passed == 0) {
    Serial.printf("\n%sAll commands failed!%s Check:\n", LOG_COLOR_RED, LOG_COLOR_RESET);
    Serial.println("  - Is device address 0? (default)");
    Serial.println("  - Datasheet command compatibility");
  } else if (passed < numTests) {
    Serial.printf("\n%sSome commands failed%s - may be normal depending on device state\n",
                  LOG_COLOR_YELLOW, LOG_COLOR_RESET);
  }
}

// ============================================================================
// Full Diagnostic Suite
// ============================================================================

inline void runFullDiagnostics(const EE871::Config& cfg) {
  Serial.println("\n");
  Serial.printf("%s=== FULL E2 BUS DIAGNOSTICS ===%s\n\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
  
  Serial.printf("Config: DATA=GPIO%d, CLOCK=GPIO%d\n", 
                static_cast<transport::E2Pins*>(cfg.busUser)->sda,
                static_cast<transport::E2Pins*>(cfg.busUser)->scl);
  Serial.printf("Timing: LOW=%u us, HIGH=%u us (%.0f Hz)\n\n",
                cfg.clockLowUs, cfg.clockHighUs,
                1000000.0f / (cfg.clockLowUs + cfg.clockHighUs));
  
  Serial.printf("%s[Step 1] Bus Levels%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  printBusLevels(cfg);
  Serial.println();
  
  Serial.printf("%s[Step 2] Pin Toggle Test%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  testPinToggle(cfg);
  Serial.println();
  
  Serial.printf("%s[Step 3] Clock Pulse Test%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  testClockPulses(cfg, 5);
  Serial.println();
  
  Serial.printf("%s[Step 4] Bus Sniff (1s)%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  sniffAndPrint(cfg, 1000);
  Serial.println();
  
  Serial.printf("%s[Step 5] Address Scan%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  scanAddresses(cfg);
  Serial.println();
  
  Serial.printf("%s[Step 6] Timing Discovery%s\n", LOG_COLOR_GREEN, LOG_COLOR_RESET);
  discoverTiming(cfg);
  Serial.println();
  
  Serial.printf("%s=== DIAGNOSTICS COMPLETE ===%s\n\n", LOG_COLOR_CYAN, LOG_COLOR_RESET);
}

// ============================================================================
// Background Bus Sniffer (hooks into transport layer) - Protocol Decoder
// ============================================================================

/// Sniffer state - global for callback access
struct SnifferState {
  bool active = false;
  bool lastScl = true;
  bool lastSda = true;
  
  // Protocol state
  enum class State { IDLE, RECEIVING_BYTE, WAITING_ACK } state = State::IDLE;
  uint8_t currentByte = 0;
  uint8_t bitCount = 0;
  bool isFirstByte = true;  // First byte after START is control byte
  bool isReadMode = false;
  uint8_t byteIndex = 0;    // Byte number in transaction
  
  // For value tracking
  uint8_t lastMainCmd = 0;
  uint8_t lastDataByte = 0;
  uint8_t pendingLowByte = 0;
  bool haveLowByte = false;
  uint8_t lowByteCmd = 0;
  
  uint32_t transitions = 0;
  uint32_t startMs = 0;
};

inline SnifferState& snifferState() {
  static SnifferState state;
  return state;
}

/// Decode control byte for display
inline const char* getCmdName(uint8_t mainCmd, bool read) {
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

/// Check if this is a low byte of a 16-bit value
inline bool isLowByteCmd(uint8_t mainCmd) {
  return mainCmd == 0xC || mainCmd == 0xE || mainCmd == 0x1;
}

/// Check if this is the matching high byte
inline bool isMatchingHighByte(uint8_t lowCmd, uint8_t highCmd) {
  if (lowCmd == 0xC && highCmd == 0xD) return true;  // MV3 (CO2 fast)
  if (lowCmd == 0xE && highCmd == 0xF) return true;  // MV4 (CO2 avg)
  if (lowCmd == 0x1 && highCmd == 0x4) return true;  // Type (group ID)
  return false;
}

/// Callback invoked by transport on every line change
inline void snifferCallback(bool scl, bool sda) {
  auto& s = snifferState();
  if (!s.active) return;
  
  // Detect START: SDA falls while SCL high
  if (s.lastScl && scl && s.lastSda && !sda) {
    Serial.print("\n>START ");
    s.state = SnifferState::State::RECEIVING_BYTE;
    s.currentByte = 0;
    s.bitCount = 0;
    s.isFirstByte = true;
    s.byteIndex = 0;
    s.transitions++;
  }
  // Detect STOP: SDA rises while SCL high  
  else if (s.lastScl && scl && !s.lastSda && sda) {
    Serial.println(" STOP");
    s.state = SnifferState::State::IDLE;
    s.transitions++;
  }
  // SCL rising edge - sample data bit
  else if (!s.lastScl && scl) {
    s.transitions++;
    
    if (s.state == SnifferState::State::RECEIVING_BYTE) {
      // Shift in data bit (MSB first)
      s.currentByte = (s.currentByte << 1) | (sda ? 1 : 0);
      s.bitCount++;
      
      if (s.bitCount == 8) {
        // Byte complete, next clock is ACK
        s.state = SnifferState::State::WAITING_ACK;
      }
    }
    else if (s.state == SnifferState::State::WAITING_ACK) {
      bool ack = !sda;  // ACK = SDA low
      
      // Print the byte
      if (s.isFirstByte) {
        // Control byte
        s.isReadMode = (s.currentByte & 0x01);
        s.lastMainCmd = (s.currentByte >> 4) & 0x0F;
        uint8_t addr = (s.currentByte >> 1) & 0x07;
        
        Serial.printf("[0x%02X %s a%d %s]", 
                      s.currentByte,
                      getCmdName(s.lastMainCmd, s.isReadMode),
                      addr,
                      ack ? "ACK" : "NAK");
        s.isFirstByte = false;
      } else {
        // Data byte - check if it's data or PEC
        if (s.byteIndex == 1) {
          // First data byte
          s.lastDataByte = s.currentByte;
          Serial.printf(" data=0x%02X(%u)", s.currentByte, s.currentByte);
          
          // Track for 16-bit assembly
          if (s.isReadMode && isLowByteCmd(s.lastMainCmd)) {
            s.pendingLowByte = s.currentByte;
            s.haveLowByte = true;
            s.lowByteCmd = s.lastMainCmd;
          } else if (s.isReadMode && s.haveLowByte && isMatchingHighByte(s.lowByteCmd, s.lastMainCmd)) {
            // We have both bytes - show combined value
            uint16_t value = (static_cast<uint16_t>(s.currentByte) << 8) | s.pendingLowByte;
            Serial.printf(" => %u", value);
            if (s.lowByteCmd == 0xC || s.lowByteCmd == 0xE) {
              Serial.print(" ppm");
            }
            s.haveLowByte = false;
          }
        } else if (s.byteIndex == 2) {
          // PEC byte
          Serial.printf(" pec=0x%02X", s.currentByte);
        }
      }
      
      s.byteIndex++;
      
      // Reset for next byte
      s.currentByte = 0;
      s.bitCount = 0;
      s.state = SnifferState::State::RECEIVING_BYTE;
    }
  }
  
  s.lastScl = scl;
  s.lastSda = sda;
}

/// Sniffer control class
class BusSniffer {
public:
  void start(const EE871::Config* cfg) {
    auto& s = snifferState();
    s.lastScl = cfg->readScl(cfg->busUser);
    s.lastSda = cfg->readSda(cfg->busUser);
    s.transitions = 0;
    s.startMs = millis();
    s.state = SnifferState::State::IDLE;
    s.currentByte = 0;
    s.bitCount = 0;
    s.isFirstByte = true;
    s.haveLowByte = false;
    s.active = true;
    
    // Register callback with transport
    transport::setSnifferCallback(snifferCallback);
    
    Serial.println("[SNIFF] ON - 'sniff 0' to stop");
  }
  
  void stop() {
    auto& s = snifferState();
    if (s.active) {
      s.active = false;
      transport::setSnifferCallback(nullptr);
      uint32_t elapsed = millis() - s.startMs;
      Serial.printf("\n[SNIFF] OFF (%lu ms, %lu edges)\n",
                    static_cast<unsigned long>(elapsed),
                    static_cast<unsigned long>(s.transitions));
    }
  }
  
  bool isActive() const { return snifferState().active; }
  
  void tick() {}
};

/// Global sniffer instance for use in examples
inline BusSniffer& sniffer() {
  static BusSniffer instance;
  return instance;
}

} // namespace e2diag
