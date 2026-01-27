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

// ============================================================================
// Status Display
// ============================================================================

inline void printStatus(uint8_t status) {
  Serial.printf("Status: 0x%02X", status);
  if (status & ee871::cmd::STATUS_CO2_ERROR_MASK) {
    Serial.print(" (CO2 error)");
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
inline BusLevels readBusLevels(const ee871::Config& cfg) {
  BusLevels lvl;
  lvl.scl = cfg.readScl(cfg.busUser);
  lvl.sda = cfg.readSda(cfg.busUser);
  return lvl;
}

/// Print current bus levels
inline void printBusLevels(const ee871::Config& cfg) {
  auto lvl = readBusLevels(cfg);
  Serial.println("=== Bus Levels ===");
  Serial.printf("  SCL: %s\n", lvl.scl ? "HIGH (idle)" : "LOW (held)");
  Serial.printf("  SDA: %s\n", lvl.sda ? "HIGH (idle)" : "LOW (held)");
  
  if (!lvl.scl && !lvl.sda) {
    Serial.println("  WARNING: Both lines LOW - bus stuck or no pull-ups!");
  } else if (!lvl.scl) {
    Serial.println("  WARNING: SCL held LOW - clock stretching or stuck!");
  } else if (!lvl.sda) {
    Serial.println("  WARNING: SDA held LOW - slave holding or stuck!");
  } else {
    Serial.println("  OK: Bus idle (both HIGH)");
  }
}

// ============================================================================
// Pin Toggle Test
// ============================================================================

/// Test if we can toggle pins and read them back
inline void testPinToggle(const ee871::Config& cfg) {
  Serial.println("=== Pin Toggle Test ===");
  Serial.println("Testing if MCU can control and read bus lines...\n");

  // Test SCL
  Serial.println("SCL pin test:");
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  bool sclHigh = cfg.readScl(cfg.busUser);
  Serial.printf("  Set HIGH -> Read: %s\n", sclHigh ? "HIGH (OK)" : "LOW (FAIL - stuck or no pull-up)");

  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  bool sclLow = cfg.readScl(cfg.busUser);
  Serial.printf("  Set LOW  -> Read: %s\n", sclLow ? "HIGH (FAIL - can't pull low)" : "LOW (OK)");

  cfg.setScl(true, cfg.busUser);  // Release
  cfg.delayUs(100, cfg.busUser);

  // Test SDA
  Serial.println("\nSDA pin test:");
  cfg.setSda(true, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  bool sdaHigh = cfg.readSda(cfg.busUser);
  Serial.printf("  Set HIGH -> Read: %s\n", sdaHigh ? "HIGH (OK)" : "LOW (FAIL - stuck or no pull-up)");

  cfg.setSda(false, cfg.busUser);
  cfg.delayUs(100, cfg.busUser);
  bool sdaLow = cfg.readSda(cfg.busUser);
  Serial.printf("  Set LOW  -> Read: %s\n", sdaLow ? "HIGH (FAIL - can't pull low)" : "LOW (OK)");

  cfg.setSda(true, cfg.busUser);  // Release
  cfg.delayUs(100, cfg.busUser);

  // Summary
  Serial.println("\nSummary:");
  bool sclOk = sclHigh && !sclLow;
  bool sdaOk = sdaHigh && !sdaLow;
  
  if (sclOk && sdaOk) {
    Serial.println("  PASS: Both pins working correctly");
  } else {
    if (!sclHigh) Serial.println("  FAIL: SCL has no pull-up or is stuck LOW");
    if (sclLow)   Serial.println("  FAIL: SCL cannot be pulled LOW by MCU");
    if (!sdaHigh) Serial.println("  FAIL: SDA has no pull-up or is stuck LOW");
    if (sdaLow)   Serial.println("  FAIL: SDA cannot be pulled LOW by MCU");
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
inline SniffResult sniffBus(const ee871::Config& cfg, uint32_t durationMs) {
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
inline void sniffAndPrint(const ee871::Config& cfg, uint32_t durationMs = 2000) {
  Serial.printf("=== Bus Sniffer (%lu ms) ===\n", durationMs);
  Serial.println("Monitoring bus activity...");
  
  auto result = sniffBus(cfg, durationMs);
  
  Serial.printf("\nResults over %lu ms:\n", result.durationMs);
  Serial.printf("  SCL transitions: %lu\n", result.sclTransitions);
  Serial.printf("  SDA transitions: %lu\n", result.sdaTransitions);
  
  if (result.sclTransitions > 0) {
    float sclFreq = (result.sclTransitions / 2.0f) / (result.durationMs / 1000.0f);
    Serial.printf("  SCL approx freq: %.1f Hz\n", sclFreq);
  }
  
  if (result.sclStuckLow) {
    Serial.println("  WARNING: SCL was stuck LOW for >100ms");
  }
  if (result.sdaStuckLow) {
    Serial.println("  WARNING: SDA was stuck LOW for >100ms");
  }
  
  if (result.sclTransitions == 0 && result.sdaTransitions == 0) {
    Serial.println("\n  No bus activity detected - bus is quiet");
  }
}

// ============================================================================
// Clock Pulse Test
// ============================================================================

/// Generate clock pulses and verify
inline void testClockPulses(const ee871::Config& cfg, int numPulses = 10) {
  Serial.printf("=== Clock Pulse Test (%d pulses) ===\n", numPulses);
  
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
    
    Serial.printf("  Pulse %2d: LOW=%s HIGH=%s\n", 
                  i + 1, 
                  readLow ? "FAIL" : "ok",
                  readHigh ? "ok" : "FAIL(stretched?)");
  }
  
  Serial.printf("\nResults: %d/%d LOW ok, %d/%d HIGH ok\n", 
                successLow, numPulses, successHigh, numPulses);
  
  if (successHigh < numPulses) {
    Serial.println("  NOTE: HIGH failures may indicate clock stretching by slave");
  }
}

// ============================================================================
// Raw Bit-Bang Functions
// ============================================================================

/// Send START condition
inline void sendStart(const ee871::Config& cfg) {
  cfg.setSda(true, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
  cfg.setSda(false, cfg.busUser);  // SDA high->low while SCL high
  cfg.delayUs(10, cfg.busUser);
  cfg.setScl(false, cfg.busUser);
  cfg.delayUs(cfg.clockLowUs, cfg.busUser);
}

/// Send STOP condition
inline void sendStop(const ee871::Config& cfg) {
  cfg.setSda(false, cfg.busUser);
  cfg.setScl(true, cfg.busUser);
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
  cfg.setSda(true, cfg.busUser);  // SDA low->high while SCL high
  cfg.delayUs(cfg.clockHighUs, cfg.busUser);
}

/// Send a byte, return true if ACK received
inline bool sendByteRaw(const ee871::Config& cfg, uint8_t data, bool verbose = false) {
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
    Serial.printf("  Sent 0x%02X -> %s\n", data, ack ? "ACK" : "NACK");
  }
  
  return ack;
}

/// Read a byte, send ACK/NACK
inline uint8_t readByteRaw(const ee871::Config& cfg, bool sendAck, bool verbose = false) {
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
    Serial.printf("  Read 0x%02X, sent %s\n", data, sendAck ? "ACK" : "NACK");
  }
  
  return data;
}

// ============================================================================
// Address Scanner
// ============================================================================

/// Scan all 8 possible E2 device addresses
inline void scanAddresses(const ee871::Config& cfg) {
  Serial.println("=== E2 Address Scanner ===");
  Serial.println("Scanning addresses 0-7 with status read (0x7x)...\n");
  
  int found = 0;
  
  for (uint8_t addr = 0; addr < 8; addr++) {
    // Status read: main=0x7, addr, read=1
    uint8_t ctrlByte = ee871::cmd::makeControlRead(ee871::cmd::MAIN_STATUS, addr);
    
    sendStart(cfg);
    bool ack = sendByteRaw(cfg, ctrlByte, false);
    
    if (ack) {
      // Read data byte
      uint8_t data = readByteRaw(cfg, true, false);  // ACK
      uint8_t pec = readByteRaw(cfg, false, false);  // NACK
      sendStop(cfg);
      
      uint8_t expectedPec = (ctrlByte + data) & 0xFF;
      bool pecOk = (pec == expectedPec);
      
      Serial.printf("  Address %d: FOUND! Status=0x%02X, PEC=%s\n", 
                    addr, data, pecOk ? "OK" : "MISMATCH");
      found++;
    } else {
      sendStop(cfg);
      Serial.printf("  Address %d: No response (NACK)\n", addr);
    }
    
    delay(10);  // Small gap between attempts
  }
  
  Serial.printf("\nFound %d device(s)\n", found);
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
inline TimingResult tryTiming(const ee871::Config& cfg, uint16_t clockUs) {
  TimingResult result = {clockUs, false, 0, false};
  
  // Create modified config with different timing
  ee871::Config testCfg = cfg;
  testCfg.clockLowUs = clockUs;
  testCfg.clockHighUs = clockUs;
  
  // Status read at address 0
  uint8_t ctrlByte = ee871::cmd::makeControlRead(ee871::cmd::MAIN_STATUS, 0);
  
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
inline void discoverTiming(const ee871::Config& cfg) {
  Serial.println("=== Timing Discovery ===");
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
      Serial.printf("ACK, data=0x%02X, PEC=%s\n", 
                    result.data, result.pecOk ? "OK" : "BAD");
      foundCount++;
    } else {
      Serial.println("NACK");
    }
    
    delay(50);  // Gap between tests
  }
  
  Serial.printf("\n%d timing(s) worked\n", foundCount);
  
  if (foundCount == 0) {
    Serial.println("\nNo timing worked. Possible issues:");
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
inline void sendRecoveryClocks(const ee871::Config& cfg) {
  Serial.println("=== Bus Recovery (9 clocks) ===");
  
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
  Serial.printf("  Final: SCL=%s SDA=%s\n", 
                lvl.scl ? "HIGH" : "LOW", 
                lvl.sda ? "HIGH" : "LOW");
}

// ============================================================================
// Deep Protocol Test
// ============================================================================

/// Verbose single transaction test
inline void testTransaction(const ee871::Config& cfg, uint8_t ctrlByte) {
  Serial.println("=== Transaction Test ===");
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
    Serial.println("ERROR: Bus not idle, aborting\n");
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
    Serial.println("   NACK received - device not responding");
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
    Serial.printf("\n5. PEC check: received=0x%02X, expected=0x%02X -> %s\n",
                  pec, expectedPec, (pec == expectedPec) ? "OK" : "MISMATCH");
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
inline void testLibraryCommands(const ee871::Config& cfg) {
  Serial.println("=== Library Command Test ===");
  Serial.println("Testing exact commands used by begin()...\n");
  
  struct CmdTest {
    uint8_t mainCmd;
    const char* name;
  };
  
  CmdTest tests[] = {
    {ee871::cmd::MAIN_TYPE_LO, "TYPE_LO (0x11)"},    // Group low
    {ee871::cmd::MAIN_TYPE_HI, "TYPE_HI (0x41)"},    // Group high
    {ee871::cmd::MAIN_TYPE_SUB, "TYPE_SUB (0x21)"},  // Subgroup
    {ee871::cmd::MAIN_AVAIL_MEAS, "AVAIL (0x31)"},   // Available measurements
    {ee871::cmd::MAIN_STATUS, "STATUS (0x71)"},     // Status
    {ee871::cmd::MAIN_MV3_LO, "MV3_LO (0xC1)"},     // CO2 fast low
    {ee871::cmd::MAIN_MV3_HI, "MV3_HI (0xD1)"},     // CO2 fast high
    {ee871::cmd::MAIN_MV4_LO, "MV4_LO (0xE1)"},     // CO2 avg low
    {ee871::cmd::MAIN_MV4_HI, "MV4_HI (0xF1)"},     // CO2 avg high
  };
  
  int numTests = sizeof(tests) / sizeof(tests[0]);
  int passed = 0;
  
  for (int i = 0; i < numTests; i++) {
    uint8_t ctrlByte = ee871::cmd::makeControlRead(tests[i].mainCmd, cfg.deviceAddress);
    
    Serial.printf("%-18s [0x%02X]: ", tests[i].name, ctrlByte);
    
    sendStart(cfg);
    bool ack = sendByteRaw(cfg, ctrlByte, false);
    
    if (ack) {
      uint8_t data = readByteRaw(cfg, true, false);
      uint8_t pec = readByteRaw(cfg, false, false);
      sendStop(cfg);
      
      uint8_t expectedPec = (ctrlByte + data) & 0xFF;
      bool pecOk = (pec == expectedPec);
      
      Serial.printf("ACK data=0x%02X PEC=%s\n", data, pecOk ? "OK" : "BAD");
      if (pecOk) passed++;
    } else {
      sendStop(cfg);
      Serial.println("NACK");
    }
    
    delay(20);
  }
  
  Serial.printf("\nPassed: %d/%d\n", passed, numTests);
  
  if (passed == 0) {
    Serial.println("\nAll commands failed! Check:");
    Serial.println("  - Is device address 0? (default)");
    Serial.println("  - Datasheet command compatibility");
  } else if (passed < numTests) {
    Serial.println("\nSome commands failed - may be normal depending on device state");
  }
}

// ============================================================================
// Full Diagnostic Suite
// ============================================================================

inline void runFullDiagnostics(const ee871::Config& cfg) {
  Serial.println("\n");
  Serial.println("##############################################");
  Serial.println("#         FULL E2 BUS DIAGNOSTICS            #");
  Serial.println("##############################################\n");
  
  Serial.printf("Config: DATA=GPIO%d, CLOCK=GPIO%d\n", 
                static_cast<transport::E2Pins*>(cfg.busUser)->sda,
                static_cast<transport::E2Pins*>(cfg.busUser)->scl);
  Serial.printf("Timing: LOW=%u us, HIGH=%u us (%.0f Hz)\n\n",
                cfg.clockLowUs, cfg.clockHighUs,
                1000000.0f / (cfg.clockLowUs + cfg.clockHighUs));
  
  Serial.println("--- Step 1: Bus Levels ---");
  printBusLevels(cfg);
  Serial.println();
  
  Serial.println("--- Step 2: Pin Toggle Test ---");
  testPinToggle(cfg);
  Serial.println();
  
  Serial.println("--- Step 3: Clock Pulse Test ---");
  testClockPulses(cfg, 5);
  Serial.println();
  
  Serial.println("--- Step 4: Bus Sniff (1s) ---");
  sniffAndPrint(cfg, 1000);
  Serial.println();
  
  Serial.println("--- Step 5: Address Scan ---");
  scanAddresses(cfg);
  Serial.println();
  
  Serial.println("--- Step 6: Timing Discovery ---");
  discoverTiming(cfg);
  Serial.println();
  
  Serial.println("##############################################");
  Serial.println("#           DIAGNOSTICS COMPLETE             #");
  Serial.println("##############################################\n");
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
  
  uint32_t transitions = 0;
  uint32_t startMs = 0;
};

inline SnifferState& snifferState() {
  static SnifferState state;
  return state;
}

/// Decode control byte for display
inline void decodeControlByte(uint8_t ctrl) {
  uint8_t mainCmd = (ctrl >> 4) & 0x0F;
  uint8_t addr = (ctrl >> 1) & 0x07;
  bool read = ctrl & 0x01;
  
  const char* cmdName = "???";
  switch (mainCmd) {
    case 0x1: cmdName = read ? "TYPE_LO" : "CUST_WR"; break;
    case 0x2: cmdName = "SUBGRP"; break;
    case 0x3: cmdName = "AVAIL"; break;
    case 0x4: cmdName = "TYPE_HI"; break;
    case 0x5: cmdName = read ? "CUST_RD" : "CUST_PTR"; break;
    case 0x7: cmdName = "STATUS"; break;
    case 0xC: cmdName = "MV3_LO"; break;
    case 0xD: cmdName = "MV3_HI"; break;
    case 0xE: cmdName = "MV4_LO"; break;
    case 0xF: cmdName = "MV4_HI"; break;
  }
  Serial.printf(" [%s addr=%d %s]", cmdName, addr, read ? "R" : "W");
}

/// Callback invoked by transport on every line change
inline void snifferCallback(bool scl, bool sda) {
  auto& s = snifferState();
  if (!s.active) return;
  
  // Detect START: SDA falls while SCL high
  if (s.lastScl && scl && s.lastSda && !sda) {
    Serial.print("\n[SNIFF] START");
    s.state = SnifferState::State::RECEIVING_BYTE;
    s.currentByte = 0;
    s.bitCount = 0;
    s.isFirstByte = true;
    s.byteIndex = 0;
    s.transitions++;
  }
  // Detect STOP: SDA rises while SCL high  
  else if (s.lastScl && scl && !s.lastSda && sda) {
    Serial.println("\n[SNIFF] STOP");
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
        Serial.printf("\n[SNIFF] TX: 0x%02X %s", s.currentByte, ack ? "ACK" : "NAK");
        decodeControlByte(s.currentByte);
        s.isFirstByte = false;
      } else {
        // Data byte
        const char* dir = s.isReadMode ? "RX" : "TX";
        Serial.printf("\n[SNIFF] %s: 0x%02X %s", dir, s.currentByte, ack ? "ACK" : "NAK");
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
  void start(const ee871::Config* cfg) {
    auto& s = snifferState();
    s.lastScl = cfg->readScl(cfg->busUser);
    s.lastSda = cfg->readSda(cfg->busUser);
    s.transitions = 0;
    s.startMs = millis();
    s.state = SnifferState::State::IDLE;
    s.currentByte = 0;
    s.bitCount = 0;
    s.isFirstByte = true;
    s.active = true;
    
    // Register callback with transport
    transport::setSnifferCallback(snifferCallback);
    
    Serial.println("[SNIFF] Started - type 'sniff 0' to stop");
  }
  
  void stop() {
    auto& s = snifferState();
    if (s.active) {
      s.active = false;
      transport::setSnifferCallback(nullptr);
      uint32_t elapsed = millis() - s.startMs;
      Serial.printf("\n[SNIFF] Stopped after %lu ms, %lu transitions\n", elapsed, s.transitions);
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
