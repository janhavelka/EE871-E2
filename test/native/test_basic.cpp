/// @file test_basic.cpp
/// @brief Basic unit tests for EE871 driver

#include <cstdio>
#include <cassert>

// Include stubs first
#include "Arduino.h"
#include "Wire.h"

// Stub implementations
SerialClass Serial;
TwoWire Wire;

// Include driver
#include "EE871/Status.h"
#include "EE871/Config.h"

using namespace ee871;

// ============================================================================
// Test Helpers
// ============================================================================

static int testsPassed = 0;
static int testsFailed = 0;

#define TEST(name) void test_##name()
#define RUN_TEST(name) do { \
  printf("Running %s... ", #name); \
  test_##name(); \
  printf("PASSED\n"); \
  testsPassed++; \
} catch (...) { \
  printf("FAILED\n"); \
  testsFailed++; \
}

#define ASSERT_TRUE(x) assert(x)
#define ASSERT_FALSE(x) assert(!(x))
#define ASSERT_EQ(a, b) assert((a) == (b))
#define ASSERT_NE(a, b) assert((a) != (b))

// ============================================================================
// Tests
// ============================================================================

TEST(status_ok) {
  Status st = Status::Ok();
  ASSERT_TRUE(st.ok());
  ASSERT_EQ(st.code, Err::OK);
}

TEST(status_error) {
  Status st = Status::Error(Err::E2_ERROR, "Test error", 42);
  ASSERT_FALSE(st.ok());
  ASSERT_EQ(st.code, Err::E2_ERROR);
  ASSERT_EQ(st.detail, 42);
}

TEST(status_in_progress) {
  Status st = Status{Err::IN_PROGRESS, 0, "In progress"};
  ASSERT_FALSE(st.ok());
  ASSERT_TRUE(st.inProgress());
}

TEST(config_defaults) {
  Config cfg;
  ASSERT_EQ(cfg.setScl, nullptr);
  ASSERT_EQ(cfg.setSda, nullptr);
  ASSERT_EQ(cfg.readScl, nullptr);
  ASSERT_EQ(cfg.readSda, nullptr);
  ASSERT_EQ(cfg.delayUs, nullptr);
  ASSERT_EQ(cfg.deviceAddress, 0);
  ASSERT_EQ(cfg.clockLowUs, 100);
  ASSERT_EQ(cfg.clockHighUs, 100);
  ASSERT_EQ(cfg.bitTimeoutUs, 25000u);
  ASSERT_EQ(cfg.byteTimeoutUs, 35000u);
  ASSERT_EQ(cfg.writeDelayMs, 150u);
  ASSERT_EQ(cfg.intervalWriteDelayMs, 300u);
  ASSERT_EQ(cfg.offlineThreshold, 5);
}

// ============================================================================
// Main
// ============================================================================

int main() {
  printf("\n=== EE871 Unit Tests ===\n\n");
  
  RUN_TEST(status_ok);
  RUN_TEST(status_error);
  RUN_TEST(status_in_progress);
  RUN_TEST(config_defaults);
  
  printf("\n=== Results: %d passed, %d failed ===\n\n", testsPassed, testsFailed);
  
  return testsFailed > 0 ? 1 : 0;
}
