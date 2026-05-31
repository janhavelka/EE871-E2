/// @file test_basic.cpp
/// @brief Native contract tests for EE871 validation and lifecycle guards.

#include <type_traits>

#include <unity.h>

#include "EE871/Config.h"
#include "EE871/EE871.h"
#include "EE871/Status.h"
#include "support/FakeE2Transport.h"

using namespace EE871;
using EE871Test::FakeE2Transport;

static_assert(!std::is_copy_constructible_v<EE871::EE871>);
static_assert(!std::is_copy_assignable_v<EE871::EE871>);
static_assert(!std::is_move_constructible_v<EE871::EE871>);
static_assert(!std::is_move_assignable_v<EE871::EE871>);

void setUp() {}
void tearDown() {}

static Status beginFakeDevice(EE871::EE871& dev,
                              FakeE2Transport& fake,
                              uint8_t offlineThreshold = 5) {
  Config cfg = fake.makeConfig(offlineThreshold);
  return dev.begin(cfg);
}

void test_status_ok() {
  Status st = Status::Ok();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OK), static_cast<uint8_t>(st.code));
}

void test_status_error() {
  Status st = Status::Error(Err::E2_ERROR, "Test error", 42);
  TEST_ASSERT_FALSE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::E2_ERROR), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(42, st.detail);
}

void test_status_in_progress() {
  Status st{Err::IN_PROGRESS, 0, "In progress"};
  TEST_ASSERT_FALSE(st.ok());
  TEST_ASSERT_TRUE(st.inProgress());
}

void test_config_defaults() {
  Config cfg;
  TEST_ASSERT_NULL(cfg.setScl);
  TEST_ASSERT_NULL(cfg.setSda);
  TEST_ASSERT_NULL(cfg.readScl);
  TEST_ASSERT_NULL(cfg.readSda);
  TEST_ASSERT_NULL(cfg.delayUs);
  TEST_ASSERT_EQUAL_UINT8(0, cfg.deviceAddress);
  TEST_ASSERT_EQUAL_UINT16(100, cfg.clockLowUs);
  TEST_ASSERT_EQUAL_UINT16(100, cfg.clockHighUs);
  TEST_ASSERT_EQUAL_UINT32(25000u, cfg.bitTimeoutUs);
  TEST_ASSERT_EQUAL_UINT32(35000u, cfg.byteTimeoutUs);
  TEST_ASSERT_EQUAL_UINT32(150u, cfg.writeDelayMs);
  TEST_ASSERT_EQUAL_UINT32(300u, cfg.intervalWriteDelayMs);
  TEST_ASSERT_EQUAL_UINT8(5, cfg.offlineThreshold);
}

void test_command_table_control_bytes_and_support() {
  TEST_ASSERT_EQUAL_UINT8(0xC5, cmd::makeControlRead(cmd::MAIN_MV3_LO, 2));
  TEST_ASSERT_EQUAL_UINT8(0x50, cmd::makeControlWrite(cmd::MAIN_CUSTOM_PTR, 0));
  TEST_ASSERT_TRUE(cmd::isReadMainCommandSupported(cmd::MAIN_MV3_LO));
  TEST_ASSERT_TRUE(cmd::isReadMainCommandSupported(cmd::MAIN_MV4_HI));
  TEST_ASSERT_FALSE(cmd::isReadMainCommandSupported(cmd::MAIN_MV1_LO));
  TEST_ASSERT_FALSE(cmd::isReadMainCommandSupported(0x06));
}

void test_co2_error_code_names() {
  TEST_ASSERT_EQUAL_STRING("supply voltage low",
                           cmd::co2ErrorCodeName(cmd::CO2_ERROR_SUPPLY_VOLTAGE_LOW));
  TEST_ASSERT_EQUAL_STRING("sensor counts low",
                           cmd::co2ErrorCodeName(cmd::CO2_ERROR_SENSOR_COUNTS_LOW));
  TEST_ASSERT_EQUAL_STRING("sensor counts high",
                           cmd::co2ErrorCodeName(cmd::CO2_ERROR_SENSOR_COUNTS_HIGH));
  TEST_ASSERT_EQUAL_STRING("supply voltage breakdown at peak",
                           cmd::co2ErrorCodeName(cmd::CO2_ERROR_SUPPLY_VOLTAGE_BREAKDOWN));
  TEST_ASSERT_EQUAL_STRING("unknown CO2 error", cmd::co2ErrorCodeName(7));
}

void test_begin_rejects_missing_callbacks() {
  EE871::EE871 dev;
  Config cfg;
  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_rejects_invalid_device_address() {
  EE871::EE871 dev;
  Config cfg;
  cfg.setScl = [](bool, void*) {};
  cfg.setSda = [](bool, void*) {};
  cfg.readScl = [](void*) { return true; };
  cfg.readSda = [](void*) { return true; };
  cfg.delayUs = [](uint32_t, void*) {};
  cfg.deviceAddress = 8;
  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
}

void test_begin_rejects_clock_timing_below_spec() {
  EE871::EE871 dev;
  Config cfg;
  cfg.setScl = [](bool, void*) {};
  cfg.setSda = [](bool, void*) {};
  cfg.readScl = [](void*) { return true; };
  cfg.readSda = [](void*) { return true; };
  cfg.delayUs = [](uint32_t, void*) {};
  cfg.clockLowUs = 99;
  cfg.clockHighUs = 100;
  Status st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
}

void test_begin_normalizes_zero_offline_threshold() {
  EE871::EE871 dev;
  Config cfg;
  cfg.setScl = [](bool, void*) {};
  cfg.setSda = [](bool, void*) {};
  cfg.readScl = [](void*) { return true; };
  cfg.readSda = [](void*) { return true; };
  cfg.delayUs = [](uint32_t, void*) {};
  cfg.offlineThreshold = 0;
  Status st = dev.begin(cfg);
  TEST_ASSERT_NOT_EQUAL(static_cast<uint8_t>(Err::INVALID_CONFIG),
                        static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.driverState()));
}

void test_default_health_aliases() {
  EE871::EE871 dev;
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_FALSE(dev.isOnline());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(dev.state()),
                          static_cast<uint8_t>(dev.driverState()));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(dev.driverState()),
                          static_cast<uint8_t>(dev.healthState()));
  TEST_ASSERT_EQUAL_UINT8(5, dev.offlineThreshold());

  SettingsSnapshot snap;
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_FALSE(snap.initialized);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(snap.state));
  TEST_ASSERT_EQUAL_UINT8(5, snap.config.offlineThreshold);
  TEST_ASSERT_EQUAL_UINT32(0u, snap.totalFailures);
  TEST_ASSERT_EQUAL_UINT32(0u, snap.totalSuccess);

  const SettingsSnapshot byValue = dev.getSettings();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(snap.state),
                          static_cast<uint8_t>(byValue.state));
  TEST_ASSERT_EQUAL_UINT8(snap.config.offlineThreshold, byValue.config.offlineThreshold);
}

void test_probe_requires_begin() {
  EE871::EE871 dev;
  Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
}

void test_recover_requires_begin() {
  EE871::EE871 dev;
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
}

void test_high_level_helpers_check_initialization_first() {
  EE871::EE871 dev;
  uint8_t byte = 0;
  uint8_t buf[16] = {};

  Status st = dev.customRead(0, nullptr, 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.readErrorCode(byte);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.readSerialNumber(nullptr);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.readPartName(buf);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.writePartName(nullptr);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.writeBusAddress(0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.writeCo2IntervalFactor(1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.writeCo2Filter(0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.writeOperatingMode(0xFF);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  st = dev.startAutoAdjust();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));
}

void test_fake_transport_begin_succeeds() {
  FakeE2Transport fake;
  EE871::EE871 dev;

  Status st = beginFakeDevice(dev, fake);

  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(dev.hasGlobalInterval());
}

void test_clock_stretch_timeout_is_bounded_and_tracked() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.resetElapsed();
  fake.setHoldSclLow(true);
  uint8_t status = 0;
  Status st = dev.readStatus(status);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(fake.elapsedUs() <= 30U);
  TEST_ASSERT_EQUAL_UINT8(1, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_pec_mismatch_probe_is_raw_but_tracked_read_updates_health() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  const uint32_t totalFailuresBeforeProbe = dev.totalFailures();
  const uint8_t consecutiveBeforeProbe = dev.consecutiveFailures();
  fake.setCorruptReadPec(true);

  Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::PEC_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(totalFailuresBeforeProbe, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(consecutiveBeforeProbe, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));

  uint8_t status = 0;
  st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::PEC_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(totalFailuresBeforeProbe + 1U, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_device_absent_probe_has_no_health_side_effect_tracked_read_fails() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.setDevicePresent(false);
  const uint32_t totalFailuresBeforeProbe = dev.totalFailures();
  const uint32_t totalSuccessBeforeProbe = dev.totalSuccess();
  Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(totalFailuresBeforeProbe, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(totalSuccessBeforeProbe, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));

  uint8_t status = 0;
  st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(totalFailuresBeforeProbe + 1U, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(1, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_custom_write_verify_mismatch_returns_precise_error() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  const uint8_t address = cmd::CUSTOM_FILTER_CO2;
  fake.setMemory(address, 0x11);
  fake.dropWritesToAddress(address, true);

  Status st = dev.customWrite(address, 0x5A);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::E2_ERROR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verify failed", st.msg);
  TEST_ASSERT_EQUAL_INT32(0x11, st.detail);
}

void test_offline_threshold_and_recover_after_replug() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 2).ok());

  fake.setDevicePresent(false);
  uint8_t status = 0;
  dev.tick(100);
  Status st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(1, dev.consecutiveFailures());

  dev.tick(200);
  st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(2, dev.consecutiveFailures());

  fake.setDevicePresent(true);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  dev.tick(300);
  st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0, dev.consecutiveFailures());
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_status_ok);
  RUN_TEST(test_status_error);
  RUN_TEST(test_status_in_progress);
  RUN_TEST(test_config_defaults);
  RUN_TEST(test_command_table_control_bytes_and_support);
  RUN_TEST(test_co2_error_code_names);
  RUN_TEST(test_begin_rejects_missing_callbacks);
  RUN_TEST(test_begin_rejects_invalid_device_address);
  RUN_TEST(test_begin_rejects_clock_timing_below_spec);
  RUN_TEST(test_begin_normalizes_zero_offline_threshold);
  RUN_TEST(test_default_health_aliases);
  RUN_TEST(test_probe_requires_begin);
  RUN_TEST(test_recover_requires_begin);
  RUN_TEST(test_high_level_helpers_check_initialization_first);
  RUN_TEST(test_fake_transport_begin_succeeds);
  RUN_TEST(test_clock_stretch_timeout_is_bounded_and_tracked);
  RUN_TEST(test_pec_mismatch_probe_is_raw_but_tracked_read_updates_health);
  RUN_TEST(test_device_absent_probe_has_no_health_side_effect_tracked_read_fails);
  RUN_TEST(test_custom_write_verify_mismatch_returns_precise_error);
  RUN_TEST(test_offline_threshold_and_recover_after_replug);
  return UNITY_END();
}

