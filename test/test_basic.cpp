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

static void assertSameStatus(const Status& expected, const Status& actual) {
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected.code),
                          static_cast<uint8_t>(actual.code));
  TEST_ASSERT_EQUAL_INT32(expected.detail, actual.detail);
  TEST_ASSERT_EQUAL_STRING(expected.msg, actual.msg);
}

static void assertDirtyWithOriginalError(const EE871::EE871& dev, const Status& st) {
  TEST_ASSERT_TRUE(dev.persistentConfigDirty());
  assertSameStatus(st, dev.persistentConfigDirtyError());

  SettingsSnapshot snap;
  TEST_ASSERT_TRUE(dev.getSettings(snap).ok());
  TEST_ASSERT_TRUE(snap.persistentConfigDirty);
  assertSameStatus(st, snap.persistentConfigDirtyError);
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
  TEST_ASSERT_EQUAL_UINT32(350000u, cfg.flashStretchTimeoutUs);
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

  cfg.clockLowUs = 100;
  cfg.clockHighUs = 99;
  st = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
}

void test_begin_validates_generated_clock_period() {
  FakeE2Transport fake;
  Config cfg = fake.makeConfig();

  cfg.clockLowUs = 100;
  cfg.clockHighUs = 1890;
  EE871::EE871 boundaryDev;
  TEST_ASSERT_TRUE(boundaryDev.begin(cfg).ok());
  boundaryDev.end();

  cfg.clockHighUs = 1891;
  EE871::EE871 belowFrequencyDev;
  Status st = belowFrequencyDev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));

  cfg.clockLowUs = 1000;
  cfg.clockHighUs = 1000;
  EE871::EE871 setupMakesPeriodTooLongDev;
  st = setupMakesPeriodTooLongDev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));

  cfg.clockLowUs = UINT16_MAX;
  cfg.clockHighUs = UINT16_MAX;
  EE871::EE871 maximumTimingDev;
  st = maximumTimingDev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));
}

void test_begin_validates_nominal_byte_deadline() {
  FakeE2Transport fake;
  Config cfg = fake.makeConfig();
  cfg.byteTimeoutUs = 1890;

  EE871::EE871 equalDeadlineDev;
  Status st = equalDeadlineDev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));

  cfg.byteTimeoutUs = 1889;
  EE871::EE871 belowDeadlineDev;
  st = belowDeadlineDev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));

  cfg.byteTimeoutUs = 1891;
  EE871::EE871 aboveDeadlineDev;
  TEST_ASSERT_TRUE(aboveDeadlineDev.begin(cfg).ok());
}

void test_begin_rejects_timeouts_above_e2_limits() {
  FakeE2Transport fake;
  Config cfg = fake.makeConfig();

  cfg.bitTimeoutUs = 25001;
  EE871::EE871 bitTimeoutDev;
  Status st = bitTimeoutDev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));

  cfg = fake.makeConfig();
  cfg.byteTimeoutUs = 35001;
  EE871::EE871 byteTimeoutDev;
  st = byteTimeoutDev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                          static_cast<uint8_t>(st.code));

  cfg = fake.makeConfig();
  cfg.bitTimeoutUs = 25000;
  cfg.byteTimeoutUs = 35000;
  EE871::EE871 boundaryDev;
  TEST_ASSERT_TRUE(boundaryDev.begin(cfg).ok());
}

void test_begin_bus_reset_reports_stuck_lines_precisely() {
  FakeE2Transport sclFake;
  sclFake.setHoldSclLow(true);
  EE871::EE871 sclDev;
  Config sclCfg = sclFake.makeConfig();
  sclCfg.bitTimeoutUs = 27;
  sclCfg.flashStretchTimeoutUs = 300003;
  Status st = sclDev.begin(sclCfg);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SCL stuck during reset", st.msg);
  TEST_ASSERT_FALSE(sclDev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(sclDev.state()));
  TEST_ASSERT_EQUAL_UINT32(sclCfg.clockLowUs + sclCfg.flashStretchTimeoutUs, sclFake.elapsedUs());

  FakeE2Transport sdaLowFake;
  sdaLowFake.setSdaStuckLow(true);
  EE871::EE871 sdaLowDev;
  st = sdaLowDev.begin(sdaLowFake.makeConfig());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SDA did not release after STOP", st.msg);
  TEST_ASSERT_FALSE(sdaLowDev.isInitialized());

  FakeE2Transport sdaHighFake;
  sdaHighFake.setSdaStuckHigh(true);
  EE871::EE871 sdaHighDev;
  st = sdaHighDev.begin(sdaHighFake.makeConfig());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SDA did not go low for START", st.msg);
  TEST_ASSERT_FALSE(sdaHighDev.isInitialized());

  FakeE2Transport sclHighFake;
  sclHighFake.setSclStuckHigh(true);
  EE871::EE871 sclHighDev;
  st = sclHighDev.begin(sclHighFake.makeConfig());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SCL did not go low for START", st.msg);
  TEST_ASSERT_TRUE(sclHighFake.masterSclReleased());
  TEST_ASSERT_TRUE(sclHighFake.masterSdaReleased());
}

void test_begin_validates_full_ee871_identity_and_co2_capability() {
  FakeE2Transport wrongGroupFake;
  wrongGroupFake.setGroup(0x1234);
  EE871::EE871 wrongGroupDev;
  Status st = wrongGroupDev.begin(wrongGroupFake.makeConfig());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_SUPPORTED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(0x1234, st.detail);
  TEST_ASSERT_FALSE(wrongGroupDev.isInitialized());

  FakeE2Transport wrongSubgroupFake;
  wrongSubgroupFake.setSubgroup(0x08);
  EE871::EE871 wrongSubgroupDev;
  st = wrongSubgroupDev.begin(wrongSubgroupFake.makeConfig());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_SUPPORTED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(0x08, st.detail);
  TEST_ASSERT_FALSE(wrongSubgroupDev.isInitialized());

  FakeE2Transport noCo2Fake;
  noCo2Fake.setAvailableMeasurements(0);
  EE871::EE871 noCo2Dev;
  st = noCo2Dev.begin(noCo2Fake.makeConfig());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_SUPPORTED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("CO2 measurement not available", st.msg);
  TEST_ASSERT_FALSE(noCo2Dev.isInitialized());

  FakeE2Transport absentFake;
  absentFake.setDevicePresent(false);
  EE871::EE871 absentDev;
  st = absentDev.begin(absentFake.makeConfig());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(absentDev.isInitialized());
}

void test_begin_recovers_sda_released_by_reset_clocks() {
  FakeE2Transport fake;
  fake.releaseSdaAfterClockRises(3);
  EE871::EE871 dev;

  Status st = dev.begin(fake.makeConfig());

  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

void test_begin_normalizes_zero_offline_threshold() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  Config cfg = fake.makeConfig();
  cfg.offlineThreshold = 0;
  Status st = dev.begin(cfg);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_TRUE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(1, dev.offlineThreshold());
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
  TEST_ASSERT_FALSE(snap.persistentConfigDirty);
  TEST_ASSERT_TRUE(snap.persistentConfigDirtyError.ok());
  TEST_ASSERT_FALSE(dev.persistentConfigDirty());
  TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());

  const SettingsSnapshot byValue = dev.getSettings();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(snap.state),
                          static_cast<uint8_t>(byValue.state));
  TEST_ASSERT_EQUAL_UINT8(snap.config.offlineThreshold, byValue.config.offlineThreshold);
  TEST_ASSERT_FALSE(byValue.persistentConfigDirty);
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

  st = dev.resyncPersistentConfig();
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

// Adjacent register pairs are read with one pointer set plus auto-increment
// reads, so a swapped byte order would be silent on zero-valued defaults.
void test_adjacent_register_pairs_assemble_in_low_high_order() {
  FakeE2Transport fake;
  fake.setMemory(cmd::CUSTOM_FW_VERSION_MAIN, 0x0A);
  fake.setMemory(cmd::CUSTOM_FW_VERSION_SUB, 0x0B);
  fake.setMemory(cmd::CUSTOM_CO2_OFFSET_L, 0x34);
  fake.setMemory(cmd::CUSTOM_CO2_OFFSET_H, 0x12);
  fake.setMemory(cmd::CUSTOM_CO2_GAIN_L, 0x78);
  fake.setMemory(cmd::CUSTOM_CO2_GAIN_H, 0x56);
  fake.setMemory(cmd::CUSTOM_CO2_POINT_L_L, 0xBC);
  fake.setMemory(cmd::CUSTOM_CO2_POINT_L_H, 0x9A);
  fake.setMemory(cmd::CUSTOM_CO2_POINT_U_L, 0xF0);
  fake.setMemory(cmd::CUSTOM_CO2_POINT_U_H, 0xDE);
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  uint8_t main = 0;
  uint8_t sub = 0;
  TEST_ASSERT_TRUE(dev.readFirmwareVersion(main, sub).ok());
  TEST_ASSERT_EQUAL_UINT8(0x0A, main);
  TEST_ASSERT_EQUAL_UINT8(0x0B, sub);

  int16_t offset = 0;
  TEST_ASSERT_TRUE(dev.readCo2Offset(offset).ok());
  TEST_ASSERT_EQUAL_INT16(0x1234, offset);

  uint16_t gain = 0;
  TEST_ASSERT_TRUE(dev.readCo2Gain(gain).ok());
  TEST_ASSERT_EQUAL_UINT16(0x5678, gain);

  uint16_t lower = 0;
  uint16_t upper = 0;
  TEST_ASSERT_TRUE(dev.readCo2CalPoints(lower, upper).ok());
  TEST_ASSERT_EQUAL_UINT16(0x9ABC, lower);
  TEST_ASSERT_EQUAL_UINT16(0xDEF0, upper);

  uint16_t interval = 0;
  TEST_ASSERT_TRUE(dev.readMeasurementInterval(interval).ok());
  TEST_ASSERT_EQUAL_UINT16(cmd::INTERVAL_MIN_DECISEC, interval);
}

void test_feature_cache_failure_fails_begin_closed() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  fake.corruptNextCustomReadPec(cmd::CUSTOM_OPERATING_MODE_SUPPORT);

  const Status st = dev.begin(fake.makeConfig());

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::PEC_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::UNINIT),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.hasSerialNumber());
  TEST_ASSERT_FALSE(dev.hasPartName());
  TEST_ASSERT_FALSE(dev.hasAddressConfig());
  TEST_ASSERT_FALSE(dev.hasGlobalInterval());
  TEST_ASSERT_FALSE(dev.hasSpecificInterval());
  TEST_ASSERT_FALSE(dev.hasFilterConfig());
  TEST_ASSERT_FALSE(dev.hasErrorCode());
  TEST_ASSERT_FALSE(dev.hasLowPowerMode());
  TEST_ASSERT_FALSE(dev.hasE2Priority());
  TEST_ASSERT_FALSE(dev.hasAutoAdjust());
}

void test_persistent_write_ranges_precede_capability_checks() {
  FakeE2Transport fake;
  fake.setMemory(cmd::CUSTOM_OPERATING_FUNCTIONS, 0);
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  const uint32_t successBefore = dev.totalSuccess();
  const uint32_t failuresBefore = dev.totalFailures();

  Status st = dev.writeBusAddress(cmd::BUS_ADDRESS_MAX + 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OUT_OF_RANGE),
                          static_cast<uint8_t>(st.code));

  st = dev.writeMeasurementInterval(cmd::INTERVAL_MIN_DECISEC - 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OUT_OF_RANGE),
                          static_cast<uint8_t>(st.code));

  st = dev.writeMeasurementInterval(cmd::INTERVAL_MAX_DECISEC + 1);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OUT_OF_RANGE),
                          static_cast<uint8_t>(st.code));

  TEST_ASSERT_EQUAL_UINT32(successBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
}

void test_operating_mode_access_fails_closed() {
  FakeE2Transport unsupportedFake;
  unsupportedFake.setMemory(cmd::CUSTOM_OPERATING_MODE_SUPPORT, 0);
  unsupportedFake.setMemory(cmd::CUSTOM_OPERATING_MODE, 0x55);
  EE871::EE871 unsupportedDev;
  TEST_ASSERT_TRUE(beginFakeDevice(unsupportedDev, unsupportedFake).ok());

  const uint32_t successBefore = unsupportedDev.totalSuccess();
  const uint32_t failuresBefore = unsupportedDev.totalFailures();
  uint8_t mode = 0xA5;
  Status st = unsupportedDev.readOperatingMode(mode);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_SUPPORTED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_HEX8(0xA5, mode);
  TEST_ASSERT_EQUAL_UINT32(successBefore, unsupportedDev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, unsupportedDev.totalFailures());

  st = unsupportedDev.writeOperatingMode(0);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_SUPPORTED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(successBefore, unsupportedDev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, unsupportedDev.totalFailures());

  FakeE2Transport invalidFake;
  invalidFake.setMemory(cmd::CUSTOM_OPERATING_MODE, 0x04);
  EE871::EE871 invalidDev;
  TEST_ASSERT_TRUE(beginFakeDevice(invalidDev, invalidFake).ok());
  mode = 0xA5;
  st = invalidDev.readOperatingMode(mode);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OUT_OF_RANGE),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_HEX8(0xA5, mode);

  invalidFake.setMemory(cmd::CUSTOM_OPERATING_MODE, 0x03);
  st = invalidDev.readOperatingMode(mode);
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_HEX8(0x03, mode);
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

void test_byte_deadline_includes_nominal_phases_after_stretch() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  Config cfg = fake.makeConfig();
  cfg.byteTimeoutUs = 1900;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());

  fake.resetElapsed();
  fake.stretchClockReleaseAfter(2, 10);
  uint8_t status = 0;
  TEST_ASSERT_TRUE(dev.readStatus(status).ok());

  fake.resetElapsed();
  fake.stretchClockReleaseAfter(2, 11);
  Status st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Byte timeout", st.msg);
  TEST_ASSERT_EQUAL_UINT8(1, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_TRUE(fake.masterSclReleased());
  TEST_ASSERT_TRUE(fake.masterSdaReleased());
}

void test_start_allows_configured_high_settle_before_sampling_sda() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.releaseSdaAfterUs(4);
  uint8_t status = 0;
  TEST_ASSERT_TRUE(dev.readStatus(status).ok());
}

void test_stop_timeout_releases_both_master_lines() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.stretchClockReleaseAfter(29, dev.getConfig().bitTimeoutUs + 1U);
  uint8_t status = 0;
  Status st = dev.readStatus(status);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(fake.masterSclReleased());
  TEST_ASSERT_TRUE(fake.masterSdaReleased());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_bus_safety_checks_cover_sda_and_do_not_track_reset() {
  EE871::EE871 uninitializedDev;
  Status st = uninitializedDev.busReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_INITIALIZED),
                          static_cast<uint8_t>(st.code));

  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  const uint32_t successBefore = dev.totalSuccess();
  const uint32_t failuresBefore = dev.totalFailures();
  TEST_ASSERT_TRUE(dev.busReset().ok());
  TEST_ASSERT_EQUAL_UINT32(successBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));

  fake.setSdaStuckLow(true);
  st = dev.checkBusIdle();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SDA stuck low", st.msg);
  TEST_ASSERT_EQUAL_UINT32(successBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());

  fake.setSdaStuckLow(false);
  fake.setHoldSclLow(true);
  st = dev.busReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SCL stuck during reset", st.msg);
  TEST_ASSERT_EQUAL_UINT32(successBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());

  fake.setHoldSclLow(false);
  fake.setSclStuckHigh(true);
  st = dev.busReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SCL did not go low during reset", st.msg);
  TEST_ASSERT_TRUE(fake.masterSclReleased());
  TEST_ASSERT_TRUE(fake.masterSdaReleased());
  TEST_ASSERT_EQUAL_UINT32(successBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());

  fake.setSclStuckHigh(false);
  fake.stretchClockReleaseAfter(10, dev.getConfig().flashStretchTimeoutUs + 1U);
  st = dev.busReset();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SCL stuck during reset STOP", st.msg);
  TEST_ASSERT_EQUAL_UINT32(successBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
}

void test_start_requires_sda_high_to_low_edge() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.setSdaStuckLow(true);

  uint8_t status = 0;
  Status st = dev.readStatus(status);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SDA stuck low before START", st.msg);
  TEST_ASSERT_EQUAL_UINT8(1, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                          static_cast<uint8_t>(dev.state()));
}

void test_measurement_reads_cover_values_boundaries_and_high_byte_failure() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.setMv3(0x1234);
  fake.setMv4(0xABCD);
  const uint32_t successBefore = dev.totalSuccess();
  uint16_t ppm = 0;
  TEST_ASSERT_TRUE(dev.readCo2Fast(ppm).ok());
  TEST_ASSERT_EQUAL_UINT16(0x1234, ppm);
  TEST_ASSERT_TRUE(dev.readCo2Average(ppm).ok());
  TEST_ASSERT_EQUAL_UINT16(0xABCD, ppm);
  TEST_ASSERT_EQUAL_UINT32(successBefore + 4U, dev.totalSuccess());

  fake.setMv3(0);
  TEST_ASSERT_TRUE(dev.readCo2Fast(ppm).ok());
  TEST_ASSERT_EQUAL_UINT16(0, ppm);
  fake.setMv4(UINT16_MAX);
  TEST_ASSERT_TRUE(dev.readCo2Average(ppm).ok());
  TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, ppm);

  ppm = 0xBEEF;
  const uint32_t failuresBefore = dev.totalFailures();
  fake.nackNextReadMainCommand(cmd::MAIN_MV3_HI);
  Status st = dev.readCo2Fast(ppm);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT16(0xBEEF, ppm);
  TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
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

void test_probe_validates_full_identity_without_health_side_effects() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  const uint32_t failuresBefore = dev.totalFailures();
  const uint32_t successesBefore = dev.totalSuccess();
  fake.setSubgroup(0x08);
  Status st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_SUPPORTED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Unexpected subgroup id", st.msg);

  fake.setSubgroup(cmd::SENSOR_SUBGROUP_ID);
  fake.setAvailableMeasurements(0);
  st = dev.probe();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_SUPPORTED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("CO2 measurement not available", st.msg);

  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

void test_absent_sampling_burst_short_circuits_to_three_failures() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 5).ok());
  fake.setDevicePresent(false);

  const uint32_t failuresBefore = dev.totalFailures();
  uint16_t ppm = 0;
  uint8_t status = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(dev.readCo2Fast(ppm).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(dev.readCo2Average(ppm).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(dev.readStatus(status).code));

  TEST_ASSERT_EQUAL_UINT32(failuresBefore + 3U, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT8(3, dev.consecutiveFailures());
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

  const uint32_t elapsedBeforeBlockedRead = fake.elapsedUs();
  const uint32_t failuresBeforeBlockedRead = dev.totalFailures();
  const uint32_t successesBeforeBlockedRead = dev.totalSuccess();
  st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(elapsedBeforeBlockedRead, fake.elapsedUs());
  TEST_ASSERT_EQUAL_UINT32(failuresBeforeBlockedRead, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBeforeBlockedRead, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  st = dev.customWrite(cmd::CUSTOM_FILTER_CO2, 10);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(elapsedBeforeBlockedRead, fake.elapsedUs());

  const uint32_t failuresBeforeProbe = dev.totalFailures();
  const uint32_t successesBeforeProbe = dev.totalSuccess();
  TEST_ASSERT_TRUE(dev.probe().ok());
  TEST_ASSERT_TRUE(fake.elapsedUs() > elapsedBeforeBlockedRead);
  TEST_ASSERT_EQUAL_UINT32(failuresBeforeProbe, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBeforeProbe, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  dev.tick(300);
  st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBeforeProbe + 1U, dev.totalSuccess());
}

void test_failed_recovery_is_atomic_and_keeps_offline_latched() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());

  fake.setDevicePresent(false);
  uint8_t status = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(dev.readStatus(status).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  fake.setDevicePresent(true);
  fake.setGroup(0x1234);
  const uint32_t failuresBefore = dev.totalFailures();
  const uint32_t successesBefore = dev.totalSuccess();
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NOT_SUPPORTED),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  fake.setGroup(cmd::SENSOR_GROUP_ID);
  st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(successesBefore + 1U, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
}

void test_recovery_refreshes_feature_cache_atomically() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
  TEST_ASSERT_TRUE(dev.hasGlobalInterval());
  TEST_ASSERT_TRUE(dev.hasLowPowerMode());
  TEST_ASSERT_TRUE(dev.hasAutoAdjust());

  fake.setDevicePresent(false);
  uint8_t status = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(dev.readStatus(status).code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));

  fake.setDevicePresent(true);
  fake.setMemory(cmd::CUSTOM_OPERATING_FUNCTIONS, 0);
  fake.setMemory(cmd::CUSTOM_OPERATING_MODE_SUPPORT, 0);
  fake.setMemory(cmd::CUSTOM_SPECIAL_FEATURES, 0);
  fake.corruptNextCustomReadPec(cmd::CUSTOM_OPERATING_MODE_SUPPORT);
  const uint32_t failuresBefore = dev.totalFailures();
  const uint32_t successesBefore = dev.totalSuccess();
  Status st = dev.recover();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::PEC_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.hasGlobalInterval());
  TEST_ASSERT_FALSE(dev.hasLowPowerMode());
  TEST_ASSERT_FALSE(dev.hasAutoAdjust());

  st = dev.recover();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_EQUAL_UINT32(successesBefore + 1U, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                          static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_FALSE(dev.hasGlobalInterval());
  TEST_ASSERT_FALSE(dev.hasLowPowerMode());
  TEST_ASSERT_FALSE(dev.hasAutoAdjust());
}

void test_interval_low_byte_write_failure_does_not_dirty() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.failNextWriteToAddress(cmd::CUSTOM_INTERVAL_L);
  Status st = dev.writeMeasurementInterval(300);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Address byte NACK", st.msg);
  TEST_ASSERT_FALSE(dev.persistentConfigDirty());
  TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(cmd::INTERVAL_MIN_DECISEC & 0xFF),
                          fake.memory(cmd::CUSTOM_INTERVAL_L));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(cmd::INTERVAL_MIN_DECISEC >> 8),
                          fake.memory(cmd::CUSTOM_INTERVAL_H));
}

void test_interval_high_byte_write_failure_sets_dirty() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.failNextWriteToAddress(cmd::CUSTOM_INTERVAL_H);
  Status st = dev.writeMeasurementInterval(300);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Address byte NACK", st.msg);
  assertDirtyWithOriginalError(dev, st);
  TEST_ASSERT_EQUAL_UINT8(0x2C, fake.memory(cmd::CUSTOM_INTERVAL_L));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(cmd::INTERVAL_MIN_DECISEC >> 8),
                          fake.memory(cmd::CUSTOM_INTERVAL_H));
}

void test_interval_verify_failure_sets_dirty_and_unrelated_read_does_not_clear() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.dropNextWriteCommitToAddress(cmd::CUSTOM_INTERVAL_H);
  Status st = dev.writeMeasurementInterval(300);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::E2_ERROR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Interval verify failed", st.msg);
  TEST_ASSERT_EQUAL_INT32(0x2C, st.detail);
  assertDirtyWithOriginalError(dev, st);

  uint8_t status = 0;
  TEST_ASSERT_TRUE(dev.readStatus(status).ok());
  assertDirtyWithOriginalError(dev, st);
}

void test_co2_offset_high_byte_failure_sets_dirty() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.failNextWriteToAddress(cmd::CUSTOM_CO2_OFFSET_H);
  Status st = dev.writeCo2Offset(0x1234);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Address byte NACK", st.msg);
  assertDirtyWithOriginalError(dev, st);
  TEST_ASSERT_EQUAL_UINT8(0x34, fake.memory(cmd::CUSTOM_CO2_OFFSET_L));
}

void test_co2_offset_low_byte_verify_failure_sets_dirty() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.dropNextWriteCommitToAddress(cmd::CUSTOM_CO2_OFFSET_L);
  Status st = dev.writeCo2Offset(0x1234);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::E2_ERROR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verify failed", st.msg);
  assertDirtyWithOriginalError(dev, st);
}

void test_co2_gain_high_byte_failure_sets_dirty() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.failNextWriteToAddress(cmd::CUSTOM_CO2_GAIN_H);
  Status st = dev.writeCo2Gain(0x5678);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Address byte NACK", st.msg);
  assertDirtyWithOriginalError(dev, st);
  TEST_ASSERT_EQUAL_UINT8(0x78, fake.memory(cmd::CUSTOM_CO2_GAIN_L));
}

void test_co2_gain_low_byte_verify_failure_sets_dirty() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.dropNextWriteCommitToAddress(cmd::CUSTOM_CO2_GAIN_L);
  Status st = dev.writeCo2Gain(0x5678);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::E2_ERROR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verify failed", st.msg);
  assertDirtyWithOriginalError(dev, st);
}

void test_part_name_first_byte_verify_failure_sets_dirty() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  const uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {
      'E', 'E', '8', '7', '1', ' ', 'B', 'E',
      'N', 'C', 'H', ' ', '0', '0', '0', '1'};
  fake.dropNextWriteCommitToAddress(cmd::CUSTOM_PART_NAME_START);
  Status st = dev.writePartName(partName);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::E2_ERROR),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verify failed", st.msg);
  assertDirtyWithOriginalError(dev, st);
}

void test_dirty_error_preserves_first_failure() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.dropNextWriteCommitToAddress(cmd::CUSTOM_INTERVAL_H);
  Status first = dev.writeMeasurementInterval(300);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::E2_ERROR),
                          static_cast<uint8_t>(first.code));
  assertDirtyWithOriginalError(dev, first);

  fake.failNextWriteToAddress(cmd::CUSTOM_CO2_GAIN_H);
  Status second = dev.writeCo2Gain(0x5678);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(second.code));
  assertDirtyWithOriginalError(dev, first);
}

void test_resync_persistent_config_clears_only_when_coherent() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.failNextWriteToAddress(cmd::CUSTOM_INTERVAL_H);
  Status dirtyCause = dev.writeMeasurementInterval(300);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(dirtyCause.code));
  assertDirtyWithOriginalError(dev, dirtyCause);

  Status st = dev.resyncPersistentConfig();
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OUT_OF_RANGE),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(dev.persistentConfigDirty());

  fake.setMemory(cmd::CUSTOM_INTERVAL_L, 0x2C);
  fake.setMemory(cmd::CUSTOM_INTERVAL_H, 0x01);
  st = dev.resyncPersistentConfig();
  TEST_ASSERT_TRUE(st.ok());
  TEST_ASSERT_FALSE(dev.persistentConfigDirty());
  TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());
}

void test_dirty_state_survives_offline() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 2).ok());

  fake.failNextWriteToAddress(cmd::CUSTOM_CO2_OFFSET_H);
  Status dirtyCause = dev.writeCo2Offset(0x1234);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(dirtyCause.code));
  assertDirtyWithOriginalError(dev, dirtyCause);

  fake.setDevicePresent(false);
  uint8_t status = 0;
  Status st = dev.readStatus(status);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                          static_cast<uint8_t>(dev.state()));
  assertDirtyWithOriginalError(dev, dirtyCause);
}

void test_transfer_failure_survives_cleanup_stop_timeout() {
  for (uint8_t scenario = 0; scenario < 4; ++scenario) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    const bool badPec = scenario == 2;
    fake.setDevicePresent(badPec);
    fake.setCorruptReadPec(badPec);
    fake.resetElapsed();
    const uint32_t stopTimeoutUs = scenario == 1
        ? dev.getConfig().flashStretchTimeoutUs : dev.getConfig().bitTimeoutUs;
    // START release + 9 control bits (or 27 read bits) + STOP release.
    fake.stretchClockReleaseAfter(badPec ? 29 : 11, stopTimeoutUs + 1U);
    uint8_t value = 0;
    const Status st = scenario == 1
        ? dev.customWrite(cmd::CUSTOM_FILTER_CO2, 10)
        : scenario == 3 ? dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2)
                        : dev.readStatus(value);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(badPec ? Err::PEC_MISMATCH : Err::NACK),
                            static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_STRING(badPec ? "PEC mismatch" : "Control byte NACK", st.msg);
    TEST_ASSERT_EQUAL_INT32(badPec ? 0x70 : 0, st.detail);
    // Fake defaults: START 108 us, each bit 210 us, failed STOP 110 us.
    TEST_ASSERT_EQUAL_UINT32(108U + (badPec ? 27U : 9U) * 210U +
                                110U + stopTimeoutUs, fake.elapsedUs());
    TEST_ASSERT_TRUE(fake.masterSclReleased());
    TEST_ASSERT_TRUE(fake.masterSdaReleased());
    TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
    TEST_ASSERT_FALSE(dev.persistentConfigDirty());
  }
}

void test_failed_recover_clears_capabilities_and_latches_from_ready_or_degraded() {
  const uint8_t thresholds[] = {1, 5, 255};
  for (uint8_t threshold : thresholds) {
    for (uint8_t failure = 0; failure < 4; ++failure) {
      FakeE2Transport fake;
      EE871::EE871 dev;
      TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, threshold).ok());
      if (threshold != 1) {
        fake.setDevicePresent(false);
        uint8_t value = 0;
        TEST_ASSERT_FALSE(dev.readStatus(value).ok());
        fake.setDevicePresent(true);
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                                static_cast<uint8_t>(dev.state()));
      }
      if (failure == 0) fake.setSubgroup(0x08);
      if (failure == 1) fake.setDevicePresent(false);
      if (failure == 2) fake.setHoldSclLow(true);
      if (failure == 3) fake.corruptNextCustomReadPec(cmd::CUSTOM_OPERATING_MODE_SUPPORT);
      const uint32_t failuresBefore = dev.totalFailures();
      const uint32_t successesBefore = dev.totalSuccess();
      const Status st = dev.recover();
      const Err expected[] = {Err::NOT_SUPPORTED, Err::NACK, Err::BUS_STUCK, Err::PEC_MISMATCH};
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected[failure]), static_cast<uint8_t>(st.code));
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                              static_cast<uint8_t>(dev.state()));
      TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
      TEST_ASSERT_EQUAL_UINT32(successesBefore, dev.totalSuccess());
      TEST_ASSERT_TRUE(dev.consecutiveFailures() >= threshold);
      TEST_ASSERT_FALSE(dev.hasSerialNumber());
      TEST_ASSERT_FALSE(dev.hasPartName());
      TEST_ASSERT_FALSE(dev.hasAddressConfig());
      TEST_ASSERT_FALSE(dev.hasGlobalInterval());
      TEST_ASSERT_FALSE(dev.hasSpecificInterval());
      TEST_ASSERT_FALSE(dev.hasFilterConfig());
      TEST_ASSERT_FALSE(dev.hasErrorCode());
      TEST_ASSERT_FALSE(dev.hasLowPowerMode());
      TEST_ASSERT_FALSE(dev.hasE2Priority());
      TEST_ASSERT_FALSE(dev.hasAutoAdjust());
      const SettingsSnapshot snap = dev.getSettings();
      TEST_ASSERT_EQUAL_UINT8(0, snap.operatingFunctions);
      TEST_ASSERT_EQUAL_UINT8(0, snap.operatingModeSupport);
      TEST_ASSERT_EQUAL_UINT8(0, snap.specialFeatures);

      fake.setSubgroup(cmd::SENSOR_SUBGROUP_ID);
      fake.setDevicePresent(true);
      fake.setHoldSclLow(false);
      fake.resetElapsed();
      TEST_ASSERT_FALSE(dev.startAutoAdjust().ok());
      uint8_t value = 0;
      TEST_ASSERT_FALSE(dev.readStatus(value).ok());
      TEST_ASSERT_FALSE(dev.resyncPersistentConfig().ok());
      TEST_ASSERT_EQUAL_UINT32(0, fake.elapsedUs());
      TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
      TEST_ASSERT_EQUAL_UINT32(successesBefore, dev.totalSuccess());
      TEST_ASSERT_TRUE(dev.recover().ok());
      TEST_ASSERT_TRUE(dev.hasAutoAdjust());
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                              static_cast<uint8_t>(dev.state()));
    }
  }
}

void test_offline_replay_marks_message_and_preserves_original_diagnostics() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
  dev.tick(123);
  fake.setCorruptReadPec(true);
  uint8_t value = 0;
  const Status failure = dev.readStatus(value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::PEC_MISMATCH), static_cast<uint8_t>(failure.code));
  TEST_ASSERT_NOT_EQUAL(0, failure.detail);
  fake.setCorruptReadPec(false);
  fake.resetElapsed();
  dev.tick(456);
  const Status replays[] = {dev.readStatus(value), dev.customWrite(cmd::CUSTOM_FILTER_CO2, 10),
                            dev.resyncPersistentConfig()};
  for (const Status& replay : replays) {
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(failure.code), static_cast<uint8_t>(replay.code));
    TEST_ASSERT_EQUAL_INT32(failure.detail, replay.detail);
    TEST_ASSERT_EQUAL_STRING("Driver offline; call recover()", replay.msg);
  }
  const SettingsSnapshot snap = dev.getSettings();
  assertSameStatus(failure, snap.lastError);
  TEST_ASSERT_EQUAL_UINT32(123, snap.lastErrorMs);
  TEST_ASSERT_EQUAL_UINT32(1, snap.totalFailures);
  TEST_ASSERT_EQUAL_UINT32(0, fake.elapsedUs());
  TEST_ASSERT_TRUE(dev.busReset().ok());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE), static_cast<uint8_t>(dev.state()));
}

void test_flash_stretch_config_boundaries() {
  const uint32_t invalid[] = {0, 299999, 5000001, UINT32_MAX};
  for (uint32_t timeout : invalid) {
    FakeE2Transport fake;
    Config cfg = fake.makeConfig();
    cfg.flashStretchTimeoutUs = timeout;
    EE871::EE871 dev;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
                            static_cast<uint8_t>(dev.begin(cfg).code));
    TEST_ASSERT_EQUAL_UINT32(0, fake.elapsedUs());
  }
  const uint32_t valid[] = {300000, 350000, 5000000};
  for (uint32_t timeout : valid) {
    FakeE2Transport fake;
    Config cfg = fake.makeConfig();
    cfg.flashStretchTimeoutUs = timeout;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  }
}

void test_flash_write_stop_stretches_commit_and_verify() {
  const uint8_t addresses[] = {0, 7};
  const uint32_t stretches[] = {150000, 300000, 350000, 350001};
  for (uint8_t address : addresses) {
    for (uint32_t stretch : stretches) {
      for (uint8_t operation = 0; operation < 2; ++operation) {
        FakeE2Transport fake;
        EE871::EE871 dev;
        Config cfg = fake.makeConfig();
        cfg.deviceAddress = address;
        TEST_ASSERT_TRUE(dev.begin(cfg).ok());
        // Each write uses START + 36 bits + STOP. Interval commits on write two.
        fake.stretchClockReleaseAfter(operation == 1 ? 76 : 38, stretch);
        Status st;
        if (operation == 0) st = dev.customWrite(cmd::CUSTOM_FILTER_CO2, 10);
        if (operation == 1) st = dev.writeMeasurementInterval(300);
        if (stretch <= dev.getConfig().flashStretchTimeoutUs) {
          TEST_ASSERT_TRUE(st.ok());
          TEST_ASSERT_FALSE(dev.persistentConfigDirty());
          TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
          if (operation == 0) TEST_ASSERT_EQUAL_UINT8(10, fake.memory(cmd::CUSTOM_FILTER_CO2));
          if (operation == 1) {
            uint16_t interval = 0;
            TEST_ASSERT_TRUE(dev.readMeasurementInterval(interval).ok());
            TEST_ASSERT_EQUAL_UINT16(300, interval);
          }
        } else {
          TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT), static_cast<uint8_t>(st.code));
          TEST_ASSERT_EQUAL_INT32(dev.getConfig().flashStretchTimeoutUs, st.detail);
          if (operation == 1) TEST_ASSERT_TRUE(dev.persistentConfigDirty());
          TEST_ASSERT_TRUE(fake.masterSclReleased());
          TEST_ASSERT_TRUE(fake.masterSdaReleased());
        }
      }
    }
  }
}

void test_accepted_interval_low_byte_stop_failure_marks_dirty() {
  const uint8_t addresses[] = {0, 7};
  for (uint8_t address : addresses) {
    FakeE2Transport fake;
    Config cfg = fake.makeConfig();
    cfg.deviceAddress = address;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    fake.stretchClockReleaseAfter(38, cfg.flashStretchTimeoutUs + 1U);
    const Status st = dev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                           static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(cfg.flashStretchTimeoutUs, st.detail);
    assertDirtyWithOriginalError(dev, st);
    TEST_ASSERT_EQUAL_UINT8(0x2C, fake.memory(cmd::CUSTOM_INTERVAL_L));
    TEST_ASSERT_EQUAL_UINT8(0, fake.memory(cmd::CUSTOM_INTERVAL_H));
    TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
    TEST_ASSERT_TRUE(fake.masterSclReleased());
    TEST_ASSERT_TRUE(fake.masterSdaReleased());
  }
}

void test_read_and_pointer_stop_use_ordinary_timeout() {
  const uint8_t addresses[] = {0, 7};
  const uint32_t timeouts[] = {27, 25000};
  const bool operations[] = {false, true};
  for (uint8_t address : addresses) {
    for (uint32_t timeout : timeouts) {
      const uint32_t stretches[] = {
          timeout - timeout % 5U - 5U, timeout, timeout + 1U, 350000U};
      for (uint32_t stretch : stretches) {
        for (bool pointer : operations) {
          FakeE2Transport fake;
          Config cfg = fake.makeConfig();
          cfg.deviceAddress = address;
          cfg.startHoldUs = 100;
          cfg.stopHoldUs = 100;
          cfg.bitTimeoutUs = timeout;
          EE871::EE871 dev;
          TEST_ASSERT_TRUE(dev.begin(cfg).ok());
          fake.setMemory(cmd::CUSTOM_FILTER_CO2, 42);
          fake.resetElapsed();
          // START + 27 read / 36 write bit clocks + STOP.
          fake.stretchClockReleaseAfter(pointer ? 38 : 29, stretch);
          uint8_t value = 0xFF;
          const Status st = pointer ? dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2)
                                    : dev.readStatus(value);
          const uint32_t nominalUs = pointer ? 8170U : 6280U;
          if (stretch <= timeout) {
            TEST_ASSERT_TRUE(st.ok());
            TEST_ASSERT_EQUAL_UINT32(nominalUs + stretch, fake.elapsedUs());
            TEST_ASSERT_EQUAL_UINT32(1, dev.totalSuccess());
            TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
            if (pointer) {
              TEST_ASSERT_TRUE(dev.readControlByte(cmd::MAIN_CUSTOM_PTR, value).ok());
              TEST_ASSERT_EQUAL_UINT8(42, value);
            } else {
              TEST_ASSERT_EQUAL_UINT8(0, value);
            }
          } else {
            TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                                   static_cast<uint8_t>(st.code));
            TEST_ASSERT_EQUAL_STRING("Clock stretch timeout", st.msg);
            TEST_ASSERT_EQUAL_INT32(timeout, st.detail);
            // An unsuccessful STOP omits both 100 us hold phases.
            TEST_ASSERT_EQUAL_UINT32(nominalUs + timeout - 200U, fake.elapsedUs());
            TEST_ASSERT_EQUAL_UINT32(0, dev.totalSuccess());
            TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
            TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::DEGRADED),
                                   static_cast<uint8_t>(dev.state()));
          }
          TEST_ASSERT_FALSE(dev.persistentConfigDirty());
          TEST_ASSERT_TRUE(fake.masterSclReleased());
          TEST_ASSERT_TRUE(fake.masterSdaReleased());
        }
      }
    }
  }
}

void test_bus_reset_flash_stretch_is_bounded_and_health_neutral() {
  const uint8_t releases[] = {1, 9, 10};
  const uint32_t stretches[] = {300000, 350000, 350001};
  for (uint8_t release : releases) {
    for (uint32_t stretch : stretches) {
      FakeE2Transport fake;
      EE871::EE871 dev;
      TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
      fake.resetElapsed();
      fake.stretchClockReleaseAfter(release, stretch);
      const Status st = dev.busReset();
      TEST_ASSERT_EQUAL(stretch <= 350000U, st.ok());
      if (!st.ok()) TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK), static_cast<uint8_t>(st.code));
      TEST_ASSERT_TRUE(fake.elapsedUs() <= 352000U);
      TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
      TEST_ASSERT_EQUAL_UINT32(0, dev.totalSuccess());
      TEST_ASSERT_TRUE(fake.masterSclReleased());
      TEST_ASSERT_TRUE(fake.masterSdaReleased());
    }
  }
}

void test_flash_budget_does_not_relax_bit_transfer_deadline() {
  FakeE2Transport fake;
  Config cfg = fake.makeConfig();
  cfg.bitTimeoutUs = 25000;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  fake.stretchClockReleaseAfter(2, 25001);
  uint8_t value = 0;
  const Status st = dev.readStatus(value);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT), static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(25000, st.detail);
  TEST_ASSERT_EQUAL_STRING("Clock stretch timeout", st.msg);
  TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
}

static Config retryConfig(FakeE2Transport& fake, uint8_t retries = 3) {
  Config cfg = fake.makeConfig(1);
  cfg.bitTimeoutUs = 25000;
  cfg.startHoldUs = 100;
  cfg.stopHoldUs = 100;
  cfg.readNackRetries = retries;
  return cfg;
}

void test_read_retry_config_rejects_invalid_without_io() {
  const uint8_t invalid[] = {4, 255};
  for (uint8_t retries : invalid) {
    FakeE2Transport fake;
    fake.setHoldSclLow(true);
    EE871::EE871 dev;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::INVALID_CONFIG),
        static_cast<uint8_t>(dev.begin(retryConfig(fake, retries)).code));
    TEST_ASSERT_EQUAL_UINT32(0, fake.elapsedUs());
    TEST_ASSERT_FALSE(dev.isInitialized());
  }
  TEST_ASSERT_EQUAL_UINT8(0, Config{}.readNackRetries);
  TEST_ASSERT_NULL(Config{}.allowReadRetry);
}

void test_measurement_status_retries_recover_on_each_allowed_attempt() {
  const uint8_t commands[] = {cmd::MAIN_MV3_LO, cmd::MAIN_MV3_HI,
      cmd::MAIN_MV4_LO, cmd::MAIN_MV4_HI, cmd::MAIN_STATUS};
  const uint8_t addresses[] = {0, 7};
  for (uint8_t address : addresses) {
    for (uint8_t mainCommand : commands) {
      for (uint8_t nacks = 1; nacks <= 3; ++nacks) {
        FakeE2Transport fake;
        Config cfg = retryConfig(fake);
        cfg.deviceAddress = address;
        EE871::EE871 dev;
        TEST_ASSERT_TRUE(dev.begin(cfg).ok());
        fake.nackNextReadMainCommand(mainCommand, nacks);
        fake.resetElapsed();
        uint8_t value = 0;
        TEST_ASSERT_TRUE(dev.readControlByte(mainCommand, value).ok());
        const uint8_t control = cmd::makeControlRead(mainCommand, address);
        TEST_ASSERT_EQUAL_UINT32(nacks + 1U, fake.controlCount(control));
        TEST_ASSERT_EQUAL_UINT32(6280U + nacks * (2500U + 1000U), fake.elapsedUs());
        TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::READY),
                               static_cast<uint8_t>(dev.state()));
        TEST_ASSERT_EQUAL_UINT32(1, dev.totalSuccess());
        TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
        const ReadRetryDiagnostics stats = dev.readRetryDiagnostics();
        TEST_ASSERT_EQUAL_UINT32(nacks, stats.controlNacks);
        TEST_ASSERT_EQUAL_UINT32(nacks, stats.retries);
        TEST_ASSERT_EQUAL_UINT32(1, stats.recovered);
        TEST_ASSERT_EQUAL_UINT32(0, stats.exhausted);
        TEST_ASSERT_EQUAL_UINT8(control, stats.lastControlByte);
        TEST_ASSERT_EQUAL_UINT8(nacks, stats.lastRetriesUsed);
        TEST_ASSERT_TRUE(stats.lastError.ok());
        TEST_ASSERT_TRUE(stats.lastRecovered);
        TEST_ASSERT_FALSE(stats.cleanupBlocked);
        TEST_ASSERT_FALSE(stats.retryVetoed);
        TEST_ASSERT_EQUAL_UINT32(stats.controlNacks, dev.getSettings().readRetry.controlNacks);
      }
    }
  }
}

void test_retry_exhaustion_counts_one_health_failure_and_preserves_session() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(dev.begin(retryConfig(fake)).ok());
  fake.nackNextReadMainCommand(cmd::MAIN_STATUS, 4);
  fake.resetElapsed();
  uint8_t value = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
      static_cast<uint8_t>(dev.readStatus(value).code));
  TEST_ASSERT_EQUAL_UINT32(4, fake.controlCount(0x71));
  TEST_ASSERT_EQUAL_UINT32(13000, fake.elapsedUs());
  TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(0, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(DriverState::OFFLINE),
                         static_cast<uint8_t>(dev.state()));
  const ReadRetryDiagnostics stats = dev.readRetryDiagnostics();
  TEST_ASSERT_EQUAL_UINT32(4, stats.controlNacks);
  TEST_ASSERT_EQUAL_UINT32(3, stats.retries);
  TEST_ASSERT_EQUAL_UINT32(1, stats.exhausted);
  TEST_ASSERT_EQUAL_UINT32(0, stats.recovered);
  TEST_ASSERT_EQUAL_UINT8(3, stats.lastRetriesUsed);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK), static_cast<uint8_t>(stats.lastError.code));
  fake.resetElapsed();
  TEST_ASSERT_FALSE(dev.readStatus(value).ok());
  TEST_ASSERT_EQUAL_UINT32(0, fake.elapsedUs());
  TEST_ASSERT_EQUAL_UINT32(4, dev.readRetryDiagnostics().controlNacks);
  TEST_ASSERT_TRUE(dev.recover().ok());
  TEST_ASSERT_EQUAL_UINT32(4, dev.readRetryDiagnostics().controlNacks);
  TEST_ASSERT_TRUE(dev.readStatus(value).ok());
  TEST_ASSERT_EQUAL_UINT32(1, dev.readRetryDiagnostics().exhausted);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
      static_cast<uint8_t>(dev.readRetryDiagnostics().lastError.code));
  dev.end();
  TEST_ASSERT_EQUAL_UINT32(0, dev.readRetryDiagnostics().controlNacks);
  TEST_ASSERT_EQUAL_UINT8(0, dev.readRetryDiagnostics().lastControlByte);
  TEST_ASSERT_TRUE(dev.begin(retryConfig(fake)).ok());
  TEST_ASSERT_EQUAL_UINT32(0, dev.getSettings().readRetry.retries);
}

void test_disabled_retry_still_counts_nack_and_metadata_keeps_event() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  Config cfg = retryConfig(fake, 0);
  cfg.offlineThreshold = 5;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  fake.nackNextReadMainCommand(cmd::MAIN_STATUS);
  uint8_t value = 0;
  TEST_ASSERT_FALSE(dev.readStatus(value).ok());
  TEST_ASSERT_EQUAL_UINT32(1, fake.controlCount(0x71));
  TEST_ASSERT_EQUAL_UINT32(1, dev.readRetryDiagnostics().controlNacks);
  TEST_ASSERT_EQUAL_UINT32(0, dev.readRetryDiagnostics().retries);
  TEST_ASSERT_EQUAL_UINT32(0, dev.readRetryDiagnostics().exhausted);
  uint8_t fwMain = 0, fwSub = 0;
  TEST_ASSERT_TRUE(dev.readFirmwareVersion(fwMain, fwSub).ok());
  TEST_ASSERT_EQUAL_UINT8(0x71, dev.readRetryDiagnostics().lastControlByte);
  TEST_ASSERT_EQUAL_UINT32(1, dev.readRetryDiagnostics().controlNacks);
}

void test_retry_respects_smaller_configured_limit() {
  for (uint8_t retries = 1; retries <= 2; ++retries) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(dev.begin(retryConfig(fake, retries)).ok());
    fake.nackNextReadMainCommand(cmd::MAIN_STATUS, 4);
    uint8_t value = 0;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(dev.readStatus(value).code));
    TEST_ASSERT_EQUAL_UINT32(retries + 1U, fake.controlCount(0x71));
    TEST_ASSERT_EQUAL_UINT32(retries, dev.readRetryDiagnostics().retries);
    TEST_ASSERT_EQUAL_UINT32(1, dev.readRetryDiagnostics().exhausted);
    TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
  }
}

void test_retry_stops_on_later_transfer_timeout() {
  FakeE2Transport fake;
  Config cfg = retryConfig(fake);
  cfg.allowReadRetry = [](void* user) {
    auto& bus = *static_cast<FakeE2Transport*>(user);
    // The next frame reaches START, then its first control-byte clock times out.
    bus.stretchClockReleaseAfter(2, 25001);
    return true;
  };
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  fake.nackNextReadMainCommand(cmd::MAIN_STATUS);
  uint8_t value = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
      static_cast<uint8_t>(dev.readStatus(value).code));
  const auto stats = dev.readRetryDiagnostics();
  TEST_ASSERT_EQUAL_UINT32(1, stats.controlNacks);
  TEST_ASSERT_EQUAL_UINT32(1, stats.retries);
  TEST_ASSERT_EQUAL_UINT32(0, stats.recovered);
  TEST_ASSERT_EQUAL_UINT32(0, stats.exhausted);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
      static_cast<uint8_t>(stats.lastError.code));
  TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
  TEST_ASSERT_TRUE(fake.masterSclReleased());
  TEST_ASSERT_TRUE(fake.masterSdaReleased());
}

void test_retry_never_replays_identity_custom_reads_or_writes() {
  const uint8_t commands[] = {cmd::MAIN_TYPE_LO, cmd::MAIN_TYPE_HI,
      cmd::MAIN_TYPE_SUB, cmd::MAIN_AVAIL_MEAS, cmd::MAIN_CUSTOM_PTR};
  for (uint8_t command : commands) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(dev.begin(retryConfig(fake)).ok());
    const uint8_t control = cmd::makeControlRead(command, 0);
    const uint32_t before = fake.controlCount(control);
    fake.nackNextReadMainCommand(command, 4);
    uint8_t value = 0;
    TEST_ASSERT_FALSE(dev.readControlByte(command, value).ok());
    TEST_ASSERT_EQUAL_UINT32(before + 1, fake.controlCount(control));
    TEST_ASSERT_EQUAL_UINT32(0, dev.readRetryDiagnostics().controlNacks);
    TEST_ASSERT_EQUAL_UINT32(0, dev.readRetryDiagnostics().retries);
  }
  for (uint8_t operation = 0; operation < 3; ++operation) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(dev.begin(retryConfig(fake)).ok());
    const uint32_t pointerBefore = fake.controlCount(0x50);
    if (operation == 0) fake.setDevicePresent(false);
    if (operation == 1) fake.failNextWriteToAddress(cmd::CUSTOM_FILTER_CO2);
    if (operation == 2) fake.nackNextReadMainCommand(cmd::MAIN_CUSTOM_PTR, 4);
    uint8_t value = 0;
    Status st;
    if (operation == 0) st = dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2);
    if (operation == 1) st = dev.customWrite(cmd::CUSTOM_FILTER_CO2, 42);
    if (operation == 2) st = dev.customRead(cmd::CUSTOM_FILTER_CO2, value);
    TEST_ASSERT_FALSE(st.ok());
    TEST_ASSERT_EQUAL_UINT32(operation == 1 ? 1 : 0, fake.controlCount(0x10));
    TEST_ASSERT_EQUAL_UINT32(pointerBefore + (operation == 1 ? 0 : 1), fake.controlCount(0x50));
    TEST_ASSERT_EQUAL_UINT32(0, dev.readRetryDiagnostics().controlNacks);
    TEST_ASSERT_EQUAL_UINT32(0, dev.readRetryDiagnostics().retries);
  }
}

void test_retry_does_not_replay_pec_timeout_or_failed_cleanup() {
  for (uint8_t scenario = 0; scenario < 4; ++scenario) {
    FakeE2Transport fake;
    Config cfg = retryConfig(fake);
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    if (scenario == 0 || scenario == 1) fake.setCorruptReadPec(true);
    if (scenario == 1 || scenario == 3) fake.nackNextReadMainCommand(cmd::MAIN_STATUS);
    if (scenario == 2) fake.stretchClockReleaseAfter(2, cfg.bitTimeoutUs + 1U);
    if (scenario == 3) fake.stretchClockReleaseAfter(11, cfg.bitTimeoutUs + 1U);
    uint8_t value = 0;
    const Status st = dev.readStatus(value);
    const Err expected[] = {Err::PEC_MISMATCH, Err::PEC_MISMATCH, Err::TIMEOUT, Err::NACK};
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(expected[scenario]), static_cast<uint8_t>(st.code));
    const ReadRetryDiagnostics stats = dev.readRetryDiagnostics();
    TEST_ASSERT_EQUAL_UINT32(scenario == 1 ? 1 : 0, stats.retries);
    TEST_ASSERT_EQUAL_UINT32(scenario == 1 || scenario == 3 ? 1 : 0, stats.controlNacks);
    TEST_ASSERT_EQUAL_UINT32(0, stats.recovered);
    TEST_ASSERT_EQUAL_UINT32(0, stats.exhausted);
    TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
    TEST_ASSERT_TRUE(fake.masterSclReleased());
    TEST_ASSERT_TRUE(fake.masterSdaReleased());
    if (scenario == 1) TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::PEC_MISMATCH), static_cast<uint8_t>(stats.lastError.code));
    if (scenario == 3) {
      TEST_ASSERT_TRUE(stats.cleanupBlocked);
      TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT), static_cast<uint8_t>(stats.lastCleanupError.code));
      TEST_ASSERT_EQUAL_INT32(cfg.bitTimeoutUs, stats.lastCleanupError.detail);
      TEST_ASSERT_EQUAL_UINT32(1, fake.controlCount(0x71));
    }
  }
}

void test_retry_guard_vetoes_before_and_after_pause() {
  const uint8_t vetoCalls[] = {1, 2, 3};
  for (uint8_t vetoCall : vetoCalls) {
    FakeE2Transport fake;
    Config cfg = retryConfig(fake);
    cfg.allowReadRetry = [](void* user) {
      return static_cast<FakeE2Transport*>(user)->allowReadRetry();
    };
    fake.vetoReadRetryOnCall(vetoCall);
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    fake.nackNextReadMainCommand(cmd::MAIN_STATUS, 4);
    fake.resetElapsed();
    uint8_t value = 0;
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK), static_cast<uint8_t>(dev.readStatus(value).code));
    TEST_ASSERT_EQUAL_UINT32(vetoCall == 1 ? 2500 : 3500, fake.elapsedUs());
    TEST_ASSERT_EQUAL_UINT32(1, fake.controlCount(0x71));
    const ReadRetryDiagnostics stats = dev.readRetryDiagnostics();
    TEST_ASSERT_TRUE(stats.retryVetoed);
    TEST_ASSERT_FALSE(stats.cleanupBlocked);
    TEST_ASSERT_EQUAL_UINT32(0, stats.retries);
    TEST_ASSERT_EQUAL_UINT8(0, stats.lastRetriesUsed);
    TEST_ASSERT_EQUAL_UINT32(0, stats.exhausted);
  }
}

void test_retry_rechecks_idle_after_pause() {
  FakeE2Transport fake;
  Config cfg = retryConfig(fake);
  cfg.allowReadRetry = [](void* user) {
    auto& bus = *static_cast<FakeE2Transport*>(user);
    (void)bus.allowReadRetry();
    if (bus.retryGuardCalls() == 2U) bus.setHoldSclLow(true);
    return true;
  };
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  fake.nackNextReadMainCommand(cmd::MAIN_STATUS);
  uint8_t value = 0;
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::NACK), static_cast<uint8_t>(dev.readStatus(value).code));
  const auto stats = dev.readRetryDiagnostics();
  TEST_ASSERT_TRUE(stats.cleanupBlocked);
  TEST_ASSERT_FALSE(stats.retryVetoed);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::BUS_STUCK), static_cast<uint8_t>(stats.lastCleanupError.code));
  TEST_ASSERT_EQUAL_UINT32(0, stats.retries);
  TEST_ASSERT_EQUAL_UINT32(1, fake.controlCount(0x71));
}

void test_high_byte_retry_preserves_latch_and_sticky_low_event() {
  const bool modes[] = {false, true};
  for (bool average : modes) {
    FakeE2Transport fake;
    Config cfg = retryConfig(fake);
    cfg.allowReadRetry = [](void* user) {
      auto& bus = *static_cast<FakeE2Transport*>(user);
      bus.setMv3(0xABCD);
      bus.setMv4(0xABCD);
      return true;
    };
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    fake.setMv3(0x1234);
    fake.setMv4(0x1234);
    const uint8_t low = average ? cmd::MAIN_MV4_LO : cmd::MAIN_MV3_LO;
    const uint8_t high = average ? cmd::MAIN_MV4_HI : cmd::MAIN_MV3_HI;
    fake.nackNextReadMainCommand(high, 3);
    uint16_t ppm = 0;
    TEST_ASSERT_TRUE((average ? dev.readCo2Average(ppm) : dev.readCo2Fast(ppm)).ok());
    TEST_ASSERT_EQUAL_UINT16(0x1234, ppm);
    TEST_ASSERT_EQUAL_UINT32(1, fake.controlCount(cmd::makeControlRead(low, 0)));
    TEST_ASSERT_EQUAL_UINT32(4, fake.controlCount(cmd::makeControlRead(high, 0)));
    TEST_ASSERT_EQUAL_UINT8(cmd::makeControlRead(high, 0), dev.readRetryDiagnostics().lastControlByte);
    uint8_t status = 0;
    TEST_ASSERT_TRUE(dev.readStatus(status).ok());
    TEST_ASSERT_EQUAL_UINT8(cmd::makeControlRead(high, 0), dev.readRetryDiagnostics().lastControlByte);
    fake.nackNextReadMainCommand(low, 1);
    TEST_ASSERT_TRUE((average ? dev.readCo2Average(ppm) : dev.readCo2Fast(ppm)).ok());
    TEST_ASSERT_EQUAL_UINT8(cmd::makeControlRead(low, 0), dev.readRetryDiagnostics().lastControlByte);
    TEST_ASSERT_TRUE(dev.readRetryDiagnostics().lastRecovered);
    TEST_ASSERT_EQUAL_UINT8(1, dev.readRetryDiagnostics().lastRetriesUsed);
    TEST_ASSERT_EQUAL_UINT32(2, dev.readRetryDiagnostics().recovered);
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_read_retry_config_rejects_invalid_without_io);
  RUN_TEST(test_measurement_status_retries_recover_on_each_allowed_attempt);
  RUN_TEST(test_retry_exhaustion_counts_one_health_failure_and_preserves_session);
  RUN_TEST(test_disabled_retry_still_counts_nack_and_metadata_keeps_event);
  RUN_TEST(test_retry_respects_smaller_configured_limit);
  RUN_TEST(test_retry_stops_on_later_transfer_timeout);
  RUN_TEST(test_retry_never_replays_identity_custom_reads_or_writes);
  RUN_TEST(test_retry_does_not_replay_pec_timeout_or_failed_cleanup);
  RUN_TEST(test_retry_guard_vetoes_before_and_after_pause);
  RUN_TEST(test_retry_rechecks_idle_after_pause);
  RUN_TEST(test_high_byte_retry_preserves_latch_and_sticky_low_event);
  RUN_TEST(test_transfer_failure_survives_cleanup_stop_timeout);
  RUN_TEST(test_failed_recover_clears_capabilities_and_latches_from_ready_or_degraded);
  RUN_TEST(test_offline_replay_marks_message_and_preserves_original_diagnostics);
  RUN_TEST(test_flash_stretch_config_boundaries);
  RUN_TEST(test_flash_write_stop_stretches_commit_and_verify);
  RUN_TEST(test_accepted_interval_low_byte_stop_failure_marks_dirty);
  RUN_TEST(test_read_and_pointer_stop_use_ordinary_timeout);
  RUN_TEST(test_bus_reset_flash_stretch_is_bounded_and_health_neutral);
  RUN_TEST(test_flash_budget_does_not_relax_bit_transfer_deadline);
  RUN_TEST(test_status_ok);
  RUN_TEST(test_status_error);
  RUN_TEST(test_status_in_progress);
  RUN_TEST(test_config_defaults);
  RUN_TEST(test_command_table_control_bytes_and_support);
  RUN_TEST(test_co2_error_code_names);
  RUN_TEST(test_begin_rejects_missing_callbacks);
  RUN_TEST(test_begin_rejects_invalid_device_address);
  RUN_TEST(test_begin_rejects_clock_timing_below_spec);
  RUN_TEST(test_begin_validates_generated_clock_period);
  RUN_TEST(test_begin_validates_nominal_byte_deadline);
  RUN_TEST(test_begin_rejects_timeouts_above_e2_limits);
  RUN_TEST(test_begin_bus_reset_reports_stuck_lines_precisely);
  RUN_TEST(test_begin_validates_full_ee871_identity_and_co2_capability);
  RUN_TEST(test_begin_recovers_sda_released_by_reset_clocks);
  RUN_TEST(test_begin_normalizes_zero_offline_threshold);
  RUN_TEST(test_default_health_aliases);
  RUN_TEST(test_probe_requires_begin);
  RUN_TEST(test_recover_requires_begin);
  RUN_TEST(test_high_level_helpers_check_initialization_first);
  RUN_TEST(test_fake_transport_begin_succeeds);
  RUN_TEST(test_feature_cache_failure_fails_begin_closed);
  RUN_TEST(test_persistent_write_ranges_precede_capability_checks);
  RUN_TEST(test_operating_mode_access_fails_closed);
  RUN_TEST(test_clock_stretch_timeout_is_bounded_and_tracked);
  RUN_TEST(test_byte_deadline_includes_nominal_phases_after_stretch);
  RUN_TEST(test_start_allows_configured_high_settle_before_sampling_sda);
  RUN_TEST(test_stop_timeout_releases_both_master_lines);
  RUN_TEST(test_bus_safety_checks_cover_sda_and_do_not_track_reset);
  RUN_TEST(test_start_requires_sda_high_to_low_edge);
  RUN_TEST(test_measurement_reads_cover_values_boundaries_and_high_byte_failure);
  RUN_TEST(test_pec_mismatch_probe_is_raw_but_tracked_read_updates_health);
  RUN_TEST(test_probe_validates_full_identity_without_health_side_effects);
  RUN_TEST(test_device_absent_probe_has_no_health_side_effect_tracked_read_fails);
  RUN_TEST(test_absent_sampling_burst_short_circuits_to_three_failures);
  RUN_TEST(test_custom_write_verify_mismatch_returns_precise_error);
  RUN_TEST(test_offline_threshold_and_recover_after_replug);
  RUN_TEST(test_failed_recovery_is_atomic_and_keeps_offline_latched);
  RUN_TEST(test_recovery_refreshes_feature_cache_atomically);
  RUN_TEST(test_adjacent_register_pairs_assemble_in_low_high_order);
  RUN_TEST(test_interval_low_byte_write_failure_does_not_dirty);
  RUN_TEST(test_interval_high_byte_write_failure_sets_dirty);
  RUN_TEST(test_interval_verify_failure_sets_dirty_and_unrelated_read_does_not_clear);
  RUN_TEST(test_co2_offset_high_byte_failure_sets_dirty);
  RUN_TEST(test_co2_offset_low_byte_verify_failure_sets_dirty);
  RUN_TEST(test_co2_gain_high_byte_failure_sets_dirty);
  RUN_TEST(test_co2_gain_low_byte_verify_failure_sets_dirty);
  RUN_TEST(test_part_name_first_byte_verify_failure_sets_dirty);
  RUN_TEST(test_dirty_error_preserves_first_failure);
  RUN_TEST(test_resync_persistent_config_clears_only_when_coherent);
  RUN_TEST(test_dirty_state_survives_offline);
  return UNITY_END();
}

