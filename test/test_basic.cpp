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
using EE871Test::StretchPhase;

static_assert(!std::is_copy_constructible_v<EE871::EE871>);
static_assert(!std::is_copy_assignable_v<EE871::EE871>);
static_assert(!std::is_move_constructible_v<EE871::EE871>);
static_assert(!std::is_move_assignable_v<EE871::EE871>);
static_assert(static_cast<uint8_t>(Err::VERIFY_MISMATCH) == 15);
static_assert(static_cast<uint8_t>(Err::OFFLINE) == 16);
static_assert(static_cast<uint8_t>(Err::CO2_SENSOR_ERROR) == 17);
static_assert(static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN) == 18);
static_assert(static_cast<uint8_t>(BeginPolicy::REQUIRE_PRESENT) == 0);
static_assert(static_cast<uint8_t>(BeginPolicy::ALLOW_ABSENT) == 1);
static_assert(static_cast<uint8_t>(OperationKind::CONTROL_READ) == 0);
static_assert(static_cast<uint8_t>(OperationKind::BUS_RESET) == 8);
static_assert(static_cast<uint8_t>(OperationKind::BEGIN_REQUIRE_PRESENT) == 9);
static_assert(static_cast<uint8_t>(OperationKind::BEGIN_ALLOW_ABSENT) == 10);
static_assert(static_cast<uint8_t>(OperationKind::PROBE_IDENTITY) == 11);
static_assert(
    static_cast<uint8_t>(
        OperationKind::RECOVER_IDENTITY_AND_CAPABILITIES) == 12);
static_assert(
    static_cast<uint8_t>(OperationKind::CHECKED_CO2_AVERAGE) == 13);
static_assert(
    static_cast<uint8_t>(OperationKind::CHECKED_CO2_FAST) == 14);
static_assert(
    static_cast<uint8_t>(OperationKind::CUSTOM_BLOCK_WRITE_VERIFY) == 15);
static_assert(
    static_cast<uint8_t>(OperationKind::RESYNC_PERSISTENT_CONFIG) == 16);
static_assert(
    static_cast<uint8_t>(OperationKind::AUTO_ADJUST_MAINTENANCE) == 17);
static_assert(
    static_cast<uint8_t>(OperationKind::BUS_ADDRESS_CHANGE) == 18);
static_assert(static_cast<uint8_t>(MutationTarget::NONE) == 0);
static_assert(static_cast<uint8_t>(MutationTarget::RAW_CUSTOM_BYTE) == 1);
static_assert(static_cast<uint8_t>(MutationTarget::PART_NAME) == 2);
static_assert(static_cast<uint8_t>(MutationTarget::BUS_ADDRESS) == 3);
static_assert(static_cast<uint8_t>(MutationTarget::GLOBAL_INTERVAL) == 4);
static_assert(static_cast<uint8_t>(MutationTarget::CO2_INTERVAL_FACTOR) == 5);
static_assert(static_cast<uint8_t>(MutationTarget::CO2_FILTER) == 6);
static_assert(static_cast<uint8_t>(MutationTarget::OPERATING_MODE) == 7);
static_assert(static_cast<uint8_t>(MutationTarget::AUTO_ADJUST) == 8);
static_assert(static_cast<uint8_t>(MutationTarget::CO2_OFFSET) == 9);
static_assert(static_cast<uint8_t>(MutationTarget::CO2_GAIN) == 10);
static_assert(static_cast<uint8_t>(MutationEffect::NONE) == 0);
static_assert(static_cast<uint8_t>(MutationEffect::NO_EFFECT) == 1);
static_assert(static_cast<uint8_t>(MutationEffect::ACKNOWLEDGED) == 2);
static_assert(static_cast<uint8_t>(MutationEffect::INDETERMINATE) == 3);
static_assert(static_cast<uint8_t>(MutationEffect::VERIFIED) == 4);
static_assert(static_cast<uint8_t>(MutationEffect::RESYNCHRONIZED) == 5);
static_assert(
    static_cast<uint8_t>(MutationEffect::OPERATOR_ACKNOWLEDGED) == 6);
static_assert(static_cast<uint8_t>(Co2ValueKind::FAST) == 0);
static_assert(static_cast<uint8_t>(Co2ValueKind::AVERAGE) == 1);
static_assert(static_cast<uint8_t>(Co2SensorError::NONE) == 0);
static_assert(
    static_cast<uint8_t>(Co2SensorError::SUPPLY_VOLTAGE_LOW) == 1);
static_assert(
    static_cast<uint8_t>(Co2SensorError::SENSOR_COUNTS_LOW) == 200);
static_assert(
    static_cast<uint8_t>(Co2SensorError::SENSOR_COUNTS_HIGH) == 201);
static_assert(
    static_cast<uint8_t>(
        Co2SensorError::SUPPLY_VOLTAGE_BREAKDOWN_AT_PEAK) == 202);
static_assert(static_cast<uint8_t>(Co2SensorError::UNKNOWN) == 255);
static_assert(cmd::STATUS_CO2_ERROR_MASK == 0x08);
static_assert(cmd::CO2_PPM_MIN == 0);
static_assert(cmd::CO2_PPM_MAX == 50000);
static_assert(cmd::CUSTOM_ADJUSTMENT_SUPPORT_RESERVED_MASK == 0xF0);
static_assert(
    cmd::CUSTOM_ADJUSTMENT_POINT_SUPPORT_RESERVED_MASK == 0xF0);
static_assert(
    cmd::CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT_RESERVED_MASK == 0xFE);
static_assert(
    cmd::CUSTOM_ADJUSTMENT_TIME_SUPPORT_RESERVED_MASK == 0xF0);
static_assert(cmd::OPERATING_FUNCTIONS_RESERVED_MASK == 0x08);
static_assert(cmd::OPERATING_MODE_SUPPORT_RESERVED_MASK == 0xFC);
static_assert(cmd::SPECIAL_FEATURES_RESERVED_MASK == 0xFE);
static_assert(cmd::OPERATING_MODE_RESERVED_MASK == 0xFC);
static_assert(cmd::AUTO_ADJUST_RESERVED_MASK == 0xFE);
static_assert(
    cmd::makeCapabilityValidationDetail(0x07, 0x08) == 0x0708);

using RawCo2ReadMethod =
    Status (EE871::EE871::*)(uint16_t&);
using RawByteReadMethod =
    Status (EE871::EE871::*)(uint8_t&);
static_assert(std::is_same_v<
              decltype(&EE871::EE871::readCo2Fast),
              RawCo2ReadMethod>);
static_assert(std::is_same_v<
              decltype(&EE871::EE871::readCo2Average),
              RawCo2ReadMethod>);
static_assert(std::is_same_v<
              decltype(&EE871::EE871::readStatus),
              RawByteReadMethod>);
static_assert(std::is_same_v<
              decltype(&EE871::EE871::readErrorCode),
              RawByteReadMethod>);
using MutationDiagnosticMethod =
    MutationDiagnostic (EE871::EE871::*)() const;
using AutoAdjustAcknowledgeMethod =
    Status (EE871::EE871::*)();
static_assert(std::is_same_v<
              decltype(&EE871::EE871::mutationDiagnostic),
              MutationDiagnosticMethod>);
static_assert(std::is_same_v<
              decltype(&EE871::EE871::acknowledgeAutoAdjustUncertainty),
              AutoAdjustAcknowledgeMethod>);

void setUp() {}
void tearDown() {}

static Status beginFakeDevice(EE871::EE871& dev,
                              FakeE2Transport& fake,
                              uint8_t offlineThreshold = 5) {
  Config cfg = fake.makeConfig(offlineThreshold);
  return dev.begin(cfg);
}

static Status beginAllowAbsent(EE871::EE871& dev,
                               FakeE2Transport& fake,
                               uint8_t offlineThreshold = 5) {
  Config cfg = fake.makeConfig(offlineThreshold);
  cfg.beginPolicy = BeginPolicy::ALLOW_ABSENT;
  return dev.begin(cfg);
}

static void forceOfflineByNack(
    EE871::EE871& dev, FakeE2Transport& fake) {
  fake.setDevicePresent(false);
  uint8_t status = 0;
  for (uint8_t i = 0; i < dev.offlineThreshold(); ++i) {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(dev.readStatus(status).code));
  }
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(DriverState::OFFLINE),
      static_cast<uint8_t>(dev.state()));
  fake.setDevicePresent(true);
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

static void assertMutationHeader(
    const EE871::EE871& dev,
    MutationTarget target,
    MutationEffect effect,
    bool unresolved,
    uint8_t firstAddress,
    uint8_t lastAddress,
    uint16_t requested,
    uint16_t acknowledged,
    uint16_t observed,
    uint16_t matched) {
  const MutationDiagnostic mutation = dev.mutationDiagnostic();
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(target),
      static_cast<uint8_t>(mutation.target));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(effect),
      static_cast<uint8_t>(mutation.effect));
  TEST_ASSERT_EQUAL(unresolved, mutation.unresolved);
  TEST_ASSERT_EQUAL_UINT8(firstAddress, mutation.firstAddress);
  TEST_ASSERT_EQUAL_UINT8(lastAddress, mutation.lastAddress);
  TEST_ASSERT_EQUAL_UINT16(requested, mutation.elementsRequested);
  TEST_ASSERT_EQUAL_UINT16(acknowledged, mutation.elementsAcknowledged);
  TEST_ASSERT_EQUAL_UINT16(observed, mutation.elementsObserved);
  TEST_ASSERT_EQUAL_UINT16(matched, mutation.elementsMatched);
}

static void assertNoBusActivity(const FakeE2Transport& fake) {
  TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
}

static void applyNearByteBudgetStretch(FakeE2Transport& fake) {
  fake.setStretch(StretchPhase::DATA_BIT, 4135U, 4096U);
}

static Config makeLargestValidTimingConfig(FakeE2Transport& fake) {
  Config cfg = fake.makeConfig();
  cfg.clockLowUs = 100;
  cfg.clockHighUs = 1890;
  cfg.startHoldUs = 65535;
  cfg.stopHoldUs = 65535;
  cfg.bitTimeoutUs = cmd::BIT_TIMEOUT_MAX_US;
  cfg.byteTimeoutUs = cmd::BYTE_TIMEOUT_MAX_US;
  cfg.writeDelayMs = cmd::WRITE_DELAY_MAX_MS;
  cfg.intervalWriteDelayMs = cmd::INTERVAL_WRITE_DELAY_MAX_MS;
  cfg.longDelaySliceMs = cmd::LONG_DELAY_SLICE_MAX_MS;
  return cfg;
}

static void assertWithinTimingBound(
    EE871::EE871& dev,
    FakeE2Transport& fake,
    OperationKind kind,
    uint16_t elementCount = 1) {
  OperationTimingBound bound;
  TEST_ASSERT_TRUE(
      dev.operationTimingBound(kind, elementCount, bound).ok());
  TEST_ASSERT_TRUE(
      fake.elapsedUs() <=
      static_cast<uint64_t>(bound.maxBlockingMs) * 1000U);
}

template <typename Call>
static void assertFirstBusTransferNack(Call call) {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.resetActivityCounters();
  fake.failAtTransferIndex(0);

  const Status st = call(dev);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NACK),
      static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_UINT32(1, fake.transactionCount());
}

static void assertIdentityInvalid(const DeviceIdentity& identity) {
  TEST_ASSERT_FALSE(identity.valid);
  TEST_ASSERT_FALSE(identity.co2Available);
  TEST_ASSERT_EQUAL_UINT16(0, identity.group);
  TEST_ASSERT_EQUAL_UINT8(0, identity.subgroup);
  TEST_ASSERT_EQUAL_UINT8(0, identity.availableMeasurements);
}

static void assertCapabilitiesInvalid(
    const CapabilitySnapshot& capabilities) {
  TEST_ASSERT_FALSE(capabilities.valid);
  TEST_ASSERT_EQUAL_UINT8(0, capabilities.customAdjustmentSupport);
  TEST_ASSERT_EQUAL_UINT8(0, capabilities.adjustmentPointSupport);
  TEST_ASSERT_EQUAL_UINT8(0, capabilities.adjustmentTimeGeneralSupport);
  TEST_ASSERT_EQUAL_UINT8(0, capabilities.adjustmentTimeSupport);
  TEST_ASSERT_EQUAL_UINT8(0, capabilities.operatingFunctions);
  TEST_ASSERT_EQUAL_UINT8(0, capabilities.operatingModeSupport);
  TEST_ASSERT_EQUAL_UINT8(0, capabilities.specialFeatures);
}

static void poisonCo2Result(Co2ReadResult& out) {
  out.kind = Co2ValueKind::FAST;
  out.ppm = 12345;
  out.ppmValid = true;
  out.statusByte = 0xFF;
  out.statusValid = true;
  out.co2Error = true;
  out.errorCode = 201;
  out.errorCodeValid = true;
  out.sensorError = Co2SensorError::SENSOR_COUNTS_HIGH;
  out.valueReadAttempted = true;
  out.statusReadAttempted = true;
  out.errorCodeReadAttempted = true;
  out.valueReadStatus =
      Status::Error(Err::TIMEOUT, "poison value", 1);
  out.statusReadStatus =
      Status::Error(Err::NACK, "poison status", 2);
  out.errorCodeReadStatus =
      Status::Error(Err::PEC_MISMATCH, "poison error", 3);
}

static void assertUnattemptedStatusAndErrorEvidence(
    const Co2ReadResult& out) {
  TEST_ASSERT_FALSE(out.statusReadAttempted);
  TEST_ASSERT_FALSE(out.statusValid);
  TEST_ASSERT_TRUE(out.statusReadStatus.ok());
  TEST_ASSERT_EQUAL_UINT8(0, out.statusByte);
  TEST_ASSERT_FALSE(out.co2Error);
  TEST_ASSERT_FALSE(out.errorCodeReadAttempted);
  TEST_ASSERT_FALSE(out.errorCodeValid);
  TEST_ASSERT_TRUE(out.errorCodeReadStatus.ok());
  TEST_ASSERT_EQUAL_UINT8(0, out.errorCode);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Co2SensorError::NONE),
      static_cast<uint8_t>(out.sensorError));
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
  TEST_ASSERT_NULL(cfg.delayMs);
  TEST_ASSERT_NULL(cfg.yield);
  TEST_ASSERT_EQUAL_UINT8(1, cfg.longDelaySliceMs);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(BeginPolicy::REQUIRE_PRESENT),
      static_cast<uint8_t>(cfg.beginPolicy));
}

void test_default_timing_config_operates_on_healthy_bus() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  const Config cfg = fake.makeDefaultTimingConfig();

  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  TEST_ASSERT_EQUAL_UINT16(100, dev.getConfig().startHoldUs);
  TEST_ASSERT_EQUAL_UINT16(100, dev.getConfig().stopHoldUs);

  uint8_t status = 0;
  TEST_ASSERT_TRUE(dev.readStatus(status).ok());
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
  TEST_ASSERT_FALSE(snap.persistentConfigDirty);
  TEST_ASSERT_TRUE(snap.persistentConfigDirtyError.ok());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(BeginPolicy::REQUIRE_PRESENT),
      static_cast<uint8_t>(snap.beginPolicy));
  TEST_ASSERT_TRUE(snap.beginProbeStatus.ok());
  assertIdentityInvalid(snap.identity);
  assertCapabilitiesInvalid(snap.capabilities);
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

void test_invalid_begin_policy_is_bus_silent_invalid_config() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  Config cfg = fake.makeConfig();
  cfg.beginPolicy = static_cast<BeginPolicy>(0xFF);
  fake.resetActivityCounters();
  fake.resetElapsed();
  OperationTimingBound bound;

  const Status queryStatus = EE871::EE871::operationTimingBound(
      cfg, OperationKind::BEGIN_REQUIRE_PRESENT, 1, bound);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(queryStatus.code));

  const Status st = dev.begin(cfg);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(st.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
  TEST_ASSERT_EQUAL_UINT64(0, fake.elapsedUs());
}

void test_strict_and_optional_absent_begin_contracts() {
  const BeginPolicy policies[] = {
      BeginPolicy::REQUIRE_PRESENT,
      BeginPolicy::ALLOW_ABSENT,
  };
  for (BeginPolicy policy : policies) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setDevicePresent(false);
    Config cfg = fake.makeConfig(3);
    cfg.beginPolicy = policy;
    fake.resetActivityCounters();
    fake.resetElapsed();

    const Status st = dev.begin(cfg);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::UNINIT),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_FALSE(dev.isOnline());
    TEST_ASSERT_EQUAL_UINT8(0, dev.consecutiveFailures());
    TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT32(0, dev.totalSuccess());
    TEST_ASSERT_TRUE(dev.lastError().ok());
    TEST_ASSERT_EQUAL_UINT32(0, dev.lastErrorMs());
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
    TEST_ASSERT_EQUAL_UINT32(1, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, false));
    TEST_ASSERT_EQUAL_UINT32(0, fake.longDelayUsCalls());
    TEST_ASSERT_EQUAL_UINT32(0, fake.delayMsTotalMs());
  }
}

void test_allow_absent_rejects_non_absence_transport_faults() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setStretch(
        StretchPhase::DATA_BIT, cmd::BIT_TIMEOUT_MAX_US + 5U);
    const Status st = beginAllowAbsent(dev, fake);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setHoldSclLow(true);
    const Status st = beginAllowAbsent(dev, fake);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::BUS_STUCK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setSdaStuckLow(true);
    const Status st = beginAllowAbsent(dev, fake);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::BUS_STUCK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCorruptReadPec(true);
    const Status st = beginAllowAbsent(dev, fake);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PEC_MISMATCH),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setDevicePresent(false);
    fake.setStretch(
        StretchPhase::STOP, cmd::BIT_TIMEOUT_MAX_US + 5U);

    const Status st = beginAllowAbsent(dev, fake);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::UNINIT),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT32(0, dev.totalSuccess());
    TEST_ASSERT_EQUAL_UINT32(1, fake.transactionCount());
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }
}

void test_all_lifecycle_identity_paths_fail_closed() {
  struct IdentityCase {
    uint16_t group;
    uint8_t subgroup;
    uint8_t availableMeasurements;
  };
  const IdentityCase cases[] = {
      {static_cast<uint16_t>(cmd::SENSOR_GROUP_ID + 1U),
       cmd::SENSOR_SUBGROUP_ID,
       cmd::AVAILABLE_MEAS_MASK},
      {cmd::SENSOR_GROUP_ID,
       static_cast<uint8_t>(cmd::SENSOR_SUBGROUP_ID + 1U),
       cmd::AVAILABLE_MEAS_MASK},
      {cmd::SENSOR_GROUP_ID, cmd::SENSOR_SUBGROUP_ID, 0},
  };

  for (const IdentityCase& identityCase : cases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setIdentity(
        identityCase.group,
        identityCase.subgroup,
        identityCase.availableMeasurements);

    const Status st = beginAllowAbsent(dev, fake);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    const SettingsSnapshot before = dev.getSettings();

    for (const IdentityCase& identityCase : cases) {
      fake.setIdentity(
          identityCase.group,
          identityCase.subgroup,
          identityCase.availableMeasurements);
      const Status st = dev.probe();
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::NOT_SUPPORTED),
          static_cast<uint8_t>(st.code));
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(before.state),
          static_cast<uint8_t>(dev.state()));
      TEST_ASSERT_EQUAL_UINT32(before.totalFailures, dev.totalFailures());
      TEST_ASSERT_EQUAL_UINT32(before.totalSuccess, dev.totalSuccess());
      TEST_ASSERT_TRUE(dev.identity().valid);
      TEST_ASSERT_TRUE(dev.capabilities().valid);
    }
  }

  for (const IdentityCase& identityCase : cases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
    forceOfflineByNack(dev, fake);
    fake.setIdentity(
        identityCase.group,
        identityCase.subgroup,
        identityCase.availableMeasurements);

    const Status st = dev.recover();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::OFFLINE),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_EQUAL_UINT32(1, dev.totalFailures());
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }
}

void test_begin_capability_load_is_complete_ordered_and_atomic() {
  for (uint32_t transferIndex = 4; transferIndex < 12; ++transferIndex) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.failAtTransferIndex(transferIndex);

    const Status st = beginFakeDevice(dev, fake);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_FALSE(dev.isInitialized());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::UNINIT),
        static_cast<uint8_t>(dev.state()));
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }

  FakeE2Transport fake;
  EE871::EE871 dev;
  fake.setIdentity(cmd::SENSOR_GROUP_ID, cmd::SENSOR_SUBGROUP_ID, 0xA8);
  fake.setCapabilities(0x0F, 0x0F, 0x01, 0x0F, 0x55, 0x03, 0x01);

  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  const DeviceIdentity identity = dev.identity();
  TEST_ASSERT_TRUE(identity.valid);
  TEST_ASSERT_TRUE(identity.co2Available);
  TEST_ASSERT_EQUAL_UINT16(cmd::SENSOR_GROUP_ID, identity.group);
  TEST_ASSERT_EQUAL_UINT8(cmd::SENSOR_SUBGROUP_ID, identity.subgroup);
  TEST_ASSERT_EQUAL_UINT8(0xA8, identity.availableMeasurements);

  const CapabilitySnapshot capabilities = dev.capabilities();
  TEST_ASSERT_TRUE(capabilities.valid);
  TEST_ASSERT_EQUAL_UINT8(0x0F, capabilities.customAdjustmentSupport);
  TEST_ASSERT_EQUAL_UINT8(0x0F, capabilities.adjustmentPointSupport);
  TEST_ASSERT_EQUAL_UINT8(0x01, capabilities.adjustmentTimeGeneralSupport);
  TEST_ASSERT_EQUAL_UINT8(0x0F, capabilities.adjustmentTimeSupport);
  TEST_ASSERT_EQUAL_UINT8(0x55, capabilities.operatingFunctions);
  TEST_ASSERT_EQUAL_UINT8(0x03, capabilities.operatingModeSupport);
  TEST_ASSERT_EQUAL_UINT8(0x01, capabilities.specialFeatures);
  const SettingsSnapshot settings = dev.getSettings();
  TEST_ASSERT_EQUAL_UINT8(0x55, settings.operatingFunctions);
  TEST_ASSERT_EQUAL_UINT8(0x03, settings.operatingModeSupport);
  TEST_ASSERT_EQUAL_UINT8(0x01, settings.specialFeatures);
  TEST_ASSERT_EQUAL_UINT32(12, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT8(cmd::MAIN_CUSTOM_PTR, fake.transactionMain(4));
  TEST_ASSERT_FALSE(fake.transactionIsRead(4));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::CUSTOM_ADJUSTMENT_SUPPORT, fake.transactionAddress(4));
  for (size_t i = 0; i < 7; ++i) {
    TEST_ASSERT_EQUAL_UINT8(
        cmd::MAIN_CUSTOM_PTR, fake.transactionMain(5 + i));
    TEST_ASSERT_TRUE(fake.transactionIsRead(5 + i));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(cmd::CUSTOM_ADJUSTMENT_SUPPORT + i),
        fake.transactionAddress(5 + i));
  }
}

void test_every_capability_reserved_bit_fails_closed_atomically() {
  struct CapabilityMask {
    uint8_t address;
    uint8_t reservedMask;
  };
  const CapabilityMask masks[] = {
      {cmd::CUSTOM_ADJUSTMENT_SUPPORT,
       cmd::CUSTOM_ADJUSTMENT_SUPPORT_RESERVED_MASK},
      {cmd::CUSTOM_ADJUSTMENT_POINT_SUPPORT,
       cmd::CUSTOM_ADJUSTMENT_POINT_SUPPORT_RESERVED_MASK},
      {cmd::CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT,
       cmd::CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT_RESERVED_MASK},
      {cmd::CUSTOM_ADJUSTMENT_TIME_SUPPORT,
       cmd::CUSTOM_ADJUSTMENT_TIME_SUPPORT_RESERVED_MASK},
      {cmd::CUSTOM_OPERATING_FUNCTIONS,
       cmd::OPERATING_FUNCTIONS_RESERVED_MASK},
      {cmd::CUSTOM_OPERATING_MODE_SUPPORT,
       cmd::OPERATING_MODE_SUPPORT_RESERVED_MASK},
      {cmd::CUSTOM_SPECIAL_FEATURES,
       cmd::SPECIAL_FEATURES_RESERVED_MASK},
  };

  for (const CapabilityMask& item : masks) {
    for (uint8_t bit = 0; bit < 8U; ++bit) {
      const uint8_t value = static_cast<uint8_t>(1U << bit);
      if ((item.reservedMask & value) == 0U) {
        continue;
      }

      FakeE2Transport fake;
      EE871::EE871 dev;
      fake.setMemory(item.address, value);

      const Status st = beginFakeDevice(dev, fake);

      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::NOT_SUPPORTED),
          static_cast<uint8_t>(st.code));
      TEST_ASSERT_EQUAL_INT32(
          cmd::makeCapabilityValidationDetail(item.address, value),
          st.detail);
      TEST_ASSERT_FALSE(dev.isInitialized());
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(DriverState::UNINIT),
          static_cast<uint8_t>(dev.state()));
      TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
      TEST_ASSERT_EQUAL_UINT32(0, dev.totalSuccess());
      assertIdentityInvalid(dev.identity());
      assertCapabilitiesInvalid(dev.capabilities());
      TEST_ASSERT_EQUAL_UINT32(12, fake.transactionCount());
    }
  }

  FakeE2Transport fake;
  EE871::EE871 dev;
  fake.setMemory(
      cmd::CUSTOM_OPERATING_FUNCTIONS,
      static_cast<uint8_t>(
          cmd::FEATURE_ADDRESS_CONFIG |
          cmd::OPERATING_FUNCTIONS_RESERVED_MASK));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NOT_SUPPORTED),
      static_cast<uint8_t>(beginFakeDevice(dev, fake).code));
  fake.resetActivityCounters();
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NOT_INITIALIZED),
      static_cast<uint8_t>(dev.writeBusAddress(1).code));
  assertNoBusActivity(fake);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(MutationTarget::NONE),
      static_cast<uint8_t>(dev.mutationDiagnostic().target));
}

void test_direct_capability_reads_share_masks_and_preserve_outputs() {
  struct CapabilityReadCase {
    uint8_t address;
    uint8_t invalidValue;
  };
  const CapabilityReadCase cases[] = {
      {cmd::CUSTOM_OPERATING_FUNCTIONS,
       cmd::OPERATING_FUNCTIONS_RESERVED_MASK},
      {cmd::CUSTOM_OPERATING_MODE_SUPPORT,
       cmd::OPERATING_MODE_SUPPORT_RESERVED_MASK},
      {cmd::CUSTOM_SPECIAL_FEATURES,
       cmd::SPECIAL_FEATURES_RESERVED_MASK},
  };

  for (const CapabilityReadCase& item : cases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(item.address, item.invalidValue);
    fake.resetActivityCounters();
    const uint32_t failuresBefore = dev.totalFailures();
    uint8_t output = 0xA5;

    Status st;
    if (item.address == cmd::CUSTOM_OPERATING_FUNCTIONS) {
      st = dev.readOperatingFunctions(output);
    } else if (
        item.address == cmd::CUSTOM_OPERATING_MODE_SUPPORT) {
      st = dev.readOperatingModeSupport(output);
    } else {
      st = dev.readSpecialFeatures(output);
    }

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(
        cmd::makeCapabilityValidationDetail(
            item.address, item.invalidValue),
        st.detail);
    TEST_ASSERT_EQUAL_UINT8(0xA5, output);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_EQUAL_UINT32(2, fake.transactionCount());
  }

  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.setMemory(cmd::CUSTOM_OPERATING_FUNCTIONS, 0x55);
  uint8_t output = 0;
  TEST_ASSERT_TRUE(dev.readOperatingFunctions(output).ok());
  TEST_ASSERT_EQUAL_UINT8(0x55, output);
}

void test_recovery_capability_semantics_clear_live_claims_from_every_state() {
  const DriverState startingStates[] = {
      DriverState::READY,
      DriverState::DEGRADED,
      DriverState::OFFLINE,
  };
  for (DriverState startingState : startingStates) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 2).ok());
    if (startingState == DriverState::DEGRADED) {
      fake.setDevicePresent(false);
      uint8_t status = 0;
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::NACK),
          static_cast<uint8_t>(dev.readStatus(status).code));
      fake.setDevicePresent(true);
    } else if (startingState == DriverState::OFFLINE) {
      forceOfflineByNack(dev, fake);
    }
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(startingState),
        static_cast<uint8_t>(dev.state()));
    const uint32_t failuresBefore = dev.totalFailures();
    fake.setMemory(
        cmd::CUSTOM_OPERATING_FUNCTIONS,
        cmd::OPERATING_FUNCTIONS_RESERVED_MASK);

    const Status st = dev.recover();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(
        cmd::makeCapabilityValidationDetail(
            cmd::CUSTOM_OPERATING_FUNCTIONS,
            cmd::OPERATING_FUNCTIONS_RESERVED_MASK),
        st.detail);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::OFFLINE),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(dev.lastError().code));
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }
}

void test_recover_rejects_each_invalid_capability_without_partial_cache() {
  const uint8_t addresses[] = {
      cmd::CUSTOM_ADJUSTMENT_SUPPORT,
      cmd::CUSTOM_ADJUSTMENT_POINT_SUPPORT,
      cmd::CUSTOM_ADJUSTMENT_TIME_GENERAL_SUPPORT,
      cmd::CUSTOM_ADJUSTMENT_TIME_SUPPORT,
      cmd::CUSTOM_OPERATING_FUNCTIONS,
      cmd::CUSTOM_OPERATING_MODE_SUPPORT,
      cmd::CUSTOM_SPECIAL_FEATURES,
  };
  const uint8_t invalidValues[] = {
      0x10, 0x10, 0x02, 0x10, 0x08, 0x04, 0x02,
  };
  for (size_t i = 0; i < sizeof(addresses); ++i) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    const uint32_t failuresBefore = dev.totalFailures();
    fake.setMemory(addresses[i], invalidValues[i]);

    const Status st = dev.recover();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(
        cmd::makeCapabilityValidationDetail(
            addresses[i], invalidValues[i]),
        st.detail);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::OFFLINE),
        static_cast<uint8_t>(dev.state()));
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }
}

void test_e2_spec_version_remains_diagnostic_only() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  fake.setMemory(cmd::CUSTOM_E2_SPEC_VERSION, 0x55);

  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  TEST_ASSERT_TRUE(dev.identity().valid);
  TEST_ASSERT_TRUE(dev.capabilities().valid);
  TEST_ASSERT_TRUE(dev.probe().ok());

  uint8_t version = 0;
  TEST_ASSERT_TRUE(dev.readE2SpecVersion(version).ok());
  TEST_ASSERT_EQUAL_UINT8(0x55, version);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(DriverState::READY),
      static_cast<uint8_t>(dev.state()));
}

void test_identity_capability_and_settings_access_is_bus_silent() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.resetActivityCounters();
  fake.resetElapsed();

  const DeviceIdentity identity = dev.identity();
  const CapabilitySnapshot capabilities = dev.capabilities();
  SettingsSnapshot settings;
  TEST_ASSERT_TRUE(dev.getSettings(settings).ok());
  const SettingsSnapshot settingsByValue = dev.getSettings();
  TEST_ASSERT_TRUE(dev.hasGlobalInterval());
  TEST_ASSERT_TRUE(identity.valid);
  TEST_ASSERT_TRUE(capabilities.valid);
  TEST_ASSERT_TRUE(settings.identity.valid);
  TEST_ASSERT_TRUE(settingsByValue.capabilities.valid);
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
  TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT64(0, fake.elapsedUs());
}

void test_checked_sample_public_contract_defaults() {
  Co2ReadResult out;

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Co2ValueKind::AVERAGE),
      static_cast<uint8_t>(out.kind));
  TEST_ASSERT_EQUAL_UINT16(0, out.ppm);
  TEST_ASSERT_FALSE(out.ppmValid);
  TEST_ASSERT_EQUAL_UINT8(0, out.statusByte);
  TEST_ASSERT_FALSE(out.statusValid);
  TEST_ASSERT_FALSE(out.co2Error);
  TEST_ASSERT_EQUAL_UINT8(0, out.errorCode);
  TEST_ASSERT_FALSE(out.errorCodeValid);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Co2SensorError::NONE),
      static_cast<uint8_t>(out.sensorError));
  TEST_ASSERT_FALSE(out.valueReadAttempted);
  TEST_ASSERT_FALSE(out.statusReadAttempted);
  TEST_ASSERT_FALSE(out.errorCodeReadAttempted);
  TEST_ASSERT_TRUE(out.valueReadStatus.ok());
  TEST_ASSERT_TRUE(out.statusReadStatus.ok());
  TEST_ASSERT_TRUE(out.errorCodeReadStatus.ok());

  RawCo2ReadMethod fast = &EE871::EE871::readCo2Fast;
  RawCo2ReadMethod average = &EE871::EE871::readCo2Average;
  RawByteReadMethod status = &EE871::EE871::readStatus;
  RawByteReadMethod errorCode = &EE871::EE871::readErrorCode;
  TEST_ASSERT_TRUE(fast != nullptr);
  TEST_ASSERT_TRUE(average != nullptr);
  TEST_ASSERT_TRUE(status != nullptr);
  TEST_ASSERT_TRUE(errorCode != nullptr);
}

void test_checked_average_and_fast_success_order_and_evidence() {
  struct Case {
    Co2ValueKind kind;
    uint16_t ppm;
    uint8_t lowMain;
    uint8_t highMain;
  };
  const Case cases[] = {
      {Co2ValueKind::AVERAGE, 1234, cmd::MAIN_MV4_LO, cmd::MAIN_MV4_HI},
      {Co2ValueKind::FAST, 4321, cmd::MAIN_MV3_LO, cmd::MAIN_MV3_HI},
  };

  for (const Case& item : cases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCo2AveragePpm(item.ppm);
    fake.setCo2FastPpm(item.ppm);
    fake.setStatusByte(0xA0);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();

    Co2ReadResult out;
    const Status st =
        item.kind == Co2ValueKind::AVERAGE
            ? dev.readCo2AverageSample(out)
            : dev.readCo2FastSample(out);

    TEST_ASSERT_TRUE(st.ok());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(item.kind),
        static_cast<uint8_t>(out.kind));
    TEST_ASSERT_EQUAL_UINT16(item.ppm, out.ppm);
    TEST_ASSERT_TRUE(out.ppmValid);
    TEST_ASSERT_EQUAL_UINT8(0xA0, out.statusByte);
    TEST_ASSERT_TRUE(out.statusValid);
    TEST_ASSERT_FALSE(out.co2Error);
    TEST_ASSERT_EQUAL_UINT8(0, out.errorCode);
    TEST_ASSERT_FALSE(out.errorCodeValid);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Co2SensorError::NONE),
        static_cast<uint8_t>(out.sensorError));
    TEST_ASSERT_TRUE(out.valueReadAttempted);
    TEST_ASSERT_TRUE(out.statusReadAttempted);
    TEST_ASSERT_FALSE(out.errorCodeReadAttempted);
    TEST_ASSERT_TRUE(out.valueReadStatus.ok());
    TEST_ASSERT_TRUE(out.statusReadStatus.ok());
    TEST_ASSERT_TRUE(out.errorCodeReadStatus.ok());

    TEST_ASSERT_EQUAL_UINT32(3, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT8(item.lowMain, fake.transactionMain(0));
    TEST_ASSERT_TRUE(fake.transactionIsRead(0));
    TEST_ASSERT_EQUAL_UINT8(item.highMain, fake.transactionMain(1));
    TEST_ASSERT_TRUE(fake.transactionIsRead(1));
    TEST_ASSERT_EQUAL_UINT8(cmd::MAIN_STATUS, fake.transactionMain(2));
    TEST_ASSERT_TRUE(fake.transactionIsRead(2));
    TEST_ASSERT_EQUAL_UINT32(1, fake.controlReadCount(item.lowMain));
    TEST_ASSERT_EQUAL_UINT32(1, fake.controlReadCount(item.highMain));
    TEST_ASSERT_EQUAL_UINT32(
        1, fake.controlReadCount(cmd::MAIN_STATUS));
    TEST_ASSERT_EQUAL_UINT32(3, dev.totalSuccess());
    TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
  }
}

void test_checked_value_and_status_failure_evidence() {
  for (uint32_t failIndex = 0; failIndex < 2; ++failIndex) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCo2AveragePpm(2345);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failAtTransferIndex(failIndex);
    Co2ReadResult out;
    poisonCo2Result(out);

    const Status st = dev.readCo2AverageSample(out);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertSameStatus(st, out.valueReadStatus);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Co2ValueKind::AVERAGE),
        static_cast<uint8_t>(out.kind));
    TEST_ASSERT_TRUE(out.valueReadAttempted);
    TEST_ASSERT_FALSE(out.ppmValid);
    TEST_ASSERT_EQUAL_UINT16(0, out.ppm);
    assertUnattemptedStatusAndErrorEvidence(out);
    TEST_ASSERT_EQUAL_UINT32(failIndex + 1U, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.controlReadCount(cmd::MAIN_STATUS));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCo2AveragePpm(2345);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failAtTransferIndex(2);
    Co2ReadResult out;
    poisonCo2Result(out);

    const Status st = dev.readCo2AverageSample(out);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_TRUE(out.valueReadAttempted);
    TEST_ASSERT_TRUE(out.valueReadStatus.ok());
    TEST_ASSERT_EQUAL_UINT16(2345, out.ppm);
    TEST_ASSERT_FALSE(out.ppmValid);
    TEST_ASSERT_TRUE(out.statusReadAttempted);
    assertSameStatus(st, out.statusReadStatus);
    TEST_ASSERT_FALSE(out.statusValid);
    TEST_ASSERT_EQUAL_UINT8(0, out.statusByte);
    TEST_ASSERT_FALSE(out.co2Error);
    TEST_ASSERT_FALSE(out.errorCodeReadAttempted);
    TEST_ASSERT_FALSE(out.errorCodeValid);
    TEST_ASSERT_TRUE(out.errorCodeReadStatus.ok());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Co2SensorError::NONE),
        static_cast<uint8_t>(out.sensorError));
    TEST_ASSERT_EQUAL_UINT32(3, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT32(
        1, fake.controlReadCount(cmd::MAIN_STATUS));
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.controlReadCount(cmd::MAIN_CUSTOM_PTR));
  }
}

void test_checked_sensor_error_mapping_and_order() {
  struct Case {
    uint8_t code;
    Co2SensorError expected;
  };
  const Case cases[] = {
      {1, Co2SensorError::SUPPLY_VOLTAGE_LOW},
      {200, Co2SensorError::SENSOR_COUNTS_LOW},
      {201, Co2SensorError::SENSOR_COUNTS_HIGH},
      {202, Co2SensorError::SUPPLY_VOLTAGE_BREAKDOWN_AT_PEAK},
      {0, Co2SensorError::UNKNOWN},
      {77, Co2SensorError::UNKNOWN},
  };

  for (const Case& item : cases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCo2AveragePpm(900);
    fake.setStatusByte(cmd::STATUS_CO2_ERROR_MASK);
    fake.setErrorCode(item.code);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();

    Co2ReadResult out;
    const Status st = dev.readCo2AverageSample(out);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::CO2_SENSOR_ERROR),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_STRING("CO2 sensor status error", st.msg);
    TEST_ASSERT_EQUAL_INT32(item.code, st.detail);
    TEST_ASSERT_EQUAL_UINT16(900, out.ppm);
    TEST_ASSERT_FALSE(out.ppmValid);
    TEST_ASSERT_TRUE(out.valueReadAttempted);
    TEST_ASSERT_TRUE(out.valueReadStatus.ok());
    TEST_ASSERT_TRUE(out.statusReadAttempted);
    TEST_ASSERT_TRUE(out.statusReadStatus.ok());
    TEST_ASSERT_TRUE(out.statusValid);
    TEST_ASSERT_EQUAL_UINT8(
        cmd::STATUS_CO2_ERROR_MASK, out.statusByte);
    TEST_ASSERT_TRUE(out.co2Error);
    TEST_ASSERT_TRUE(out.errorCodeReadAttempted);
    TEST_ASSERT_TRUE(out.errorCodeReadStatus.ok());
    TEST_ASSERT_TRUE(out.errorCodeValid);
    TEST_ASSERT_EQUAL_UINT8(item.code, out.errorCode);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(item.expected),
        static_cast<uint8_t>(out.sensorError));

    TEST_ASSERT_EQUAL_UINT32(5, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT8(cmd::MAIN_MV4_LO, fake.transactionMain(0));
    TEST_ASSERT_EQUAL_UINT8(cmd::MAIN_MV4_HI, fake.transactionMain(1));
    TEST_ASSERT_EQUAL_UINT8(cmd::MAIN_STATUS, fake.transactionMain(2));
    TEST_ASSERT_EQUAL_UINT8(cmd::MAIN_CUSTOM_PTR, fake.transactionMain(3));
    TEST_ASSERT_FALSE(fake.transactionIsRead(3));
    TEST_ASSERT_TRUE(fake.transactionHasAddress(3));
    TEST_ASSERT_EQUAL_UINT8(
        cmd::CUSTOM_ERROR_CODE, fake.transactionAddress(3));
    TEST_ASSERT_EQUAL_UINT8(cmd::MAIN_CUSTOM_PTR, fake.transactionMain(4));
    TEST_ASSERT_TRUE(fake.transactionIsRead(4));
    TEST_ASSERT_TRUE(fake.transactionHasAddress(4));
    TEST_ASSERT_EQUAL_UINT8(
        cmd::CUSTOM_ERROR_CODE, fake.transactionAddress(4));
    TEST_ASSERT_FALSE(fake.pointerReadStartedEarly());
  }
}

void test_checked_sensor_error_capability_gate() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  fake.setCapabilities(
      cmd::FEATURE_CO2_CUSTOM_ADJUSTMENT,
      cmd::FEATURE_CO2_ADJUSTMENT_POINT,
      cmd::FEATURE_CUSTOM_ADJUSTMENT_TIME_GENERAL,
      cmd::FEATURE_CO2_ADJUSTMENT_TIME,
      cmd::FEATURE_SERIAL_NUMBER,
      cmd::MODE_SUPPORT_LOW_POWER,
      cmd::SPECIAL_FEATURE_AUTO_ADJUST);
  fake.setCo2FastPpm(800);
  fake.setStatusByte(0x18);
  fake.setErrorCode(200);
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  TEST_ASSERT_FALSE(dev.hasErrorCode());
  fake.resetActivityCounters();

  Co2ReadResult out;
  const Status st = dev.readCo2FastSample(out);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::CO2_SENSOR_ERROR),
      static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_INT32(0x18, st.detail);
  TEST_ASSERT_TRUE(out.statusValid);
  TEST_ASSERT_TRUE(out.co2Error);
  TEST_ASSERT_FALSE(out.ppmValid);
  TEST_ASSERT_FALSE(out.errorCodeReadAttempted);
  TEST_ASSERT_FALSE(out.errorCodeValid);
  TEST_ASSERT_TRUE(out.errorCodeReadStatus.ok());
  TEST_ASSERT_EQUAL_UINT8(0, out.errorCode);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Co2SensorError::UNKNOWN),
      static_cast<uint8_t>(out.sensorError));
  TEST_ASSERT_EQUAL_UINT32(3, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT32(
      0, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, false));
  TEST_ASSERT_EQUAL_UINT32(
      0, fake.controlReadCount(cmd::MAIN_CUSTOM_PTR));
}

void test_checked_error_code_transfer_failures_are_precise() {
  for (uint32_t failIndex = 3; failIndex <= 4; ++failIndex) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCo2AveragePpm(777);
    fake.setStatusByte(cmd::STATUS_CO2_ERROR_MASK);
    fake.setErrorCode(202);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failAtTransferIndex(failIndex);
    const uint32_t failuresBefore = dev.totalFailures();

    Co2ReadResult out;
    const Status st = dev.readCo2AverageSample(out);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_TRUE(out.valueReadStatus.ok());
    TEST_ASSERT_TRUE(out.statusReadStatus.ok());
    TEST_ASSERT_TRUE(out.statusValid);
    TEST_ASSERT_TRUE(out.co2Error);
    TEST_ASSERT_EQUAL_UINT16(777, out.ppm);
    TEST_ASSERT_FALSE(out.ppmValid);
    TEST_ASSERT_TRUE(out.errorCodeReadAttempted);
    assertSameStatus(st, out.errorCodeReadStatus);
    TEST_ASSERT_FALSE(out.errorCodeValid);
    TEST_ASSERT_EQUAL_UINT8(0, out.errorCode);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Co2SensorError::UNKNOWN),
        static_cast<uint8_t>(out.sensorError));
    TEST_ASSERT_EQUAL_UINT32(failIndex + 1U, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT32(
        failIndex == 3U ? 0U : 1U,
        fake.controlReadCount(cmd::MAIN_CUSTOM_PTR));
    TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::DEGRADED),
        static_cast<uint8_t>(dev.state()));
  }
}

void test_checked_range_raw_compatibility_and_health_domains() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCo2AveragePpm(cmd::CO2_PPM_MAX);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

    Co2ReadResult out;
    TEST_ASSERT_TRUE(dev.readCo2AverageSample(out).ok());
    TEST_ASSERT_TRUE(out.ppmValid);
    TEST_ASSERT_EQUAL_UINT16(cmd::CO2_PPM_MAX, out.ppm);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCo2AveragePpm(
        static_cast<uint16_t>(cmd::CO2_PPM_MAX + 1U));
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    const uint32_t failuresBefore = dev.totalFailures();
    const uint32_t successesBefore = dev.totalSuccess();

    Co2ReadResult out;
    const Status st = dev.readCo2AverageSample(out);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_STRING("CO2 ppm out of range", st.msg);
    TEST_ASSERT_EQUAL_INT32(cmd::CO2_PPM_MAX + 1U, st.detail);
    TEST_ASSERT_EQUAL_UINT16(cmd::CO2_PPM_MAX + 1U, out.ppm);
    TEST_ASSERT_FALSE(out.ppmValid);
    TEST_ASSERT_TRUE(out.statusValid);
    TEST_ASSERT_FALSE(out.co2Error);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT32(successesBefore + 3U, dev.totalSuccess());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    const uint16_t rawPpm =
        static_cast<uint16_t>(cmd::CO2_PPM_MAX + 1U);
    fake.setCo2AveragePpm(rawPpm);
    fake.setCo2FastPpm(rawPpm);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

    uint16_t average = 0;
    uint16_t fast = 0;
    TEST_ASSERT_TRUE(dev.readCo2Average(average).ok());
    TEST_ASSERT_TRUE(dev.readCo2Fast(fast).ok());
    TEST_ASSERT_EQUAL_UINT16(rawPpm, average);
    TEST_ASSERT_EQUAL_UINT16(rawPpm, fast);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setStatusByte(cmd::STATUS_CO2_ERROR_MASK);
    fake.setErrorCode(1);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    const uint32_t failuresBefore = dev.totalFailures();
    const uint32_t successesBefore = dev.totalSuccess();

    Co2ReadResult out;
    const Status st = dev.readCo2FastSample(out);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::CO2_SENSOR_ERROR),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT32(successesBefore + 5U, dev.totalSuccess());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
  }
}

void test_checked_methods_are_bus_silent_while_offline() {
  const Co2ValueKind kinds[] = {
      Co2ValueKind::AVERAGE,
      Co2ValueKind::FAST,
  };

  for (Co2ValueKind kind : kinds) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
    forceOfflineByNack(dev, fake);
    fake.resetActivityCounters();
    fake.resetElapsed();
    const uint32_t failuresBefore = dev.totalFailures();
    const uint32_t successesBefore = dev.totalSuccess();
    Co2ReadResult out;
    poisonCo2Result(out);

    const Status st =
        kind == Co2ValueKind::AVERAGE
            ? dev.readCo2AverageSample(out)
            : dev.readCo2FastSample(out);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OFFLINE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(kind),
        static_cast<uint8_t>(out.kind));
    TEST_ASSERT_TRUE(out.valueReadAttempted);
    assertSameStatus(st, out.valueReadStatus);
    TEST_ASSERT_FALSE(out.ppmValid);
    TEST_ASSERT_EQUAL_UINT16(0, out.ppm);
    assertUnattemptedStatusAndErrorEvidence(out);
    TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
    TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
    TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT64(0, fake.elapsedUs());
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT32(successesBefore, dev.totalSuccess());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::OFFLINE),
        static_cast<uint8_t>(dev.state()));
  }
}

void test_co2_calibration_capability_helpers_are_bus_silent() {
  struct Case {
    uint8_t offsetGain;
    uint8_t points;
    bool expectOffsetGain;
    bool expectPoints;
  };
  const Case cases[] = {
      {cmd::FEATURE_CO2_CUSTOM_ADJUSTMENT, 0, true, false},
      {0, cmd::FEATURE_CO2_ADJUSTMENT_POINT, false, true},
  };

  for (const Case& item : cases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setCapabilities(
        item.offsetGain,
        item.points,
        0,
        0,
        0,
        0,
        0);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.resetElapsed();

    TEST_ASSERT_EQUAL(
        item.expectOffsetGain, dev.hasCo2OffsetGain());
    TEST_ASSERT_EQUAL(
        item.expectPoints, dev.hasCo2AdjustmentPoints());
    TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
    TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
    TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT64(0, fake.elapsedUs());
  }
}

void test_checked_timing_bounds_are_bus_silent_and_conservative() {
  const OperationKind kinds[] = {
      OperationKind::CHECKED_CO2_AVERAGE,
      OperationKind::CHECKED_CO2_FAST,
  };

  {
    FakeE2Transport fake;
    const Config cfg = fake.makeConfig();
    for (OperationKind kind : kinds) {
      OperationTimingBound bound;
      fake.resetActivityCounters();
      fake.resetElapsed();
      TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
          cfg, kind, 1, bound).ok());
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(kind),
          static_cast<uint8_t>(bound.kind));
      TEST_ASSERT_EQUAL_UINT16(1, bound.elementCount);
      TEST_ASSERT_EQUAL_UINT32(936, bound.maxBlockingMs);
      TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
      TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
      TEST_ASSERT_EQUAL_UINT64(0, fake.elapsedUs());
    }
  }

  for (OperationKind kind : kinds) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    fake.setStatusByte(cmd::STATUS_CO2_ERROR_MASK);
    fake.setErrorCode(202);
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.resetElapsed();
    fake.setStretch(StretchPhase::DATA_BIT, 4135, 128);
    OperationTimingBound bound;
    TEST_ASSERT_TRUE(dev.operationTimingBound(kind, 1, bound).ok());
    TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
    TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());

    Co2ReadResult out;
    const Status st =
        kind == OperationKind::CHECKED_CO2_AVERAGE
            ? dev.readCo2AverageSample(out)
            : dev.readCo2FastSample(out);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::CO2_SENSOR_ERROR),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(5, fake.transactionCount());
    TEST_ASSERT_FALSE(fake.pointerReadStartedEarly());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <=
        static_cast<uint64_t>(bound.maxBlockingMs) * 1000U);
  }
}

void test_clock_stretch_timeout_is_bounded_and_tracked() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.resetElapsed();
  fake.setStretch(StretchPhase::DATA_BIT, cmd::BIT_TIMEOUT_MAX_US + 5U);
  uint8_t status = 0;
  Status st = dev.readStatus(status);

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::TIMEOUT),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_TRUE(fake.elapsedUs() <= 26000U);
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

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::VERIFY_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verification mismatch", st.msg);
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

void test_offline_normal_operations_are_bus_silent_and_cannot_revive() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
  forceOfflineByNack(dev, fake);
  fake.resetActivityCounters();
  fake.resetElapsed();

  const uint32_t failuresBefore = dev.totalFailures();
  const uint32_t successesBefore = dev.totalSuccess();
  uint8_t value = 0;
  Status st = dev.readStatus(value);
  assertSameStatus(
      Status::Error(Err::OFFLINE, "Driver is offline; call recover()"), st);
  st = dev.customWrite(cmd::CUSTOM_FILTER_CO2, 4);
  assertSameStatus(
      Status::Error(Err::OFFLINE, "Driver is offline; call recover()"), st);
  st = dev.writeCo2Filter(4);
  assertSameStatus(
      Status::Error(Err::OFFLINE, "Driver is offline; call recover()"), st);

  TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
  TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT64(0, fake.elapsedUs());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBefore, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(DriverState::OFFLINE),
      static_cast<uint8_t>(dev.state()));
}

void test_runtime_failure_offline_has_same_explicit_recovery_latch() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 2).ok());
  fake.setDevicePresent(false);
  uint8_t value = 0;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NACK),
      static_cast<uint8_t>(dev.readStatus(value).code));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NACK),
      static_cast<uint8_t>(dev.readStatus(value).code));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(DriverState::OFFLINE),
      static_cast<uint8_t>(dev.state()));

  fake.setDevicePresent(true);
  fake.resetActivityCounters();
  const uint32_t failuresBefore = dev.totalFailures();
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::OFFLINE),
      static_cast<uint8_t>(dev.readStatus(value).code));
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());

  TEST_ASSERT_TRUE(dev.checkBusIdle().ok());
  TEST_ASSERT_TRUE(dev.busReset().ok());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(DriverState::OFFLINE),
      static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());

  fake.resetActivityCounters();
  TEST_ASSERT_TRUE(dev.probe().ok());
  TEST_ASSERT_EQUAL_UINT32(4, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(DriverState::OFFLINE),
      static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
}

void test_offline_probe_is_health_cache_and_diagnostic_neutral() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
  forceOfflineByNack(dev, fake);
  const SettingsSnapshot before = dev.getSettings();
  fake.resetActivityCounters();

  TEST_ASSERT_TRUE(dev.probe().ok());

  const SettingsSnapshot after = dev.getSettings();
  TEST_ASSERT_EQUAL_UINT32(4, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(before.state),
      static_cast<uint8_t>(after.state));
  TEST_ASSERT_EQUAL_UINT32(before.totalFailures, after.totalFailures);
  TEST_ASSERT_EQUAL_UINT32(before.totalSuccess, after.totalSuccess);
  TEST_ASSERT_EQUAL_UINT32(before.lastOkMs, after.lastOkMs);
  TEST_ASSERT_EQUAL_UINT32(before.lastErrorMs, after.lastErrorMs);
  TEST_ASSERT_EQUAL_UINT8(
      before.consecutiveFailures, after.consecutiveFailures);
  assertSameStatus(before.lastError, after.lastError);
  assertSameStatus(before.beginProbeStatus, after.beginProbeStatus);
  TEST_ASSERT_TRUE(after.identity.valid);
  TEST_ASSERT_TRUE(after.capabilities.valid);

  fake.setGroup(static_cast<uint16_t>(cmd::SENSOR_GROUP_ID + 1U));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NOT_SUPPORTED),
      static_cast<uint8_t>(dev.probe().code));
  const SettingsSnapshot afterFailure = dev.getSettings();
  TEST_ASSERT_EQUAL_UINT32(after.totalFailures, afterFailure.totalFailures);
  TEST_ASSERT_EQUAL_UINT32(after.totalSuccess, afterFailure.totalSuccess);
  assertSameStatus(after.beginProbeStatus, afterFailure.beginProbeStatus);
  TEST_ASSERT_TRUE(afterFailure.identity.valid);
  TEST_ASSERT_TRUE(afterFailure.capabilities.valid);
}

void test_successful_recover_reloads_and_atomically_publishes_cache() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
  forceOfflineByNack(dev, fake);
  fake.setIdentity(cmd::SENSOR_GROUP_ID, cmd::SENSOR_SUBGROUP_ID, 0x88);
  fake.setCapabilities(1, 2, 1, 4, 5, 2, 1);

  TEST_ASSERT_TRUE(dev.recover().ok());

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(DriverState::READY),
      static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0, dev.consecutiveFailures());
  TEST_ASSERT_TRUE(dev.identity().valid);
  TEST_ASSERT_EQUAL_UINT8(0x88, dev.identity().availableMeasurements);
  TEST_ASSERT_TRUE(dev.capabilities().valid);
  TEST_ASSERT_EQUAL_UINT8(1, dev.capabilities().customAdjustmentSupport);
  TEST_ASSERT_EQUAL_UINT8(2, dev.capabilities().adjustmentPointSupport);
  TEST_ASSERT_EQUAL_UINT8(
      1, dev.capabilities().adjustmentTimeGeneralSupport);
  TEST_ASSERT_EQUAL_UINT8(4, dev.capabilities().adjustmentTimeSupport);
  TEST_ASSERT_EQUAL_UINT8(5, dev.capabilities().operatingFunctions);
  TEST_ASSERT_EQUAL_UINT8(2, dev.capabilities().operatingModeSupport);
  TEST_ASSERT_EQUAL_UINT8(1, dev.capabilities().specialFeatures);
  TEST_ASSERT_TRUE(dev.getSettings().beginProbeStatus.ok());
}

void test_offline_recovery_reset_and_every_reload_failure_stays_offline() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
    forceOfflineByNack(dev, fake);
    fake.setHoldSclLow(true);
    fake.resetActivityCounters();

    const Status st = dev.recover();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::BUS_STUCK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::OFFLINE),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_TRUE(dev.consecutiveFailures() >= dev.offlineThreshold());
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }

  for (uint32_t transferIndex = 0; transferIndex < 12; ++transferIndex) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 1).ok());
    forceOfflineByNack(dev, fake);
    fake.resetActivityCounters();
    fake.failAtTransferIndex(transferIndex);

    const Status st = dev.recover();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::OFFLINE),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_TRUE(dev.consecutiveFailures() >= dev.offlineThreshold());
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }
}

void test_degraded_recovery_uses_transfer_health_and_semantic_latch() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 5).ok());
    fake.setDevicePresent(false);
    dev.tick(100);
    uint8_t value = 0;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(dev.readStatus(value).code));
    const uint32_t failuresBefore = dev.totalFailures();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::DEGRADED),
        static_cast<uint8_t>(dev.state()));

    fake.setDevicePresent(true);
    fake.resetActivityCounters();
    fake.failAtTransferIndex(11);
    const Status st = dev.recover();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::DEGRADED),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_EQUAL_UINT8(1, dev.consecutiveFailures());
    TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
    TEST_ASSERT_TRUE(dev.identity().valid);
    TEST_ASSERT_TRUE(dev.capabilities().valid);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 5).ok());
    fake.setDevicePresent(false);
    dev.tick(100);
    uint8_t value = 0;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(dev.readStatus(value).code));
    const uint32_t failuresBefore = dev.totalFailures();
    const uint32_t errorTimeBefore = dev.lastErrorMs();
    fake.setDevicePresent(true);
    fake.setSubgroup(
        static_cast<uint8_t>(cmd::SENSOR_SUBGROUP_ID + 1U));
    dev.tick(200);

    const Status st = dev.recover();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::OFFLINE),
        static_cast<uint8_t>(dev.state()));
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT32(errorTimeBefore, dev.lastErrorMs());
    TEST_ASSERT_TRUE(dev.consecutiveFailures() >= dev.offlineThreshold());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(dev.lastError().code));
    assertIdentityInvalid(dev.identity());
    assertCapabilitiesInvalid(dev.capabilities());
  }
}

void test_end_clears_policy_runtime_diagnostics_and_caches() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake, 3).ok());

  dev.end();

  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(DriverState::UNINIT),
      static_cast<uint8_t>(dev.state()));
  TEST_ASSERT_EQUAL_UINT8(0, dev.consecutiveFailures());
  TEST_ASSERT_EQUAL_UINT32(0, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(0, dev.totalSuccess());
  assertIdentityInvalid(dev.identity());
  assertCapabilitiesInvalid(dev.capabilities());
  const SettingsSnapshot settings = dev.getSettings();
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(BeginPolicy::REQUIRE_PRESENT),
      static_cast<uint8_t>(settings.beginPolicy));
  TEST_ASSERT_TRUE(settings.beginProbeStatus.ok());
  assertIdentityInvalid(settings.identity);
  assertCapabilitiesInvalid(settings.capabilities);
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

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OUT_OF_RANGE),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Interval must be 150-36000 (15-3600s)", st.msg);
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

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::VERIFY_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verification mismatch", st.msg);
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

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::VERIFY_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verification mismatch", st.msg);
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

  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::VERIFY_MISMATCH),
                          static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verification mismatch", st.msg);
  assertDirtyWithOriginalError(dev, st);
}

void test_dirty_error_preserves_first_failure() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.dropNextWriteCommitToAddress(cmd::CUSTOM_INTERVAL_H);
  Status first = dev.writeMeasurementInterval(300);
  TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(Err::OUT_OF_RANGE),
                          static_cast<uint8_t>(first.code));
  assertDirtyWithOriginalError(dev, first);

  fake.failNextWriteToAddress(cmd::CUSTOM_CO2_GAIN_H);
  Status second = dev.writeCo2Gain(0x5678);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
                          static_cast<uint8_t>(second.code));
  fake.resetActivityCounters();
  second = dev.writeCo2Gain(0x5678);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
      static_cast<uint8_t>(second.code));
  assertNoBusActivity(fake);
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

  fake.resetActivityCounters();
  st = dev.writeCo2Filter(1);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
      static_cast<uint8_t>(st.code));
  assertNoBusActivity(fake);
  assertDirtyWithOriginalError(dev, dirtyCause);
}

void test_mutation_contract_defaults_and_single_byte_effect_phases() {
  {
    EE871::EE871 dev;
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_FALSE(mutation.unresolved);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationTarget::NONE),
        static_cast<uint8_t>(mutation.target));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationEffect::NONE),
        static_cast<uint8_t>(mutation.effect));
    TEST_ASSERT_EQUAL_UINT16(0, mutation.elementsRequested);
    TEST_ASSERT_EQUAL_UINT16(0, mutation.elementsAcknowledged);
    TEST_ASSERT_EQUAL_UINT16(0, mutation.elementsObserved);
    TEST_ASSERT_EQUAL_UINT16(0, mutation.elementsMatched);
    TEST_ASSERT_FALSE(mutation.preObservedValueValid);
    TEST_ASSERT_FALSE(mutation.observedValueValid);
    TEST_ASSERT_TRUE(mutation.cause.ok());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    TEST_ASSERT_TRUE(dev.customWrite(0x20, 0x5A).ok());
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::VERIFIED,
        false,
        0x20,
        0x20,
        1,
        1,
        1,
        1);
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_EQUAL_UINT8(0x5A, mutation.attemptedValue);
    TEST_ASSERT_TRUE(mutation.observedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0x5A, mutation.observedValue);
    TEST_ASSERT_TRUE(mutation.cause.ok());
    TEST_ASSERT_EQUAL_UINT32(3, fake.transactionCount());
    TEST_ASSERT_FALSE(dev.persistentConfigDirty());
    TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    const Status st = dev.writeBusAddress(8);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    assertNoBusActivity(fake);
    assertMutationHeader(
        dev,
        MutationTarget::NONE,
        MutationEffect::NONE,
        false,
        0,
        0,
        0,
        0,
        0,
        0);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setCompletionStretches(
        cmd::WRITE_DELAY_PROTOCOL_MIN_MS * 1000U - 100U,
        105U);
    const Status st = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::ACKNOWLEDGED,
        true,
        0x20,
        0x20,
        1,
        1,
        0,
        0);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.nackNextFinalAck();
    fake.setCompletionStretches(
        cmd::WRITE_DELAY_PROTOCOL_MIN_MS * 1000U - 100U,
        105U);
    const Status st = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::NO_EFFECT,
        false,
        0x20,
        0x20,
        1,
        0,
        0,
        0);
    TEST_ASSERT_EQUAL_UINT8(0, fake.memory(0x20));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failNextWriteToAddress(0x20);
    const Status st = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::NO_EFFECT,
        false,
        0x20,
        0x20,
        1,
        0,
        0,
        0);
    assertSameStatus(st, dev.mutationDiagnostic().cause);
    TEST_ASSERT_FALSE(dev.persistentConfigDirty());
    TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setStretch(
        StretchPhase::FINAL_ACK,
        cmd::WRITE_DELAY_PROTOCOL_MIN_MS * 1000U + 5U);
    const Status st = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::INDETERMINATE,
        true,
        0x20,
        0x20,
        1,
        0,
        0,
        0);
    assertSameStatus(st, dev.mutationDiagnostic().cause);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setStretch(
        StretchPhase::STOP,
        cmd::WRITE_DELAY_PROTOCOL_MIN_MS * 1000U + 5U);
    const Status st = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::ACKNOWLEDGED,
        true,
        0x20,
        0x20,
        1,
        1,
        0,
        0);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setStretch(
        StretchPhase::FINAL_ACK,
        cmd::WRITE_DELAY_PROTOCOL_MIN_MS * 1000U);
    const Status st = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_TRUE(st.ok());
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::VERIFIED,
        false,
        0x20,
        0x20,
        1,
        1,
        1,
        1);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failAtTransferIndex(1);
    const Status st = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::ACKNOWLEDGED,
        true,
        0x20,
        0x20,
        1,
        1,
        0,
        0);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setMemory(0x20, 0xA5);
    fake.dropNextWriteCommitToAddress(0x20);
    const Status st = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0xA5, st.detail);
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::ACKNOWLEDGED,
        true,
        0x20,
        0x20,
        1,
        1,
        1,
        0);
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.observedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0xA5, mutation.observedValue);
    assertSameStatus(st, mutation.cause);
  }
}

void test_typed_mutations_share_exact_target_evidence() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
    for (uint8_t i = 0; i < cmd::CUSTOM_PART_NAME_LEN; ++i) {
      partName[i] = static_cast<uint8_t>('A' + i);
    }
    TEST_ASSERT_TRUE(dev.writePartName(partName).ok());
    assertMutationHeader(
        dev,
        MutationTarget::PART_NAME,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_PART_NAME_START,
        static_cast<uint8_t>(
            cmd::CUSTOM_PART_NAME_START + cmd::CUSTOM_PART_NAME_LEN - 1U),
        cmd::CUSTOM_PART_NAME_LEN,
        cmd::CUSTOM_PART_NAME_LEN,
        cmd::CUSTOM_PART_NAME_LEN,
        cmd::CUSTOM_PART_NAME_LEN);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());
    assertMutationHeader(
        dev,
        MutationTarget::GLOBAL_INTERVAL,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_INTERVAL_L,
        cmd::CUSTOM_INTERVAL_H,
        2,
        2,
        2,
        2);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(dev.writeCo2IntervalFactor(-3).ok());
    assertMutationHeader(
        dev,
        MutationTarget::CO2_INTERVAL_FACTOR,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_CO2_INTERVAL_FACTOR,
        cmd::CUSTOM_CO2_INTERVAL_FACTOR,
        1,
        1,
        1,
        1);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(dev.writeCo2Filter(7).ok());
    assertMutationHeader(
        dev,
        MutationTarget::CO2_FILTER,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_FILTER_CO2,
        cmd::CUSTOM_FILTER_CO2,
        1,
        1,
        1,
        1);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(dev.writeOperatingMode(3).ok());
    assertMutationHeader(
        dev,
        MutationTarget::OPERATING_MODE,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_OPERATING_MODE,
        cmd::CUSTOM_OPERATING_MODE,
        1,
        1,
        1,
        1);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(dev.startAutoAdjust().ok());
    assertMutationHeader(
        dev,
        MutationTarget::AUTO_ADJUST,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_AUTO_ADJUST,
        cmd::CUSTOM_AUTO_ADJUST,
        1,
        1,
        1,
        1);
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.preObservedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0, mutation.preObservedValue);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(dev.writeCo2Offset(-321).ok());
    assertMutationHeader(
        dev,
        MutationTarget::CO2_OFFSET,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_CO2_OFFSET_L,
        cmd::CUSTOM_CO2_OFFSET_H,
        2,
        2,
        2,
        2);
    int16_t observed = 0;
    TEST_ASSERT_TRUE(dev.readCo2Offset(observed).ok());
    TEST_ASSERT_EQUAL_INT16(-321, observed);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(dev.writeCo2Gain(0x1234).ok());
    assertMutationHeader(
        dev,
        MutationTarget::CO2_GAIN,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_CO2_GAIN_L,
        cmd::CUSTOM_CO2_GAIN_H,
        2,
        2,
        2,
        2);
  }
}

void test_part_name_failures_retain_exact_element_counts() {
  uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
  for (uint8_t i = 0; i < cmd::CUSTOM_PART_NAME_LEN; ++i) {
    partName[i] = static_cast<uint8_t>(0x40U + i);
  }

  for (uint8_t failedElement = 0;
       failedElement < cmd::CUSTOM_PART_NAME_LEN;
       ++failedElement) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.failNextWriteToAddress(
        static_cast<uint8_t>(
            cmd::CUSTOM_PART_NAME_START + failedElement));

    const Status st = dev.writePartName(partName);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::PART_NAME,
        failedElement == 0U
            ? MutationEffect::NO_EFFECT
            : MutationEffect::ACKNOWLEDGED,
        failedElement != 0U,
        cmd::CUSTOM_PART_NAME_START,
        static_cast<uint8_t>(
            cmd::CUSTOM_PART_NAME_START + cmd::CUSTOM_PART_NAME_LEN - 1U),
        cmd::CUSTOM_PART_NAME_LEN,
        failedElement,
        failedElement,
        failedElement);
    TEST_ASSERT_EQUAL_UINT8(
        partName[failedElement],
        dev.mutationDiagnostic().attemptedValue);
  }
}

void test_interval_and_co2_pairs_report_exact_partial_progress() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failNextWriteToAddress(cmd::CUSTOM_INTERVAL_L);
    const Status st = dev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::GLOBAL_INTERVAL,
        MutationEffect::NO_EFFECT,
        false,
        cmd::CUSTOM_INTERVAL_L,
        cmd::CUSTOM_INTERVAL_H,
        2,
        0,
        0,
        0);
    TEST_ASSERT_EQUAL_UINT32(1, fake.transactionCount());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failNextWriteToAddress(cmd::CUSTOM_INTERVAL_H);
    const Status st = dev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::GLOBAL_INTERVAL,
        MutationEffect::ACKNOWLEDGED,
        true,
        cmd::CUSTOM_INTERVAL_L,
        cmd::CUSTOM_INTERVAL_H,
        2,
        1,
        0,
        0);
    TEST_ASSERT_EQUAL_UINT32(2, fake.transactionCount());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failAtTransferIndex(2);
    const Status st = dev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::GLOBAL_INTERVAL,
        MutationEffect::ACKNOWLEDGED,
        true,
        cmd::CUSTOM_INTERVAL_L,
        cmd::CUSTOM_INTERVAL_H,
        2,
        2,
        0,
        0);
    TEST_ASSERT_EQUAL_UINT32(3, fake.transactionCount());
  }

  const struct PairCase {
    MutationTarget target;
    uint8_t firstAddress;
    uint8_t secondAddress;
    bool gain;
  } pairCases[] = {
      {MutationTarget::CO2_OFFSET,
       cmd::CUSTOM_CO2_OFFSET_L,
       cmd::CUSTOM_CO2_OFFSET_H,
       false},
      {MutationTarget::CO2_GAIN,
       cmd::CUSTOM_CO2_GAIN_L,
       cmd::CUSTOM_CO2_GAIN_H,
       true},
  };
  for (const PairCase& item : pairCases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failNextWriteToAddress(item.secondAddress);
    const Status st = item.gain
                          ? dev.writeCo2Gain(0x1234)
                          : dev.writeCo2Offset(0x1234);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        item.target,
        MutationEffect::ACKNOWLEDGED,
        true,
        item.firstAddress,
        item.secondAddress,
        2,
        1,
        1,
        1);
    TEST_ASSERT_EQUAL_UINT32(4, fake.transactionCount());
  }
}

void test_unresolved_mutation_blocks_mutations_but_allows_reads() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.dropNextWriteCommitToAddress(0x20);
  const Status first = dev.customWrite(0x20, 0x5A);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::VERIFY_MISMATCH),
      static_cast<uint8_t>(first.code));
  const MutationDiagnostic retained = dev.mutationDiagnostic();

  const uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
  fake.resetActivityCounters();
  const Status blocked[] = {
      dev.customWrite(0x21, 1),
      dev.writePartName(partName),
      dev.writeBusAddress(1),
      dev.writeMeasurementInterval(300),
      dev.writeCo2IntervalFactor(1),
      dev.writeCo2Filter(1),
      dev.writeOperatingMode(1),
      dev.startAutoAdjust(),
      dev.writeCo2Offset(1),
      dev.writeCo2Gain(1),
  };
  for (const Status& st : blocked) {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
        static_cast<uint8_t>(st.code));
  }
  assertNoBusActivity(fake);
  assertSameStatus(retained.cause, dev.mutationDiagnostic().cause);

  uint8_t status = 0;
  uint16_t ppm = 0;
  uint8_t raw = 0;
  Co2ReadResult checked;
  TEST_ASSERT_TRUE(dev.readStatus(status).ok());
  TEST_ASSERT_TRUE(dev.readCo2Average(ppm).ok());
  TEST_ASSERT_TRUE(dev.customRead(0x21, raw).ok());
  TEST_ASSERT_TRUE(dev.readCo2FastSample(checked).ok());
  TEST_ASSERT_TRUE(fake.transactionCount() > 0U);
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  assertSameStatus(retained.cause, dev.mutationDiagnostic().cause);

  TEST_ASSERT_TRUE(dev.probe().ok());
  TEST_ASSERT_TRUE(dev.checkBusIdle().ok());
  TEST_ASSERT_TRUE(dev.busReset().ok());
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  assertSameStatus(retained.cause, dev.mutationDiagnostic().cause);
}

void test_target_resync_preserves_first_cause_and_records_outcome() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(0x20);
    const Status first = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(first.code));

    fake.resetActivityCounters();
    fake.failAtTransferIndex(0);
    const Status resyncFailure = dev.resyncPersistentConfig();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(resyncFailure.code));
    TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
    assertSameStatus(first, dev.mutationDiagnostic().cause);

    fake.setMemory(0x20, 0x5A);
    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::VERIFIED,
        false,
        0x20,
        0x20,
        1,
        1,
        1,
        1);
    assertSameStatus(first, dev.mutationDiagnostic().cause);
    TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(0x20);
    const Status first = dev.customWrite(0x20, 0x5A);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(first.code));
    fake.setMemory(0x20, 0x33);

    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertMutationHeader(
        dev,
        MutationTarget::RAW_CUSTOM_BYTE,
        MutationEffect::RESYNCHRONIZED,
        false,
        0x20,
        0x20,
        1,
        1,
        1,
        0);
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.observedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0x33, mutation.observedValue);
    assertSameStatus(first, mutation.cause);
    TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());
  }
}

void test_every_ordinary_target_uses_target_specific_resync() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
    for (uint8_t i = 0; i < cmd::CUSTOM_PART_NAME_LEN; ++i) {
      partName[i] = static_cast<uint8_t>('a' + i);
    }
    fake.dropNextWriteCommitToAddress(
        cmd::CUSTOM_PART_NAME_START);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(dev.writePartName(partName).code));
    for (uint8_t i = 0; i < cmd::CUSTOM_PART_NAME_LEN; ++i) {
      fake.setMemory(
          static_cast<uint8_t>(cmd::CUSTOM_PART_NAME_START + i),
          partName[i]);
    }
    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertMutationHeader(
        dev,
        MutationTarget::PART_NAME,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_PART_NAME_START,
        static_cast<uint8_t>(
            cmd::CUSTOM_PART_NAME_START + cmd::CUSTOM_PART_NAME_LEN - 1U),
        cmd::CUSTOM_PART_NAME_LEN,
        1,
        cmd::CUSTOM_PART_NAME_LEN,
        cmd::CUSTOM_PART_NAME_LEN);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(
        cmd::CUSTOM_CO2_INTERVAL_FACTOR);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(
            dev.writeCo2IntervalFactor(-3).code));
    fake.setMemory(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0xFD);
    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertMutationHeader(
        dev,
        MutationTarget::CO2_INTERVAL_FACTOR,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_CO2_INTERVAL_FACTOR,
        cmd::CUSTOM_CO2_INTERVAL_FACTOR,
        1,
        1,
        1,
        1);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(cmd::CUSTOM_FILTER_CO2);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(dev.writeCo2Filter(9).code));
    fake.setMemory(cmd::CUSTOM_FILTER_CO2, 9);
    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertMutationHeader(
        dev,
        MutationTarget::CO2_FILTER,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_FILTER_CO2,
        cmd::CUSTOM_FILTER_CO2,
        1,
        1,
        1,
        1);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(
        cmd::CUSTOM_OPERATING_MODE);
    const Status first = dev.writeOperatingMode(3);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(first.code));
    fake.setMemory(cmd::CUSTOM_OPERATING_MODE, 4);
    const Status incompatible = dev.resyncPersistentConfig();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(incompatible.code));
    TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
    assertSameStatus(first, dev.mutationDiagnostic().cause);
    fake.setMemory(cmd::CUSTOM_OPERATING_MODE, 3);
    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertMutationHeader(
        dev,
        MutationTarget::OPERATING_MODE,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_OPERATING_MODE,
        cmd::CUSTOM_OPERATING_MODE,
        1,
        1,
        1,
        1);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(cmd::CUSTOM_INTERVAL_H);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(
            dev.writeMeasurementInterval(300).code));
    fake.setMemory(cmd::CUSTOM_INTERVAL_L, 0x2C);
    fake.setMemory(cmd::CUSTOM_INTERVAL_H, 0x01);
    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertMutationHeader(
        dev,
        MutationTarget::GLOBAL_INTERVAL,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_INTERVAL_L,
        cmd::CUSTOM_INTERVAL_H,
        2,
        2,
        2,
        2);
  }

  const struct PairCase {
    MutationTarget target;
    uint8_t firstAddress;
    uint8_t secondAddress;
    bool gain;
  } pairCases[] = {
      {MutationTarget::CO2_OFFSET,
       cmd::CUSTOM_CO2_OFFSET_L,
       cmd::CUSTOM_CO2_OFFSET_H,
       false},
      {MutationTarget::CO2_GAIN,
       cmd::CUSTOM_CO2_GAIN_L,
       cmd::CUSTOM_CO2_GAIN_H,
       true},
  };
  for (const PairCase& item : pairCases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(item.firstAddress);
    const Status st = item.gain
                          ? dev.writeCo2Gain(0x1234)
                          : dev.writeCo2Offset(0x1234);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(st.code));
    fake.setMemory(item.firstAddress, 0x34);
    fake.setMemory(item.secondAddress, 0x12);
    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertMutationHeader(
        dev,
        item.target,
        MutationEffect::VERIFIED,
        false,
        item.firstAddress,
        item.secondAddress,
        2,
        1,
        2,
        2);
  }
}

void test_recover_and_cache_getters_do_not_hide_uncertainty() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.dropNextWriteCommitToAddress(0x20);
  const Status first = dev.customWrite(0x20, 0x5A);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::VERIFY_MISMATCH),
      static_cast<uint8_t>(first.code));

  fake.resetActivityCounters();
  TEST_ASSERT_TRUE(dev.recover().ok());
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  assertSameStatus(first, dev.mutationDiagnostic().cause);
  TEST_ASSERT_TRUE(fake.transactionCount() > 0U);

  fake.resetActivityCounters();
  const MutationDiagnostic mutation = dev.mutationDiagnostic();
  SettingsSnapshot out;
  TEST_ASSERT_TRUE(dev.getSettings(out).ok());
  const SettingsSnapshot byValue = dev.getSettings();
  assertNoBusActivity(fake);
  TEST_ASSERT_TRUE(mutation.unresolved);
  TEST_ASSERT_TRUE(out.mutation.unresolved);
  TEST_ASSERT_TRUE(byValue.mutation.unresolved);
  assertSameStatus(first, out.mutation.cause);
  assertSameStatus(first, byValue.mutation.cause);
}

void test_bus_address_change_requires_explicit_candidate_session() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  Config original = fake.makeConfig();
  TEST_ASSERT_TRUE(dev.begin(original).ok());
  fake.resetActivityCounters();

  const Status write = dev.writeBusAddress(3);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
      static_cast<uint8_t>(write.code));
  assertMutationHeader(
      dev,
      MutationTarget::BUS_ADDRESS,
      MutationEffect::ACKNOWLEDGED,
      true,
      cmd::CUSTOM_BUS_ADDRESS,
      cmd::CUSTOM_BUS_ADDRESS,
      1,
      1,
      0,
      0);
  TEST_ASSERT_EQUAL_UINT8(0, dev.getConfig().deviceAddress);
  TEST_ASSERT_EQUAL_UINT32(1, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT32(
      1, fake.countTransactionsAtDeviceAddress(0));
  TEST_ASSERT_EQUAL_UINT32(
      0, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, true));

  fake.resetActivityCounters();
  const Status oldSessionResync = dev.resyncPersistentConfig();
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
      static_cast<uint8_t>(oldSessionResync.code));
  assertNoBusActivity(fake);

  dev.end();
  Config candidate = original;
  candidate.deviceAddress = 3;
  fake.resetActivityCounters();
  const Status wrongSession = dev.begin(candidate);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NACK),
      static_cast<uint8_t>(wrongSession.code));
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  TEST_ASSERT_EQUAL_UINT32(
      fake.transactionCount(),
      fake.countTransactionsAtDeviceAddress(3));

  fake.setRespondingDeviceAddress(3);
  fake.resetActivityCounters();
  TEST_ASSERT_TRUE(dev.begin(candidate).ok());
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  TEST_ASSERT_EQUAL_UINT32(
      fake.transactionCount(),
      fake.countTransactionsAtDeviceAddress(3));

  fake.resetActivityCounters();
  TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
  assertMutationHeader(
      dev,
      MutationTarget::BUS_ADDRESS,
      MutationEffect::VERIFIED,
      false,
      cmd::CUSTOM_BUS_ADDRESS,
      cmd::CUSTOM_BUS_ADDRESS,
      1,
      1,
      1,
      1);
  TEST_ASSERT_EQUAL_UINT32(
      fake.transactionCount(),
      fake.countTransactionsAtDeviceAddress(3));
  TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());
}

void test_auto_adjust_observation_resync_and_operator_acknowledgement() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.failAtTransferIndex(0);
    const Status st = dev.startAutoAdjust();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false));
    assertMutationHeader(
        dev,
        MutationTarget::NONE,
        MutationEffect::NONE,
        false,
        0,
        0,
        0,
        0,
        0,
        0);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(
        cmd::CUSTOM_AUTO_ADJUST,
        cmd::AUTO_ADJUST_RUNNING_MASK);
    fake.resetActivityCounters();
    const Status st = dev.startAutoAdjust();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::BUSY),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false));
    TEST_ASSERT_FALSE(dev.mutationDiagnostic().unresolved);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(cmd::CUSTOM_AUTO_ADJUST, 0);
    fake.dropNextWriteCommitToAddress(cmd::CUSTOM_AUTO_ADJUST);
    fake.resetActivityCounters();
    const Status st = dev.startAutoAdjust();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
        static_cast<uint8_t>(st.code));
    assertMutationHeader(
        dev,
        MutationTarget::AUTO_ADJUST,
        MutationEffect::ACKNOWLEDGED,
        true,
        cmd::CUSTOM_AUTO_ADJUST,
        cmd::CUSTOM_AUTO_ADJUST,
        1,
        1,
        1,
        0);
    MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.preObservedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0, mutation.preObservedValue);
    TEST_ASSERT_TRUE(mutation.observedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0, mutation.observedValue);
    TEST_ASSERT_EQUAL_UINT32(
        1, fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false));

    fake.resetActivityCounters();
    const Status prematureAck =
        dev.acknowledgeAutoAdjustUncertainty();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::INVALID_PARAM),
        static_cast<uint8_t>(prematureAck.code));
    assertNoBusActivity(fake);

    const uint32_t writesBefore =
        fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false);
    const Status unresolved = dev.resyncPersistentConfig();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
        static_cast<uint8_t>(unresolved.code));
    TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
    TEST_ASSERT_EQUAL_UINT32(
        writesBefore,
        fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false));

    fake.resetActivityCounters();
    TEST_ASSERT_TRUE(dev.acknowledgeAutoAdjustUncertainty().ok());
    assertNoBusActivity(fake);
    assertMutationHeader(
        dev,
        MutationTarget::AUTO_ADJUST,
        MutationEffect::OPERATOR_ACKNOWLEDGED,
        false,
        cmd::CUSTOM_AUTO_ADJUST,
        cmd::CUSTOM_AUTO_ADJUST,
        1,
        1,
        1,
        0);
    TEST_ASSERT_TRUE(dev.persistentConfigDirtyError().ok());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(cmd::CUSTOM_AUTO_ADJUST, 0);
    fake.dropNextWriteCommitToAddress(cmd::CUSTOM_AUTO_ADJUST);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
        static_cast<uint8_t>(dev.startAutoAdjust().code));
    fake.setMemory(
        cmd::CUSTOM_AUTO_ADJUST,
        cmd::AUTO_ADJUST_RUNNING_MASK);
    fake.resetActivityCounters();

    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false));
    assertMutationHeader(
        dev,
        MutationTarget::AUTO_ADJUST,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_AUTO_ADJUST,
        cmd::CUSTOM_AUTO_ADJUST,
        1,
        1,
        1,
        1);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    const Status st = dev.acknowledgeAutoAdjustUncertainty();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::INVALID_PARAM),
        static_cast<uint8_t>(st.code));
    assertNoBusActivity(fake);
  }
}

void test_unresolved_mutation_survives_end_and_begin_attempts() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  Config cfg = fake.makeConfig();
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  fake.dropNextWriteCommitToAddress(0x20);
  const Status cause = dev.customWrite(0x20, 0x5A);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::VERIFY_MISMATCH),
      static_cast<uint8_t>(cause.code));

  fake.resetActivityCounters();
  const Status repeatedBegin = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::ALREADY_INITIALIZED),
      static_cast<uint8_t>(repeatedBegin.code));
  assertNoBusActivity(fake);
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  assertSameStatus(cause, dev.mutationDiagnostic().cause);

  dev.end();
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  assertSameStatus(cause, dev.mutationDiagnostic().cause);

  fake.setDevicePresent(false);
  const Status failedBegin = dev.begin(cfg);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NACK),
      static_cast<uint8_t>(failedBegin.code));
  TEST_ASSERT_FALSE(dev.isInitialized());
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  assertSameStatus(cause, dev.mutationDiagnostic().cause);

  fake.setDevicePresent(true);
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
  assertSameStatus(cause, dev.mutationDiagnostic().cause);
  TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
  TEST_ASSERT_FALSE(dev.mutationDiagnostic().unresolved);
}

void test_typed_capability_guards_are_bus_silent() {
  FakeE2Transport fake;
  fake.setCapabilities(0, 0, 0, 0, 0, 0, 0);
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.resetActivityCounters();

  uint8_t serial[cmd::CUSTOM_SERIAL_LEN] = {};
  uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
  uint8_t address = 6;
  uint16_t interval = 1234;
  int8_t factor = 42;
  uint8_t filter = 43;
  uint8_t mode = 44;
  bool autoAdjustRunning = true;
  int16_t offsetValue = 123;
  uint16_t gainValue = 456;
  uint16_t lower = 789;
  uint16_t upper = 987;

  const Status statuses[] = {
      dev.readSerialNumber(serial),
      dev.readPartName(partName),
      dev.writePartName(partName),
      dev.readBusAddress(address),
      dev.writeBusAddress(1),
      dev.readMeasurementInterval(interval),
      dev.writeMeasurementInterval(300),
      dev.readCo2IntervalFactor(factor),
      dev.writeCo2IntervalFactor(1),
      dev.readCo2Filter(filter),
      dev.writeCo2Filter(1),
      dev.readOperatingMode(mode),
      dev.writeOperatingMode(0),
      dev.readAutoAdjustStatus(autoAdjustRunning),
      dev.startAutoAdjust(),
      dev.readCo2Offset(offsetValue),
      dev.writeCo2Offset(1),
      dev.readCo2Gain(gainValue),
      dev.writeCo2Gain(1),
      dev.readCo2CalPoints(lower, upper),
  };
  for (const Status& status : statuses) {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(status.code));
  }

  TEST_ASSERT_EQUAL_UINT8(6, address);
  TEST_ASSERT_EQUAL_UINT16(1234, interval);
  TEST_ASSERT_EQUAL_INT8(42, factor);
  TEST_ASSERT_EQUAL_UINT8(43, filter);
  TEST_ASSERT_EQUAL_UINT8(44, mode);
  TEST_ASSERT_TRUE(autoAdjustRunning);
  TEST_ASSERT_EQUAL_INT16(123, offsetValue);
  TEST_ASSERT_EQUAL_UINT16(456, gainValue);
  TEST_ASSERT_EQUAL_UINT16(789, lower);
  TEST_ASSERT_EQUAL_UINT16(987, upper);
  assertNoBusActivity(fake);
  TEST_ASSERT_FALSE(dev.mutationDiagnostic().unresolved);

  TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
  assertNoBusActivity(fake);

  const Status invalidStatuses[] = {
      dev.writeBusAddress(cmd::BUS_ADDRESS_MAX + 1U),
      dev.writeMeasurementInterval(cmd::INTERVAL_MIN_DECISEC - 1U),
      dev.writeOperatingMode(0x04),
  };
  for (const Status& status : invalidStatuses) {
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(status.code));
  }
  assertNoBusActivity(fake);

  {
    FakeE2Transport partialFake;
    partialFake.setCapabilities(
        0,
        0,
        0,
        0,
        cmd::FEATURE_FILTER_CONFIG,
        0,
        0);
    EE871::EE871 partialDev;
    TEST_ASSERT_TRUE(beginFakeDevice(partialDev, partialFake).ok());
    partialFake.setMemory(cmd::CUSTOM_FILTER_CO2, 3);
    partialFake.resetActivityCounters();

    TEST_ASSERT_TRUE(partialDev.resyncPersistentConfig().ok());
    TEST_ASSERT_EQUAL_UINT32(2, partialFake.transactionCount());
    TEST_ASSERT_EQUAL_UINT32(
        1,
        partialFake.countTransactions(cmd::MAIN_CUSTOM_PTR, false));
    TEST_ASSERT_EQUAL_UINT32(
        1,
        partialFake.countTransactions(cmd::MAIN_CUSTOM_PTR, true));
    TEST_ASSERT_EQUAL_UINT8(
        cmd::CUSTOM_FILTER_CO2, partialFake.transactionAddress(0));
  }

  {
    FakeE2Transport modeFake;
    EE871::EE871 modeDev;
    TEST_ASSERT_TRUE(beginFakeDevice(modeDev, modeFake).ok());
    modeFake.dropNextWriteCommitToAddress(
        cmd::CUSTOM_OPERATING_MODE);
    const Status first = modeDev.writeOperatingMode(3);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(first.code));
    const Status cause = modeDev.mutationDiagnostic().cause;

    modeDev.end();
    modeFake.setCapabilities(0, 0, 0, 0, 0, 0, 0);
    TEST_ASSERT_TRUE(beginFakeDevice(modeDev, modeFake).ok());
    modeFake.resetActivityCounters();
    const Status resync = modeDev.resyncPersistentConfig();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(resync.code));
    assertNoBusActivity(modeFake);
    TEST_ASSERT_TRUE(modeDev.mutationDiagnostic().unresolved);
    assertSameStatus(cause, modeDev.mutationDiagnostic().cause);
  }
}

void test_typed_persisted_reads_fail_closed_without_health_penalty() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(cmd::CUSTOM_BUS_ADDRESS, 8);
    const uint32_t failuresBefore = dev.totalFailures();
    uint8_t output = 6;
    const Status st = dev.readBusAddress(output);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(8, st.detail);
    TEST_ASSERT_EQUAL_UINT8(6, output);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
  }

  const uint16_t invalidIntervals[] = {
      static_cast<uint16_t>(cmd::INTERVAL_MIN_DECISEC - 1U),
      static_cast<uint16_t>(cmd::INTERVAL_MAX_DECISEC + 1U),
  };
  for (uint16_t invalid : invalidIntervals) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(
        cmd::CUSTOM_INTERVAL_L,
        static_cast<uint8_t>(invalid & 0xFFU));
    fake.setMemory(
        cmd::CUSTOM_INTERVAL_H,
        static_cast<uint8_t>(invalid >> 8));
    const uint32_t failuresBefore = dev.totalFailures();
    uint16_t output = 1234;
    const Status st = dev.readMeasurementInterval(output);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(invalid, st.detail);
    TEST_ASSERT_EQUAL_UINT16(1234, output);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0);
    const uint32_t failuresBefore = dev.totalFailures();
    int8_t output = 42;
    const Status st = dev.readCo2IntervalFactor(output);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0, st.detail);
    TEST_ASSERT_EQUAL_INT8(42, output);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(cmd::CUSTOM_OPERATING_MODE, 0x04);
    const uint32_t failuresBefore = dev.totalFailures();
    uint8_t output = 0xA5;
    const Status st = dev.readOperatingMode(output);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0x04, st.detail);
    TEST_ASSERT_EQUAL_UINT8(0xA5, output);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
  }

  {
    FakeE2Transport fake;
    fake.setCapabilities(
        cmd::FEATURE_CO2_CUSTOM_ADJUSTMENT,
        cmd::FEATURE_CO2_ADJUSTMENT_POINT,
        cmd::FEATURE_CUSTOM_ADJUSTMENT_TIME_GENERAL,
        cmd::FEATURE_CO2_ADJUSTMENT_TIME,
        0x55,
        cmd::MODE_SUPPORT_LOW_POWER,
        cmd::SPECIAL_FEATURE_AUTO_ADJUST);
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(
        cmd::CUSTOM_OPERATING_MODE,
        cmd::OPERATING_MODE_E2_PRIORITY_MASK);
    uint8_t output = 0xA5;
    const Status st = dev.readOperatingMode(output);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(
        cmd::OPERATING_MODE_E2_PRIORITY_MASK, st.detail);
    TEST_ASSERT_EQUAL_UINT8(0xA5, output);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(cmd::CUSTOM_AUTO_ADJUST, 0x02);
    const uint32_t failuresBefore = dev.totalFailures();
    bool output = true;
    const Status st = dev.readAutoAdjustStatus(output);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0x02, st.detail);
    TEST_ASSERT_TRUE(output);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
  }
}

void test_typed_persisted_value_boundaries_and_filter_remain_simple() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  const uint8_t validAddresses[] = {0, 7};
  for (uint8_t address : validAddresses) {
    fake.setMemory(cmd::CUSTOM_BUS_ADDRESS, address);
    uint8_t output = 0xFF;
    TEST_ASSERT_TRUE(dev.readBusAddress(output).ok());
    TEST_ASSERT_EQUAL_UINT8(address, output);
  }

  const uint16_t validIntervals[] = {
      cmd::INTERVAL_MIN_DECISEC,
      cmd::INTERVAL_MAX_DECISEC,
  };
  for (uint16_t interval : validIntervals) {
    fake.setMemory(
        cmd::CUSTOM_INTERVAL_L,
        static_cast<uint8_t>(interval & 0xFFU));
    fake.setMemory(
        cmd::CUSTOM_INTERVAL_H,
        static_cast<uint8_t>(interval >> 8));
    uint16_t output = 0;
    TEST_ASSERT_TRUE(dev.readMeasurementInterval(output).ok());
    TEST_ASSERT_EQUAL_UINT16(interval, output);
  }

  const uint8_t factorRawValues[] = {0x80, 0xFF, 0x01, 0x7F};
  const int8_t expectedFactors[] = {-128, -1, 1, 127};
  for (size_t i = 0; i < sizeof(factorRawValues); ++i) {
    fake.setMemory(
        cmd::CUSTOM_CO2_INTERVAL_FACTOR, factorRawValues[i]);
    int8_t output = 0;
    TEST_ASSERT_TRUE(dev.readCo2IntervalFactor(output).ok());
    TEST_ASSERT_EQUAL_INT8(expectedFactors[i], output);
  }
  for (int8_t factor : expectedFactors) {
    TEST_ASSERT_TRUE(dev.writeCo2IntervalFactor(factor).ok());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(factor),
        fake.memory(cmd::CUSTOM_CO2_INTERVAL_FACTOR));
  }

  for (uint8_t mode = 0; mode <= 3; ++mode) {
    fake.setMemory(cmd::CUSTOM_OPERATING_MODE, mode);
    uint8_t output = 0xFF;
    TEST_ASSERT_TRUE(dev.readOperatingMode(output).ok());
    TEST_ASSERT_EQUAL_UINT8(mode, output);
    TEST_ASSERT_TRUE(dev.writeOperatingMode(mode).ok());
  }

  for (uint8_t raw = 0; raw <= 1; ++raw) {
    fake.setMemory(cmd::CUSTOM_AUTO_ADJUST, raw);
    bool running = false;
    TEST_ASSERT_TRUE(dev.readAutoAdjustStatus(running).ok());
    TEST_ASSERT_EQUAL(raw != 0U, running);
  }

  fake.setMemory(cmd::CUSTOM_FILTER_CO2, 0xFF);
  uint8_t filter = 0;
  TEST_ASSERT_TRUE(dev.readCo2Filter(filter).ok());
  TEST_ASSERT_EQUAL_UINT8(0xFF, filter);
  TEST_ASSERT_TRUE(dev.writeCo2Filter(0xA5).ok());
  TEST_ASSERT_EQUAL_UINT8(0xA5, fake.memory(cmd::CUSTOM_FILTER_CO2));

  TEST_ASSERT_TRUE(
      dev.writeMeasurementInterval(
          cmd::INTERVAL_MIN_DECISEC).ok());
  TEST_ASSERT_TRUE(
      dev.writeMeasurementInterval(
          cmd::INTERVAL_MAX_DECISEC).ok());
}

void test_typed_write_validation_precedence_is_bus_silent() {
  {
    EE871::EE871 dev;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_INITIALIZED),
        static_cast<uint8_t>(dev.writeBusAddress(8).code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_INITIALIZED),
        static_cast<uint8_t>(
            dev.writeMeasurementInterval(
                cmd::INTERVAL_MIN_DECISEC - 1U).code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_INITIALIZED),
        static_cast<uint8_t>(dev.writeCo2IntervalFactor(0).code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_INITIALIZED),
        static_cast<uint8_t>(dev.writeOperatingMode(0x04).code));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    const Status invalid[] = {
        dev.writeBusAddress(8),
        dev.writeMeasurementInterval(
            cmd::INTERVAL_MIN_DECISEC - 1U),
        dev.writeMeasurementInterval(
            cmd::INTERVAL_MAX_DECISEC + 1U),
        dev.writeCo2IntervalFactor(0),
        dev.writeOperatingMode(0x04),
        dev.customWrite(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0),
    };
    for (const Status& st : invalid) {
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::OUT_OF_RANGE),
          static_cast<uint8_t>(st.code));
    }
    assertNoBusActivity(fake);
    TEST_ASSERT_FALSE(dev.mutationDiagnostic().unresolved);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationTarget::NONE),
        static_cast<uint8_t>(dev.mutationDiagnostic().target));
  }

  {
    FakeE2Transport fake;
    fake.setCapabilities(
        0, 0, 0, 0, 0x55, cmd::MODE_SUPPORT_LOW_POWER, 0);
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    const Status st = dev.writeOperatingMode(
        cmd::OPERATING_MODE_E2_PRIORITY_MASK);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(
        cmd::OPERATING_MODE_E2_PRIORITY_MASK, st.detail);
    assertNoBusActivity(fake);
    TEST_ASSERT_FALSE(dev.mutationDiagnostic().unresolved);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    forceOfflineByNack(dev, fake);
    fake.resetActivityCounters();
    const Status invalid[] = {
        dev.writeBusAddress(8),
        dev.writeMeasurementInterval(
            cmd::INTERVAL_MIN_DECISEC - 1U),
        dev.writeCo2IntervalFactor(0),
        dev.writeOperatingMode(0x04),
    };
    for (const Status& st : invalid) {
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::OFFLINE),
          static_cast<uint8_t>(st.code));
    }
    assertNoBusActivity(fake);
    TEST_ASSERT_FALSE(dev.mutationDiagnostic().unresolved);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(0x20);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(dev.customWrite(0x20, 0x5A).code));
    fake.resetActivityCounters();
    const Status invalid[] = {
        dev.writeBusAddress(8),
        dev.writeMeasurementInterval(
            cmd::INTERVAL_MIN_DECISEC - 1U),
        dev.writeCo2IntervalFactor(0),
        dev.writeOperatingMode(0x04),
    };
    for (const Status& st : invalid) {
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
          static_cast<uint8_t>(st.code));
    }
    assertNoBusActivity(fake);
  }
}

void test_auto_adjust_reserved_pre_and_post_states_fail_closed() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setMemory(cmd::CUSTOM_AUTO_ADJUST, 0x02);
    fake.resetActivityCounters();

    const Status st = dev.startAutoAdjust();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0x02, st.detail);
    TEST_ASSERT_EQUAL_UINT32(
        0,
        fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false));
    TEST_ASSERT_FALSE(dev.mutationDiagnostic().unresolved);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationTarget::NONE),
        static_cast<uint8_t>(dev.mutationDiagnostic().target));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.replaceNextWriteCommitToAddress(
        cmd::CUSTOM_AUTO_ADJUST, 0x02);

    const Status st = dev.startAutoAdjust();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0x02, st.detail);
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.unresolved);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationEffect::ACKNOWLEDGED),
        static_cast<uint8_t>(mutation.effect));
    TEST_ASSERT_TRUE(mutation.observedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0x02, mutation.observedValue);
    assertSameStatus(st, mutation.cause);
  }
}

void test_semantic_invalid_post_write_uses_typed_validators() {
  struct PostCase {
    uint8_t address;
    uint8_t replacement;
    MutationTarget target;
  };
  const PostCase cases[] = {
      {cmd::CUSTOM_CO2_INTERVAL_FACTOR,
       0,
       MutationTarget::CO2_INTERVAL_FACTOR},
      {cmd::CUSTOM_OPERATING_MODE,
       0x04,
       MutationTarget::OPERATING_MODE},
  };
  for (const PostCase& item : cases) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.replaceNextWriteCommitToAddress(
        item.address, item.replacement);
    const uint32_t failuresBefore = dev.totalFailures();

    const Status st =
        item.target == MutationTarget::CO2_INTERVAL_FACTOR
            ? dev.writeCo2IntervalFactor(-2)
            : dev.writeOperatingMode(3);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(item.replacement, st.detail);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.unresolved);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(item.target),
        static_cast<uint8_t>(mutation.target));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationEffect::ACKNOWLEDGED),
        static_cast<uint8_t>(mutation.effect));
    TEST_ASSERT_EQUAL_UINT8(item.replacement, mutation.observedValue);
    assertSameStatus(st, mutation.cause);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.replaceNextWriteCommitToAddress(
        cmd::CUSTOM_INTERVAL_H, 0);
    const uint32_t failuresBefore = dev.totalFailures();

    const Status st = dev.writeMeasurementInterval(300);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(44, st.detail);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.unresolved);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationTarget::GLOBAL_INTERVAL),
        static_cast<uint8_t>(mutation.target));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationEffect::ACKNOWLEDGED),
        static_cast<uint8_t>(mutation.effect));
    TEST_ASSERT_TRUE(mutation.observedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0, mutation.observedValue);
    assertSameStatus(st, mutation.cause);
  }
}

void test_semantic_invalid_unresolved_resync_preserves_original_cause() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(
        cmd::CUSTOM_CO2_INTERVAL_FACTOR);
    const Status original = dev.writeCo2IntervalFactor(-3);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(original.code));
    fake.setMemory(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0);

    const Status st = dev.resyncPersistentConfig();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0, st.detail);
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.unresolved);
    TEST_ASSERT_NOT_EQUAL(
        static_cast<uint8_t>(MutationEffect::RESYNCHRONIZED),
        static_cast<uint8_t>(mutation.effect));
    TEST_ASSERT_TRUE(mutation.observedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0, mutation.observedValue);
    assertSameStatus(original, mutation.cause);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(cmd::CUSTOM_INTERVAL_L);
    const Status original = dev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(original.code));
    const uint16_t invalid =
        static_cast<uint16_t>(cmd::INTERVAL_MIN_DECISEC - 1U);
    fake.setMemory(
        cmd::CUSTOM_INTERVAL_L,
        static_cast<uint8_t>(invalid & 0xFFU));
    fake.setMemory(
        cmd::CUSTOM_INTERVAL_H,
        static_cast<uint8_t>(invalid >> 8));

    const Status st = dev.resyncPersistentConfig();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(invalid, st.detail);
    TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
    assertSameStatus(original, dev.mutationDiagnostic().cause);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.dropNextWriteCommitToAddress(cmd::CUSTOM_AUTO_ADJUST);
    const Status original = dev.startAutoAdjust();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
        static_cast<uint8_t>(original.code));
    fake.setMemory(cmd::CUSTOM_AUTO_ADJUST, 0x02);

    const Status st = dev.resyncPersistentConfig();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0x02, st.detail);
    const MutationDiagnostic mutation = dev.mutationDiagnostic();
    TEST_ASSERT_TRUE(mutation.unresolved);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationEffect::ACKNOWLEDGED),
        static_cast<uint8_t>(mutation.effect));
    TEST_ASSERT_TRUE(mutation.observedValueValid);
    TEST_ASSERT_EQUAL_UINT8(0x02, mutation.observedValue);
    assertSameStatus(original, mutation.cause);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    const Status original = dev.writeBusAddress(2);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
        static_cast<uint8_t>(original.code));
    dev.end();
    fake.setRespondingDeviceAddress(2);
    Config cfg = fake.makeConfig();
    cfg.deviceAddress = 2;
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    fake.setMemory(cmd::CUSTOM_BUS_ADDRESS, 8);

    const Status st = dev.resyncPersistentConfig();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(8, st.detail);
    TEST_ASSERT_TRUE(dev.mutationDiagnostic().unresolved);
    assertSameStatus(original, dev.mutationDiagnostic().cause);
  }
}

void test_full_resync_uses_all_typed_persisted_validators() {
  enum class InvalidTarget : uint8_t {
    ADDRESS,
    INTERVAL,
    FACTOR,
    MODE,
    AUTO_ADJUST,
  };
  const InvalidTarget targets[] = {
      InvalidTarget::ADDRESS,
      InvalidTarget::INTERVAL,
      InvalidTarget::FACTOR,
      InvalidTarget::MODE,
      InvalidTarget::AUTO_ADJUST,
  };
  for (InvalidTarget target : targets) {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    int32_t expectedDetail = 0;
    switch (target) {
      case InvalidTarget::ADDRESS:
        fake.setMemory(cmd::CUSTOM_BUS_ADDRESS, 8);
        expectedDetail = 8;
        break;
      case InvalidTarget::INTERVAL: {
        const uint16_t invalid =
            static_cast<uint16_t>(
                cmd::INTERVAL_MAX_DECISEC + 1U);
        fake.setMemory(
            cmd::CUSTOM_INTERVAL_L,
            static_cast<uint8_t>(invalid & 0xFFU));
        fake.setMemory(
            cmd::CUSTOM_INTERVAL_H,
            static_cast<uint8_t>(invalid >> 8));
        expectedDetail = invalid;
        break;
      }
      case InvalidTarget::FACTOR:
        fake.setMemory(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0);
        break;
      case InvalidTarget::MODE:
        fake.setMemory(cmd::CUSTOM_OPERATING_MODE, 0x04);
        expectedDetail = 0x04;
        break;
      case InvalidTarget::AUTO_ADJUST:
        fake.setMemory(cmd::CUSTOM_AUTO_ADJUST, 0x02);
        expectedDetail = 0x02;
        break;
    }
    const uint32_t failuresBefore = dev.totalFailures();

    const Status st = dev.resyncPersistentConfig();

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(expectedDetail, st.detail);
    TEST_ASSERT_FALSE(dev.mutationDiagnostic().unresolved);
    TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(DriverState::READY),
        static_cast<uint8_t>(dev.state()));
  }
}

void test_raw_custom_write_protected_address_dispatch_is_exhaustive() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

    const uint8_t singletonReadOnly[] = {
        cmd::CUSTOM_ERROR_CODE,
        cmd::CUSTOM_POINTER_LOW,
        cmd::CUSTOM_POINTER_HIGH,
    };
    for (uint16_t address = cmd::CUSTOM_FW_VERSION_MAIN;
         address <= cmd::CUSTOM_SPECIAL_FEATURES;
         ++address) {
      fake.resetActivityCounters();
      const Status st =
          dev.customWrite(static_cast<uint8_t>(address), 0x5A);
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::NOT_SUPPORTED),
          static_cast<uint8_t>(st.code));
      assertNoBusActivity(fake);
    }
    for (uint16_t address = cmd::CUSTOM_CO2_POINT_L_L;
         address <= cmd::CUSTOM_CO2_POINT_U_H;
         ++address) {
      fake.resetActivityCounters();
      const Status st =
          dev.customWrite(static_cast<uint8_t>(address), 0x5A);
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::NOT_SUPPORTED),
          static_cast<uint8_t>(st.code));
      assertNoBusActivity(fake);
    }
    for (uint16_t address = cmd::CUSTOM_SERIAL_START;
         address <
         cmd::CUSTOM_SERIAL_START + cmd::CUSTOM_SERIAL_LEN;
         ++address) {
      fake.resetActivityCounters();
      const Status st =
          dev.customWrite(static_cast<uint8_t>(address), 0x5A);
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::NOT_SUPPORTED),
          static_cast<uint8_t>(st.code));
      assertNoBusActivity(fake);
    }
    for (uint8_t address : singletonReadOnly) {
      fake.resetActivityCounters();
      const Status st = dev.customWrite(address, 0x5A);
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::NOT_SUPPORTED),
          static_cast<uint8_t>(st.code));
      assertNoBusActivity(fake);
    }

    const uint8_t unsafePairAddresses[] = {
        cmd::CUSTOM_CO2_OFFSET_L,
        cmd::CUSTOM_CO2_OFFSET_H,
        cmd::CUSTOM_CO2_GAIN_L,
        cmd::CUSTOM_CO2_GAIN_H,
        cmd::CUSTOM_INTERVAL_L,
        cmd::CUSTOM_INTERVAL_H,
    };
    for (uint8_t address : unsafePairAddresses) {
      fake.resetActivityCounters();
      const Status st = dev.customWrite(address, 0x5A);
      TEST_ASSERT_EQUAL_UINT8(
          static_cast<uint8_t>(Err::NOT_SUPPORTED),
          static_cast<uint8_t>(st.code));
      assertNoBusActivity(fake);
    }

    fake.resetActivityCounters();
    Status st = dev.customWrite(cmd::CUSTOM_BUS_ADDRESS, 8);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    assertNoBusActivity(fake);

    fake.resetActivityCounters();
    st = dev.customWrite(cmd::CUSTOM_AUTO_ADJUST, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NOT_SUPPORTED),
        static_cast<uint8_t>(st.code));
    assertNoBusActivity(fake);

    fake.resetActivityCounters();
    st = dev.customWrite(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    assertNoBusActivity(fake);

    fake.resetActivityCounters();
    st = dev.customWrite(cmd::CUSTOM_OPERATING_MODE, 0x04);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::OUT_OF_RANGE),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(0x04, st.detail);
    assertNoBusActivity(fake);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(
        dev.customWrite(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0xFE).ok());
    assertMutationHeader(
        dev,
        MutationTarget::CO2_INTERVAL_FACTOR,
        MutationEffect::VERIFIED,
        false,
        cmd::CUSTOM_CO2_INTERVAL_FACTOR,
        cmd::CUSTOM_CO2_INTERVAL_FACTOR,
        1,
        1,
        1,
        1);
    TEST_ASSERT_EQUAL_UINT8(
        0xFE, fake.memory(cmd::CUSTOM_CO2_INTERVAL_FACTOR));
    int8_t factor = 0;
    TEST_ASSERT_TRUE(dev.readCo2IntervalFactor(factor).ok());
    TEST_ASSERT_EQUAL_INT8(-2, factor);

    fake.setMemory(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0x80);
    TEST_ASSERT_TRUE(dev.readCo2IntervalFactor(factor).ok());
    TEST_ASSERT_EQUAL_INT8(-128, factor);

    fake.setMemory(cmd::CUSTOM_CO2_INTERVAL_FACTOR, 0xFF);
    TEST_ASSERT_TRUE(dev.readCo2IntervalFactor(factor).ok());
    TEST_ASSERT_EQUAL_INT8(-1, factor);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(
        dev.customWrite(cmd::CUSTOM_FILTER_CO2, 9).ok());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationTarget::CO2_FILTER),
        static_cast<uint8_t>(dev.mutationDiagnostic().target));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(
        dev.customWrite(cmd::CUSTOM_OPERATING_MODE, 3).ok());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationTarget::OPERATING_MODE),
        static_cast<uint8_t>(dev.mutationDiagnostic().target));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    TEST_ASSERT_TRUE(
        dev.customWrite(cmd::CUSTOM_AUTO_ADJUST, 1).ok());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationTarget::AUTO_ADJUST),
        static_cast<uint8_t>(dev.mutationDiagnostic().target));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    const Status st =
        dev.customWrite(cmd::CUSTOM_BUS_ADDRESS, 2);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(MutationTarget::BUS_ADDRESS),
        static_cast<uint8_t>(dev.mutationDiagnostic().target));
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, true));
  }
}

void test_interval_pair_uses_one_deferred_commit_then_reads_both() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.resetActivityCounters();
  fake.resetElapsed();

  TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());

  TEST_ASSERT_EQUAL_UINT32(5, fake.transactionCount());
  TEST_ASSERT_EQUAL_UINT8(
      cmd::MAIN_CUSTOM_WRITE, fake.transactionMain(0));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::CUSTOM_INTERVAL_L, fake.transactionAddress(0));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::MAIN_CUSTOM_WRITE, fake.transactionMain(1));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::CUSTOM_INTERVAL_H, fake.transactionAddress(1));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::MAIN_CUSTOM_PTR, fake.transactionMain(2));
  TEST_ASSERT_FALSE(fake.transactionIsRead(2));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::CUSTOM_INTERVAL_L, fake.transactionAddress(2));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::MAIN_CUSTOM_PTR, fake.transactionMain(3));
  TEST_ASSERT_TRUE(fake.transactionIsRead(3));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::CUSTOM_INTERVAL_L, fake.transactionAddress(3));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::MAIN_CUSTOM_PTR, fake.transactionMain(4));
  TEST_ASSERT_TRUE(fake.transactionIsRead(4));
  TEST_ASSERT_EQUAL_UINT8(
      cmd::CUSTOM_INTERVAL_H, fake.transactionAddress(4));
  TEST_ASSERT_EQUAL_UINT32(
      2, fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false));
  TEST_ASSERT_EQUAL_UINT32(
      1, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, false));
  TEST_ASSERT_EQUAL_UINT32(
      2, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, true));
  TEST_ASSERT_FALSE(fake.intervalTransactionStartedEarly());
  assertWithinTimingBound(
      dev, fake, OperationKind::INTERVAL_WRITE_VERIFY);

  {
    FakeE2Transport mismatchFake;
    EE871::EE871 mismatchDev;
    TEST_ASSERT_TRUE(
        beginFakeDevice(mismatchDev, mismatchFake).ok());
    mismatchFake.resetActivityCounters();
    mismatchFake.dropNextWriteCommitToAddress(
        cmd::CUSTOM_INTERVAL_L);

    const Status mismatch =
        mismatchDev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::VERIFY_MISMATCH),
        static_cast<uint8_t>(mismatch.code));
    TEST_ASSERT_EQUAL_INT32(
        cmd::INTERVAL_MIN_DECISEC & 0xFFU, mismatch.detail);
    assertMutationHeader(
        mismatchDev,
        MutationTarget::GLOBAL_INTERVAL,
        MutationEffect::ACKNOWLEDGED,
        true,
        cmd::CUSTOM_INTERVAL_L,
        cmd::CUSTOM_INTERVAL_H,
        2,
        2,
        2,
        1);
    TEST_ASSERT_EQUAL_UINT32(5, mismatchFake.transactionCount());
    TEST_ASSERT_EQUAL_UINT32(
        1,
        mismatchFake.countTransactions(
            cmd::MAIN_CUSTOM_PTR, false));
    TEST_ASSERT_EQUAL_UINT32(
        2,
        mismatchFake.countTransactions(
            cmd::MAIN_CUSTOM_PTR, true));
  }
}

void test_new_mutation_timing_kinds_cover_worst_case_fake_time() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.writePartName(partName).ok());
    assertWithinTimingBound(
        dev, fake, OperationKind::PART_NAME_WRITE_VERIFY);
    OperationTimingBound genericBound;
    TEST_ASSERT_TRUE(dev.operationTimingBound(
        OperationKind::CUSTOM_BLOCK_WRITE_VERIFY,
        cmd::CUSTOM_PART_NAME_LEN,
        genericBound).ok());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <=
        static_cast<uint64_t>(genericBound.maxBlockingMs) * 1000U);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.customWrite(0x20, 0x5A).ok());
    assertWithinTimingBound(
        dev, fake, OperationKind::CUSTOM_BYTE_WRITE_VERIFY);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());
    assertWithinTimingBound(
        dev, fake, OperationKind::INTERVAL_WRITE_VERIFY);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.startAutoAdjust().ok());
    assertWithinTimingBound(
        dev, fake, OperationKind::AUTO_ADJUST_MAINTENANCE);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    const Status st = dev.writeBusAddress(2);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PERSISTENT_STATE_UNCERTAIN),
        static_cast<uint8_t>(st.code));
    assertWithinTimingBound(
        dev, fake, OperationKind::BUS_ADDRESS_CHANGE);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.resyncPersistentConfig().ok());
    assertWithinTimingBound(
        dev, fake, OperationKind::RESYNC_PERSISTENT_CONFIG);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    Config cfg = fake.makeConfig();
    OperationTimingBound beginBound;
    TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
        cfg,
        OperationKind::BEGIN_REQUIRE_PRESENT,
        1,
        beginBound).ok());
    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <=
        static_cast<uint64_t>(beginBound.maxBlockingMs) * 1000U);

    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.recover().ok());
    assertWithinTimingBound(
        dev,
        fake,
        OperationKind::RECOVER_IDENTITY_AND_CAPABILITIES);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setStatusByte(cmd::STATUS_CO2_ERROR_MASK);
    fake.setErrorCode(cmd::CO2_ERROR_SENSOR_COUNTS_LOW);
    applyNearByteBudgetStretch(fake);
    fake.resetElapsed();
    Co2ReadResult result;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::CO2_SENSOR_ERROR),
        static_cast<uint8_t>(
            dev.readCo2AverageSample(result).code));
    assertWithinTimingBound(
        dev, fake, OperationKind::CHECKED_CO2_AVERAGE);
  }
}

void test_public_bus_api_first_transfer_failure_matrix() {
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.probe();
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.recover();
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.resyncPersistentConfig();
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t value = 0xA5;
    return dev.readControlByte(cmd::MAIN_STATUS, value);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint16_t value = 0xA5A5;
    return dev.readU16(
        cmd::MAIN_MV4_LO, cmd::MAIN_MV4_HI, value);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.setCustomPointer(0x20);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t value = 0xA5;
    return dev.customRead(0x20, value);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t values[2] = {0xA5, 0x5A};
    return dev.customRead(0x20, values, 2);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.customWrite(0x20, 0x5A);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.writeMeasurementInterval(300);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint16_t group = 0xA5A5;
    return dev.readGroup(group);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t subgroup = 0xA5;
    return dev.readSubgroup(subgroup);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t bits = 0xA5;
    return dev.readAvailableMeasurements(bits);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t main = 0xA5;
    uint8_t sub = 0x5A;
    return dev.readFirmwareVersion(main, sub);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t version = 0xA5;
    return dev.readE2SpecVersion(version);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t bits = 0xA5;
    return dev.readOperatingFunctions(bits);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t bits = 0xA5;
    return dev.readOperatingModeSupport(bits);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t bits = 0xA5;
    return dev.readSpecialFeatures(bits);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t serial[cmd::CUSTOM_SERIAL_LEN] = {};
    return dev.readSerialNumber(serial);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
    return dev.readPartName(partName);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    const uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
    return dev.writePartName(partName);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t address = 0xA5;
    return dev.readBusAddress(address);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.writeBusAddress(2);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint16_t interval = 0xA5A5;
    return dev.readMeasurementInterval(interval);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    int8_t factor = 42;
    return dev.readCo2IntervalFactor(factor);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.writeCo2IntervalFactor(-2);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t filter = 0xA5;
    return dev.readCo2Filter(filter);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.writeCo2Filter(7);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t mode = 0xA5;
    return dev.readOperatingMode(mode);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.writeOperatingMode(3);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    bool running = true;
    return dev.readAutoAdjustStatus(running);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.startAutoAdjust();
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    int16_t offset = 123;
    return dev.readCo2Offset(offset);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.writeCo2Offset(123);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint16_t gain = 456;
    return dev.readCo2Gain(gain);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    return dev.writeCo2Gain(456);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint16_t lower = 123;
    uint16_t upper = 456;
    return dev.readCo2CalPoints(lower, upper);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t status = 0xA5;
    return dev.readStatus(status);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint8_t code = 0xA5;
    return dev.readErrorCode(code);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint16_t ppm = 0xA5A5;
    return dev.readCo2Fast(ppm);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    uint16_t ppm = 0xA5A5;
    return dev.readCo2Average(ppm);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    Co2ReadResult out;
    return dev.readCo2AverageSample(out);
  });
  assertFirstBusTransferNack([](EE871::EE871& dev) {
    Co2ReadResult out;
    return dev.readCo2FastSample(out);
  });

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setHoldSclLow(true);
    const Status st = dev.busReset();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::BUS_STUCK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
  }
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setSdaStuckLow(true);
    const Status st = dev.checkBusIdle();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::BUS_STUCK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
  }
}

static Status staticControlBound(
    const Config& config, OperationTimingBound& out) {
  return EE871::EE871::operationTimingBound(
      config, OperationKind::CONTROL_READ, 1, out);
}

void test_config_protocol_timing_boundaries() {
  FakeE2Transport fake;
  Config cfg = fake.makeConfig();
  OperationTimingBound out;

  TEST_ASSERT_TRUE(staticControlBound(cfg, out).ok());

  cfg.clockLowUs = 100;
  cfg.clockHighUs = 1890;
  TEST_ASSERT_TRUE(staticControlBound(cfg, out).ok());

  cfg.clockHighUs = 100;
  TEST_ASSERT_TRUE(staticControlBound(cfg, out).ok());

  cfg.clockHighUs = 1891;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, out).code));

  cfg = fake.makeConfig();
  cfg.clockLowUs = 99;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, out).code));
  cfg = fake.makeConfig();
  cfg.clockHighUs = 99;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, out).code));

  cfg = fake.makeConfig();
  cfg.bitTimeoutUs = 0;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, out).code));
  cfg = fake.makeConfig();
  cfg.bitTimeoutUs = cmd::BIT_TIMEOUT_MAX_US + 1U;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, out).code));
  cfg = fake.makeConfig();
  cfg.byteTimeoutUs = cfg.bitTimeoutUs - 1U;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, out).code));
  cfg = fake.makeConfig();
  cfg.byteTimeoutUs = cmd::BYTE_TIMEOUT_MAX_US + 1U;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, out).code));
}

void test_config_delay_normalization_and_limits() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  Config cfg = fake.makeConfig();
  cfg.writeDelayMs = 0;
  cfg.intervalWriteDelayMs = 0;
  cfg.longDelaySliceMs = 0;
  cfg.offlineThreshold = 0;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  TEST_ASSERT_EQUAL_UINT32(
      cmd::WRITE_DELAY_PROTOCOL_MIN_MS, dev.getConfig().writeDelayMs);
  TEST_ASSERT_EQUAL_UINT32(
      cmd::INTERVAL_WRITE_DELAY_PROTOCOL_MIN_MS,
      dev.getConfig().intervalWriteDelayMs);
  TEST_ASSERT_EQUAL_UINT8(1, dev.getConfig().longDelaySliceMs);
  TEST_ASSERT_EQUAL_UINT8(1, dev.getConfig().offlineThreshold);

  OperationTimingBound zeroDelays;
  OperationTimingBound belowMinimums;
  cfg = fake.makeConfig();
  cfg.writeDelayMs = 0;
  cfg.intervalWriteDelayMs = 0;
  TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
      cfg, OperationKind::INTERVAL_WRITE_VERIFY, 1, zeroDelays).ok());
  cfg.writeDelayMs = 149;
  cfg.intervalWriteDelayMs = 299;
  TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
      cfg, OperationKind::INTERVAL_WRITE_VERIFY, 1, belowMinimums).ok());
  TEST_ASSERT_EQUAL_UINT32(
      zeroDelays.maxBlockingMs, belowMinimums.maxBlockingMs);

  cfg = fake.makeConfig();
  cfg.writeDelayMs = cmd::WRITE_DELAY_MAX_MS;
  cfg.intervalWriteDelayMs = cmd::INTERVAL_WRITE_DELAY_MAX_MS;
  cfg.longDelaySliceMs = cmd::LONG_DELAY_SLICE_MAX_MS;
  TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
      cfg, OperationKind::INTERVAL_WRITE_VERIFY, 1, belowMinimums).ok());

  cfg.writeDelayMs = cmd::WRITE_DELAY_MAX_MS + 1U;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, belowMinimums).code));
  cfg = fake.makeConfig();
  cfg.intervalWriteDelayMs = cmd::INTERVAL_WRITE_DELAY_MAX_MS + 1U;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, belowMinimums).code));
  cfg = fake.makeConfig();
  cfg.longDelaySliceMs = cmd::LONG_DELAY_SLICE_MAX_MS + 1U;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(staticControlBound(cfg, belowMinimums).code));
}

void test_sda_low_before_start_is_bus_stuck_without_false_start() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.resetActivityCounters();
  fake.setSdaStuckLow(true);

  uint8_t status = 0;
  const Status st = dev.readStatus(status);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::BUS_STUCK),
      static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("SDA low before START", st.msg);
  TEST_ASSERT_EQUAL_UINT32(0, fake.transactionCount());
}

void test_scl_stuck_is_precise_at_idle_reset_and_in_transaction() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());

  fake.setHoldSclLow(true);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::BUS_STUCK),
      static_cast<uint8_t>(dev.checkBusIdle().code));
  const uint32_t failuresBeforeReset = dev.totalFailures();
  const uint32_t successesBeforeReset = dev.totalSuccess();
  const DriverState stateBeforeReset = dev.state();
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::BUS_STUCK),
      static_cast<uint8_t>(dev.busReset().code));
  TEST_ASSERT_EQUAL_UINT32(failuresBeforeReset, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBeforeReset, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(stateBeforeReset),
      static_cast<uint8_t>(dev.state()));
  fake.setHoldSclLow(false);
  TEST_ASSERT_TRUE(dev.busReset().ok());
  TEST_ASSERT_EQUAL_UINT32(failuresBeforeReset, dev.totalFailures());
  TEST_ASSERT_EQUAL_UINT32(successesBeforeReset, dev.totalSuccess());
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(stateBeforeReset),
      static_cast<uint8_t>(dev.state()));

  fake.setStretch(StretchPhase::DATA_BIT, 25005);
  uint8_t status = 0;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::TIMEOUT),
      static_cast<uint8_t>(dev.readStatus(status).code));
}

void test_valid_stop_hold_above_bit_timeout_succeeds() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  Config cfg = fake.makeConfig();
  cfg.stopHoldUs = 30000;

  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  uint8_t status = 0;
  TEST_ASSERT_TRUE(dev.readStatus(status).ok());
}

void test_bit_and_byte_deadline_boundaries() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setStretch(StretchPhase::DATA_BIT, 25000);
    uint8_t status = 0;
    TEST_ASSERT_TRUE(dev.readStatus(status).ok());
  }

  const uint32_t exactByteDurations[8] = {
      4140, 4140, 4140, 4140, 4140, 4140, 4140, 4130};
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setStretchSequence(
        StretchPhase::DATA_BIT, exactByteDurations, 8);
    uint8_t status = 0;
    TEST_ASSERT_TRUE(dev.readStatus(status).ok());
  }

  const uint32_t overByteDurations[8] = {
      4140, 4140, 4140, 4140, 4140, 4140, 4140, 4135};
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setStretchSequence(
        StretchPhase::DATA_BIT, overByteDurations, 8);
    uint8_t status = 0;
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(dev.readStatus(status).code));
  }
}

void test_write_pre_pec_phases_keep_ordinary_deadlines() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setStretch(StretchPhase::DATA_BIT, 25005);

    const Status st =
        dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x42);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setStretch(StretchPhase::ACK_BIT, 25005);

    const Status st =
        dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
  }
}

void test_custom_write_completion_uses_one_total_budget() {
  uint64_t baselineElapsed = 0;
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(
        dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x42).ok());
    baselineElapsed = fake.elapsedUs();
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetElapsed();
    fake.setStretch(StretchPhase::FINAL_ACK, 149700);
    TEST_ASSERT_TRUE(
        dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x42).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed, fake.elapsedUs());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetElapsed();
    fake.setStretch(StretchPhase::FINAL_ACK, 150000);
    TEST_ASSERT_TRUE(
        dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x42).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed, fake.elapsedUs());
    assertWithinTimingBound(
        dev, fake, OperationKind::CUSTOM_BYTE_WRITE_VERIFY);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setStretch(StretchPhase::FINAL_ACK, 150005);
    const Status st =
        dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x42);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(150000, st.detail);
    TEST_ASSERT_EQUAL_UINT32(1, fake.transactionCount());
    assertMutationHeader(
        dev,
        MutationTarget::CO2_FILTER,
        MutationEffect::INDETERMINATE,
        true,
        cmd::CUSTOM_FILTER_CO2,
        cmd::CUSTOM_FILTER_CO2,
        1,
        0,
        0,
        0);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetElapsed();
    fake.setCompletionStretches(100000, 50000);
    TEST_ASSERT_TRUE(
        dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x42).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed, fake.elapsedUs());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setCompletionStretches(100000, 50005);
    const Status st =
        dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x42);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(150000, st.detail);
    assertMutationHeader(
        dev,
        MutationTarget::CO2_FILTER,
        MutationEffect::ACKNOWLEDGED,
        true,
        cmd::CUSTOM_FILTER_CO2,
        cmd::CUSTOM_FILTER_CO2,
        1,
        1,
        0,
        0);
  }
}

void test_final_pec_nack_cleanup_uses_completion_budget() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.setMemory(cmd::CUSTOM_FILTER_CO2, 0x11);
  const uint32_t failuresBefore = dev.totalFailures();
  fake.nackNextFinalAck();
  fake.setStretch(StretchPhase::STOP, 30000);

  const Status st =
      dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x42);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NACK),
      static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("PEC NACK", st.msg);
  TEST_ASSERT_EQUAL_UINT8(0x11, fake.memory(cmd::CUSTOM_FILTER_CO2));
  TEST_ASSERT_TRUE(fake.busLinesIdle());
  TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
}

void test_precise_primary_errors_precede_failed_stop_cleanup() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setCorruptReadPec(true);
    fake.setStretch(StretchPhase::STOP, 25005);
    const uint32_t failuresBefore = dev.totalFailures();

    uint8_t status = 0;
    const Status st = dev.readStatus(status);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::PEC_MISMATCH),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT32(failuresBefore + 1U, dev.totalFailures());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.setDevicePresent(false);
    fake.setStretch(StretchPhase::STOP, 25005);

    uint8_t status = 0;
    const Status st = dev.readStatus(status);

    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_STRING("Control byte NACK", st.msg);
  }
}

void test_pointer_stop_completion_and_dependent_read_ordering() {
  uint64_t baselineElapsed = 0;
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(
        dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2).ok());
    baselineElapsed = fake.elapsedUs();
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetElapsed();
    fake.setStretch(StretchPhase::STOP, 149700);
    TEST_ASSERT_TRUE(
        dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed, fake.elapsedUs());
    TEST_ASSERT_EQUAL_UINT32(0, fake.longDelaySlices());
    TEST_ASSERT_EQUAL_UINT32(0, fake.delayMsTotalMs());
    TEST_ASSERT_EQUAL_UINT32(300, fake.lastDelayUs());

    uint8_t value = 0;
    TEST_ASSERT_TRUE(
        dev.customRead(cmd::CUSTOM_FILTER_CO2, value).ok());
    TEST_ASSERT_FALSE(fake.pointerReadStartedEarly());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetElapsed();
    fake.setStretch(StretchPhase::STOP, 150000);
    TEST_ASSERT_TRUE(
        dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed, fake.elapsedUs());
    TEST_ASSERT_EQUAL_UINT32(0, fake.longDelaySlices());
    TEST_ASSERT_EQUAL_UINT32(0, fake.delayMsTotalMs());
    TEST_ASSERT_EQUAL_UINT32(4, fake.lastDelayUs());
    assertWithinTimingBound(
        dev, fake, OperationKind::CUSTOM_POINTER_WRITE);

    uint8_t value = 0;
    TEST_ASSERT_TRUE(
        dev.customRead(cmd::CUSTOM_FILTER_CO2, value).ok());
    TEST_ASSERT_FALSE(fake.pointerReadStartedEarly());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setStretch(StretchPhase::STOP, 150005);
    const Status st =
        dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(150000, st.detail);
    TEST_ASSERT_EQUAL_UINT32(1, fake.transactionCount());
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, true));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetElapsed();
    fake.setStretch(StretchPhase::DATA_BIT, 1000, 1, 31);
    TEST_ASSERT_TRUE(
        dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed + 1000U, fake.elapsedUs());

    uint8_t value = 0;
    TEST_ASSERT_TRUE(
        dev.customRead(cmd::CUSTOM_FILTER_CO2, value).ok());
    TEST_ASSERT_FALSE(fake.pointerReadStartedEarly());
  }
}

void test_startup_feature_reads_wait_for_pointer_completion() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  TEST_ASSERT_FALSE(fake.pointerReadStartedEarly());
}

void test_interval_commit_completion_boundaries() {
  uint64_t baselineElapsed = 0;
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());
    baselineElapsed = fake.elapsedUs();
    TEST_ASSERT_TRUE(fake.transactionHasAddress(0));
    TEST_ASSERT_EQUAL_UINT8(
        cmd::CUSTOM_INTERVAL_L, fake.transactionAddress(0));
    TEST_ASSERT_TRUE(fake.transactionHasAddress(1));
    TEST_ASSERT_EQUAL_UINT8(
        cmd::CUSTOM_INTERVAL_H, fake.transactionAddress(1));
    TEST_ASSERT_FALSE(fake.intervalTransactionStartedEarly());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.resetElapsed();
    fake.setStretch(
        StretchPhase::FINAL_ACK, 299700, 1, 1);
    TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed, fake.elapsedUs());
    TEST_ASSERT_EQUAL_UINT8(
        cmd::CUSTOM_INTERVAL_L, fake.transactionAddress(0));
    TEST_ASSERT_EQUAL_UINT8(
        cmd::CUSTOM_INTERVAL_H, fake.transactionAddress(1));
    TEST_ASSERT_FALSE(fake.intervalTransactionStartedEarly());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.resetElapsed();
    fake.setStretch(StretchPhase::DATA_BIT, 1000, 1, 63);
    TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed + 1000U, fake.elapsedUs());
    TEST_ASSERT_FALSE(fake.intervalTransactionStartedEarly());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.resetElapsed();
    fake.setStretch(
        StretchPhase::FINAL_ACK, 300000, 1, 1);
    TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed, fake.elapsedUs());
    assertWithinTimingBound(
        dev, fake, OperationKind::INTERVAL_WRITE_VERIFY);
    TEST_ASSERT_FALSE(fake.intervalTransactionStartedEarly());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setStretch(
        StretchPhase::FINAL_ACK, 300005, 1, 1);
    const Status st = dev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(300000, st.detail);
    TEST_ASSERT_TRUE(dev.persistentConfigDirty());
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, true));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.resetElapsed();
    fake.setStretch(
        StretchPhase::STOP, 300000, 1, 1);
    TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());
    TEST_ASSERT_EQUAL_UINT64(baselineElapsed, fake.elapsedUs());
    assertWithinTimingBound(
        dev, fake, OperationKind::INTERVAL_WRITE_VERIFY);
    TEST_ASSERT_FALSE(fake.intervalTransactionStartedEarly());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setStretch(
        StretchPhase::STOP, 300005, 1, 1);
    const Status st = dev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_INT32(300000, st.detail);
    TEST_ASSERT_TRUE(dev.persistentConfigDirty());
    TEST_ASSERT_EQUAL_UINT32(
        0, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, true));
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
    fake.resetActivityCounters();
    fake.setStretch(StretchPhase::FINAL_ACK, 25005);
    const Status st = dev.writeMeasurementInterval(300);
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::TIMEOUT),
        static_cast<uint8_t>(st.code));
    TEST_ASSERT_EQUAL_UINT8(
        cmd::CUSTOM_INTERVAL_L, fake.transactionAddress(0));
    TEST_ASSERT_EQUAL_UINT32(
        1, fake.countTransactions(cmd::MAIN_CUSTOM_WRITE, false));
  }
}

void test_block_custom_read_uses_one_pointer_and_auto_increment() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.setMemory(0x20, 0xA1);
  fake.setMemory(0x21, 0xB2);
  fake.setMemory(0x22, 0xC3);
  fake.resetActivityCounters();

  uint8_t values[3] = {};
  TEST_ASSERT_TRUE(dev.customRead(0x20, values, 3).ok());
  TEST_ASSERT_EQUAL_UINT8(0xA1, values[0]);
  TEST_ASSERT_EQUAL_UINT8(0xB2, values[1]);
  TEST_ASSERT_EQUAL_UINT8(0xC3, values[2]);
  TEST_ASSERT_EQUAL_UINT32(
      1, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, false));
  TEST_ASSERT_EQUAL_UINT32(
      3, fake.countTransactions(cmd::MAIN_CUSTOM_PTR, true));
  TEST_ASSERT_TRUE(fake.transactionHasAddress(0));
  TEST_ASSERT_EQUAL_UINT8(0x20, fake.transactionAddress(0));
  TEST_ASSERT_TRUE(fake.transactionHasAddress(1));
  TEST_ASSERT_EQUAL_UINT8(0x20, fake.transactionAddress(1));
  TEST_ASSERT_TRUE(fake.transactionHasAddress(2));
  TEST_ASSERT_EQUAL_UINT8(0x21, fake.transactionAddress(2));
  TEST_ASSERT_TRUE(fake.transactionHasAddress(3));
  TEST_ASSERT_EQUAL_UINT8(0x22, fake.transactionAddress(3));
  TEST_ASSERT_FALSE(fake.pointerReadStartedEarly());
}

void test_verify_mismatch_does_not_increment_transport_failures() {
  FakeE2Transport fake;
  EE871::EE871 dev;
  TEST_ASSERT_TRUE(beginFakeDevice(dev, fake).ok());
  fake.dropWritesToAddress(cmd::CUSTOM_FILTER_CO2, true);
  const uint32_t failuresBefore = dev.totalFailures();

  const Status st = dev.customWrite(cmd::CUSTOM_FILTER_CO2, 0x5A);

  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::VERIFY_MISMATCH),
      static_cast<uint8_t>(st.code));
  TEST_ASSERT_EQUAL_STRING("Write verification mismatch", st.msg);
  TEST_ASSERT_EQUAL_UINT32(failuresBefore, dev.totalFailures());
}

void test_long_wait_callbacks_are_sliced_and_bit_timing_does_not_yield() {
  uint64_t callbackElapsedUs = 0;
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    Config cfg = fake.makeConfig();
    cfg.longDelaySliceMs = 50;
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(
        dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2).ok());
    TEST_ASSERT_EQUAL_UINT32(3, fake.longDelaySlices());
    TEST_ASSERT_EQUAL_UINT32(150, fake.delayMsTotalMs());
    TEST_ASSERT_EQUAL_UINT32(50, fake.maxDelayMsSliceMs());
    TEST_ASSERT_EQUAL_UINT32(3, fake.yieldCount());
    TEST_ASSERT_EQUAL_UINT32(4, fake.lastDelayUs());
    callbackElapsedUs = fake.elapsedUs();

    fake.resetElapsed();
    fake.setStretch(StretchPhase::DATA_BIT, 1000);
    uint8_t status = 0;
    TEST_ASSERT_TRUE(dev.readStatus(status).ok());
    TEST_ASSERT_EQUAL_UINT32(0, fake.yieldCount());
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    Config cfg = fake.makeConfig();
    cfg.delayMs = nullptr;
    cfg.longDelaySliceMs = 50;
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(
        dev.setCustomPointer(cmd::CUSTOM_FILTER_CO2).ok());
    TEST_ASSERT_EQUAL_UINT32(0, fake.longDelaySlices());
    TEST_ASSERT_EQUAL_UINT32(3, fake.yieldCount());
    TEST_ASSERT_EQUAL_UINT32(3, fake.longDelayUsCalls());
    TEST_ASSERT_EQUAL_UINT32(50000, fake.longDelayUsDuration(0));
    TEST_ASSERT_EQUAL_UINT32(50000, fake.longDelayUsDuration(1));
    TEST_ASSERT_EQUAL_UINT32(50000, fake.longDelayUsDuration(2));
    TEST_ASSERT_EQUAL_UINT64(150000, fake.longDelayUsTotalUs());
    TEST_ASSERT_EQUAL_UINT32(50000, fake.lastDelayUs());
    TEST_ASSERT_EQUAL_UINT64(callbackElapsedUs, fake.elapsedUs());
  }
}

void test_operation_timing_bounds_are_exact_for_every_kind() {
  FakeE2Transport fake;
  const Config cfg = fake.makeConfig();
  struct Case {
    OperationKind kind;
    uint16_t count;
    uint32_t expectedMs;
  };
  const Case cases[] = {
      {OperationKind::CONTROL_READ, 1, 156},
      {OperationKind::CUSTOM_POINTER_WRITE, 1, 316},
      {OperationKind::CUSTOM_BYTE_READ, 1, 471},
      {OperationKind::CUSTOM_BLOCK_READ, 3, 781},
      {OperationKind::CUSTOM_BYTE_WRITE_VERIFY, 1, 786},
      {OperationKind::INTERVAL_WRITE_VERIFY, 1, 1282},
      {OperationKind::PART_NAME_WRITE_VERIFY, 1, 12573},
      {OperationKind::RAW_CO2_READ, 1, 311},
      {OperationKind::BUS_RESET, 1, 252},
      {OperationKind::BEGIN_REQUIRE_PRESENT, 1, 2274},
      {OperationKind::BEGIN_ALLOW_ABSENT, 1, 2274},
      {OperationKind::PROBE_IDENTITY, 1, 621},
      {OperationKind::RECOVER_IDENTITY_AND_CAPABILITIES, 1, 2274},
      {OperationKind::CHECKED_CO2_AVERAGE, 1, 936},
      {OperationKind::CHECKED_CO2_FAST, 1, 936},
      {OperationKind::CUSTOM_BLOCK_WRITE_VERIFY, 2, 1572},
      {OperationKind::CUSTOM_BLOCK_WRITE_VERIFY, 3, 2358},
      {OperationKind::CUSTOM_BLOCK_WRITE_VERIFY, 16, 12573},
      {OperationKind::RESYNC_PERSISTENT_CONFIG, 1, 7027},
      {OperationKind::AUTO_ADJUST_MAINTENANCE, 1, 1257},
      {OperationKind::BUS_ADDRESS_CHANGE, 1, 316},
  };

  for (const Case& item : cases) {
    OperationTimingBound out;
    TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
        cfg, item.kind, item.count, out).ok());
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(item.kind),
        static_cast<uint8_t>(out.kind));
    TEST_ASSERT_EQUAL_UINT16(item.count, out.elementCount);
    TEST_ASSERT_EQUAL_UINT32(item.expectedMs, out.maxBlockingMs);
  }
}

void test_operation_timing_queries_are_bus_silent_and_validate_counts() {
  FakeE2Transport fake;
  Config cfg = fake.makeConfig();
  OperationTimingBound out{
      OperationKind::BUS_RESET, 99, 0xA5A5A5A5U};
  fake.resetElapsed();
  const uint32_t readsBefore = fake.lineReads();
  const uint32_t writesBefore = fake.lineWrites();

  TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
      cfg, OperationKind::CONTROL_READ, 1, out).ok());
  TEST_ASSERT_EQUAL_UINT32(readsBefore, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(writesBefore, fake.lineWrites());
  TEST_ASSERT_EQUAL_UINT64(0, fake.elapsedUs());
  TEST_ASSERT_EQUAL_UINT32(0, fake.delayCalls());
  TEST_ASSERT_EQUAL_UINT32(0, fake.longDelaySlices());
  TEST_ASSERT_EQUAL_UINT32(0, fake.yieldCount());

  cfg.clockLowUs = 99;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_CONFIG),
      static_cast<uint8_t>(EE871::EE871::operationTimingBound(
          cfg, OperationKind::CONTROL_READ, 1, out).code));
  TEST_ASSERT_EQUAL_UINT32(readsBefore, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(writesBefore, fake.lineWrites());

  cfg = fake.makeConfig();
  const OperationTimingBound sentinel{
      OperationKind::BUS_RESET, 99, 0xA5A5A5A5U};
  out = sentinel;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(EE871::EE871::operationTimingBound(
          cfg, OperationKind::CONTROL_READ, 2, out).code));
  TEST_ASSERT_EQUAL_UINT32(sentinel.maxBlockingMs, out.maxBlockingMs);
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(EE871::EE871::operationTimingBound(
          cfg, OperationKind::CUSTOM_BLOCK_READ, 0, out).code));
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::INVALID_PARAM),
      static_cast<uint8_t>(EE871::EE871::operationTimingBound(
          cfg, OperationKind::CUSTOM_BLOCK_READ, 257, out).code));
  TEST_ASSERT_EQUAL_UINT32(readsBefore, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(writesBefore, fake.lineWrites());

  EE871::EE871 dev;
  TEST_ASSERT_EQUAL_UINT8(
      static_cast<uint8_t>(Err::NOT_INITIALIZED),
      static_cast<uint8_t>(dev.operationTimingBound(
          OperationKind::CONTROL_READ, 1, out).code));
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  fake.resetActivityCounters();
  TEST_ASSERT_TRUE(dev.operationTimingBound(
      OperationKind::CONTROL_READ, 1, out).ok());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
  TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
}

void test_lifecycle_timing_bounds_are_bus_silent_and_conservative() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    Config cfg = fake.makeConfig();
    OperationTimingBound bound;
    TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
        cfg, OperationKind::BEGIN_REQUIRE_PRESENT, 1, bound).ok());
    TEST_ASSERT_EQUAL_UINT32(0, fake.lineReads());
    TEST_ASSERT_EQUAL_UINT32(0, fake.lineWrites());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <= static_cast<uint64_t>(bound.maxBlockingMs) * 1000U);

    TEST_ASSERT_TRUE(dev.operationTimingBound(
        OperationKind::PROBE_IDENTITY, 1, bound).ok());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.probe().ok());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <= static_cast<uint64_t>(bound.maxBlockingMs) * 1000U);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    Config cfg = fake.makeConfig();
    cfg.beginPolicy = BeginPolicy::ALLOW_ABSENT;
    fake.setDevicePresent(false);
    OperationTimingBound bound;
    TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
        cfg, OperationKind::BEGIN_ALLOW_ABSENT, 1, bound).ok());
    fake.resetElapsed();
    TEST_ASSERT_EQUAL_UINT8(
        static_cast<uint8_t>(Err::NACK),
        static_cast<uint8_t>(dev.begin(cfg).code));
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <= static_cast<uint64_t>(bound.maxBlockingMs) * 1000U);

    fake.setDevicePresent(true);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    forceOfflineByNack(dev, fake);
    TEST_ASSERT_TRUE(dev.operationTimingBound(
        OperationKind::RECOVER_IDENTITY_AND_CAPABILITIES, 1, bound).ok());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.recover().ok());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <= static_cast<uint64_t>(bound.maxBlockingMs) * 1000U);
  }
}

void test_largest_valid_timing_bound_is_exact_and_does_not_wrap() {
  FakeE2Transport fake;
  Config cfg = makeLargestValidTimingConfig(fake);
  OperationTimingBound out;

  TEST_ASSERT_TRUE(EE871::EE871::operationTimingBound(
      cfg, OperationKind::CUSTOM_BLOCK_READ, 256, out).ok());
  TEST_ASSERT_EQUAL_UINT32(112246, out.maxBlockingMs);

  EE871::EE871 dev;
  TEST_ASSERT_TRUE(dev.begin(cfg).ok());
  fake.resetElapsed();
  uint8_t values[cmd::CUSTOM_MEMORY_SIZE] = {};
  TEST_ASSERT_TRUE(dev.customRead(0, values, sizeof(values)).ok());
  TEST_ASSERT_TRUE(
      fake.elapsedUs() <=
      static_cast<uint64_t>(out.maxBlockingMs) * 1000U);
}

void test_largest_valid_timing_bounds_cover_production_primitives() {
  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    const Config cfg = makeLargestValidTimingConfig(fake);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    OperationTimingBound out;
    TEST_ASSERT_TRUE(dev.operationTimingBound(
        OperationKind::BUS_RESET, 1, out).ok());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.busReset().ok());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <=
        static_cast<uint64_t>(out.maxBlockingMs) * 1000U);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    const Config cfg = makeLargestValidTimingConfig(fake);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    OperationTimingBound out;
    TEST_ASSERT_TRUE(dev.operationTimingBound(
        OperationKind::INTERVAL_WRITE_VERIFY, 1, out).ok());
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.writeMeasurementInterval(300).ok());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <=
        static_cast<uint64_t>(out.maxBlockingMs) * 1000U);
  }

  {
    FakeE2Transport fake;
    EE871::EE871 dev;
    const Config cfg = makeLargestValidTimingConfig(fake);
    TEST_ASSERT_TRUE(dev.begin(cfg).ok());
    OperationTimingBound partNameBound;
    TEST_ASSERT_TRUE(dev.operationTimingBound(
        OperationKind::PART_NAME_WRITE_VERIFY,
        1,
        partNameBound).ok());
    TEST_ASSERT_EQUAL_UINT32(180412, partNameBound.maxBlockingMs);
    OperationTimingBound genericBound;
    TEST_ASSERT_TRUE(dev.operationTimingBound(
        OperationKind::CUSTOM_BLOCK_WRITE_VERIFY,
        cmd::CUSTOM_PART_NAME_LEN,
        genericBound).ok());
    TEST_ASSERT_EQUAL_UINT32(
        partNameBound.maxBlockingMs, genericBound.maxBlockingMs);

    uint8_t partName[cmd::CUSTOM_PART_NAME_LEN] = {};
    fake.resetElapsed();
    TEST_ASSERT_TRUE(dev.writePartName(partName).ok());
    TEST_ASSERT_TRUE(
        fake.elapsedUs() <=
        static_cast<uint64_t>(partNameBound.maxBlockingMs) * 1000U);
  }
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_status_ok);
  RUN_TEST(test_status_error);
  RUN_TEST(test_status_in_progress);
  RUN_TEST(test_config_defaults);
  RUN_TEST(test_default_timing_config_operates_on_healthy_bus);
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
  RUN_TEST(test_invalid_begin_policy_is_bus_silent_invalid_config);
  RUN_TEST(test_strict_and_optional_absent_begin_contracts);
  RUN_TEST(test_allow_absent_rejects_non_absence_transport_faults);
  RUN_TEST(test_all_lifecycle_identity_paths_fail_closed);
  RUN_TEST(test_begin_capability_load_is_complete_ordered_and_atomic);
  RUN_TEST(test_every_capability_reserved_bit_fails_closed_atomically);
  RUN_TEST(test_direct_capability_reads_share_masks_and_preserve_outputs);
  RUN_TEST(
      test_recovery_capability_semantics_clear_live_claims_from_every_state);
  RUN_TEST(
      test_recover_rejects_each_invalid_capability_without_partial_cache);
  RUN_TEST(test_e2_spec_version_remains_diagnostic_only);
  RUN_TEST(test_identity_capability_and_settings_access_is_bus_silent);
  RUN_TEST(test_checked_sample_public_contract_defaults);
  RUN_TEST(test_checked_average_and_fast_success_order_and_evidence);
  RUN_TEST(test_checked_value_and_status_failure_evidence);
  RUN_TEST(test_checked_sensor_error_mapping_and_order);
  RUN_TEST(test_checked_sensor_error_capability_gate);
  RUN_TEST(test_checked_error_code_transfer_failures_are_precise);
  RUN_TEST(test_checked_range_raw_compatibility_and_health_domains);
  RUN_TEST(test_checked_methods_are_bus_silent_while_offline);
  RUN_TEST(test_co2_calibration_capability_helpers_are_bus_silent);
  RUN_TEST(test_checked_timing_bounds_are_bus_silent_and_conservative);
  RUN_TEST(test_clock_stretch_timeout_is_bounded_and_tracked);
  RUN_TEST(test_pec_mismatch_probe_is_raw_but_tracked_read_updates_health);
  RUN_TEST(test_device_absent_probe_has_no_health_side_effect_tracked_read_fails);
  RUN_TEST(test_custom_write_verify_mismatch_returns_precise_error);
  RUN_TEST(test_offline_threshold_and_recover_after_replug);
  RUN_TEST(test_offline_normal_operations_are_bus_silent_and_cannot_revive);
  RUN_TEST(test_runtime_failure_offline_has_same_explicit_recovery_latch);
  RUN_TEST(test_offline_probe_is_health_cache_and_diagnostic_neutral);
  RUN_TEST(test_successful_recover_reloads_and_atomically_publishes_cache);
  RUN_TEST(test_offline_recovery_reset_and_every_reload_failure_stays_offline);
  RUN_TEST(test_degraded_recovery_uses_transfer_health_and_semantic_latch);
  RUN_TEST(test_end_clears_policy_runtime_diagnostics_and_caches);
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
  RUN_TEST(test_mutation_contract_defaults_and_single_byte_effect_phases);
  RUN_TEST(test_typed_mutations_share_exact_target_evidence);
  RUN_TEST(test_part_name_failures_retain_exact_element_counts);
  RUN_TEST(test_interval_and_co2_pairs_report_exact_partial_progress);
  RUN_TEST(test_unresolved_mutation_blocks_mutations_but_allows_reads);
  RUN_TEST(test_target_resync_preserves_first_cause_and_records_outcome);
  RUN_TEST(test_every_ordinary_target_uses_target_specific_resync);
  RUN_TEST(test_recover_and_cache_getters_do_not_hide_uncertainty);
  RUN_TEST(test_bus_address_change_requires_explicit_candidate_session);
  RUN_TEST(
      test_auto_adjust_observation_resync_and_operator_acknowledgement);
  RUN_TEST(test_unresolved_mutation_survives_end_and_begin_attempts);
  RUN_TEST(test_typed_capability_guards_are_bus_silent);
  RUN_TEST(test_typed_persisted_reads_fail_closed_without_health_penalty);
  RUN_TEST(test_typed_persisted_value_boundaries_and_filter_remain_simple);
  RUN_TEST(test_typed_write_validation_precedence_is_bus_silent);
  RUN_TEST(test_auto_adjust_reserved_pre_and_post_states_fail_closed);
  RUN_TEST(test_semantic_invalid_post_write_uses_typed_validators);
  RUN_TEST(
      test_semantic_invalid_unresolved_resync_preserves_original_cause);
  RUN_TEST(test_full_resync_uses_all_typed_persisted_validators);
  RUN_TEST(test_raw_custom_write_protected_address_dispatch_is_exhaustive);
  RUN_TEST(test_interval_pair_uses_one_deferred_commit_then_reads_both);
  RUN_TEST(test_new_mutation_timing_kinds_cover_worst_case_fake_time);
  RUN_TEST(test_public_bus_api_first_transfer_failure_matrix);
  RUN_TEST(test_config_protocol_timing_boundaries);
  RUN_TEST(test_config_delay_normalization_and_limits);
  RUN_TEST(test_sda_low_before_start_is_bus_stuck_without_false_start);
  RUN_TEST(test_scl_stuck_is_precise_at_idle_reset_and_in_transaction);
  RUN_TEST(test_valid_stop_hold_above_bit_timeout_succeeds);
  RUN_TEST(test_bit_and_byte_deadline_boundaries);
  RUN_TEST(test_write_pre_pec_phases_keep_ordinary_deadlines);
  RUN_TEST(test_custom_write_completion_uses_one_total_budget);
  RUN_TEST(test_final_pec_nack_cleanup_uses_completion_budget);
  RUN_TEST(test_precise_primary_errors_precede_failed_stop_cleanup);
  RUN_TEST(test_pointer_stop_completion_and_dependent_read_ordering);
  RUN_TEST(test_startup_feature_reads_wait_for_pointer_completion);
  RUN_TEST(test_interval_commit_completion_boundaries);
  RUN_TEST(test_block_custom_read_uses_one_pointer_and_auto_increment);
  RUN_TEST(test_verify_mismatch_does_not_increment_transport_failures);
  RUN_TEST(test_long_wait_callbacks_are_sliced_and_bit_timing_does_not_yield);
  RUN_TEST(test_operation_timing_bounds_are_exact_for_every_kind);
  RUN_TEST(test_operation_timing_queries_are_bus_silent_and_validate_counts);
  RUN_TEST(test_lifecycle_timing_bounds_are_bus_silent_and_conservative);
  RUN_TEST(test_largest_valid_timing_bound_is_exact_and_does_not_wrap);
  RUN_TEST(test_largest_valid_timing_bounds_cover_production_primitives);
  return UNITY_END();
}

