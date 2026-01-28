/// @file CommandTable.h
/// @brief Control bytes, custom addresses, and bit definitions for EE871 E2
#pragma once

#include <cstdint>

namespace ee871 {
namespace cmd {

// ============================================================================
// Control Byte Construction
// ============================================================================

static constexpr uint8_t DEVICE_ADDRESS_MIN = 0;
static constexpr uint8_t DEVICE_ADDRESS_MAX = 7;
static constexpr uint8_t DEFAULT_DEVICE_ADDRESS = 0;

static constexpr uint8_t RW_READ = 0x01;
static constexpr uint8_t RW_WRITE = 0x00;
static constexpr uint8_t ADDR_SHIFT = 1;
static constexpr uint8_t MAIN_SHIFT = 4;

static constexpr uint8_t makeControlByte(uint8_t mainCommandNibble,
                                         uint8_t deviceAddress,
                                         bool read) {
  return static_cast<uint8_t>((mainCommandNibble << MAIN_SHIFT) |
                              ((deviceAddress & 0x07) << ADDR_SHIFT) |
                              (read ? RW_READ : RW_WRITE));
}

static constexpr uint8_t makeControlRead(uint8_t mainCommandNibble,
                                         uint8_t deviceAddress) {
  return makeControlByte(mainCommandNibble, deviceAddress, true);
}

static constexpr uint8_t makeControlWrite(uint8_t mainCommandNibble,
                                          uint8_t deviceAddress) {
  return makeControlByte(mainCommandNibble, deviceAddress, false);
}

// ============================================================================
// Main Command Nibbles (b7..b4)
// ============================================================================

static constexpr uint8_t MAIN_TYPE_LO = 0x1;       // Read: type low, Write: custom write (0x10)
static constexpr uint8_t MAIN_CUSTOM_WRITE = 0x1;  // Write: direct custom byte (0x10)
static constexpr uint8_t MAIN_TYPE_SUB = 0x2;
static constexpr uint8_t MAIN_AVAIL_MEAS = 0x3;
static constexpr uint8_t MAIN_TYPE_HI = 0x4;
static constexpr uint8_t MAIN_CUSTOM_PTR = 0x5;   // Read=0x51, Write=0x50
static constexpr uint8_t MAIN_STATUS = 0x7;
static constexpr uint8_t MAIN_MV1_LO = 0x8;
static constexpr uint8_t MAIN_MV1_HI = 0x9;
static constexpr uint8_t MAIN_MV2_LO = 0xA;
static constexpr uint8_t MAIN_MV2_HI = 0xB;
static constexpr uint8_t MAIN_MV3_LO = 0xC;
static constexpr uint8_t MAIN_MV3_HI = 0xD;
static constexpr uint8_t MAIN_MV4_LO = 0xE;
static constexpr uint8_t MAIN_MV4_HI = 0xF;

// ============================================================================
// Device Identity and Capabilities
// ============================================================================

static constexpr uint16_t SENSOR_GROUP_ID = 0x0367;    // EE871 group
static constexpr uint8_t SENSOR_SUBGROUP_ID = 0x09;
static constexpr uint8_t AVAILABLE_MEAS_MASK = 0x08;   // Bit3 = CO2

// ============================================================================
// Status / Measurements
// ============================================================================

static constexpr uint8_t STATUS_CO2_ERROR_BIT = 3;
static constexpr uint8_t STATUS_CO2_ERROR_MASK = 0x08;

// ============================================================================
// Custom Memory Map (0x00..0xFF)
// ============================================================================

static constexpr uint8_t CUSTOM_FW_VERSION_MAIN = 0x00;
static constexpr uint8_t CUSTOM_FW_VERSION_SUB = 0x01;
static constexpr uint8_t CUSTOM_E2_SPEC_VERSION = 0x02;

static constexpr uint8_t CUSTOM_OPERATING_FUNCTIONS = 0x07;
static constexpr uint8_t CUSTOM_OPERATING_MODE_SUPPORT = 0x08;
static constexpr uint8_t CUSTOM_SPECIAL_FEATURES = 0x09;

// CO2 calibration (signed offset, unsigned gain and points)
static constexpr uint8_t CUSTOM_CO2_OFFSET_L = 0x58;
static constexpr uint8_t CUSTOM_CO2_OFFSET_H = 0x59;
static constexpr uint8_t CUSTOM_CO2_GAIN_L = 0x5A;
static constexpr uint8_t CUSTOM_CO2_GAIN_H = 0x5B;
static constexpr uint8_t CUSTOM_CO2_POINT_L_L = 0x5C;
static constexpr uint8_t CUSTOM_CO2_POINT_L_H = 0x5D;
static constexpr uint8_t CUSTOM_CO2_POINT_U_L = 0x5E;
static constexpr uint8_t CUSTOM_CO2_POINT_U_H = 0x5F;

static constexpr uint8_t CUSTOM_SERIAL_START = 0xA0;
static constexpr uint8_t CUSTOM_SERIAL_LEN = 16;
static constexpr uint8_t CUSTOM_PART_NAME_START = 0xB0;
static constexpr uint8_t CUSTOM_PART_NAME_LEN = 16;

static constexpr uint8_t CUSTOM_BUS_ADDRESS = 0xC0;
static constexpr uint8_t CUSTOM_ERROR_CODE = 0xC1;

static constexpr uint8_t CUSTOM_INTERVAL_L = 0xC6;
static constexpr uint8_t CUSTOM_INTERVAL_H = 0xC7;
static constexpr uint8_t CUSTOM_CO2_INTERVAL_FACTOR = 0xCB;

static constexpr uint8_t CUSTOM_FILTER_CO2 = 0xD3;
static constexpr uint8_t CUSTOM_OPERATING_MODE = 0xD8;
static constexpr uint8_t CUSTOM_AUTO_ADJUST = 0xD9;

static constexpr uint8_t CUSTOM_POINTER_LOW = 0xFE;
static constexpr uint8_t CUSTOM_POINTER_HIGH = 0xFF;

// ============================================================================
// Interval / Range Limits
// ============================================================================

static constexpr uint16_t INTERVAL_MIN_DECISEC = 150;   // 15.0 s
static constexpr uint16_t INTERVAL_MAX_DECISEC = 36000; // 3600.0 s
static constexpr uint8_t BUS_ADDRESS_MIN = 0;
static constexpr uint8_t BUS_ADDRESS_MAX = 7;

// Bus reset: minimum clock pulses with SDA high to reset slave state machine
static constexpr uint8_t BUS_RESET_CLOCKS = 9;

// ============================================================================
// Feature Flags
// ============================================================================

// CUSTOM_OPERATING_FUNCTIONS (0x07)
static constexpr uint8_t FEATURE_SERIAL_NUMBER = 0x01;
static constexpr uint8_t FEATURE_PART_NAME = 0x02;
static constexpr uint8_t FEATURE_ADDRESS_CONFIG = 0x04;
static constexpr uint8_t FEATURE_GLOBAL_INTERVAL = 0x10;
static constexpr uint8_t FEATURE_SPECIFIC_INTERVAL = 0x20;
static constexpr uint8_t FEATURE_FILTER_CONFIG = 0x40;
static constexpr uint8_t FEATURE_ERROR_CODE = 0x80;

// CUSTOM_OPERATING_MODE_SUPPORT (0x08)
static constexpr uint8_t MODE_SUPPORT_LOW_POWER = 0x01;
static constexpr uint8_t MODE_SUPPORT_E2_PRIORITY = 0x02;

// CUSTOM_SPECIAL_FEATURES (0x09)
static constexpr uint8_t SPECIAL_FEATURE_AUTO_ADJUST = 0x01;

// CUSTOM_OPERATING_MODE (0xD8)
static constexpr uint8_t OPERATING_MODE_MEASUREMODE_MASK = 0x01;
static constexpr uint8_t OPERATING_MODE_E2_PRIORITY_MASK = 0x02;

// CUSTOM_AUTO_ADJUST (0xD9)
static constexpr uint8_t AUTO_ADJUST_RUNNING_MASK = 0x01;

} // namespace cmd
} // namespace ee871
