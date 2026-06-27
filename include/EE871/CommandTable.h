/// @file CommandTable.h
/// @brief Control bytes, custom addresses, and bit definitions for EE871 E2
#pragma once

#include <cstdint>

namespace EE871 {
namespace cmd {

// ============================================================================
// Control Byte Construction
// ============================================================================

static constexpr uint8_t DEVICE_ADDRESS_MIN = 0;     ///< Minimum E2 protocol device address.
static constexpr uint8_t DEVICE_ADDRESS_MAX = 7;     ///< Maximum E2 protocol device address.
static constexpr uint8_t DEFAULT_DEVICE_ADDRESS = 0; ///< Factory/default EE871 E2 address.

static constexpr uint8_t RW_READ = 0x01;   ///< Control-byte R/W bit for reads.
static constexpr uint8_t RW_WRITE = 0x00;  ///< Control-byte R/W bit for writes.
static constexpr uint8_t ADDR_SHIFT = 1;   ///< Bit offset of the 3-bit E2 device address.
static constexpr uint8_t MAIN_SHIFT = 4;   ///< Bit offset of the 4-bit main command nibble.

/// Build an E2 control byte from main command, device address, and direction.
/// @param mainCommandNibble Command nibble placed in bits 7..4.
/// @param deviceAddress E2 device address, masked to 0..7.
/// @param read true for read direction, false for write direction.
/// @return Encoded control byte.
static constexpr uint8_t makeControlByte(uint8_t mainCommandNibble,
                                         uint8_t deviceAddress,
                                         bool read) {
  return static_cast<uint8_t>((mainCommandNibble << MAIN_SHIFT) |
                              ((deviceAddress & 0x07) << ADDR_SHIFT) |
                              (read ? RW_READ : RW_WRITE));
}

/// Build a read-direction E2 control byte.
/// @param mainCommandNibble Command nibble placed in bits 7..4.
/// @param deviceAddress E2 device address, masked to 0..7.
/// @return Encoded read control byte.
static constexpr uint8_t makeControlRead(uint8_t mainCommandNibble,
                                         uint8_t deviceAddress) {
  return makeControlByte(mainCommandNibble, deviceAddress, true);
}

/// Build a write-direction E2 control byte.
/// @param mainCommandNibble Command nibble placed in bits 7..4.
/// @param deviceAddress E2 device address, masked to 0..7.
/// @return Encoded write control byte.
static constexpr uint8_t makeControlWrite(uint8_t mainCommandNibble,
                                          uint8_t deviceAddress) {
  return makeControlByte(mainCommandNibble, deviceAddress, false);
}

// ============================================================================
// Main Command Nibbles (b7..b4)
// ============================================================================

static constexpr uint8_t MAIN_TYPE_LO = 0x1;       ///< Read sensor group low byte; write direct custom byte (0x10).
static constexpr uint8_t MAIN_CUSTOM_WRITE = 0x1;  ///< Write direct custom memory byte (control byte 0x10 at address 0).
static constexpr uint8_t MAIN_TYPE_SUB = 0x2;      ///< Read sensor subgroup byte.
static constexpr uint8_t MAIN_AVAIL_MEAS = 0x3;    ///< Read available physical measurements bitfield.
static constexpr uint8_t MAIN_TYPE_HI = 0x4;       ///< Read sensor group high byte.
static constexpr uint8_t MAIN_CUSTOM_PTR = 0x5;    ///< Read custom memory (0x51) or set custom pointer (0x50).
static constexpr uint8_t MAIN_STATUS = 0x7;        ///< Read status byte; may trigger a new measurement.
static constexpr uint8_t MAIN_MV1_LO = 0x8;        ///< Measurement value 1 low byte; unsupported for EE871 CO2.
static constexpr uint8_t MAIN_MV1_HI = 0x9;        ///< Measurement value 1 high byte; unsupported for EE871 CO2.
static constexpr uint8_t MAIN_MV2_LO = 0xA;        ///< Measurement value 2 low byte; unsupported for EE871 CO2.
static constexpr uint8_t MAIN_MV2_HI = 0xB;        ///< Measurement value 2 high byte; unsupported for EE871 CO2.
static constexpr uint8_t MAIN_MV3_LO = 0xC;        ///< CO2 fast-response MV3 low byte.
static constexpr uint8_t MAIN_MV3_HI = 0xD;        ///< CO2 fast-response MV3 high byte.
static constexpr uint8_t MAIN_MV4_LO = 0xE;        ///< CO2 averaged MV4 low byte.
static constexpr uint8_t MAIN_MV4_HI = 0xF;        ///< CO2 averaged MV4 high byte.

/// Check whether a main-command read is defined for EE871 CO2 operation.
/// @param mainCommandNibble Command nibble before address/RW bits are added.
/// @return true for identity, custom-pointer read, status, MV3, and MV4 reads.
static constexpr bool isReadMainCommandSupported(uint8_t mainCommandNibble) {
  return mainCommandNibble == MAIN_TYPE_LO ||
         mainCommandNibble == MAIN_TYPE_SUB ||
         mainCommandNibble == MAIN_AVAIL_MEAS ||
         mainCommandNibble == MAIN_TYPE_HI ||
         mainCommandNibble == MAIN_CUSTOM_PTR ||
         mainCommandNibble == MAIN_STATUS ||
         mainCommandNibble == MAIN_MV3_LO ||
         mainCommandNibble == MAIN_MV3_HI ||
         mainCommandNibble == MAIN_MV4_LO ||
         mainCommandNibble == MAIN_MV4_HI;
}

// ============================================================================
// Device Identity and Capabilities
// ============================================================================

static constexpr uint16_t SENSOR_GROUP_ID = 0x0367;  ///< EE871 sensor group identifier.
static constexpr uint8_t SENSOR_SUBGROUP_ID = 0x09;  ///< EE871 sensor subgroup identifier.
static constexpr uint8_t AVAILABLE_MEAS_MASK = 0x08; ///< Available-measurements bit for CO2.

// ============================================================================
// Status / Measurements
// ============================================================================

static constexpr uint8_t STATUS_CO2_ERROR_BIT = 3;   ///< CO2 error bit index in the status byte.
static constexpr uint8_t STATUS_CO2_ERROR_MASK = 0x08; ///< CO2 error bit mask in the status byte.

// ============================================================================
// Custom Memory Map (0x00..0xFF)
// ============================================================================

static constexpr uint8_t CUSTOM_FW_VERSION_MAIN = 0x00; ///< Firmware main version custom-memory address.
static constexpr uint8_t CUSTOM_FW_VERSION_SUB = 0x01;  ///< Firmware sub version custom-memory address.
static constexpr uint8_t CUSTOM_E2_SPEC_VERSION = 0x02; ///< Device E2 specification version address.

static constexpr uint8_t CUSTOM_OPERATING_FUNCTIONS = 0x07;    ///< Feature-support flags address.
static constexpr uint8_t CUSTOM_OPERATING_MODE_SUPPORT = 0x08; ///< Operating-mode support flags address.
static constexpr uint8_t CUSTOM_SPECIAL_FEATURES = 0x09;       ///< Special-feature support flags address.

static constexpr uint8_t CUSTOM_CO2_OFFSET_L = 0x58;   ///< CO2 offset low byte, signed ppm.
static constexpr uint8_t CUSTOM_CO2_OFFSET_H = 0x59;   ///< CO2 offset high byte, signed ppm.
static constexpr uint8_t CUSTOM_CO2_GAIN_L = 0x5A;     ///< CO2 gain low byte, raw gain value.
static constexpr uint8_t CUSTOM_CO2_GAIN_H = 0x5B;     ///< CO2 gain high byte, raw gain value.
static constexpr uint8_t CUSTOM_CO2_POINT_L_L = 0x5C;  ///< Lower CO2 calibration point low byte.
static constexpr uint8_t CUSTOM_CO2_POINT_L_H = 0x5D;  ///< Lower CO2 calibration point high byte.
static constexpr uint8_t CUSTOM_CO2_POINT_U_L = 0x5E;  ///< Upper CO2 calibration point low byte.
static constexpr uint8_t CUSTOM_CO2_POINT_U_H = 0x5F;  ///< Upper CO2 calibration point high byte.

static constexpr uint8_t CUSTOM_SERIAL_START = 0xA0;    ///< First byte of 16-byte serial number.
static constexpr uint8_t CUSTOM_SERIAL_LEN = 16;        ///< Serial number byte length.
static constexpr uint8_t CUSTOM_PART_NAME_START = 0xB0; ///< First byte of 16-byte part name.
static constexpr uint8_t CUSTOM_PART_NAME_LEN = 16;     ///< Part name byte length.

static constexpr uint8_t CUSTOM_BUS_ADDRESS = 0xC0; ///< Persistent E2 bus address register.
static constexpr uint8_t CUSTOM_ERROR_CODE = 0xC1;  ///< CO2 error-code register.

static constexpr uint8_t CUSTOM_INTERVAL_L = 0xC6;            ///< Global measurement interval low byte.
static constexpr uint8_t CUSTOM_INTERVAL_H = 0xC7;            ///< Global measurement interval high byte.
static constexpr uint8_t CUSTOM_CO2_INTERVAL_FACTOR = 0xCB;   ///< CO2-specific interval factor register.

static constexpr uint8_t CUSTOM_FILTER_CO2 = 0xD3;      ///< CO2 filter setting register.
static constexpr uint8_t CUSTOM_OPERATING_MODE = 0xD8;  ///< Runtime operating mode register.
static constexpr uint8_t CUSTOM_AUTO_ADJUST = 0xD9;     ///< Auto-adjust control/status register.

static constexpr uint8_t CUSTOM_POINTER_LOW = 0xFE;     ///< Low byte of custom pointer visibility register.
static constexpr uint8_t CUSTOM_POINTER_HIGH = 0xFF;    ///< High byte of custom pointer visibility register.
static constexpr uint16_t CUSTOM_MEMORY_SIZE = 256;     ///< EE871 custom-memory address space size.

// ============================================================================
// Interval / Range Limits
// ============================================================================

static constexpr uint16_t INTERVAL_MIN_DECISEC = 150;   ///< Minimum global interval, 15.0 s.
static constexpr uint16_t INTERVAL_MAX_DECISEC = 36000; ///< Maximum global interval, 3600.0 s.
static constexpr uint16_t CO2_PPM_MIN = 0;              ///< Minimum documented CO2 value in ppm.
static constexpr uint16_t CO2_PPM_MAX = 50000;          ///< Maximum documented CO2 value in ppm.
static constexpr uint8_t BUS_ADDRESS_MIN = 0;           ///< Minimum persistent bus address.
static constexpr uint8_t BUS_ADDRESS_MAX = 7;           ///< Maximum persistent bus address.

static constexpr uint8_t BUS_RESET_CLOCKS = 9; ///< Minimum clocks with SDA high to reset slave state machine.

static constexpr uint32_t WRITE_DELAY_MAX_MS = 5000; ///< Maximum accepted 0x10/0x50 write delay configuration.
static constexpr uint32_t INTERVAL_WRITE_DELAY_MAX_MS = 5000; ///< Maximum accepted interval write delay configuration.

// ============================================================================
// CO2 Error Codes (read from custom memory 0xC1 when status bit3 set)
// ============================================================================

static constexpr uint8_t CO2_ERROR_SUPPLY_VOLTAGE_LOW = 1;       ///< Supply voltage low CO2 error code.
static constexpr uint8_t CO2_ERROR_SENSOR_COUNTS_LOW = 200;      ///< Sensor counts low CO2 error code.
static constexpr uint8_t CO2_ERROR_SENSOR_COUNTS_HIGH = 201;     ///< Sensor counts high CO2 error code.
static constexpr uint8_t CO2_ERROR_SUPPLY_VOLTAGE_BREAKDOWN = 202; ///< Supply voltage breakdown-at-peak CO2 error code.

/// Convert a documented EE871 CO2 error code to a static string.
/// @param code Error code read from custom memory 0xC1.
/// @return Static error description, or "unknown CO2 error".
inline const char* co2ErrorCodeName(uint8_t code) {
  switch (code) {
    case 0:
      return "no error";
    case CO2_ERROR_SUPPLY_VOLTAGE_LOW:
      return "supply voltage low";
    case CO2_ERROR_SENSOR_COUNTS_LOW:
      return "sensor counts low";
    case CO2_ERROR_SENSOR_COUNTS_HIGH:
      return "sensor counts high";
    case CO2_ERROR_SUPPLY_VOLTAGE_BREAKDOWN:
      return "supply voltage breakdown at peak";
    default:
      return "unknown CO2 error";
  }
}

// ============================================================================
// Feature Flags
// ============================================================================

// CUSTOM_OPERATING_FUNCTIONS (0x07)
static constexpr uint8_t FEATURE_SERIAL_NUMBER = 0x01;      ///< Serial number support bit in 0x07.
static constexpr uint8_t FEATURE_PART_NAME = 0x02;          ///< Part name support bit in 0x07.
static constexpr uint8_t FEATURE_ADDRESS_CONFIG = 0x04;     ///< Bus-address configuration support bit in 0x07.
static constexpr uint8_t FEATURE_GLOBAL_INTERVAL = 0x10;    ///< Global interval configuration support bit in 0x07.
static constexpr uint8_t FEATURE_SPECIFIC_INTERVAL = 0x20;  ///< Specific interval configuration support bit in 0x07.
static constexpr uint8_t FEATURE_FILTER_CONFIG = 0x40;      ///< Filter configuration support bit in 0x07.
static constexpr uint8_t FEATURE_ERROR_CODE = 0x80;         ///< Error-code register support bit in 0x07.

// CUSTOM_OPERATING_MODE_SUPPORT (0x08)
static constexpr uint8_t MODE_SUPPORT_LOW_POWER = 0x01;  ///< Low-power mode support bit in 0x08.
static constexpr uint8_t MODE_SUPPORT_E2_PRIORITY = 0x02; ///< E2-priority mode support bit in 0x08.

// CUSTOM_SPECIAL_FEATURES (0x09)
static constexpr uint8_t SPECIAL_FEATURE_AUTO_ADJUST = 0x01; ///< Auto-adjust support bit in 0x09.

// CUSTOM_OPERATING_MODE (0xD8)
static constexpr uint8_t OPERATING_MODE_MEASUREMODE_MASK = 0x01; ///< Runtime mode bit: low-power vs freerunning.
static constexpr uint8_t OPERATING_MODE_E2_PRIORITY_MASK = 0x02; ///< Runtime mode bit: E2 priority vs measurement priority.

// CUSTOM_AUTO_ADJUST (0xD9)
static constexpr uint8_t AUTO_ADJUST_RUNNING_MASK = 0x01; ///< Auto-adjust running bit in 0xD9.

} // namespace cmd
} // namespace EE871
