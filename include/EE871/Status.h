/// @file Status.h
/// @brief Error codes and status handling for EE871 driver
#pragma once

#include <cstdint>

namespace EE871 {

/// @brief Error codes for all fallible EE871 public operations.
///
/// Transport failures are mapped into these framework-neutral values; Arduino,
/// ESP-IDF, GPIO, or platform-specific error codes are not exposed by the core
/// API.
enum class Err : uint8_t {
  OK = 0,                    ///< Operation successful
  NOT_INITIALIZED,           ///< begin() not called
  INVALID_CONFIG,            ///< Invalid configuration parameter
  E2_ERROR,                  ///< E2 bus communication failure
  TIMEOUT,                   ///< Operation timed out
  INVALID_PARAM,             ///< Invalid parameter value
  DEVICE_NOT_FOUND,          ///< Device not responding on E2 bus
  PEC_MISMATCH,              ///< PEC validation failed
  NACK,                      ///< Missing ACK/NACK on bus
  BUSY,                      ///< Device is busy
  IN_PROGRESS,               ///< Operation scheduled; call tick() to complete
  BUS_STUCK,                 ///< Bus lines stuck (SDA or SCL held low)
  ALREADY_INITIALIZED,       ///< begin() called without end()
  OUT_OF_RANGE,              ///< Value out of valid range
  NOT_SUPPORTED,             ///< Feature not supported by this device/firmware
  CO2_SENSOR_ERROR           ///< EE871 status byte reports a CO2 measurement error
};

/// @brief Status structure returned by all fallible operations.
///
/// Status values are small, trivially copyable diagnostics. msg points to a
/// static string owned by the library or caller; it is not heap allocated and
/// callers must not free it.
struct Status {
  Err code = Err::OK;        ///< Framework-neutral status code.
  int32_t detail = 0;        ///< Operation-specific detail, such as address, actual readback byte, or E2 error code.
  const char* msg = "";      ///< Static string describing the status.

  /// @brief Construct an OK/empty status.
  constexpr Status() = default;

  /// @brief Construct a status explicitly.
  /// @param codeIn Framework-neutral status code.
  /// @param detailIn Operation-specific diagnostic detail.
  /// @param msgIn Static status message pointer.
  constexpr Status(Err codeIn, int32_t detailIn, const char* msgIn)
      : code(codeIn), detail(detailIn), msg(msgIn) {}

  /// @return true if operation succeeded.
  constexpr bool ok() const { return code == Err::OK; }
  
  /// @return true if operation is in progress and should not be counted as a failure.
  constexpr bool inProgress() const { return code == Err::IN_PROGRESS; }
  
  /// @brief Create a success status.
  /// @return Status with code Err::OK, detail 0, and message "OK".
  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }
  
  /// @brief Create an error status.
  /// @param err Framework-neutral error code.
  /// @param message Static diagnostic message.
  /// @param detailCode Optional operation-specific detail.
  /// @return Status containing the supplied error fields.
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

} // namespace EE871
