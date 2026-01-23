/// @file Status.h
/// @brief Error codes and status handling for EE871 driver
#pragma once

#include <cstdint>

namespace ee871 {

/// Error codes for all EE871 operations
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
  IN_PROGRESS                ///< Operation scheduled; call tick() to complete
};

/// Status structure returned by all fallible operations
struct Status {
  Err code = Err::OK;
  int32_t detail = 0;        ///< Implementation-specific detail (e.g., E2 error code)
  const char* msg = "";      ///< Static string describing the error

  /// @return true if operation succeeded
  constexpr bool ok() const { return code == Err::OK; }
  
  /// @return true if operation in progress (not a failure)
  constexpr bool inProgress() const { return code == Err::IN_PROGRESS; }
  
  /// Create a success status
  static constexpr Status Ok() { return Status{Err::OK, 0, "OK"}; }
  
  /// Create an error status
  static constexpr Status Error(Err err, const char* message, int32_t detailCode = 0) {
    return Status{err, detailCode, message};
  }
};

} // namespace ee871
