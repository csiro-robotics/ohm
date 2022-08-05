// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUGPUAPICHECK_H
#define GPUGPUAPICHECK_H

#include "gpuConfig.h"

#include "gpuException.h"

namespace gputil
{
/// Raised due to an exception in the underlying GPU SDK.
class gputilAPI ApiException : public Exception  // NOLINT(cppcoreguidelines-special-member-functions)
{
public:
  /// Construct an API exception.
  /// @param error_code The underlying SDK error code - e.g., a @c cudaError_t .
  /// @param msg Optional test for the api error. When null, the @c errorCodeString() for @p error_code is used.
  /// @param filename Optional name of file the exception is thrown from.
  /// @param line_number Optional line number where the exception is thrown.
  ApiException(int error_code, const char *msg, const char *filename = nullptr, int line_number = 0);

  /// @overload
  explicit inline ApiException(int error_code)
    : ApiException(error_code, nullptr)
  {}

  /// Move constructor.
  /// @param other Object to move.
  ApiException(ApiException &&other) noexcept;

  /// Copy constructor.
  /// @param other Object to copy.
  ApiException(const ApiException &other) noexcept;

  /// A helper function which converts the @p error_code into a string message.
  /// @param error_code The GPU API error code to retrieve a message for - e.g., a @c cudaError_t .
  /// @return A string which identifies the @p error_code . Some codes may be unknown.
  static const char *errorCodeString(int error_code);

  /// Query the error code value for the exception.
  /// @return The underlying SDK error code - e.g., a @c cudaError_t .
  inline int errorCode() const { return error_code_; }

private:
  int error_code_;
};
}  // namespace gputil

#endif  // GPUGPUAPICHECK_H
