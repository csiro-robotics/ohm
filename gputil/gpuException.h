// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUEXCEPTION_H
#define GPUEXCEPTION_H

#include "gpuConfig.h"
#include "gpuThrow.h"

#include <exception>

namespace gputil
{
/// Base class for exceptions thrown from this library.
class gputilAPI Exception : public std::exception
{
public:
  /// Construct an exception.
  /// @param msg Optional exception message.
  /// @param filename Optional filename where the exception was thrown.
  /// @param line_number Optional line number where the exception was thrown.
  Exception(const char *msg = nullptr, const char *filename = nullptr, int line_number = 0);
  /// Move constructor.
  /// @param other The object to move.
  Exception(Exception &&other) noexcept;

  /// Destructor.
  ~Exception() override;

  /// Query the exception message. This is a combination of the constructed message, filename and line number.
  /// @return The exception message.
  const char *what() const noexcept override;

protected:
  /// Build an exception message from the given base @p message , @p filename and @p line_number .
  ///
  /// Formatted as `<filename>(<line_number>): <message>` where possible. Optional items may be omitted.
  ///
  /// @param message The base exception message.
  /// @param filename Optional filename where the exception was thrown.
  /// @param line_number Optional line number where the exception was thrown.
  void setMessage(const char *message, const char *filename = nullptr, int line_number = 0);

private:
  char *message_;
};
}  // namespace gputil

#endif  // GPUEXCEPTION_H
