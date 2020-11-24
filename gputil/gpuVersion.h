// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUVERSION_H
#define GPUVERSION_H

#include "gpuConfig.h"

#include <cinttypes>

namespace gputil
{
/// Version number structure for the GPU device/API split as `<major>.<minor>.<patch>`.
struct Version
{
  uint16_t major = 0;  ///< Major version part.
  uint16_t minor = 0;  ///< Minor version part.
  uint16_t patch = 0;  ///< Patch version part.

  /// Equality operator.
  /// @param other The object to compare against.
  /// @return True if the two versions exactly match.
  inline bool operator==(const Version &other) const
  {
    return major == other.major && minor == other.minor && patch == other.patch;
  }
};
}  // namespace gputil

#endif  // GPUVERSION_H
