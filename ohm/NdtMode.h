//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2021
//
#ifndef OHM_NDTMODE_H
#define OHM_NDTMODE_H

#include "OhmConfig.h"

#include <string>
namespace ohm
{
/// NDT mapping mode
enum class NdtMode
{
  kNone,  ///< Ndt not in use.
  /// Ndt occupancy map mode. This uses the occupancy, mean and covariance layers.
  kOccupancy,
  /// Ndt traversability map. A super set of @c Mode::kOccupancy which adds intensity and covariance hit/miss count.
  kTraversability  ///< Ndt traversability mode.
};

/// Convert an @c NdtMode value to a display string. @c "<unknown>" on failure.
std::string ohm_API ndtModeToString(NdtMode mode);
/// Convert a string value to an @c NdtMode or @c NdtMode::kNone on failure. Inverse of @c ndtModeToString()
NdtMode ohm_API ndtModeFromString(const std::string &str);
}  // namespace ohm

#endif  // OHM_NDTMODE_H
