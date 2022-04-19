// Copyright (c) 2022
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_DUAL_RETURN_H
#define OHM_DUAL_RETURN_H

#include "OhmConfig.h"

#include <cinttypes>
#include <limits>

namespace ohm
{
/// Stores information about secondary samples (dual returns) collected in a voxel.
///
/// Lidar dual returns may be considered as secondary samples in OHM provdied the secondary sample layer is used.
/// Each voxel stores the information provided in this structure. This stores the number of secondary samples recorded
/// by a voxel and additional values to provide the mean distance between primary and secondary samples for this voxel
/// and the standard deviation thereof.
///
/// @note The @c range_mean is stored as a @c uint16_t value in order to keep the voxel size at 8 bytes. The range is
/// quantised to 1000th of the input value. Ohm generally expects metres, so the range is generally in millimetres.
/// The quantisation factor is exposed via @c secondarySampleQuantisationFactor() .
///
/// Use @c secondarySampleRangeMean() and @c secondarySampleRangeStdDev() in order to extract the range mean and
/// standard deviation values.
struct VoxelSecondarySample
{
  /// Used to calculate the standard deviation, @c m2 aggregates the squared distance from the mean.
  float m2;
  /// Standard mean distance between the primary sample and the secondary sample for secondary samples falling in this
  /// voxel.
  uint16_t range_mean;
  /// The number of secondary samples which have been collected in this voxel.
  uint16_t count;
};

/// Quantisation factor used for @c VoxelSecondarySample::range_mean . This relates the normal occupancy map units to
/// the quantised value using:
///
/// @code{.unparsed}
///   range_units = secondarySampleQuantisationFactor() * map_units
/// @endcode
///
/// @return A scalar for transforming from map units to @c VoxelSecondarySample::range_mean units.
inline constexpr double secondarySampleQuantisationFactor()
{
  return 1000.0;
}

/// The maximum range value which can be stored in @c VoxelSecondarySample::range_mean .
/// @return The maximum range value for @c VoxelSecondarySample::range_mean .
inline constexpr double secondarySampleMaxRange()
{
  return (std::numeric_limits<decltype(VoxelSecondarySample::range_mean)>::max() - 1u) /
         secondarySampleQuantisationFactor();
}

/// Extract the mean distance between primary and secondary samplex for @p voxel .
///
/// This converts @c VoxelSecondarySample::range_mean back into map units.
///
/// @param voxel The voxel to read.
/// @return The average distance between primary and secondary samples for @p voxel .
inline double secondarySampleRangeMean(const VoxelSecondarySample &voxel)
{
  return voxel.range_mean * secondarySampleQuantisationFactor();
}

/// Extract the standard deviation of the distance between primary and secondary samplex for @p voxel .
///
/// This converts @c VoxelSecondarySample::m2 into a standard deviation.
///
/// @param voxel The voxel to read.
/// @return The standard deviation of the distance between primary and secondary samples for @p voxel .
inline double secondarySampleRangeStdDev(const VoxelSecondarySample &voxel)
{
  return std::sqrt(double(voxel.m2) / double(voxel.count));
}

/// Update statistics for an additional secondary sample in @p voxel .
///
/// @param voxel The voxel to update in which the secondary sample lies.
/// @param range The distance between the primary and secondary samples. Must be positive.
inline void addSecondarySample(VoxelSecondarySample &voxel, double range)
{
  // Using Wellford's algorithm.
  // Clamp range
  range = std::min(range, secondarySampleMaxRange());
  double range_mean = voxel.range_mean * secondarySampleQuantisationFactor();
  ++voxel.count;
  const double delta = range - range_mean;
  range_mean += delta / voxel.count;
  voxel.range_mean = uint16_t(range_mean / secondarySampleQuantisationFactor());
  const double delta2 = range - range_mean;
  voxel.m2 += float(delta * delta2);
}

/// @overload
inline void addSecondarySample(Voxel<VoxelSecondarySample> &voxel, double range)
{
  VoxelSecondarySample data = voxel.data();
  addSecondarySample(data, range);
  voxel.write(data);
}
}  // namespace ohm

#endif  // OHM_DUAL_RETURN_H
