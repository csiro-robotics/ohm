// Copyright (c) 2022
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_DUAL_RETURN_H
#define OHM_DUAL_RETURN_H

#include "OhmConfig.h"

#include <cinttypes>

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
/// quantised to 100th of the input value. Ohm generally expects metres, so the range is generally in centimetres.
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

inline constexpr double secondarySampleQuantisationFactor()
{
  return 100.0;
}

inline double secondarySampleRangeMean(const VoxelSecondarySample &voxel)
{
  return voxel.range_mean * secondarySampleQuantisationFactor();
}

inline double secondarySampleRangeStdDev(const VoxelSecondarySample &voxel)
{
  return std::sqrt(double(voxel.m2) / double(voxel.count));
}

inline void addSecondarySample(VoxelSecondarySample &voxel, double range_to_primary)
{
  // Using Wellford's algorithm.
  double range_mean = range_mean * secondarySampleQuantisationFactor();
  ++voxel.count;
  const double delta = range_to_primary - range_mean;
  range_mean += delta / voxel.count;
  voxel.range_mean = uint16_t(range_mean / secondarySampleQuantisationFactor());
  const double delta2 = range_to_primary - range_mean;
  voxel.m2 += float(delta * delta2);
}

inline void addSecondarySample(Voxel<VoxelSecondarySample> &voxel, double range_to_primary)
{
  VoxelSecondarySample data = voxel.data();
  addSecondarySample(data, range_to_primary);
  voxel.write(data);
}
}  // namespace ohm

#endif  // OHM_DUAL_RETURN_H
