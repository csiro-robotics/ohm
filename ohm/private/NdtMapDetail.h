// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef NDTMAPDETAIL_H
#define NDTMAPDETAIL_H

#include "OhmConfig.h"

#include "MapProbability.h"

namespace ohm
{
class OccupancyMap;

/// Internal details associated with a @c NdtMap extension to an @c OccupancyMap .
struct NdtMapDetail
{
  /// The target occupancy map.
  OccupancyMap *map = nullptr;
  /// Range sensor noise estimate
  float sensor_noise = 0.05f;  // NOLINT(readability-magic-numbers)
  /// Number of samples required before using NDT logic in a miss integration.
  unsigned sample_threshold = 3;
  /// Rate at which ray intersections with NDT ellipsoids errode voxels. Range [0, 1] with 1 yielding stronger
  /// effects.
  float adaptation_rate = 0.7f;  // NOLINT(readability-magic-numbers)
  /// Low probability value threshold used to re-initialise covariance matrix and mean.
  /// Used with @c reinitialise_covariance_point_count in @c calculateHitWithCovariance()
  float reinitialise_covariance_threshold = probabilityToValue(0.2f);  // NOLINT(readability-magic-numbers)
  /// Upper point count limit required to reinitialise the covariance matrix. Used with
  /// @c reinitialise_covariance_threshold in @c calculateHitWithCovariance()
  unsigned reinitialise_covariance_point_count = 100;
  /// True if @p map is a borrowed pointer, false to take ownership and delete it.
  bool borrowed_map = false;
  /// Debug tracing enabled? Requires 3es
  bool trace = false;
};
}  // namespace ohm

#endif  // NDTMAPDETAIL_H
