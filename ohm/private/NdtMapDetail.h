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

  struct NdtMapDetail
  {
    /// The target occupancy map.
    OccupancyMap *map = nullptr;
    /// Range sensor noise estimate
    float sensor_noise = 0.05f;
    /// Number of samples required before using NDT logic in a miss integration.
    unsigned sample_threshold = 4;
    /// Low probability value trehsold used to re-initialise covariance matrix and mean.
    /// Used with @c reinitialise_covariance_point_count in @c calculateHitWithCovariance()
    float reinitialise_covariance_theshold = valueToProbability(0.1f);
    /// Upper point count limit required to reinitialise the covariance matrix. Used with @c
    /// reinitialise_covariance_theshold in @c calculateHitWithCovariance()
    unsigned reinitialise_covariance_point_count = 10;
    /// True if @p map is a borrowed pointer, false to take ownership and delete it.
    bool borrowed_map = false;
    /// Debug tracing enabled? Requires 3es
    bool trace = false;
  };
}  // namespace ohm

#endif // NDTMAPDETAIL_H
