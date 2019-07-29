// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYTYPE_H
#define OCCUPANCYTYPE_H

#include "OhmConfig.h"

namespace ohm
{
  /// An enumeration of the types of @c Voxel states available.
  enum OccupancyType
  {
    /// Invalid/null voxel.
    kNull = -2,
    /// Uncertain: no data recorded or available for the voxel.
    kUncertain = -1,
    /// Know to be empty or free (traversable).
    kFree = 0,
    /// Occupied voxel.
    kOccupied = 1
  };

  const char ohm_API *occupancyTypeToString(int occupancy_type);
}  // namespace ohm

#endif  // OCCUPANCYTYPE_H_
