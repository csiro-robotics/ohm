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
    Null = -2,
    /// Uncertain: no data recorded or available for the voxel.
    Uncertain = -1,
    /// Know to be empty or free (traversable).
    Free = 0,
    /// Occupied voxel.
    Occupied = 1
  };

  const char ohm_API *occupancyTypeToString(int occupancy_type);
}  // namespace ohm

#endif  // OCCUPANCYTYPE_H_
