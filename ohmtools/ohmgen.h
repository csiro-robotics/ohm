// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGEN_H_
#define OHMGEN_H_

#include "ohmtoolsconfig.h"

namespace ohm
{
  class OccupancyMap;
}

namespace ohmgen
{
  /// Fill @p map with empty space over the specified rectangular region.
  ///
  /// Each voxel has one miss integrated.
  ///
  /// @param map The map to fill.
  /// @param x1 The lower extents X coordinate in @em voxels, included.
  /// @param y1 The lower extents Y coordinate in @em voxels, included.
  /// @param z1 The lower extents Z coordinate in @em voxels, included.
  /// @param x2 The upper extents X coordinate in @em voxels, excluded.
  /// @param y2 The upper extents Y coordinate in @em voxels, excluded.
  /// @param z2 The upper extents Z coordinate in @em voxels, excluded.
  /// @param expectEmptyMap Do we expect the map to begin empty?
  void fillMapWithEmptySpace(ohm::OccupancyMap &map,
                             int x1, int y1, int z1, int x2, int y2, int z2,
                             bool expectEmptyMap = true);

  /// Fill @p map as if we had a sensor in the middle of a cubic room.
  ///
  /// For each voxel on the walls, we simulate integrating a ray sample from the origin to the wall voxel.
  /// We integrate a hit in the wall sample and misses in the voxels connecting the wall voxel to the origin.
  ///
  /// @param map The map to fill.
  /// @param boundaryRange Distance to the wall from the origin.
  /// @param voxelStep Specifies the voxel step to make along the walls. This allows wholes
  ///   to be created in the wall sampling.
  void cubicRoom(ohm::OccupancyMap &map, float boundaryRange, int voxelStep = 1);
}

#endif // OHMGEN_H_
