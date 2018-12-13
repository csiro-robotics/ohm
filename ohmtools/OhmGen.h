// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMTOOLS_OHMGEN_H
#define OHMTOOLS_OHMGEN_H

#include "OhmToolsConfig.h"

#include <glm/fwd.hpp>

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
  /// @param expect_empty_map Do we expect the map to begin empty?
  void ohmtools_API fillMapWithEmptySpace(ohm::OccupancyMap &map, int x1, int y1, int z1, int x2, int y2, int z2,
                                          bool expect_empty_map = true);

  /// Fill @p map as if we had a sensor in the middle of a box.
  ///
  /// For each voxel on the walls, we simulate integrating a ray sample from the origin to the wall voxel.
  /// We integrate a hit in the wall sample and misses in the voxels connecting the wall voxel to the origin.
  ///
  /// @param map The map to fill.
  /// @param min_ext The minimum extents for the box. Defines the lower wall corner.
  /// @param max_ext The maximum extents for the box. Defines the lower wall corner.
  /// @param voxel_step Specifies the voxel step to make along the walls. This allows wholes
  ///   to be created in the wall sampling.
  void ohmtools_API boxRoom(ohm::OccupancyMap &map, const glm::dvec3 &min_ext, const glm::dvec3 &max_ext,
                            int voxel_step = 1);

  /// Generate a map containing a slope within the given extents. The slope is built in X/Y with Z adjusted to the
  /// desired slope. The Z value to @p max_ext is ignored, when the Z value of @p min_ext seeds the initial height.
  /// @param map The map to generate a slope in.
  /// @param angle_deg The angle of the slope in degrees.
  /// @param min_ext Defines the minimum of the slope extents.
  /// @param max_ext Defines the maximum of the slope extents.
  /// @param voxel_step Specifies the voxel step to make along the slope plane.
  void ohmtools_API slope(ohm::OccupancyMap &map, double angle_deg, const glm::dvec3 &min_ext,
                          const glm::dvec3 &max_ext, int voxel_step = 1);
}  // namespace ohmgen

#endif  // OHMTOOLS_OHMGEN_H
