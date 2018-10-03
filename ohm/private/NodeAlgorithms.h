// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_NODEALGORITHMS_H
#define OHM_NODEALGORITHMS_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

namespace ohm
{
  class OccupancyKey;
  class OccupancyMap;

  /// A utility function for calculating the @c voxelSearchHalfExtents parameter for @c calcualteNearestNeighbour.
  /// @param map The map being searched.
  /// @param search_radius The search radius of interest.
  /// @return The calculated @c voxelSearchHalfExtents argument.
  glm::ivec3 calculateVoxelSearchHalfExtents(const OccupancyMap &map, float search_radius);

  /// Search for the nearest occupied neighbour of @p node and return the range.
  /// @param node_key The node we are operating on.
  /// @param map The map to which @p node belongs.
  /// @param voxel_search_half_extents Defines how far along each axis to search neighbours for in both + and - directions.
  /// @param unknown_as_occupied Treat unknown/unexplored nodes as occupied?
  /// @param ignore_self Only consider neighbours. Ignore the voxel itself if it is occupied.
  /// @param search_range Radius to search and report.
  /// @param axis_scaling Scaling applied along each axis to distort the search space.
  /// @param report_unscaled_distance Set to true in order to report the result without applying @c axisScaling.
  ///     The same voxel is selected regardless of this value.
  /// @return -1 if there are no occupied nodes within the @p voxelSearchHalfExtents range, or the range of
  ///   the nearest obstacle. Zero when @p node itself is occupied.
  float calculateNearestNeighbour(const OccupancyKey &node_key, const OccupancyMap &map,
                                  const glm::ivec3 &voxel_search_half_extents,
                                  bool unknown_as_occupied, bool ignore_self, float search_range = 0,
                                  const glm::vec3 &axis_scaling = glm::vec3(1.0f, 1.0f, 1.0f),
                                  bool report_unscaled_distance = false);
}

#endif // OHM_NODEALGORITHMS_H
