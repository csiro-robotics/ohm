// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYNODEALGORITHMS_H_
#define OCCUPANCYNODEALGORITHMS_H_

#include "ohmconfig.h"

#include <glm/glm.hpp>

namespace ohm
{
  class OccupancyKey;
  class OccupancyMap;

  /// A utility function for calculating the @c voxelSearchHalfExtents parameter for @c calcualteNearestNeighbour.
  /// @param map The map being searched.
  /// @param searchRadius The search radius of interest.
  /// @return The calculated @c voxelSearchHalfExtents argument.
  glm::ivec3 calculateVoxelSearchHalfExtents(const OccupancyMap &map, float searchRadius);

  /// Search for the nearest occupied neighbour of @p node and return the range.
  /// @param nodeKey The node we are operating on.
  /// @param map The map to which @p node belongs.
  /// @param searchHalfExtents Defines how far along each axis to search neighbours for in both + and - directions.
  /// @param unknownAsOccupied Treat unknown/unexplored nodes as occupied?
  /// @param ignoreSelf Only consider neighbours. Ignore the voxel itself if it is occupied.
  /// @param axisScaling Scaling applied along each axis to distort the search space.
  /// @param reportUnscaledDistance Set to true in order to report the result without applying @c axisScaling.
  ///     The same voxel is selected regardless of this value.
  /// @return -1 if there are no occupied nodes within the @p voxelSearchHalfExtents range, or the range of
  ///   the nearest obstacle. Zero when @p node itself is occupied.
  float calculateNearestNeighbour(const OccupancyKey &nodeKey, const OccupancyMap &map,
                                  const glm::ivec3 &voxelSearchHalfExtents,
                                  bool unknownAsOccupied, bool ignoreSelf, float searchRange = 0,
                                  const glm::vec3 &axisScaling = glm::vec3(1.0f, 1.0f, 1.0f),
                                  bool reportUnscaledDistance = false);
}

#endif // OCCUPANCYNODEALGORITHMS_H_
