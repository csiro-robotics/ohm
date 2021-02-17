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
class Key;
class OccupancyMap;

/// A utility function for calculating the @c voxel_search_half_extents parameter for @c calculateNearestNeighbour.
/// @param map The map being searched.
/// @param search_radius The search radius of interest.
/// @return The calculated @c voxel_search_half_extents argument.
glm::ivec3 ohm_API calculateVoxelSearchHalfExtents(const OccupancyMap &map, float search_radius);

/// Search for the nearest occupied neighbour of @p voxel and return the range.
/// @param voxel_key The voxel we are operating on.
/// @param map The map to which @p voxel belongs.
/// @param voxel_search_half_extents Defines how far along each axis to search neighbours for in both + and -
/// directions.
/// @param unobserved_as_occupied Treat unknown/unexplored voxels as occupied?
/// @param ignore_self Only consider neighbours. Ignore the voxel itself if it is occupied.
/// @param search_range Radius to search and report.
/// @param axis_scaling Scaling applied along each axis to distort the search space.
/// @param report_unscaled_distance Set to true in order to report the result without applying @c axis_scaling.
///     The same voxel is selected regardless of this value.
/// @return -1 if there are no occupied voxels within the @p voxel_search_half_extents range, or the range of
///   the nearest obstacle. Zero when @p voxel itself is occupied.
float ohm_API calculateNearestNeighbour(const Key &voxel_key, const OccupancyMap &map,
                                        const glm::ivec3 &voxel_search_half_extents, bool unobserved_as_occupied,
                                        bool ignore_self, float search_range = 0,
                                        const glm::vec3 &axis_scaling = glm::vec3(1.0f, 1.0f, 1.0f),
                                        bool report_unscaled_distance = false);
}  // namespace ohm

#endif  // OHM_NODEALGORITHMS_H
