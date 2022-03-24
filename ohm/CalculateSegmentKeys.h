// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CALCULATESEGMENTKEYS_H
#define CALCULATESEGMENTKEYS_H

#include "OhmConfig.h"

#include <cstddef>

#include <glm/fwd.hpp>

namespace ohm
{
class KeyList;
class OccupancyMap;

/// This populates a @c KeyList with the voxel @c Key values intersected by a line segment.
///
/// This utility function leverages @c walkSegmentKeys() in order to calculate the set of voxel @c Key values
/// intersected by the line segment @p start_point to @p end_point .
///
/// @param[out] keys Populates with the keys intersected by the specified line segment.
/// @param map The occupancy map to calculate key segments for.
/// @param start_point The coordinate of the line segment start or sensor position. Global map frame.
/// @param end_point The coordinate of the line segment end or sample position. Global map frame.
/// @param include_end_point True to include the voxel containing the @p end_point , false to omit this voxel,
///     even when the @p start_point is in the same voxel (this case would generate an empty list).
size_t ohm_API calculateSegmentKeys(KeyList &keys, const OccupancyMap &map, const glm::dvec3 &start_point,
                                    const glm::dvec3 &end_point, bool include_end_point = true);
}  // namespace ohm

#endif  // CALCULATESEGMENTKEYS_H
