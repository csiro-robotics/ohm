// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef OHMTESTCOMMON_WALKSEGMENTKEYSLEGACY_H
#define OHMTESTCOMMON_WALKSEGMENTKEYSLEGACY_H

#include <ohm/KeyList.h>

#include <glm/vec3.hpp>

#include <functional>

namespace ohm
{
class Key;
class OccupancyMap;

/// Function signature for the visit function called from @c walkSegmentKeys().
/// @param key The key of the current voxel being visited.
/// @param enter_range Range at which the voxel is entered. detail required
/// @param exit_range Range at which the voxel is entered. detail required
/// @return True to keep walking the voxels along the ray, false to abort walking the ray.
using WalkSegmentFunc = std::function<bool(const Key &, double, double)>;

/// The legacy implementation of line walking in ohm for use in tests which compare with legacy generated data.
///
/// The @p walk_func is invoked for each voxel in the line segment connecting @p start_point to @p end_point .
///
/// Based on J. Amanatides and A. Woo, "A fast voxel traversal algorithm for raytracing," 1987.
///
/// @param walk_func The callable object to invoke for each traversed voxel key.
/// @param start_point The start of the line in 3D space.
/// @param end_point The end of the line in 3D space.
/// @param include_end_point Should be @c true if @p walkFunc should be called for the voxel containing
///     @c endPoint, when it does not lie in the same voxel as @p startPoint.
/// @param map The occupancy map to walk in.
/// @param length_epsilon Small length at which a segment is considered degenerate.
/// @return The number of voxels traversed. This includes @p endPoint when @p includeEndPoint is true.
size_t walkSegmentKeysLegacy(WalkSegmentFunc walk_func, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                             bool include_end_point, const ohm::OccupancyMap &map,
                             double length_epsilon = 1e-6);  // NOLINT(readability-magic-numbers)

/// Function which calculates segment keys using legacy line walking.
inline size_t calculateSegmentKeysLegacy(KeyList &keys, const OccupancyMap &map, const glm::dvec3 &start_point,
                                         const glm::dvec3 &end_point, bool include_end_point)
{
  keys.clear();
  return ohm::walkSegmentKeysLegacy(
    [&keys](const Key &key, double, double) {
      keys.add(key);
      return true;
    },
    start_point, end_point, include_end_point, map);
}

}  // namespace ohm

#endif  // OHMTESTCOMMON_WALKSEGMENTKEYSLEGACY_H
