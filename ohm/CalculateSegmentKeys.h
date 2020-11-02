// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CALCULATESEGMENTKEYS_H
#define CALCULATESEGMENTKEYS_H

#include "OhmConfig.h"

#include <cstddef>

#include "Key.h"
#include "OccupancyMap.h"

#include <glm/fwd.hpp>

namespace ohm
{
class Key;
class KeyList;
class OccupancyMap;

/// A utility key adaptor around an @c OccupancyMap for use with @c walkSegmentKeys() .
struct ohm_API WalkKeyAdaptor
{
  /// Map reference.
  const ohm::OccupancyMap &map;

  /// Create an adaptor for @p map .
  /// @param map The map to adapt
  inline WalkKeyAdaptor(const ohm::OccupancyMap &map)
    : map(map)
  {}

  /// Resolve a point @p pt to a voxel key.
  /// @param pt The point of interest.
  /// @return The key for @p pt
  inline ohm::Key voxelKey(const glm::dvec3 &pt) const { return map.voxelKey(pt); }
  /// Check if @p key is null.
  /// @param key The key to test
  /// @return True if @p key is a null entry
  inline bool isNull(const ohm::Key &key) const { return key.isNull(); }
  /// Resolve a @p key to the corresponding voxel centre coordinate.
  /// @param key The key of interest.
  /// @return The coordinate at the centre of the voxel which @p key reference.
  inline glm::dvec3 voxelCentre(const ohm::Key &key) const { return map.voxelCentreLocal(key); }
  /// Adjust the value of @p key by stepping it along @p axis
  /// @param key The key to modify.
  /// @param axis The axis to modifier where 0, 1, 2 map to X, Y, Z respectively.
  /// @param dir The direction to step: must be 1 or -1
  inline void stepKey(ohm::Key &key, int axis, int dir) const { map.stepKey(key, axis, dir); }
  /// Query the voxel resolution.
  /// @return The voxel resolution.
  inline double voxelResolution(int /*axis*/) const { return map.resolution(); }
};

/// This populates a @c KeyList with the voxel @c Key values intersected by a line segment.
///
/// This utility function leverages @c walkSegmentKeys() in order to calculate the set of voxel @c Key values
/// intersected by the line segment @p start_point to @p end_point .
///
/// @param[out] keys Populates with the keys intersected by the specified line segment.
/// @param map The occupancy map to calculate key segments for.
/// @param start_point The coordinate of the line segment start or sensor position. Global map frame.
/// @param end_point The coordinate of the line segment end or sample position. Global map frame.
/// @param include_end_point True to include the voxel containing the @p end_point , false to ommit this voxel,
///     even when the @p start_point is in the same voxel (this case would generate an empty list).
size_t ohm_API calculateSegmentKeys(KeyList &keys, const OccupancyMap &map, const glm::dvec3 &start_point,
                                    const glm::dvec3 &end_point, bool include_end_point);
}  // namespace ohm

#endif  // CALCULATESEGMENTKEYS_H
