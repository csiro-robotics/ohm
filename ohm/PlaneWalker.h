// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_PLANEWALKER_H
#define OHM_PLANEWALKER_H

#include "OhmConfig.h"

#include "Key.h"
#include "PlaneWalkVisitMode.h"
#include "UpAxis.h"

#include <array>

namespace ohm
{
class OccupancyMap;

/// Helper class for walking a plane in the heightmap given any up axis.
/// Manages walking the correct axis based on the @c UpAxis.
///
/// Usage:
/// - Initialise
/// - call @c begin().
/// - do work
/// - call @c walkNext() and loop if true.
class ohm_API PlaneWalker
{
public:
  const OccupancyMap &map;  ///< Map to walk voxels in.
  const Key min_ext_key;    ///< The starting voxel key (inclusive).
  const Key max_ext_key;    ///< The last voxel key (inclusive).
  const Key plane_key;      ///< The key which was used to seed the plane, setting the height.
  /// Mapping of the indices to walk, supporting various heightmap up axes. Element 2 is always the up axis, where
  /// elements 0 and 1 are the horizontal axes.
  const std::array<int, 3> axis_indices;

  /// Constructor.
  /// @param map The map to walk voxels in.
  /// @param min_ext_key The starting voxel key (inclusive).
  /// @param max_ext_key The last voxel key (inclusive).
  /// @param up_axis Specifies the up axis for the map.
  /// @param plane_key_ptr Optional key specification for the @c plane_key .
  PlaneWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis,
              const Key *plane_key_ptr = nullptr);

  inline const Key &minKey() const { return min_ext_key; }
  inline const Key &maxKey() const { return max_ext_key; }

  /// Initialse @p key To the first voxel to walk.
  /// @param[out] key Set to the first key to be walked.
  /// @return True if the key is valid, false if there is nothing to walk.
  bool begin(Key &key) const;

  /// Walk the next key in the sequence.
  /// @param[in,out] key Modifies to be the next key to be walked.
  /// @param[out] is_first_column_visit Set to true if this is the first visit to the column of @p key.
  /// @return True if the key is valid, false if walking is complete.
  bool walkNext(Key &key, bool &is_first_column_visit) const;

  /// For API compatibility. Does nothing.
  /// @return 0
  inline size_t visit(const Key & /*key*/, PlaneWalkVisitMode /*mode*/) { return 0u; }  // NOLINT
  /// For API compatibility. Does nothing.
  /// @return 0
  inline size_t visit(const Key & /*key*/, PlaneWalkVisitMode /*mode*/, std::array<Key, 8> & /*neighbours*/)  // NOLINT
  {
    return 0u;
  }
};
}  // namespace ohm

#endif  // OHM_PLANEWALKER_H
