// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_PLANEFILLWALKER_H
#define OHM_PLANEFILLWALKER_H

#include "OhmConfig.h"

#include "ohm/Key.h"
#include "ohm/UpAxis.h"

#include <glm/vec3.hpp>

#include <cinttypes>
#include <queue>

namespace ohm
{
class Key;
class OccupancyMap;

/// Helper class for walking a plane in the heightmap given any up axis using a flood fill pattern.
/// Manages walking the correct axis based on the @c UpAxis.
///
/// Usage:
/// - Initialise
/// - call @c begin().
/// - do work
/// - call @c walkNext() and loop if true.
///
/// The fill walker normally adds neighbours as it walks each voxel. However, the neighbours setting
/// @p auto_add_neighbours to false on construction allows manual neighbour management. This Allows the height of
/// the plane to be adjusted during walking as in heightmap terrain following.
///
/// Nodes may be revisited according to the @c Revist behaviour passed to @c addNeighbours() . With
/// @c auto_add_neighbours this is always @c Revisit::kNone.
class ohm_API PlaneFillWalker
{
public:
  /// Revisit behaviour for @c addNeighbours()
  enum class Revisit : int
  {
    kNone = 0,  ///< No revisiting allowed.
    kAll,       ///< Always revsit, adding neighbours again. Only the most recently added key is ever returned though.
    kLower,     ///< Revisit if the key appears lower along the up axis.
    kHigher     ///< Revisit if the key is higher along the up axis.
  };

  const OccupancyMap &map;     ///< Map to walk voxels in.
  const Key &min_ext_key;      ///< The starting voxel key (inclusive).
  const Key &max_ext_key;      ///< The last voxel key (inclusive).
  const glm::ivec3 key_range;  ///< The range between @c min_ext_key and @c max_ext_key .
  /// Mapping of the indices to walk, supporting various heightmap up axes. Element 2 is always the up axis, where
  /// elements 0 and 1 are the horizontal axes.
  int axis_indices[3] = { 0, 0, 0 };
  const bool auto_add_neighbours = false;  ///< Automatically voxels heighbours to touched voxels?
  std::queue<Key> open_list;               ///< Remaining voxels to (re)process.
  /// Identifies which bounded keys have been visited. A negative value indices no visit, a zero or positive value
  /// indicates the offset from the min_ext_key at which the node was visited
  std::vector<int> visit_list;

  /// Constructor.
  /// @param map The map to walk voxels in.
  /// @param min_ext_key The starting voxel key (inclusive).
  /// @param max_ext_key The last voxel key (inclusive).
  /// @param up_axis Specifies the up axis for the map.
  /// @param auto_add_neighbours True to automatically add horizontal neighbours to the open list when visiting
  /// voxels.
  PlaneFillWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis,
                  bool auto_add_neighbours = true);

  /// Initialse @p key To the first voxel to walk.
  /// @param[out] key Set to the first key to be walked.
  /// @return True if the key is valid, false if there is nothing to walk.
  bool begin(Key &key);

  /// Walk the next key in the sequence.
  /// @param[in,out] key Modifies to be the next key to be walked.
  /// @return True if the key is valid, false if walking is complete.
  bool walkNext(Key &key);

  /// Add neigbhours of @p key to the open list.
  /// @param key The key of interest.
  /// @param revisit_behaviour How to deal with voxels which have already been visited.
  void addNeighbours(const Key &key, Revisit revisit_behaviour = Revisit::kNone);

  /// Marks the given key as visited.
  /// @param key The key to visit.
  void touch(const Key &key);

private:
  unsigned touchIndex(const Key &key);
  int visitHeight(const Key &key) const;
};
}  // namespace ohm

#endif  // OHM_PLANEFILLWALKER_H
