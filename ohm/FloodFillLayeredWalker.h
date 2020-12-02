// Copyright (c) 2020
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

#include <array>
#include <cinttypes>
#include <queue>

namespace ohm
{
class Key;
class OccupancyMap;

/// Helper class for walking heightmap generation in a floodfill pattern with the intention of generating a multi-layer
/// heightmap.
///
/// Manages walking the correct axis based on the @c UpAxis.
///
/// Usage:
/// - Initialise
/// - call @c begin(key) to set the initial key
/// - [[ @c visit(key) ]] - maybe necessary for the first key to get the candiate ground height correct.
/// - Start work on key:
///   - @c addNeighbours(key)
///   - do work
///   - call @c walkNext(key) and loop if true.
///
/// The fill walker normally adds neighbours as it walks each voxel. However, the neighbours setting
/// @p auto_add_neighbours to false on construction allows manual neighbour management. This Allows the height of
/// the plane to be adjusted during walking as in heightmap terrain following.
///
/// Nodes may be revisited according to the @c Revist behaviour passed to @c addNeighbours() . With
/// @c auto_add_neighbours this is always @c Revisit::kNone.
class ohm_API FloodFillLayerWalker
{
public:
  /// Entry used to track node visiting in the @c visit_list. The pair loosly track a visiting height range. The first
  /// item is the minimum height at which a voxel has been visited, while the second is the maximum height at which it
  /// has been visited. Matching entries indicate a single visit, unless a negative value is present. A negative value
  /// indices no visit, a zero or positive value indicates the offset from the min_ext_key at which the node was
  /// visited.
  struct Visit
  {
    using List = std::vector<Visit>;

    /// Height at which the cell has been visited.
    int height;
    /// A psuedo linked next next pointer - represents the next index into the visit list using 1-based indexing. This
    /// makes zero a null value.
    unsigned next;
  };

  const OccupancyMap &map;     ///< Map to walk voxels in.
  const Key &min_ext_key;      ///< The starting voxel key (inclusive).
  const Key &max_ext_key;      ///< The last voxel key (inclusive).
  const glm::ivec3 key_range;  ///< The range between @c min_ext_key and @c max_ext_key .
  /// Mapping of the indices to walk, supporting various heightmap up axes. Element 2 is always the up axis, where
  /// elements 0 and 1 are the horizontal axes.
  std::array<int, 3> axis_indices = { 0, 0, 0 };
  const bool auto_add_neighbours = false;  ///< Automatically voxels heighbours to touched voxels?
  std::queue<Key> open_list;               ///< Remaining voxels to (re)process.
  /// Identifies which bounded keys have been visited.
  Visit::List visit_list;
  /// A grid of 1-based indices into the @c visit_list. The @c visit_grid is sized to match grid of keys defined by the
  /// 2D region from @c min_ext_key to @c max_ext_key. The values are 1-based indices into the @c visit_list with zero
  /// marking a null value.
  ///
  /// A cell is visited if it has a non-zero value. The visit height can be found by following the index into the
  /// @c visit_list. Multiple visits can be followed by chaining indices using @c Visit::next.
  std::vector<int> visit_grid;

  /// Constructor.
  /// @param map The map to walk voxels in.
  /// @param min_ext_key The starting voxel key (inclusive).
  /// @param max_ext_key The last voxel key (inclusive).
  /// @param up_axis Specifies the up axis for the map.
  /// @param auto_add_neighbours True to automatically add horizontal neighbours to the open list when visiting
  /// voxels.
  FloodFillLayerWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis,
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
  size_t addNeighbours(const Key &key, std::array<Key, 8> &added_neighbours,
                       Revisit revisit_behaviour = Revisit::kNone);
  size_t addNeighbours(const Key &key, Revisit revisit_behaviour = Revisit::kNone)
  {
    std::array<Key, 8> ignore;
    return addNeighbours(key, ignore, revisit_behaviour);
  }

private:
  void visit(int grid_index, int visit_height);
  inline bool visited(int grid_index) const { visit_grid[grid_index] > 0; }
  bool revisitAll(int grid_index, int visit_height) const;
  bool revisitHigher(int grid_index, int visit_height) const;
  bool revisitLower(int grid_index, int visit_height) const;
  inline bool revisitNone(int grid_index) const { return !visited(grid_index); }

  unsigned touchIndex(const Key &key);
  /// Query the Visit entry height for @p key.
  int visitHeight(const Key &key) const;
};
}  // namespace ohm

#endif  // OHM_PLANEFILLWALKER_H
