// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_PLANEFILLWALKER_H
#define OHM_PLANEFILLWALKER_H

#include "OhmConfig.h"

#include "Key.h"
#include "PlaneWalkVisitMode.h"
#include "UpAxis.h"

#include <glm/vec3.hpp>

#include <array>
#include <cinttypes>
#include <deque>
#include <vector>

namespace ohm
{
class Key;
class OccupancyMap;

/// Helper class for walking a plane in the heightmap given any up axis using a flood fill pattern.
/// Manages walking the correct axis based on the @c UpAxis.
///
/// Usage:
/// - Initialise the walker and a key to start iteration from.
/// - call @c begin(key) with the initial key value. The @p key will be clamped to the range.
/// - Start loop:
///   - do work on key, refining it to be the actual key of interest.
///   - call @c visit(key) - where @c key value may be a refinement of the original key. This will push it's neighbours.
///   - call @c walkNext(key) and loop if true.
class ohm_API PlaneFillWalker
{
public:
  /// Revisit behaviour for @c visit()
  enum class Revisit : int
  {
    kNone = 0,  ///< No revisiting allowed.
    kLower,     ///< Revisit if the key appears lower along the up axis.
    kHigher     ///< Revisit if the key is higher along the up axis.
  };

  /// Entry used to track node visiting in the @c visit_list. This tracks the height at which the cell has been visited
  /// with -1 indicating an unvisited cell. The height is calculated by @c keyHeight() and marks the vertical voxel
  /// offset from the @c minKey().
  struct Visit
  {
    /// Height at which the cell has been visited. The height is an offset from the @c minKey() height. -1 denotes an
    /// unvisited cell.
    int height = -1;

    /// Mark the voxel as being visited at the specified height.
    /// @param visit_height The the visiting height.
    inline void visit(int visit_height) { height = visit_height; }
    /// Query if the voxel has been visited.
    /// @return True if visited.
    inline bool visited() const { return height >= 0; }
    /// Query whether we can revisit a voxel at the given height allowing revisiting if the new height is greater than
    /// any previous visit height.
    /// @param visit_height The new height we wish to visit at.
    /// @return True if the voxel should be revisited.
    inline bool revisitHigher(int visit_height) const { return !visited() || visit_height > height; }
    /// Query whether we can revisit a voxel at the given height allowing revisiting if the new height is less than
    /// any previous visit height.
    /// @param visit_height The new height we wish to visit at.
    /// @return True if the voxel should be revisited.
    inline bool revisitLower(int visit_height) const { return !visited() || visit_height < height; }
    /// For revisiting API compatibility - semantically equivalent to @c visited().
    /// @return True if the voxel can be (re)visited.
    inline bool revisitNone(int /*visit_height*/) const { return !visited(); }
  };

  const OccupancyMap &map;     ///< Map to walk voxels in.
  const Key min_ext_key;       ///< The starting voxel key (inclusive).
  const Key max_ext_key;       ///< The last voxel key (inclusive).
  const glm::ivec3 key_range;  ///< The range between @c min_ext_key and @c max_ext_key .
  /// Mapping of the indices to walk, supporting various heightmap up axes. Element 2 is always the up axis, where
  /// elements 0 and 1 are the horizontal axes.
  const std::array<int, 3> axis_indices;
  /// Defines how to treat attempts to revisit a node.
  Revisit revisit_behaviour = Revisit::kLower;

  /// Constructor.
  /// @param map The map to walk voxels in.
  /// @param min_ext_key The starting voxel key (inclusive).
  /// @param max_ext_key The last voxel key (inclusive).
  /// @param up_axis Specifies the up axis for the map.
  /// @param revisit_behaviour Controls what to do when visiting voxels in a column which has already been visited.
  PlaneFillWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis,
                  Revisit revisit_behaviour = Revisit::kLower);

  /// Query the minimum key value to walk from.
  /// @return The mimimum key value.
  inline const Key &minKey() const { return min_ext_key; }
  /// Query the maximum key value to walk to.
  /// @return The maximum key value.
  inline const Key &maxKey() const { return max_ext_key; }

  /// Initialise @p key To the first voxel to walk.
  /// @param[out] key Set to the first key to be walked.
  /// @return True if the key is valid, false if there is nothing to walk.
  bool begin(Key &key);

  /// Walk the next key in the sequence.
  /// @param[in,out] key Modifies to be the next key to be walked.
  /// @return True if the key is valid, false if walking is complete.
  bool walkNext(Key &key);

  /// Call this function when visiting a voxel at the given @p key. The keys neighbouring @p key (on the walk plane)
  /// are added to the open list, provided they are not already on the open list. The added neighbouring keys are
  /// filled in @p neighbours with the number of neighbours added given in the return value.
  /// @param key The key being visited. Must fall within the @c range.minKey() and @c range.maxKey() bounds.
  /// @param neighbours Populated with any neighbours of @p key added to the open list.
  /// @param mode Affects how to expand neighbours when visiting the voxel at @p key.
  /// @return The number of neighbours added.
  size_t visit(const Key &key, PlaneWalkVisitMode mode, std::array<Key, 8> &neighbours);
  /// @overload
  inline size_t visit(const Key &key, PlaneWalkVisitMode mode)
  {
    std::array<Key, 8> discard_neighbours;
    return visit(key, mode, discard_neighbours);
  }

  /// Marks the given key as visited.
  /// @param key The key to visit.
  void touch(const Key &key);

private:
  unsigned gridIndex(const Key &key);
  /// Query the Visit entry height for @p key.
  int keyHeight(const Key &key) const;

  std::deque<Key> open_list_;  ///< Remaining voxels to (re)process.
  /// A grid tracking the heights at which each planar item has been visited. The grid is sized for the 2D region from
  /// @c min_ext_key to @c max_ext_key. A height value < 0 indicates not having been visited.
  std::vector<Visit> visit_grid_;
};
}  // namespace ohm

#endif  // OHM_PLANEFILLWALKER_H
