// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_PLANEFILLLAYEREDWALKER_H
#define OHM_PLANEFILLLAYEREDWALKER_H

#include "OhmConfig.h"

#include "Key.h"
#include "KeyRange.h"
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

/// Helper class for walking heightmap generation in a floodfill pattern with the intention of generating a multi-layer
/// heightmap.
///
/// Manages walking the correct axis based on the @c UpAxis.
///
/// Usage:
/// - Initialise the walker and a key value to start iteration from.
/// - call @c begin(key) with the initial key value. The @p key will be clamped to the range.
/// - Start loop:
///   - do work on key, refining it to be the actual key of interest.
///   - call @c visit(key) - where @c key value may be a refinement of the original key. This will push it's neighbours.
///   - call @c walkNext(key) and loop if true.
class ohm_API PlaneFillLayeredWalker
{
public:
  /// Entry used to track node visiting in the @c opended_list_. The pair loosly track a visiting height range. The
  /// first item is the minimum height at which a voxel has been visited, while the second is the maximum height at
  /// which it has been visited. Matching entries indicate a single visit, unless a negative value is present. A
  /// negative value indices no visit, a zero or positive value indicates the offset from the @c range.minKey() at which
  /// the node was visited.
  struct Opened
  {
    using List = std::vector<Opened>;

    /// Height at which the cell has been visited.
    int height;
    /// A psuedo linked next next pointer - represents the next index into the visit list using 1-based indexing. This
    /// makes zero a null value.
    unsigned next;
  };

  const OccupancyMap &map;     ///< Map to walk voxels in.
  const KeyRange range;        ///< Specifies the key extents to visit. The up axis is used to limit visiting heights.
  const glm::ivec3 key_range;  ///< The key range covered by @c range.
  /// Mapping of the indices to walk, supporting various heightmap up axes. Element 2 is always the up axis, where
  /// elements 0 and 1 are the horizontal axes.
  const std::array<int, 3> axis_indices;
  /// Sign of the up axis [1, -1].
  const int up_sign;

  /// Constructor.
  /// @param map The map to walk voxels in.
  /// @param min_ext_key The minimal extents to visit.
  /// @param max_ext_key The maximal extents to visit.
  /// @param up_axis Defines the up axis for the plane being visited.
  PlaneFillLayeredWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis);

  inline const Key &minKey() const { return range.minKey(); }
  inline const Key &maxKey() const { return range.maxKey(); }

  /// Begin walking keys starting from the given @p key.
  ///
  /// After calling @c begin(), it is expected that the key value will be processed and possibly modified along the
  /// up axis. For example in heightmap generation, the key may initially define the seed key for finding a valid
  /// surface. The surface itself may appear at a different height. It is the modified height key which should be passed
  /// to @c visit() before calling @c walkNext(). In the case where no surface is found, the unmodified key may be
  /// given to @c visit().
  ///
  /// @param[in,out] key The seed key for walking. The key will be clamped to @c range.
  /// @return True if the range defines a valid region to walk.
  bool begin(Key &key);

  /// Walk the next key in the sequence.
  /// @param[in,out] key Modifies to be the next key to be walked.
  /// @return True if the key is valid, false if walking is complete.
  bool walkNext(Key &key);

  /// Call this function when visiting a voxel at the given @p key. The keys neighbouring @p key (on the walk plane)
  /// are added to the open list, provided they are not already on the open list. The added neighouring keys are
  /// filled in @p neighbours with the number of neighbours added given in the return value.
  ///
  /// Note: when @p mode is @c kAddUnvisitedNeighbours the neighbours in the layere *above* @c key are added. This
  /// encourages traversal up slopes connected to the ground especially when there is no clearance constraint.
  /// @p mode should be @c kAddUnvisitedColumnNeighbours when a valid ground candidate cound not be found in the column
  /// of @p key. This mode will add neighbours on the same level as @p key, but only if the column itself has not been
  /// visited. This encourages traversal of unobserved space.
  ///
  /// @param key The key being visited. Must fall within the @c range.minKey() and @c range.maxKey() bounds.
  /// @param neighbours Populted with any neighbours of @p key added to the open list.
  /// @return The number of neighbours added.
  size_t visit(const Key &key, PlaneWalkVisitMode mode, std::array<Key, 8> &neighbours);
  /// @overload
  inline size_t visit(const Key &key, PlaneWalkVisitMode mode)
  {
    std::array<Key, 8> discard_neighbours;
    return visit(key, mode, discard_neighbours);
  }

private:
  /// Mark the item at @p grid_index in the @c opened_grid_ as being opened at the given @c visit_height.
  /// @param grid_index An index into @c opened_grid_
  /// @param visit_height The neight at which to visit.
  void open(int grid_index, int visit_height);
  bool hasOpened(int grid_index, int visit_height) const;
  inline bool hasOpened(int grid_index) const { return opened_grid_[grid_index] > 0; }

  /// Calculate the @c opened_grid_ index for the given @p key.
  unsigned gridIndexForKey(const Key &key);
  /// Query the Opened entry height for @p key.
  int keyHeight(const Key &key) const;

  std::deque<Key> open_list_;  ///< Remaining voxels to (re)process.
  /// Identifies which bounded keys have been opened.
  Opened::List opened_list_;
  /// A grid of 1-based indices into the @c opened_list_. The @c opened_grid_ is sized to match grid of keys defined by
  /// the 2D region from @c range.minKey() to @c range.maxKey(). The values are 1-based indices into the @c
  /// opened_list_ with zero marking a null value.
  ///
  /// A cell is opened if it has a non-zero value. The visit height can be found by following the index into the
  /// @c opened_list_. Multiple visits can be followed by chaining indices using @c Opened::next.
  std::vector<int> opened_grid_;
};
}  // namespace ohm

#endif  // OHM_PLANEFILLLAYEREDWALKER_H
