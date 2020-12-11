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
///
/// The fill walker normally adds neighbours as it walks each voxel. However, the neighbours setting
/// @p auto_add_neighbours to false on construction allows manual neighbour management. This Allows the height of
/// the plane to be adjusted during walking as in heightmap terrain following.
///
/// Nodes may be revisited according to the @c Revist behaviour passed to @c visit() . With
/// @c auto_add_neighbours this is always @c Revisit::kNone.
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

  /// Entry used to track node visiting in the @c visit_list. The pair loosly track a visiting height range. The first
  /// item is the minimum height at which a voxel has been visited, while the second is the maximum height at which it
  /// has been visited. Matching entries indicate a single visit, unless a negative value is present. A negative value
  /// indices no visit, a zero or positive value indicates the offset from the min_ext_key at which the node was
  /// visited.
  struct Visit
  {
    /// Height at which the cell has been visited.
    int height = -1;

    inline void visit(int visit_height) { height = visit_height; }
    inline bool visited() const { return height >= 0; }
    inline bool revisitHigher(int visit_height) const { return !visited() || visit_height > height; }
    inline bool revisitLower(int visit_height) const { return !visited() || visit_height < height; }
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
  /// @param auto_add_neighbours True to automatically add horizontal neighbours to the open list when visiting
  /// voxels.
  PlaneFillWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis,
                  Revisit revisit_behaviour = Revisit::kLower);

  inline const Key &minKey() const { return min_ext_key; }
  inline const Key &maxKey() const { return max_ext_key; }

  /// Initialse @p key To the first voxel to walk.
  /// @param[out] key Set to the first key to be walked.
  /// @return True if the key is valid, false if there is nothing to walk.
  bool begin(Key &key);

  /// Walk the next key in the sequence.
  /// @param[in,out] key Modifies to be the next key to be walked.
  /// @return True if the key is valid, false if walking is complete.
  bool walkNext(Key &key);

  /// Call this function when visiting a voxel at the given @p key. The keys neighbouring @p key (on the walk plane)
  /// are added to the open list, provided they are not already on the open list. The added neighouring keys are
  /// filled in @p neighbours with the number of neighbours added given in the return value.
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
