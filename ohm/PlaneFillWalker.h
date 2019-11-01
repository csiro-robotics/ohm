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
  /// @c auto_add_neighbours this is always @c Revisit::None.
  class ohm_API PlaneFillWalker
  {
  public:
    /// Revisit behaviour for @c addNeighbours()
    enum class Revisit : int
    {
      None = 0, ///< No revisiting allowed.
      All,      ///< Always revsit, adding neighbours again. Only the most recently added key is ever returned though.
      Lower,    ///< Revisit if the key appears lower along the up axis.
      Higher    ///< Revisit if the key is higher along the up axis.
    };

    const OccupancyMap &map;
    const Key &min_ext_key, &max_ext_key;
    const glm::ivec3 key_range;
    int axis_indices[3] = { 0, 0, 0 };
    const bool auto_add_neighbours = false;
    std::queue<Key> open_list;
    /// Identifies which bounded keys have been visited. A negative value indices no visit, a zero or positive value
    /// indicates the offset from the min_ext_key at which the node was visited
    std::vector<int> visit_list;

    PlaneFillWalker(const OccupancyMap &map, const Key &min_ext_key, const Key &max_ext_key, UpAxis up_axis,
                    bool auto_add_neighbours = true);

    bool begin(Key &key);  // NOLINT(google-runtime-references)

    bool walkNext(Key &key);  // NOLINT(google-runtime-references)

    void addNeighbours(const Key &key, Revisit revisit_behaviour = Revisit::None);
    void touch(const Key &key);

  private:
    unsigned touchIndex(const Key &key);
    int visitHeight(const Key &key) const;
  };
}  // namespace ohm

#endif  // OHM_PLANEFILLWALKER_H
