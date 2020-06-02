//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef RAYMAPPERINTERFACE_H
#define RAYMAPPERINTERFACE_H

#include "OhmConfig.h"

#include "Key.h"
#include "Voxel.h"

#include <glm/fwd.hpp>

namespace ohm
{
  /// The default map interface for a @c RayMapperInterface . This version is primarily designed to handle the
  /// @c OccupancyMap class.
  template <typename MAP>
  struct RayMapperInterface
  {
    /// Extract an @c OccupancyMap from the @c MAP type (mutable).
    /// @param map The map object to resolve into an @c OccupancyMap .
    /// @return The @c OccupancyMap associated with @p map .
    static OccupancyMap &occupancyMap(MAP &map) { return map; }
    /// Extract an @c OccupancyMap from the @c MAP type (const).
    /// @param map The map object to resolve into an @c OccupancyMap .
    /// @return The @c OccupancyMap associated with @p map .
    static const OccupancyMap &occupancyMap(const MAP &map) { return map; }
    /// Retrieve the active @c RayFilterFunction for the given @c map . May be empty.
    /// @param map The map object to operate on.
    /// @return The active @p RayFilterFunction , possibly empty.
    static RayFilterFunction rayFilter(const MAP &map) { return map.rayFilter(); }
    /// Query the voxel key within @p map for the given @p point .
    /// @param map The map object to operate on.
    /// @param point The point to fetch a key for.
    /// @return The @c Key value for @p point .
    static Key voxelKey(MAP &map, const glm::dvec3 &point) { return map.voxelKey(point); }
    /// Fetch a mutable @c Voxel for the given @p Key in the given @p map . Must be created if it does not exist.
    /// @param map The map object to operate on.
    /// @param key The target voxel key.
    /// @param cache Optional @p MapCache to assist in lookup.
    /// @return The @c Voxel for @p key which is created as needed.
    static Voxel voxel(MAP &map, const Key &key, MapCache *cache) { return map.voxel(key, true, cache); }

    /// Update the given @p voxel probability with a hit result.
    /// @param map The map object to operate on.
    /// @param voxel The voxel to update.
    /// @param start The start point of the ray being updated.
    /// @param end The end point of the ray being updated which lies with @p voxel .
    static void integrateHit(MAP &map, Voxel &voxel, const glm::dvec3 & /*start*/, const glm::dvec3 &end)
    {
      map.integrateHit(voxel, end);
    }

    /// Update the given @p voxel probability with a miss result.
    /// @param map The map object to operate on.
    /// @param voxel The voxel to update. This voxel lies along the line segment `[start, end]`.
    /// @param start The start point of the ray being updated.
    /// @param end The end point of the ray being updated.
    static void integrateMiss(MAP &map, Voxel &voxel, const glm::dvec3 & /*start*/, const glm::dvec3 & /*end*/)
    {
      map.integrateMiss(voxel);
    }
  };
}  // namespace ohm

#endif  // RAYMAPPERINTERFACE_H
