//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#ifndef NDTRAYMAPPERINTERFACE_H
#define NDTRAYMAPPERINTERFACE_H

#include "OhmConfig.h"

#include "NdtMap.h"
#include "RayMapperInterface.h"

namespace ohm
{
  /// @c RayMapperInterface for the @c NdtMap .
  template <>
  struct RayMapperInterface<NdtMap>
  {
    /// @override
    static OccupancyMap &occupancyMap(NdtMap &map) { return map.map(); }
    /// @override
    static const OccupancyMap &occupancyMap(const NdtMap &map) { return map.map(); }
    /// @override
    static RayFilterFunction rayFilter(const NdtMap &map) { return map.map().rayFilter(); }
    /// @override
    static Key voxelKey(NdtMap &map, const glm::dvec3 &sample) { return map.map().voxelKey(sample); }
    /// @override
    static Voxel voxel(NdtMap &map, const Key &key, MapCache *cache)
    {
      return map.map().voxel(key, true, cache);
    }

    /// @override
    static void integrateHit(NdtMap &map, Voxel &voxel, const glm::dvec3 &start, const glm::dvec3 &end)
    {
      map.integrateHit(voxel, start, end);
    }

    /// @override
    static void integrateMiss(NdtMap &map, Voxel &voxel, const glm::dvec3 &start, const glm::dvec3 &end)
    {
      map.integrateMiss(voxel, start, end);
    }
  };
}  // namespace ohm

#endif  // NDTRAYMAPPERINTERFACE_H
