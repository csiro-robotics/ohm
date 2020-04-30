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
  template <typename MAP>
  struct RayMapperInterface
  {
    static OccupancyMap &occupancyMap(MAP &map) { return map; }
    static const OccupancyMap &occupancyMap(const MAP &map) { return map; }
    static RayFilterFunction rayFilter(const MAP &map) { return map.rayFilter(); }
    static Key voxelKey(MAP &map, const glm::dvec3 &sample) { return map.voxelKey(sample); }
    static Voxel voxel(MAP &map, const Key &key, MapCache *cache) { return map.voxel(key, true, cache); }

    static void integrateHit(MAP &map, Voxel &voxel, const glm::dvec3 & /*start*/, const glm::dvec3 &end)
    {
      map.integrateHit(voxel, end);
    }

    static void integrateMiss(MAP &map, Voxel &voxel, const glm::dvec3 & /*start*/, const glm::dvec3 & /*end*/)
    {
      map.integrateMiss(voxel);
    }
  };
}  // namespace ohm

#endif  // RAYMAPPERINTERFACE_H
