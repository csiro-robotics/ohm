//
// Author: Kazys Stepanas
// Copyright (c) CSIRO 2020
//
#include "RayMapperBase.h"

#include "KeyList.h"
#include "OccupancyMap.h"

#include <ohmutil/LineWalk.h>

using namespace ohm;

namespace
{
  struct KeyAdaptor
  {
    const OccupancyMap &map;

    inline KeyAdaptor(const OccupancyMap &map)
      : map(map)
    {}

    inline Key voxelKey(const glm::dvec3 &pt) const { return map.voxelKey(pt); }
    inline bool isNull(const Key &key) const { return key.isNull(); }
    inline glm::dvec3 voxelCentre(const Key &key) const { return map.voxelCentreLocal(key); }
    inline void stepKey(Key &key, int axis, int dir) const  // NOLINT(google-runtime-references)
    {
      map.stepKey(key, axis, dir);
    }
    inline double voxelResolution(int /*axis*/) const { return map.resolution(); }
  };
}  // namespace

RayMapperBase::RayMapperBase() = default;

RayMapperBase::~RayMapperBase() = default;

size_t RayMapperBase::calculateSegmentKeys(const OccupancyMap &map, KeyList &keys, const glm::dvec3 &start_point,
                                           const glm::dvec3 &end_point, bool include_end_point)
{
  const glm::dvec3 start_point_local = glm::dvec3(start_point - map.origin());
  const glm::dvec3 end_point_local = glm::dvec3(end_point - map.origin());

  keys.clear();
  return ohm::walkSegmentKeys<Key>([&keys](const Key &key) { keys.add(key); }, start_point_local, end_point_local,
                                   include_end_point, KeyAdaptor(map));
}
