// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "CalculateSegmentKeys.h"

#include "Key.h"
#include "KeyList.h"
#include "OccupancyMap.h"

#include <ohmutil/LineWalk.h>

namespace
{
  struct KeyAdaptor
  {
    const ohm::OccupancyMap &map;

    inline KeyAdaptor(const ohm::OccupancyMap &map)
      : map(map)
    {}

    inline ohm::Key voxelKey(const glm::dvec3 &pt) const { return map.voxelKey(pt); }
    inline bool isNull(const ohm::Key &key) const { return key.isNull(); }
    inline glm::dvec3 voxelCentre(const ohm::Key &key) const { return map.voxelCentreLocal(key); }
    inline void stepKey(ohm::Key &key, int axis, int dir) const  // NOLINT(google-runtime-references)
    {
      map.stepKey(key, axis, dir);
    }
    inline double voxelResolution(int /*axis*/) const { return map.resolution(); }
  };
}  // namespace


namespace ohm
{
  size_t calculateSegmentKeys(KeyList &keys, const OccupancyMap &map, const glm::dvec3 &start_point,
                              const glm::dvec3 &end_point, bool include_end_point)
  {
    const glm::dvec3 start_point_local = glm::dvec3(start_point - map.origin());
    const glm::dvec3 end_point_local = glm::dvec3(end_point - map.origin());

    keys.clear();
    return ohm::walkSegmentKeys<Key>([&keys](const Key &key) { keys.add(key); }, start_point_local, end_point_local,
                                     include_end_point, KeyAdaptor(map));
  }
}  // namespace ohm
