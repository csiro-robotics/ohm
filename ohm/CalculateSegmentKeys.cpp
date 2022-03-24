// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "CalculateSegmentKeys.h"

#include "KeyList.h"

#include <ohmutil/LineWalk.h>

namespace ohm
{
size_t calculateSegmentKeys(KeyList &keys, const OccupancyMap &map, const glm::dvec3 &start_point,
                            const glm::dvec3 &end_point, bool include_end_point)
{
  keys.clear();
  return ohm::walkSegmentKeys<Key>(
    [&keys](const Key &key, double, double, const glm::ivec3 &) {
      keys.add(key);
      return true;
    },
    start_point, end_point, include_end_point, WalkKeyAdaptor(map));
}
}  // namespace ohm
