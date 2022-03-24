// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "CalculateSegmentKeys.h"

#include "KeyList.h"
#include "LineWalk.h"

namespace ohm
{
size_t calculateSegmentKeys(KeyList &keys, const OccupancyMap &map, const glm::dvec3 &start_point,
                            const glm::dvec3 &end_point, bool include_end_point)
{
  keys.clear();
  return walkSegmentKeys(
    LineWalkContext(map,
                    [&keys](const Key &key, double enter_range, double exit_range, const glm::ivec3 &stepped) {
                      (void)enter_range;  // Unused
                      (void)exit_range;   // Unused
                      (void)stepped;      // Unused

                      keys.add(key);
                      return true;
                    }),
    start_point, end_point, include_end_point);
}
}  // namespace ohm
