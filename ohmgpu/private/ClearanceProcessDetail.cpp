// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ClearanceProcessDetail.h"

#include <ohm/OccupancyMap.h>

#include <algorithm>

namespace ohm
{
void ClearanceProcessDetail::getWork(OccupancyMap &map)
{
  map.calculateDirtyClearanceExtents(&min_dirty_region, &max_dirty_region, 1);
  current_dirty_cursor = min_dirty_region;
}


void ClearanceProcessDetail::stepCursor(const glm::i16vec3 &step)
{
  if (current_dirty_cursor.x < max_dirty_region.x)
  {
    current_dirty_cursor.x = std::min<int>(current_dirty_cursor.x + step.x, max_dirty_region.x);
  }
  else
  {
    current_dirty_cursor.x = min_dirty_region.x;
    if (current_dirty_cursor.y < max_dirty_region.y)
    {
      current_dirty_cursor.y = std::min<int>(current_dirty_cursor.y + step.y, max_dirty_region.y);
    }
    else
    {
      current_dirty_cursor.y = min_dirty_region.y;
      if (current_dirty_cursor.z < max_dirty_region.z)
      {
        current_dirty_cursor.z = std::min<int>(current_dirty_cursor.z + step.z, max_dirty_region.z);
      }
      else
      {
        current_dirty_cursor = max_dirty_region + glm::i16vec3(1);
      }
    }
  }
}
}  // namespace ohm
