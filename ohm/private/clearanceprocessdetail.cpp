// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "clearanceprocessdetail.h"

#include "occupancymap.h"

#include <algorithm>

using namespace ohm;

void ClearanceProcessDetail::getWork(OccupancyMap &map)
{
  map.calculateDirtyClearanceExtents(&minDirtyRegion, &maxDirtyRegion, 1);
  currentDirtyCursor = minDirtyRegion;
}


void ClearanceProcessDetail::stepCursor(const glm::i16vec3 &step)
{
  if (currentDirtyCursor.x < maxDirtyRegion.x)
  {
    currentDirtyCursor.x = std::min<int>(currentDirtyCursor.x + step.x, maxDirtyRegion.x);
  }
  else
  {
    currentDirtyCursor.x = minDirtyRegion.x;
    if (currentDirtyCursor.y < maxDirtyRegion.y)
    {
      currentDirtyCursor.y = std::min<int>(currentDirtyCursor.y + step.y, maxDirtyRegion.y);
    }
    else
    {
      currentDirtyCursor.y = minDirtyRegion.y;
      if (currentDirtyCursor.z < maxDirtyRegion.z)
      {
        currentDirtyCursor.z = std::min<int>(currentDirtyCursor.z + step.z, maxDirtyRegion.z);
      }
      else
      {
        currentDirtyCursor = maxDirtyRegion + glm::i16vec3(1);
      }
    }
  }
}
