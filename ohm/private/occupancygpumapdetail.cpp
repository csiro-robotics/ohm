// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancygpumapdetail.h"

#include "gpucache.h"
#include "gpucacheparams.h"
#include "occupancymap.h"
#include "occupancymapdetail.h"

using namespace ohm;

GpuCache *ohm::initialiseGpuCache(OccupancyMap &map, size_t layerGpuMemSize, bool mappableBuffers)
{
  OccupancyMapDetail *detail = map.detail();
  if (!detail->gpuCache)
  {
    detail->gpuCache = new GpuCache(map, layerGpuMemSize);

    // Create default layers.
    unsigned mappableFlag = (mappableBuffers) ? GCFMappable : 0;
    detail->gpuCache->createCache(GCID_Occupancy, GpuCacheParams{ 0, DL_Occupancy, GCFRead | GCFWrite | mappableFlag });
    detail->gpuCache->createCache(GCID_Clearance, GpuCacheParams{ 0, DL_Occupancy, GCFRead | mappableFlag });
  }
  return detail->gpuCache;
}
