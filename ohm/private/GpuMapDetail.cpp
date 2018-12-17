// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuMapDetail.h"

#include "DefaultLayer.h"
#include "GpuCache.h"
#include "GpuCacheParams.h"
#include "GpuTransformSamples.h"
#include "OccupancyMap.h"
#include "OccupancyMapDetail.h"
#include "GpuMap.h"

#include <gputil/gpuDevice.h>

using namespace ohm;

GpuMapDetail::~GpuMapDetail()
{
  if (!borrowed_map)
  {
    delete map;
  }
  delete transform_samples;
}


GpuCache *ohm::initialiseGpuCache(OccupancyMap &map, size_t layer_gpu_mem_size, unsigned flags)
{
  OccupancyMapDetail *detail = map.detail();
  if (!detail->gpu_cache)
  {
    detail->gpu_cache = new GpuCache(map, layer_gpu_mem_size);

    // Create default layers.
    unsigned mappable_flag = 0;
    if (flags & gpumap::kGpuForceMappedBuffers)
    {
      mappable_flag |= kGcfMappable;
    }
    else if (flags & gpumap::kGpuAllowMappedBuffers)
    {
      // Use mapped buffers if device has unified host memory.
      if (detail->gpu_cache->gpu().unifiedMemory())
      {
        mappable_flag |= kGcfMappable;
      }
    }

    const int occupancy_layer = map.layout().occupancyLayer();
    if (occupancy_layer >= 0)
    {
      detail->gpu_cache->createCache(kGcIdOccupancy,
                                     GpuCacheParams{ 0, occupancy_layer, kGcfRead | kGcfWrite | mappable_flag });
    }

    // Note: we create teh clearance gpu cache if we have a clearance layer, but it caches the occupancy_layer as that
    // is the information it reads.
    if (map.layout().clearanceLayer() >= 0)
    {
      // Use of occupancy_layer below is correct. See comment above.
      detail->gpu_cache->createCache(kGcIdClearance, GpuCacheParams{ 0, occupancy_layer, kGcfRead | mappable_flag });
    }
  }
  return detail->gpu_cache;
}
