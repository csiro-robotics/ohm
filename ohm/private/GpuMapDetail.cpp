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

using namespace ohm;

GpuMapDetail::~GpuMapDetail()
{
  if (!borrowed_map)
  {
    delete map;
  }
  delete transform_samples;
}


GpuCache *ohm::initialiseGpuCache(OccupancyMap &map, size_t layer_gpu_mem_size, bool mappable_buffers)
{
  OccupancyMapDetail *detail = map.detail();
  if (!detail->gpu_cache)
  {
    detail->gpu_cache = new GpuCache(map, layer_gpu_mem_size);

    // Create default layers.
    const unsigned mappable_flag = (mappable_buffers) ? kGcfMappable : 0;
    detail->gpu_cache->createCache(kGcIdOccupancy,
                                   GpuCacheParams{ 0, kDlOccupancy, kGcfRead | kGcfWrite | mappable_flag });
    detail->gpu_cache->createCache(kGcIdClearance, GpuCacheParams{ 0, kDlOccupancy, kGcfRead | mappable_flag });
  }
  return detail->gpu_cache;
}
