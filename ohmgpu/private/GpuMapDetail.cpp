// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuMapDetail.h"

#include "GpuCache.h"
#include "GpuCacheParams.h"
#include "GpuMap.h"
#include "GpuTransformSamples.h"

#include <ohm/DefaultLayer.h>
#include <ohm/MapLayer.h>
#include <ohm/OccupancyMap.h>
#include <ohm/private/OccupancyMapDetail.h>

#include <gputil/gpuDevice.h>

using namespace ohm;

namespace
{
  void onOccupancyLayerChunkSync(MapChunk *chunk, const glm::u8vec3 &region_dimensions)
  {
    chunk->searchAndUpdateFirstValid(region_dimensions);
  }
}  // namespace

GpuMapDetail::~GpuMapDetail()
{
  if (!borrowed_map)
  {
    delete map;
  }
}

GpuCache *ohm::initialiseGpuCache(OccupancyMap &map, size_t layer_gpu_mem_size, unsigned flags)
{
  OccupancyMapDetail *detail = map.detail();
  GpuCache *gpu_cache = static_cast<GpuCache *>(detail->gpu_cache);
  if (!gpu_cache)
  {
    gpu_cache = new GpuCache(map, layer_gpu_mem_size, flags);
    detail->gpu_cache = gpu_cache;

    reinitialiseGpuCache(gpu_cache, map, flags);
  }

  return gpu_cache;
}


void ohm::reinitialiseGpuCache(GpuCache *gpu_cache, OccupancyMap &map, unsigned flags)
{
  if (gpu_cache)
  {
    gpu_cache->clear();
    gpu_cache->removeLayers();

    // Create default layers.
    unsigned mappable_flag = 0;
    if (flags & gpumap::kGpuForceMappedBuffers)
    {
      mappable_flag |= kGcfMappable;
    }
    else if (flags & gpumap::kGpuAllowMappedBuffers)
    {
      // Use mapped buffers if device has unified host memory.
      if (gpu_cache->gpu().unifiedMemory())
      {
        mappable_flag |= kGcfMappable;
      }
    }

    const int occupancy_layer = map.layout().occupancyLayer();
    if (occupancy_layer >= 0)
    {
      gpu_cache->createCache(
        kGcIdOccupancy,
        // On sync, ensure the first valid voxel is updated.
        GpuCacheParams{ 0, occupancy_layer, kGcfRead | kGcfWrite | mappable_flag, &onOccupancyLayerChunkSync });
    }

    // Initialise the voxel mean layer.
    const int mean_layer = map.layout().meanLayer();
    if (mean_layer >= 0)
    {
      gpu_cache->createCache(kGcIdVoxelMean, GpuCacheParams{ 0, mean_layer, kGcfRead | kGcfWrite | mappable_flag });
    }

    const int covariance_layer = map.layout().covarianceLayer();
    if (covariance_layer >= 0)
    {
      // TODO: (KS) add the write flag if we move to being able to process the samples on GPU too.
      gpu_cache->createCache(
        kGcIdCovariance, GpuCacheParams{ 0, covariance_layer, kGcfRead | kGcfWrite | mappable_flag });
    }

    // Note: we create the clearance gpu cache if we have a clearance layer, but it caches the occupancy_layer as that
    // is the information it reads.
    if (map.layout().clearanceLayer() >= 0)
    {
      // Use of occupancy_layer below is correct. See comment above.
      gpu_cache->createCache(kGcIdClearance, GpuCacheParams{ 0, occupancy_layer, kGcfRead | mappable_flag });
    }
  }
}
