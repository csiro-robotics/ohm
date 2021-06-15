// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuCache.h"

#include "GpuLayerCache.h"
#include "GpuLayerCacheParams.h"

#include "OhmGpu.h"

#include "private/GpuMapDetail.h"

#include <ohm/MapChunk.h>
#include <ohm/VoxelBlock.h>
#include <ohm/VoxelBuffer.h>

#include <gputil/gpuDevice.h>

#include <memory>
#include <vector>

namespace ohm
{
// Support pre C++17 ODR
constexpr size_t GpuCache::kMiB;
constexpr size_t GpuCache::kGiB;
constexpr size_t GpuCache::kDefaultLayerMemSize;
constexpr size_t GpuCache::kDefaultTargetMemSize;


struct GpuCacheDetail
{
  std::vector<std::unique_ptr<GpuLayerCache>> layer_caches;
  gputil::Device gpu;
  gputil::Queue gpu_queue;
  OccupancyMap *map = nullptr;
  size_t target_gpu_alloc_size = 0;
  unsigned flags = 0;
};


GpuCache::GpuCache(OccupancyMap &map, size_t target_gpu_alloc_size, unsigned flags)
  : imp_(new GpuCacheDetail)
{
  imp_->gpu = ohm::gpuDevice();
  imp_->gpu_queue = imp_->gpu.defaultQueue();
  imp_->map = &map;
  imp_->target_gpu_alloc_size = target_gpu_alloc_size;
  imp_->flags = flags;
}


GpuCache::~GpuCache()
{
  delete imp_;
}


void GpuCache::reinitialise()
{
  reinitialiseGpuCache(this, *imp_->map, imp_->flags);
}


void GpuCache::flush()
{
  for (auto &&layer : imp_->layer_caches)
  {
    if (layer)
    {
      layer->syncToMainMemory();
    }
  }
}


void GpuCache::clear()
{
  for (auto &&layer : imp_->layer_caches)
  {
    if (layer)
    {
      layer->clear();
    }
  }
}


void GpuCache::removeLayers()
{
  imp_->layer_caches.clear();
}


void GpuCache::remove(const glm::i16vec3 &region_key)
{
  for (auto &&layer : imp_->layer_caches)
  {
    if (layer)
    {
      layer->remove(region_key);
    }
  }
}


bool GpuCache::syncLayerTo(MapChunk &dst_chunk, unsigned dst_layer, const MapChunk &src_chunk, unsigned src_layer)
{
  // Iterate the layers looking for the appropriate one. Note we accept the first one which matches the src_layer index.
  // There are situations where this could be incorrect as two layer caches use the same source layer, however,
  // this should be ok with how the layer indices are placed. See assumptions on @c GpuCacheId .
  //
  // Note (KS): this iteration isn't particularly efficient, but we should have small enough layers sets. A mapping of
  // src_layer to cache may be useful later.
  for (auto &&layer : imp_->layer_caches)
  {
    if (layer && layer->layerIndex() == src_layer)
    {
      VoxelBuffer<VoxelBlock> dst(dst_chunk.voxel_blocks[dst_layer]);
      if (layer->syncToExternal(dst, src_chunk.region.coord))
      {
        return true;
      }
      break;
    }
  }

  return false;
}


MapRegionCache *GpuCache::findLayerCache(unsigned layer)
{
  // Iterate the layers looking for the appropriate one. Note we accept the first one which matches the src_layer index.
  // There are situations where this could be incorrect as two layer caches use the same source layer, however,
  // this should be ok with how the layer indices are placed. See assumptions on @c GpuCacheId .
  //
  // Note (KS): this iteration isn't particularly efficient, but we should have small enough layers sets. A mapping of
  // src_layer to cache may be useful later.
  for (auto &&layer_cache : imp_->layer_caches)
  {
    if (layer_cache && layer_cache->layerIndex() == layer)
    {
      return layer_cache.get();
    }
  }

  return nullptr;
}


size_t GpuCache::targetGpuAllocSize() const
{
  return imp_->target_gpu_alloc_size;
}


unsigned GpuCache::layerCount() const
{
  return unsigned(imp_->layer_caches.size());
}


GpuLayerCache *GpuCache::createCache(unsigned id, const GpuLayerCacheParams &params)
{
  while (id >= imp_->layer_caches.size())
  {
    imp_->layer_caches.push_back(nullptr);
  }

  if (imp_->layer_caches[id])
  {
    // Already allocated
    return nullptr;
  }

  const size_t layer_mem_size = (params.gpu_mem_size) ? params.gpu_mem_size : kDefaultLayerMemSize;
  imp_->layer_caches[id] = std::make_unique<GpuLayerCache>(imp_->gpu, imp_->gpu_queue, *imp_->map, params.map_layer,
                                                           layer_mem_size, params.flags, params.on_sync);

  return imp_->layer_caches[id].get();
}


GpuLayerCache *GpuCache::layerCache(unsigned id)
{
  if (id >= imp_->layer_caches.size())
  {
    return nullptr;
  }

  return imp_->layer_caches[id].get();
}


gputil::Device &GpuCache::gpu()
{
  return imp_->gpu;
}


const gputil::Device &GpuCache::gpu() const
{
  return imp_->gpu;
}


gputil::Queue &GpuCache::gpuQueue()
{
  return imp_->gpu_queue;
}


const gputil::Queue &GpuCache::gpuQueue() const
{
  return imp_->gpu_queue;
}
}  // namespace ohm
