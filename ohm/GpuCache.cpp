// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuCache.h"

#include "GpuCacheParams.h"
#include "GpuLayerCache.h"

#include "OhmGpu.h"

#include <gputil/gpuDevice.h>

#include <memory>
#include <vector>

using namespace ohm;

namespace ohm
{
  struct GpuCacheDetail
  {
    std::vector<std::unique_ptr<GpuLayerCache>> layer_caches;
    gputil::Device gpu;
    gputil::Queue gpu_queue;
    OccupancyMap *map = nullptr;
    size_t default_gpu_mem_size = 0;
  };
}  // namespace ohm


GpuCache::GpuCache(OccupancyMap &map, size_t default_gpu_mem_size)
  : imp_(new GpuCacheDetail)
{
  imp_->gpu = ohm::gpuDevice();
  imp_->gpu_queue = imp_->gpu.defaultQueue();
  imp_->map = &map;
  imp_->default_gpu_mem_size = default_gpu_mem_size;
}


GpuCache::~GpuCache()
{
  delete imp_;
}


size_t GpuCache::defaultGpuMemSize() const
{
  return imp_->default_gpu_mem_size;
}


unsigned GpuCache::layerCount() const
{
  return unsigned(imp_->layer_caches.size());
}


GpuLayerCache *GpuCache::createCache(unsigned id, const GpuCacheParams &params)
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

  const size_t layer_mem_size = (params.gpu_mem_size) ? params.gpu_mem_size : imp_->default_gpu_mem_size;
  GpuLayerCache *new_cache = new GpuLayerCache(imp_->gpu, imp_->gpu_queue, *imp_->map, params.map_layer, layer_mem_size,
                                               params.flags, params.on_sync);
  imp_->layer_caches[id] = std::unique_ptr<GpuLayerCache>(new_cache);

  return new_cache;
}


GpuLayerCache *GpuCache::layerCache(unsigned id)
{
  if (id >= imp_->layer_caches.size())
  {
    return nullptr;
  }

  return imp_->layer_caches[id].get();
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
