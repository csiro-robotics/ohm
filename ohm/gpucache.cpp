// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpucache.h"

#include "gpucacheparams.h"
#include "gpulayercache.h"

#include "occupancygpu.h"

#include <gputil/gpudevice.h>

#include <vector>

using namespace ohm;

namespace ohm
{
  struct GpuCacheDetail
  {
    std::vector<GpuLayerCache *> layerCaches;
    gputil::Device gpu;
    gputil::Queue gpuQueue;
    OccupancyMap *map = nullptr;
    size_t defaultGpuMemSize = 0;
  };
}


GpuCache::GpuCache(OccupancyMap &map, size_t defaultGpuMemSize)
  : _imp(new GpuCacheDetail)
{
  _imp->gpu = ohm::gpuDevice();
  _imp->gpuQueue = _imp->gpu.defaultQueue();
  _imp->map = &map;
  _imp->defaultGpuMemSize = defaultGpuMemSize;
}


GpuCache::~GpuCache()
{
  delete _imp;
}


size_t GpuCache::defaultGpuMemSize() const
{
  return _imp->defaultGpuMemSize;
}


unsigned GpuCache::layerCount() const
{
  return unsigned(_imp->layerCaches.size());
}


GpuLayerCache *GpuCache::createCache(unsigned id, const GpuCacheParams &params)
{
  while (id >= _imp->layerCaches.size())
  {
    _imp->layerCaches.push_back(nullptr);
  }

  if (_imp->layerCaches[id])
  {
    // Already allocated
    return nullptr;
  }

  const size_t layerMemSize = (params.gpuMemSize) ? params.gpuMemSize : _imp->defaultGpuMemSize;
  GpuLayerCache *newCache = new GpuLayerCache(_imp->gpu, _imp->gpuQueue,
    *_imp->map, params.mapLayer, layerMemSize, params.flags);
  _imp->layerCaches[id] = newCache;

  return newCache;
}


GpuLayerCache *GpuCache::layerCache(unsigned id)
{
  if (id >= _imp->layerCaches.size())
  {
    return nullptr;
  }

  return _imp->layerCaches[id];
}


gputil::Device &GpuCache::gpu()
{
  return _imp->gpu;
}


const gputil::Device &GpuCache::gpu() const
{
  return _imp->gpu;
}


gputil::Queue &GpuCache::gpuQueue()
{
  return _imp->gpuQueue;
}


const gputil::Queue &GpuCache::gpuQueue() const
{
  return _imp->gpuQueue;
}
