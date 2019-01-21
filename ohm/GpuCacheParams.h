// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUCACHEPARAMS_H
#define GPUCACHEPARAMS_H

#include "OhmConfig.h"

#include "GpuCachePostSyncHandler.h"

namespace ohm
{
  /// Flags used to create a @c GpuLayerCache.
  enum GpuCacheFlag
  {
    /// Will read voxel data from host to GPU memory.
    kGcfRead = (1 << 0),
    /// Will write voxel data from GPU to host memory.
    kGcfWrite = (1 << 1),
    /// Using buffers mappable to host memory? Can result in faster data transfer.
    kGcfMappable = (1 << 2),

    /// Default creation flags.
    kGcfDefaultFlags = kGcfRead | kGcfMappable
  };

  /// Parameters used in creating a @c GpuCacheLayer in @c GpuCache::createCache().
  struct GpuCacheParams
  {
    /// The size (bytes) of the GPU cache buffer. Use zero to choose the default size specified by the @c GpuCache.
    size_t gpu_mem_size = 0;
    /// The @c MapLayer index the cache pulls data from.
    int map_layer = 0;
    /// Cache creation flags.
    unsigned flags = kGcfDefaultFlags;
    GpuCachePostSyncHandler on_sync;

    GpuCacheParams() = default;
    GpuCacheParams(size_t mem_size, int layer, unsigned flags,
                   const GpuCachePostSyncHandler &on_sync = GpuCachePostSyncHandler())
      : gpu_mem_size(mem_size)
      , map_layer(layer)
      , flags(flags)
      , on_sync(on_sync)
    {}
  };
}  // namespace ohm

#endif  // GPUCACHEPARAMS_H
