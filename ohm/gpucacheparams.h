// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUCACHEPARAMS_H_
#define GPUCACHEPARAMS_H_

#include "ohmconfig.h"

namespace ohm
{
  /// Flags used to create a @c GpuLayerCache.
  enum GpuCacheFlag
  {
    /// Will read voxel data from host to GPU memory.
    GCFRead = (1 << 0),
    /// Will write voxel data from GPU to host memory.
    GCFWrite = (1 << 1),
    /// Using buffers mappable to host memory? Can result in faster data transfer.
    GCFMappable = (1 << 2),

    /// Default creation flags.
    GCFDefaultFlags = GCFRead | GCFMappable
  };

  /// Parameters used in creating a @c GpuCacheLayer in @c GpuCache::createCache().
  struct GpuCacheParams
  {
    /// The size (bytes) of the GPU cache buffer. Use zero to choose the default size specified by the @c GpuCache.
    size_t gpuMemSize = 0;
    /// The @c MapLayer index the cache pulls data from.
    int mapLayer = 0;
    /// Cache creation flags.
    unsigned flags = GCFDefaultFlags;

    GpuCacheParams() = default;
    GpuCacheParams(size_t memSize, int layer, unsigned flags)
      : gpuMemSize(memSize), mapLayer(layer), flags(flags)
    {}
  };
}

#endif // GPUCACHEPARAMS_H_
