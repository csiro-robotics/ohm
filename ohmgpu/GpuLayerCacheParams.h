// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPULAYERCACHEPARAMS_H
#define GPULAYERCACHEPARAMS_H

#include "OhmGpuConfig.h"

#include "GpuCachePostSyncHandler.h"

#include <algorithm>

namespace ohm
{
/// Flags used to create a @c GpuLayerCache.
enum GpuLayerCacheFlag : unsigned
{
  /// Will read voxel data from host to GPU memory.
  kGcfRead = (1u << 0u),
  /// Will write voxel data from GPU to host memory.
  kGcfWrite = (1u << 1u),
  /// Using buffers mappable to host memory? Can result in faster data transfer.
  kGcfMappable = (1u << 2u),

  /// Default creation flags.
  kGcfDefaultFlags = kGcfRead | kGcfMappable
};

/// Parameters used in creating a @c GpuCacheLayer in @c GpuCache::createCache().
struct ohmgpu_API GpuLayerCacheParams
{
  /// The size (bytes) of the GPU cache buffer. Use zero to choose the default size specified by the @c GpuCache.
  size_t gpu_mem_size = 0;
  /// The @c MapLayer index the cache pulls data from.
  int map_layer = 0;
  /// @c GpuLayerCacheFlag cache creation flags.
  unsigned flags = kGcfDefaultFlags;
  /// Callback invoked on synchronising memory back to CPU.
  /// The calling thread may not be the main thread, so the function must be threadsafe.
  GpuCachePostSyncHandler on_sync;

  /// Default constructor.
  inline GpuLayerCacheParams() = default;
  /// Construct with the given member values.
  /// @param mem_size Target GPU cache size.
  /// @param layer The @c MapLayer index which the GPU layer cache synchronised with.
  /// @param flags @c GpuLayerCacheFlag cache creation flags.
  /// @param on_sync Callback to invoke on synchronising back to CPU.
  inline GpuLayerCacheParams(size_t mem_size, int layer, unsigned flags,
                             GpuCachePostSyncHandler on_sync = GpuCachePostSyncHandler())
    : gpu_mem_size(mem_size)
    , map_layer(layer)
    , flags(flags)
    , on_sync(std::move(on_sync))
  {}
};
}  // namespace ohm

#endif  // GPULAYERCACHEPARAMS_H
