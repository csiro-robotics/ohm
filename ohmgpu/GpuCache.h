// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPUCACHE_H
#define OHMGPU_GPUCACHE_H

#include "OhmGpuConfig.h"

#include <ohm/MapRegionCache.h>

#include <cstddef>

namespace gputil
{
class Device;
class Queue;
}  // namespace gputil

namespace ohm
{
struct GpuCacheDetail;
struct GpuLayerCacheParams;
class GpuLayerCache;
class OccupancyMap;

/// IDs for GPU caches.
///
/// Note: there are assumptions made about the ordering of these indices:
///
/// - IDs for read/write layers appear first. For example, @c kGcIdOccupancy which reads/writes occupancy appears
///   before @c kGcIdClearance which only reads occupancy and writes clearance values.
///
/// Locations where these assumptions are most relevant:
///
/// - @c GpuCache::syncLayerTo()
enum GpuCacheId
{
  /// Cache used for populating the map occupancy values.
  kGcIdOccupancy,
  /// Cache of occupancy map values when calculating voxel clearance values. Does not write back to host.
  ///
  /// This cache uploads data from the voxel occupancy layer, but never downloads back to CPU. Instead, the
  /// @c RoiRangeFill class maintains results in it's own buffer and copies these results back to the CPU layer.
  /// This means that the @c GpuLayerCache for the "clearance" voxel layer is always out of sync between CPU and GPU -
  /// they are designed to store different data.
  kGcIdClearance,
  /// Cache used for sub voxel positioning.
  kGcIdVoxelMean,
  /// Cache used for @c CovarianceVoxel data.
  kGcIdCovariance,
};

/// Provides access to the @c GpuLayerCache objects used to cache host voxel data in GPU memory and manage
/// synchronisation.
///
/// The GPU cache contains a number of @c GpuLayerCache objects, each of which is bound to a particular @c MapLayer.
/// Each layer cache is initialised in the same way; sized to @c layerGpuMemSize() bytes and optionally mappable
/// (@c mappableBuffers()) to host memory.
///
/// @c GpuLayerCache objects are instantiates by @c layerCache(). Each cache is identified by user defined ID
/// and set to cache data from a specific @c MapLayer.
///
/// The @c GpuCache also defines the @c gputil::Device and @p gputil::Queue associated with GPU operations.
///
/// For more information on layer cache usage see @c GpuLayerCache.
class ohmgpu_API GpuCache : public MapRegionCache
{
public:
  /// Define for 1 MiB in bytes.
  static constexpr size_t kMiB = 1024ull * 1024ull;
  /// Define for 1 GiB in bytes.
  static constexpr size_t kGiB = 1024ull * 1024ull * 1024ull;
  /// The default byte size of each GPU layer if not specified.
  static constexpr size_t kDefaultLayerMemSize = kGiB / 2;
  /// Default total memory size to target.
  static constexpr size_t kDefaultTargetMemSize = 1 * kGiB;

  /// Instantiate the @c GpuCache for @p map.
  /// @param map The map to cache data for.
  /// @param target_gpu_alloc_size The GPU memory target size, distributed across all allocated @c GpuLayerCache
  /// objects (bytes). See @c targetGpuAllocSize() .
  /// @param flags The @c GpuFlag values to initialise the cache with.
  explicit GpuCache(OccupancyMap &map, size_t target_gpu_alloc_size = kDefaultTargetMemSize, unsigned flags = 0);

  /// Destructor, cleaning up all owned @c GpuLayerCache objects.
  ~GpuCache() override;

  /// Flush and clear the cache, then reallocate GPU buffers. For use when voxel layout changes.
  void reinitialise() override;

  /// Sync to main memory.
  void flush() override;

  /// Flush sync to main memory then drop all cache entries. Cache layers are preserved.
  void clear() override;

  /// Remove all layer caches.
  void removeLayers();

  /// Remove a particular region from the cache.
  /// @param region_key The region to flush from the cache.
  void remove(const glm::i16vec3 &region_key) override;

  /// Implement synching from this cache to another location.
  ///
  /// @param dst_chunk The chunk object to sync to.
  /// @param dst_layer The index to sync to in @c MapChunk::voxel_blocks .
  /// @param src_chunk The chunk to sync from.
  /// @param src_layer The layer to sync from.
  /// @return True if the source chunk/layer pairing are cached by this object and have been copied to the destination
  ///   chunk/layer pairing.
  bool syncLayerTo(MapChunk &dst_chunk, unsigned dst_layer, const MapChunk &src_chunk, unsigned src_layer) override;

  /// Find the @c GpuLayerCache for @p layer .
  MapRegionCache *findLayerCache(unsigned layer) override;

  /// Query the target GPU memory allocation byte size. This is the target allocation accross all @c GpuLayerCache
  /// objects and is distributed amongst these objects. The distribution is weighted so that layers requiring more
  /// voxel data are given more of the allocation.
  ///
  /// @note Specifying an allocation size too small to cover all regions expected to be accessed in a single GPU
  /// batch update will result in undefined behaviour.
  ///
  /// @return The default size of for a layer cache in bytes.
  size_t targetGpuAllocSize() const;

  /// Returns the number of indexable layers. Some may be null.
  /// @return The number of indexable layers.
  unsigned layerCount() const;

  /// Creates a new @c GpuLayerCache with the specified @p params.
  ///
  /// The @p id should always be in a relative low range as a @c vector is used to allocate and access
  /// cache pointers.
  ///
  /// See @c GpuLayerCacheParams for configuration details.
  ///
  /// Fails when:
  /// - A layer cache with @p id already exists.
  ///
  /// @param id The unique ID for the cache.
  /// @param params Cache configuration.
  /// @return A pointer to the new cache on success, null on failure.
  GpuLayerCache *createCache(unsigned id, const GpuLayerCacheParams &params);

  /// Request or create a new @c GpuLayerCache.
  ///
  /// The cache is uniquely identified by the specified @p id, the sematics of which depend on usage. A default
  /// set of IDs is defined in @c GpuCacheId The call also specifies a @p layer which identifies the @c MapLayer
  /// associated with the cache.
  GpuLayerCache *layerCache(unsigned id);

  /// Access the GPU @c gputil::Device associated with GPU operations.
  /// @return The bound @c gputil::Device.
  gputil::Device &gpu();
  /// @overload
  const gputil::Device &gpu() const;
  /// Access the GPU @c gputil::Queue associated with GPU operations.
  /// @return The bound @c gputil::Queue.
  gputil::Queue &gpuQueue();
  /// @overload
  const gputil::Queue &gpuQueue() const;

private:
  GpuCacheDetail *imp_;
};
}  // namespace ohm

#endif  // OHMGPU_GPUCACHE_H
