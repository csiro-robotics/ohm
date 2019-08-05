// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPULAYERCACHE_H
#define OHMGPU_GPULAYERCACHE_H

#include "OhmGpuConfig.h"

#include "GpuCachePostSyncHandler.h"

#include <glm/glm.hpp>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuQueue.h>

namespace ohm
{
  struct GpuCacheEntry;
  struct GpuLayerCacheDetail;
  struct MapChunk;
  class MapLayer;
  class OccupancyMap;

  /// Defines a GPU memory cache of voxel data.
  ///
  /// The cache may be used to ensure voxels from a @c MapChunk::voxel_maps associated with a @c MapLayer are uploaded
  /// to GPU and optionally downloaded back to gpu. A layer cache should only be created via @c GpuCache with the
  /// appropriate @c GpuCacheParams and @c GpuCacheFlag selection. Once created a cache is bound to transfer voxel data
  /// to/from a specific @c MapLayer only.
  ///
  /// The GPU memory is allocated as a single, large memory buffer. Voxel data are uploaded to available regions within
  /// this buffer when calling @p upload(). The return value identified the byte offset into the buffer where data for
  /// the specific region are located. The region data persist in the cache as long as possible. The oldest region is
  /// dropped when @c upload() is called and the cache is full.
  ///
  /// Typical usage is as follows:
  /// - Start a batch with @c beginBatch()
  /// - @c upload() or @c allocate()
  ///   - @c upload() the data from the required @c MapChunk sections using the batch from @c beginBatch().
  ///   - @c allocate() creates space for the chunk, but does not upload. This is for operations which download to the
  ///     chunk only.
  /// - Execute GPU code.
  /// - Synchronise data back to host memory.
  ///
  /// Using @c beginBatch() returns a rolling batch marker. When this batch marker is passed to @p upload() the
  /// cache ensures no region with the same batch marker is removed from the cache. In this way all regions required for
  /// a GPU program may be locked in the cache. The @p upload() call may fail when using batch markers if the cache is
  /// not large enough.
  class ohmgpu_API GpuLayerCache
  {
  public:
    /// Status return values for @c upload().
    enum CacheStatus
    {
      /// The region has been previously uploaded to the GPU and exists in the cache.
      kCacheExisting,
      /// The region has been newly uploaded to GPU (in progress).
      kCacheNew,
      /// The cache is full and the region will not be uploaded on this call.
      /// Only when @c upload() @c batchMarker is used.
      kCacheFull,
    };

    /// Flags for caching methods affecting @c upload() and @c allocate()
    enum CacheFlag
    {
      /// Allow creation of the region chunk in the map if it does not exist?
      kAllowRegionCreate = (1 << 0),
      /// Download of results is not required for this region.
      /// Note changes in this flag value on may force the cache to stall.
      kSkipDownload = (1 << 1),
      /// Force upload even if already cached.
      kForceUpload = (1 << 2),
    };

    /// Create a new layer cache.
    ///
    /// This allocated a GPU buffer up to @c targetGpuMemSize bytes. The actual target allocation is a multiple of
    /// the data size for a single map chunk. This may be padded slightly further to create an optimal GPU buffer size.
    /// The result will be set to a number of regions not to exceed @p targetGpuMemSize, however the padding may
    /// result is exceeding the target value. The allocation is also limited to half the GPU memory size.
    ///
    /// @param gpu The GPU device to allocate in.
    /// @param gpu_queue GPU queue used to execute memory transfer in.
    /// @param map The map to which the cache belongs.
    /// @param layer_index Identifies @c MapLayer from which to cache voxel data.
    /// @param target_gpu_mem_size The maximum allowed buffer size (bytes).
    /// @param flags Creation flags: see @c GpuCacheFlag. @c GCFRead is currently mandatory.
    /// @param on_sync Defines a function to invoke after a @c MapChunk is synched to main memory (CPU).
    GpuLayerCache(const gputil::Device &gpu, const gputil::Queue &gpu_queue,
                  OccupancyMap &map,  // NOLINT(google-runtime-references)
                  unsigned layer_index, size_t target_gpu_mem_size, unsigned flags,
                  GpuCachePostSyncHandler on_sync = GpuCachePostSyncHandler());

    /// Release the GPU cache. Does not synchronise to host memory.
    ~GpuLayerCache();

    /// Generate a new batch marker for use with @c upload() @c batchMarker parameter.
    /// @return The next rolling batch marker.
    unsigned beginBatch();

    /// Identifies the voxel layer the GPU cache operates on. See @c MapLayout and @c MapChunk::voxel_maps.
    /// @return The voxel layer index this cache operates on.
    unsigned layerIndex() const;

    /// Handler invoked after a MapChunk has been synchronised to main memory.
    /// @return The function object invoked on sync.
    const GpuCachePostSyncHandler &onSyncHandler() const;

    /// Allocate space for voxel data from the @c MapChunk identified by @c regionKey to GPU.
    /// The GPU memory is directly associated with the @c MapChunk voxel layer given by
    /// @c layerIndex().
    ///
    /// This allocates space for the voxel layer from the @c MapChunk defined by @c regionKey,
    /// leaving the GPU memory uninitialised. The assumption is that the GPU will set the correct
    /// values which are to be downloaded to CPU later. Allocation is skipped when the layer is
    /// already in the GPU cache.
    ///
    /// Allocation supports batching operations for a group of regions to be processed
    /// collectively. When @p batchMarker is non-zero, the @p batchMarker is associated with the
    /// region. The @c GpuCache will not remove any regions from the cache which share the same
    /// @p batchMarker. This means that a call to @c allocate() or @c upload() may fail to allocate the requested
    /// region. In this case, the return value is ~0u and the @p status is set to @c CacheFull.
    /// Canonically, only the @p status value should be reliably used.
    ///
    /// Note that each region may only have one @c batchMarker associated with it. Only the most
    /// recent @c batchMarker value is associated with a region in the cache. Also note that
    /// a @c batchMarker of zero will not overwrite the previously associated marker value.
    ///
    /// @param map The map from which we are uploading data.
    /// @param region_key The key for the region to upload.
    /// @param[out] chunk Set to the @c chunk identified by @p regionKey. Null if the region does not exist and @c
    ///     AllowRegionCreate is false.
    /// @param[out] event Optional to request the GPU event marking the any outstanding operations on the cache memory.
    ///     Only relevant when the region is already in the cache.s
    /// @param[out] status Optional to request the status of the cache entry.
    /// @param batch_marker Optionally set to demarcate a group of upload operations.
    /// @param flags a combination of @c CacheFlag flag values.
    /// @return The byte offset into @c buffer() where the region is being uploaded. ~0u on
    ///     failure to upload as determined by the use of the @p batchMarker.
    size_t allocate(OccupancyMap &map,                                 // NOLINT(google-runtime-references)
                    const glm::i16vec3 &region_key, MapChunk *&chunk,  // NOLINT(google-runtime-references)
                    gputil::Event *event, CacheStatus *status = nullptr, unsigned batch_marker = 0,
                    unsigned flags = 0u);

    /// Queue upload of voxel data from the @c MapChunk identified by @c regionKey to GPU.
    /// Uploads voxel data for the @c MapChunk voxel layer given by @c layerIndex().
    ///
    /// This takes the associated voxel layer from the @c MapChunk defined by @c regionKey,
    /// demarcates space for the chunk in  the GPU cache and uploads the CPU data to GPU.
    /// When the region chunk does not exist, GPU memory is still allocated and initialised with the
    /// default clear value for that layer. Upload is skipped when the cache is already up to date
    /// for the specified region. Upload behaviour may be modified by setting @c flags as per @c CacheFlag.
    ///
    /// Data upload supports batching the upload for a group of regions to be processed
    /// collectively. When @p batchMarker is non-zero, the @p batchMarker is associated with the
    /// region. The @c GpuCache will not remove any regions from the cache which share the same
    /// @p batchMarker. This means that a call to @c allocate() or @c upload() may fail to allocate the requested
    /// region. In this case, the return value is ~0u and the @p status is set to @c CacheFull.
    /// Canonically, only the @p status value should be reliably used.
    ///
    /// Note that each region may only have one @c batchMarker associated with it. Only the most
    /// recent @c batchMarker value is associated with a region in the cache. Also note that
    /// a @c batchMarker of zero will not overwrite the previously associated marker value.
    ///
    /// @param map The map from which we are uploading data.
    /// @param region_key The key for the region to upload.
    /// @param[out] chunk Set to the @c chunk identified by @p regionKey. Null if the region does not exist and @c
    ///     AllowRegionCreate is false.
    /// @param[out] event Optional to request the GPU event marking the completion of the upload.
    /// @param[out] status Optional to request the status of the requested upload.
    /// @param batch_marker Optionally set to demarcate a group of upload operations.
    /// @param flags a combination of @c CacheFlag flag values.
    /// @return The byte offset into @c buffer() where the region is being uploaded. ~0u on
    ///     failure to upload as determined by the use of the @p batchMarker.
    size_t upload(OccupancyMap &map,                                 // NOLINT(google-runtime-references)
                  const glm::i16vec3 &region_key, MapChunk *&chunk,  // NOLINT(google-runtime-references)
                  gputil::Event *event, CacheStatus *status = nullptr, unsigned batch_marker = 0, unsigned flags = 0u);

    /// Lookup the cache to see if the chunk identified by @p regionKey is present in the cache.
    ///
    /// If the cache is present, then this method:
    /// - Sets @p offset to the byte offset into @p buffer() where the region is stored (if provided).
    /// - Sets @p currentEvent to the most recent @c gputil::Event associated with data transfer for this region
    ///   (if provided).
    /// - Returns @c true.
    ///
    /// @param map The map which owns the cache.
    /// @param region_key The key for the map chunk of interest.
    /// @param[out] offset Set to the byte offset in @c buffer() if in the cache. May be null.
    /// @param[out] current_event Set to the most recent GPU data manipulation event associated with this chunk.
    ///   May be null.
    /// @return True if the region is cached, false otherwise.
    bool lookup(OccupancyMap &map,  // NOLINT(google-runtime-references)
                const glm::i16vec3 &region_key, size_t *offset, gputil::Event *current_event = nullptr);

    /// Access the GPU memory buffer to which regions are uploaded. Required as an argument to GPU kernels.
    /// @return The cache's region GPU buffer.
    gputil::Buffer *buffer() const;

    /// Update the most recent event affecting the memory buffer used by @p chunk.
    ///
    /// Use case is for when executing a kernel that touched the GPU buffer associated with @p chunk.
    /// Any time such a kernel is executed, the chunk's event must be updated to the completion
    /// of that kernel, effectively locking the chunk on GPU until the kernel completes.
    ///
    /// Does nothing if @p chunk is not currently cached, though that is a logical error.
    ///
    /// @param chunk The map chunk to update the most recent event for.
    /// @param event The most recent event to associate with @p chunk.
    void updateEvent(MapChunk &chunk, gputil::Event &event);  // NOLINT(google-runtime-references)

    /// Update the event for all cached regions marked with @p batchMarker (see @c upload()).
    /// @param batch_marker The maker to match against.
    /// @param event The most recent event to associate.
    void updateEvents(unsigned batch_marker, gputil::Event &event);  // NOLINT(google-runtime-references)

    /// Remove data associated with @p region_key from the cache.
    /// This will block until outstanding operations relating to @p chunk complete, but will not explicitly sync data
    /// back to the host.
    /// @param region_key The key of the region to remove from the cache.
    void remove(const glm::i16vec3 &region_key);

    /// Synchronise GPU memory for @p chunk back to main memory.
    ///
    /// Safe to call when @p chunk is not currently uploaded to GPU, in which case this method does little.
    ///
    /// This method blocks until synchronisation has completed.
    ///
    /// @param chunk The map chunk to synchronise.
    void syncToMainMemory(const MapChunk &chunk);

    /// An overload which accepts a chunk pointer, which may be null.
    /// @param chunk The map chunk to synchronise.
    inline void syncToMainMemory(const MapChunk *chunk)
    {
      if (chunk)
      {
        syncToMainMemory(*chunk);
      }
    }

    /// @overload
    void syncToMainMemory(const glm::i16vec3 &region_key);

    /// Synchronise all GPU chunk memory back to main memory. This may block while outstanding GPU operations complete.
    void syncToMainMemory();

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

    /// Query the number of regions currently in the cache.
    /// @return The number of cached regions.
    unsigned cachedCount() const;

    /// Query the number of regions the cache can hold.
    /// @return The number of regions the cache can hold at any one time.
    unsigned cacheSize() const;

    /// Total size of the GPU buffer(s) allocated by this layer cache.
    /// @return The byte size of this cache's GPU buffer(s).
    unsigned bufferSize() const;

    /// Bytes used by each cache entry in the GPU buffer.
    /// @return The byte size of each cached chunk in GPU.
    unsigned chunkSize() const;

    /// Clear the cache then reallocate using the initial constraints. This should be called whenever the layout of
    /// the associated @c MapLayer changes, although this should be a rare occurrence.
    ///
    /// This method does not perform a @c syncToMainMemory() before clearing the cache. When reorganising layers
    /// the sync should be performed before the reorganisation, not on reallocation.
    ///
    /// @param map The map to which the @c GpuLayerCache belongs.
    void reallocate(const OccupancyMap &map);

    /// Drop all cache entries. Call @c syncToMainMemory() first if data should be synched first.
    void clear();

  private:
    /// Internal cache resolution/allocation function. The @p upload flag controls whether the call
    /// just makes space for the chunk, or if it uploads data s well.
    GpuCacheEntry *resolveCacheEntry(OccupancyMap &map,  // NOLINT(google-runtime-references)
                                     const glm::i16vec3 &region_key,
                                     MapChunk *&chunk,  // NOLINT(google-runtime-references)
                                     gputil::Event *event, CacheStatus *status, unsigned batch_marker, unsigned flags,
                                     bool upload);

    void allocateBuffers(const OccupancyMap &map, const MapLayer &layer, size_t target_gpu_mem_size);

    /// Sync a cache entry to main memory.
    ///
    /// Note: this method invoked the @c GpuCachePostSyncHandler only when @p wait_on_sync is true.
    /// When @p wait_on_sync is false, the caller must resolve calling the @c GpuCachePostSyncHandler.
    /// @param entry The cache entry.
    /// @param wait_on_sync Wait for sync to finish?
    void syncToMainMemory(GpuCacheEntry &entry, bool wait_on_sync);  // NOLINT(google-runtime-references)

    GpuCacheEntry *findCacheEntry(const glm::i16vec3 &region_key);
    const GpuCacheEntry *findCacheEntry(const glm::i16vec3 &region_key) const;

    GpuCacheEntry *findCacheEntry(const MapChunk &chunk);
    const GpuCacheEntry *findCacheEntry(const MapChunk &chunk) const;

    GpuLayerCacheDetail *imp_;
  };


  // class GpuLayerCacheWriteLock
  // {
  // public:
  //   inline GpuLayerCacheWriteLock(GpuLayerCache &cache) : _cache(&cache) {}
  //   inline ~GpuLayerCacheWriteLock() { unlock(); }

  //   inline void unlock()
  //   {
  //     if (_cache)
  //     {
  //       // _cache->setWriteLock(false);
  //       _cache = nullptr;
  //     }
  //   }

  // private:
  //   GpuLayerCache *_cache;
  // };
}  // namespace ohm

#endif  // OHMGPU_GPULAYERCACHE_H
