// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_GPUMAP_H
#define OHM_GPUMAP_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

namespace gputil
{
  class PinnedBuffer;
}

namespace ohm
{
  class GpuCache;
  struct GpuMapDetail;
  class OccupancyMap;

  namespace gpumap
  {
    /// Enable GPU usage for the given @p map. This creates a GPU cache for the @p map using the
    /// default cache layer memory size and mappable GPU buffers.
    ///
    /// Ignored if the @p map already is GPU enabled, simply returning the existing @c GpuMap.
    ///
    /// @param map The map to enable GPU usage on.
    /// @return The @c GpuCache for the map. Null if GPU code is not enabled.
    GpuCache * ohm_API enableGpu(OccupancyMap &map);

    /// Enable GPU usage for the given @p map.
    ///
    /// Ignored if the @p map already is GPU enabled.
    ///
    /// @param map The map to enable GPU usage on.
    /// @param layer_gpu_mem_size GPU memory buffer size per map layer.
    /// @param mappable_buffers True to use mapped GPU buffers, false to use queued data transfer.
    /// @return The @c GpuCache for the map. Null if GPU code is not enabled.
    GpuCache *ohm_API enableGpu(OccupancyMap &map, size_t layer_gpu_mem_size, bool mappable_buffers = true);

    // /// Reports the status of setting up the associated GPU program for populating the map.
    // ///
    // /// integrateRays() only functions when this function reports @c true. Otherwise calls to @c integrateRays() are
    // /// ignored and this class is non-functional.
    // ///
    // /// @return True when the GPU has been successfully initialised.
    // bool gpuOk();

    /// Sync all GPU layer memory to main memory ensuring main memory is up to date.
    ///
    /// Note: individual @c GpuLayerCache syncing is preferred to global synching.
    ///
    /// Does nothing if the map has no GPU cache or GPU code is disabled.
    /// @param map The map to sync GPU memory for.
    void ohm_API sync(OccupancyMap &map);

    /// Sync a specific GPU memory layer to main memory.
    ///
    /// Does nothing if the map has no GPU cache or GPU code is disabled.
    /// @param map The map to sync GPU memory for.
    /// @param layer_index The index of the layer to sync.
    void ohm_API sync(OccupancyMap &map, unsigned layer_index);

    /// Retrieves the GPU cache used by @p map if GPU usage has been enabled for @p map.
    /// @return The GPU cache for @p map.
    GpuCache * ohm_API gpuCache(OccupancyMap &map);
  }

  /// A wrapper for an @c OccupancyMap that uses GPU to update and manage the wrapped map.
  ///
  /// All map updates should go via this class, not directly to the @c OccupancyMap in order to ensure the map is
  /// correctly maintained. The general usage is:
  /// - Build batches for line segments/sample rays to integrate.
  /// - Call @c integrateRays() with the collected line segments.
  /// - Do the same with additional sample data.
  /// - When required, call @c sync() to ensure CPU memory is up to date.
  ///
  /// The call to @c sync() is required before operating on the @c OccupancyMap directly.
  ///
  /// Use of this class can provide significant gains over pure CPU ray integration using @c OccupancyMap member
  /// functions.
  ///
  /// @par Known issues
  /// - There can sometimes be a significant pause in syncing from GPU. This appears to relate to waiting on the
  ///   OpenCL event, which should already be complete. This may have been due to a now fixed memory overwrite.
  /// - The GPU code may sometimes have excessive contention on updating the value of a single voxel. The GPU code is
  ///   written to abort after a large number of iterations. This also may have been worsened by the memory overwrite.
  /// - With Visual Studio 15.4, the unit tests using this class, such as GpuMap.Populate, often crash in debug.
  ///   Release mode appears ok.
  ///
  /// @todo Evaluate delay creation of @c MapChunk objects. This requires tracking whether a region has been updated
  /// at all by GPU. We only download data for touched regions and create the @c MapChunk then.
  ///
  /// @todo This class has been deprecated as the GPU cache is now included in the OccupancyMap.
  class ohm_API GpuMap
  {
  public:
    /// The default value for @c maxRangeFilter(). Initially 500.
    static const double kDefaultMaxRange;

    /// Construct @c GpuMap support capabilities around @p map. The @p map pointer may be borrowed or owned.
    ///
    /// If @p gpuMemSize is not specified, then up to 1GiB or 3/4 of the GPU memory will be allocated for the GPU
    /// cache, whichever is smaller.
    ///
    /// @param map The map to wrap.
    /// @param borrowed_map True to borrow the map, @c false for this object to take ownership.
    /// @param expected_point_count The expected point count for calls to @c integrateRays(). Used as a hint.
    /// @param gpu_mem_size Optionally specify the target GPU cache memory to allocate.
    GpuMap(OccupancyMap *map, bool borrowed_map = true, unsigned expected_point_count = 2048, size_t gpu_mem_size = 0u);

    /// Destructor. Will wait on outstanding GPU operations first and destroy the @c map() if not using a
    /// @c borrowedPointer().
    ~GpuMap();

    /// Reports the status of setting up the associated GPU program for populating the map.
    ///
    /// integrateRays() only functions when this function reports @c true. Otherwise calls to @c integrateRays() are
    /// ignored and this class is non-functional.
    ///
    /// @return True when the GPU has been successfully initialised.
    bool gpuOk() const;

    /// Request the @c OccupancyMap this @c GpuMap wraps.
    /// @return The underlying @c OccupancyMap.
    OccupancyMap &map();
    /// Request the @c OccupancyMap this @c GpuMap wraps.
    /// @return The underlying @c OccupancyMap.
    const OccupancyMap &map() const;

    /// Is the @c GpuMap using a borrowed pointer to @c map()?
    /// If not borrowed, then the @c GpuMap will destroy the @c OccupancyMap on destruction.
    bool borrowedMap() const;

    /// Returns the maximum allowed range for points added via @c integrateRays(). Excessively long rays may tie up
    /// the GPU unreasonably, so this value is set to avoid this situation. The default is initialised to
    /// @c DefaultMaxRange.
    /// @return The current max range filter value.
    double maxRangeFilter() const;

    /// Sets the maximum ray range allowed. See @c maxRangeFilter(). Setting zero turns max range filtering off.
    /// @param range The new range, zero to disable (not advised!)
    void setMaxRangeFilter(double range);

    /// Sync the GPU memory use for the occupancy layer to main memory ensuring main memory is up to date.
    void syncOccupancy();

    /// Integrate the given @p rays into the map. The @p rays form a list of origin/sample pairs for which
    /// we generally consider the sample voxel as a hit when (increasing occupancy) and all other voxels as misses
    /// (free space). The sample may be treated as a miss when @p endPointsAsOccupied is false.
    ///
    /// This function prepares and queues a GPU update of the affected regions. The time spent in this function is
    /// variable as it may have to wait for existing GPU operations to complete before queuing new operations.
    ///
    /// Points in @p rays are filtered for @c NaN values and for rays longer than @c maxRangeFilter(). This is to avoid
    /// unwanted GPU hangups. Ideally such values should never be passed.
    ///
    /// Note: This call is ignored if @c gpuOk() is @c false.
    ///
    /// @param rays Array of origin/sample point pairs.
    /// @param point_count The number of points in @p rays. The ray count is half this value.
    /// @param end_points_as_occupied When @c true, the end points of the rays increase the occupancy probability.
    ///   Otherwise they decrease the probability just as the rest of the ray.
    /// @return The number of rays integrated. Zero indicates a failure when @p pointCount is not zero.
    ///   In this case either the GPU is unavailable, or all @p rays are invalid.
    unsigned integrateRays(const glm::dvec3 *rays, unsigned point_count, bool end_points_as_occupied = true);

    /// Internal use: get the GPU cache used by this map.
    /// @return The GPU cache this map uses.
    GpuCache *gpuCache() const;

  private:
    /// Wait for previous ray batch, as indicated by @p bufferIndex, to complete.
    /// @param buffer_index Identifies the batch to wait on.
    void waitOnPreviousOperation(int buffer_index);

    /// Enqueue a region for update.
    ///
    /// Ensure the following:
    /// - The region is added to @p map()
    /// - The region is in the @c GpuCache
    /// - Details of the region are added to @p regionsBuffer and @p offsetsBuffer
    ///
    /// This will call @c finaliseBatch() when the chunk cache is full will the still unprocessed batch. This ensures
    /// the current batch executes. The function will then recurse once (using @c allowRetry) modifying the
    /// @p regionsBuffer and @p offsetsBuffer to fill alternative GPU buffers for the next batch.
    ///
    /// @param region_hash The hash code for the region of interest.
    /// @param region_key The key for the region of interest.
    /// @param[in,out] regions_buffer GPU buffer to upload @c regionKey to. Will be assigned a different buffer when the
    ///   @c GpuLayerCache is full.
    /// @param[in,out] offsets_buffer GPU buffer to upload the memory offset for the region to. Will be assigned a
    ///   different buffer when the @c GpuLayerCache is full.
    /// @param end_points_as_occupied When @c true, the end points of the rays increase the occupancy probability.
    ///   Otherwise they decrease the probability just as the rest of the ray.
    /// @param allow_retry Allow recursion when the @c GpuLayerCache?
    void enqueueRegion(unsigned region_hash, const glm::i16vec3 &region_key,
                       gputil::PinnedBuffer &regions_buffer, gputil::PinnedBuffer &offsets_buffer,
                       bool end_points_as_occupied, bool allow_retry);

    /// Finalise the current ray/region batch and start executing GPU kernel.
    /// @param[in,out] regions_buffer GPU buffer containing uploaded region keys. Will be unpinned.
    /// @param[in,out] offsets_buffer GPU buffer containing memory offsets for regions. Will be unpinned.
    /// @param end_points_as_occupied When @c true, the end points of the rays increase the occupancy probability.
    ///   Otherwise they decrease the probability just as the rest of the ray.
    void finaliseBatch(gputil::PinnedBuffer &regions_buffer, gputil::PinnedBuffer &offsets_buffer, bool end_points_as_occupied);

    GpuMapDetail *imp_;
  };
}

#endif // OHM_GPUMAP_H
