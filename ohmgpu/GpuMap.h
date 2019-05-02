// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPUMAP_H
#define OHMGPU_GPUMAP_H

#include "OhmGpuConfig.h"

#include <ohm/RayFilter.h>
#include <ohm/RayFlag.h>

#include <glm/glm.hpp>

namespace gputil
{
  class Event;
  class Buffer;
  class PinnedBuffer;
}  // namespace gputil

namespace ohm
{
  class Aabb;
  class GpuCache;
  struct GpuMapDetail;
  class OccupancyMap;
  class RayFilter;

  namespace gpumap
  {
    /// Flags for GPU initialisation.
    enum GpuFlag
    {
      /// Allow host (mappable) buffers. Used if device/host memory is unified.
      kGpuAllowMappedBuffers = (1 << 0),
      /// Force mappable buffers.
      kGpuForceMappedBuffers = (1 << 1),
    };

    /// Enable GPU usage for the given @p map. This creates a GPU cache for the @p map using the
    /// default cache layer memory size and mappable GPU buffers.
    ///
    /// Ignored if the @p map already is GPU enabled, simply returning the existing @c GpuMap.
    ///
    /// @param map The map to enable GPU usage on.
    /// @return The @c GpuCache for the map. Null if GPU code is not enabled.
    GpuCache *ohmgpu_API enableGpu(OccupancyMap &map);

    /// Enable GPU usage for the given @p map.
    ///
    /// Ignored if the @p map already is GPU enabled.
    ///
    /// @param map The map to enable GPU usage on.
    /// @param layer_gpu_mem_size GPU memory buffer size per map layer.
    /// @param gpu_flags @c GpuFlag values controlling initialisation.
    /// @return The @c GpuCache for the map. Null if GPU code is not enabled.
    GpuCache *ohmgpu_API enableGpu(OccupancyMap &map, size_t layer_gpu_mem_size,
                                unsigned gpu_flags = kGpuAllowMappedBuffers);

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
    void ohmgpu_API sync(OccupancyMap &map);

    /// Sync a specific GPU memory layer to main memory.
    ///
    /// Does nothing if the map has no GPU cache or GPU code is disabled.
    /// @param map The map to sync GPU memory for.
    /// @param layer_index The index of the layer to sync.
    void ohmgpu_API sync(OccupancyMap &map, unsigned layer_index);

    /// Retrieves the GPU cache used by @p map if GPU usage has been enabled for @p map.
    /// @return The GPU cache for @p map.
    GpuCache *ohmgpu_API gpuCache(OccupancyMap &map);
  }  // namespace gpumap

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
  class ohmgpu_API GpuMap
  {
  public:
    /// Construct @c GpuMap support capabilities around @p map. The @p map pointer may be borrowed or owned.
    ///
    /// If @p gpuMemSize is not specified, then up to 1GiB or 3/4 of the GPU memory will be allocated for the GPU
    /// cache, whichever is smaller.
    ///
    /// @param map The map to wrap.
    /// @param borrowed_map True to borrow the map, @c false for this object to take ownership.
    /// @param expected_element_count The expected point count for calls to @c integrateRays(). Used as a hint.
    /// @param gpu_mem_size Optionally specify the target GPU cache memory to allocate.
    GpuMap(OccupancyMap *map, bool borrowed_map = true, unsigned expected_element_count = 2048,
           size_t gpu_mem_size = 0u);

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

    /// Sync the GPU memory use for the occupancy layer to main memory ensuring main memory is up to date.
    void syncOccupancy();

    /// Set the range filter applied to all rays given to @c integrateRays(). Setting a null filter ensures no
    /// filtering is performed. The default behaviour is to use the same filter as the @c OccupancyMap.
    ///
    /// Note: not having a filter for a GpuMap is highly inadvisable; bad data such as infinite NaN points will cause
    /// the GPU to hang.
    ///
    /// @param ray_filter The range filter to install and apply to @c integrateRays().
    ///   Accepts a null pointer, which clears the filter.
    void setRayFilter(const RayFilterFunction &ray_filter);

    /// Get the installed range filter applied to all rays given to @c integrateRays().
    /// @return The installed range filter.
    const RayFilterFunction &rayFilter() const;

    /// Get the range filter actually being used. This will be the one belonging to the wrapped @c OccupancyMap when
    /// the @c GpuMap does not have an explicitly installed filter.
    /// @return The range filter currently in use.
    const RayFilterFunction &effectiveRayFilter() const;

    /// Clears the @c rayFilter(). Unlike the same method in @c OccupancyMap, this is not the same a setting a null
    /// filter. For the @p GpuMap, @c clearRayFilter() restores the default behaviour of using the same filter
    /// as the underlying @c OccupancyMap.
    void clearRayFilter();

    /// Access the @c OccupancyMap::hitValue() for API compatibility.
    /// @return @c OccupancyMap::hitValue().
    float hitValue() const;

    /// Pass-through to @c OccupancyMap::setHitValue() for API compatibility.
    /// @param value New hit value.
    void setHitValue(float value);

    /// Access the @c OccupancyMap::missValue() for API compatibility.
    /// @return @c OccupancyMap::missValue().
    float missValue() const;

    /// Pass-through to @c OccupancyMap::setMissValue() for API compatibility.
    /// @param value New miss value.
    void setMissValue(float value);

    /// Integrate the given @p rays into the map. The @p rays form a list of origin/sample pairs for which
    /// we generally consider the sample voxel as a hit when (increasing occupancy) and all other voxels as misses
    /// (free space). The sample may be treated as a miss when @p endPointsAsOccupied is false.
    ///
    /// This function prepares and queues a GPU update of the affected regions. The time spent in this function is
    /// variable as it may have to wait for existing GPU operations to complete before queuing new operations.
    ///
    /// Points are filtered using the @c effectiveRayFilter(). It is highly advisable to always have a filter
    /// installed which removes infinite and NaN points and long sample rays.
    ///
    /// Note: This call is ignored if @c gpuOk() is @c false.
    ///
    /// @param rays Array of origin/sample point pairs.
    /// @param element_count The number of points in @p rays. The ray count is half this value.
    /// @param region_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
    /// @return The number of rays integrated. Zero indicates a failure when @p pointCount is not zero.
    ///   In this case either the GPU is unavailable, or all @p rays are invalid.
    unsigned integrateRays(const glm::dvec3 *rays, unsigned element_count, unsigned region_update_flags = kRfDefault);

    /// Integrate a ray clearing pattern into the map. A clearing is integrated as a set of rays using the @c RayFlag
    /// set: @c kRfStopOnFirstOccupied, @c kRfClearOnly. This has the effect of reducing the probability of the first
    /// occupied voxel encountered, then stopping ray traversal.
    ///
    /// Clearing patterns are intended as a way to remove samples which would otherwise persist. This may occur when
    /// lidar samples are added from a transient object, the object moves, but no new samples are attained behind the
    /// transient samples. A clearing pattern will erode these. The erosion is countered by resampling where obstacles
    /// persist.
    ///
    /// @param rays The set of clearing pattern rays to integrate into the map.
    /// @param element_count Number of origin/end point pairs in @p rays. Must be even/
    void applyClearingPattern(const glm::dvec3 *rays, unsigned element_count);

    /// An overload which builds a clearance pattern from a cone.
    /// @param apex The apex of the cone.
    /// @param cone_axis The central axis of the cone.
    /// @param cone_angle The angle between @p cone_axis and the cone wall (radians).
    /// @param range The length of each ray in the cone. Note this makes for a round bottom cone.
    /// @param angular_resolution The angular resolution to build the cone to (radians).
    void applyClearingPattern(const glm::dvec3 &apex, const glm::dvec3 &cone_axis, double cone_angle, double range, double angular_resolution = 0);

    /// Internal use: get the GPU cache used by this map.
    /// @return The GPU cache this map uses.
    GpuCache *gpuCache() const;

  private:
    /// Cache the correct GPU program to cater for @c with_sub_voxels. Releases the existing program first when
    /// @p force is true or @p with_sub_voxels does not match the cached program.
    /// @param with_sub_voxels True to cache the program which supports sub-voxel positioning (@ref subvoxel).
    /// @param force Force release and program caching even if already correct. Must be used on initialisation.
    void cacheGpuProgram(bool with_sub_voxels, bool force);

    /// Release the current GPU program.
    void releaseGpuProgram();

    /// Implementation for various ways we can integrate rays into the map. See @c integrateRays() for general detail.
    /// @param rays Array of origin/sample point pairs. Expect either @c glm::dvec3 (preferred) or @c glm::vec3.
    /// @param element_count The number of points in @p rays. The ray count is half this value.
    /// @param region_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
    /// @param filter Filter function apply to each ray before passing to GPU. May be empty.
    /// @return The number of rays integrated. Zero indicates a failure when @p pointCount is not zero.
    ///   In this case either the GPU is unavailable, or all @p rays are invalid.
    template <typename VEC_TYPE>
    unsigned integrateRaysT(const VEC_TYPE *rays, unsigned element_count, unsigned region_update_flags,
                            const RayFilterFunction &filter);

    /// Wait for previous ray batch, as indicated by @p buffer_index, to complete.
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
    /// @param region_key The key for the region of interest.
    /// @param[in,out] regions_buffer GPU buffer to upload @c regionKey to. Will be assigned a different buffer when the
    ///   @c GpuLayerCache is full.
    /// @param[in,out] offsets_buffer GPU buffer to upload the memory offset for the region to. Will be assigned a
    ///   different buffer when the @c GpuLayerCache is full.
    /// @param region_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
    /// @param allow_retry Allow recursion when the @c GpuLayerCache?
    void enqueueRegion(const glm::i16vec3 &region_key, gputil::PinnedBuffer &regions_buffer,
                       gputil::PinnedBuffer &offsets_buffer, unsigned region_update_flags, bool allow_retry);

    /// Finalise the current ray/region batch and start executing GPU kernel.
    /// @param[in,out] regions_buffer GPU buffer containing uploaded region keys. Will be unpinned.
    /// @param[in,out] offsets_buffer GPU buffer containing memory offsets for regions. Will be unpinned.
    /// @param region_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
    void finaliseBatch(gputil::PinnedBuffer &regions_buffer, gputil::PinnedBuffer &offsets_buffer,
                       unsigned region_update_flags);

    GpuMapDetail *imp_;
  };
}  // namespace ohm

#endif  // OHMGPU_GPUMAP_H
