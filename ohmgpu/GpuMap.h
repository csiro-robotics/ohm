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
#include <ohm/RayMapper.h>

#include <glm/glm.hpp>

#include <functional>

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

struct VoxelUploadInfo;

namespace gpumap
{
/// Flags for GPU initialisation.
enum GpuFlag : unsigned
{
  /// Allow host (mappable) buffers. Used if device/host memory is unified.
  kGpuAllowMappedBuffers = (1u << 0u),
  /// Force mappable buffers.
  kGpuForceMappedBuffers = (1u << 1u),
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
/// @param target_gpu_mem_size Target GPU memory usage. This is split amongst the active, default layers.
/// @param gpu_flags @c GpuFlag values controlling initialisation.
/// @return The @c GpuCache for the map. Null if GPU code is not enabled.
GpuCache *ohmgpu_API enableGpu(OccupancyMap &map, size_t target_gpu_mem_size,
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

// --- Internal support items ---
/// Callback for walkRegions().
using RegionWalkFunction = std::function<void(const glm::i16vec3 &, const glm::dvec3 &, const glm::dvec3 &)>;

/// Walk the regions touched by the line connecting @p start_point and @p end_point invoking @c on_visit for each
/// touched region. This is similar @c walkSegmentKeys() in @c LineWalk.h .
///
/// @param map The map to walk.
/// @param start_point The line/ray origin.
/// @param end_point The line/ray end point.
/// @param on_visit Function to call for each visited region.
void ohmgpu_API walkRegions(const OccupancyMap &map, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                            const RegionWalkFunction &on_visit);
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
class ohmgpu_API GpuMap : public RayMapper
{
protected:
  /// Constructor for derived classes to call.
  /// @param detail The pimpl data struture for the map. Must not be null. May be a derivation of @c GpuMapDetail .
  /// @param expected_element_count The expected point count for calls to @c integrateRays(). Used as a hint.
  /// @param gpu_mem_size Optionally specify the target GPU cache memory to allocate.
  explicit GpuMap(GpuMapDetail *detail, unsigned expected_element_count = 2048,  // NOLINT(readability-magic-numbers)
                  size_t gpu_mem_size = 0u);

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
  explicit GpuMap(OccupancyMap *map, bool borrowed_map = true,
                  unsigned expected_element_count = 2048,  // NOLINT(readability-magic-numbers)
                  size_t gpu_mem_size = 0u);

  /// Destructor. Will wait on outstanding GPU operations first and destroy the @c map() if not using a
  /// @c borrowedPointer().
  ~GpuMap() override;

  /// Reports the status of setting up the associated GPU program for populating the map.
  ///
  /// integrateRays() only functions when this function reports @c true. Otherwise calls to @c integrateRays() are
  /// ignored and this class is non-functional.
  ///
  /// @return True when the GPU has been successfully initialised.
  bool gpuOk() const;

  /// Validate function from @c RayMapper . Based on @c gpuOk() result.
  /// @return True if validated and @c integrateRays() is safe to call.
  inline bool valid() const override { return gpuOk(); }

  /// Request the @c OccupancyMap this @c GpuMap wraps.
  /// @return The underlying @c OccupancyMap.
  OccupancyMap &map();
  /// Request the @c OccupancyMap this @c GpuMap wraps.
  /// @return The underlying @c OccupancyMap.
  const OccupancyMap &map() const;

  /// Is the @c GpuMap using a borrowed pointer to @c map()?
  /// If not borrowed, then the @c GpuMap will destroy the @c OccupancyMap on destruction.
  bool borrowedMap() const;

  /// Sync the GPU memory use for the occupancy, voxel mean and NDT layers to main memory ensuring main memory is up
  /// to date.
  void syncVoxels();

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

  /// Get the ray segment length used to break up long rays. See @c setRaySegmentLength()
  /// @return The ray length at which to break up rays.
  double raySegmentLength() const;

  /// Set the ray segment length used to break up long rays.
  ///
  /// Long rays in a GPU warp (CUDA terminology) can cause the warp to do very little work as a single long ray can
  /// cause many additional iteractions when the rest of the warp has finished its work. To combat this, we can break
  /// up long rays into multiple parts or segments. Setting @c setRaySegmentLength() to a value greater than the map
  /// resolution enables this feature. Note, however, this should be a "reasonable value" for the given resolution
  /// or the GPU will be overloaded doing very little work per thread with significant overheads.
  ///
  /// Each ray longer than the specified @p length is broken in to segments of approximately equal length. Essentially
  /// the @c raySegmentLength() sets the upper bound for each segment, but each segment will have equal length.
  ///
  /// @param length The ray length at which to break up rays.
  void setRaySegmentLength(double length);

  /// Query if rays are being grouped by sample before upload to GPU.
  ///
  /// This is only set for algorithms which require grouping of rays such as the NDT update used by @c GpuNdtMap .
  /// It is not a user configurable property because of its algorithmic dependency. Rays are grouped by pre sorting
  /// in CPU before GPU upload. This means that ray batches are reordered before upload.
  ///
  /// @return True if rays are grouped by sample voxel before upload to GPU.
  bool groupedRays() const;

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
  /// @param intensities Optional--for each ray, intensity of the return (element_count/2 elements).
  /// @param region_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
  /// @return The number of rays integrated. Zero indicates a failure when @p pointCount is not zero.
  ///   In this case either the GPU is unavailable, or all @p rays are invalid.
  size_t integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                       unsigned region_update_flags) override;

  using RayMapper::integrateRays;

  /// Internal use: get the GPU cache used by this map.
  /// @return The GPU cache this map uses.
  GpuCache *gpuCache() const;

protected:
  /// Some derivation uses of the @c GpuMap allow initialisation with a null map, though this is an edge case.
  ///
  /// This function allows the map pointer to be set. It is expected that use cases where the map can change always have
  /// @p borrowed_map @c true .
  ///
  /// No public API @c GpuMap implementation should support a null map.
  void setMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size,
              bool force_gpu_program_release);

  /// Called after @c syncVoxels() completes synching known data.
  virtual inline void onSyncVoxels(int buffer_index) { (void)buffer_index; }

  /// Set the status of @c sortedRays() .
  ///
  /// This should only be set for algorithms which require ray grouping. For example, @c GpuNdtMap sorts rays for
  /// performance reasons.
  ///
  /// @param group The grouping status to set.
  void setGroupedRays(bool group);

  /// Cache the correct GPU program to cater for @c with_voxel_mean. Releases the existing program first when
  /// @p force is true or @p with_voxel_mean does not match the cached program.
  /// @param with_voxel_mean True to cache the program which supports voxel mean positioning (@ref voxelmean).
  /// @param with_traversal Include traversal calculations? Requires "traversal" layer.
  /// @param force Force release and program caching even if already correct. Must be used on initialisation.
  virtual void cacheGpuProgram(bool with_voxel_mean, bool with_traversal, bool force);

  /// Release the current GPU program.
  virtual void releaseGpuProgram();

  /// Implementation for various ways we can integrate rays into the map. See @c integrateRays() for general detail.
  /// @param rays Array of origin/sample point pairs. Expect either @c glm::dvec3 (preferred) or @c glm::vec3.
  /// @param element_count The number of points in @p rays. The ray count is half this value.
  /// @param intensities Optional--for each ray, intensity of the return (element_count/2 elements).
  /// @param region_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
  /// @param filter Filter function apply to each ray before passing to GPU. May be empty.
  /// @return The number of rays integrated. Zero indicates a failure when @p pointCount is not zero.
  ///   In this case either the GPU is unavailable, or all @p rays are invalid.
  size_t integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                       unsigned region_update_flags, const RayFilterFunction &filter);

  /// Wait for previous ray batch, as indicated by @p buffer_index, to complete.
  /// @param buffer_index Identifies the batch to wait on.
  void waitOnPreviousOperation(int buffer_index);

  /// Enqueue upload of voxel data for the regions specified in @c GpuMapDetail::voxel_upload_info .
  ///
  /// May trigger a limited number of attempts to  @c finaliseBatch() if @c enqueueRegion() fails.
  /// @param buffer_index Index of active data buffers from @c GpuMapDetail to used.
  /// @param region_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
  virtual void enqueueRegions(int buffer_index, unsigned region_update_flags);

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
  /// @param buffer_index Index of active data buffers from @c GpuMapDetail to used.
  virtual bool enqueueRegion(const glm::i16vec3 &region_key, int buffer_index);

  /// Finalise the current ray/region batch and start executing GPU kernel.
  /// @param region_update_flags Flags controlling ray integration behaviour. See @c RayFlag.
  virtual void finaliseBatch(unsigned region_update_flags);

  GpuMapDetail *imp_;  ///< Internal pimpl data.
};
}  // namespace ohm

#endif  // OHMGPU_GPUMAP_H
