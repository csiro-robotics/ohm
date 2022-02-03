// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPUMAPDETAIL_H
#define OHMGPU_GPUMAPDETAIL_H

#include "OhmGpuConfig.h"

#include "GpuCache.h"
#include "RayItem.h"

#include <ohm/Key.h>
#include <ohm/RayFilter.h>

#include <glm/glm.hpp>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>

#include <ohmutil/VectorHash.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#endif                              // __GNUC__
#include <ska/bytell_hash_map.hpp>  // NOLINT
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif  // __GNUC__

#include <array>
#include <vector>

namespace ohm
{
class OccupancyMap;
class GpuProgramRef;
class GpuTransformSamples;

/// Tracks information about voxel data being uploaded.
struct VoxelUploadInfo
{
  /// A GPU buffer which tracks the voxel memory offsets for each region being uploaded. This contains uint64_t
  /// offsets, one for each region being uploaded. It corresponds directly to the region keys in
  /// @c GpuMapDetail::region_offset_buffers .
  ///
  /// @todo comment about how this contains offsets in voxels, not bytes as relating to data in the a
  /// @c GpuLayerCache buffer.
  gputil::Buffer offsets_buffer;
  /// Pinned wrapper to @c offsets_buffer
  gputil::PinnedBuffer offsets_buffer_pinned;
  /// Last upload event to @c offsets_buffer
  gputil::Event offset_upload_event;
  /// Last voxel upload event vai the corresponding @c GpuLayerCache .
  gputil::Event voxel_upload_event;
  /// The @c GpuLayerCache in the @c GpuCache which this structure references.
  int gpu_layer_id = 0;
  /// Allow the region to be instantiated in CPU before uploading? Otherwise the default memory block is created.
  bool allow_region_creation = true;
  /// Skip syncing this @c gpu_layer_id back to CPU on voxel sync?
  bool skip_cpu_sync = false;

  inline VoxelUploadInfo() = default;

  inline VoxelUploadInfo(int gpu_layer_id, const gputil::Device &gpu,
                         unsigned prealloc_elements = 1024u)  // NOLINT(readability-magic-numbers)
    : offsets_buffer(gpu, sizeof(uint64_t) * prealloc_elements, gputil::kBfReadHost)
    , gpu_layer_id(gpu_layer_id)
  {}
};

struct GpuMapDetail
{
  using RegionSet = ska::bytell_hash_set<glm::i16vec3, Vector3Hash<glm::i16vec3>>;

  static const unsigned kBuffersCount = 2;
  OccupancyMap *map;
  // Ray/key buffer upload event pairs.
  /// Events for @p key_buffers
  std::array<gputil::Event, kBuffersCount> key_upload_events;
  /// Buffers for start/end voxel keys for each ray pair: GpuKey
  std::array<gputil::Buffer, kBuffersCount> key_buffers;
  /// Events for @p ray_buffers
  std::array<gputil::Event, kBuffersCount> ray_upload_events;
  /// Buffers of rays to process float3 pairs. Coordinates are local to the centre of the start voxel for each pair.
  std::array<gputil::Buffer, kBuffersCount> ray_buffers;
  /// Buffers holding only sample points for each ray in @p ray_buffers. Occupancy algorithms do not need this
  /// information only the current voxel and whether it's the sample voxelor not  - flagged in the key_buffers -
  /// are important. However, algorithms such as TSDF need to know the distance to the final sample point. We may
  /// lose this information either by GPU ray segmentation (rays split into multiple segments) or by ray filtering
  /// where rays may be truncated. This buffer is populated when @p use_original_sample_buffers is set and contains
  /// one item per ray. The coordinates uploaded are relative to the corresponding ray end voxel (second item in each
  /// ray/voxel pair), using the voxel centre as the local origin. For unclipped rays, this will be the same as the end
  /// point.
  std::array<gputil::Buffer, kBuffersCount> original_ray_buffers;
  /// Events for @p original_ray_buffers
  std::array<gputil::Event, kBuffersCount> original_ray_upload_events;
  /// Buffers to upload sample intensities - a single floating point value per sample.
  std::array<gputil::Buffer, kBuffersCount> intensities_buffers;
  /// Events for @p intensities_buffers
  std::array<gputil::Event, kBuffersCount> intensities_upload_events;
  /// Buffers to upload sample timestamps - a uint32 value per sample - quantised, relative time.
  std::array<gputil::Buffer, kBuffersCount> timestamps_buffers;
  /// Events for @p timestamps_buffers
  std::array<gputil::Event, kBuffersCount> timestamps_upload_events;

  std::array<gputil::Event, kBuffersCount> region_key_upload_events;
  std::array<gputil::Buffer, kBuffersCount> region_key_buffers;
  std::array<gputil::Event, kBuffersCount> region_update_events;

  // Item 0 is always the occupancy layer.
  std::array<std::vector<VoxelUploadInfo>, kBuffersCount> voxel_upload_info;
  /// Vector used to group/sort rays when @c group_rays is `true`.
  std::vector<RayItem> grouped_rays;

  GpuProgramRef *program_ref = nullptr;
  gputil::Kernel update_kernel;

  RayFilterFunction ray_filter;
  bool custom_ray_filter = false;

  /// Number of rays (origin/sample pairs) in the corresponding ray_buffers items.
  std::array<unsigned, kBuffersCount> ray_counts = { 0, 0 };
  /// Number of rays (origin/sample pairs) in the corresponding ray_buffers items which contain unclipped end (sample)
  /// points.
  std::array<unsigned, kBuffersCount> unclipped_sample_counts = { 0, 0 };
  unsigned transform_count = 0;
  std::array<unsigned, kBuffersCount> region_counts = { 0, 0 };

  int next_buffers_index = 0;
  /// Set of processing regions.
  RegionSet regions;

  /// Long rays are broken into segments of this length or smaller (when value is > 0).
  double ray_segment_length = 0;

  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the occupancy layer.
  int occupancy_uidx = -1;
  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the mean layer.
  int mean_uidx = -1;
  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the traversal layer.
  int traversal_uidx = -1;
  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the touch time layer.
  int touch_time_uidx = -1;
  /// Index into @c voxel_upload_info buffers at which we have the @c VoxelUploadInfo for the incident normal layer.
  int incident_normal_uidx = -1;

  /// Used as @c GpuLayerCache::upload() @c batchMarker argument.
  unsigned batch_marker = 1;  // Will cycle odd numbers to avoid zero.
  /// Should rays be grouped (sorted) before GPU upload. Should only be set for some algorthims, like NDT (required).
  bool group_rays = false;
  /// Should we populate @p original_ray_buffers, storing the unclipped rays points for each ray? See
  /// comments on @p original_ray_buffers.
  bool use_original_ray_buffers = false;
  /// True if the @p map is borrowed. False, if @p map is owned here and must be deleted on destruction.
  bool borrowed_map = false;
  /// Have GPU resources been initialised?
  bool gpu_ok = false;
  /// Have cached a GPU program with sub-voxel positioning ( @c VoxelMean )?
  /// @todo I think this is defunct.
  bool cached_sub_voxel_program = false;
  /// Support voxel mean GPU cache layer? This is enabled by default, but can be disabled in specific derivations.
  bool support_voxel_mean = true;
  /// Support traversal GPU cache layer? This is enabled by default, but can be disabled in specific derivations.
  bool support_traversal = true;

  GpuMapDetail(OccupancyMap *map, bool borrowed_map)
    : map(map)
    , borrowed_map(borrowed_map)
  {}

  virtual ~GpuMapDetail();
};


/// Ensure the GPU cache is initialised. Ok to call if already initialised.
/// @param flags @c GpuFlag values.
GpuCache *initialiseGpuCache(OccupancyMap &map, size_t target_gpu_mem_size, unsigned flags);


/// (Re)initialise the given GPU @p gpu_cache to reflect the given @p map layout.
/// @param flags @c GpuFlag values.
void reinitialiseGpuCache(GpuCache *gpu_cache, OccupancyMap &map, unsigned flags);
}  // namespace ohm

#endif  // OHMGPU_GPUMAPDETAIL_H
