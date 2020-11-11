// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPUMAPDETAIL_H
#define OHMGPU_GPUMAPDETAIL_H

#include "OhmGpuConfig.h"

#include "GpuCache.h"

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
  int gpu_layer_id;

  inline VoxelUploadInfo() = default;

  inline VoxelUploadInfo(int gpu_layer_id, const gputil::Device &gpu,
                         unsigned prealloc_elements = 1024u)  // NOLINT(readability-magic-numbers)
    : offsets_buffer(gpu, sizeof(uint64_t) * prealloc_elements, gputil::kBfReadHost)
    , gpu_layer_id(gpu_layer_id)
  {}
};

/// Structure used for sorting algorithm to group rays before upload.
struct RayItem
{
  /// Origin of the ray. Unless clipped, this is the sensor location.
  glm::dvec3 origin;
  /// End point of the sample ray. Unless clipped, this is the location of the sample detection.
  glm::dvec3 sample;
  /// Map @c Key corresponding to @p origin .
  Key origin_key;
  /// Map @c Key corresponding to @p sample .
  Key sample_key;
  /// @c RayFilterFlag values corresponding to any modification which have been made to @p origin and @p sample .
  unsigned filter_flags;
};

struct GpuMapDetail
{
  using RegionSet = ska::bytell_hash_set<glm::i16vec3, Vector3Hash<glm::i16vec3>>;

  static const unsigned kBuffersCount = 2;
  OccupancyMap *map;
  // Ray/key buffer upload event pairs.
  /// Events for key_buffers
  std::array<gputil::Event, kBuffersCount> key_upload_events;
  /// Buffers for start/end voxel keys for each ray pair: GpuKey
  std::array<gputil::Buffer, kBuffersCount> key_buffers;
  /// Events for ray_buffers
  std::array<gputil::Event, kBuffersCount> ray_upload_events;
  /// Buffers of rays to process float3 pairs. Coordinates are local to the centre of the start voxel for each pair.
  std::array<gputil::Buffer, kBuffersCount> ray_buffers;

  std::array<gputil::Event, kBuffersCount> region_key_upload_events;
  std::array<gputil::Buffer, kBuffersCount> region_key_buffers;
  std::array<gputil::Event, kBuffersCount> region_update_events;

  // Item 0 is always the occupancy layer.
  std::array<std::vector<VoxelUploadInfo>, kBuffersCount> voxel_upload_info;
  /// Vector used to group/sort rays when @c group_rays is `true`.
  std::vector<RayItem> grouped_rays;

  GpuProgramRef *g_program_ref = nullptr;
  gputil::Kernel update_kernel;

  RayFilterFunction ray_filter;
  bool custom_ray_filter = false;

  std::array<unsigned, kBuffersCount> ray_counts = { 0, 0 };
  unsigned transform_count = 0;
  std::array<unsigned, kBuffersCount> region_counts = { 0, 0 };

  int next_buffers_index = 0;
  /// Set of processing regions.
  RegionSet regions;

  /// Used as @c GpuLayerCache::upload() @c batchMarker argument.
  unsigned batch_marker = 1;  // Will cycle odd numbers to avoid zero.
  /// Should rays be grouped (sorted) before GPU upload. Should only be set for some algorthims, like NDT (required).
  bool group_rays = false;
  bool borrowed_map = false;
  bool gpu_ok = false;
  bool cached_sub_voxel_program = false;

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
