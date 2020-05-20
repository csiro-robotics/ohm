// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_GPUMAPDETAIL_H
#define OHMGPU_GPUMAPDETAIL_H

#include "OhmGpuConfig.h"

#include "GpuCache.h"

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

    inline VoxelUploadInfo(int gpu_layer_id, const gputil::Device &gpu, unsigned prealloc_elements = 1024u)
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
    /// Events for key_buffers
    gputil::Event key_upload_events[kBuffersCount];
    /// Buffers for start/end voxel keys for each ray pair: GpuKey
    gputil::Buffer key_buffers[kBuffersCount];
    /// Events for ray_buffers
    gputil::Event ray_upload_events[kBuffersCount];
    /// Buffers of rays to process float3 pairs. Coordinates are local to the centre of the start voxel for each pair.
    gputil::Buffer ray_buffers[kBuffersCount];

    gputil::Event region_key_upload_events[kBuffersCount];
    // gputil::Event region_offset_upload_events[kBuffersCount];
    gputil::Buffer region_key_buffers[kBuffersCount];
    // gputil::Buffer region_offset_buffers[kBuffersCount];
    gputil::Event region_update_events[kBuffersCount];

    // Item 0 is always the occupancy layer.
    std::vector<VoxelUploadInfo> voxel_upload_info[kBuffersCount];

    GpuProgramRef *program_ref = nullptr;
    gputil::Kernel update_kernel;

    /// Identifies the rays uploaded on the current @c GpuMap::integrateRays() call. Each element is an identifier to a
    /// ray in the current @c rays argument of  @c GpuMap::integrateRays() where the index to the sensor position is
    /// calculated as `current_ray_ids[i] * 2` and the sample position as `current_ray_ids[i] * 2 + 1`.
    ///
    /// This array is only valid for the duraction of the current @c GpuMap::integrateRays() call for derivations
    /// such as the @c GpuNdtMap to use.
    std::vector<unsigned> current_ray_ids;

    RayFilterFunction ray_filter;
    bool custom_ray_filter = false;

    unsigned ray_counts[kBuffersCount] = { 0, 0 };
    unsigned transform_count = 0;
    unsigned region_counts[kBuffersCount] = { 0, 0 };

    int next_buffers_index = 0;
    /// Set of processing regions.
    RegionSet regions;

    /// Used as @c GpuLayerCache::upload() @c batchMarker argument.
    unsigned batch_marker = 1;  // Will cycle odd numbers to avoid zero.
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
  GpuCache *initialiseGpuCache(OccupancyMap &map,  // NOLINT(google-runtime-references)
                               size_t layer_gpu_mem_size, unsigned flags);


  /// (Re)initialise the given GPU @p gpu_cache to reflect the given @p map layout.
  /// @param flags @c GpuFlag values.
  void reinitialiseGpuCache(GpuCache *gpu_cache, OccupancyMap &map,  // NOLINT(google-runtime-references)
                            unsigned flags);
}  // namespace ohm

#endif  // OHMGPU_GPUMAPDETAIL_H
