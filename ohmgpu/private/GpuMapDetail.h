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

#include <ohmutil/VectorHash.h>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wconversion"
#endif // __GNUC__
#include <ska/bytell_hash_map.hpp> // NOLINT
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif // __GNUC__

#include <vector>

namespace ohm
{
  class OccupancyMap;
  class GpuProgramRef;
  class GpuTransformSamples;

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
    /// Buffers for sub-voxel positioning: uint
    gputil::Buffer sub_voxel_buffers[kBuffersCount];

    gputil::Event region_key_upload_events[kBuffersCount];
    gputil::Event region_offset_upload_events[kBuffersCount];
    gputil::Buffer region_key_buffers[kBuffersCount];
    gputil::Buffer region_offset_buffers[kBuffersCount];

    gputil::Event region_update_events[kBuffersCount];

    /// CPU buffer to read back results from GPU ray transformation.
    /// Only ever used in a transient fashion.
    std::vector<glm::vec4> transformed_rays;

    GpuProgramRef *program_ref = nullptr;
    gputil::Kernel update_kernel;

    GpuTransformSamples *transform_samples = nullptr;

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

    ~GpuMapDetail();
  };


  /// Ensure the GPU cache is initialised. Ok to call if already initialised.
  /// @param flags @c GpuFlag values.
  GpuCache *initialiseGpuCache(OccupancyMap &map, size_t layer_gpu_mem_size, unsigned flags);
}  // namespace ohm

#endif  // OHMGPU_GPUMAPDETAIL_H
