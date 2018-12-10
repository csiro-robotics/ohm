// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_GPUMAPDETAIL_H
#define OHM_GPUMAPDETAIL_H

#include "OhmConfig.h"

#include "GpuCache.h"
#include "RayFilter.h"

#include <glm/glm.hpp>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuKernel.h>

#include <unordered_map>
#include <vector>

namespace ohm
{
  class OccupancyMap;
  class GpuTransformSamples;

  struct GpuMapDetail
  {
    static const unsigned kBuffersCount = 2;
    OccupancyMap *map;
    typedef std::unordered_multimap<unsigned, glm::i16vec3> RegionKeyMap;
    // Ray/key buffer upload event pairs.
    gputil::Event key_upload_events[kBuffersCount];
    gputil::Buffer key_buffers[kBuffersCount];
    gputil::Event ray_upload_events[kBuffersCount];
    gputil::Buffer ray_buffers[kBuffersCount];
    gputil::Buffer transform_positions_buffer;
    gputil::Buffer transform_rotations_buffer;
    gputil::Buffer transform_times_buffer;

    gputil::Event region_key_upload_events[kBuffersCount];
    gputil::Event region_offset_upload_events[kBuffersCount];
    gputil::Buffer region_key_buffers[kBuffersCount];
    gputil::Buffer region_offset_buffers[kBuffersCount];

    gputil::Event region_update_events[kBuffersCount];

    /// CPU buffer to read back results from GPU ray transformation.
    /// Only ever used in a transient fashion.
    std::vector<glm::vec4> transformed_rays;
    //gputil::Device gpu;
    gputil::Kernel update_kernel;

    GpuTransformSamples *transform_samples = nullptr;

    RayFilterFunction ray_filter;
    bool custom_ray_filter = false;

    unsigned ray_counts[kBuffersCount] = { 0, 0 };
    unsigned transform_count = 0;
    unsigned region_counts[kBuffersCount] = { 0, 0 };

    int next_buffers_index = 0;
    // Should be a multi-map in case of hash clashes.
    RegionKeyMap regions;
    /// Used as @c GpuLayerCache::upload() @c batchMarker argument.
    unsigned batch_marker = 1;  // Will cycle odd numbers to avoid zero.
    bool borrowed_map = false;
    bool gpu_ok = false;

    GpuMapDetail(OccupancyMap *map, bool borrowed_map)
      : map(map)
      , borrowed_map(borrowed_map)
    {}

    ~GpuMapDetail();

    RegionKeyMap::iterator findRegion(unsigned region_hash, const glm::i16vec3 &region_key);
    RegionKeyMap::const_iterator findRegion(unsigned region_hash, const glm::i16vec3 &region_key) const;

  protected:
    template <typename ITER, typename T>
    static ITER findRegion(T &regions, unsigned region_hash, const glm::i16vec3 &region_key);
  };


  /// Ensure the GPU cache is initialised. Ok to call if already initialised.
  /// @param flags @c GpuFlag values.
  GpuCache *initialiseGpuCache(OccupancyMap &map, size_t layer_gpu_mem_size, unsigned flags);

  inline GpuMapDetail::RegionKeyMap::iterator GpuMapDetail::findRegion(const unsigned region_hash,
                                                                       const glm::i16vec3 &region_key)
  {
    return findRegion<RegionKeyMap::iterator>(regions, region_hash, region_key);
  }


  inline GpuMapDetail::RegionKeyMap::const_iterator GpuMapDetail::findRegion(const unsigned region_hash,
                                                                             const glm::i16vec3 &region_key) const
  {
    return findRegion<RegionKeyMap::const_iterator>(regions, region_hash, region_key);
  }

  template <typename ITER, typename T>
  ITER GpuMapDetail::findRegion(T &regions, const unsigned region_hash, const glm::i16vec3 &region_key)
  {
    ITER iter = regions.find(region_hash);
    while (iter != regions.end() && iter->first == region_hash && iter->second != region_key)
    {
      ++iter;
    }

    if (iter != regions.end() && iter->first == region_hash && iter->second == region_key)
    {
      return iter;
    }

    return regions.end();
  }
}  // namespace ohm

#endif  // OHM_GPUMAPDETAIL_H
