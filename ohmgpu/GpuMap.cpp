// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research
// Organisation (CSIRO) ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuMap.h"

#include "GpuCache.h"
#include "GpuKey.h"
#include "GpuLayerCache.h"
#include "GpuTransformSamples.h"
#include "OhmGpu.h"

#include <ohm/Aabb.h>
#include <ohm/DefaultLayer.h>
#include <ohm/MapChunk.h>
#include <ohm/MapRegion.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/RayFilter.h>
#include <ohm/VoxelMean.h>

#include "private/GpuMapDetail.h"
#include "private/GpuProgramRef.h"

#include <ohm/private/OccupancyMapDetail.h>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuProgram.h>

#include <glm/ext.hpp>

#include <array>
#include <cassert>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <iostream>

/// Enable to verify ray sorting pushes unclipped samples to the begining of the list.
#define OHM_GPU_VERIFY_SORT 0

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#include "RegionUpdateResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdate);
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdateSubVox);
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdateWithTraversal);
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdateSubVoxWithTraversal);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

namespace ohm
{
namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_sub_vox("RegionUpdate", GpuProgramRef::kSourceString, RegionUpdateCode,  // NOLINT
                                    RegionUpdateCode_length, { "-DVOXEL_MEAN" });
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_sub_vox_with_traversal("RegionUpdate", GpuProgramRef::kSourceString,
                                                   RegionUpdateCode,  // NOLINT
                                                   RegionUpdateCode_length, { "-DVOXEL_MEAN", "-DTRAVERSAL" });
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_sub_vox("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u,
                                    { "-DVOXEL_MEAN" });
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_sub_vox_with_traversal("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u,
                                                   { "-DVOXEL_MEAN", "-DTRAVERSAL" });
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_no_sub("RegionUpdate", GpuProgramRef::kSourceString, RegionUpdateCode,  // NOLINT
                                   RegionUpdateCode_length);
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_no_sub_with_traversal("RegionUpdate", GpuProgramRef::kSourceString,
                                                  RegionUpdateCode,  // NOLINT
                                                  RegionUpdateCode_length, { "-DTRAVERSAL" });
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_no_sub("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u);
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_no_sub_with_traversal("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u,
                                                  { "-DTRAVERSAL" });
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

const double kDefaultMaxRayRange = 1000.0;

inline bool goodRay(const glm::dvec3 &start, const glm::dvec3 &end, double max_range = kDefaultMaxRayRange)
{
  bool is_good = true;
  if (glm::any(glm::isnan(start)))
  {
    // std::cerr << "NAN start point" << std::endl;
    is_good = false;
  }
  if (glm::any(glm::isnan(end)))
  {
    // std::cerr << "NAN end point" << std::endl;
    is_good = false;
  }

  const glm::dvec3 ray = end - start;
  if (max_range > 0 && glm::dot(ray, ray) > max_range * max_range)
  {
    // std::cerr << "Ray too long: (" <<
    // glm::distance(start, end) << "): " << start << " ->
    // " << end << std::endl;
    is_good = false;
  }

  return is_good;
}

#if OHM_GPU_VERIFY_SORT
/// Verify that rays are sorted such that all unclipped samples come first.
unsigned verifySort(const std::vector<RayItem> &rays)
{
  unsigned failure_count = 0;
  bool allow_samples = true;
  for (const auto &ray : rays)
  {
    if (!allow_samples && (ray.filter_flags & kRffClippedEnd) == 0)
    {
      ++failure_count;
    }
    if (ray.filter_flags & kRffClippedEnd)
    {
      allow_samples = false;
    }
  }

  if (failure_count)
  {
    throw failure_count;
  }

  return failure_count;
}
#endif  // OHM_GPU_VERIFY_SORT
}  // namespace

namespace gpumap
{
GpuCache *enableGpu(OccupancyMap &map)
{
  return enableGpu(map, GpuCache::kDefaultTargetMemSize, kGpuAllowMappedBuffers);
}


GpuCache *enableGpu(OccupancyMap &map, size_t target_gpu_mem_size, unsigned flags)
{
  OccupancyMapDetail &map_imp = *map.detail();
  if (map_imp.gpu_cache)
  {
    return static_cast<GpuCache *>(map_imp.gpu_cache);
  }

  initialiseGpuCache(map, target_gpu_mem_size, flags);
  return static_cast<GpuCache *>(map_imp.gpu_cache);
}


void sync(OccupancyMap &map)
{
  if (GpuCache *cache = gpuCache(map))
  {
    for (unsigned i = 0; i < cache->layerCount(); ++i)
    {
      if (GpuLayerCache *layer = cache->layerCache(i))
      {
        layer->syncToMainMemory();
      }
    }
  }
}


void sync(OccupancyMap &map, unsigned layer_index)
{
  if (GpuCache *cache = gpuCache(map))
  {
    if (GpuLayerCache *layer = cache->layerCache(layer_index))
    {
      layer->syncToMainMemory();
    }
  }
}


GpuCache *gpuCache(OccupancyMap &map)
{
  return static_cast<GpuCache *>(map.detail()->gpu_cache);
}

void walkRegions(const OccupancyMap &map, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                 const RegionWalkFunction &on_visit)
{
  // see "A Faster Voxel Traversal Algorithm for Ray
  // Tracing" by Amanatides & Woo
  const glm::i16vec3 start_point_key = map.regionKey(start_point);
  const glm::i16vec3 end_point_key = map.regionKey(end_point);
  const glm::dvec3 start_point_local = glm::vec3(start_point - map.origin());
  const glm::dvec3 end_point_local = glm::vec3(end_point - map.origin());

  glm::dvec3 direction = glm::vec3(end_point - start_point);
  double length = glm::dot(direction, direction);

  // Very small segments which straddle a voxel boundary can be problematic. We want to avoid
  // a sqrt on a very small number, but be robust enough to handle the situation.
  // To that end, we skip normalising the direction for directions below a tenth of the voxel.
  // Then we will exit either with start/end voxels being the same, or we will step from start
  // to end in one go.
  const bool valid_length = (length >= 0.1 * map.resolution() * 0.1 * map.resolution());
  if (valid_length)
  {
    length = std::sqrt(length);
    direction *= 1.0 / length;
  }

  if (start_point_key == end_point_key)  // || !valid_length)
  {
    on_visit(start_point_key, start_point, end_point);
    return;
  }

  if (!valid_length)
  {
    // Start/end points are in different, but adjacent voxels. Prevent issues with the loop by
    // early out.
    on_visit(start_point_key, start_point, end_point);
    on_visit(end_point_key, start_point, end_point);
    return;
  }

  std::array<int, 3> step = { 0, 0, 0 };
  glm::dvec3 region;
  std::array<double, 3> time_max;
  std::array<double, 3> time_delta;
  std::array<double, 3> time_limit;
  double next_region_border;
  double direction_axis_inv;
  const glm::dvec3 region_resolution = map.regionSpatialResolution();
  glm::i16vec3 current_key = start_point_key;

  region = map.regionCentreLocal(current_key);

  // Compute step direction, increments and maximums along
  // each axis.
  for (unsigned i = 0; i < 3; ++i)
  {
    if (direction[i] != 0)
    {
      direction_axis_inv = 1.0 / direction[i];
      step[i] = (direction[i] > 0) ? 1 : -1;
      // Time delta is the ray time between voxel
      // boundaries calculated for each axis.
      time_delta[i] = region_resolution[i] * std::abs(direction_axis_inv);
      // Calculate the distance from the origin to the
      // nearest voxel edge for this axis.
      next_region_border = region[i] + double(step[i]) * 0.5 * region_resolution[i];
      time_max[i] = (next_region_border - start_point_local[i]) * direction_axis_inv;
      time_limit[i] =
        std::abs((end_point_local[i] - start_point_local[i]) * direction_axis_inv);  // +0.5f *
                                                                                     // regionResolution[i];
    }
    else
    {
      time_max[i] = time_delta[i] = std::numeric_limits<double>::max();
      time_limit[i] = 0;
    }
  }

  bool limit_reached = false;
  int axis;
  while (!limit_reached && current_key != end_point_key)
  {
    on_visit(current_key, start_point, end_point);

    if (time_max[0] < time_max[2])
    {
      axis = (time_max[0] < time_max[1]) ? 0 : 1;
    }
    else
    {
      axis = (time_max[1] < time_max[2]) ? 1 : 2;
    }

    limit_reached = std::abs(time_max[axis]) > time_limit[axis];
    current_key[axis] += step[axis];
    time_max[axis] += time_delta[axis];
  }

  // Touch the last region.
  on_visit(current_key, start_point, end_point);
}
}  // namespace gpumap

GpuMap::GpuMap(GpuMapDetail *detail, unsigned expected_element_count, size_t gpu_mem_size)
  : imp_(detail)
{
  // Map and borrowed_map already set. Temporarily cache and clear them so we don't unnecessarily release them.
  auto *map = detail->map;
  bool borrowed_map = detail->borrowed_map;
  detail->map = nullptr;
  detail->borrowed_map = false;
  setMap(map, borrowed_map, expected_element_count, gpu_mem_size, true);
}


GpuMap::GpuMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size)
  : GpuMap(new GpuMapDetail(map, borrowed_map), expected_element_count, gpu_mem_size)
{}


GpuMap::~GpuMap()
{
  GpuMap::releaseGpuProgram();
  delete imp_;
}


bool GpuMap::gpuOk() const
{
  return imp_->gpu_ok;
}


OccupancyMap &GpuMap::map()
{
  return *imp_->map;
}


const OccupancyMap &GpuMap::map() const
{
  return *imp_->map;
}


bool GpuMap::borrowedMap() const
{
  return imp_->borrowed_map;
}


void GpuMap::syncVoxels()
{
  const int sync_index = (imp_->next_buffers_index + 1) % GpuMapDetail::kBuffersCount;
  if (imp_->map)
  {
    // TODO(KS): split the logic for starting synching and waiting on completion.
    // This will allow us to kick synching off all layers in parallel and should reduce the overall latency.
    for (const auto &voxel_info : imp_->voxel_upload_info[sync_index])
    {
      if (!voxel_info.skip_cpu_sync)
      {
        gpumap::sync(*imp_->map, voxel_info.gpu_layer_id);
      }
    }
  }
  onSyncVoxels(sync_index);
}


void GpuMap::setRayFilter(const RayFilterFunction &ray_filter)
{
  imp_->ray_filter = ray_filter;
  imp_->custom_ray_filter = true;
}


const RayFilterFunction &GpuMap::rayFilter() const
{
  return imp_->ray_filter;
}


const RayFilterFunction &GpuMap::effectiveRayFilter() const
{
  return (imp_->custom_ray_filter || !imp_->map) ? imp_->ray_filter : imp_->map->rayFilter();
}


void GpuMap::clearRayFilter()
{
  imp_->ray_filter = nullptr;
  imp_->custom_ray_filter = false;
}


float GpuMap::hitValue() const
{
  return imp_->map->hitValue();
}


void GpuMap::setHitValue(float value)
{
  imp_->map->setHitValue(value);
}


float GpuMap::missValue() const
{
  return imp_->map->missValue();
}


void GpuMap::setMissValue(float value)
{
  imp_->map->setMissValue(value);
}


double GpuMap::raySegmentLength() const
{
  return imp_->ray_segment_length;
}


void GpuMap::setRaySegmentLength(double length)
{
  imp_->ray_segment_length = length;
}


bool GpuMap::groupedRays() const
{
  return imp_->group_rays;
}


size_t GpuMap::integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                             unsigned region_update_flags)
{
  return integrateRays(rays, element_count, intensities, region_update_flags, effectiveRayFilter());
}


GpuCache *GpuMap::gpuCache() const
{
  return (imp_->map) ? static_cast<GpuCache *>(imp_->map->detail()->gpu_cache) : nullptr;
}


void GpuMap::setMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size,
                    bool force_gpu_program_release)
{
  // Must ensure we aren't waiting on any GPU operations.
  if (imp_->map)
  {
    syncVoxels();

    // Delete the map if nt borrowed.
    if (!imp_->borrowed_map)
    {
      delete imp_->map;
    }
  }

  // Change the map
  imp_->map = map;
  imp_->borrowed_map = map && borrowed_map;

  if (map)
  {
    // Note: some uses of the GpuMap allow a null map pointer, but this is something of an edge case.
    GpuCache &gpu_cache = *gpumap::enableGpu(*imp_->map, gpu_mem_size, gpumap::kGpuAllowMappedBuffers);

    const unsigned prealloc_region_count = 1024u;
    for (unsigned i = 0; i < GpuMapDetail::kBuffersCount; ++i)
    {
      imp_->key_buffers[i] =
        gputil::Buffer(gpu_cache.gpu(), sizeof(GpuKey) * expected_element_count, gputil::kBfReadHost);
      imp_->ray_buffers[i] =
        gputil::Buffer(gpu_cache.gpu(), sizeof(gputil::float3) * expected_element_count, gputil::kBfReadHost);
      imp_->intensities_buffers[i] =
        gputil::Buffer(gpu_cache.gpu(), sizeof(float) * expected_element_count, gputil::kBfReadHost);
      imp_->region_key_buffers[i] =
        gputil::Buffer(gpu_cache.gpu(), sizeof(gputil::int3) * prealloc_region_count, gputil::kBfReadHost);

      // Add structures for managing uploads of regino offsets to the cache buffer.
      imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdOccupancy, gpu_cache.gpu()));
      if (imp_->support_voxel_mean && imp_->map->voxelMeanEnabled())
      {
        imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdVoxelMean, gpu_cache.gpu()));
      }
      if (imp_->support_traversal && imp_->map->traversalEnabled())
      {
        imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdTraversal, gpu_cache.gpu()));
      }
    }

    cacheGpuProgram(imp_->support_voxel_mean && imp_->map->voxelMeanEnabled(),
                    imp_->support_traversal && imp_->map->traversalEnabled(), force_gpu_program_release);
  }
}


void GpuMap::setGroupedRays(bool group)
{
  imp_->group_rays = group;
}


void GpuMap::cacheGpuProgram(bool with_voxel_mean, bool with_traversal, bool force)
{
  if (imp_->program_ref)
  {
    if (!force && with_voxel_mean == imp_->cached_sub_voxel_program)
    {
      return;
    }
  }

  releaseGpuProgram();

  GpuCache &gpu_cache = *gpuCache();
  imp_->gpu_ok = true;
  imp_->cached_sub_voxel_program = with_voxel_mean;
  if (with_traversal)
  {
    imp_->program_ref =
      (with_voxel_mean) ? &g_program_ref_sub_vox_with_traversal : &g_program_ref_no_sub_with_traversal;
  }
  else
  {
    imp_->program_ref = (with_voxel_mean) ? &g_program_ref_sub_vox : &g_program_ref_no_sub;
  }

  if (imp_->program_ref->addReference(gpu_cache.gpu()))
  {
    if (with_traversal)
    {
      imp_->update_kernel = (!with_voxel_mean) ?
                              GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), regionRayUpdateWithTraversal) :
                              GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), regionRayUpdateSubVoxWithTraversal);
    }
    else
    {
      imp_->update_kernel = (!with_voxel_mean) ?
                              GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), regionRayUpdate) :
                              GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), regionRayUpdateSubVox);
    }
    imp_->update_kernel.calculateOptimalWorkGroupSize();

    imp_->gpu_ok = imp_->update_kernel.isValid();
  }
  else
  {
    imp_->gpu_ok = false;
  }
}


void GpuMap::releaseGpuProgram()
{
  if (imp_ && imp_->update_kernel.isValid())
  {
    imp_->update_kernel = gputil::Kernel();
  }

  if (imp_ && imp_->program_ref)
  {
    imp_->program_ref->releaseReference();
    imp_->program_ref = nullptr;
  }
}


size_t GpuMap::integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                             unsigned region_update_flags, const RayFilterFunction &filter)
{
  if (!imp_->map)
  {
    return 0u;
  }

  if (!imp_->gpu_ok)
  {
    return 0u;
  }

  OccupancyMap &map = *imp_->map;
  GpuCache *gpu_cache = gpumap::enableGpu(map);

  if (!gpu_cache)
  {
    return 0u;
  }

  if (element_count == 0)
  {
    return 0u;
  }

  if (glm::length2(rays[0] - glm::dvec3(-0.19624062134997244, 0.67741350134145961, 0.92654910028562598)) < 1e-12)
  {
    int stopme = 1;
  }

  // Ensure we are using the correct GPU program. Voxel mean support may have changed.
  cacheGpuProgram(imp_->support_voxel_mean && map.voxelMeanEnabled(), imp_->support_traversal && map.traversalEnabled(),
                  false);

  // Resolve the buffer index to use. We need to support cases where buffer is already one fo the imp_->ray_buffers.
  // Check this first.
  // We still need a buffer index for event tracking.
  const int buf_idx = imp_->next_buffers_index;
  waitOnPreviousOperation(buf_idx);

  // Touch the map to update stamping.
  map.touch();

  // Get the GPU cache.
  GpuLayerCache &layer_cache = *gpu_cache->layerCache(kGcIdOccupancy);
  imp_->batch_marker = layer_cache.beginBatch();

  // Region walking function tracking which regions are
  // affected by a ray.
  const auto region_func = [this](const glm::i16vec3 &region_key, const glm::dvec3 & /*origin*/,
                                  const glm::dvec3 & /*sample*/) {
    if (imp_->regions.find(region_key) == imp_->regions.end())
    {
      imp_->regions.insert(region_key);
    }
  };

  // Declare pinned buffers for use in upload_ray delegate.
  gputil::PinnedBuffer keys_pinned;
  gputil::PinnedBuffer rays_pinned;
  gputil::PinnedBuffer intensities_pinned;

  // Build region set and upload rays.
  imp_->regions.clear();

  std::array<gputil::float3, 2> ray_gpu;
  unsigned uploaded_ray_count = 0u;
  unsigned unclipped_samples = 0u;
  GpuKey line_start_key_gpu{};
  GpuKey line_end_key_gpu{};

  const bool use_filter = bool(filter);

  // We have two loops we could run:
  // 1. processing rays as is
  // 2. grouping rays by sample voxel
  //
  // In either case we use the same code to actually upload
  // We set add_ray_upload to contain the "loop body" for both cases.
  using RayUploadFunc = std::function<void(const RayItem &)>;
  const RayUploadFunc upload_ray_core = [&](const RayItem &ray)  //
  {
    // Upload if not preloaded.
    line_start_key_gpu.region[0] = ray.origin_key.regionKey()[0];
    line_start_key_gpu.region[1] = ray.origin_key.regionKey()[1];
    line_start_key_gpu.region[2] = ray.origin_key.regionKey()[2];
    line_start_key_gpu.voxel[0] = ray.origin_key.localKey()[0];
    line_start_key_gpu.voxel[1] = ray.origin_key.localKey()[1];
    line_start_key_gpu.voxel[2] = ray.origin_key.localKey()[2];
    line_start_key_gpu.voxel[3] = 0;

    line_end_key_gpu.region[0] = ray.sample_key.regionKey()[0];
    line_end_key_gpu.region[1] = ray.sample_key.regionKey()[1];
    line_end_key_gpu.region[2] = ray.sample_key.regionKey()[2];
    line_end_key_gpu.voxel[0] = ray.sample_key.localKey()[0];
    line_end_key_gpu.voxel[1] = ray.sample_key.localKey()[1];
    line_end_key_gpu.voxel[2] = ray.sample_key.localKey()[2];
    line_end_key_gpu.voxel[3] = (ray.filter_flags & kRffClippedEnd) ? 1 : 0;

    keys_pinned.write(&line_start_key_gpu, sizeof(line_start_key_gpu), (uploaded_ray_count * 2 + 0) * sizeof(GpuKey));
    keys_pinned.write(&line_end_key_gpu, sizeof(line_end_key_gpu), (uploaded_ray_count * 2 + 1) * sizeof(GpuKey));

    // Localise the ray to single precision.
    // We change the ray coordinates to be relative to the end voxel centre. This assist later in voxel mean
    // calculations which are all relative to that voxel centre. Normally in CPU we have to make this adjustment
    // every time. We can avoid the adjustment via this logic.
    const glm::dvec3 end_voxel_centre = map.voxelCentreGlobal(ray.sample_key);
    ray_gpu[0].x = float(ray.origin.x - end_voxel_centre.x);
    ray_gpu[0].y = float(ray.origin.y - end_voxel_centre.y);
    ray_gpu[0].z = float(ray.origin.z - end_voxel_centre.z);
    ray_gpu[1].x = float(ray.sample.x - end_voxel_centre.x);
    ray_gpu[1].y = float(ray.sample.y - end_voxel_centre.y);
    ray_gpu[1].z = float(ray.sample.z - end_voxel_centre.z);
    rays_pinned.write(ray_gpu.data(), sizeof(ray_gpu), uploaded_ray_count * sizeof(ray_gpu));
    ++uploaded_ray_count;

    // Increment unclipped_samples if this sample isn't clipped.
    unclipped_samples += (ray.filter_flags & kRffClippedEnd) == 0;

    // std::cout << i / 2 << ' ' << imp_->map->voxelKey(rays[i + 0]) << " -> " << imp_->map->voxelKey(rays[i + 1]) <<
    // "
    // "
    //          << ray_start << ':' << ray_end << "  <=>  " << rays[i + 0] << " -> " << rays[i + 1] << std::endl;
    // std::cout << "dirs: " << (ray_end - ray_start) << " vs " << (ray_end_d - ray_start_d) << std::endl;
    gpumap::walkRegions(*imp_->map, ray.origin, ray.sample, region_func);
  };

  const RayUploadFunc upload_ray_with_intensity = [&](const RayItem &ray)  //
  {
    // Note: upload_count tracks the number of float3 items uploaded, so it increments by 2 each step - sensor & sample.
    // Intensities are matched one per sample, so we halve it's value here.
    intensities_pinned.write(&ray.intensity, sizeof(ray.intensity), uploaded_ray_count * sizeof(ray.intensity));
    upload_ray_core(ray);
  };

  const RayUploadFunc upload_ray = (intensities) ? upload_ray_with_intensity : upload_ray_core;
  // Reserve memory for the current ray set.
  imp_->grouped_rays.clear();
  imp_->grouped_rays.reserve(element_count / 2);

  // Take the input rays and move it into imp_->grouped_rays. In this process we do two things:
  // 1. Apply the ray filter (if any). This can change the origin and/or sample points.
  // 2. Break up long rays into multiple grouped_rays entries
  const double resolution = imp_->map->resolution();
  RayItem ray{};
  for (unsigned i = 0; i < element_count; i += 2)
  {
    ray.origin = rays[i + 0];
    ray.sample = rays[i + 1];
    ray.intensity = intensities ? intensities[i >> 1] : 0;
    ray.filter_flags = 0;

    if (use_filter)
    {
      if (!filter(&ray.origin, &ray.sample, &ray.filter_flags))
      {
        // Bad ray.
        continue;
      }
    }

    if (imp_->ray_segment_length > resolution)
    {
      // ray_length starts as a squared value.
      double ray_length = glm::length2(ray.sample - ray.origin);
      // Ensure we maintain the kRffClippedEnd for the original filtered ray.
      const unsigned last_part_clipped_end = ray.filter_flags & kRffClippedEnd;
      if (ray_length >= imp_->ray_segment_length * imp_->ray_segment_length)
      {
        // We have a long ray. Break it up into even segments according to the ray_segment_length.
        // Get a true length (not squared)
        ray_length = std::sqrt(ray_length);
        // Divide by the segment length to work out how many segments we need.
        // Round up.
        const int part_count = int(std::ceil(ray_length / imp_->ray_segment_length));
        // Work out the part length.
        const double part_length = ray_length / double(part_count);
        const glm::dvec3 dir = (ray.sample - ray.origin) / ray_length;
        // Cache the initial origin and sample points. We're about to make modifications.
        const glm::dvec3 origin = ray.origin;
        const glm::dvec3 sample = ray.sample;
        // Change ray.sample to the ray.origin to make the loop logic consistent as we transition iterations.
        ray.sample = ray.origin;
        for (int i = 1; i < part_count; ++i)
        {
          // New origin is the previous 'sample'
          ray.origin = ray.sample;
          // Vector line equation.
          ray.sample = origin + double(i) * part_length * dir;

          ray.origin_key = map.voxelKey(ray.origin);
          ray.sample_key = map.voxelKey(ray.sample);

          // Mark the ray flags to ensure we don't update the sample voxel. It will appear in the next line segment
          // again. We'll update it there.
          ray.filter_flags |= kRffClippedEnd;

          imp_->grouped_rays.emplace_back(ray);

          // Mark the origin as having been moved for the next iteration. Not really used yet.
          ray.filter_flags |= kRffClippedStart;
        }

        // We still need to add the last segment. This will not have kRffClippedStart unless the filter did this.
        ray.filter_flags &= ~kRffClippedEnd;
        ray.filter_flags |= last_part_clipped_end;
        ray.origin = ray.sample;
        ray.sample = sample;
      }
    }

    // This always adds either the full, unsegmented ray, or the last part for a segmented ray.
    ray.origin_key = map.voxelKey(ray.origin);
    ray.sample_key = map.voxelKey(ray.sample);

    imp_->grouped_rays.emplace_back(ray);
  }

  if (imp_->group_rays)
  {
    // Sort the rays. Order does not matter asside from ensuring the rays are grouped by sample voxel.
    // Despite the extra CPU work, this has proven faster for NDT update in GpuNdtMap because the GPU can do much less
    // work.
    std::sort(
      imp_->grouped_rays.begin(), imp_->grouped_rays.end(),
      [](const RayItem &a, const RayItem &b) -> bool  //
      {
        // For the comparison, we merge the region key values into a single 64-bit value
        // and the same for the local index. Then we compare the resulting values.
        // The final ordering is irrelevant in terms of which is "less". The goal is to group
        // items with the same sample key.

        const int clipped_a = !!(a.filter_flags & kRffClippedEnd);
        const int clipped_b = !!(b.filter_flags & kRffClippedEnd);

        // Multiplier/shift value to move a region key axis such that each axis is in its own bit set.
        const int64_t region_stride = 0x10000u;
        // Shift and mask together the region key axes
        const int64_t region_index_a = static_cast<int64_t>(a.sample_key.regionKey().x) +
                                       static_cast<int64_t>(region_stride * a.sample_key.regionKey().y) +
                                       region_stride * region_stride * static_cast<int64_t>(a.sample_key.regionKey().z);
        const int64_t region_index_b = static_cast<int64_t>(b.sample_key.regionKey().x) +
                                       region_stride * static_cast<int64_t>(b.sample_key.regionKey().y) +
                                       region_stride * region_stride * static_cast<int64_t>(b.sample_key.regionKey().z);
        // Multiplier/shift value to move a local key axis such that each axis is in its own bit set.
        const uint32_t local_stride = 0x100u;
        // Shift and mask together the local key axes
        const uint32_t local_index_a = uint32_t(a.sample_key.localKey().x) +
                                       local_stride * uint32_t(a.sample_key.localKey().y) +
                                       local_stride * local_stride * uint32_t(a.sample_key.localKey().z);
        const uint32_t local_index_b = uint32_t(b.sample_key.localKey().x) +
                                       local_stride * uint32_t(b.sample_key.localKey().y) +
                                       local_stride * local_stride * uint32_t(b.sample_key.localKey().z);
        // Compare the results. We sort such that:
        // - Items with unclipped end points come first.
        // - By region index next
        // - Finally by local index.
        return clipped_a < clipped_b || (clipped_a == clipped_b && region_index_a < region_index_b) ||
               (clipped_a == clipped_b && region_index_a == region_index_b && local_index_a < local_index_b);
      });
#if OHM_GPU_VERIFY_SORT
    verifySort(imp_->grouped_rays);
#endif  // OHM_GPU_VERIFY_SORT
  }

  // Reserve GPU memory for the rays.
  imp_->key_buffers[buf_idx].resize(sizeof(GpuKey) * 2 * imp_->grouped_rays.size());
  imp_->ray_buffers[buf_idx].resize(sizeof(gputil::float3) * 2 * imp_->grouped_rays.size());

  // Declare pinned buffers for use in upload_ray delegate.
  keys_pinned = gputil::PinnedBuffer(imp_->key_buffers[buf_idx], gputil::kPinWrite);
  rays_pinned = gputil::PinnedBuffer(imp_->ray_buffers[buf_idx], gputil::kPinWrite);
  if (intensities)
  {
    imp_->intensities_buffers[buf_idx].resize(sizeof(float) * imp_->grouped_rays.size());
    intensities_pinned = gputil::PinnedBuffer(imp_->intensities_buffers[buf_idx], gputil::kPinWrite);
  }

  // Upload to GPU.
  for (const RayItem &ray : imp_->grouped_rays)
  {
    upload_ray(ray);
  }

  // Asynchronous unpin. Kernels will wait on the associated event.
  keys_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->key_upload_events[buf_idx]);
  rays_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->ray_upload_events[buf_idx]);
  if (intensities)
  {
    intensities_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->ray_upload_events[buf_idx]);
  }

  imp_->ray_counts[buf_idx] = uploaded_ray_count;
  imp_->unclipped_sample_counts[buf_idx] = unclipped_samples;

  if (uploaded_ray_count == 0)
  {
    return 0u;
  }

  enqueueRegions(buf_idx, region_update_flags);

  return uploaded_ray_count * 2;
}


void GpuMap::waitOnPreviousOperation(int buffer_index)
{
  // Wait first on the event known to complete last.
  imp_->region_update_events[buffer_index].wait();
  imp_->region_update_events[buffer_index].release();

  imp_->key_upload_events[buffer_index].wait();
  imp_->key_upload_events[buffer_index].release();
  imp_->ray_upload_events[buffer_index].wait();
  imp_->ray_upload_events[buffer_index].release();

  imp_->region_key_upload_events[buffer_index].wait();
  imp_->region_key_upload_events[buffer_index].release();

  for (VoxelUploadInfo &upload_info : imp_->voxel_upload_info[buffer_index])
  {
    upload_info.offset_upload_event.wait();
    upload_info.offset_upload_event.release();
  }
}


void GpuMap::enqueueRegions(int buffer_index, unsigned region_update_flags)
{
  // For each region we need to enqueue the voxel data for that region. Within the GpuCache, each GpuLayerCache
  // manages the voxel data for a voxel layer and uploads into a single buffer for that layer returning an offset into
  // that buffer. For each (relevant) layer, we need to record the memory offset and upload corresponding event. These
  // need to be later fed to the update kernel. Size the region buffers.
  imp_->region_key_buffers[buffer_index].elementsResize<gputil::int3>(imp_->regions.size());

  for (VoxelUploadInfo &upload_info : imp_->voxel_upload_info[buffer_index])
  {
    upload_info.offsets_buffer.elementsResize<uint64_t>(imp_->regions.size());
    upload_info.offsets_buffer_pinned = gputil::PinnedBuffer(upload_info.offsets_buffer, gputil::kPinWrite);
  }

  // Execute on each region.
  gputil::PinnedBuffer regions_buffer(imp_->region_key_buffers[buffer_index], gputil::kPinWrite);

  GpuCache &gpu_cache = *this->gpuCache();
  for (const auto &region_key : imp_->regions)
  {
    const int try_limit = 2;
    for (int tries = 0; tries < try_limit; ++tries)
    {
      if (enqueueRegion(region_key, buffer_index))
      {
        // std::cout << "region: [" << regionKey.x << ' ' <<
        // regionKey.y << ' ' << regionKey.z << ']' <<
        // std::endl;
        gputil::int3 gpu_region_key = { region_key.x, region_key.y, region_key.z };
        regions_buffer.write(&gpu_region_key, sizeof(gpu_region_key),
                             imp_->region_counts[buffer_index] * sizeof(gpu_region_key));
        ++imp_->region_counts[buffer_index];
        break;
      }

      if (tries + 1 < try_limit)
      {
        // Enqueue region failed. Flush pending operations and before trying again.
        const int previous_buf_idx = buffer_index;

        regions_buffer.unpin(&gpu_cache.gpuQueue(), nullptr, &imp_->region_key_upload_events[buffer_index]);
        for (VoxelUploadInfo &upload_info : imp_->voxel_upload_info[buffer_index])
        {
          upload_info.offsets_buffer_pinned.unpin(&gpu_cache.gpuQueue(), nullptr, &upload_info.offset_upload_event);
        }
        finaliseBatch(region_update_flags);

        // Repin these buffers, but the index has changed.
        const unsigned regions_processed = imp_->region_counts[buffer_index];
        buffer_index = imp_->next_buffers_index;
        waitOnPreviousOperation(buffer_index);

        // Copy the rays buffer from the batch we just
        // finalised.
        gputil::copyBuffer(imp_->ray_buffers[buffer_index], imp_->ray_buffers[previous_buf_idx], &gpu_cache.gpuQueue(),
                           nullptr, &imp_->ray_upload_events[previous_buf_idx]);
        imp_->ray_counts[buffer_index] = imp_->ray_counts[previous_buf_idx];

        // This statement should always be true, but it would be bad to underflow.
        if (regions_processed < imp_->regions.size())
        {
          // Size the region buffers.
          imp_->region_key_buffers[buffer_index].gputil::Buffer::elementsResize<gputil::int3>(imp_->regions.size() -
                                                                                              regions_processed);

          for (VoxelUploadInfo &upload_info : imp_->voxel_upload_info[buffer_index])
          {
            upload_info.offsets_buffer.elementsResize<uint64_t>(imp_->regions.size() - regions_processed);
            upload_info.offsets_buffer_pinned = gputil::PinnedBuffer(upload_info.offsets_buffer, gputil::kPinWrite);
          }

          regions_buffer = gputil::PinnedBuffer(imp_->region_key_buffers[buffer_index], gputil::kPinWrite);
        }
      }
      else
      {
        // TODO(KS): throw with more information.
        std::cout << "Failed to enqueue region data" << std::endl;
      }
    }
  }

  regions_buffer.unpin(&gpu_cache.gpuQueue(), nullptr, &imp_->region_key_upload_events[buffer_index]);
  for (VoxelUploadInfo &upload_info : imp_->voxel_upload_info[buffer_index])
  {
    upload_info.offsets_buffer_pinned.unpin(&gpu_cache.gpuQueue(), nullptr, &upload_info.offset_upload_event);
  }

  finaliseBatch(region_update_flags);
}


bool GpuMap::enqueueRegion(const glm::i16vec3 &region_key, int buffer_index)
{
  // Upload chunk to GPU.
  MapChunk *chunk = nullptr;
  // uint64_t mem_offset;
  GpuLayerCache::CacheStatus status;

  GpuCache &gpu_cache = *this->gpuCache();

  for (VoxelUploadInfo &voxel_info : imp_->voxel_upload_info[buffer_index])
  {
    GpuLayerCache &layer_cache = *gpu_cache.layerCache(voxel_info.gpu_layer_id);
    unsigned cache_flags = 0;
    cache_flags |= voxel_info.allow_region_creation * GpuLayerCache::kAllowRegionCreate;
    cache_flags |= voxel_info.skip_cpu_sync * GpuLayerCache::kSkipDownload;
    auto mem_offset = uint64_t(layer_cache.upload(*imp_->map, region_key, chunk, &voxel_info.voxel_upload_event,
                                                  &status, imp_->batch_marker, cache_flags));

    if (status == GpuLayerCache::kCacheFull)
    {
      return false;
    }

    // std::cout << "region: [" << regionKey.x << ' ' <<
    // regionKey.y << ' ' << regionKey.z << ']' <<
    // std::endl;
    voxel_info.offsets_buffer_pinned.write(&mem_offset, sizeof(mem_offset),
                                           imp_->region_counts[buffer_index] * sizeof(mem_offset));
  }

  return true;
}


void GpuMap::finaliseBatch(unsigned region_update_flags)
{
  const int buf_idx = imp_->next_buffers_index;
  const OccupancyMapDetail *map = imp_->map->detail();

  // Complete region data upload.
  GpuCache &gpu_cache = *this->gpuCache();
  GpuLayerCache &occupancy_layer_cache = *gpu_cache.layerCache(kGcIdOccupancy);
  GpuLayerCache *mean_layer_cache = nullptr;
  GpuLayerCache *traversal_layer_cache = nullptr;

  if (imp_->support_voxel_mean && imp_->map->voxelMeanEnabled())
  {
    mean_layer_cache = gpu_cache.layerCache(kGcIdVoxelMean);
  }

  if (imp_->support_traversal && imp_->map->traversalEnabled())
  {
    traversal_layer_cache = gpu_cache.layerCache(kGcIdTraversal);
  }

  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  const unsigned region_count = imp_->region_counts[buf_idx];
  const unsigned ray_count = imp_->ray_counts[buf_idx];
  int next_upload_buffer = 0;
  gputil::Dim3 global_size(ray_count);
  gputil::Dim3 local_size(std::min<size_t>(imp_->update_kernel.optimalWorkGroupSize(), ray_count));
  gputil::EventList wait({ imp_->key_upload_events[buf_idx], imp_->ray_upload_events[buf_idx],
                           imp_->region_key_upload_events[buf_idx],
                           imp_->voxel_upload_info[buf_idx][next_upload_buffer].offset_upload_event,
                           imp_->voxel_upload_info[buf_idx][next_upload_buffer].voxel_upload_event });
  ++next_upload_buffer;

  // Add voxel mean offset upload events.
  if (mean_layer_cache)
  {
    wait.add(imp_->voxel_upload_info[buf_idx][next_upload_buffer].offset_upload_event);
    wait.add(imp_->voxel_upload_info[buf_idx][next_upload_buffer].voxel_upload_event);
    ++next_upload_buffer;
  }
  if (traversal_layer_cache)
  {
    wait.add(imp_->voxel_upload_info[buf_idx][next_upload_buffer].offset_upload_event);
    wait.add(imp_->voxel_upload_info[buf_idx][next_upload_buffer].voxel_upload_event);
    ++next_upload_buffer;
  }

  // Supporting voxel mean and traversal are putting us at the limit of what we can support using this sort of
  // conditional invocation.
  if (traversal_layer_cache)
  {
    if (mean_layer_cache)
    {
      imp_->update_kernel(global_size, local_size, wait, imp_->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                          // Kernel args begin:
                          gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                          gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][0].offsets_buffer),
                          gputil::BufferArg<VoxelMean>(*mean_layer_cache->buffer()),
                          gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][1].offsets_buffer),
                          gputil::BufferArg<float>(*traversal_layer_cache->buffer()),
                          gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][2].offsets_buffer),
                          gputil::BufferArg<gputil::int3>(imp_->region_key_buffers[buf_idx]), region_count,
                          gputil::BufferArg<GpuKey>(imp_->key_buffers[buf_idx]),
                          gputil::BufferArg<gputil::float3>(imp_->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                          float(map->resolution), map->miss_value, map->hit_value, map->occupancy_threshold_value,
                          map->min_voxel_value, map->max_voxel_value, region_update_flags);
    }
    else
    {
      imp_->update_kernel(global_size, local_size, wait, imp_->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                          // Kernel args begin:
                          gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                          gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][0].offsets_buffer),
                          gputil::BufferArg<float>(*traversal_layer_cache->buffer()),
                          gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][1].offsets_buffer),
                          gputil::BufferArg<gputil::int3>(imp_->region_key_buffers[buf_idx]), region_count,
                          gputil::BufferArg<GpuKey>(imp_->key_buffers[buf_idx]),
                          gputil::BufferArg<gputil::float3>(imp_->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                          float(map->resolution), map->miss_value, map->hit_value, map->occupancy_threshold_value,
                          map->min_voxel_value, map->max_voxel_value, region_update_flags);
    }
  }
  else
  {
    {
      if (mean_layer_cache)
      {
        imp_->update_kernel(global_size, local_size, wait, imp_->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                            // Kernel args begin:
                            gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                            gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][0].offsets_buffer),
                            gputil::BufferArg<VoxelMean>(*mean_layer_cache->buffer()),
                            gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][1].offsets_buffer),
                            gputil::BufferArg<gputil::int3>(imp_->region_key_buffers[buf_idx]), region_count,
                            gputil::BufferArg<GpuKey>(imp_->key_buffers[buf_idx]),
                            gputil::BufferArg<gputil::float3>(imp_->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                            float(map->resolution), map->miss_value, map->hit_value, map->occupancy_threshold_value,
                            map->min_voxel_value, map->max_voxel_value, region_update_flags);
      }
      else
      {
        imp_->update_kernel(global_size, local_size, wait, imp_->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                            // Kernel args begin:
                            gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                            gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][0].offsets_buffer),
                            gputil::BufferArg<gputil::int3>(imp_->region_key_buffers[buf_idx]), region_count,
                            gputil::BufferArg<GpuKey>(imp_->key_buffers[buf_idx]),
                            gputil::BufferArg<gputil::float3>(imp_->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                            float(map->resolution), map->miss_value, map->hit_value, map->occupancy_threshold_value,
                            map->min_voxel_value, map->max_voxel_value, region_update_flags);
      }
    }
  }

  // Update most recent chunk GPU event.
  occupancy_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
  if (mean_layer_cache)
  {
    mean_layer_cache->updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
  }
  if (traversal_layer_cache)
  {
    traversal_layer_cache->updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
  }

  // std::cout << imp_->region_counts[bufIdx] << "
  // regions\n" << std::flush;

  imp_->region_counts[buf_idx] = 0;
  // Start a new batch for the GPU layers.
  imp_->batch_marker = occupancy_layer_cache.beginBatch();
  if (mean_layer_cache)
  {
    mean_layer_cache->beginBatch(imp_->batch_marker);
  }
  if (traversal_layer_cache)
  {
    traversal_layer_cache->beginBatch(imp_->batch_marker);
  }
  imp_->next_buffers_index = (imp_->next_buffers_index + 1) % GpuMapDetail::kBuffersCount;
}
}  // namespace ohm
