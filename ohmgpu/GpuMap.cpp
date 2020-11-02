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

#include <cassert>
#include <cmath>
#include <functional>
#include <initializer_list>
#include <iostream>

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#include "RegionUpdateResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdate);
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdateSubVox);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

using namespace ohm;

namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref_sub_vox("RegionUpdate", GpuProgramRef::kSourceString, RegionUpdateCode,  // NOLINT
                                    RegionUpdateCode_length, { "-DVOXEL_MEAN" });
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref_sub_vox("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u,
                                    { "-DVOXEL_MEAN" });
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref_no_sub("RegionUpdate", GpuProgramRef::kSourceString, RegionUpdateCode,  // NOLINT
                                   RegionUpdateCode_length);
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref_no_sub("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u);
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

  using RegionWalkFunction = std::function<void(const glm::i16vec3 &, const glm::dvec3 &, const glm::dvec3 &)>;

  void walkRegions(const OccupancyMap &map, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                   const RegionWalkFunction &func)
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
      func(start_point_key, start_point, end_point);
      return;
    }

    if (!valid_length)
    {
      // Start/end points are in different, but adjacent voxels. Prevent issues with the loop by
      // early out.
      func(start_point_key, start_point, end_point);
      func(end_point_key, start_point, end_point);
      return;
    }

    int step[3] = { 0 };
    glm::dvec3 region;
    double time_max[3];
    double time_delta[3];
    double time_limit[3];
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
        next_region_border = region[i] + step[i] * 0.5f * region_resolution[i];
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
      func(current_key, start_point, end_point);

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
    func(current_key, start_point, end_point);
  }

  inline bool goodRay(const glm::dvec3 &start, const glm::dvec3 &end, double max_range = 500.0)
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
}  // namespace


GpuCache *ohm::gpumap::enableGpu(OccupancyMap &map)
{
  return enableGpu(map, GpuCache::kDefaultTargetMemSize, kGpuAllowMappedBuffers);
}


GpuCache *ohm::gpumap::enableGpu(OccupancyMap &map, size_t target_gpu_mem_size, unsigned flags)
{
  OccupancyMapDetail &map_imp = *map.detail();
  if (map_imp.gpu_cache)
  {
    return static_cast<GpuCache *>(map_imp.gpu_cache);
  }

  initialiseGpuCache(map, target_gpu_mem_size, flags);
  return static_cast<GpuCache *>(map_imp.gpu_cache);
}


void ohm::gpumap::sync(OccupancyMap &map)
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


void ohm::gpumap::sync(OccupancyMap &map, unsigned layer_index)
{
  if (GpuCache *cache = gpuCache(map))
  {
    if (GpuLayerCache *layer = cache->layerCache(layer_index))
    {
      layer->syncToMainMemory();
    }
  }
}


GpuCache *ohm::gpumap::gpuCache(OccupancyMap &map)
{
  return static_cast<GpuCache *>(map.detail()->gpu_cache);
}


GpuMap::GpuMap(GpuMapDetail *detail, unsigned expected_element_count, size_t gpu_mem_size)
  : imp_(detail)
{
  GpuCache &gpu_cache = *gpumap::enableGpu(*imp_->map, gpu_mem_size, gpumap::kGpuAllowMappedBuffers);

  const unsigned prealloc_region_count = 1024u;
  for (unsigned i = 0; i < GpuMapDetail::kBuffersCount; ++i)
  {
    imp_->key_buffers[i] =
      gputil::Buffer(gpu_cache.gpu(), sizeof(GpuKey) * expected_element_count, gputil::kBfReadHost);
    imp_->ray_buffers[i] =
      gputil::Buffer(gpu_cache.gpu(), sizeof(gputil::float3) * expected_element_count, gputil::kBfReadHost);
    imp_->region_key_buffers[i] =
      gputil::Buffer(gpu_cache.gpu(), sizeof(gputil::int3) * prealloc_region_count, gputil::kBfReadHost);

    // Add structures for managing uploads of regino offsets to the cache buffer.
    imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdOccupancy, gpu_cache.gpu()));
    if (imp_->map->voxelMeanEnabled())
    {
      imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdVoxelMean, gpu_cache.gpu()));
    }
  }

  cacheGpuProgram(imp_->map->voxelMeanEnabled(), true);
}


GpuMap::GpuMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size)
  : GpuMap(new GpuMapDetail(map, borrowed_map), expected_element_count, gpu_mem_size)
{}


GpuMap::~GpuMap()
{
  releaseGpuProgram();
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
  if (imp_->map)
  {
    // TODO: (KS) split the logic for starting synching and waiting on completion.
    // This will allow us to kick synching off all layers in parallel and should reduce the overall latency.
    gpumap::sync(*imp_->map, kGcIdOccupancy);
    gpumap::sync(*imp_->map, kGcIdVoxelMean);
    gpumap::sync(*imp_->map, kGcIdCovariance);
  }
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


bool GpuMap::groupedRays() const
{
  return imp_->group_rays;
}


size_t GpuMap::integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                             unsigned region_update_flags)
{
  return integrateRays(rays, element_count, intensities, region_update_flags, effectiveRayFilter());
}


void GpuMap::applyClearingPattern(const glm::dvec3 *rays, size_t element_count)
{
  // Only apply the good ray filter.
  const auto clearing_ray_filter = [](glm::dvec3 *start, glm::dvec3 *end, unsigned *filter_flags)  //
  {                                                                                                //
    return goodRayFilter(start, end, filter_flags, 1e4);
  };
  const unsigned flags = kRfEndPointAsFree | kRfStopOnFirstOccupied | kRfClearOnly;
  integrateRays(rays, element_count, nullptr, flags, clearing_ray_filter);
}


void GpuMap::applyClearingPattern(const glm::dvec3 &apex, const glm::dvec3 &cone_axis, double cone_angle, double range,
                                  double angular_resolution)
{
  // Build a set of rays to process from the cone definition.
  if (angular_resolution <= 0)
  {
    // Set the default angular resolution.
    angular_resolution = glm::radians(2.0);
  }

  // Ensure cone_axis is normalised.
  const glm::dvec3 cone_normal = glm::normalize(cone_axis);

  // First setup an axis perpendicular to the cone_axis in order to be able to walk the circle. For this we will
  // just swizzle the cone_axis components.
  const glm::dvec3 deflection_base = glm::dvec3(cone_normal.z, cone_normal.x, cone_normal.y);

  // Build the ray set. Here we walk start with the deflection_base, and gradually rotate it around cone_axis at the
  // requested angular_resolution. As we rotate around cone_axis, we create rays which are deviate from cone_axis
  // by an ever increasing amount along deflection_base to the requested angular_resolution.
  std::vector<glm::dvec3> rays;
  // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter)
  for (double circle_angle = 0; circle_angle < 2.0 * M_PI; circle_angle += angular_resolution)
  {
    // Rotate deflection_base around cone_axis by the circle_angle.
    const glm::dquat circle_rotation = glm::angleAxis(circle_angle, cone_normal);
    const glm::dvec3 deflection_axis = circle_rotation * deflection_base;

    // Now deflect by deflection axis at an ever increasing angle.
    // NOLINTNEXTLINE(clang-analyzer-security.FloatLoopCounter)
    for (double deflection_angle = 0; deflection_angle <= cone_angle; deflection_angle += angular_resolution)
    {
      const glm::dquat deflection = glm::angleAxis(deflection_angle, deflection_axis);
      const glm::dvec3 ray = deflection * cone_normal;
      const glm::dvec3 end_point = apex + ray * range;
      rays.push_back(apex);
      rays.push_back(end_point);
    }
  }

  // Now apply these rays.
  applyClearingPattern(rays.data(), unsigned(rays.size()));
}


GpuCache *GpuMap::gpuCache() const
{
  return static_cast<GpuCache *>(imp_->map->detail()->gpu_cache);
}


void GpuMap::setGroupedRays(bool group)
{
  imp_->group_rays = group;
}


void GpuMap::cacheGpuProgram(bool with_voxel_mean, bool force)
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
  imp_->program_ref = (with_voxel_mean) ? &program_ref_sub_vox : &program_ref_no_sub;

  if (imp_->program_ref->addReference(gpu_cache.gpu()))
  {
    imp_->update_kernel = (!with_voxel_mean) ? GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), regionRayUpdate) :
                                               GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), regionRayUpdateSubVox);
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


size_t GpuMap::integrateRays(const glm::dvec3 *rays, size_t element_count, const float * /*intensities*/,
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

  // Ensure we are using the correct GPU program. Voxel mean support may have changed.
  cacheGpuProgram(map.voxelMeanEnabled(), false);

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

  // Reserve GPU memory for the rays.
  imp_->key_buffers[buf_idx].resize(sizeof(GpuKey) * element_count);
  imp_->ray_buffers[buf_idx].resize(sizeof(gputil::float3) * element_count);

  gputil::PinnedBuffer keys_pinned(imp_->key_buffers[buf_idx], gputil::kPinWrite);
  gputil::PinnedBuffer rays_pinned(imp_->ray_buffers[buf_idx], gputil::kPinWrite);

  // Build region set and upload rays.
  imp_->regions.clear();

  gputil::float3 ray_gpu[2];
  unsigned upload_count = 0u;
  GpuKey line_start_key_gpu{}, line_end_key_gpu{};

  const bool use_filter = bool(filter);

  // We have two loops we could run:
  // 1. processing rays as is
  // 2. grouping rays by sample voxel
  //
  // In either case we use the same code to actually upload
  // We set add_ray_upload to contain the "loop body" for both cases.
  const auto upload_ray = [&](const RayItem &ray)  //
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

    keys_pinned.write(&line_start_key_gpu, sizeof(line_start_key_gpu), (upload_count + 0) * sizeof(GpuKey));
    keys_pinned.write(&line_end_key_gpu, sizeof(line_end_key_gpu), (upload_count + 1) * sizeof(GpuKey));

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
    rays_pinned.write(ray_gpu, sizeof(ray_gpu), upload_count * sizeof(gputil::float3));
    upload_count += 2;

    // std::cout << i / 2 << ' ' << imp_->map->voxelKey(rays[i + 0]) << " -> " << imp_->map->voxelKey(rays[i + 1]) << "
    // "
    //          << ray_start << ':' << ray_end << "  <=>  " << rays[i + 0] << " -> " << rays[i + 1] << std::endl;
    // std::cout << "dirs: " << (ray_end - ray_start) << " vs " << (ray_end_d - ray_start_d) << std::endl;
    walkRegions(*imp_->map, ray.origin, ray.sample, region_func);
  };

  RayItem ray{};
  if (!imp_->group_rays)
  {
    // Not grouping rays. Simple upload as is. For each ray, call upload_ray .
    for (unsigned i = 0; i < element_count; i += 2)
    {
      ray.origin = rays[i + 0];
      ray.sample = rays[i + 1];
      ray.filter_flags = 0;

      if (use_filter)
      {
        if (!filter(&ray.origin, &ray.sample, &ray.filter_flags))
        {
          // Bad ray.
          continue;
        }
      }

      ray.origin_key = map.voxelKey(ray.origin);
      ray.sample_key = map.voxelKey(ray.sample);
      upload_ray(ray);
    }
  }
  else
  {
    // Grouped ray upload. We must first sort rays by sample voxel.
    // Despite the extra CPU work, this has proven faster for NDT update in GpuNdtMap because the GPU can do much less
    // work.

    // Reserve memory for the current ray set.
    imp_->grouped_rays.clear();
    imp_->grouped_rays.reserve(element_count / 2);

    // Populate the sorting structure.
    for (unsigned i = 0; i < element_count; i += 2)
    {
      ray.origin = rays[i + 0];
      ray.sample = rays[i + 1];
      ray.filter_flags = 0;

      if (use_filter)
      {
        if (!filter(&ray.origin, &ray.sample, &ray.filter_flags))
        {
          // Bad ray.
          continue;
        }
      }

      ray.origin_key = map.voxelKey(ray.origin);
      ray.sample_key = map.voxelKey(ray.sample);
      imp_->grouped_rays.emplace_back(ray);
    }

    /// Sort the rays. Order does not matter asside from ensuring the rays are grouped by sample voxel.
    std::sort(imp_->grouped_rays.begin(), imp_->grouped_rays.end(),
              [](const RayItem &a, const RayItem &b) -> bool  //
              {
                // For the comparison, we merge the region key values into a single 64-bit value
                // and the same for the local index. Then we compare the resulting values.
                // The final ordering is irrelevant in terms of which is "less". The goal is to group
                // items with the same sample key.

                // Multiplier/shift value to move a region key axis such that each axis is in its own bit set.
                const int64_t region_stride = 0x10000u;
                // Shift and mask together the region key axes
                const int64_t region_index_a = (int64_t)a.sample_key.regionKey().x +
                                               (int64_t)region_stride * a.sample_key.regionKey().y +
                                               region_stride * region_stride * (int64_t)a.sample_key.regionKey().z;
                const int64_t region_index_b = (int64_t)b.sample_key.regionKey().x +
                                               region_stride * (int64_t)b.sample_key.regionKey().y +
                                               region_stride * region_stride * (int64_t)b.sample_key.regionKey().z;
                // Multiplier/shift value to move a local key axis such that each axis is in its own bit set.
                const uint32_t local_stride = 0x100u;
                // Shift and mask together the local key axes
                const uint32_t local_index_a = uint32_t(a.sample_key.localKey().x) +
                                               local_stride * uint32_t(a.sample_key.localKey().y) +
                                               local_stride * local_stride * uint32_t(a.sample_key.localKey().z);
                const uint32_t local_index_b = uint32_t(b.sample_key.localKey().x) +
                                               local_stride * uint32_t(b.sample_key.localKey().y) +
                                               local_stride * local_stride * uint32_t(b.sample_key.localKey().z);
                // Compare the results.
                return region_index_a < region_index_b ||
                       region_index_a == region_index_b && local_index_a < local_index_b;
              });

    // Upload to GPU.
    for (const RayItem &ray : imp_->grouped_rays)
    {
      upload_ray(ray);
    }
  }

  // Asynchronous unpin. Kernels will wait on the associated event.
  keys_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->key_upload_events[buf_idx]);
  rays_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->ray_upload_events[buf_idx]);

  imp_->ray_counts[buf_idx] = unsigned(upload_count / 2);

  if (upload_count == 0)
  {
    return 0u;
  }

  enqueueRegions(buf_idx, region_update_flags);

  return upload_count;
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
  // For each region we need to enqueue the voxel data for that region. Within the GpuCache, each GpuLayerCache manages
  // the voxel data for a voxel layer and uploads into a single buffer for that layer returning an offset into that
  // buffer. For each (relevant) layer, we need to record the memory offset and upload corresponding event.
  // These need to be later fed to the update kernel.
  // Size the region buffers.
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
      else if (tries + 1 < try_limit)
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

          regions_buffer = gputil::PinnedBuffer(imp_->region_key_buffers[buffer_index], gputil::kPinRead);
        }
      }
      else
      {
        // TODO: throw with more information.
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
    uint64_t mem_offset = uint64_t(layer_cache.upload(*imp_->map, region_key, chunk, &voxel_info.voxel_upload_event,
                                                      &status, imp_->batch_marker, GpuLayerCache::kAllowRegionCreate));

    if (status == GpuLayerCache::kCacheFull)
    {
      return false;
    }

    // std::cout << "region: [" << regionKey.x << ' ' <<
    // regionKey.y << ' ' << regionKey.z << ']' <<
    // std::endl;
    voxel_info.offsets_buffer_pinned.write(&mem_offset, sizeof(mem_offset),
                                           imp_->region_counts[buffer_index] * sizeof(mem_offset));

    // Mark the region as dirty.
    chunk->dirty_stamp = chunk->touched_stamps[layer_cache.layerIndex()] = imp_->map->stamp();
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

  if (imp_->map->voxelMeanEnabled())
  {
    mean_layer_cache = gpu_cache.layerCache(kGcIdVoxelMean);
  }

  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  const unsigned region_count = imp_->region_counts[buf_idx];
  const unsigned ray_count = imp_->ray_counts[buf_idx];
  gputil::Dim3 global_size(ray_count);
  gputil::Dim3 local_size(std::min<size_t>(imp_->update_kernel.optimalWorkGroupSize(), ray_count));
  gputil::EventList wait({ imp_->key_upload_events[buf_idx], imp_->ray_upload_events[buf_idx],
                           imp_->region_key_upload_events[buf_idx],
                           imp_->voxel_upload_info[buf_idx][0].offset_upload_event,
                           imp_->voxel_upload_info[buf_idx][0].voxel_upload_event });

  // Add voxel mean offset upload events.
  if (mean_layer_cache)
  {
    wait.add(imp_->voxel_upload_info[buf_idx][1].offset_upload_event);
    wait.add(imp_->voxel_upload_info[buf_idx][1].voxel_upload_event);
  }

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

  // gpu_cache.gpuQueue().flush();

  // Update most recent chunk GPU event.
  occupancy_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
  if (mean_layer_cache)
  {
    mean_layer_cache->updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
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
  imp_->next_buffers_index = 1 - imp_->next_buffers_index;
}
