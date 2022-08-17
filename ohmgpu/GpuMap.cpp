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
#include "RayItem.h"

#include <ohm/Aabb.h>
#include <ohm/DefaultLayer.h>
#include <ohm/MapChunk.h>
#include <ohm/MapRegion.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/RayFilter.h>
#include <ohm/VoxelMean.h>
#include <ohm/VoxelTouchTime.h>

#include <ohmutil/GlmStream.h>

#include "private/GpuMapDetail.h"
#include "private/GpuProgramRef.h"

#include <ohm/private/OccupancyMapDetail.h>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuProgram.h>

#include <logutil/Logger.h>

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
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdateOccupancy);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

namespace ohm
{
namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref("RegionUpdate", GpuProgramRef::kSourceString, RegionUpdateCode,  // NOLINT
                            RegionUpdateCode_length);
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl");  // NOLINT
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

const double kDefaultMaxRayRange = 1000.0;

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


void GpuMap::syncVoxels(const std::vector<int> &layer_indices)
{
  // Sync layers specified in layer_indices. Supports map copy functions.
  const int sync_index = (imp_->next_buffers_index + 1) % GpuMapDetail::kBuffersCount;
  auto *cache = gpuCache();
  if (imp_->map)
  {
    for (const auto &voxel_info : imp_->voxel_upload_info[sync_index])
    {
      const auto *layer_cache = cache->layerCache(voxel_info.gpu_layer_id);
      if (layer_cache && !voxel_info.skip_cpu_sync &&
          std::find(layer_indices.begin(), layer_indices.end(), int(layer_cache->layerIndex())) != layer_indices.end())
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
                             const double *timestamps, unsigned region_update_flags)
{
  return integrateRays(rays, element_count, intensities, timestamps, region_update_flags, effectiveRayFilter());
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
      if (imp_->use_original_ray_buffers)
      {
        imp_->original_ray_buffers[i] =
          gputil::Buffer(gpu_cache.gpu(), sizeof(gputil::float3) * expected_element_count, gputil::kBfReadHost);
      }
      imp_->intensities_buffers[i] =
        gputil::Buffer(gpu_cache.gpu(), sizeof(float) * expected_element_count, gputil::kBfReadHost);
      imp_->timestamps_buffers[i] =
        gputil::Buffer(gpu_cache.gpu(), sizeof(uint32_t) * expected_element_count, gputil::kBfReadHost);
      imp_->region_key_buffers[i] =
        gputil::Buffer(gpu_cache.gpu(), sizeof(gputil::int3) * prealloc_region_count, gputil::kBfReadHost);

      // Add structures for managing uploads of regino offsets to the cache buffer.
      imp_->occupancy_uidx = int(imp_->voxel_upload_info[i].size());  // Set twice to the same value, but that's ok.
      imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdOccupancy, gpu_cache.gpu()));
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

  // Ensure voxel mean VoxelUploadInfo is present, or removed
  imp_->mean_uidx = enableVoxelUpload(int(kGcIdVoxelMean), with_voxel_mean);
  imp_->traversal_uidx = enableVoxelUpload(int(kGcIdTraversal), with_traversal);
  imp_->touch_time_uidx = enableVoxelUpload(int(kGcIdTouchTime), imp_->map->touchTimeEnabled());
  imp_->incident_normal_uidx = enableVoxelUpload(int(kGcIdIncidentNormal), imp_->map->incidentNormalEnabled());

  GpuCache &gpu_cache = *gpuCache();
  imp_->gpu_ok = true;
  imp_->cached_sub_voxel_program = with_voxel_mean;
  imp_->program_ref = &g_program_ref;

  if (imp_->program_ref->addReference(gpu_cache.gpu()))
  {
    imp_->update_kernel = GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), regionRayUpdateOccupancy);
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
                             const double *timestamps, unsigned region_update_flags, const RayFilterFunction &filter)
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

  // Drop intensity and timestamps if we do not have the map layers to support it. This saves on uploading unnecesary
  // GPU data.
  if (map.layout().intensityLayer() < 0)
  {
    intensities = nullptr;
  }
  if (map.layout().layerIndex(default_layer::touchTimeLayerName()) < 0)
  {
    timestamps = nullptr;
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

  double timebase = 0;
  if (timestamps)
  {
    timebase = map.updateFirstRayTime(*timestamps);
    region_update_flags |= kRfInternalTimestamps;
  }

  // Get the GPU cache.
  GpuLayerCache *layer_cache = gpu_cache->layerCache(kGcIdOccupancy);
  if (!layer_cache)
  {
    layer_cache = gpu_cache->layerCache(kGcIdTsdf);
  }
  if (!layer_cache)
  {
    logutil::error("GpuMap cannot resolve occupancy or TSDF layer.\n");
    return 0u;
  }
  imp_->batch_marker = layer_cache->beginBatch();

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
  gputil::PinnedBuffer original_rays_pinned;
  gputil::PinnedBuffer intensities_pinned;
  gputil::PinnedBuffer timestamps_pinned;

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

    if (imp_->use_original_ray_buffers)
    {
      // Upload the original sample, made relative to this ray's end voxel.
      ray_gpu[0].x = float(ray.original_origin.x - end_voxel_centre.x);
      ray_gpu[0].y = float(ray.original_origin.y - end_voxel_centre.y);
      ray_gpu[0].z = float(ray.original_origin.z - end_voxel_centre.z);
      ray_gpu[1].x = float(ray.original_sample.x - end_voxel_centre.x);
      ray_gpu[1].y = float(ray.original_sample.y - end_voxel_centre.y);
      ray_gpu[1].z = float(ray.original_sample.z - end_voxel_centre.z);
      // Note: we are only uploading the first item from ray_gpu - the original sample position.
      original_rays_pinned.write(ray_gpu.data(), sizeof(ray_gpu), uploaded_ray_count * sizeof(ray_gpu));
    }

    ++uploaded_ray_count;

    // Increment unclipped_samples if this sample isn't clipped.
    unclipped_samples += (ray.filter_flags & kRffClippedEnd) == 0;

    // logutil::trace(i / 2, ' ', imp_->map->voxelKey(rays[i + 0]), " -> ", imp_->map->voxelKey(rays[i + 1]), ' ',
    //                 ray_start, ':', ray_end, "  <=>  ", rays[i + 0], " -> ", rays[i + 1], '\n');
    // logutil::trace("dirs: ", (ray_end - ray_start), " vs ", (ray_end_d - ray_start_d), '\n');
    gpumap::walkRegions(*imp_->map, ray.origin, ray.sample, region_func);
  };

  const RayUploadFunc upload_ray_with_intensity_or_timestamp = [&](const RayItem &ray)  //
  {
    // Note: upload_count tracks the number of float3 items uploaded, so it increments by 2 each step - sensor & sample.
    // Intensities are matched one per sample, so we halve it's value here.
    if (intensities)
    {
      intensities_pinned.write(&ray.intensity, sizeof(ray.intensity), uploaded_ray_count * sizeof(ray.intensity));
    }
    if (timestamps)
    {
      timestamps_pinned.write(&ray.timestamp, sizeof(ray.timestamp), uploaded_ray_count * sizeof(ray.timestamp));
    }
    upload_ray_core(ray);
  };

  const RayUploadFunc upload_ray =
    (intensities || timestamps) ? upload_ray_with_intensity_or_timestamp : upload_ray_core;
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
    ray.original_origin = ray.origin = rays[i + 0];
    ray.original_sample = ray.sample = rays[i + 1];
    ray.intensity = (intensities) ? intensities[i >> 1] : 0;
    ray.timestamp = (timestamps) ? encodeVoxelTouchTime(timebase, timestamps[i >> 1]) : 0;
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
    std::sort(imp_->grouped_rays.begin(), imp_->grouped_rays.end());
#if OHM_GPU_VERIFY_SORT
    verifySort(imp_->grouped_rays);
#endif  // OHM_GPU_VERIFY_SORT
  }

  // Reserve GPU memory for the rays.
  imp_->key_buffers[buf_idx].resize(sizeof(GpuKey) * 2 * imp_->grouped_rays.size());
  imp_->ray_buffers[buf_idx].resize(sizeof(gputil::float3) * 2 * imp_->grouped_rays.size());

  if (imp_->use_original_ray_buffers)
  {
    imp_->original_ray_buffers[buf_idx].resize(sizeof(gputil::float3) * 2 * imp_->grouped_rays.size());
  }

  // Declare pinned buffers for use in upload_ray delegate.
  keys_pinned = gputil::PinnedBuffer(imp_->key_buffers[buf_idx], gputil::kPinWrite);
  rays_pinned = gputil::PinnedBuffer(imp_->ray_buffers[buf_idx], gputil::kPinWrite);
  if (imp_->use_original_ray_buffers)
  {
    original_rays_pinned = gputil::PinnedBuffer(imp_->original_ray_buffers[buf_idx], gputil::kPinWrite);
  }
  if (intensities)
  {
    imp_->intensities_buffers[buf_idx].resize(sizeof(float) * imp_->grouped_rays.size());
    intensities_pinned = gputil::PinnedBuffer(imp_->intensities_buffers[buf_idx], gputil::kPinWrite);
  }
  if (timestamps)
  {
    imp_->timestamps_buffers[buf_idx].resize(sizeof(uint32_t) * imp_->grouped_rays.size());
    timestamps_pinned = gputil::PinnedBuffer(imp_->timestamps_buffers[buf_idx], gputil::kPinWrite);
  }

  // Upload to GPU.
  for (const RayItem &ray : imp_->grouped_rays)
  {
    upload_ray(ray);
  }

  // Asynchronous unpin. Kernels will wait on the associated event.
  keys_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->key_upload_events[buf_idx]);
  rays_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->ray_upload_events[buf_idx]);
  if (imp_->use_original_ray_buffers)
  {
    original_rays_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->original_ray_upload_events[buf_idx]);
  }
  if (intensities)
  {
    intensities_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->intensities_upload_events[buf_idx]);
  }
  if (timestamps)
  {
    timestamps_pinned.unpin(&gpu_cache->gpuQueue(), nullptr, &imp_->timestamps_upload_events[buf_idx]);
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
  // need to be later fed to the update kernel.

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
        // logutil::trace("region: [", region_key.x, ' ', region_key.y, ' ', region_key.z, "]\n");
        gputil::int3 gpu_region_key = { region_key.x, region_key.y, region_key.z };
        regions_buffer.write(&gpu_region_key, sizeof(gpu_region_key),
                             imp_->region_counts[buffer_index] * sizeof(gpu_region_key));
        ++imp_->region_counts[buffer_index];
        break;  // Break the tries loop into the outer loop.
      }

      if (tries + 1 < try_limit)
      {
        // Enqueue region failed. Flush pending all pending operations and before trying again in the current buffer.
        regions_buffer.unpin(&gpu_cache.gpuQueue(), nullptr, &imp_->region_key_upload_events[buffer_index]);
        for (VoxelUploadInfo &upload_info : imp_->voxel_upload_info[buffer_index])
        {
          upload_info.offsets_buffer_pinned.unpin(&gpu_cache.gpuQueue(), nullptr, &upload_info.offset_upload_event);
        }

        // We may have failed to enqueue anything. Only finalise if there's something to do.
        if (imp_->region_counts[buffer_index])
        {
          finaliseBatch(region_update_flags);
        }

        // Because we are reusing the same rays buffer, we need to cache the following two events and restore them
        // after waitOnPreviousOperation(), which would release them.
        gputil::Event key_upload_events = imp_->key_upload_events[buffer_index];
        gputil::Event ray_upload_events = imp_->ray_upload_events[buffer_index];

        // Since our cache memory is full, we should wait for all operations to complete as we may need a clear cache.
        // We re-use the same buffer_index to avoid copying ray data.
        for (int i = 0; i < int(GpuMapDetail::kBuffersCount); ++i)
        {
          waitOnPreviousOperation(i);
        }

        // Use the same buffer.
        // FIXME(KS): this is programming by side-effects. Should probably pass the buffer index to finaliseBatch()
        // instead of letting it read imp_->next_buffers_index.
        imp_->next_buffers_index = buffer_index;
        imp_->key_upload_events[buffer_index] = key_upload_events;
        imp_->ray_upload_events[buffer_index] = ray_upload_events;

        // Re-pin buffers.
        regions_buffer.pin();
        for (VoxelUploadInfo &upload_info : imp_->voxel_upload_info[buffer_index])
        {
          upload_info.offsets_buffer_pinned.pin();
        }
      }
      else
      {
        // TODO(KS): throw with more information.
        logutil::error("Failed to enqueue region data\n");
      }
    }
  }

  regions_buffer.unpin(&gpu_cache.gpuQueue(), nullptr, &imp_->region_key_upload_events[buffer_index]);
  for (VoxelUploadInfo &upload_info : imp_->voxel_upload_info[buffer_index])
  {
    upload_info.offsets_buffer_pinned.unpin(&gpu_cache.gpuQueue(), nullptr, &upload_info.offset_upload_event);
  }

  if (!imp_->region_key_upload_events[buffer_index].isValid())
  {
    logutil::error("Invalid region key upload event\n");
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
      logutil::warn("GpuLayerCache full: [", layer_cache.layerIndex(), "] ",
                    (layer_cache.layerIndex() < imp_->map->layout().layerCount()) ?
                      imp_->map->layout().layer(layer_cache.layerIndex()).name() :
                      "<out-of-range>",
                    "\n");
      return false;
    }

    // logutil::trace("region: [", region_key.x, ' ', region_key.y, ' ', region_key.z, "]\n");
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
  GpuLayerCache *touch_times_layer_cache = nullptr;
  GpuLayerCache *incidents_layer_cache = nullptr;

  const int occ_uidx = imp_->occupancy_uidx;
  const int mean_uidx = imp_->mean_uidx;
  const int traversal_uidx = imp_->traversal_uidx;
  const int touch_time_uidx = imp_->touch_time_uidx;
  const int incident_normal_uidx = imp_->incident_normal_uidx;

  if (imp_->support_voxel_mean && imp_->map->voxelMeanEnabled())
  {
    mean_layer_cache = gpu_cache.layerCache(kGcIdVoxelMean);
  }

  if (imp_->support_traversal && imp_->map->traversalEnabled())
  {
    traversal_layer_cache = gpu_cache.layerCache(kGcIdTraversal);
  }

  if (imp_->map->touchTimeEnabled())
  {
    touch_times_layer_cache = gpu_cache.layerCache(kGcIdTouchTime);
  }

  if (imp_->map->incidentNormalEnabled())
  {
    incidents_layer_cache = gpu_cache.layerCache(kGcIdIncidentNormal);
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

  // Add supplemental buffer upload events
  // Note: we do not wait on original_ray_upload_events even if imp_->use_original_ray_buffers because we do not use the
  // original_ray_buffers in this algorithm. Ones that do should add it to the wait list here.
  if (imp_->intensities_upload_events[buf_idx].isValid())
  {
    wait.add(imp_->intensities_upload_events[buf_idx]);
  }
  if (imp_->timestamps_upload_events[buf_idx].isValid())
  {
    wait.add(imp_->timestamps_upload_events[buf_idx]);
  }

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
  if (touch_times_layer_cache)
  {
    wait.add(imp_->voxel_upload_info[buf_idx][next_upload_buffer].offset_upload_event);
    wait.add(imp_->voxel_upload_info[buf_idx][next_upload_buffer].voxel_upload_event);
    ++next_upload_buffer;
  }
  if (incidents_layer_cache)
  {
    wait.add(imp_->voxel_upload_info[buf_idx][next_upload_buffer].offset_upload_event);
    wait.add(imp_->voxel_upload_info[buf_idx][next_upload_buffer].voxel_upload_event);
    ++next_upload_buffer;
  }

  // Supporting voxel mean and traversal are putting us at the limit of what we can support using this sort of
  // conditional invocation.
  imp_->update_kernel(
    global_size, local_size, wait, imp_->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
    // Kernel args begin:
    // Occupancy voxels and offsets.
    gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
    gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][occ_uidx].offsets_buffer),
    // Mean voxels and offsets.
    gputil::BufferArg<VoxelMean>(mean_layer_cache ? mean_layer_cache->buffer() : nullptr),
    gputil::BufferArg<uint64_t>(mean_layer_cache ? &imp_->voxel_upload_info[buf_idx][mean_uidx].offsets_buffer :
                                                   nullptr),
    // Traversal voxels and offsets.
    gputil::BufferArg<float>(traversal_layer_cache ? traversal_layer_cache->buffer() : nullptr),
    gputil::BufferArg<uint64_t>(
      traversal_layer_cache ? &imp_->voxel_upload_info[buf_idx][traversal_uidx].offsets_buffer : nullptr),
    // Touch times voxels and offsets.
    gputil::BufferArg<uint32_t>(touch_times_layer_cache ? touch_times_layer_cache->buffer() : nullptr),
    gputil::BufferArg<uint64_t>(
      touch_times_layer_cache ? &imp_->voxel_upload_info[buf_idx][touch_time_uidx].offsets_buffer : nullptr),
    // Incident normal voxels and offsets.
    gputil::BufferArg<uint32_t>(incidents_layer_cache ? incidents_layer_cache->buffer() : nullptr),
    gputil::BufferArg<uint64_t>(
      incidents_layer_cache ? &imp_->voxel_upload_info[buf_idx][incident_normal_uidx].offsets_buffer : nullptr),
    // Region keys and region count
    gputil::BufferArg<gputil::int3>(imp_->region_key_buffers[buf_idx]), region_count,
    // Ray start/end keys
    gputil::BufferArg<GpuKey>(imp_->key_buffers[buf_idx]),
    // Ray start end points, local to end voxel and ray count
    gputil::BufferArg<gputil::float3>(imp_->ray_buffers[buf_idx]), ray_count,
    // Input touch times buffer
    gputil::BufferArg<uint32_t>((region_update_flags & kRfInternalTimestamps) ? &imp_->timestamps_buffers[buf_idx] :
                                                                                nullptr),
    // Region dimensions, map resolution, ray adjustment (miss), sample adjustment (hit)
    region_dim_gpu, float(map->resolution), map->miss_value, map->hit_value,
    // Occupied threshold, min occupancy, max occupancy, update flags.
    map->occupancy_threshold_value, map->min_voxel_value, map->max_voxel_value, region_update_flags);

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

  // logutil::trace(imp_->region_counts[buf_idx], "regions\n");

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


int GpuMap::enableVoxelUpload(int cache_id, bool enable)
{
  GpuCache &gpu_cache = *gpuCache();
  for (size_t i = 0; i < imp_->voxel_upload_info[0].size(); ++i)
  {
    if (imp_->voxel_upload_info[0][i].gpu_layer_id == cache_id)
    {
      if (enable)
      {
        // Request present and already is. Nothing to do.
        return int(i);
      }
      // Request removal and found present. Remove.
      imp_->voxel_upload_info[0].erase(imp_->voxel_upload_info[0].begin() + i);
      imp_->voxel_upload_info[1].erase(imp_->voxel_upload_info[1].begin() + i);
      return -1;
    }
  }

  // Not found.
  if (enable)
  {
    // Not found and requesting present. Add.
    imp_->voxel_upload_info[0].emplace_back(VoxelUploadInfo(cache_id, gpu_cache.gpu()));
    imp_->voxel_upload_info[1].emplace_back(VoxelUploadInfo(cache_id, gpu_cache.gpu()));
    return int(imp_->voxel_upload_info[0].size() - 1);
  }

  return -1;
}
}  // namespace ohm
