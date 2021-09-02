// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuNdtMap.h"

#include "private/GpuNdtMapDetail.h"

#include "GpuCache.h"
#include "GpuKey.h"
#include "GpuLayerCache.h"

#include "private/GpuProgramRef.h"

#include <ohm/MapChunk.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelMean.h>

#include <ohm/private/OccupancyMapDetail.h>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuProgram.h>

#include <glm/ext.hpp>
#include <glm/gtx/norm.hpp>

// Must come after glm includes due to usage on GPU.
#include <ohm/CovarianceVoxel.h>

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#include "CovarianceHitNdtResource.h"
#include "RegionUpdateResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdateNdt);
GPUTIL_CUDA_DECLARE_KERNEL(covarianceHitNdt);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

namespace ohm
{
namespace
{
// Note on -DNDT=
// We need to define NDT to the correct value depending on the algorithm to use. See NdtModeDef.cl
// 0 => no NDT
// 1 => NDT occupancy map
// 2 => NDT traverability map
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_ndt_miss("RegionUpdate", GpuProgramRef::kSourceString, RegionUpdateCode,  // NOLINT
                                     RegionUpdateCode_length, { "-DNDT" });
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_ndt_hit("CovarianceHitNdt", GpuProgramRef::kSourceString, CovarianceHitNdtCode,  // NOLINT
                                    CovarianceHitNdtCode_length);
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_ndt_miss("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u, { "-DNDT" });
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_ndt_hit("CovarianceHitNdt", GpuProgramRef::kSourceFile, "CovarianceHitNdt.cl");  // NOLINT
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
}  // namespace


GpuNdtMap::GpuNdtMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size,
                     NdtMode ndt_mode)
  : GpuMap(new GpuNdtMapDetail(map, borrowed_map, ndt_mode), expected_element_count, gpu_mem_size)
{
  // Ensure voxel mean and covariance layers are present.

  for (int i = 0; i < 2; ++i)
  {
    if (ndt_mode != NdtMode::kNone)
    {
      detail()->cov_uidx = int(imp_->voxel_upload_info[i].size());  // Set twice to the same value, but that's ok.
      imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdCovariance, gpuCache()->gpu()));
    }
    if (ndt_mode == NdtMode::kTraversability)
    {
      detail()->intensity_uidx = int(imp_->voxel_upload_info[i].size());  // Set twice to the same value, but that's ok.
      imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdIntensity, gpuCache()->gpu()));
      detail()->hit_miss_uidx = int(imp_->voxel_upload_info[i].size());  // Set twice to the same value, but that's ok.
      imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdHitMiss, gpuCache()->gpu()));
    }
  }

  // Cache the correct GPU program.
  cacheGpuProgram(true, map && imp_->support_traversal && map->traversalEnabled(), true);

  // Rays should be grouped by sample voxel for performance reasons. The GPU update assumes this grouping.
  setGroupedRays(true);
}  // namespace ohm


GpuNdtMap::~GpuNdtMap()
{
  GpuNdtMap::releaseGpuProgram();
}


void GpuNdtMap::setSensorNoise(float noise_range)
{
  GpuNdtMapDetail *imp = detail();
  imp->ndt_map.setSensorNoise(noise_range);
}


float GpuNdtMap::sensorNoise() const
{
  const GpuNdtMapDetail *imp = detail();
  return imp->ndt_map.sensorNoise();
}


NdtMap &GpuNdtMap::ndtMap()
{
  GpuNdtMapDetail *imp = detail();
  return imp->ndt_map;
}


const NdtMap &GpuNdtMap::ndtMap() const
{
  const GpuNdtMapDetail *imp = detail();
  return imp->ndt_map;
}


void GpuNdtMap::debugDraw() const
{
  const GpuNdtMapDetail *imp = detail();
  imp->ndt_map.debugDraw();
}


GpuNdtMapDetail *GpuNdtMap::detail()
{
  return static_cast<GpuNdtMapDetail *>(imp_);
}


const GpuNdtMapDetail *GpuNdtMap::detail() const
{
  return static_cast<const GpuNdtMapDetail *>(imp_);
}


void GpuNdtMap::cacheGpuProgram(bool /*with_voxel_mean*/, bool /*with_traversal_rate*/, bool force)
{
  if (imp_->program_ref)
  {
    if (!force)
    {
      return;
    }
  }

  releaseGpuProgram();

  GpuCache &gpu_cache = *gpuCache();
  GpuNdtMapDetail *imp = detail();
  imp->gpu_ok = true;
  imp->cached_sub_voxel_program = true;

  switch (imp->ndt_map.mode())
  {
  default:  // Unknown mode
    imp_->gpu_ok = false;
    break;
  case NdtMode::kOccupancy:
  case NdtMode::kTraversability:
    imp->program_ref = &g_program_ref_ndt_miss;
    imp->cov_hit_program_ref = &g_program_ref_ndt_hit;
    imp->gpu_ok = imp->program_ref->addReference(gpu_cache.gpu()) && imp_->gpu_ok;
    imp->gpu_ok = imp->cov_hit_program_ref->addReference(gpu_cache.gpu()) && imp_->gpu_ok;

    if (imp_->gpu_ok)
    {
      imp->update_kernel = GPUTIL_MAKE_KERNEL(imp->program_ref->program(), regionRayUpdateNdt);
      imp->cov_hit_kernel = GPUTIL_MAKE_KERNEL(imp->cov_hit_program_ref->program(), covarianceHitNdt);
    }
    break;
  }

  if (imp_->gpu_ok)
  {
    imp->update_kernel.calculateOptimalWorkGroupSize();
    imp->cov_hit_kernel.calculateOptimalWorkGroupSize();
    imp->gpu_ok = imp->update_kernel.isValid() && imp->cov_hit_kernel.isValid();
  }
}


void GpuNdtMap::finaliseBatch(unsigned region_update_flags)
{
  const int buf_idx = imp_->next_buffers_index;
  GpuNdtMapDetail *imp = detail();

  // Wait for: upload of ray keys, upload of rays, upload of region key mapping.
  gputil::EventList wait(
    { imp->key_upload_events[buf_idx], imp->ray_upload_events[buf_idx], imp->region_key_upload_events[buf_idx] });

  // Add wait for region voxel offsets
  for (auto &upload_info : imp->voxel_upload_info[buf_idx])
  {
    wait.add(upload_info.offset_upload_event);
    wait.add(upload_info.voxel_upload_event);
  }

  // We will track which caches we depend on to manage event waiting.
  TouchedCacheSet touched_caches = { nullptr };

  switch (imp->ndt_map.mode())
  {
  case NdtMode::kOccupancy:
  case NdtMode::kTraversability:
    invokeNdt(region_update_flags, buf_idx, wait, touched_caches);
    break;
  default:
    // Invalid mode et.
    break;
  }

  // Update most recent chunk GPU event.
  for (auto &&layer_cache : touched_caches)
  {
    if (layer_cache)
    {
      layer_cache->updateEvents(imp->batch_marker, imp->region_update_events[buf_idx]);
    }
  }

  // std::cout << imp->region_counts[bufIdx] << "
  // regions\n" << std::flush;

  imp->region_counts[buf_idx] = 0;
  // Start a new batch for the GPU layers.
  bool have_batch = false;
  for (auto &&layer_cache : touched_caches)
  {
    if (layer_cache)
    {
      if (have_batch)
      {
        layer_cache->beginBatch(imp->batch_marker);
      }
      else
      {
        imp->batch_marker = layer_cache->beginBatch();
      }
      layer_cache->updateEvents(imp->batch_marker, imp->region_update_events[buf_idx]);
    }
  }

  imp->next_buffers_index = 1 - imp->next_buffers_index;
}  // namespace ohm


void GpuNdtMap::releaseGpuProgram()
{
  GpuMap::releaseGpuProgram();
  GpuNdtMapDetail *imp = detail();
  if (imp && imp->cov_hit_kernel.isValid())
  {
    imp->cov_hit_kernel = gputil::Kernel();
  }

  if (imp && imp->cov_hit_program_ref)
  {
    imp->cov_hit_program_ref->releaseReference();
    imp->cov_hit_program_ref = nullptr;
  }
}


void GpuNdtMap::invokeNdt(unsigned region_update_flags, int buf_idx, gputil::EventList &wait,
                          TouchedCacheSet &touched_caches)
{
  GpuNdtMapDetail *imp = detail();
  const OccupancyMapDetail *map = imp->map->detail();

  // Complete region data upload.
  GpuCache &gpu_cache = *gpuCache();

  GpuLayerCache &occupancy_layer_cache = *gpu_cache.layerCache(kGcIdOccupancy);
  GpuLayerCache &mean_layer_cache = *gpu_cache.layerCache(kGcIdVoxelMean);
  GpuLayerCache &cov_voxel_layer_cache = *gpu_cache.layerCache(kGcIdCovariance);
  GpuLayerCache *intensity_layer_cache = nullptr;
  GpuLayerCache *hit_miss_layer_cache = nullptr;
  GpuLayerCache *traversal_layer_cache = nullptr;
  GpuLayerCache *touch_times_layer_cache = nullptr;
  GpuLayerCache *incidents_layer_cache = nullptr;

  const int occ_uidx = detail()->occupancy_uidx;
  const int mean_uidx = detail()->mean_uidx;
  const int cov_uidx = detail()->cov_uidx;
  const int intense_uidx = detail()->intensity_uidx;
  const int hit_miss_uidx = detail()->hit_miss_uidx;
  const int traversal_uidx = detail()->traversal_uidx;
  const int touch_time_uidx = imp_->touch_time_uidx;
  const int incident_normal_uidx = imp_->incident_normal_uidx;

  if (imp->ndt_map.mode() == NdtMode::kTraversability)
  {
    intensity_layer_cache = gpu_cache.layerCache(kGcIdIntensity);
    hit_miss_layer_cache = gpu_cache.layerCache(kGcIdHitMiss);
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

  size_t tidx = 0;
  touched_caches[tidx++] = &occupancy_layer_cache;
  touched_caches[tidx++] = &mean_layer_cache;
  touched_caches[tidx++] = &cov_voxel_layer_cache;
  if (intensity_layer_cache)
  {
    touched_caches[tidx++] = intensity_layer_cache;
  }
  if (hit_miss_layer_cache)
  {
    touched_caches[tidx++] = hit_miss_layer_cache;
  }
  if (traversal_layer_cache)
  {
    touched_caches[tidx++] = traversal_layer_cache;
  }
  if (touch_times_layer_cache)
  {
    touched_caches[tidx++] = touch_times_layer_cache;
  }
  if (incidents_layer_cache)
  {
    touched_caches[tidx++] = incidents_layer_cache;
  }
  for (; tidx < touched_caches.size(); ++tidx)
  {
    touched_caches[tidx] = nullptr;
  }

  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  const unsigned region_count = imp->region_counts[buf_idx];
  const unsigned ray_count = imp->ray_counts[buf_idx];
  // Rays are sorted such that unclipped entries come first. We only need to consider these.
  const unsigned sample_count = imp->unclipped_sample_counts[buf_idx];

  gputil::Event first_kernel_event;
  gputil::Dim3 global_size;
  gputil::Dim3 local_size;

  unsigned modify_flags = (!(region_update_flags & kRfEndPointAsFree)) ? kRfExcludeSample : 0u;

  if (!(region_update_flags & kRfExcludeRay))
  {
    global_size = gputil::Dim3(ray_count);
    local_size = gputil::Dim3(std::min<size_t>(imp->update_kernel.optimalWorkGroupSize(), ray_count));
    imp->update_kernel(
      global_size, local_size, wait, first_kernel_event, &gpu_cache.gpuQueue(),
      // Kernel args begin:
      // Occupancy voxels and offsets.
      gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
      gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][occ_uidx].offsets_buffer),
      // Mean voxels and offsets.
      gputil::BufferArg<VoxelMean>(*mean_layer_cache.buffer()),
      gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][mean_uidx].offsets_buffer),
      // Covariance voxels and offsets.
      gputil::BufferArg<CovarianceVoxel>(*cov_voxel_layer_cache.buffer()),
      gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][cov_uidx].offsets_buffer),
      // Hit/miss voxels and offsets.
      gputil::BufferArg<HitMissCount>(hit_miss_layer_cache ? hit_miss_layer_cache->buffer() : nullptr),
      gputil::BufferArg<uint64_t>(
        hit_miss_layer_cache ? &imp->voxel_upload_info[buf_idx][hit_miss_uidx].offsets_buffer : nullptr),
      // Traversal voxels and offsets.
      gputil::BufferArg<float>(traversal_layer_cache ? traversal_layer_cache->buffer() : nullptr),
      gputil::BufferArg<uint64_t>(
        traversal_layer_cache ? &imp->voxel_upload_info[buf_idx][traversal_uidx].offsets_buffer : nullptr),
      // Touch times voxels and offsets.
      gputil::BufferArg<uint32_t>(nullptr), gputil::BufferArg<uint64_t>(nullptr),  // No touch times for miss update
      // Incident normal voxels and offsets.
      gputil::BufferArg<uint32_t>(nullptr), gputil::BufferArg<uint64_t>(nullptr),  // No incidents for miss update
                                                                                   // Region keys and region count
      gputil::BufferArg<gputil::int3>(imp->region_key_buffers[buf_idx]), region_count,
      // Ray start/end keys
      gputil::BufferArg<GpuKey>(imp->key_buffers[buf_idx]),
      // Ray start end points, local to end voxel and ray count
      gputil::BufferArg<gputil::float3>(imp->ray_buffers[buf_idx]), ray_count,
      // Input touch times buffer
      gputil::BufferArg<uint32_t>(nullptr),  // No touch times for miss update
      // Region dimensions, map resolution, ray adjustment (miss), sample adjustment (hit)
      region_dim_gpu, float(map->resolution), map->miss_value, map->hit_value,
      // Occupied threshold, min occupancy, max occupancy.
      map->occupancy_threshold_value, map->min_voxel_value, map->max_voxel_value,
      // Update flags, NDT adaptation rate, NDT model sensor noise
      region_update_flags | modify_flags, imp->ndt_map.adaptationRate(), imp->ndt_map.sensorNoise());
    wait.clear();
    wait.add(first_kernel_event);
  }

  if (!(region_update_flags & (kRfExcludeSample | kRfEndPointAsFree)) && sample_count > 0)
  {
    // NDT can only have one CovarianceHitNdtTm batch in flight because it does not support contension. Ensure previous
    // one has completed and it waits on the kernel above to finish too.
    waitOnPreviousOperation(1 - buf_idx);

    global_size = gputil::Dim3(sample_count);
    local_size = gputil::Dim3(std::min<size_t>(imp->cov_hit_kernel.optimalWorkGroupSize(), sample_count));
    imp->cov_hit_kernel(
      global_size, local_size, wait, imp->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
      // Kernel args begin:
      // Occupancy voxels and offsets.
      gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
      gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][occ_uidx].offsets_buffer),
      // Mean voxels and offsets.
      gputil::BufferArg<VoxelMean>(*mean_layer_cache.buffer()),
      gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][mean_uidx].offsets_buffer),
      // Covariance voxels and offsets.
      gputil::BufferArg<CovarianceVoxel>(*cov_voxel_layer_cache.buffer()),
      gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][cov_uidx].offsets_buffer),
      // Intensity voxels and offsets.
      gputil::BufferArg<IntensityMeanCov>(intensity_layer_cache ? intensity_layer_cache->buffer() : nullptr),
      gputil::BufferArg<uint64_t>(
        intensity_layer_cache ? &imp->voxel_upload_info[buf_idx][intense_uidx].offsets_buffer : nullptr),
      // Hit/miss voxels and offsets.
      gputil::BufferArg<HitMissCount>(hit_miss_layer_cache ? hit_miss_layer_cache->buffer() : nullptr),
      gputil::BufferArg<uint64_t>(
        hit_miss_layer_cache ? &imp->voxel_upload_info[buf_idx][hit_miss_uidx].offsets_buffer : nullptr),
      // Traversal voxels and offsets.
      gputil::BufferArg<float>(traversal_layer_cache ? traversal_layer_cache->buffer() : nullptr),
      gputil::BufferArg<uint64_t>(
        traversal_layer_cache ? &imp->voxel_upload_info[buf_idx][traversal_uidx].offsets_buffer : nullptr),
      // Touch times voxels and offsets.
      gputil::BufferArg<uint32_t>(touch_times_layer_cache ? touch_times_layer_cache->buffer() : nullptr),
      gputil::BufferArg<uint64_t>(
        touch_times_layer_cache ? &imp_->voxel_upload_info[buf_idx][touch_time_uidx].offsets_buffer : nullptr),
      // Incident normal voxels and offsets.
      gputil::BufferArg<uint32_t>(incidents_layer_cache ? incidents_layer_cache->buffer() : nullptr),
      gputil::BufferArg<uint64_t>(
        incidents_layer_cache ? &imp_->voxel_upload_info[buf_idx][incident_normal_uidx].offsets_buffer : nullptr),
      // Region keys and region count
      gputil::BufferArg<gputil::int3>(imp->region_key_buffers[buf_idx]), region_count,
      // Ray start/end keys
      gputil::BufferArg<GpuKey>(imp->key_buffers[buf_idx]),
      // Ray start end points, local to end voxel and sample count
      gputil::BufferArg<gputil::float3>(imp->ray_buffers[buf_idx]), sample_count,
      // Input touch times buffer
      gputil::BufferArg<uint32_t>((region_update_flags & kRfInternalTimestamps) ? &imp_->timestamps_buffers[buf_idx] :
                                                                                  nullptr),
      // Input intensities buffer
      gputil::BufferArg<float>(intensity_layer_cache ? &imp->intensities_buffers[buf_idx] : nullptr),
      // Region dimensions, map resolution, sample adjustment (hit), occupancy threshold, occupancy max value
      region_dim_gpu, float(map->resolution), map->hit_value, map->occupancy_threshold_value, map->max_voxel_value,
      // Intensity covariance initialisation, NDT sample threshold, NDT adaptation rate
      imp->ndt_map.initialIntensityCovariance(), imp->ndt_map.ndtSampleThreshold(), imp->ndt_map.adaptationRate(),
      // NDT model sensor noise, NDt reinitialisation covariance threshold
      imp->ndt_map.sensorNoise(), imp->ndt_map.reinitialiseCovarianceThreshold(),
      // NDT reinitialisation sample count
      imp->ndt_map.reinitialiseCovariancePointCount());
  }
  else
  {
    imp->region_update_events[buf_idx] = first_kernel_event;
  }
}
}  // namespace ohm
