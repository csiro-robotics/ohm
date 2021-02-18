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
#include "CovarianceHitResource.h"
#include "RegionUpdateResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdateNdt);
GPUTIL_CUDA_DECLARE_KERNEL(covarianceHit);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

/// Controls the GPU ordering of kernels to update free space and samples. When 0, the order is free space then samples.
/// When 1, this order is reversed.
///
/// Running the NDT samples first will given better results in new areas, but currently comes with a significant
/// performance hit as we wait more on kernels
#define REVERSE_KERNEL_ORDER 0

namespace ohm
{
namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_ndt_miss("RegionUpdate", GpuProgramRef::kSourceString, RegionUpdateCode,  // NOLINT
                                     RegionUpdateCode_length, { "-DVOXEL_MEAN", "-DNDT" });
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_ndt_miss("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u,
                                     { "-DVOXEL_MEAN", "-DNDT" });
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_cov_hit("CovarianceHit", GpuProgramRef::kSourceString, CovarianceHitCode,  // NOLINT
                                    CovarianceHitCode_length, { "-DVOXEL_MEAN", "-DNDT" });
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_cov_hit("CovarianceHit", GpuProgramRef::kSourceFile, "CovarianceHit.cl", 0u,
                                    { "-DVOXEL_MEAN", "-DNDT" });
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
}  // namespace


GpuNdtMap::GpuNdtMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size)
  : GpuMap(new GpuNdtMapDetail(map, borrowed_map), expected_element_count, gpu_mem_size)
{
  // Ensure voxel mean and covariance layers are present.
  for (int i = 0; i < 2; ++i)
  {
    imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdCovariance, gpuCache()->gpu()));
  }

  // Cache the correct GPU program.
  cacheGpuProgram(true, true);


  // Rays should be grouped by sample voxel for performance reasons. The GPU update assumes this grouping.
  setGroupedRays(true);
}


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


void GpuNdtMap::cacheGpuProgram(bool /*with_voxel_mean*/, bool force)
{
  if (imp_->g_program_ref)
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
  imp->g_program_ref = &g_program_ref_ndt_miss;

  if (imp->g_program_ref->addReference(gpu_cache.gpu()))
  {
    imp->update_kernel = GPUTIL_MAKE_KERNEL(imp->g_program_ref->program(), regionRayUpdateNdt);
    imp->update_kernel.calculateOptimalWorkGroupSize();
    imp->gpu_ok = imp->update_kernel.isValid();
  }
  else
  {
    imp->gpu_ok = false;
  }

  if (imp->gpu_ok)
  {
    imp->cov_hit_program_ref = &g_program_ref_cov_hit;

    if (imp->cov_hit_program_ref->addReference(gpu_cache.gpu()))
    {
      imp->cov_hit_kernel = GPUTIL_MAKE_KERNEL(imp->cov_hit_program_ref->program(), covarianceHit);
      imp->cov_hit_kernel.calculateOptimalWorkGroupSize();
      imp->gpu_ok = imp->cov_hit_kernel.isValid();
    }
    else
    {
      imp->gpu_ok = false;
    }
  }
}


void GpuNdtMap::finaliseBatch(unsigned region_update_flags)
{
  const int buf_idx = imp_->next_buffers_index;
  GpuNdtMapDetail *imp = detail();
  const OccupancyMapDetail *map = imp->map->detail();

  // Complete region data upload.
  GpuCache &gpu_cache = *this->gpuCache();
  GpuLayerCache &occupancy_layer_cache = *gpu_cache.layerCache(kGcIdOccupancy);
  GpuLayerCache &mean_layer_cache = *gpu_cache.layerCache(kGcIdVoxelMean);
  GpuLayerCache &cov_voxel_layer_cache = *gpu_cache.layerCache(kGcIdCovariance);

  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  const unsigned region_count = imp->region_counts[buf_idx];
  const unsigned ray_count = imp->ray_counts[buf_idx];
  gputil::Dim3 global_size(ray_count);
  gputil::Dim3 local_size(std::min<size_t>(imp->update_kernel.optimalWorkGroupSize(), ray_count));
  // Wait for: upload of ray keys, upload of rays, upload of region key mapping.
  gputil::EventList wait(
    { imp->key_upload_events[buf_idx], imp->ray_upload_events[buf_idx], imp->region_key_upload_events[buf_idx] });

  // Add wait for region voxel offsets
  for (auto &upload_info : imp->voxel_upload_info[buf_idx])
  {
    wait.add(upload_info.offset_upload_event);
    wait.add(upload_info.voxel_upload_event);
  }

  unsigned modify_flags = (!(region_update_flags & kRfEndPointAsFree)) ? kRfExcludeSample : 0u;

#if REVERSE_KERNEL_ORDER
  gputil::Event hit_event;

  if (!(region_update_flags & (kRfExcludeSample | kRfEndPointAsFree)))
  {
    // NDT can only have one CovarianceHit batch in flight because it does not support contension. Ensure previous one
    // has completed and it waits on the kernel above to finish too.
    waitOnPreviousOperation(1 - buf_idx);

    local_size = gputil::Dim3(std::min<size_t>(imp->cov_hit_kernel.optimalWorkGroupSize(), ray_count));
    imp->cov_hit_kernel(global_size, local_size, wait, hit_event, &gpu_cache.gpuQueue(),
                        // Kernel args begin:
                        gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                        gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][0].offsets_buffer),
                        gputil::BufferArg<VoxelMean>(*mean_layer_cache.buffer()),
                        gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][1].offsets_buffer),
                        gputil::BufferArg<CovarianceVoxel>(*cov_voxel_layer_cache.buffer()),
                        gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][2].offsets_buffer),
                        gputil::BufferArg<gputil::int3>(imp->region_key_buffers[buf_idx]), region_count,
                        gputil::BufferArg<GpuKey>(imp->key_buffers[buf_idx]),
                        gputil::BufferArg<gputil::float3>(imp->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                        float(map->resolution), map->hit_value, map->occupancy_threshold_value, map->max_voxel_value,
                        imp->ndt_map.sensorNoise(), imp->ndt_map.reinitialiseCovarianceThreshold(),
                        imp->ndt_map.reinitialiseCovariancePointCount());

    wait.clear();
    wait.add(hit_event);
  }

  if (!(region_update_flags & kRfExcludeRay))
  {
    local_size = gputil::Dim3(std::min<size_t>(imp->update_kernel.optimalWorkGroupSize(), ray_count));
    imp->update_kernel(global_size, local_size, wait, imp->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                       // Kernel args begin:
                       gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                       gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][0].offsets_buffer),
                       gputil::BufferArg<VoxelMean>(*mean_layer_cache.buffer()),
                       gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][1].offsets_buffer),
                       gputil::BufferArg<CovarianceVoxel>(*cov_voxel_layer_cache.buffer()),
                       gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][2].offsets_buffer),
                       gputil::BufferArg<gputil::int3>(imp->region_key_buffers[buf_idx]), region_count,
                       gputil::BufferArg<GpuKey>(imp->key_buffers[buf_idx]),
                       gputil::BufferArg<gputil::float3>(imp->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                       float(map->resolution), map->miss_value, map->hit_value, map->occupancy_threshold_value,
                       map->min_voxel_value, map->max_voxel_value, region_update_flags | modify_flags,
                       imp->ndt_map.sensorNoise());
  }
  else
  {
    imp->region_update_events[buf_idx] = hit_event;
  }

#else   // REVERSE_KERNEL_ORDER
  gputil::Event miss_event;
  if (!(region_update_flags & kRfExcludeRay))
  {
    imp->update_kernel(global_size, local_size, wait, miss_event, &gpu_cache.gpuQueue(),
                       // Kernel args begin:
                       gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                       gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][0].offsets_buffer),
                       gputil::BufferArg<VoxelMean>(*mean_layer_cache.buffer()),
                       gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][1].offsets_buffer),
                       gputil::BufferArg<CovarianceVoxel>(*cov_voxel_layer_cache.buffer()),
                       gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][2].offsets_buffer),
                       gputil::BufferArg<gputil::int3>(imp->region_key_buffers[buf_idx]), region_count,
                       gputil::BufferArg<GpuKey>(imp->key_buffers[buf_idx]),
                       gputil::BufferArg<gputil::float3>(imp->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                       float(map->resolution), map->miss_value, map->hit_value, map->occupancy_threshold_value,
                       map->min_voxel_value, map->max_voxel_value, region_update_flags | modify_flags,
                       imp->ndt_map.adaptationRate(), imp->ndt_map.sensorNoise());

    wait.clear();
    wait.add(miss_event);
  }

  if (!(region_update_flags & (kRfExcludeSample | kRfEndPointAsFree)))
  {
    // NDT can only have one CovarianceHit batch in flight because it does not support contension. Ensure previous one
    // has completed and it waits on the kernel above to finish too.
    waitOnPreviousOperation(1 - buf_idx);

    local_size = gputil::Dim3(std::min<size_t>(imp->cov_hit_kernel.optimalWorkGroupSize(), ray_count));
    imp->cov_hit_kernel(global_size, local_size, wait, imp->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                        // Kernel args begin:
                        gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                        gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][0].offsets_buffer),
                        gputil::BufferArg<VoxelMean>(*mean_layer_cache.buffer()),
                        gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][1].offsets_buffer),
                        gputil::BufferArg<CovarianceVoxel>(*cov_voxel_layer_cache.buffer()),
                        gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][2].offsets_buffer),
                        gputil::BufferArg<gputil::int3>(imp->region_key_buffers[buf_idx]), region_count,
                        gputil::BufferArg<GpuKey>(imp->key_buffers[buf_idx]),
                        gputil::BufferArg<gputil::float3>(imp->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                        float(map->resolution), map->hit_value, map->occupancy_threshold_value, map->max_voxel_value,
                        imp->ndt_map.sensorNoise(), imp->ndt_map.reinitialiseCovarianceThreshold(),
                        imp->ndt_map.reinitialiseCovariancePointCount());
  }
  else
  {
    imp->region_update_events[buf_idx] = miss_event;
  }
#endif  // REVERSE_KERNEL_ORDER

  // Update most recent chunk GPU event.
  occupancy_layer_cache.updateEvents(imp->batch_marker, imp->region_update_events[buf_idx]);
  mean_layer_cache.updateEvents(imp->batch_marker, imp->region_update_events[buf_idx]);
  cov_voxel_layer_cache.updateEvents(imp->batch_marker, imp->region_update_events[buf_idx]);

  // std::cout << imp->region_counts[bufIdx] << "
  // regions\n" << std::flush;

  imp->region_counts[buf_idx] = 0;
  // Start a new batch for the GPU layers.
  imp->batch_marker = occupancy_layer_cache.beginBatch();
  mean_layer_cache.beginBatch(imp->batch_marker);
  cov_voxel_layer_cache.beginBatch(imp->batch_marker);
  imp->next_buffers_index = 1 - imp->next_buffers_index;
}


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
}  // namespace ohm
