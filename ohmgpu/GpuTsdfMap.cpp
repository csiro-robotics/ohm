// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "GpuTsdfMap.h"

#include "private/GpuTsdfMapDetail.h"

#include "GpuCache.h"
#include "GpuKey.h"
#include "GpuLayerCache.h"

#include "private/GpuProgramRef.h"

#include <ohm/DefaultLayer.h>
#include <ohm/Logger.h>
#include <ohm/MapChunk.h>
#include <ohm/MapLayout.h>
#include <ohm/OccupancyMap.h>
#include <ohm/VoxelTsdf.h>

#include <ohm/private/OccupancyMapDetail.h>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuProgram.h>

#include <glm/ext.hpp>
#include <glm/gtx/norm.hpp>

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#include "TsdfUpdateResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(tsdfRayUpdate);
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
GpuProgramRef g_program_ref_tsdf("TsdfUpdate", GpuProgramRef::kSourceString, TsdfUpdateCode,  // NOLINT
                                 TsdfUpdateCode_length, {});
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref_tsdf("TsdfUpdate", GpuProgramRef::kSourceFile, "TsdfUpdate.cl", 0u, {});
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
}  // namespace


GpuTsdfMap::GpuTsdfMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size)
  : GpuMap(new GpuTsdfMapDetail(map, borrowed_map), expected_element_count, gpu_mem_size)
{
  // Ensure tsdf layer is present.
  if (map->layout().layerIndex(default_layer::tsdfLayerName()) == -1)
  {
    // Copy and update layout then update in the map.
    MapLayout layout = map->layout();
    addTsdf(layout);
    map->updateLayout(layout);
  }

  // Buffer management is a bit messy because it's a retrofit.
  for (int i = 0; i < 2; ++i)
  {
    // We only want TSDF data, so clear what the base class added here.
    imp_->voxel_upload_info[i].clear();
    detail()->tsdf_uidx = int(imp_->voxel_upload_info[i].size());  // Set twice to the same value, but that's ok.
    imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdTsdf, gpuCache()->gpu()));
  }

  // Only using TSDF.
  imp_->occupancy_uidx = -1;
  imp_->mean_uidx = -1;
  imp_->traversal_uidx = -1;
  imp_->touch_time_uidx = -1;
  imp_->incident_normal_uidx = -1;

  // Cache the correct GPU program.
  cacheGpuProgram(false, false, true);
}  // namespace ohm


GpuTsdfMap::~GpuTsdfMap()
{
  GpuTsdfMap::releaseGpuProgram();
}


void GpuTsdfMap::setTsdfOptions(const TsdfOptions &options)
{
  GpuTsdfMapDetail *imp = detail();
  imp->tsdf_options = options;
  if (imp->map)
  {
    updateMapInfo(imp->map->mapInfo(), imp->tsdf_options);
  }
}


const TsdfOptions &GpuTsdfMap::tsdfOptions() const
{
  const GpuTsdfMapDetail *imp = detail();
  return imp->tsdf_options;
}


void GpuTsdfMap::setMaxWeight(float max_weight)
{
  GpuTsdfMapDetail *imp = detail();
  imp->tsdf_options.max_weight = max_weight;
  if (imp->map)
  {
    updateMapInfo(imp->map->mapInfo(), imp->tsdf_options);
  }
}


float GpuTsdfMap::maxWeight() const
{
  const GpuTsdfMapDetail *imp = detail();
  return imp->tsdf_options.max_weight;
}


void GpuTsdfMap::setDefaultTruncationDistance(float default_truncation_distance)
{
  GpuTsdfMapDetail *imp = detail();
  imp->tsdf_options.default_truncation_distance = default_truncation_distance;
  if (imp->map)
  {
    updateMapInfo(imp->map->mapInfo(), imp->tsdf_options);
  }
}


float GpuTsdfMap::defaultTruncationDistance() const
{
  const GpuTsdfMapDetail *imp = detail();
  return imp->tsdf_options.default_truncation_distance;
}


void GpuTsdfMap::setDropoffEpsilon(float dropoff_epsilon)
{
  GpuTsdfMapDetail *imp = detail();
  imp->tsdf_options.dropoff_epsilon = dropoff_epsilon;
  if (imp->map)
  {
    updateMapInfo(imp->map->mapInfo(), imp->tsdf_options);
  }
}


float GpuTsdfMap::dropoffEpsilon() const
{
  const GpuTsdfMapDetail *imp = detail();
  return imp->tsdf_options.dropoff_epsilon;
}


void GpuTsdfMap::setSparsityCompensationFactor(float sparsity_compensation_factor)
{
  GpuTsdfMapDetail *imp = detail();
  imp->tsdf_options.sparsity_compensation_factor = sparsity_compensation_factor;
  if (imp->map)
  {
    updateMapInfo(imp->map->mapInfo(), imp->tsdf_options);
  }
}


float GpuTsdfMap::sparsityCompensationFactor() const
{
  const GpuTsdfMapDetail *imp = detail();
  return imp->tsdf_options.sparsity_compensation_factor;
}


GpuTsdfMapDetail *GpuTsdfMap::detail()
{
  return static_cast<GpuTsdfMapDetail *>(imp_);
}


const GpuTsdfMapDetail *GpuTsdfMap::detail() const
{
  return static_cast<const GpuTsdfMapDetail *>(imp_);
}


void GpuTsdfMap::cacheGpuProgram(bool /*with_voxel_mean*/, bool /*with_traversal_rate*/, bool force)
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
  GpuTsdfMapDetail *imp = detail();
  imp->gpu_ok = true;
  imp->cached_sub_voxel_program = true;

  imp->program_ref = &g_program_ref_tsdf;
  imp->gpu_ok = imp->program_ref->addReference(gpu_cache.gpu()) && imp_->gpu_ok;

  if (imp_->gpu_ok)
  {
    imp->update_kernel = GPUTIL_MAKE_KERNEL(imp->program_ref->program(), tsdfRayUpdate);
  }

  if (imp_->gpu_ok)
  {
    imp->update_kernel.calculateOptimalWorkGroupSize();
    imp->gpu_ok = imp->update_kernel.isValid();
  }
}


void GpuTsdfMap::finaliseBatch(unsigned region_update_flags)
{
  const int buf_idx = imp_->next_buffers_index;
  const OccupancyMapDetail *map = imp_->map->detail();
  GpuTsdfMapDetail *imp = detail();

  // We will track which caches we depend on to manage event waiting.
  GpuCache &gpu_cache = *this->gpuCache();
  GpuLayerCache &tsdf_layer_cache = *gpu_cache.layerCache(kGcIdTsdf);
  const int tsdf_uidx = imp->tsdf_uidx;
  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  if (!imp_->use_original_ray_buffers)
  {
    ohm::logger::error("TSDF algorithm requires GPU samples_buffers, but the flag has been disabled. Aborting.\n");
    return;
  }

  const unsigned region_count = imp_->region_counts[buf_idx];
  const unsigned ray_count = imp_->ray_counts[buf_idx];
  gputil::Dim3 global_size(ray_count);
  gputil::Dim3 local_size(std::min<size_t>(imp_->update_kernel.optimalWorkGroupSize(), ray_count));

  // Note: we also wait on original_ray_upload_events here as the samples are required to calculate the TSDF distances.
  gputil::EventList wait({ imp_->key_upload_events[buf_idx], imp_->ray_upload_events[buf_idx],
                           imp_->original_ray_upload_events[buf_idx], imp_->region_key_upload_events[buf_idx],
                           imp_->voxel_upload_info[buf_idx][tsdf_uidx].offset_upload_event,
                           imp_->voxel_upload_info[buf_idx][tsdf_uidx].voxel_upload_event });

  // Supporting voxel mean and traversal are putting us at the limit of what we can support using this sort of
  // conditional invocation.
  imp_->update_kernel(global_size, local_size, wait, imp_->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                      // Kernel args begin:
                      // Tsdf voxels and offsets.
                      gputil::BufferArg<VoxelTsdf>(*tsdf_layer_cache.buffer()),
                      gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][tsdf_uidx].offsets_buffer),
                      // Region keys and region count
                      gputil::BufferArg<gputil::int3>(imp_->region_key_buffers[buf_idx]), region_count,
                      // Ray start/end keys
                      gputil::BufferArg<GpuKey>(imp_->key_buffers[buf_idx]),
                      // Ray start end points, local to end voxel and ray count
                      gputil::BufferArg<gputil::float3>(imp_->ray_buffers[buf_idx]),
                      // Original ray sensor/samples buffer.
                      gputil::BufferArg<gputil::float3>(imp_->original_ray_buffers[buf_idx]), ray_count,
                      // Region dimensions, map resolution, TSDF settings.
                      region_dim_gpu, float(map->resolution), imp->tsdf_options.max_weight,
                      imp->tsdf_options.default_truncation_distance, imp->tsdf_options.dropoff_epsilon,
                      imp->tsdf_options.sparsity_compensation_factor, region_update_flags);

  // Update most recent chunk GPU event.
  tsdf_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);

  // ohm::logger::trace(imp_->region_counts[buf_idx], "regions\n");

  imp_->region_counts[buf_idx] = 0;
  // Start a new batch for the GPU layers.
  imp_->batch_marker = tsdf_layer_cache.beginBatch();
  imp_->next_buffers_index = (imp_->next_buffers_index + 1) % GpuMapDetail::kBuffersCount;
}
}  // namespace ohm
