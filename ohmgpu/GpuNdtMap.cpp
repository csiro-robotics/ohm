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
#include <ohm/NdtVoxel.h>
#include <ohm/MapCache.h>
#include <ohm/OccupancyMap.h>

#include <ohm/private/OccupancyMapDetail.h>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuProgram.h>

#include <glm/ext.hpp>

using namespace ohm;

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(regionRayUpdateNdt);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref_ndt("RegionUpdate", GpuProgramRef::kSourceString, RegionUpdateCode,  // NOLINT
                                RegionUpdateCode_length, { "-DVOXEL_MEAN", "-DNDT" });
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref_ndt("RegionUpdate", GpuProgramRef::kSourceFile, "RegionUpdate.cl", 0u,
                                { "-DVOXEL_MEAN", "-DNDT" });
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
}  // namespace


GpuNdtMap::GpuNdtMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size)
  : GpuMap(new GpuNdtMapDetail(map, borrowed_map), expected_element_count, gpu_mem_size)
{
  for (int i = 0; i < 2; ++i)
  {
    imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdNdt));
  }

  // Cache the correct GPU program.
  cacheGpuProgram(true, true);
}


const NdtMap &GpuNdtMap::ndtMap() const
{
  const GpuNdtMapDetail *imp = detail();
  return imp->ndt_map;
}


size_t GpuNdtMap::integrateRays(const glm::dvec3 *rays, size_t element_count, unsigned region_update_flags)
{
  size_t update_count = GpuMap::integrateRays(rays, element_count, region_update_flags);

  gpumap::sync(*imp_->map, kGcIdOccupancy);

  // Process the ray samples we've integrated.
  GpuNdtMapDetail *imp = detail();
  ohm::MapCache cache;
  for (unsigned ray_id : imp->current_ray_ids)
  {
    const glm::dvec3 &sensor = rays[ray_id * 2 + 0];
    const glm::dvec3 &sample = rays[ray_id * 2 + 1];
    const ohm::Key key = imp->map->voxelKey(sample);

    // OHM_ASSERT(!key.IsNull());
    ohm::Voxel voxel = imp->map->voxel(key, true, &cache);
    imp->ndt_map.integrateHit(voxel, sensor, sample);
  }

  return update_count;
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
  if (imp_->program_ref)
  {
    if (!force)
    {
      return;
    }
  }

  releaseGpuProgram();

  GpuCache &gpu_cache = *gpuCache();
  imp_->cached_sub_voxel_program = true;
  imp_->program_ref = &program_ref_ndt;

  if (imp_->program_ref->addReference(gpu_cache.gpu()))
  {
    imp_->update_kernel = GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), regionRayUpdateNdt);
    imp_->update_kernel.calculateOptimalWorkGroupSize();
    imp_->gpu_ok = imp_->update_kernel.isValid();
  }
  else
  {
    imp_->gpu_ok = false;
  }
}


void GpuNdtMap::finaliseBatch(unsigned region_update_flags)
{
  const int buf_idx = imp_->next_buffers_index;
  const OccupancyMapDetail *map = imp_->map->detail();

  // Complete region data upload.
  GpuCache &gpu_cache = *this->gpuCache();
  GpuLayerCache &occupancy_layer_cache = *gpu_cache.layerCache(kGcIdOccupancy);
  GpuLayerCache &mean_layer_cache = *gpu_cache.layerCache(kGcIdVoxelMean);
  GpuLayerCache &ndt_voxel_layer_cache = *gpu_cache.layerCache(kGcIdNdt);

  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  const unsigned region_count = imp_->region_counts[buf_idx];
  const unsigned ray_count = imp_->ray_counts[buf_idx];
  gputil::Dim3 global_size(ray_count);
  gputil::Dim3 local_size(std::min<size_t>(imp_->update_kernel.optimalWorkGroupSize(), ray_count));
  gputil::EventList wait(
    { imp_->key_upload_events[buf_idx], imp_->ray_upload_events[buf_idx], imp_->region_key_upload_events[buf_idx] });

  for (size_t i = 0; i < imp_->voxel_upload_info[buf_idx].size(); ++i)
  {
    wait.add(imp_->voxel_upload_info[buf_idx][i].offset_upload_event);
  }

  imp_->update_kernel(global_size, local_size, wait, imp_->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                      // Kernel args begin:
                      gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                      gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][0].offsets_buffer),
                      gputil::BufferArg<unsigned>(*mean_layer_cache.buffer()),
                      gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][1].offsets_buffer),
                      gputil::BufferArg<NdtVoxel>(*ndt_voxel_layer_cache.buffer()),
                      gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][2].offsets_buffer),
                      gputil::BufferArg<gputil::int3>(imp_->region_key_buffers[buf_idx]), region_count,
                      gputil::BufferArg<GpuKey>(imp_->key_buffers[buf_idx]),
                      gputil::BufferArg<gputil::float3>(imp_->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                      float(map->resolution), map->miss_value, map->hit_value, map->occupancy_threshold_value,
                      map->min_voxel_value, map->max_voxel_value, region_update_flags);

  // gpu_cache.gpuQueue().flush();

  // Update most recent chunk GPU event.
  occupancy_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
  // mean_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
  ndt_voxel_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);

  // std::cout << imp_->region_counts[bufIdx] << "
  // regions\n" << std::flush;

  imp_->region_counts[buf_idx] = 0;
  // Start a new batch for the GPU layers.
  imp_->batch_marker = occupancy_layer_cache.beginBatch();
  mean_layer_cache.beginBatch(imp_->batch_marker);
  ndt_voxel_layer_cache.beginBatch(imp_->batch_marker);
  imp_->next_buffers_index = 1 - imp_->next_buffers_index;
}
