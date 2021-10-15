// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RaysQueryDetailGpu.h"

#include "private/GpuProgramRef.h"

#include "GpuKey.h"
#include "GpuLayerCache.h"

#include <ohm/OccupancyMap.h>

#include <ohm/private/OccupancyMapDetail.h>

#include <gputil/gpuBuffer.h>
#include <gputil/gpuEventList.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuProgram.h>

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#include "RaysQueryResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(raysQuery);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

namespace ohm
{
namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref("RaysQuery", GpuProgramRef::kSourceString, RaysQueryCode,  // NOLINT
                            RaysQueryCode_length);
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
// NOLINTNEXTLINE(cert-err58-cpp)
GpuProgramRef g_program_ref("RaysQuery", GpuProgramRef::kSourceFile, "RaysQuery.cl", 0u);
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
}  // namespace

RaysQueryMapWrapper::RaysQueryMapWrapper()
  : GpuMap(new RaysQueryMapWrapperDetail)
{
  RaysQueryMapWrapperDetail *imp = detail();
  imp->support_voxel_mean = false;
}


RaysQueryMapWrapper::~RaysQueryMapWrapper()
{}


void RaysQueryMapWrapper::setMap(OccupancyMap *map)
{
  const unsigned expected_query_count = 2048u;
  GpuMap::setMap(map, true, expected_query_count, 0, false);
  RaysQueryMapWrapperDetail *imp = detail();
  if (map)
  {
    if (GpuCache *gpu_cache = gpuCache())
    {
      if (!imp->results_gpu.isValid())
      {
        imp->results_gpu =
          gputil::Buffer(gpu_cache->gpu(), sizeof(RaysQueryResult) * expected_query_count, gputil::kBfWriteHost);
      }
    }

    // Disable instantiation of regions and syncing voxels back to CPU. This is a read only operation.
    for (size_t i = 0; i < imp->voxel_upload_info.size(); ++i)
    {
      for (auto &voxel_info : imp->voxel_upload_info[i])
      {
        voxel_info.allow_region_creation = false;
        voxel_info.skip_cpu_sync = true;
      }
    }
  }
  imp->results_cpu.clear();
}


void RaysQueryMapWrapper::setVolumeCoefficient(float coefficient)
{
  detail()->volume_coefficient = coefficient;
}


float RaysQueryMapWrapper::volumeCoefficient() const
{
  return detail()->volume_coefficient;
}


const std::vector<RaysQueryResult> &RaysQueryMapWrapper::results() const
{
  return detail()->results_cpu;
}


size_t RaysQueryMapWrapper::integrateRays(const glm::dvec3 *rays, size_t element_count, const float *intensities,
                                          const double *timestamps, unsigned ray_update_flags)
{
  RaysQueryMapWrapperDetail *imp = detail();
  imp->results_cpu.clear();
  imp->needs_sync = true;
  return GpuMap::integrateRays(rays, element_count, intensities, timestamps, ray_update_flags);
}


void RaysQueryMapWrapper::onSyncVoxels(int buffer_index)
{
  (void)buffer_index;  // unused
  RaysQueryMapWrapperDetail *imp = detail();
  if (imp->needs_sync)
  {
    imp->results_event.wait();
  }
}


RaysQueryMapWrapperDetail *RaysQueryMapWrapper::detail()
{
  return static_cast<RaysQueryMapWrapperDetail *>(imp_);
}


const RaysQueryMapWrapperDetail *RaysQueryMapWrapper::detail() const
{
  return static_cast<const RaysQueryMapWrapperDetail *>(imp_);
}


void RaysQueryMapWrapper::cacheGpuProgram(bool with_voxel_mean, bool with_traversal, bool force)
{
  (void)with_voxel_mean;  // unused
  (void)with_traversal;   // unused
  if (imp_->program_ref)
  {
    if (!force)
    {
      return;
    }
  }

  releaseGpuProgram();

  GpuCache &gpu_cache = *gpuCache();
  imp_->gpu_ok = true;
  imp_->cached_sub_voxel_program = with_voxel_mean;
  imp_->program_ref = &g_program_ref;

  if (imp_->program_ref->addReference(gpu_cache.gpu()))
  {
    imp_->update_kernel = GPUTIL_MAKE_KERNEL(imp_->program_ref->program(), raysQuery);
    imp_->update_kernel.calculateOptimalWorkGroupSize();

    imp_->gpu_ok = imp_->update_kernel.isValid();
  }
  else
  {
    imp_->gpu_ok = false;
  }
}


void RaysQueryMapWrapper::finaliseBatch(unsigned region_update_flags)
{
  (void)region_update_flags;  // unused

  RaysQueryMapWrapperDetail *imp = detail();
  const int buf_idx = imp->next_buffers_index;
  const OccupancyMapDetail *map = imp->map->detail();

  // Complete region data upload.
  GpuCache &gpu_cache = *this->gpuCache();
  GpuLayerCache &occupancy_layer_cache = *gpu_cache.layerCache(kGcIdOccupancy);

  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  const unsigned region_count = imp->region_counts[buf_idx];
  const unsigned ray_count = imp->ray_counts[buf_idx];
  imp->results_gpu.elementsResize<RaysQueryResult>(ray_count);
  gputil::Dim3 global_size(ray_count);
  gputil::Dim3 local_size(std::min<size_t>(imp->update_kernel.optimalWorkGroupSize(), ray_count));
  gputil::EventList wait(
    { imp->key_upload_events[buf_idx], imp->ray_upload_events[buf_idx], imp->region_key_upload_events[buf_idx],
      imp->voxel_upload_info[buf_idx][0].offset_upload_event, imp->voxel_upload_info[buf_idx][0].voxel_upload_event });


  imp->update_kernel(global_size, local_size, wait, imp->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                     // Kernel args begin:
                     gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                     gputil::BufferArg<uint64_t>(imp->voxel_upload_info[buf_idx][0].offsets_buffer),
                     gputil::BufferArg<gputil::int3>(imp->region_key_buffers[buf_idx]), region_count,
                     gputil::BufferArg<GpuKey>(imp->key_buffers[buf_idx]),
                     gputil::BufferArg<gputil::float3>(imp->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                     float(map->resolution), map->occupancy_threshold_value, imp->volume_coefficient,
                     gputil::BufferArg<RaysQueryResult>(imp->results_gpu));
  // gpu_cache.gpuQueue().flush();


  // Update most recent chunk GPU event.
  occupancy_layer_cache.updateEvents(imp->batch_marker, imp->region_update_events[buf_idx]);

  // Enqueu reading the results.
  imp->results_cpu.resize(ray_count);
  imp->results_gpu.readElements(imp->results_cpu.data(), ray_count, 0, &gpu_cache.gpuQueue(),
                                &imp->region_update_events[buf_idx], &imp->results_event);

  // std::cout << imp->region_counts[bufIdx] << "
  // regions\n" << std::flush;

  imp->region_counts[buf_idx] = 0;
  // Start a new batch for the GPU layers.
  imp->batch_marker = occupancy_layer_cache.beginBatch();
  imp->next_buffers_index = 1 - imp->next_buffers_index;
}

}  // namespace ohm
