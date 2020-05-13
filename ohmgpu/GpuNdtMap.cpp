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

#include <ohm/MapChunk.h>
#include <ohm/NdtVoxel.h>
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

namespace ohm
{
}

GpuNdtMap::GpuNdtMap(OccupancyMap *map, bool borrowed_map, unsigned expected_element_count, size_t gpu_mem_size)
  : GpuMap(new GpuNdtMapDetail(map, borrowed_map), expected_element_count, gpu_mem_size)
{
  for (int i = 0; i < 2; ++i)
  {
    imp_->voxel_upload_info[i].emplace_back(VoxelUploadInfo(kGcIdNdt));
  }
}

GpuNdtMapDetail *GpuNdtMap::detail()
{
  return static_cast<GpuNdtMapDetail *>(imp_);
}


const GpuNdtMapDetail *GpuNdtMap::detail() const
{
  return static_cast<const GpuNdtMapDetail *>(imp_);
}


void GpuNdtMap::finaliseBatch(unsigned region_update_flags)
{
  const int buf_idx = imp_->next_buffers_index;
  const OccupancyMapDetail *map = imp_->map->detail();

  // Complete region data upload.
  GpuCache &gpu_cache = *this->gpuCache();
  GpuLayerCache &occupancy_layer_cache = *gpu_cache.layerCache(kGcIdOccupancy);
  // GpuLayerCache &sub_voxel_layer_cache = *gpu_cache.layerCache(kGcIdSubVoxel);
  GpuLayerCache &ndt_voxel_layer_cache = *gpu_cache.layerCache(kGcIdNdt);

  // if (imp_->map->subVoxelsEnabled())
  // {
  //   sub_voxel_layer_cache = gpu_cache.layerCache(kGcIdSubVoxel);
  // }

  // Enqueue update kernel.
  const gputil::int3 region_dim_gpu = { map->region_voxel_dimensions.x, map->region_voxel_dimensions.y,
                                        map->region_voxel_dimensions.z };

  const unsigned region_count = imp_->region_counts[buf_idx];
  const unsigned ray_count = imp_->ray_counts[buf_idx];
  gputil::Dim3 global_size(ray_count);
  gputil::Dim3 local_size(std::min<size_t>(imp_->update_kernel.optimalWorkGroupSize(), ray_count));
  gputil::EventList wait({ imp_->key_upload_events[buf_idx], imp_->ray_upload_events[buf_idx],
                           imp_->region_key_upload_events[buf_idx] });

  for (size_t i = 0; i < imp_->voxel_upload_info[buf_idx].size(); ++i)
  {
    wait.add(imp_->voxel_upload_info[buf_idx][i].offset_upload_event);
  }

  imp_->update_kernel(global_size, local_size, wait, imp_->region_update_events[buf_idx], &gpu_cache.gpuQueue(),
                      // Kernel args begin:
                      gputil::BufferArg<float>(*occupancy_layer_cache.buffer()),
                      gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][0].offsets_buffer),
                      // gputil::BufferArg<unsigned>(*sub_voxel_layer_cache.buffer()),
                      // gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][1].offsets_buffer),
                      gputil::BufferArg<NdtVoxel>(*ndt_voxel_layer_cache.buffer()),
                        gputil::BufferArg<uint64_t>(imp_->voxel_upload_info[buf_idx][1].offsets_buffer),
                      gputil::BufferArg<gputil::int3>(imp_->region_key_buffers[buf_idx]), region_count,
                      gputil::BufferArg<GpuKey>(imp_->key_buffers[buf_idx]),
                      gputil::BufferArg<gputil::float3>(imp_->ray_buffers[buf_idx]), ray_count, region_dim_gpu,
                      float(map->resolution), map->miss_value, map->hit_value, map->occupancy_threshold_value,
                      map->min_voxel_value, map->max_voxel_value, float(map->sub_voxel_weighting),
                      region_update_flags);

  // gpu_cache.gpuQueue().flush();

  // Update most recent chunk GPU event.
  occupancy_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
  // sub_voxel_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);
  ndt_voxel_layer_cache.updateEvents(imp_->batch_marker, imp_->region_update_events[buf_idx]);

  // std::cout << imp_->region_counts[bufIdx] << "
  // regions\n" << std::flush;

  imp_->region_counts[buf_idx] = 0;
  // Start a new batch for the GPU layers.
  imp_->batch_marker = occupancy_layer_cache.beginBatch();
  // sub_voxel_layer_cache.beginBatch(imp_->batch_marker);
  ndt_voxel_layer_cache.beginBatch(imp_->batch_marker);
  imp_->next_buffers_index = 1 - imp_->next_buffers_index;
}
