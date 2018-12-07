// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RoiRangeFill.h"

#include "cl/clProgram.h"
#include "GpuCache.h"
#include "GpuKey.h"
#include "GpuLayerCache.h"
#include "MapLayer.h"
#include "Key.h"
#include "OccupancyMap.h"
#include "OccupancyUtil.h"
#include "DefaultLayer.h"
#include "private/OccupancyMapDetail.h"
#include "private/GpuMapDetail.h"

#include <gputil/gpuPlatform.h>
#include <gputil/gpuPinnedBuffer.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <limits>

#define KERNEL_PROFILING 0
#include <ohmutil/Profile.h>
#include "GpuMap.h"

using namespace ohm;

namespace roirangefill
{
  int initGpu(gputil::Device &gpu);
  void releaseGpu();
  int invoke(const OccupancyMapDetail &map,
             RoiRangeFill &query, GpuCache &gpu_cache,
             GpuLayerCache &clearance_layer_cache,
             const glm::ivec3 &input_data_extents,
             const std::vector<gputil::Event> &upload_events);
}


namespace
{
  void finishRegion(const glm::i16vec3 &region_key, OccupancyMap &map, RoiRangeFill &query, GpuCache &gpu_cache,
                    GpuLayerCache &clearance_cache, const glm::ivec3 &batch_voxel_extents,
                    const std::vector<gputil::Event> &upload_events)
  {
    PROFILE(finishRegion);

    PROFILE(invoke);
    // Ensure sufficient working voxel memory size.
    for (int i = 0; i < 2; ++i)
    {
      query.gpuWork(i).elementsResize<gputil::char4>(volumeOf(batch_voxel_extents));
    }

    roirangefill::invoke(*map.detail(), query, gpu_cache, clearance_cache, batch_voxel_extents, upload_events);
    PROFILE_END(invoke);

    PROFILE(download);
    // Download back to main memory.
    MapChunk *region = map.region(region_key);
    if (region)
    {
      gputil::PinnedBuffer clearance_buffer(query.gpuRegionClearanceBuffer(), gputil::kPinRead);
      const MapLayer &clearance_layer = map.layout().layer(kDlClearance);
      uint8_t *dst = clearance_layer.voxels(*region);
      clearance_buffer.read(dst, clearance_layer.layerByteSize(map.regionVoxelDimensions()));
    }
    PROFILE_END(download);
  }
}


RoiRangeFill::RoiRangeFill(gputil::Device &gpu)
{
  gpu_ = gpu;

  // Initialise buffer to dummy size. We'll resize as required.
  gpu_corner_voxel_key_ = gputil::Buffer(gpu, sizeof(GpuKey), gputil::kBfReadHost);
  gpu_region_keys_ = gputil::Buffer(gpu, 32 * sizeof(gputil::int3), gputil::kBfReadHost);
  gpu_occupancy_region_offsets_ = gputil::Buffer(gpu, 32 * sizeof(gputil::ulong1), gputil::kBfReadHost);
  // Place holder allocation:
  gpu_region_clearance_buffer_ = gputil::Buffer(gpu, 4, gputil::kBfReadHost);
  for (auto &gpu_work : gpu_work_)
  {
    gpu_work = gputil::Buffer(gpu, 1 * sizeof(gputil::char4), gputil::kBfReadWrite);
  }

  valid_ = roirangefill::initGpu(gpu) == 0;
}


RoiRangeFill::~RoiRangeFill()
{
  // Release buffers first.
  gpu_corner_voxel_key_ = gputil::Buffer();
  gpu_region_keys_ = gputil::Buffer();
  gpu_occupancy_region_offsets_ = gputil::Buffer();
  gpu_region_clearance_buffer_ = gputil::Buffer();
  gpu_work_[0] = gputil::Buffer();
  gpu_work_[1] = gputil::Buffer();
  gpu_ = gputil::Device();
  roirangefill::releaseGpu();
}


bool RoiRangeFill::calculateForRegion(OccupancyMap &map, const glm::i16vec3 &region_key)
{
  PROFILE(RoiRangeFill);

  PROFILE(prime);

  // Calculate the voxel extents of the query. This size depends on the size of a single region plus the padding
  // required to ensure we reach the search range.
  const unsigned voxel_padding = unsigned(std::ceil(search_radius_ / map.resolution()));

  // Ensure cache is initialised.
  GpuCache *gpu_cache = initialiseGpuCache(map, GpuCache::kDefaultLayerMemSize, gpumap::kGpuAllowMappedBuffers);
  GpuLayerCache *occupancy_cache = gpu_cache->layerCache(kGcIdOccupancy);
  GpuLayerCache *clearance_cache = gpu_cache->layerCache(kGcIdClearance);

  // // Set the occupancyCache to read only mode. We need to copy query from it, but not back.
  // // This supports multiple threads.
  // GpuLayerCacheWriteLock occupancyWriteLock(*occupancyCache);

  // 1. voxelExtents contains the full range of the query.
  // 2. Determine the optimal buffer size to cover this range. This is based on GPU capabilities.
  const unsigned max_batch_size = std::min(occupancy_cache->cacheSize(), clearance_cache->cacheSize());
  const glm::ivec3 region_padding((voxel_padding + map.regionVoxelDimensions().x - 1) / map.regionVoxelDimensions().x,
                                  (voxel_padding + map.regionVoxelDimensions().y - 1) / map.regionVoxelDimensions().y,
                                  (voxel_padding + map.regionVoxelDimensions().z - 1) / map.regionVoxelDimensions().z);

  // Voxel grid we need to perform the calculations for.
  const glm::ivec3 batch_calc_extents(map.regionVoxelDimensions());
  // Voxel grid we need to upload (padded on the calc extents).
  const glm::ivec3 batch_voxel_extents = batch_calc_extents + 2 * glm::ivec3(voxel_padding);
  assert(map.regionVoxelDimensions().x <= batch_voxel_extents.x &&
          map.regionVoxelDimensions().y <= batch_voxel_extents.y &&
          map.regionVoxelDimensions().z <= batch_voxel_extents.z);

  const unsigned required_cache_size = volumeOf(glm::ivec3(1, 1, 1) + 2 * region_padding);
  // Must be able to support uploading 1 region plus enough padding regions to complete the query.
  if (max_batch_size < required_cache_size)
  {
    std::cerr << "RoiRangeFill: Gpu cache too small. Required " << required_cache_size
              << " supported: " << max_batch_size << std::endl;
    return false;
  }

  PROFILE_END(prime);
  PROFILE(gpuExec);
  std::vector<gputil::Event> upload_events;
  // Iterate the region grid, uploading the batch.
  const glm::ivec3 region_min = glm::ivec3(region_key) - region_padding;
  const glm::ivec3 region_max = glm::ivec3(region_key) + region_padding;

  // const glm::ivec3 batchMin(x, y, z);
  // const glm::ivec3 batchMax = batchMin + regionStep - glm::ivec3(1);
  //const unsigned occupancy_batch_marker = occupancy_cache->beginBatch();
  unsigned clearance_batch_marker = clearance_cache->beginBatch();

  // Set up the key marking the lower corner of the working group.
  const Key corner_voxel_key(region_key, glm::u8vec3(0));
  const GpuKey gpu_key = { corner_voxel_key.regionKey().x, corner_voxel_key.regionKey().y,
                          corner_voxel_key.regionKey().z, corner_voxel_key.localKey().x,
                          corner_voxel_key.localKey().y,  corner_voxel_key.localKey().z };
  gpu_corner_voxel_key_.write(&gpu_key, sizeof(gpu_key));

  PROFILE(upload);
  // Prepare region key and offset buffers.
  // Size the region buffers.
  gpu_region_keys_.elementsResize<gputil::int3>(volumeOf(glm::ivec3(1) + 2 * region_padding));
  gpu_occupancy_region_offsets_.elementsResize<gputil::ulong1>(
    volumeOf(glm::ivec3(1) + 2 * region_padding));

  gpu_region_clearance_buffer_.resize(
    map.layout().layer(kDlClearance).layerByteSize(map.regionVoxelDimensions()));

  gputil::PinnedBuffer region_keys(gpu_region_keys_, gputil::kPinWrite);
  gputil::PinnedBuffer occupancy_region_offsets(gpu_occupancy_region_offsets_, gputil::kPinWrite);
  region_count_ = 0;

  for (int z = region_min.z; z <= region_max.z; ++z)
  {
    for (int y = region_min.y; y <= region_max.y; ++y)
    {
      for (int x = region_min.x; x <= region_max.x; ++x)
      {
        const glm::ivec3 current_region_coord = glm::ivec3(x, y, z);
        MapChunk *chunk = nullptr;
        gputil::Event upload_event;
        GpuLayerCache::CacheStatus clearance_status;

        const unsigned gpu_cache_flags = GpuLayerCache::kSkipDownload;

        // Copy region occupancy voxels into the clearanceCache. This may come from either
        // a. The occupancyCache
        // b. Main memory.
        gputil::ulong1 clearance_mem_offset = 0u;
        size_t occupancy_mem_offset = 0u;

        gputil::Event occupancy_event;
        if (occupancy_cache->lookup(map, current_region_coord, &occupancy_mem_offset, &occupancy_event))
        {
          // Queue copying from occupancy cache.
          clearance_mem_offset = clearance_cache->allocate(
            map, current_region_coord, chunk, nullptr, &clearance_status, clearance_batch_marker, gpu_cache_flags);

          gputil::Buffer *occupancy_buffer = occupancy_cache->buffer();
          gputil::Buffer *clearance_buffer = clearance_cache->buffer();

          gputil::copyBuffer(*clearance_buffer, clearance_mem_offset, *occupancy_buffer, occupancy_mem_offset,
                              map.layout().layer(kDlOccupancy).layerByteSize(map.regionVoxelDimensions()),
                              &clearance_cache->gpuQueue(), &occupancy_event, &upload_event);
          clearance_cache->updateEvent(*chunk, upload_event);
        }
        else
        {
          // Copy from main memory.
          clearance_mem_offset = clearance_cache->upload(
            map, current_region_coord, chunk, &upload_event, &clearance_status, clearance_batch_marker, gpu_cache_flags);
        }
        assert(clearance_status != GpuLayerCache::kCacheFull);

        if (clearance_status != GpuLayerCache::kCacheFull)
        {
          upload_events.push_back(upload_event);
          region_keys.write(glm::value_ptr(current_region_coord), sizeof(current_region_coord),
                            region_count_ * sizeof(gputil::int3));
          occupancy_region_offsets.write(&clearance_mem_offset, sizeof(clearance_mem_offset),
                                        region_count_ * sizeof(gputil::ulong1));
          ++region_count_;
        }
        else
        {
          std::cerr << "RoiRangeFill: GPU cache full. Results invalid.\n" << std::flush;
          return false;
        }
      }
    }
  }

  PROFILE_END(upload)

  // All regions for this patch pushed. Make the calculation.
  // TODO: async unpin.
  region_keys.unpin();
  occupancy_region_offsets.unpin();
  finishRegion(region_key, map, *this, *gpu_cache, *clearance_cache, batch_voxel_extents, upload_events);
  upload_events.clear();

  return true;
}
