// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RoiRangeFill.h"

#include "cl/clprogram.h"
#include "gpucache.h"
#include "gpukey.h"
#include "gpulayercache.h"
#include "maplayer.h"
#include "occupancykey.h"
#include "occupancymap.h"
#include "occupancyutil.h"
#include "ohmdefaultlayers.h"
#include "private/occupancymapdetail.h"
#include "private/occupancygpumapdetail.h"

#include <gputil/gpuplatform.h>
#include <gputil/gpupinnedbuffer.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <limits>

#define KERNEL_PROFILING 0
#ifdef OHM_PROFILE
#define PROFILING 1
#endif // OHM_PROFILE
#include <ohmutil/profile.h>

using namespace ohm;

namespace roirangefill
{
  int initGpu(gputil::Device &gpu);
  void releaseGpu();
  int invoke(const OccupancyMapDetail &map,
             RoiRangeFill &query, GpuCache &gpuCache,
             GpuLayerCache &clearanceLayerCache,
             const glm::ivec3 &inputDataExtents,
             const std::vector<gputil::Event> &uploadEvents);
}


namespace
{
  void finishRegion(const glm::i16vec3 &regionKey, OccupancyMap &map, RoiRangeFill &query, GpuCache &gpuCache,
                    GpuLayerCache &clearanceCache, const glm::ivec3 &batchVoxelExtents,
                    const std::vector<gputil::Event> &uploadEvents)
  {
    PROFILE(finishRegion);

    PROFILE(invoke);
    // Ensure sufficient working voxel memory size.
    for (int i = 0; i < 2; ++i)
    {
      query.gpuWork(i).elementsResize<gputil::char4>(volumeOf(batchVoxelExtents));
    }

    const int iterations = int(std::ceil(query.searchRadius() / map.resolution()));
    roirangefill::invoke(*map.detail(), query, gpuCache, clearanceCache, batchVoxelExtents, uploadEvents);
    PROFILE_END(invoke);

    PROFILE(download);
    // Download back to main memory.
    MapChunk *region = map.region(regionKey);
    if (region)
    {
      gputil::PinnedBuffer clearanceBuffer(query.gpuRegionClearanceBuffer(), gputil::PinRead);
      const MapLayer &clearanceLayer = map.layout().layer(DL_Clearance);
      uint8_t *dst = clearanceLayer.voxels(*region);
      clearanceBuffer.read(dst, clearanceLayer.layerByteSize(map.regionVoxelDimensions()));
    }
    PROFILE_END(download);
  }
}


RoiRangeFill::RoiRangeFill(gputil::Device &gpu)
{
  gpu_ = gpu;

  // Initialise buffer to dummy size. We'll resize as required.
  gpu_corner_voxel_key_ = gputil::Buffer(gpu, sizeof(GpuKey), gputil::BF_ReadHost);
  gpu_region_keys_ = gputil::Buffer(gpu, 32 * sizeof(gputil::int3), gputil::BF_ReadHost);
  gpu_occupancy_region_offsets_ = gputil::Buffer(gpu, 32 * sizeof(gputil::ulong1), gputil::BF_ReadHost);
  // Place holder allocation:
  gpu_region_clearance_buffer_ = gputil::Buffer(gpu, 4, gputil::BF_ReadHost);
  for (int i = 0; i < 2; ++i)
  {
    gpu_work_[i] = gputil::Buffer(gpu, 1 * sizeof(gputil::char4), gputil::BF_ReadWrite);
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


bool RoiRangeFill::calculateForRegion(OccupancyMap &map, const glm::i16vec3 &regionKey)
{
  PROFILE(RoiRangeFill);

  PROFILE(prime);

  // Calculate the voxel extents of the query. This size depends on the size of a single region plus the padding
  // required to ensure we reach the search range.
  const unsigned voxelPadding = unsigned(std::ceil(search_radius_ / map.resolution()));

  // Ensure cache is initialised.
  GpuCache *gpuCache = initialiseGpuCache(map, GpuCache::DefaultLayerMemSize, true);
  GpuLayerCache *occupancyCache = gpuCache->layerCache(GCID_Occupancy);
  GpuLayerCache *clearanceCache = gpuCache->layerCache(GCID_Clearance);

  // // Set the occupancyCache to read only mode. We need to copy query from it, but not back.
  // // This supports multiple threads.
  // GpuLayerCacheWriteLock occupancyWriteLock(*occupancyCache);

  // 1. voxelExtents contains the full range of the query.
  // 2. Determine the optimal buffer size to cover this range. This is based on GPU capabilities.
  const unsigned maxBatchSize = std::min(occupancyCache->cacheSize(), clearanceCache->cacheSize());
  const glm::ivec3 regionPadding((voxelPadding + map.regionVoxelDimensions().x - 1) / map.regionVoxelDimensions().x,
                                  (voxelPadding + map.regionVoxelDimensions().y - 1) / map.regionVoxelDimensions().y,
                                  (voxelPadding + map.regionVoxelDimensions().z - 1) / map.regionVoxelDimensions().z);

  // Voxel grid we need to perform the calculations for.
  const glm::ivec3 batchCalcExtents(map.regionVoxelDimensions());
  // Voxel grid we need to upload (padded on the calc extents).
  const glm::ivec3 batchVoxelExtents = batchCalcExtents + 2 * glm::ivec3(voxelPadding);
  assert(map.regionVoxelDimensions().x <= batchVoxelExtents.x &&
          map.regionVoxelDimensions().y <= batchVoxelExtents.y &&
          map.regionVoxelDimensions().z <= batchVoxelExtents.z);

  const unsigned requiredCacheSize = volumeOf(glm::ivec3(1, 1, 1) + 2 * regionPadding);
  // Must be able to support uploading 1 region plus enough padding regions to complete the query.
  if (maxBatchSize < requiredCacheSize)
  {
    std::cerr << "RoiRangeFill: Gpu cache too small. Required " << requiredCacheSize
              << " supported: " << maxBatchSize << std::endl;
    return false;
  }

  PROFILE_END(prime);
  PROFILE(gpuExec);
  std::vector<gputil::Event> uploadEvents;
  // Iterate the region grid, uploading the batch.
  glm::ivec3 regionMin = glm::ivec3(regionKey) - regionPadding;
  glm::ivec3 regionMax = glm::ivec3(regionKey) + regionPadding;

  // const glm::ivec3 batchMin(x, y, z);
  // const glm::ivec3 batchMax = batchMin + regionStep - glm::ivec3(1);
  const unsigned occupancyBatchMarker = occupancyCache->beginBatch();
  unsigned clearanceBatchMarker = clearanceCache->beginBatch();

  // Set up the key marking the lower corner of the working group.
  const OccupancyKey cornerVoxelKey(regionKey, glm::u8vec3(0));
  const GpuKey gpuKey = { cornerVoxelKey.regionKey().x, cornerVoxelKey.regionKey().y,
                          cornerVoxelKey.regionKey().z, cornerVoxelKey.localKey().x,
                          cornerVoxelKey.localKey().y,  cornerVoxelKey.localKey().z };
  gpu_corner_voxel_key_.write(&gpuKey, sizeof(gpuKey));

  PROFILE(upload);
  // Prepare region key and offset buffers.
  // Size the region buffers.
  gpu_region_keys_.elementsResize<gputil::int3>(volumeOf(glm::ivec3(1) + 2 * regionPadding));
  gpu_occupancy_region_offsets_.elementsResize<gputil::ulong1>(
    volumeOf(glm::ivec3(1) + 2 * regionPadding));

  gpu_region_clearance_buffer_.resize(
    map.layout().layer(DL_Clearance).layerByteSize(map.regionVoxelDimensions()));

  gputil::PinnedBuffer regionKeys(gpu_region_keys_, gputil::PinWrite);
  gputil::PinnedBuffer occupancyRegionOffsets(gpu_occupancy_region_offsets_, gputil::PinWrite);
  region_count_ = 0;

  for (int z = regionMin.z; z <= regionMax.z; ++z)
  {
    for (int y = regionMin.y; y <= regionMax.y; ++y)
    {
      for (int x = regionMin.x; x <= regionMax.x; ++x)
      {
        const glm::ivec3 currentRegionCoord = glm::ivec3(x, y, z);
        MapChunk *chunk = nullptr;
        gputil::Event uploadEvent;
        GpuLayerCache::CacheStatus clearanceStatus;

        const unsigned gpuCacheFlags = GpuLayerCache::SkipDownload;

        // Copy region occupancy voxels into the clearanceCache. This may come from either
        // a. The occupancyCache
        // b. Main memory.
        gputil::ulong1 clearanceMemOffset = 0u;
        size_t occupancyMemOffset = 0u;

        gputil::Event occupancyEvent;
        if (occupancyCache->lookup(map, currentRegionCoord, &occupancyMemOffset, &occupancyEvent))
        {
          // Queue copying from occupancy cache.
          clearanceMemOffset = clearanceCache->allocate(
            map, currentRegionCoord, chunk, nullptr, &clearanceStatus, occupancyBatchMarker, gpuCacheFlags);

          gputil::Buffer *occupancyBuffer = occupancyCache->buffer();
          gputil::Buffer *clearanceBuffer = clearanceCache->buffer();

          gputil::copyBuffer(*clearanceBuffer, clearanceMemOffset, *occupancyBuffer, occupancyMemOffset,
                              map.layout().layer(DL_Occupancy).layerByteSize(map.regionVoxelDimensions()),
                              &clearanceCache->gpuQueue(), &occupancyEvent, &uploadEvent);
          clearanceCache->updateEvent(*chunk, uploadEvent);
        }
        else
        {
          // Copy from main memory.
          clearanceMemOffset = clearanceCache->upload(
            map, currentRegionCoord, chunk, &uploadEvent, &clearanceStatus, occupancyBatchMarker, gpuCacheFlags);
        }
        assert(clearanceStatus != GpuLayerCache::CacheFull);

        if (clearanceStatus != GpuLayerCache::CacheFull)
        {
          uploadEvents.push_back(uploadEvent);
          regionKeys.write(glm::value_ptr(currentRegionCoord), sizeof(currentRegionCoord),
                            region_count_ * sizeof(gputil::int3));
          occupancyRegionOffsets.write(&clearanceMemOffset, sizeof(clearanceMemOffset),
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
  regionKeys.unpin();
  occupancyRegionOffsets.unpin();
  finishRegion(regionKey, map, *this, *gpuCache, *clearanceCache, batchVoxelExtents, uploadEvents);
  uploadEvents.clear();

  return true;
}
