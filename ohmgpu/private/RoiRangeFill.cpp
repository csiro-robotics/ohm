// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "RoiRangeFill.h"

#include "GpuCache.h"
#include "GpuKey.h"
#include "GpuLayerCache.h"
#include "GpuMap.h"

#include "private/GpuMapDetail.h"
#include "private/GpuProgramRef.h"

#include <ohm/DefaultLayer.h>
#include <ohm/Key.h>
#include <ohm/MapLayer.h>
#include <ohm/OccupancyMap.h>
#include <ohm/OccupancyUtil.h>
#include <ohm/QueryFlag.h>
#include <ohm/VoxelBlock.h>
#include <ohm/VoxelBuffer.h>
#include <ohm/private/OccupancyMapDetail.h>

#include <gputil/gpuPinnedBuffer.h>
#include <gputil/gpuPlatform.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <limits>

#define KERNEL_PROFILING 0
#include <ohmutil/Profile.h>

#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
#include "RoiRangeFillResource.h"
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL

#if GPUTIL_TYPE == GPUTIL_CUDA
GPUTIL_CUDA_DECLARE_KERNEL(seedRegionVoxels);
GPUTIL_CUDA_DECLARE_KERNEL(seedFromOuterRegions);
GPUTIL_CUDA_DECLARE_KERNEL(propagateObstacles);
GPUTIL_CUDA_DECLARE_KERNEL(migrateResults);
#endif  // GPUTIL_TYPE == GPUTIL_CUDA

using namespace ohm;

namespace
{
#if defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref("RoiRangeFill", GpuProgramRef::kSourceString,  // NOLINT
                            RoiRangeFillCode, RoiRangeFillCode_length);
#else   // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
  GpuProgramRef program_ref("RoiRangeFill", GpuProgramRef::kSourceFile, "RoiRangeFill.cl", 0u);
#endif  // defined(OHM_EMBED_GPU_CODE) && GPUTIL_TYPE == GPUTIL_OPENCL
}  // namespace

RoiRangeFill::RoiRangeFill(gputil::Device &gpu)
{
  gpu_ = gpu;

  // Initialise buffer to dummy size. We'll resize as required.
  gpu_corner_voxel_key_ = gputil::Buffer(gpu, sizeof(GpuKey), gputil::kBfReadHost);
  gpu_region_keys_ = gputil::Buffer(gpu, 32 * sizeof(gputil::int3), gputil::kBfReadHost);
  gpu_occupancy_region_offsets_ = gputil::Buffer(gpu, 32 * sizeof(uint64_t), gputil::kBfReadHost);
  // Place holder allocation:
  gpu_region_clearance_buffer_ = gputil::Buffer(gpu, 4, gputil::kBfReadHost);
  for (auto &gpu_work : gpu_work_)
  {
    gpu_work = gputil::Buffer(gpu, 1 * sizeof(gputil::char4), gputil::kBfReadWrite);
  }
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

  releaseGpuProgram();
}


bool RoiRangeFill::calculateForRegion(OccupancyMap &map, const glm::i16vec3 &region_key)
{
  PROFILE(RoiRangeFill);

  const int clearance_layer_index = map.layout().clearanceLayer();
  if (clearance_layer_index < 0)
  {
    return false;
  }

  PROFILE(prime);

  // Calculate the voxel extents of the query. This size depends on the size of a single region plus the padding
  // required to ensure we reach the search range.
  const unsigned voxel_padding = unsigned(std::ceil(search_radius_ / map.resolution()));

  // Ensure cache is initialised.
  GpuCache *gpu_cache = initialiseGpuCache(map, GpuCache::kDefaultTargetMemSize, gpumap::kGpuAllowMappedBuffers);
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
  // const unsigned occupancy_batch_marker = occupancy_cache->beginBatch();
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
  gpu_occupancy_region_offsets_.elementsResize<uint64_t>(volumeOf(glm::ivec3(1) + 2 * region_padding));

  gpu_region_clearance_buffer_.resize(
    map.layout().layer(clearance_layer_index).layerByteSize(map.regionVoxelDimensions()));

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
        uint64_t clearance_mem_offset = 0u;
        size_t occupancy_mem_offset = 0u;

        gputil::Event occupancy_event;
        if (occupancy_cache->lookup(map, current_region_coord, &occupancy_mem_offset, &occupancy_event))
        {
          // Queue copying from occupancy cache.
          clearance_mem_offset = clearance_cache->allocate(map, current_region_coord, chunk, nullptr, &clearance_status,
                                                           clearance_batch_marker, gpu_cache_flags);

          gputil::Buffer *occupancy_buffer = occupancy_cache->buffer();
          gputil::Buffer *clearance_buffer = clearance_cache->buffer();

          gputil::copyBuffer(
            *clearance_buffer, clearance_mem_offset, *occupancy_buffer, occupancy_mem_offset,
            map.layout().layer(map.layout().occupancyLayer()).layerByteSize(map.regionVoxelDimensions()),
            &clearance_cache->gpuQueue(), &occupancy_event, &upload_event);
          clearance_cache->updateEvent(*chunk, upload_event);
        }
        else
        {
          // Copy from main memory.
          clearance_mem_offset = clearance_cache->upload(map, current_region_coord, chunk, &upload_event,
                                                         &clearance_status, clearance_batch_marker, gpu_cache_flags);
        }
        assert(clearance_status != GpuLayerCache::kCacheFull);

        if (clearance_status != GpuLayerCache::kCacheFull)
        {
          upload_events.push_back(upload_event);
          region_keys.write(glm::value_ptr(current_region_coord), sizeof(current_region_coord),
                            region_count_ * sizeof(gputil::int3));
          occupancy_region_offsets.write(&clearance_mem_offset, sizeof(clearance_mem_offset),
                                         region_count_ * sizeof(uint64_t));
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
  // TODO(KS): async unpin.
  region_keys.unpin();
  occupancy_region_offsets.unpin();
  finishRegion(region_key, map, *this, *gpu_cache, *clearance_cache, batch_voxel_extents, upload_events);
  upload_events.clear();

  return true;
}


void RoiRangeFill::cacheGpuProgram(bool force)
{
  if (!force && program_ref_ != nullptr)
  {
    // Already loaded.
    return;
  }

  releaseGpuProgram();

  program_ref_ = &program_ref;
  if (program_ref_->addReference(gpu_))
  {
    seed_kernel_ = GPUTIL_MAKE_KERNEL(program_ref_->program(), seedRegionVoxels);
    seed_outer_kernel_ = GPUTIL_MAKE_KERNEL(program_ref_->program(), seedFromOuterRegions);
    propagate_kernel_ = GPUTIL_MAKE_KERNEL(program_ref_->program(), propagateObstacles);
    migrate_kernel_ = GPUTIL_MAKE_KERNEL(program_ref_->program(), migrateResults);

    if (!seed_kernel_.isValid() || !seed_outer_kernel_.isValid() || !propagate_kernel_.isValid() ||
        !migrate_kernel_.isValid())
    {
      releaseGpuProgram();
    }
    else
    {
      seed_kernel_.calculateOptimalWorkGroupSize();
      seed_outer_kernel_.calculateOptimalWorkGroupSize();
      // Add local voxels cache.
      propagate_kernel_.addLocal([](size_t workgroup_size) {
        // Convert workgroupSize to a cubic dimension (conservatively) then add
        // padding of 1 either size. This forms the actual size, but ends up being
        // a conservative estimate of the memory requirement.
        const size_t cubic_size = size_t(std::ceil(std::pow(double(workgroup_size), 1.0 / 3.0))) + 2;
        return sizeof(gputil::char4) * cubic_size * cubic_size * cubic_size;
      });

      propagate_kernel_.calculateOptimalWorkGroupSize();

      migrate_kernel_.calculateOptimalWorkGroupSize();
    }
  }
}


void RoiRangeFill::releaseGpuProgram()
{
  seed_kernel_ = gputil::Kernel();
  seed_outer_kernel_ = gputil::Kernel();
  propagate_kernel_ = gputil::Kernel();
  migrate_kernel_ = gputil::Kernel();

  if (program_ref_)
  {
    program_ref_->releaseReference();
  }
}


void RoiRangeFill::finishRegion(const glm::i16vec3 &region_key, OccupancyMap &map, RoiRangeFill &query,
                                GpuCache &gpu_cache, GpuLayerCache &clearance_cache,
                                const glm::ivec3 &batch_voxel_extents, const std::vector<gputil::Event> &upload_events)
{
  PROFILE(finishRegion);

  PROFILE(invoke);
  // Ensure sufficient working voxel memory size.
  for (int i = 0; i < 2; ++i)
  {
    query.gpuWork(i).elementsResize<gputil::char4>(volumeOf(batch_voxel_extents));
  }

  invoke(*map.detail(), query, gpu_cache, clearance_cache, batch_voxel_extents, upload_events);
  PROFILE_END(invoke);

  PROFILE(download);
  // Download back to main memory.
  MapChunk *region = map.region(region_key);
  if (region)
  {
    gputil::PinnedBuffer clearance_buffer(query.gpuRegionClearanceBuffer(), gputil::kPinRead);
    VoxelBuffer<VoxelBlock> voxels(region->voxel_blocks[map.layout().clearanceLayer()]);
    uint8_t *dst = voxels.voxelMemory();
    clearance_buffer.read(dst, voxels.voxelMemorySize());
  }
  PROFILE_END(download);
}


int RoiRangeFill::invoke(const OccupancyMapDetail &map, RoiRangeFill &query, GpuCache &gpu_cache,
                         GpuLayerCache &clearance_layer_cache, const glm::ivec3 &input_data_extents,
                         const std::vector<gputil::Event> &upload_events)
{
  int err = 0;

  // zbatch: how do we batch layers in Z to increase work per thread?
  const int zbatch = 1;  // std::max<int>(map.regionVoxelDimensions.z, 32);

  // Convert to CL types and inputs.
  // Region voxel dimensions
  const gputil::int3 region_voxel_extents_gpu = { map.region_voxel_dimensions.x, map.region_voxel_dimensions.y,
                                                  map.region_voxel_dimensions.z };
  // Padding voxel extents from ROI.
  const gputil::int3 padding_gpu = { (input_data_extents.x - map.region_voxel_dimensions.x) / 2,
                                     (input_data_extents.y - map.region_voxel_dimensions.y) / 2,
                                     (input_data_extents.z - map.region_voxel_dimensions.z) / 2 };

  gputil::float3 axis_scaling_gpu = { query.axisScaling().x, query.axisScaling().y, query.axisScaling().z };

  gputil::Queue &queue = gpu_cache.gpuQueue();

  // Select kernels based on voxel mean usage or not.
  cacheGpuProgram(false);

  PROFILE(seed);
  // For now just wait on the sync events here.
  // The alternative is to repack the events into kernelEvents via a cl::Event conversion.
  // Ultimately, I need to remove the use of the C++ OpenCL wrapper if I'm using gputil.
  gputil::Event::wait(upload_events.data(), upload_events.size());

  gputil::Event seed_kernel_event, seed_outer_kernel_event;

  unsigned kernel_algorithm_flags = 0;
  if (query.queryFlags() & kQfUnknownAsOccupied)
  {
    kernel_algorithm_flags |= 1;
  }

  int src_buffer_index = 0;
  // Initial seeding is just a single region extents in X/Y, with Z divided by the batch size (round up to ensure
  // coverage).
  const gputil::Dim3 seed_grid(size_t(region_voxel_extents_gpu.x), size_t(region_voxel_extents_gpu.y),
                               size_t((region_voxel_extents_gpu.z + zbatch - 1) / zbatch));

  gputil::Dim3 global_size, local_size;
  seed_kernel_.calculateGrid(&global_size, &local_size, seed_grid);

  err = seed_kernel_(
    global_size, local_size, seed_kernel_event, &queue,
    // Kernel arguments
    gputil::BufferArg<GpuKey>(gpu_corner_voxel_key_), gputil::BufferArg<float>(*clearance_layer_cache.buffer()),
    gputil::BufferArg<gputil::char4>(query.gpuWork(src_buffer_index)),
    gputil::BufferArg<gputil::int3>(query.gpuRegionKeys()),
    gputil::BufferArg<uint64_t>(query.gpuOccupancyRegionOffsets()), query.regionCount(), region_voxel_extents_gpu,
    region_voxel_extents_gpu, float(map.occupancy_threshold_value), kernel_algorithm_flags, zbatch);
  if (err)
  {
    return err;
  }

  // Seed from data outside of the ROI.
  const int seed_outer_batch = 32;
  const size_t padding_volume = volumeOf(input_data_extents) - volumeOf(map.region_voxel_dimensions);

  global_size = gputil::Dim3((padding_volume + seed_outer_batch - 1) / seed_outer_batch);
  local_size = gputil::Dim3(256);

  err = seed_outer_kernel_(
    global_size, local_size, gputil::EventList({ seed_kernel_event }), seed_outer_kernel_event, &queue,
    // Kernel arguments
    gputil::BufferArg<GpuKey>(query.gpuCornerVoxelKey()), gputil::BufferArg<float *>(*clearance_layer_cache.buffer()),
    gputil::BufferArg<gputil::char4>(query.gpuWork(src_buffer_index)),
    gputil::BufferArg<gputil::int3>(query.gpuRegionKeys()),
    gputil::BufferArg<uint64_t>(query.gpuOccupancyRegionOffsets()), query.regionCount(), region_voxel_extents_gpu,
    region_voxel_extents_gpu, padding_gpu, axis_scaling_gpu, float(map.occupancy_threshold_value),
    kernel_algorithm_flags, seed_outer_batch);

  if (err)
  {
    return err;
  }

  queue.flush();

  // #ifdef OHM_PROFILE
  //   seed_kernel_event.wait();
  // #endif // OHM_PROFILE
  PROFILE_END(seed);

  PROFILE(propagate);

  propagate_kernel_.calculateGrid(
    &global_size, &local_size,
    gputil::Dim3(region_voxel_extents_gpu.x, region_voxel_extents_gpu.y, region_voxel_extents_gpu.z));

  gputil::Event previous_event = seed_outer_kernel_event;
  gputil::Event propagate_event;

  const int propagation_iterations = int(std::ceil(query.searchRadius() / map.resolution));
  // std::cout << "Iterations: " << propagationIterations << std::endl;
  for (int i = 0; i < propagation_iterations; ++i)
  {
    err = propagate_kernel_(global_size, local_size, { previous_event }, propagate_event, &queue,
                            // Kernel args
                            gputil::BufferArg<gputil::char4>(query.gpuWork(src_buffer_index)),
                            gputil::BufferArg<gputil::char4>(query.gpuWork(1 - src_buffer_index)),
                            region_voxel_extents_gpu, float(query.searchRadius()), axis_scaling_gpu
                            // , __local char4 *localVoxels
    );

    if (err)
    {
      return err;
    }

    previous_event = propagate_event;
    src_buffer_index = 1 - src_buffer_index;

    queue.flush();
  }

  // #ifdef OHM_PROFILE
  //   previous_event.wait();
  // #endif // OHM_PROFILE

  PROFILE_END(propagate);
  if (query.queryFlags() & kQfReportUnscaledResults)
  {
    axis_scaling_gpu = { 1, 1, 1 };
  }

  PROFILE(migrate);

  // Only queue migration kernel for the target region.
  migrate_kernel_.calculateGrid(
    &global_size, &local_size,
    gputil::Dim3(region_voxel_extents_gpu.x, region_voxel_extents_gpu.y, region_voxel_extents_gpu.z));

  gputil::Event migrate_event;
  err = migrate_kernel_(global_size, local_size, gputil::EventList({ previous_event }), migrate_event, &queue,
                        // Kernel args
                        gputil::BufferArg<gputil::char4>(query.gpuRegionClearanceBuffer()),
                        gputil::BufferArg<gputil::char4>(query.gpuWork(src_buffer_index)), region_voxel_extents_gpu,
                        region_voxel_extents_gpu, float(query.searchRadius()), float(map.resolution), axis_scaling_gpu,
                        unsigned(query.queryFlags()));

  if (err)
  {
    return err;
  }

  queue.flush();

  previous_event = migrate_event;
  previous_event.wait();
  PROFILE_END(migrate);

  queue.finish();

  return err;
}
