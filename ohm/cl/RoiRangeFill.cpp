// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "private/RoiRangeFill.h"

#include "cl/clProgram.h"
#include "GpuCache.h"
#include "GpuLayerCache.h"
#include "QueryFlag.h"
#include "OccupancyUtil.h"
#include "private/OccupancyMapDetail.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <clu/clu.h>
#include <clu/cluKernel.h>

#include <gputil/cl/gpuQueueDetail.h>

#include <mutex>

#ifdef OHM_EMBED_GPU_CODE
#include "RoiRangeFillResource.h"
#endif // OHM_EMBED_GPU_CODE

#define KERNEL_PROFILING 0
#ifdef OHM_PROFILE
#define PROFILING 1
#endif // OHM_PROFILE
#include <ohmutil/Profile.h>

using namespace ohm;

namespace
{
  std::mutex program_mutex;
  cl::Program program;
  clu::Kernel seed_kernel;
  clu::Kernel seed_outer_kernel;
  clu::Kernel propagate_kernel;
  clu::Kernel migrate_kernel;
  int program_ref = 0;
}

namespace roirangefill
{
  int initGpu(gputil::Device &gpu)
  {
    std::lock_guard<std::mutex> guard(program_mutex);

    if (program())
    {
      // Already initialised.
      ++program_ref;
      return CL_SUCCESS;
    }

    std::vector<std::string> build_args;
    build_args.push_back("-cl-std=CL" OHM_OPENCL_STD);

    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    const char *source_file = "RoiRangeFill.cl";
  #ifdef OHM_EMBED_GPU_CODE
    clerr = initProgramFromString(program, gpu, RoiRangeFillCode, sourceFile, &build_args);
  #else  // OHM_EMBED_GPU_CODE
    clerr = initProgramFromSource(program, gpu, source_file, &build_args);
  #endif // OHM_EMBED_GPU_CODE

    if (clerr != CL_SUCCESS)
    {
      return clerr;
    }

    clerr = seed_kernel.setEntry(program, "seedRegionVoxels");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel seedRegionVoxels() in " << source_file << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    seed_kernel.calculateOptimalWorkGroupSize();

    clerr = seed_outer_kernel.setEntry(program, "seedFromOuterRegions");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel seedFromOuterRegions() in " << source_file << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    seed_outer_kernel.calculateOptimalWorkGroupSize();

    clerr = propagate_kernel.setEntry(program, "propagateObstacles");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel propagateObstacles() in " << source_file << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    // Add local voxels cache.
    propagate_kernel.addLocal([] (size_t workgroup_size)
    {
      // Convert workgroupSize to a cubic dimension (conservatively) then add
      // padding of 1 either size. This forms the actual size, but ends up being
      // a conservative estimate of the memory requirement.
      const size_t cubic_size = size_t(std::ceil(std::pow(double(workgroup_size), 1.0 / 3.0))) + 2;
      return sizeof(cl_char4) * cubic_size * cubic_size * cubic_size;
    });

    propagate_kernel.calculateOptimalWorkGroupSize();

    clerr = migrate_kernel.setEntry(program, "migrateResults");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel migrateResults() in " << source_file << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    migrate_kernel.calculateOptimalWorkGroupSize();

    program_ref = 1;
    return 0;
  }


  void releaseGpu()
  {
    // Release program.
    std::lock_guard<std::mutex> guard(program_mutex);

    if (--program_ref <= 0)
    {
      program_ref = 0;
      seed_kernel = clu::Kernel();
      propagate_kernel = clu::Kernel();
      migrate_kernel = clu::Kernel();
      program = cl::Program();
    }
  }


  int invoke(const OccupancyMapDetail &map,
             RoiRangeFill &query, GpuCache &gpu_cache,
             GpuLayerCache &clearance_layer_cache,
             const glm::ivec3 &input_data_extents,
             const std::vector<gputil::Event> &upload_events)
  {
    cl_int clerr = CL_SUCCESS;
    clu::KernelGrid grid;

    cl::CommandQueue &queue = gpu_cache.gpuQueue().internal()->queue;

    // zbatch: how do we batch layers in Z to increase work per thread?
    const cl_int zbatch = 1;// std::max<int>(map.regionVoxelDimensions.z, 32);

    // Convert to CL types and inputs.
    // Region voxel dimensions
    const cl_int3 region_voxel_extents_cl = { map.region_voxel_dimensions.x, map.region_voxel_dimensions.y, map.region_voxel_dimensions.z };
    // Padding voxel extents from ROI.
    const cl_int3 padding_cl = { (input_data_extents.x - map.region_voxel_dimensions.x) / 2,
      (input_data_extents.y - map.region_voxel_dimensions.y) / 2,
      (input_data_extents.z - map.region_voxel_dimensions.z) / 2 };

    cl_float3 axis_scaling_cl = { query.axisScaling().x, query.axisScaling().y, query.axisScaling().z };

    PROFILE(seed);
    // For now just wait on the sync events here.
    // The alternative is to repack the events into kernelEvents via a cl::Event conversion.
    // Ultimately, I need to remove the use of the C++ OpenCL wrapper if I'm using gputil.
    gputil::Event::wait(upload_events.data(), upload_events.size());

    cl::Event seed_kernel_event, seed_outer_kernel_event;
    clu::EventList kernel_events(nullptr, 0, &seed_kernel_event);

    int src_buffer_index = 0;
    // Initial seeding is just a single region extents in X/Y, with Z divided by the batch size (round up to ensure coverage).
    const cl_int3 seed_grid = { region_voxel_extents_cl.x, region_voxel_extents_cl.y, (region_voxel_extents_cl.z + zbatch - 1) / zbatch };
    calculateGrid(grid, seed_kernel, query.gpu(), seed_grid);

    clerr = seed_kernel(queue, grid, kernel_events,
                      query.gpuCornerVoxelKey().arg<cl_mem>(),
                      clearance_layer_cache.buffer()->arg<cl_mem>(),
                      query.gpuWork(src_buffer_index).arg<cl_mem>(),
                      query.gpuRegionKeys().arg<cl_mem>(),
                      query.gpuOccupancyRegionOffsets().arg<cl_mem>(),
                      query.regionCount(),
                      region_voxel_extents_cl,
                      region_voxel_extents_cl,
                      float(map.occupancy_threshold_value),
                      cl_uint(query.queryFlags()),
                      zbatch
                      );
    if (!clu::checkError(std::cerr, clerr, "queue seed obstacles"))
    {
      return clerr;
    }

    // Seed from data outside of the ROI.
    const cl_int seed_outer_batch = 32;
    const size_t padding_volume = volumeOf(input_data_extents) - volumeOf(map.region_voxel_dimensions);
    grid = clu::KernelGrid(clu::KernelSize((padding_volume + seed_outer_batch - 1) / seed_outer_batch), clu::KernelSize(256));
    kernel_events = clu::EventList(&seed_kernel_event, 1, &seed_outer_kernel_event);
    clerr = seed_outer_kernel(queue, grid, kernel_events,
                      query.gpuCornerVoxelKey().arg<cl_mem>(),
                      clearance_layer_cache.buffer()->arg<cl_mem>(),
                      query.gpuWork(src_buffer_index).arg<cl_mem>(),
                      query.gpuRegionKeys().arg<cl_mem>(),
                      query.gpuOccupancyRegionOffsets().arg<cl_mem>(),
                      query.regionCount(),
                      region_voxel_extents_cl,
                      region_voxel_extents_cl,
                      padding_cl,
                      axis_scaling_cl,
                      float(map.occupancy_threshold_value),
                      cl_uint(query.queryFlags()),
                      seed_outer_batch
                      );

    if (!clu::checkError(std::cerr, clerr, "queue seed outer obstacles"))
    {
      return clerr;
    }

    clerr = queue.flush();

    if (!clu::checkError(std::cerr, clerr, "flush seed obstacles"))
    {
      return clerr;
    }
  #ifdef OHM_PROFILE
    seedOuterKernelEvent.wait();
  #endif // OHM_PROFILE
    PROFILE_END(seed);

    PROFILE(propagate);

    calculateGrid(grid, propagate_kernel, query.gpu(), region_voxel_extents_cl);

    cl::Event previous_event = seed_outer_kernel_event;
    cl::Event propagate_event;

    const int propagation_iterations = int(std::ceil(query.searchRadius() / map.resolution));
    // std::cout << "Iterations: " << propagationIterations << std::endl;
    for (int i = 0; i < propagation_iterations; ++i)
    {
      kernel_events = clu::EventList(&previous_event, 1, &propagate_event);
      clerr = propagate_kernel(queue, grid, kernel_events,
                              query.gpuWork(src_buffer_index).arg<cl_mem>(),
                              query.gpuWork(1 - src_buffer_index).arg<cl_mem>(),
                              region_voxel_extents_cl,
                              float(query.searchRadius()),
                              axis_scaling_cl
                              // , __local char4 *localVoxels
                            );
      if (!clu::checkError(std::cerr, clerr, "queue propagate"))
      {
        return clerr;
      }

      previous_event = propagate_event;
      src_buffer_index = 1 - src_buffer_index;
      clerr = queue.flush();

      if (!clu::checkError(std::cerr, clerr, "flush propagate"))
      {
        return clerr;
      }
    }

  #ifdef OHM_PROFILE
    previousEvent.wait();
  #endif // OHM_PROFILE

    PROFILE_END(propagate);
    if (query.queryFlags() & kQfReportUnscaledResults)
    {
      axis_scaling_cl = { 1, 1, 1 };
    }

    PROFILE(migrate);

    // Only queue migration kernel for the target region.
    calculateGrid(grid, migrate_kernel, query.gpu(), region_voxel_extents_cl);

    cl::Event migrate_event;
    kernel_events = clu::EventList(&previous_event, 1, &migrate_event);
    clerr = migrate_kernel(queue, grid, kernel_events,
                          query.gpuRegionClearanceBuffer().arg<cl_mem>(),
                          query.gpuWork(src_buffer_index).arg<cl_mem>(),
                          region_voxel_extents_cl,
                          region_voxel_extents_cl,
                          float(query.searchRadius()),
                          float(map.resolution),
                          axis_scaling_cl,
                          cl_uint(query.queryFlags())
                        );

    if (!clu::checkError(std::cerr, clerr, "queue migrate results"))
    {
      return clerr;
    }

    clerr = queue.flush();
    if (!clu::checkError(std::cerr, clerr, "flush migrate results"))
    {
      return clerr;
    }

    previous_event = migrate_event;
    previous_event.wait();
    PROFILE_END(migrate);

    clerr = queue.finish();
    if (!clu::checkError(std::cerr, clerr, "ranges finish"))
    {
      return clerr;
    }

    return clerr;
  }
}
