// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "NearestNeighbours.h"

#include "cl/clProgram.h"
#include "QueryFlag.h"
#include "private/OccupancyMapDetail.h"
#include "private/NearestNeighboursDetail.h"
#include "OhmGpu.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <clu/clu.h>
#include <clu/cluKernel.h>

#include <gputil/cl/gpuQueueDetail.h>

#include <functional>
#include <mutex>

#ifdef OHM_EMBED_GPU_CODE
#include "NearestNeighboursQueryResource.h"
#endif // OHM_EMBED_GPU_CODE

// See nearestneighbours.cl define of the same name.
// #define CACHE_LOCAL_RESULTS

#define SHOW_INFO 0

using namespace ohm;

namespace
{
  std::mutex program_mutex;
  cl::Program program;
  clu::Kernel nn_kernel;
  clu::Kernel info_kernel;
  int program_ref = 0;
}

int initialiseNnGpuProgram(NearestNeighboursDetail &query, gputil::Device &gpu)
{
  if (!gpu.isValid())
  {
    return CL_DEVICE_NOT_AVAILABLE;
  }

  std::lock_guard<std::mutex> guard(program_mutex);

  query.gpu = gpu;

  if (program())
  {
    // Already initialised.
    ++program_ref;
    return CL_SUCCESS;
  }

  // Compile and initialise.
  cl_int clerr = CL_SUCCESS;
  std::vector<std::string> build_args;
#ifdef CACHE_LOCAL_RESULTS
  buildArgs.push_back("-D CACHE_LOCAL_RESULTS");
#endif // CACHE_LOCAL_RESULTS
#ifdef VALIDATE_KEYS
  buildArgs.push_back("-D VALIDATE_KEYS");
#endif // VALIDATE_KEYS

  build_args.push_back(ohm::gpuBuildStdArg());

  const char *source_file = "NearestNeighboursQuery.cl";
#ifdef OHM_EMBED_GPU_CODE
  clerr = initProgramFromString(program, gpu, NearestNeighboursQueryCode, source_file, &build_args);
#else  // OHM_EMBED_GPU_CODE
  clerr = initProgramFromSource(program, gpu, source_file, &build_args);
#endif // OHM_EMBED_GPU_CODE

  if (clerr != CL_SUCCESS)
  {
    return clerr;
  }

  clerr = nn_kernel.setEntry(program, "nearestNeighbours");

  if (clerr != CL_SUCCESS)
  {
    program = cl::Program();
    std::cerr << "Failed to resolve kernel nearestNeighbours() in " << source_file << ": " << clu::errorCodeString(clerr) << '\n';
    return clerr;
  }

#ifdef CACHE_LOCAL_RESULTS
  // Add local ranges argument.
  nn_kernel.addLocal([] (size_t workgroupSize)
  {
    return sizeof(float) * workgroupSize;
  });
  // Add region keys argument.
  nn_kernel.addLocal([] (size_t workgroupSize)
  {
    return sizeof(cl_short3) * workgroupSize;
  });
  // Add local voxel keys argument.
  nn_kernel.addLocal([] (size_t workgroupSize)
  {
    return sizeof(cl_uchar3) * workgroupSize;
  });
#endif // CACHE_LOCAL_RESULTS

  nn_kernel.calculateOptimalWorkGroupSize();

  clerr = info_kernel.setEntry(program, "showNNInfo");

  if (clerr != CL_SUCCESS)
  {
    program = cl::Program();
    std::cerr << "Failed to resolve kernel showNNInfo() in " << source_file << ": " << clu::errorCodeString(clerr) << '\n';
    return clerr;
  }

  info_kernel.calculateOptimalWorkGroupSize();

  program_ref = 1;
  return 0;
}


int invokeNnQueryGpu(const OccupancyMapDetail &map, NearestNeighboursDetail &query, NearestNeighboursDetail::GpuData &gpu_data)
{
  cl_int clerr = CL_SUCCESS;
  clu::KernelGrid grid;
  grid.work_group_size = nn_kernel.calculateOptimalWorkGroupSize();
  if (grid.work_group_size[0] > gpu_data.queued_nodes)
  {
    grid.work_group_size = gpu_data.queued_nodes;
  }
  grid.global_size = gpu_data.queued_nodes;
  grid.global_size = grid.adjustedGlobal();

  // Work in local coordinates on the GPU for better precision (double support not guaranteed, so we use single).
  const glm::dvec3 near_point_local = query.near_point - map.origin;
  const cl_float3 near_point_cl = { float(near_point_local.x), float(near_point_local.y), float(near_point_local.z), 0 };
  const cl_uchar3 voxel_dim_cl = { map.region_voxel_dimensions.x, map.region_voxel_dimensions.y, map.region_voxel_dimensions.z };
  const cl_float3 region_spatial_dim_cl = { cl_float(map.region_spatial_dimensions.x), cl_float(map.region_spatial_dimensions.y), cl_float(map.region_spatial_dimensions.z) };

#if SHOW_INFO
  cl::Event kernelExecEvent;
  clu::EventList kernelEvents(nullptr, 0, &kernelExecEvent);
#else  // SHOW_INFO
  clu::EventList kernel_events;
#endif // SHOW_INFO

  cl::CommandQueue &queue = gpu_data.queue.internal()->queue;
  clerr = nn_kernel(queue, grid, kernel_events,
                   voxel_dim_cl,
                   region_spatial_dim_cl,
                   gpu_data.gpu_nodes.arg<cl_mem>(),
                   gpu_data.gpu_node_region_keys.arg<cl_mem>(),
                   gpu_data.gpu_node_voxel_keys.arg<cl_mem>(),
                   gpu_data.gpu_ranges.arg<cl_mem>(),
                   gpu_data.gpu_result_region_keys.arg<cl_mem>(),
                   gpu_data.gpu_result_node_keys.arg<cl_mem>(),
                   gpu_data.gpu_result_count.arg<cl_mem>(),
                   near_point_cl,
                   float(query.search_radius),
                   float(map.occupancy_threshold_value),
                   float(map.resolution),
                   cl_int((query.query_flags & kQfUnknownAsOccupied) ? 1 : 0),
                   gpu_data.queued_nodes
                   // , __local float *localRanges
                   // , __local short3 *localVoxelKeys
                   // , __local int3 *localRegionKeys
                  );

#if SHOW_INFO
  grid.work_group_size = 1;
  grid.global_size= 1;
  cl::Event info_wait_event = kernel_exec_event;
  cl::Event info_exec_event;
  kernel_events = clu::EventList(&info_wait_event, 1, &info_exec_event);
  info_kernel(queue, grid, kernel_events, gpu_data.gpuResultCount.arg<cl_mem>());
#endif // SHOW_INFO

  return clerr;
}


void releaseNnGpu(NearestNeighboursDetail &/*query*/)
{
  std::lock_guard<std::mutex> guard(program_mutex);

  if (--program_ref <= 0)
  {
    program_ref = 0;
    nn_kernel = clu::Kernel();
    info_kernel = clu::Kernel();
    program = cl::Program();
  }
}
