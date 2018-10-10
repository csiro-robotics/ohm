// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "LineKeysQuery.h"

#include "cl/clProgram.h"
#include "MapRegion.h"
#include "OccupancyMap.h"
#include "private/LineKeysQueryDetail.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <clu/clu.h>
#include <clu/cluKernel.h>

#include <gputil/cl/gpuDeviceDetail.h>
#include <gputil/cl/gpuQueueDetail.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <mutex>
#include <sstream>

#ifdef OHM_EMBED_GPU_CODE
#include "LineKeysResource.h"
#endif // OHM_EMBED_GPU_CODE

using namespace ohm;

namespace
{
  std::mutex program_mutex;
  cl::Program program;
  clu::Kernel line_keys_kernel;
  int program_ref = 0;
}


int initialiseLineKeysGpuProgram(LineKeysQueryDetail &query, gputil::Device &gpu)
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

  build_args.push_back("-cl-std=CL" OHM_OPENCL_STD);
  build_args.push_back("-Werror");

  const char *source_file = "LineKeys.cl";
#ifdef OHM_EMBED_GPU_CODE
  clerr = initProgramFromString(program, gpu, LineKeysCode, source_file, &build_args);
#else  // OHM_EMBED_GPU_CODE
  clerr = initProgramFromSource(program, gpu, source_file, &build_args);
#endif // OHM_EMBED_GPU_CODE

  if (clerr != CL_SUCCESS)
  {
    return clerr;
  }

  clerr = line_keys_kernel.setEntry(program, "calculateLines");

  if (clerr != CL_SUCCESS)
  {
    program = cl::Program();
    std::cerr << "Failed to resolve kernel calculateLines() in " << source_file << ": " << clu::errorCodeString(clerr) << '\n';
    return clerr;
  }

  line_keys_kernel.calculateOptimalWorkGroupSize();

  program_ref = 1;
  return 0;
}


int invokeLineKeysQueryGpu(LineKeysQueryDetail &query, LineKeysQueryDetail::GpuData &gpu_data, bool (*completion_func)(LineKeysQueryDetail &))
{
  cl_int clerr = CL_SUCCESS;
  clu::KernelGrid grid;

  grid.work_group_size = line_keys_kernel.calculateOptimalWorkGroupSize();
  grid.global_size = query.rays.size() / 2;
  if (grid.work_group_size[0] > query.rays.size() / 2)
  {
    grid.work_group_size = query.rays.size() / 2;
  }
  grid.global_size = grid.adjustedGlobal();

  //cl_float3 mapOrigin = { (float)query.map->origin().x, (float)query.map->origin().y, (float)query.map->origin().z };
  const cl_int3 region_dim = { query.map->regionVoxelDimensions().x, query.map->regionVoxelDimensions().y, query.map->regionVoxelDimensions().z };

  // Ensure all memory transfers have completed.
  gpu_data.queue.insertBarrier();
  cl::CommandQueue &queue = gpu_data.queue.internal()->queue;
  clerr = line_keys_kernel(queue, grid, clu::EventList(),
                         gpu_data.linesOut.arg<cl_mem>(),
                         gpu_data.maxKeysPerLine,
                         gpu_data.linePoints.arg<cl_mem>(),
                         cl_uint(query.rays.size() / 2),
                         region_dim,
                         float(query.map->resolution())
                  );

  if (clerr)
  {
    clu::checkError(std::cerr, clerr, "calculateLineKeys()");
    return clerr;
  }

  if (completion_func)
  {
    auto callback = [completion_func, &query] ()
    {
      completion_func(query);
    };

    // Queue up the completion function.
    gpu_data.queue.queueCallback(callback);
  }

  clerr = queue.flush();

  if (clerr)
  {
    clu::checkError(std::cerr, clerr, "calculateLineKeys - flush");
    return clerr;
  }

  return clerr;
}


void releaseLineKeysGpu(LineKeysQueryDetail &/*query*/)
{
  std::lock_guard<std::mutex> guard(program_mutex);

  if (--program_ref <= 0)
  {
    program_ref = 0;
    line_keys_kernel = clu::Kernel();
    program = cl::Program();
  }
}
