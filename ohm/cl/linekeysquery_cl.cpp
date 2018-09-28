// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancylinekeysquery.h"

#include "cl/clprogram.h"
#include "mapregion.h"
#include "occupancymap.h"
#include "private/occupancylinekeysquerydetail.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <clu.h>
#include <clukernel.h>
#include <cluprogram.h>

#include <cl/gpudevicedetail.h>
#include <cl/gpuqueuedetail.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <mutex>
#include <sstream>

#ifdef OHM_EMBED_GPU_CODE
#include "linekeysResource.h"
#endif // OHM_EMBED_GPU_CODE

using namespace ohm;

namespace
{
  std::mutex programMutex;
  cl::Program program;
  clu::Kernel lineKeysKernel;
  int programRef = 0;
}


int initialiseLineKeysGpuProgram(LineKeysQueryDetail &query, gputil::Device &gpu)
{
  if (!gpu.isValid())
  {
    return CL_DEVICE_NOT_AVAILABLE;
  }

  std::lock_guard<std::mutex> guard(programMutex);

  query.gpu = gpu;

  if (program())
  {
    // Already initialised.
    ++programRef;
    return CL_SUCCESS;
  }

  // Compile and initialise.
  cl_int clerr = CL_SUCCESS;
  std::vector<std::string> buildArgs;
#ifdef CACHE_LOCAL_RESULTS
  buildArgs.push_back("-D CACHE_LOCAL_RESULTS");
#endif // CACHE_LOCAL_RESULTS
#ifdef VALIDATE_KEYS
  buildArgs.push_back("-D VALIDATE_KEYS");
#endif // VALIDATE_KEYS

  buildArgs.push_back("-cl-std=CL" OHM_OPENCL_STD);
  buildArgs.push_back("-Werror");

  const char *sourceFile = "linekeys.cl";
#ifdef OHM_EMBED_GPU_CODE
  clerr = initProgramFromString(program, gpu, linekeysCode, sourceFile, &buildArgs);
#else  // OHM_EMBED_GPU_CODE
  clerr = initProgramFromSource(program, gpu, sourceFile, &buildArgs);
#endif // OHM_EMBED_GPU_CODE

  if (clerr != CL_SUCCESS)
  {
    return clerr;
  }

  clerr = lineKeysKernel.setEntry(program, "calculateLines");

  if (clerr != CL_SUCCESS)
  {
    program = cl::Program();
    std::cerr << "Failed to resolve kernel calculateLines() in " << sourceFile << ": " << clu::errorCodeString(clerr) << '\n';
    return clerr;
  }

  lineKeysKernel.calculateOptimalWorkGroupSize();

  programRef = 1;
  return 0;
}


int invokeLineKeysQueryGpu(LineKeysQueryDetail &query, LineKeysQueryDetail::GpuData &gpuData, bool (*completionFunc)(LineKeysQueryDetail &))
{
  cl_int clerr = CL_SUCCESS;
  clu::KernelGrid grid;

  grid.workGroupSize = lineKeysKernel.calculateOptimalWorkGroupSize();
  grid.globalSize = query.rays.size() / 2;
  if (grid.workGroupSize[0] > query.rays.size() / 2)
  {
    grid.workGroupSize = query.rays.size() / 2;
  }
  grid.globalSize = grid.adjustedGlobal();

  //cl_float3 mapOrigin = { (float)query.map->origin().x, (float)query.map->origin().y, (float)query.map->origin().z };
  cl_int3 regionDim = { query.map->regionVoxelDimensions().x, query.map->regionVoxelDimensions().y, query.map->regionVoxelDimensions().z };

  // Ensure all memory transfers have completed.
  gpuData.queue.insertBarrier();
  cl::CommandQueue &queue = gpuData.queue.internal()->queue;
  clerr = lineKeysKernel(queue, grid, clu::EventList(),
                         gpuData.linesOut.arg<cl_mem>(),
                         gpuData.maxKeysPerLine,
                         gpuData.linePoints.arg<cl_mem>(),
                         (cl_uint)(query.rays.size() / 2),
                         regionDim,
                         (float)query.map->resolution()
                  );

  if (clerr)
  {
    clu::checkError(std::cerr, clerr, "calculateLineKeys()");
    return clerr;
  }

  if (completionFunc)
  {
    auto callback = [completionFunc, &query] ()
    {
      completionFunc(query);
    };

    // Queue up the completion function.
    gpuData.queue.queueCallback(callback);
  }

  clerr = queue.flush();

  if (clerr)
  {
    clu::checkError(std::cerr, clerr, "calculateLineKeys - flush");
    return clerr;
  }

  return clerr;
}


void releaseLineKeysGpu(LineKeysQueryDetail &query)
{
  std::lock_guard<std::mutex> guard(programMutex);

  if (--programRef <= 0)
  {
    programRef = 0;
    lineKeysKernel = clu::Kernel();
    program = cl::Program();
  }
}
