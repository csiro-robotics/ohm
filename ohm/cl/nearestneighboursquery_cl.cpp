// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "occupancynearestneighbours.h"

#include "cl/clprogram.h"
#include "mapregion.h"
#include "occupancykey.h"
#include "occupancymap.h"
#include "occupancyqueryflag.h"
#include "private/occupancymapdetail.h"
#include "private/occupancynearestneighboursdetail.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <clu/clu.h>
#include <clu/clukernel.h>
#include <clu/cluprogram.h>

#include <gputil/cl/gpudevicedetail.h>
#include <gputil/cl/gpuqueuedetail.h>

#include <algorithm>
#include <functional>
#include <limits>
#include <mutex>
#include <sstream>

#ifdef OHM_EMBED_GPU_CODE
#include "nearestneighboursqueryResource.h"
#endif // OHM_EMBED_GPU_CODE

// See nearestneighbours.cl define of the same name.
// #define CACHE_LOCAL_RESULTS

#define SHOW_INFO 0

using namespace ohm;

namespace
{
  std::mutex programMutex;
  cl::Program program;
  clu::Kernel nnKernel;
  clu::Kernel infoKernel;
  int programRef = 0;
}

int initialiseNNGpuProgram(NearestNeighboursDetail &query, gputil::Device &gpu)
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

  const char *sourceFile = "nearestneighboursquery.cl";
#ifdef OHM_EMBED_GPU_CODE
  clerr = initProgramFromString(program, gpu, nearestneighboursqueryCode, sourceFile, &buildArgs);
#else  // OHM_EMBED_GPU_CODE
  clerr = initProgramFromSource(program, gpu, sourceFile, &buildArgs);
#endif // OHM_EMBED_GPU_CODE

  if (clerr != CL_SUCCESS)
  {
    return clerr;
  }

  clerr = nnKernel.setEntry(program, "nearestNeighbours");

  if (clerr != CL_SUCCESS)
  {
    program = cl::Program();
    std::cerr << "Failed to resolve kernel nearestNeighbours() in " << sourceFile << ": " << clu::errorCodeString(clerr) << '\n';
    return clerr;
  }

#ifdef CACHE_LOCAL_RESULTS
  // Add local ranges argument.
  nnKernel.addLocal([] (size_t workgroupSize)
  {
    return sizeof(float) * workgroupSize;
  });
  // Add region keys argument.
  nnKernel.addLocal([] (size_t workgroupSize)
  {
    return sizeof(cl_short3) * workgroupSize;
  });
  // Add local voxel keys argument.
  nnKernel.addLocal([] (size_t workgroupSize)
  {
    return sizeof(cl_uchar3) * workgroupSize;
  });
#endif // CACHE_LOCAL_RESULTS

  nnKernel.calculateOptimalWorkGroupSize();

  clerr = infoKernel.setEntry(program, "showNNInfo");

  if (clerr != CL_SUCCESS)
  {
    program = cl::Program();
    std::cerr << "Failed to resolve kernel showNNInfo() in " << sourceFile << ": " << clu::errorCodeString(clerr) << '\n';
    return clerr;
  }

  infoKernel.calculateOptimalWorkGroupSize();

  programRef = 1;
  return 0;
}


int invokeNNQueryGpu(const OccupancyMapDetail &map, NearestNeighboursDetail &query, NearestNeighboursDetail::GpuData &gpuData)
{
  cl_int clerr = CL_SUCCESS;
  clu::KernelGrid grid;
  grid.workGroupSize = nnKernel.calculateOptimalWorkGroupSize();
  if (grid.workGroupSize[0] > gpuData.queuedNodes)
  {
    grid.workGroupSize = gpuData.queuedNodes;
  }
  grid.globalSize = gpuData.queuedNodes;
  grid.globalSize = grid.adjustedGlobal();

  // Work in local coordinates on the GPU for better precision (double support not guaranteed, so we use single).
  const glm::dvec3 nearPointLocal = query.nearPoint - map.origin;
  const cl_float3 nearPointCL = { float(nearPointLocal.x), float(nearPointLocal.y), float(nearPointLocal.z), 0 };
  const cl_uchar3 voxelDimCL = { map.regionVoxelDimensions.x, map.regionVoxelDimensions.y, map.regionVoxelDimensions.z };
  const cl_float3 regionSpatialDimCL = { cl_float(map.regionSpatialDimensions.x), cl_float(map.regionSpatialDimensions.y), cl_float(map.regionSpatialDimensions.z) };

#if SHOW_INFO
  cl::Event kernelExecEvent;
  clu::EventList kernelEvents(nullptr, 0, &kernelExecEvent);
#else  // SHOW_INFO
  clu::EventList kernelEvents;
#endif // SHOW_INFO

  cl::CommandQueue &queue = gpuData.queue.internal()->queue;
  clerr = nnKernel(queue, grid, kernelEvents,
                   voxelDimCL,
                   regionSpatialDimCL,
                   gpuData.gpuNodes.arg<cl_mem>(),
                   gpuData.gpuNodeRegionKeys.arg<cl_mem>(),
                   gpuData.gpuNodeVoxelKeys.arg<cl_mem>(),
                   gpuData.gpuRanges.arg<cl_mem>(),
                   gpuData.gpuResulRegionKeys.arg<cl_mem>(),
                   gpuData.gpuResultNodeKeys.arg<cl_mem>(),
                   gpuData.gpuResultCount.arg<cl_mem>(),
                   nearPointCL,
                   float(query.searchRadius),
                   float(map.occupancyThresholdValue),
                   float(map.resolution),
                   cl_int((query.queryFlags & QF_UnknownAsOccupied) ? 1 : 0),
                   gpuData.queuedNodes
                   // , __local float *localRanges
                   // , __local short3 *localVoxelKeys
                   // , __local int3 *localRegionKeys
                  );

#if SHOW_INFO
  grid.workGroupSize = 1;
  grid.globalSize = 1;
  cl::Event infoWaitEvent = kernelExecEvent;
  cl::Event infoExecEvent;
  kernelEvents = clu::EventList(&infoWaitEvent, 1, &infoExecEvent);
  infoKernel(queue, grid, kernelEvents, gpuData.gpuResultCount.arg<cl_mem>());
#endif // SHOW_INFO

  return clerr;
}


void releaseNNGpu(NearestNeighboursDetail &query)
{
  std::lock_guard<std::mutex> guard(programMutex);

  if (--programRef <= 0)
  {
    programRef = 0;
    nnKernel = clu::Kernel();
    infoKernel = clu::Kernel();
    program = cl::Program();
  }
}
