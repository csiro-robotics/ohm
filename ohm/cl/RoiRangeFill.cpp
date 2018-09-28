// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "private/RoiRangeFill.h"

#include "cl/clprogram.h"
#include "gpucache.h"
#include "gpulayercache.h"
#include "occupancyqueryflag.h"
#include "occupancyutil.h"
#include "private/occupancymapdetail.h"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <clu.h>
#include <clukernel.h>

#include <cl/gpuqueuedetail.h>

#include <mutex>

#ifdef OHM_EMBED_GPU_CODE
#include "roirangefillResource.h"
#endif // OHM_EMBED_GPU_CODE

#define KERNEL_PROFILING 0
#ifdef OHM_PROFILE
#define PROFILING 1
#endif // OHM_PROFILE
#include <profile.h>

using namespace ohm;

namespace
{
  std::mutex programMutex;
  cl::Program program;
  clu::Kernel seedKernel;
  clu::Kernel seedOuterKernel;
  clu::Kernel propagateKernel;
  clu::Kernel migrateKernel;
  int programRef = 0;
}

namespace roirangefill
{
  int initGpu(gputil::Device &gpu)
  {
    std::lock_guard<std::mutex> guard(programMutex);

    if (program())
    {
      // Already initialised.
      ++programRef;
      return CL_SUCCESS;
    }

    std::vector<std::string> buildArgs;
    buildArgs.push_back("-cl-std=CL" OHM_OPENCL_STD);

    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    const char *sourceFile = "roirangefill.cl";
  #ifdef OHM_EMBED_GPU_CODE
    clerr = initProgramFromString(program, gpu, roirangefillCode, sourceFile, &buildArgs);
  #else  // OHM_EMBED_GPU_CODE
    clerr = initProgramFromSource(program, gpu, sourceFile, &buildArgs);
  #endif // OHM_EMBED_GPU_CODE

    if (clerr != CL_SUCCESS)
    {
      return clerr;
    }

    clerr = seedKernel.setEntry(program, "seedRegionVoxels");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel seedRegionVoxels() in " << sourceFile << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    seedKernel.calculateOptimalWorkGroupSize();

    clerr = seedOuterKernel.setEntry(program, "seedFromOuterRegions");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel seedFromOuterRegions() in " << sourceFile << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    seedOuterKernel.calculateOptimalWorkGroupSize();

    clerr = propagateKernel.setEntry(program, "propagateObstacles");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel propagateObstacles() in " << sourceFile << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    // Add local voxels cache.
    propagateKernel.addLocal([] (size_t workgroupSize)
    {
      // Convert workgroupSize to a cubic dimension (conservatively) then add
      // padding of 1 either size. This forms the actual size, but ends up being
      // a conservative estimate of the memory requirement.
      const size_t cubicSize = (size_t)std::ceil(std::pow(double(workgroupSize), 1.0 / 3.0)) + 2;
      return sizeof(cl_char4) * cubicSize * cubicSize * cubicSize;
    });

    propagateKernel.calculateOptimalWorkGroupSize();

    clerr = migrateKernel.setEntry(program, "migrateResults");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel migrateResults() in " << sourceFile << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    migrateKernel.calculateOptimalWorkGroupSize();

    programRef = 1;
    return 0;
  }


  void releaseGpu()
  {
    // Release program.
    std::lock_guard<std::mutex> guard(programMutex);

    if (--programRef <= 0)
    {
      programRef = 0;
      seedKernel = clu::Kernel();
      propagateKernel = clu::Kernel();
      migrateKernel = clu::Kernel();
      program = cl::Program();
    }
  }


  int invoke(const OccupancyMapDetail &map,
             RoiRangeFill &query, GpuCache &gpuCache,
             GpuLayerCache &clearanceLayerCache,
             const glm::ivec3 &inputDataExtents,
             const std::vector<gputil::Event> &uploadEvents)
  {
    cl_int clerr = CL_SUCCESS;
    clu::KernelGrid grid;

    cl::CommandQueue &queue = gpuCache.gpuQueue().internal()->queue;

    // zbatch: how do we batch layers in Z to increase work per thread?
    const cl_int zbatch = 1;// std::max<int>(map.regionVoxelDimensions.z, 32);

    // Convert to CL types and inputs.
    // Region voxel dimensions
    const cl_int3 regionVoxelExtentsCL = { map.regionVoxelDimensions.x, map.regionVoxelDimensions.y, map.regionVoxelDimensions.z };
    // Padding voxel extents from ROI.
    const cl_int3 paddingCL = { (inputDataExtents.x - map.regionVoxelDimensions.x) / 2,
      (inputDataExtents.y - map.regionVoxelDimensions.y) / 2,
      (inputDataExtents.z - map.regionVoxelDimensions.z) / 2 };

    cl_float3 axisScalingCL = { query.axisScaling().x, query.axisScaling().y, query.axisScaling().z };

    PROFILE(seed);
    // For now just wait on the sync events here.
    // The alternative is to repack the events into kernelEvents via a cl::Event conversion.
    // Ultimately, I need to remove the use of the C++ OpenCL wrapper if I'm using gputil.
    gputil::Event::wait(uploadEvents.data(), uploadEvents.size());

    cl::Event seedKernelEvent, seedOuterKernelEvent;
    clu::EventList kernelEvents(nullptr, 0, &seedKernelEvent);

    int srcBufferIndex = 0;
    // Initial seeding is just a single region extents in X/Y, with Z divided by the batch size (round up to ensure coverage).
    const cl_int3 seedGrid = { regionVoxelExtentsCL.x, regionVoxelExtentsCL.y, (regionVoxelExtentsCL.z + zbatch - 1) / zbatch };
    calculateGrid(grid, seedKernel, query.gpu(), seedGrid);

    clerr = seedKernel(queue, grid, kernelEvents,
                      query.gpuCornerVoxelKey().arg<cl_mem>(),
                      clearanceLayerCache.buffer()->arg<cl_mem>(),
                      query.gpuWork(srcBufferIndex).arg<cl_mem>(),
                      query.gpuRegionKeys().arg<cl_mem>(),
                      query.gpuOccupancyRegionOffsets().arg<cl_mem>(),
                      query.regionCount(),
                      regionVoxelExtentsCL,
                      regionVoxelExtentsCL,
                      float(map.occupancyThresholdValue),
                      cl_uint(query.queryFlags()),
                      zbatch
                      );
    if (!clu::checkError(std::cerr, clerr, "queue seed obstacles"))
    {
      return clerr;
    }

    // Seed from data outside of the ROI.
    const cl_int seedOuterBatch = 32;
    const size_t paddingVolume = volumeOf(inputDataExtents) - volumeOf(map.regionVoxelDimensions);
    grid = clu::KernelGrid(clu::KernelSize((paddingVolume + seedOuterBatch - 1) / seedOuterBatch), clu::KernelSize(256));
    kernelEvents = clu::EventList(&seedKernelEvent, 1, &seedOuterKernelEvent);
    clerr = seedOuterKernel(queue, grid, kernelEvents,
                      query.gpuCornerVoxelKey().arg<cl_mem>(),
                      clearanceLayerCache.buffer()->arg<cl_mem>(),
                      query.gpuWork(srcBufferIndex).arg<cl_mem>(),
                      query.gpuRegionKeys().arg<cl_mem>(),
                      query.gpuOccupancyRegionOffsets().arg<cl_mem>(),
                      query.regionCount(),
                      regionVoxelExtentsCL,
                      regionVoxelExtentsCL,
                      paddingCL,
                      axisScalingCL,
                      float(map.occupancyThresholdValue),
                      cl_uint(query.queryFlags()),
                      seedOuterBatch
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

    calculateGrid(grid, propagateKernel, query.gpu(), regionVoxelExtentsCL);

    cl::Event previousEvent = seedOuterKernelEvent;
    cl::Event propagateEvent;

    const int propagationIterations = int(std::ceil(query.searchRadius() / map.resolution));
    // std::cout << "Iterations: " << propagationIterations << std::endl;
    for (int i = 0; i < propagationIterations; ++i)
    {
      kernelEvents = clu::EventList(&previousEvent, 1, &propagateEvent);
      clerr = propagateKernel(queue, grid, kernelEvents,
                              query.gpuWork(srcBufferIndex).arg<cl_mem>(),
                              query.gpuWork(1 - srcBufferIndex).arg<cl_mem>(),
                              regionVoxelExtentsCL,
                              float(query.searchRadius()),
                              axisScalingCL
                              // , __local char4 *localVoxels
                            );
      if (!clu::checkError(std::cerr, clerr, "queue propagate"))
      {
        return clerr;
      }

      previousEvent = propagateEvent;
      srcBufferIndex = 1 - srcBufferIndex;
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
    if (query.queryFlags() & QF_ReportUnscaledResults)
    {
      axisScalingCL = { 1, 1, 1 };
    }

    PROFILE(migrate);

    // Only queue migration kernel for the target region.
    calculateGrid(grid, migrateKernel, query.gpu(), regionVoxelExtentsCL);

    cl::Event migrateEvent;
    kernelEvents = clu::EventList(&previousEvent, 1, &migrateEvent);
    clerr = migrateKernel(queue, grid, kernelEvents,
                          query.gpuRegionClearanceBuffer().arg<cl_mem>(),
                          query.gpuWork(srcBufferIndex).arg<cl_mem>(),
                          regionVoxelExtentsCL,
                          regionVoxelExtentsCL,
                          float(query.searchRadius()),
                          float(map.resolution),
                          axisScalingCL,
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

    previousEvent = migrateEvent;
    previousEvent.wait();
    PROFILE_END(migrate);

    clerr = queue.finish();
    if (!clu::checkError(std::cerr, clerr, "ranges finish"))
    {
      return clerr;
    }

    return clerr;
  }
}
