// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "ohmconfig.h"

#include "cl/clprogram.h"
#include "private/occupancygpumapdetail.h"

#include <gpuplatform.h>

#ifdef OHM_PROFILE
#include "occupancyutil.h"
#include "ohmutil.h"
#endif //  OHM_PROFILE
#include "ohmutil.h"

#include <gpudevice.h>
#include <gpuqueue.h>

#include <clu.h>
#include <clukernel.h>
#include <cluprogram.h>

#include <cl/gpudevicedetail.h>
#include <cl/gpueventdetail.h>
#include <cl/gpuqueuedetail.h>

#include <algorithm>
#include <mutex>
#include <initializer_list>

#ifdef OHM_EMBED_GPU_CODE
#include "regionupdateResource.h"
#endif // OHM_EMBED_GPU_CODE

namespace
{
  std::mutex programMutex;
  cl::Program program;
  clu::Kernel updateKernel;
  int programRef = 0;
}

namespace ohm
{
  int initialiseRegionUpdateGpu(gputil::Device &gpu)
  {
    if (!gpu.isValid())
    {
      std::cerr << "No GPU initialised\n" << std::flush;
      return CL_DEVICE_NOT_AVAILABLE;
    }

    std::lock_guard<std::mutex> guard(programMutex);

    if (program())
    {
      // Already initialised.
      ++programRef;
      return CL_SUCCESS;
    }

    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    const char *sourceFile = "regionupdate.cl";
    std::vector<std::string> args;
    args.push_back("-cl-std=CL" OHM_OPENCL_STD);

#ifdef OHM_EMBED_GPU_CODE
    clerr = initProgramFromString(program, gpu, regionupdateCode, sourceFile, &args);
#else  // OHM_EMBED_GPU_CODE
    clerr = initProgramFromSource(program, gpu, sourceFile, &args);
#endif // OHM_EMBED_GPU_CODE

    if (clerr != CL_SUCCESS)
    {
      return clerr;
    }

    clerr = updateKernel.setEntry(program, "regionRayUpdate");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel regionRayUpdate() in " << sourceFile << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    updateKernel.calculateOptimalWorkGroupSize();

    programRef = 1;
    return 0;
  }


  void releaseRegionUpdateGpu()
  {
    std::lock_guard<std::mutex> guard(programMutex);

    if (--programRef <= 0)
    {
      programRef = 0;
      updateKernel = clu::Kernel();
      program = cl::Program();
    }
  }

  int updateRegion(gputil::Device &gpu,
                   gputil::Queue &queue,
                   gputil::Buffer &chunkMem,
                   gputil::Buffer &regionKeyBuffer, gputil::Buffer &regionOffsetBuffer,
                   unsigned regionCount,
                   gputil::Buffer &rayMem, unsigned rayCount,
                   const glm::ivec3 &regionVoxelDimensions, double voxelResolution,
                   float adjustMiss, float adjustHit,
                   float minVoxelValue, float maxVoxelValue,
                   std::initializer_list<gputil::Event> events,
                   gputil::Event *completionEvent)
  {
    cl_int clerr = CL_SUCCESS;
    clu::KernelGrid grid;

    cl::Event execEvent;
    std::vector<cl::Event> waitOnEvents;
    clu::EventList kernelEvents(nullptr, 0, &execEvent);

    waitOnEvents.reserve(events.size());
    for (const gputil::Event &event : events)
    {
      if (event.isValid())
      {
        waitOnEvents.push_back(cl::Event(event.detail()->event));
        // The constructor above won't increment the reference count, but will decrement.
        clRetainEvent(event.detail()->event);
      }
    }

    if (!waitOnEvents.empty())
    {
      kernelEvents.waitOnEvents = waitOnEvents.data();
      kernelEvents.eventCount = unsigned(waitOnEvents.size());
    }

    const cl_int3 regionDimOcl = { regionVoxelDimensions.x, regionVoxelDimensions.y, regionVoxelDimensions.z };

    grid.globalSize = rayCount;
    grid.workGroupSize = std::min<size_t>(updateKernel.optimalWorkGroupSize(), rayCount);

    cl::CommandQueue &queueOcl = queue.internal()->queue;
    auto startTime = std::chrono::high_resolution_clock::now();
    clerr = updateKernel(queueOcl, grid, kernelEvents,
                         chunkMem.arg<cl_mem>(),
                         regionKeyBuffer.arg<cl_mem>(), regionOffsetBuffer.arg<cl_mem>(),
                         regionCount,
                         rayMem.arg<cl_mem>(), rayCount,
                         regionDimOcl, float(voxelResolution),
                         adjustMiss, adjustHit,
                         minVoxelValue, maxVoxelValue);
    auto endTime = std::chrono::high_resolution_clock::now();
    // std::cout << "updateRegion QUEUE " << rayCount << " rays " << endTime - startTime << '\n';

    if (completionEvent)
    {
      completionEvent->release();
      completionEvent->detail()->event = execEvent();
      clRetainEvent(execEvent());
    }

    if (clerr != CL_SUCCESS)
    {
      std::cerr << "Error executing regionRayUpdate kernel: " << clu::errorCodeString(clerr) << std::endl;
      return clerr;
    }

    //clFlush(queueOcl());

//#ifdef OHM_PROFILE
//    {
//      //------------------------------------------------------------------------------
//      clFinish(queueOcl());
//
//      cl_ulong startTime = 0, endTime = 0;
//      clerr = clGetEventProfilingInfo(execEvent(), CL_PROFILING_COMMAND_SUBMIT, sizeof(startTime), &startTime, nullptr);
//      if (clerr != CL_SUCCESS)
//      {
//        std::cerr << "Profile submit failed: " << clu::errorCodeString(clerr) << std::endl;
//      }
//      clerr = clGetEventProfilingInfo(execEvent(), CL_PROFILING_COMMAND_END, sizeof(endTime), &endTime, nullptr);
//      if (clerr != CL_SUCCESS)
//      {
//        std::cerr << "Profile submit failed: " << clu::errorCodeString(clerr) << std::endl;
//      }
//
//      cl_ulong elapsedNs = endTime - startTime;
//      std::cout << "Kernel exec: " << std::chrono::nanoseconds(elapsedNs) << '\n';
//      //------------------------------------------------------------------------------
//    }
//#endif //  OHM_PROFILE

    return clerr;
  }
}
