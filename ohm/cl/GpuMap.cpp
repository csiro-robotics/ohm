// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmConfig.h"

#include "cl/clProgram.h"
#include "private/GpuMapDetail.h"

#ifdef OHM_PROFILE
#include <ohm/OccupancyUtil.h>
#endif //  OHM_PROFILE
#include <ohmutil/OhmUtil.h>

#include <gputil/gpuPlatform.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuQueue.h>

#include <clu/clu.h>
#include <clu/cluKernel.h>
#include <clu/cluProgram.h>

#include <gputil/cl/gpuDeviceDetail.h>
#include <gputil/cl/gpuEventDetail.h>
#include <gputil/cl/gpuQueueDetail.h>

#include <algorithm>
#include <mutex>
#include <initializer_list>

#ifdef OHM_EMBED_GPU_CODE
#include "RegionUpdateResource.h"
#endif // OHM_EMBED_GPU_CODE

namespace
{
  std::mutex program_mutex;
  cl::Program program;
  clu::Kernel update_kernel;
  int program_ref = 0;
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

    std::lock_guard<std::mutex> guard(program_mutex);

    if (program())
    {
      // Already initialised.
      ++program_ref;
      return CL_SUCCESS;
    }

    // Compile and initialise.
    cl_int clerr = CL_SUCCESS;
    const char *source_file = "regionupdate.cl";
    std::vector<std::string> args;
    args.push_back("-cl-std=CL" OHM_OPENCL_STD);

#ifdef OHM_EMBED_GPU_CODE
    clerr = initProgramFromString(program, gpu, RegionUpdateCode, source_file, &args);
#else  // OHM_EMBED_GPU_CODE
    clerr = initProgramFromSource(program, gpu, source_file, &args);
#endif // OHM_EMBED_GPU_CODE

    if (clerr != CL_SUCCESS)
    {
      return clerr;
    }

    clerr = update_kernel.setEntry(program, "regionRayUpdate");

    if (clerr != CL_SUCCESS)
    {
      program = cl::Program();
      std::cerr << "Failed to resolve kernel regionRayUpdate() in " << source_file << ": " << clu::errorCodeString(clerr) << '\n';
      return clerr;
    }

    update_kernel.calculateOptimalWorkGroupSize();

    program_ref = 1;
    return 0;
  }


  void releaseRegionUpdateGpu()
  {
    std::lock_guard<std::mutex> guard(program_mutex);

    if (--program_ref <= 0)
    {
      program_ref = 0;
      update_kernel = clu::Kernel();
      program = cl::Program();
    }
  }

  int updateRegion(gputil::Device &gpu,
                   gputil::Queue &queue,
                   gputil::Buffer &chunk_mem,
                   gputil::Buffer &region_key_buffer, gputil::Buffer &region_offset_buffer,
                   unsigned region_count,
                   gputil::Buffer &ray_mem, unsigned ray_count,
                   const glm::ivec3 &region_voxel_dimensions, double voxel_resolution,
                   float adjust_miss, float adjust_hit,
                   float min_voxel_value, float max_voxel_value,
                   std::initializer_list<gputil::Event> events,
                   gputil::Event *completion_event)
  {
    cl_int clerr = CL_SUCCESS;
    clu::KernelGrid grid;

    cl::Event exec_event;
    std::vector<cl::Event> wait_on_events;
    clu::EventList kernel_events(nullptr, 0, &exec_event);

    wait_on_events.reserve(events.size());
    for (const gputil::Event &event : events)
    {
      if (event.isValid())
      {
        wait_on_events.push_back(cl::Event(event.detail()->event));
        // The constructor above won't increment the reference count, but will decrement.
        clRetainEvent(event.detail()->event);
      }
    }

    if (!wait_on_events.empty())
    {
      kernel_events.wait_on_events = wait_on_events.data();
      kernel_events.event_count = unsigned(wait_on_events.size());
    }

    const cl_int3 region_dim_ocl = { region_voxel_dimensions.x, region_voxel_dimensions.y, region_voxel_dimensions.z };

    grid.global_size = ray_count;
    grid.work_group_size = std::min<size_t>(update_kernel.optimalWorkGroupSize(), ray_count);

    cl::CommandQueue &queue_ocl = queue.internal()->queue;
    //auto start_time = std::chrono::high_resolution_clock::now();
    clerr = update_kernel(queue_ocl, grid, kernel_events,
                         chunk_mem.arg<cl_mem>(),
                         region_key_buffer.arg<cl_mem>(), region_offset_buffer.arg<cl_mem>(),
                         region_count,
                         ray_mem.arg<cl_mem>(), ray_count,
                         region_dim_ocl, float(voxel_resolution),
                         adjust_miss, adjust_hit,
                         min_voxel_value, max_voxel_value);
    //auto end_time = std::chrono::high_resolution_clock::now();
    // std::cout << "updateRegion QUEUE " << rayCount << " rays " << endTime - startTime << '\n';

    if (completion_event)
    {
      completion_event->release();
      completion_event->detail()->event = exec_event();
      clRetainEvent(exec_event());
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
