// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUKERNEL2_H
#define GPUKERNEL2_H

#include "gpuConfig.h"

#include "gpuBuffer.h"

#include "cl/gpuEventDetail.h"
#include "cl/gpuKernelDetail.h"
#include "cl/gpuQueueDetail.h"

#include <clu/cluKernel.h>

#include <malloc.h>

namespace clu
{
  /// Override kernel argument setting using @c gputil::Buffer to map to @c cl_mem.
  inline cl_int setKernelArg(cl::Kernel &kernel, int arg_index, const gputil::Buffer &arg)
  {
    cl_mem mem = arg.arg<cl_mem>();
    return ::clSetKernelArg(kernel(), arg_index, sizeof(mem), &mem);
  }

  /// Override kernel argument setting using @c gputil::BufferArg to map to @c cl_mem.
  template <typename T>
  inline cl_int setKernelArg(cl::Kernel &kernel, int arg_index, const gputil::BufferArg<T> &arg)
  {
    cl_mem mem = arg.buffer.arg<cl_mem>();
    return ::clSetKernelArg(kernel(), arg_index, sizeof(mem), &mem);
  }
}  // namespace clu

namespace gputil
{
  template <typename... ARGS>
  int Kernel::operator()(const Dim3 &global_size, const Dim3 &local_size, Queue *queue, ARGS... args)
  {
    clu::KernelGrid grid;
    grid.work_group_size = clu::KernelSize(local_size.x, local_size.y, local_size.z);
    grid.global_size = clu::KernelSize(global_size.x, global_size.y, global_size.z);
    cl::CommandQueue &queue_cl = (queue) ? queue->internal()->queue : device().defaultQueue().internal()->queue;
    return detail()->kernel(queue_cl, grid, clu::EventList(), args...);
  }


  template <typename... ARGS>
  int Kernel::operator()(const Dim3 &global_size, const Dim3 &local_size, Event &completion_event, Queue *queue,
                         ARGS... args)
  {
    clu::KernelGrid grid;
    clu::EventList events_clu;
    cl::Event completion_tracker;
    events_clu.completion = &completion_tracker;

    grid.work_group_size = clu::KernelSize(local_size.x, local_size.y, local_size.z);
    grid.global_size = clu::KernelSize(global_size.x, global_size.y, global_size.z);
    cl::CommandQueue &queue_cl = (queue) ? queue->internal()->queue : device().defaultQueue().internal()->queue;
    int err = detail()->kernel(queue_cl, grid, events_clu, args...);
    completion_event.release();
    completion_event.detail()->event = completion_tracker();
    clRetainEvent(completion_tracker());
    return err;
  }


  template <typename... ARGS>
  int Kernel::operator()(const Dim3 &global_size, const Dim3 &local_size, const EventList &event_list, Queue *queue,
                         ARGS... args)
  {
    clu::KernelGrid grid;
    clu::EventList events_clu;

    events_clu.event_count = unsigned(event_list.count());
    if (events_clu.event_count)
    {
      events_clu.wait_on_events = static_cast<cl::Event *>(alloca(sizeof(cl::Event) * events_clu.event_count));
      for (unsigned i = 0; i < events_clu.event_count; ++i)
      {
        // Placement new into stack allocation.
        ::new (&events_clu.wait_on_events[i]) cl::Event(event_list.events()[i].detail()->event, true);
      }
    }

    grid.work_group_size = clu::KernelSize(local_size.x, local_size.y, local_size.z);
    grid.global_size = clu::KernelSize(global_size.x, global_size.y, global_size.z);
    cl::CommandQueue &queue_cl = (queue) ? queue->internal()->queue : device().defaultQueue().internal()->queue;
    int err = detail()->kernel(queue_cl, grid, events_clu, args...);

    // Cleanup stack objects.
    // TODO: RAIA for this while avoiding a head allocation.
    for (unsigned i = 0; i < events_clu.event_count; ++i)
    {
      using namespace cl;
      // Call destructor in stack allocation.
      events_clu.wait_on_events[i].~Event();
    }

    return err;
  }


  template <typename... ARGS>
  int Kernel::operator()(const Dim3 &global_size, const Dim3 &local_size, const EventList &event_list,
                         Event &completion_event, Queue *queue, ARGS... args)
  {
    clu::KernelGrid grid;
    clu::EventList events_clu;
    cl_uint ref_count = 0;

    events_clu.event_count = unsigned(event_list.count());
    if (events_clu.event_count)
    {
      events_clu.wait_on_events = static_cast<cl::Event *>(alloca(sizeof(cl::Event) * events_clu.event_count));
      for (unsigned i = 0; i < events_clu.event_count; ++i)
      {
        // Placement new into stack allocation.
        ::new (&events_clu.wait_on_events[i]) cl::Event(event_list.events()[i].detail()->event, true);
      }
    }
    cl::Event completion_tracker;
    events_clu.completion = &completion_tracker;

    grid.work_group_size = clu::KernelSize(local_size.x, local_size.y, local_size.z);
    grid.global_size = clu::KernelSize(global_size.x, global_size.y, global_size.z);
    cl::CommandQueue &queue_cl = (queue) ? queue->internal()->queue : device().defaultQueue().internal()->queue;
    int err = detail()->kernel(queue_cl, grid, events_clu, args...);

    // Cleanup stack objects.
    // TODO: RAIA for this while avoiding a head allocation.
    for (unsigned i = 0; i < events_clu.event_count; ++i)
    {
      using namespace cl;
      // Call destructor in stack allocation.
      events_clu.wait_on_events[i].~Event();
    }

    completion_event.release();
    completion_event.detail()->event = completion_tracker();
    clRetainEvent(completion_tracker());
    return err;
  }


  class Program;
  Kernel openCLKernel(Program &program, const char *kernel_name);
}  // namespace gputil

#endif  // GPUKERNEL2_H
