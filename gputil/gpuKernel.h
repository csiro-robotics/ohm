// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUKERNEL_H
#define GPUKERNEL_H

#include "gpuConfig.h"

#include <functional>

namespace gputil
{
  struct KernelDetail;
  class Buffer;
  class Device;
  class EventList;
  class Event;
  class Queue;

  struct gputilAPI Dim3
  {
    size_t x = 1, y = 1, z = 1;

    Dim3() {}
    Dim3(size_t x, size_t y = 1, size_t z = 1)
      : x(x)
      , y(y)
      , z(z)
    {}
  };

  template <typename T>
  struct gputilAPI BufferArg
  {
    using ArgType = T;
    inline BufferArg(Buffer &buffer)
      : buffer(buffer)
    {}
    Buffer &buffer;
  };

  /// Local memory calculation function.
  /// @param work_group_size The work group total size.
  /// @return The number of bytes required for a group this size.
  using LocalMemFunc = std::function<size_t(size_t)>;

  /// Defines a callable kernel object.
  ///
  /// For OpenCL, this wraps the OpenCL kernel object and is initialised using <tt>gputil::openCLKernel()</tt>
  /// using a @c Program and the entry point name.
  ///
  /// For CUDA, this wraps a function pointer which calls the CUDA kernel and is created using
  /// <tt>gputil::cudaKernel()</tt>.
  ///
  /// There is no implementation indendent way of creating a @c Kernel.
  ///
  /// Invoking the kernel requires at least a global and local size (threads and blocks size). OpenCL global offset
  /// is not supported. A @c Queue pointer must be passed, though may be null, as it marks the beginning of device
  /// arguments. An @c Event object to track completion and an @p EventList to wait on before executing may also be
  /// optionally given any any combination. @c Buffer objects must be wrapped in a @c BufferArg in order to define
  /// (pointer) type on the device.
  ///
  /// A kernel invocation then takes this form:
  /// @code{.unparsed}
  ///   kernel(global_size, local_size[, wait_on_events][, completion_event], queue, ...args);
  /// @endcode
  ///
  /// Local memory is sized by using @c addLocal() which defines a functional object to define requires local memory
  /// size based on the total local memory size.
  class gputilAPI Kernel
  {
  public:
    Kernel();
    Kernel(Kernel &&other);

    ~Kernel();

    bool isValid() const;

    /// Add local memory calculation.
    ///
    /// Local memory is calculated by invoking the given function, passing the single dimensional work group size
    /// in order to calculate the required work group local memory size in bytes. This function is invoked just
    /// prior to invoking the kernel and when calculating the optimal work group size.
    ///
    /// Under CUDA, local memory requirements are tallied and passed to the kernel function hook given the total local
    /// memory required.
    ///
    /// Under OpenCL, each @c addLocal() call adds a local memory argument to the end of the argument list.
    ///
    /// @param local_calc The functional object used to calculate local memory size requirements.
    void addLocal(const LocalMemFunc &local_calc);

    size_t calculateOptimalWorkGroupSize();
    size_t optimalWorkGroupSize() const;

    template <typename... ARGS>
    int operator()(const Dim3 &global_size, const Dim3 &local_size, Queue *queue, ARGS... args);

    template <typename... ARGS>
    int operator()(const Dim3 &global_size, const Dim3 &local_size, Event &completion_event, Queue *queue, 
                   ARGS... args);

    template <typename... ARGS>
    int operator()(const Dim3 &global_size, const Dim3 &local_size, const EventList &event_list, Queue *queue,
                   ARGS... args);

    template <typename... ARGS>
    int operator()(const Dim3 &global_size, const Dim3 &local_size, const EventList &event_list,
                   Event &completion_event, Queue *queue, ARGS... args);

    KernelDetail *detail() const { return imp_; }

    Device device();

    Kernel &operator=(Kernel &&other);

  private:
    KernelDetail *imp_;
  };
}  // namespace gputil

#if GPUTIL_TYPE == GPUTIL_OPENCL
#include "cl/gpuKernel2.h"
#elif GPUTIL_TYPE == GPUTIL_CUDA
#include "cuda/gpuKernel2.h"
#else  // GPUTIL_TYPE == ???
#error Unknown GPU base API
#endif  // GPUTIL_TYPE

#endif  // GPUKERNEL_H
