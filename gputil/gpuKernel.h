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

  /// A helper structure for defining the number of global work items or the size of a local work group.
  ///
  /// Unused dimensions must have a value of 1 to ensure the correct @c volume().
  struct gputilAPI Dim3
  {
    /// Dimension X/first value.
    size_t x = 0;
    /// Dimension Y/second value.
    size_t y = 1;
    /// Dimension Z/third value.
    size_t z = 1;

    /// Default constructor: @c volume() is 1.
    Dim3() = default;

    /// Constructor.
    /// @param x The x item count.
    /// @param y The y item count.
    /// @param z The z item count.
    Dim3(size_t x, size_t y = 1, size_t z = 1)
      : x(x)
      , y(y)
      , z(z)
    {}

    /// Defines the total item size by multiplying the XYZ dimensions.
    inline size_t volume() const { return x * y * z; }

    /// Index operator.
    /// @param i The element index [0, 2].
    /// @return The value of the corresponding element.
    inline size_t operator[](int i) const
    {
      switch (i)
      {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      default:
        break; // Fallout
      }
      return 0;
    }

    /// @overload
    inline size_t &operator[](int i)
    {
      switch (i)
      {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      default:
        break; // Fall out to invalid.
      }
      static size_t invalid = 0u;
      return invalid;
    }

    /// @overload
    inline size_t operator[](unsigned i) const { return operator[](int(i)); }
    /// @overload
    inline size_t &operator[](unsigned i) { return operator[](int(i)); }

    /// @overload
    inline size_t operator[](size_t i) const { return operator[](int(i)); }
    /// @overload
    inline size_t &operator[](size_t i) { return operator[](int(i)); }
  };

  /// A helper class for passing buffer arguments to a kernel.
  ///
  /// Given a kernel with the signature: @c myKernel(__global float3 *buffer), the argument may be passed as follows:
  /// @code
  /// void bufferArgExample(gputil::Kernel &kernel, gputil::Buffer &buffer)
  /// {
  ///   kernel(nullptr, gputil::BufferArg<gputil::float3>(buffer));
  /// }
  /// @endcode
  ///
  /// Note how the template type is the target type without any decoration (such as pointer).
  template <typename T>
  struct gputilAPI BufferArg
  {
    /// The type to cast to in the GPU kernel.
    using ArgType = T;

    /// Constructor.
    /// @param buffer The buffer to wrap.
    inline BufferArg(Buffer &buffer)  // NOLINT(google-runtime-references)
      : buffer(&buffer)
    {}

    /// Alternative constructor supporting a null buffer argument. Passing NULL ensures the kernel argument on device
    /// is also null.
    /// @param buffer A pointer to the buffer to wrap or null for a null argument on device.
    inline BufferArg(Buffer *buffer = nullptr)
      : buffer(buffer)
    {}

    /// A reference to the wrapped buffer.
    Buffer *buffer;
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
    /// Construct an empty kernel.
    Kernel();
    /// Move constructor
    /// @param other The object to move.
    Kernel(Kernel &&other) noexcept;

    /// Destuctor - ensures @c release() is called.
    ~Kernel();

    /// Query if the kernel has been correctly setup.
    /// @return True if the kernel has been setup with error and can be invoked.
    bool isValid() const;

    /// Release the GPU kernel.
    void release();

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

    /// Calculate the optimal size (or volume) of a local work group. This attempts to gain maximum occupancy while
    /// considering the required local memory usage.
    /// @return The optimal work group size.
    size_t calculateOptimalWorkGroupSize();

    /// Fetch the previously calculated optimal work group size (see @c calculateOptimalWorkGroupSize()).
    /// @return The optimal work group size.
    size_t optimalWorkGroupSize() const;

    /// Calculate the appropriate global and work group sizes for executing this @c Kernel to process
    /// @p total_work_items items. The aim is to gain maximum local thread occupancy.
    ///
    /// The @p total_work_items defines a volume of items to process. The global size is set appropriately to cover
    /// the @p total_work_items with the @p local_size set to cover these in a grid pattern with consideration to the
    /// device capabilities and maximum occupancy. This includes maximum work group sizes and local memory constraints.
    ///
    /// @param[out] global_size Set to the grid global size. This may be larger than @p total_work_group_items to ensure
    ///   an exact multiple of the @p local_size.
    /// @param[out] local_size Set to the local work group size required to cover the @p global_size/@p
    /// total_work_items.
    /// @param total_work_items The total volume of items to process.
    void calculateGrid(gputil::Dim3 *global_size, gputil::Dim3 *local_size, const gputil::Dim3 &total_work_items);

    /// Queue an invocation of the kernel with the given global and local sizes and the specified queue - arguments
    /// follow.
    /// @param global_size The global dimensions of the of GPU thread pool to run the the kernel with.
    /// @param local_size The local thread division of the @p global_size .
    /// @param queue The GPU queue to invoke the kernel on. May be null to used the default queue
    /// @param args Arguments to pass to the kernel.
    /// @return Zero on success or an SDK error code on falure - e.g., @c cudaSuccess .
    template <typename... ARGS>
    int operator()(const Dim3 &global_size, const Dim3 &local_size, Queue *queue, ARGS... args);

    /// Queue an invocation of the kernel with the given global and local sizes and the specified queue - arguments
    /// follow. The @p completion_event is set to mark the completion of the invocation and can be used for
    /// synchronisation. For example waiting on @p completion_event on CPU blocks the CPU until the kernel completes.
    /// @param global_size The global dimensions of the of GPU thread pool to run the the kernel with.
    /// @param local_size The local thread division of the @p global_size .
    /// @param completion_event Event object modified to mark the completion of the kernel execution.
    /// @param queue The GPU queue to invoke the kernel on. May be null to used the default queue
    /// @param args Arguments to pass to the kernel.
    /// @return Zero on success or an SDK error code on falure - e.g., @c cudaSuccess .
    template <typename... ARGS>
    int operator()(const Dim3 &global_size, const Dim3 &local_size,
                   Event &completion_event,  // NOLINT(google-runtime-references)
                   Queue *queue, ARGS... args);

    /// Queue an invocation of the kernel to start after all @p event_list items complete.
    /// @param global_size The global dimensions of the of GPU thread pool to run the the kernel with.
    /// @param local_size The local thread division of the @p global_size .
    /// @param event_list Events which must complete before kernel execution can start.
    /// @param queue The GPU queue to invoke the kernel on. May be null to used the default queue
    /// @param args Arguments to pass to the kernel.
    /// @return Zero on success or an SDK error code on falure - e.g., @c cudaSuccess .
    template <typename... ARGS>
    int operator()(const Dim3 &global_size, const Dim3 &local_size, const EventList &event_list, Queue *queue,
                   ARGS... args);


    /// Queue an invocation of the kernel to start after all @p event_list items complete. The @p completion_event is
    /// set to mark the completion of the invocation and can be used for synchronisation. For example waiting on @p
    /// completion_event on CPU blocks the CPU until the kernel completes.
    /// @param global_size The global dimensions of the of GPU thread pool to run the the kernel with.
    /// @param local_size The local thread division of the @p global_size .
    /// @param event_list Events which must complete before kernel execution can start.
    /// @param completion_event Event object modified to mark the completion of the kernel execution.
    /// @param queue The GPU queue to invoke the kernel on. May be null to used the default queue
    /// @param args Arguments to pass to the kernel.
    /// @return Zero on success or an SDK error code on falure - e.g., @c cudaSuccess .
    template <typename... ARGS>
    int operator()(const Dim3 &global_size, const Dim3 &local_size, const EventList &event_list,
                   Event &completion_event, Queue *queue, ARGS... args);  // NOLINT(google-runtime-references)

    /// Internal kernel representation.
    /// @return The internal representation.
    KernelDetail *detail() const { return imp_; }

    /// Query the @c Device which the kernel has been associated with and on which the kernel will run when invoked.
    /// @return The kernel's @c Device .
    Device device();

    /// Move assignment.
    /// @param other The object to move.
    /// @return `*this`
    Kernel &operator=(Kernel &&other) noexcept;

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
