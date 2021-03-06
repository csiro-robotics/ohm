//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2017
//
#ifndef CLUKERNEL_H
#define CLUKERNEL_H

#include "clu.h"

#include <array>
#include <functional>
#include <iostream>

namespace clu
{
/// Helper class for setting OpenCL kernel arguments based on their type
///
/// The default implementation is to call @c cl::Kernel::setArg() with the given value.
template <typename T>
struct KernelArgHandler
{
  /// Set a kernel argument.
  /// @param kernel The OpenCL kernel to set an argument for.
  /// @param arg_index Index of the argument to set.
  /// @param arg The argument value.
  static cl_int set(cl::Kernel &kernel, int arg_index, const T &arg) { return kernel.setArg(arg_index, arg); }
};

/// Explicitly handle @c cl_mem type. Changing to the C++ <tt>cl2.hpp</tt> API changed how @c cl::Kernel handled
/// pointer arguments. In practice it expects all buffer arguments to be of C++ @c cl::Buffer, @c cl::Image,
/// @c cl::Pipe, etc types. Any pointer invokes @c clSetKernelArgSVMPointer() instead of @c clSetKernelArg() which
/// also captures @c cl_mem type as it is a pointer typedef. This results in incorrect behaviour and we correct
/// for this by calling the C function @c clSetKernelArg explicitly here.
template <>
struct KernelArgHandler<cl_mem>
{
  /// Set a kernel argument.
  /// @param kernel The OpenCL kernel to set an argument for.
  /// @param arg_index Index of the argument to set.
  /// @param arg The argument value.
  static cl_int set(cl::Kernel &kernel,

                    int arg_index, cl_mem arg)
  {
    // Disable lint warning - we do mean to take the size of a pointer.
    // NOLINTNEXTLINE(bugprone-sizeof-expression)
    return ::clSetKernelArg(kernel(), arg_index, sizeof(arg), &arg);
  }
};


template <typename T>
cl_int setKernelArgs2(cl::Kernel &kernel, int &arg_index, const T &arg)
{
  const cl_int clerr = KernelArgHandler<T>::set(kernel, arg_index, arg);
  ++arg_index;
  clu::checkError(std::cerr, clerr, "Arg", arg_index);
  return clerr;
}


template <typename T, typename... ARGS>
cl_int setKernelArgs2(cl::Kernel &kernel, int &arg_index, const T &arg, ARGS... args)
{
  const cl_int clerr = KernelArgHandler<T>::set(kernel, arg_index, arg);
  ++arg_index;
  clu::checkError(std::cerr, clerr, "Arg", arg_index);
  const cl_int clerr_other = setKernelArgs2(kernel, arg_index, args...);
  if (clerr == CL_SUCCESS)
  {
    return clerr_other;
  }
  return clerr;
}


/// Sets the arguments for @p kernel to the passed arguments, in order.
/// Each of the @p args is passed to @c cl::Kernel::setArg() in turn, modifying the indexing as
/// required. On any argument failure, the error is reported to @c std::cerr. Each argument is
/// set regardless of early failures, so each failure will be logged.
///
/// The function then returns either @c CL_SUCCESS or the first failure error value.
///
/// @param kernel The kernel to set arguments for.
/// @param args The arguments to pass. Captured by const reference.
/// @return @c CL_SUCCESS on success or the first failure error code on failure.
template <typename... ARGS>
cl_int setKernelArgs(cl::Kernel &kernel, ARGS... args)
{
  int arg_index = 0;
  return setKernelArgs2(kernel, arg_index, args...);
}

/// Class which defines the size and dimensions of a kernel local or global size.
class KernelSize
{
public:
  /// Size dimensionality.
  enum Dimension : unsigned
  {
    kDNull,  ///< Not set
    kD1D,    ///< One dimensional size.
    kD2D,    ///< Two dimensional size.
    kD3D     ///< Three dimensional size.
  };

  /// Intiantite a zero size, null dimensinoed kernel.
  inline KernelSize() = default;

  /// Instantiate a 1D kernel size.
  /// @param a Size of the first dimension.
  inline KernelSize(size_t a)  // NOLINT(google-explicit-constructor)
  {
    sizes_[0] = a;
    sizes_[1] = sizes_[2] = 0;
  }

  /// Instantiate a 2D kernel size.
  /// @param a Size of the first dimension.
  /// @param b Size of the second dimension.
  inline KernelSize(size_t a, size_t b)
  {
    sizes_[0] = a;
    sizes_[1] = b;
    sizes_[2] = 0;
  }

  /// Instantiate a 3D kernel size.
  /// @param a Size of the first dimension.
  /// @param b Size of the second dimension.
  /// @param c Size of the third dimension.
  inline KernelSize(size_t a, size_t b, size_t c)
  {
    sizes_[0] = a;
    sizes_[1] = b;
    sizes_[2] = c;
  }

  /// Copy constructor.
  /// @param other Object to copy.
  inline KernelSize(const KernelSize &other) { *this = other; }

  inline ~KernelSize() = default;

  /// Query the dimensionality of the size specification.
  ///
  /// The dimensionality is determined by the first zero value in the size specification. A zero size in the first
  /// dimension implies @c kDNull , a zero size in the second implies @c kD1D , etc.
  /// @return The dimensionality of this size specification.
  inline Dimension dimensions() const
  {
    return (sizes_[2]) ? kD3D : ((sizes_[1]) ? kD2D : ((sizes_[0]) ? kD1D : kDNull));
  }

  /// Query the size of the selected dimension.
  /// @param dim The dimension to query in the range `[0, 2]`.
  /// @return The size of the queried dimension.
  inline size_t size(int dim = 0) const
  {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    return sizes_[dim];
  }

  /// Query the size of the indexed dimension.
  /// @param dim The dimension to query in the range `[0, 2]`.
  /// @return The size of the queried dimension.
  inline size_t operator[](int dim) const
  {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    return sizes_[dim];
  }
  /// Get a reference to the size of the indexed dimension.
  /// @param dim The dimension to query in the range `[0, 2]`.
  /// @return The size of the queried dimension.
  inline size_t &operator[](int dim)
  {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
    return sizes_[dim];
  }

  /// Convert the @c KernelSize for use in kernel invocation.
  /// @return A pointer to the size array.
  inline const size_t *arg() const { return sizes_.data(); }

  /// Query if the size specification is null.
  /// @return True if the dimensionality is @c kDNull .
  inline bool isNull() const { return dimensions() == kDNull; }
  /// Query if the size specification is valid.
  /// @return True if the dimensionality is not @c kDNull .
  inline bool isValid() const { return !isNull(); }

  /// Calculate the product of the sizes to calculate a kernel "volume".
  /// @return The product of the dimension values.
  inline size_t volume() const
  {
    size_t total = sizes_[0];
    total *= dimensions() > kD1D ? sizes_[1] : 1;
    total *= dimensions() > kD2D ? sizes_[2] : 1;
    return total;
  }

  /// Assignment operator.
  /// @param other Object to assign
  /// @return `*this`
  inline KernelSize &operator=(const KernelSize &other)  // NOLINT(bugprone-unhandled-self-assignment)
  {
    // Note: self assignment will be fine.
    sizes_[0] = other.sizes_[0];
    sizes_[1] = other.sizes_[1];
    sizes_[2] = other.sizes_[2];
    return *this;
  }

  /// @c kD1 or @c kDNull assignment operator. Assigning a non zero value sets the size specification to a @c k1D
  /// specification of @p size . Setting a zero value makes this a @c kDNull specification.
  /// @param size The size value to assign.
  /// @return `*this`
  inline KernelSize &operator=(size_t size)
  {
    sizes_[0] = size;
    sizes_[1] = sizes_[2] = 0;
    return *this;
  }

private:
  std::array<size_t, 3> sizes_ = { 0, 0, 0 };
};


/// A helper structure for setting the execution size of a kernel.
///
/// Specifies a @c global_offset , applied to the global kernel size queries in OpenCL, a @c global_size controlling
/// the total number of GPU threads and a @c work_group_size setting the number of threads in each work group.
/// The dimensionality of all must match excepting that @c global_offset may be null implying a zero offset.
///
/// The @c global_size is used as a guide, but will be adjusted up by @c adjustGlobal() to be a multiple of the
/// @c work_group_size .
struct KernelGrid
{
  KernelSize global_offset;    ///< Global kernel size offset.
  KernelSize global_size;      ///< Global kernel size
  KernelSize work_group_size;  ///< Work group size.

  /// Default constructor. All size values are null.
  inline KernelGrid() = default;

  /// Create a kernel grid speficiation.
  /// @param global_offset The kernel global offset value. May be null
  /// @param global_size The global thread size.
  /// @param work_group_size The workgroup thread size.
  inline KernelGrid(const KernelSize &global_offset, const KernelSize &global_size, const KernelSize &work_group_size)
    : global_offset(global_offset)
    , global_size(global_size)
    , work_group_size(work_group_size)
  {}

  /// Create a kernel grid specification with implied zero offset.
  /// @param global_size The global thread size.
  /// @param work_group_size The workgroup thread size.
  inline KernelGrid(const KernelSize &global_size, const KernelSize &work_group_size)
    : KernelGrid(KernelSize(), global_size, work_group_size)
  {}

  /// Calculates an adjusted global size such that it is a multiple of the work group size.
  /// @return A global size such that it is a multiple of the work group size in each valid dimension.
  KernelSize adjustedGlobal() const;

  /// Validate the grid specification.
  ///
  /// Requires that the dimensionality of the global and workgroup sizes match. The global offset must also match
  /// if specified, but may be null.
  /// @return True if the specification is valid.
  inline bool isValid() const
  {
    return global_size.isValid() && work_group_size.isValid() &&
           global_size.dimensions() == work_group_size.dimensions() &&
           (global_size.dimensions() == global_offset.dimensions() || global_offset.isNull());
  }
};

/// Encapsulates events to complete before a kernel can execute as well as exposing an event to mark
/// kernel completion.
///
/// Used as an argument to @c Kernel::operator() to aid in overloading event setup.
struct EventList
{
  /// The even object to be used to mark completion of the queued operation (optional).
  cl::Event *completion = nullptr;
  cl::Event *wait_on_events = nullptr;  ///< Events to wait on before continuing the execution queue (optional).
  unsigned event_count = 0;             ///< Number of items in @c wait_on_events.

  /// Create an empty event list.
  inline EventList() = default;

  /// Create an event list with only a completion event.
  /// @param completion_event Event object to use to mark queue completion for the requested operation.
  explicit inline EventList(cl::Event *completion_event)
    : completion(completion_event)
  {}

  /// Create an event list to wait on with optional completion.
  /// @param wait_on Events to wait on before continuing queue execution.
  /// @param event_count Number of items in @p wait_on .
  /// @param completion_event Optional event object to use to mark queue completion for the requested operation.
  inline EventList(cl::Event *wait_on, unsigned event_count, cl::Event *completion_event = nullptr)
    : completion(completion_event)
    , wait_on_events(wait_on)
    , event_count(event_count)
  {}
};


/// A utility class for referencing, setting up and invoking OpenCL kernels.
///
/// @par Local Memory Arguments
/// The @c Kernel class supports a fixed number of local memory arguments appearing either before
/// all other arguments, or after all other arguments. The kernel may have at most @c MAX_LOCAL_MEM_ARGS
/// such arguments. Each local memory argument has an associated @c LocalMemArgSizeFunc function
/// object, which is used to determine the size of the local memory argument (bytes).
/// Each size function is invoked passing the local work group size allowing the function to determine
/// the required size on a per thread basis.
///
/// All local memory arguments must be registed sequentially using @p addLocal() before calling
/// @c calculateOptimalWorkGroupSize(). The @c calculateOptimalWorkGroupSize() function also uses
/// the function object to determine the best local size for the kernel maximising occupancy,
/// partly based on local memory requirements.
///
/// By default all local memory arguments are assumed to come after other arguments passed to the
/// @c operator(), but this may be changed by calling @c setLocalMemFirst().
class Kernel
{
public:
  /// Maximum supported number of local memory arguments which may be added with @c addLocal() .
  static const int kMaxLocalMemArgs = 8;

  /// Typedef for functional objects used to calculate the size of a local memory argument.
  /// The argument passed is the work group size while the return value specifies the
  /// required local memory for the corresponding work group size (bytes).
  using LocalMemArgSizeFunc = std::function<size_t(size_t)>;

  /// Default constructor.
  Kernel();
  /// Create a kernel from @p program with the given @p entry_point (kernel function) name.
  /// @param program Compiled program object to find the kernel entry point in.
  /// @param entry_point Kernel function name to resolve in @p program .
  /// @param log Optional error logging stream.
  Kernel(cl::Program &program, const char *entry_point, std::ostream *log = nullptr);
  /// Constructor to wrap an existing OpenCL C++ API kernel object.
  /// @param cl_kernel The kernel object to wrap.
  explicit Kernel(cl::Kernel &cl_kernel);

  /// Is this a valid kernel?
  bool isValid() const;

  /// Set the kernel entry point.
  /// @param program The program to find the kernel in.
  /// @param entry_point The kernel entry point/function name.
  /// @param log Optional error logging stream.
  /// @return @c CL_SUCCESS on success, an OpenCL error code otherwise.
  cl_int setEntry(cl::Program &program, const char *entry_point, std::ostream *log = nullptr);

  /// Set local memory arguments ordering. See class notes.
  /// @param first True to have local memory arguments before other arguments.
  inline void setLocalMemFirst(bool first) { local_mem_first_ = first; }

  /// Do local memory arguments come before other arguments?
  /// @return True if local memory arguments appear before other arguments.
  inline bool localMemFirst() const { return local_mem_first_; }

  /// Adds a local memory argument whose is calculated by invoking @p argFunc.
  ///
  /// See class notes on local memory arguments.
  ///
  /// Note: no more than 8 local arguments arg supported.
  ///
  /// @param arg_func The function to invoke when calculating the local memory size (bytes).
  /// @return The index of the local argument. This is not the same as the final argument index,
  ///   just the index into the set of local arguments, zero based. -1 on failure.
  cl_int addLocal(const LocalMemArgSizeFunc &arg_func);

  /// Calculates the optimal work group size for the kernel based on the compiled code and local memory usage.
  ///
  /// This calculates the work group size which achieves maximum occupancy. This is affected by
  /// the device characteristics and the local memory requirements calculated using the functions registed
  /// calling @c addLocal().
  ///
  /// The resulting size is one dimensional, so it is up to user code to determine the best subdivisions
  /// when using multiple dimensions.
  ///
  /// The result is returned, but also stored and accessible by calling @c optimalWorkGroupSize().
  ///
  /// @return The calculated, optimal size.
  size_t calculateOptimalWorkGroupSize();

  /// Queries the optimal work group size calculated by @c calculateOptimalWorkGroupSize().
  /// Zero if not calculated yet.
  /// @return The optimal work group size for maximum occupancy.
  inline size_t optimalWorkGroupSize() const { return optimal_work_group_size_; }

  /// Invocation operator. This invoked the kernel passing the given template arguments.
  ///
  /// The operator requires three standard arguments to specify the @c cl::CommandQueue,
  /// the execution offset and sizes (@c KernelGrid), the events preceeding execution and
  /// the completion event. Both preceeding and completion events are wrapped up in the
  /// @c events argument. Arguments following these standard arguments are passed to the
  /// kernel itself.
  ///
  /// Local memory arguments may also be set up using the function objects previously passed
  /// to @c addLocal() calls. See class notes.
  ///
  /// The @c EventList object is expected to be transient and may be:
  /// - @c EventList() empty for no preceeding events and no completion event.
  /// - @c EventList(cl::Event *) a single event pointer used to resolve the event to mark completion
  /// - @c EventList(cl::Event *, size_t, cl::Event *) to specify a list of events to wait on
  ///   (first argument), the number of items in the list and the completion event.
  ///
  /// Note that this is a non-blocking function and the kernel will not have completed on return.
  ///
  /// @param queue The command queue to execute on.
  /// @param grid The execution sizes.
  /// @param events Event list to wait on and completion events.
  /// @param args Arguments to pass to the kernel.
  /// @return @c CL_SUCCESS on success, or an error code on failure.
  template <typename... ARGS>
  cl_int operator()(cl::CommandQueue &queue, const KernelGrid &grid, const EventList &events, ARGS... args)
  {
    cl_int clerr = preInvoke(args...);
    if (clerr != CL_SUCCESS)
    {
      return clerr;
    }
    return invoke(queue, grid, events);
  }

  /// Access the internal OpenCL kernel object.
  /// @return The kernel object.
  inline cl::Kernel &kernel() { return kernel_; }
  /// Access the internal OpenCL kernel object.
  /// @return The kernel object.
  inline const cl::Kernel &kernel() const { return kernel_; }

private:
  /// Preinvocation setup.
  /// @param args Arguments to pass to the kernel.
  template <typename... ARGS>
  inline cl_int preInvoke(ARGS... args)
  {
    int arg_count = (!local_mem_first_) ? 0 : local_mem_arg_count_;
    cl_int clerr = setKernelArgs2(kernel_, arg_count, args...);
    if (clerr != CL_SUCCESS)
    {
      return clerr;
    }
    clerr = setLocalMemArgs(arg_count);
    if (clerr != CL_SUCCESS)
    {
      return clerr;
    }
    return clerr;
  }

  /// Setup local memory arguments.
  /// @param arg_count Number of non-local mem args when @c _localMemFirst is false, otherwise it's the total args.
  /// @return @c CL_SUCCESS on success, an OpenCL error code on failure.
  cl_int setLocalMemArgs(int arg_count);

  cl_int invoke(cl::CommandQueue &queue, const KernelGrid &grid, const EventList &events);

  cl::Kernel kernel_;
  size_t optimal_work_group_size_ = 0;
  std::array<LocalMemArgSizeFunc, kMaxLocalMemArgs> local_mem_args_;
  int local_mem_arg_count_ = 0;
  bool local_mem_first_ = false;
};
}  // namespace clu

#endif  // CLUKERNEL_H
