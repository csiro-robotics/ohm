//
// @author Kazys Stepanas
//
// Copyright (c) CSIRO 2017
//
#ifndef CLUKERNEL_H_
#define CLUKERNEL_H_

#include "clu.h"

#include "cl.hpp"

#include <iostream>
#include <functional>

namespace clu
{
  /// @internal
  template <typename T>
  cl_int setKernelArgs_(cl::Kernel &kernel, int &argIndex, const T &arg)
  {
    cl_int clerr = kernel.setArg(argIndex, arg);
    ++argIndex;
    clu::checkError(std::cerr, clerr, "Arg", argIndex);
    return clerr;
  }


  /// @internal
  template <typename T, typename ... ARGS>
  cl_int setKernelArgs_(cl::Kernel &kernel, int &argIndex, const T &arg, ARGS ... args)
  {
    cl_int clerr = kernel.setArg(argIndex, arg);
    ++argIndex;
    clu::checkError(std::cerr, clerr, "Arg", argIndex);
    cl_int clerrOther = setKernelArgs_(kernel, argIndex, args...);
    if (clerr == CL_SUCCESS)
    {
      return clerrOther;
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
  template <typename ... ARGS>
  cl_int setKernelArgs(cl::Kernel &kernel, ARGS ... args)
  {
    int argIndex = 0;
    return setKernelArgs_(kernel, argIndex, args...);
  }


  class KernelSize
  {
  public:
    enum Dimension : unsigned
    {
      DNULL,
      D1D,
      D2D,
      D3D
    };

    inline KernelSize() { _sizes[0] = _sizes[1] = _sizes[2] = 0; }

    inline KernelSize(size_t a)
    {
      _sizes[0] = a;
      _sizes[1] = _sizes[2] = 0;
    }

    inline KernelSize(size_t a, size_t b)
    {
      _sizes[0] = a;
      _sizes[1] = b;
      _sizes[2] = 0;
    }

    inline KernelSize(size_t a, size_t b, size_t c)
    {
      _sizes[0] = a;
      _sizes[1] = b;
      _sizes[2] = c;
    }

    inline KernelSize(const KernelSize &other) { *this = other; }

    inline Dimension dimensions() const
    {
      return (_sizes[2]) ? D3D : ((_sizes[1]) ? D2D : ((_sizes[0]) ? D1D : DNULL));
    }

    inline size_t size(int dim = 0) const { return _sizes[dim]; }

    inline size_t operator[](int dim) const { return _sizes[dim]; }
    inline size_t &operator[](int dim) { return _sizes[dim]; }

    inline const size_t *arg() const { return _sizes; }

    inline bool isNull() const { return dimensions() == DNULL; }
    inline bool isValid() const { return !isNull(); }

    inline size_t volume() const
    {
      size_t total = _sizes[0];
      total *= dimensions() > D1D ? _sizes[1] : 1;
      total *= dimensions() > D2D ? _sizes[2] : 1;
      return total;
    }

    inline KernelSize &operator = (const KernelSize &other)
    {
      _sizes[0] = other._sizes[0];
      _sizes[1] = other._sizes[1];
      _sizes[2] = other._sizes[2];
      return *this;
    }

    inline KernelSize &operator = (size_t size)
    {
      _sizes[0] = size;
      _sizes[1] = _sizes[2] = 0;
      return *this;
    }

  private:
    size_t _sizes[3];
  };


  /// A helper structure for setting the execution size of a kernel.
  struct KernelGrid
  {
    KernelSize globalOffset;
    KernelSize globalSize;
    KernelSize workGroupSize;

    inline KernelGrid() {}

    inline KernelGrid(const KernelSize &globalOffset, const KernelSize &globalSize, const KernelSize &workGroupSize)
      : globalOffset(globalOffset)
      , globalSize(globalSize)
      , workGroupSize(workGroupSize)
    {
    }

    inline KernelGrid(const KernelSize &globalSize, const KernelSize &workGroupSize) : KernelGrid(KernelSize(), globalSize, workGroupSize) {}

    /// Calculates an adjusted global size such that it is a multiple of the work group size.
    /// @return A global size such that it is a multiple of the work group size in each valid dimension.
    KernelSize adjustedGlobal() const;

    inline bool isValid() const
    {
      if (globalSize.isValid() && workGroupSize.isValid() &&
          globalSize.dimensions() == workGroupSize.dimensions() &&
         (globalSize.dimensions() == globalOffset.dimensions() || globalOffset.isNull()))
      {
        for (unsigned i = 0; i < globalSize.dimensions(); ++i)
        {
          if (globalSize.size(i) < workGroupSize.size(i))
          {
            return false;
          }
        }
        return true;
      }

      return false;
    }
  };

  /// Encapsulates events to complete before a kernel can execute as well as exposing an event to mark
  /// kernel completion.
  ///
  /// Used as an argument to @c Kernel::operator() to aid in overloading event setup.
  struct EventList
  {
    cl::Event *completion;
    cl::Event *waitOnEvents;
    unsigned eventCount;

    inline EventList() : completion(nullptr), waitOnEvents(nullptr), eventCount(0) {}
    inline EventList(cl::Event *completionEvent) : completion(completionEvent), waitOnEvents(nullptr), eventCount(0) {}
    inline EventList(cl::Event *waitOn, unsigned eventCount, cl::Event *completionEvent = nullptr)
      : completion(completionEvent), waitOnEvents(eventCount ? waitOn : nullptr), eventCount(eventCount) {}
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
    enum
    {
      MAX_LOCAL_MEM_ARGS = 8,
    };

    /// Typedef for functional objects used to calculate the size of a local memory argument.
    /// The argument passed is the work group size while the return value specifies the
    /// required local memory for the corresponding work group size (bytes).
    typedef std::function<size_t (size_t)> LocalMemArgSizeFunc;

    Kernel();
    Kernel(cl::Program &program, const char *entryPoint, std::ostream *log = nullptr);
    Kernel(cl::Kernel &clKernel);

    /// Is this a valid kernel?
    bool isValid() const;

    /// Set the kernel entry point.
    /// @param program The program to find the kernel in.
    /// @param entryPoint The kernel entry point/function name.
    /// @return @c CL_SUCCESS on success, an OpenCL error code otherwise.
    cl_int setEntry(cl::Program &program, const char *entryPoint, std::ostream *log = nullptr);

    /// Set local memory arguments ordering. See class notes.
    /// @param first True to have local memory arguments before other arguments.
    inline void setLocalMemFirst(bool first) { _localMemFirst = first; }

    /// Do local memory arguments come before other arguments?
    /// @return True if local memory arguments appear before other arguments.
    inline bool localMemFirst() const { return _localMemFirst; }

    /// Adds a local memory argument whose is calculated by invoking @p argFunc.
    ///
    /// See class notes on local memory arguments.
    ///
    /// Note: no more than 8 local arguments arg supported.
    ///
    /// @param argFunc The function to invoke when calculating the local memory size (bytes).
    /// @return The index of the local argument. This is not the same as the final argument index,
    ///   just the index into the set of local arguments, zero based. -1 on failure.
    cl_int addLocal(const LocalMemArgSizeFunc &argFunc);

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
    inline const size_t optimalWorkGroupSize() const { return _optimalWorkGroupSize; }

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
    template <typename ... ARGS>
    cl_int operator()(cl::CommandQueue &queue,
                      const KernelGrid &grid,
                      const EventList &events,
                      ARGS ... args)
    {
      cl_int clerr;
      clerr = preInvoke(args...);
      if (clerr != CL_SUCCESS) { return clerr; }
      return invoke(queue, grid, events);
    }

    inline cl::Kernel &kernel() { return _kernel; }
    inline const cl::Kernel &kernel() const { return _kernel; }

  private:
    /// Preinvocation setup.
    /// @param args Arguments to pass to the kernel.
    template <typename ... ARGS>
    inline cl_int preInvoke(ARGS ... args)
    {
      cl_int clerr;
      int argCount = (!_localMemFirst) ? 0 : _localMemArgCount;
      clerr = setKernelArgs_(_kernel, argCount, args...);
      if (clerr != CL_SUCCESS) { return clerr; }
      clerr = setLocalMemArgs(argCount);
      if (clerr != CL_SUCCESS) { return clerr; }
      return clerr;
    }

    /// Setup local memory arguments.
    /// @param argCount Number of non-local mem args when @c _localMemFirst is false, otherwise it's the total args.
    /// @return @c CL_SUCCESS on success, an OpenCL error code on failure.
    cl_int setLocalMemArgs(int argCount);

    cl_int invoke(cl::CommandQueue &queue, const KernelGrid &grid, const EventList &events);

    cl::Kernel _kernel;
    size_t _optimalWorkGroupSize;
    LocalMemArgSizeFunc _localMemArgs[MAX_LOCAL_MEM_ARGS];
    int _localMemArgCount;
    bool _localMemFirst;
  };
}

#endif // CLUKERNEL_H_
