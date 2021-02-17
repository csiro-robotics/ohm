// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUKERNEL2_H
#define GPUKERNEL2_H

#include "gpuConfig.h"

#include "gputil/gpuBuffer.h"
#include "gputil/gpuEventList.h"

#include "gputil/cuda/gpuBufferDetail.h"
#include "gputil/cuda/gpuEventDetail.h"
#include "gputil/cuda/gpuKernelDetail.h"
#include "gputil/cuda/gpuQueueDetail.h"

#include "gputil/cuda/cutil_decl.h"

#include <cuda_runtime.h>

#include <cstdlib>

#define GPUTIL_BUILD_FROM_FILE(program, file_name, build_args) 0
#define GPUTIL_BUILD_FROM_SOURCE(program, source, source_length, build_args) 0
#define GPUTIL_MAKE_KERNEL(program, kernel_name) \
  gputil::cudaKernel(program, kernel_name##Ptr(), kernel_name##OptimalGroupSizeCalculator())

namespace gputil
{
namespace cuda
{
inline size_t countArgs()
{
  return 0u;
}

template <typename ARG, typename... ARGS>
inline size_t countArgs(const ARG &, ARGS... args)  // NOLINT(readability-named-parameter)
{
  return 1u + countArgs(args...);
}

template <typename ARG>
inline void *collateArgPtr(ARG *arg)
{
  // Necessary evils
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-const-cast, cppcoreguidelines-pro-type-reinterpret-cast)
  return const_cast<void *>(reinterpret_cast<const void *>(arg));
}

template <typename T>
inline void *collateArgPtr(BufferArg<T> *arg)
{
  return arg->buffer->argPtr();
}

template <typename T>
inline void *collateArgPtr(const BufferArg<T> *arg)
{
  return arg->buffer->argPtr();
}

inline void collateArgs(unsigned /*index*/, void ** /*collated_args*/)
{
  // NOOP
}

template <typename ARG, typename... ARGS>
inline void collateArgs(unsigned index, void **collated_args, const ARG &arg, ARGS... args)
{
  collated_args[index] = collateArgPtr(arg);
  collateArgs(index + 1, collated_args, args...);
}

int preInvokeKernel(const Device &device);
int invokeKernel(const KernelDetail &imp, const Dim3 &global_size, const Dim3 &local_size, const EventList *event_list,
                 Event *completion_event, Queue *queue, void **args, size_t arg_count);
}  // namespace cuda

template <typename... ARGS>
int Kernel::operator()(const Dim3 &global_size, const Dim3 &local_size, Queue *queue, ARGS... args)
{
  // Prime device
  int err = 0;
  err = cuda::preInvokeKernel(device());
  if (err)
  {
    return err;
  }

  // Collate arguments into void **
  size_t arg_count = cuda::countArgs(args...);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  void **collated_args = (arg_count) ? reinterpret_cast<void **>(alloca(arg_count * sizeof(void *))) : nullptr;
  // Capture args by pointer as we will be packing that address into collated_args and it must stay valid.
  cuda::collateArgs(0, collated_args, &args...);

  // Invoke
  err = cuda::invokeKernel(*detail(), global_size, local_size, nullptr, nullptr, queue, collated_args, arg_count);
  return err;
}


template <typename... ARGS>
int Kernel::operator()(const Dim3 &global_size, const Dim3 &local_size, Event &completion_event, Queue *queue,
                       ARGS... args)
{
  // Prime device
  int err = 0;
  err = cuda::preInvokeKernel(device());
  if (err)
  {
    return err;
  }

  // Collate arguments into void **
  size_t arg_count = cuda::countArgs(args...);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  void **collated_args = (arg_count) ? reinterpret_cast<void **>(alloca(arg_count * sizeof(void *))) : nullptr;
  // Capture args by pointer as we will be packing that address into collated_args and it must stay valid.
  cuda::collateArgs(0, collated_args, &args...);

  // Invoke
  err =
    cuda::invokeKernel(*detail(), global_size, local_size, nullptr, &completion_event, queue, collated_args, arg_count);
  return err;
}


template <typename... ARGS>
int Kernel::operator()(const Dim3 &global_size, const Dim3 &local_size, const EventList &event_list, Queue *queue,
                       ARGS... args)
{
  // Prime device
  int err = 0;
  err = cuda::preInvokeKernel(device());
  if (err)
  {
    return err;
  }

  // Collate arguments into void **
  size_t arg_count = cuda::countArgs(args...);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  void **collated_args = (arg_count) ? reinterpret_cast<void **>(alloca(arg_count * sizeof(void *))) : nullptr;
  // Capture args by pointer as we will be packing that address into collated_args and it must stay valid.
  cuda::collateArgs(0, collated_args, &args...);

  // Invoke
  err = cuda::invokeKernel(*detail(), global_size, local_size, &event_list, nullptr, queue, collated_args, arg_count);
  return err;
}


template <typename... ARGS>
int Kernel::operator()(const Dim3 &global_size, const Dim3 &local_size, const EventList &event_list,
                       Event &completion_event, Queue *queue, ARGS... args)
{
  // Prime device
  int err = 0;
  err = cuda::preInvokeKernel(device());
  if (err)
  {
    return err;
  }

  // Collate arguments into void **
  size_t arg_count = cuda::countArgs(args...);
  // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
  void **collated_args = (arg_count) ? reinterpret_cast<void **>(alloca(arg_count * sizeof(void *))) : nullptr;
  // Capture args by pointer as we will be packing that address into collated_args and it must stay valid.
  cuda::collateArgs(0, collated_args, &args...);

  // Invoke
  err = cuda::invokeKernel(*detail(), global_size, local_size, &event_list, &completion_event, queue, collated_args,
                           arg_count);
  return err;
}


Kernel cudaKernel(Program &program, const void *kernel_function_ptr,
                  const gputil::OptimalGroupSizeCalculation &group_calc);
}  // namespace gputil

#endif  // GPUKERNEL2_H
