// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuKernel.h"

#include "gputil/gpuApiException.h"
#include "gputil/gpuBuffer.h"
#include "gputil/gpuDevice.h"
#include "gputil/gpuEventList.h"
#include "gputil/gpuThrow.h"

#include "gputil/cuda/gpuBufferDetail.h"
#include "gputil/cuda/gpuDeviceDetail.h"
#include "gputil/cuda/gpuEventDetail.h"
#include "gputil/cuda/gpuKernelDetail.h"
#include "gputil/cuda/gpuQueueDetail.h"

#include <cuda_runtime.h>

#include <algorithm>
#include <cstring>

namespace gputil
{
namespace cuda
{
int preInvokeKernel(const Device &device)
{
  // Select device.
  cudaError_t err = cudaSuccess;
  err = cudaSetDevice(device.detail()->device);
  GPUAPICHECK(err, cudaSuccess, err);
  // cudaDeviceProp info;
  // memset(&info, 0, sizeof(info));
  // err = cudaGetDeviceProperties(&info, device.detail()->device);
  // GPUAPICHECK(err, cudaSuccess, err);
  return err;
}


size_t calcSharedMemSize(const KernelDetail &imp, size_t block_threads)
{
  // To calculate the total size, sum the desired sizes with 8 byte alignment in between. This may be larger than
  // required.
  size_t shared_mem_size = 0u;
  const size_t alignment = 8u;
  size_t alignment_remainder = 0;
  for (auto &&local_mem_func : imp.local_mem_args)
  {
    shared_mem_size += local_mem_func(block_threads);
    // Align to 8 bytes.
    alignment_remainder = shared_mem_size % alignment;
    shared_mem_size += !!(alignment_remainder) * (alignment - alignment_remainder);
  }
  return shared_mem_size;
}


int invokeKernel(const KernelDetail &imp, const Dim3 &global_size, const Dim3 &local_size, const EventList *event_list,
                 Event *completion_event, Queue *queue, void **args, size_t /*arg_count*/)
{
  cudaError_t err = cudaSuccess;

  // TODO(KS): Enable once kernel definition arguments are counted.
  // if (arg_count != imp.arg_count)
  // {
  //   err = cudaErrorLaunchFailure;
  //   const char *msg = (arg_count > imp.arg_count) ? "too many arguments" : "too few arguments";
  //   GPUTHROW(ApiException(err, msg), err);
  //   return err;
  // }

  // Resolve shared memory size
  const size_t shared_mem_size = calcSharedMemSize(imp, local_size.volume());

  // Resolve cuda stream.
  cudaStream_t cuda_stream = (queue) ? queue->internal()->obj() : nullptr;

  // Make the stream wait on the event list.
  if (event_list)
  {
    const Event *event = event_list->events();
    for (size_t i = 0; i < event_list->count(); ++i, ++event)
    {
      if (event && event->detail() && event->detail()->obj())
      {
        cudaEvent_t cuda_event = event->detail()->obj();
        const unsigned flags = 0u;  // Must be zero at the time of implementation.
        err = cudaStreamWaitEvent(cuda_stream, cuda_event, flags);
        GPUAPICHECK(err, cudaSuccess, err);
      }
    }
  }

  // int n = 0;
  // std::vector<void *> dummy_args(5);
  // for (int i = 0; i < 5; ++i)
  // {
  //   dummy_args[i] = nullptr;
  // }
  // dummy_args[3] = &n;

  // args = dummy_args.data();

  // Launch kernel.
  dim3 grid_dim;
  grid_dim.x = (local_size.x) ? unsigned((global_size.x + local_size.x - 1) / local_size.x) : 1u;
  grid_dim.y = (local_size.y) ? unsigned((global_size.y + local_size.y - 1) / local_size.y) : 1u;
  grid_dim.z = (local_size.z) ? unsigned((global_size.z + local_size.z - 1) / local_size.z) : 1u;
  err = cudaLaunchKernel(imp.cuda_kernel_function, grid_dim,
                         dim3(unsigned(local_size.x), unsigned(local_size.y), unsigned(local_size.z)), args,
                         shared_mem_size, cuda_stream);
  GPUAPICHECK(err, cudaSuccess, err);

  // Hook up completion event.
  if (completion_event)
  {
    // Create new event.
    completion_event->release();
    err = cudaEventRecord(completion_event->detail()->obj(), cuda_stream);
    GPUAPICHECK(err, cudaSuccess, err);
  }
  return err;
}
}  // namespace cuda

Kernel::Kernel()
  : imp_(new KernelDetail)
{}


Kernel::Kernel(Kernel &&other) noexcept
  : imp_(other.imp_)
{
  other.imp_ = nullptr;
}


Kernel::~Kernel()
{
  delete imp_;
}


bool Kernel::isValid() const
{
  return imp_ && imp_->cuda_kernel_function;
}


void Kernel::release()
{
  delete imp_;
  imp_ = nullptr;
}

void Kernel::addLocal(const std::function<size_t(size_t)> &local_calc)
{
  imp_->local_mem_args.push_back(local_calc);
}


size_t Kernel::calculateOptimalWorkGroupSize()
{
  const size_t default_workgroup_size = 8;
  if (!imp_->maximum_potential_workgroup_size)
  {
    int err = cuda::preInvokeKernel(device());
    GPUAPICHECK(err, cudaSuccess, 0u);
    const auto calc_shared_mem_size = [this](size_t block_size)  //
    {                                                            //
      return cuda::calcSharedMemSize(*imp_, block_size);
    };
    err = imp_->optimal_group_size_calc(&imp_->maximum_potential_workgroup_size, calc_shared_mem_size);
    if (!imp_->maximum_potential_workgroup_size)
    {
      imp_->maximum_potential_workgroup_size = default_workgroup_size;
    }
    GPUAPICHECK(err, cudaSuccess, 0);
  }

  return imp_->maximum_potential_workgroup_size;
}


size_t Kernel::optimalWorkGroupSize() const
{
  return imp_->maximum_potential_workgroup_size;
}

void Kernel::calculateGrid(gputil::Dim3 *global_size, gputil::Dim3 *local_size, const gputil::Dim3 &total_work_items)
{
  if (!isValid())
  {
    *global_size = *local_size = Dim3(0, 0, 0);
    return;
  }

  const size_t calc_volume = total_work_items.x * total_work_items.y * total_work_items.z;
  const size_t target_group_size = std::min(calculateOptimalWorkGroupSize(), calc_volume);
  const Device &gpu = imp_->program.device();

  cudaError_t err = cudaSuccess;
  cudaDeviceProp cuda_info{};
  memset(&cuda_info, 0, sizeof(cuda_info));
  err = cudaGetDeviceProperties(&cuda_info, gpu.detail()->device);
  GPUAPICHECK2(err, cudaSuccess);

  // Try to setup the workgroup as a cubic spatial division.
  const double cube_root = 1.0f / 3.0f;
  auto target_dimension_value = unsigned(std::floor(std::pow(double(target_group_size), cube_root)));
  if (target_dimension_value < 1)
  {
    target_dimension_value = 1;
  }

  // Set the target dimensions to the minimum of the target and the max work group size.
  const double sqr_root = 1.0f / 2.0f;
  local_size->z = std::min<size_t>(cuda_info.maxThreadsDim[2], target_dimension_value);
  // Lint(KS): division should be OK to convert to double for square root, then to int.
  // NOLINTNEXTLINE(bugprone-integer-division)
  target_dimension_value = unsigned(std::floor(std::pow(double(target_group_size / local_size->z), sqr_root)));
  local_size->y = std::min<size_t>(cuda_info.maxThreadsDim[1], target_dimension_value);
  target_dimension_value = unsigned(std::max<size_t>(target_group_size / (local_size->y * local_size->z), 1));
  local_size->x = std::min<size_t>(cuda_info.maxThreadsDim[0], target_dimension_value);

  // Reduce size to <= targetGroupSize
  int i = 2;  // Start by reducing 3rd dimension.
  while (i >= 0 && local_size->volume() > target_group_size)
  {
    if ((*local_size)[i] > 1)
    {
      (*local_size)[i] = (*local_size)[i] - 1;
    }
    else
    {
      // Don't reduce to less than 1. Next dimension.
      --i;
    }
  }

  // max work item sizes
  global_size->x = local_size->x * (total_work_items.x + local_size->x - 1) / local_size->x;
  global_size->y = local_size->y * (total_work_items.y + local_size->y - 1) / local_size->y;
  global_size->z = local_size->z * (total_work_items.z + local_size->z - 1) / local_size->z;
}


Device Kernel::device()
{
  if (!isValid())
  {
    return Device();
  }

  return imp_->program.device();
}


Kernel &Kernel::operator=(Kernel &&other) noexcept
{
  delete imp_;
  imp_ = other.imp_;
  other.imp_ = nullptr;
  return *this;
}


Kernel cudaKernel(Program &program, const void *kernel_function_ptr,
                  const gputil::OptimalGroupSizeCalculation &group_calc)
{
  Kernel kernel;
  kernel.detail()->cuda_kernel_function = kernel_function_ptr;
  kernel.detail()->optimal_group_size_calc = group_calc;
  kernel.detail()->program = program;
  // TODO(KS): count arguments
  return kernel;
}
}  // namespace gputil
