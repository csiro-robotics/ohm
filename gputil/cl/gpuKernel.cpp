// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpuKernel.h"

#include "gpuApiException.h"
#include "gpuDevice.h"
#include "gpuKernelDetail.h"
#include "gpuProgram.h"
#include "gpuProgramDetail.h"
#include "gpuThrow.h"

#include "gpuDeviceDetail.h"

#include <algorithm>

using namespace gputil;

Kernel::Kernel()
  : imp_(new KernelDetail)
{
  imp_->kernel.setLocalMemFirst(false);
}


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
  return imp_ && imp_->kernel.isValid();
}


void Kernel::release()
{
  delete imp_;
  imp_ = nullptr;
}

void Kernel::addLocal(const std::function<size_t(size_t)> &local_calc)
{
  imp_->local_mem_args.push_back(local_calc);
  imp_->kernel.addLocal(local_calc);
}


size_t Kernel::calculateOptimalWorkGroupSize()
{
  return imp_->kernel.calculateOptimalWorkGroupSize();
}


size_t Kernel::optimalWorkGroupSize() const
{
  return imp_->kernel.optimalWorkGroupSize();
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

  // Try to setup the workgroup as a cubic spatial division.
  // However we have to consider where the max work item dimensions don't allow this.
  cl_uint group_dim = 3;
  cl_int err = 0;
  err =
    clGetDeviceInfo(gpu.detail()->device(), CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(group_dim), &group_dim, nullptr);
  if (err)
  {
    GPUTHROW2(ApiException(err));
  }
  size_t *max_work_size = static_cast<size_t *>(alloca(sizeof(size_t) * group_dim));
  err = clGetDeviceInfo(gpu.detail()->device(), CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(*max_work_size) * group_dim,
                        max_work_size, nullptr);
  if (err)
  {
    GPUTHROW2(ApiException(err));
  }
  max_work_size[0] = std::min<size_t>(max_work_size[0], total_work_items.x);
  max_work_size[1] = std::min<size_t>(max_work_size[1], total_work_items.y);
  max_work_size[2] = std::min<size_t>(max_work_size[2], total_work_items.z);

  unsigned target_dimension_value = unsigned(std::floor(std::pow(float(target_group_size), 1.0f / 3.0f)));
  if (target_dimension_value < 1)
  {
    target_dimension_value = 1;
  }

  // Set the target dimensions to the minimum of the target and the max work group size.
  local_size->z = std::min<size_t>(max_work_size[2], target_dimension_value);
  target_dimension_value = unsigned(std::floor(std::pow(float(target_group_size / local_size->z), 1.0f / 2.0f)));
  local_size->y = std::min<size_t>(max_work_size[1], target_dimension_value);
  target_dimension_value = unsigned(std::max<size_t>(target_group_size / (local_size->y * local_size->z), 1));
  local_size->x = std::min<size_t>(max_work_size[0], target_dimension_value);

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


namespace gputil
{
Kernel openCLKernel(Program &program, const char *kernel_name)
{
  Kernel kernel;

  cl_int err = 0;
  if (!program.isValid())
  {
    err = CL_INVALID_PROGRAM;
    GPUTHROW(ApiException(err), Kernel());
  }

  err = kernel.detail()->kernel.setEntry(program.detail()->program, kernel_name);
  if (err)
  {
    GPUTHROW(ApiException(err), Kernel());
  }
  kernel.detail()->program = program;
  return kernel;
}
}  // namespace gputil
