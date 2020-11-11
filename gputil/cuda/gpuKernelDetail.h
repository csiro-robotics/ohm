// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUKERNELDETAIL_H
#define GPUKERNELDETAIL_H

#include "gpuConfig.h"

#include "gputil/gpuDevice.h"
#include "gputil/gpuProgram.h"

#include <gputil/cuda/cutil_decl.h>

#include <functional>

namespace gputil
{
struct KernelDetail
{
  const void *cuda_kernel_function = nullptr;
  OptimalGroupSizeCalculation optimal_group_size_calc;
  size_t arg_count = 0u;
  std::vector<std::function<size_t(size_t)>> local_mem_args;
  size_t maximum_potential_workgroup_size = 0;
  Program program;
};
}  // namespace gputil

#endif  // GPUKERNELDETAIL_H
