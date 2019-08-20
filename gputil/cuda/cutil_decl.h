// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CUTIL_DECL_H
#define CUTIL_DECL_H

#include <functional>

// Helper macros for exposing CUDA kernel functions.

namespace gputil
{
  using SharedMemCalculation = std::function<size_t(size_t)>;
  using OptimalGroupSizeCalculation = std::function<int(size_t *, const SharedMemCalculation &)>;
}  // namespace gputil

#define _GPUTIL_DECLARE_KERNEL(kernel_name) const void *kernel_name##Ptr()
#define _GPUTIL_DECLARE_GROUP_CALC(kernel_name) \
  gputil::OptimalGroupSizeCalculation kernel_name##OptimalGroupSizeCalculator()

/// Delcaration of the a function which exposes a CUDA kernel for use with gputil.
/// Returns a pointer to the CUDA kernel.
#define GPUTIL_CUDA_DECLARE_KERNEL(kernel_name) \
  _GPUTIL_DECLARE_KERNEL(kernel_name);          \
  _GPUTIL_DECLARE_GROUP_CALC(kernel_name)


/// Definition of the a function which exposes a CUDA kernel for use with gputil.
#define GPUTIL_CUDA_DEFINE_KERNEL(kernel_name)                                                                 \
  __host__ _GPUTIL_DECLARE_KERNEL(kernel_name) { return (const void *)&(kernel_name); }                        \
  __host__ _GPUTIL_DECLARE_GROUP_CALC(kernel_name)                                                             \
  {                                                                                                            \
    return [](size_t *optimal_size, const gputil::SharedMemCalculation &shared_mem_calc) {                     \
      int min_grid_size = 0, optimal_block_size = 0;                                                           \
      cudaError_t err = cudaSuccess;                                                                           \
      err = ::cudaOccupancyMaxPotentialBlockSizeVariableSMem(&min_grid_size, &optimal_block_size, kernel_name, \
                                                             shared_mem_calc);                                 \
      *optimal_size = size_t(optimal_block_size);                                                              \
      return err;                                                                                              \
    };                                                                                                         \
  }

#endif  // CUTIL_DECL_H
