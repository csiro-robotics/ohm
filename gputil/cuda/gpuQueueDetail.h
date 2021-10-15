// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUQUEUEDETAIL_H
#define GPUQUEUEDETAIL_H

#include "gpuConfig.h"

#include "gputil/gpuApiException.h"
#include "gputil/gpuThrow.h"

#include <cuda_runtime.h>

#include <utility>

namespace gputil
{
struct QueueDetail
{
  cudaStream_t queue = nullptr;
  bool force_synchronous = false;

  inline ~QueueDetail()
  {
    if (queue)
    {
      cudaError_t err = cudaStreamDestroy(queue);
      if (err != cudaSuccess)
      {
        // Dangerous to throw from destructor, so just log on exceptions.
        gputil::log(gputil::ApiException(err, nullptr, __FILE__, __LINE__));
      }
    }
  }
};
}  // namespace gputil

#endif  // GPUQUEUEDETAIL_H
