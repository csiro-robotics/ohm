// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gputil/gpuApiException.h"

#include <cuda_runtime.h>

using namespace gputil;

const char *ApiException::errorCodeString(int error_code)
{
  return cudaGetErrorString(static_cast<cudaError_t>(error_code));
}
