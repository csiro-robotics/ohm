// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPLATFORM_H
#define GPUPLATFORM_H

#include "gpuConfig.h"

#if GPUTIL_TYPE == GPUTIL_OPENCL
#include "cl/gpuPlatform2.h"
#elif GPUTIL_TYPE == GPUTIL_CUDA
#include "cuda/gpuPlatform2.h"
#else  // GPUTIL_TYPE == ???
#error Unknown GPU base API
#endif // GPUTIL_TYPE

#endif // GPUPLATFORM_H
