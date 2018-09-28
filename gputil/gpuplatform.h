// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPLATFORM_H_
#define GPUPLATFORM_H_

#include "gpuconfig.h"

#if GPUTIL_TYPE == GPUTIL_OPENCL
#include "cl/gpuplatform2.h"
#elif GPUTIL_TYPE == GPUTIL_CUDA
#include "cuda/gpuplatform2.h"
#else  // GPUTIL_TYPE == ???
#error Unknown GPU base API
#endif // GPUTIL_TYPE

#endif // GPUPLATFORM_H_
