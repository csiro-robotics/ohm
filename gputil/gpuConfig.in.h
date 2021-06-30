// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUCONFIG_H
#define GPUCONFIG_H

#include "gputilExport.h"

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif  // _USE_MATH_DEFINES
#ifndef NOMINMAX
#define NOMINMAX
#endif  // NOMINMAX
#ifndef NOMINMAX
#define NOMINMAX
#endif  // NOMINMAX
#include <cmath>

// clang-format off
#define GPUTIL_NONE 0
#define GPUTIL_OPENCL 1
#define GPUTIL_CUDA 2
// clang-format on

#endif  // GPUCONFIG_H
