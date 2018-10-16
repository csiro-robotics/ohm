// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUKERNELDETAIL_H
#define GPUKERNELDETAIL_H

#include "gpuConfig.h"

#include "gpuDevice.h"
#include "gpuProgram.h"

#include <clu/cluKernel.h>

#include <functional>

namespace gputil
{
  struct KernelDetail
  {
    Device gpu;
    clu::Kernel kernel;
    Program program;
    std::vector<std::function<size_t(size_t)>> local_mem_args;
  };
}

#endif // GPUKERNELDETAIL_H
