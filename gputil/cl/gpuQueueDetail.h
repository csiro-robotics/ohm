// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUQUEUEDETAIL_H
#define GPUQUEUEDETAIL_H

#include "gpuConfig.h"

#include <clu/clu.h>

namespace gputil
{
  struct QueueDetail
  {
    cl::CommandQueue queue;
  };
}

#endif // GPUQUEUEDETAIL_H
