// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUEVENTDETAIL_H
#define GPUEVENTDETAIL_H

#include "gpuConfig.h"

#include <clu/clu.h>

namespace gputil
{
  struct EventDetail
  {
    cl_event event;
  };
}  // namespace gputil

#endif  // GPUEVENTDETAIL_H
