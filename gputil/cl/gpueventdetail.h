// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUEVENTDETAIL_H_
#define GPUEVENTDETAIL_H_

#include "gpuconfig.h"

#include <clu.h>

namespace gputil
{
  struct EventDetail
  {
    cl_event event;
  };
}

#endif // GPUEVENTDETAIL_H_
