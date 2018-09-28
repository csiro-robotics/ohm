// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPINMODE_H_
#define GPUPINMODE_H_

#include "gpuconfig.h"

#include "gpupinmode.h"

namespace gputil
{
  // Pinning functions. Need to check CUDA for an equivalent to OpenCL pinning.
  enum PinMode
  {
    PinRead,
    PinWrite,
    PinReadWrite
  };
}

#endif // GPUPINMODE_H_
