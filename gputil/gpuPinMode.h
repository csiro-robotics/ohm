// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPINMODE_H
#define GPUPINMODE_H

#include "gpuConfig.h"

namespace gputil
{
  // Pinning functions. Need to check CUDA for an equivalent to OpenCL pinning.
  enum PinMode
  {
    kPinRead,
    kPinWrite,
    kPinReadWrite
  };
}

#endif // GPUPINMODE_H
