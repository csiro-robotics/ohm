// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUDEVICEDETAIL_H_
#define GPUDEVICEDETAIL_H_

#include "gpuconfig.h"

#include <cuda_runtime.h>

#include <string>

namespace gputil
{
  struct DeviceDetail
  {
    int device;
    std::string name;
    std::string info;

    inline DeviceDetail()
      : device(-1)
    {}
  };
}

#endif // GPUDEVICEDETAIL_H_
