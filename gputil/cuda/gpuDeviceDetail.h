// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUDEVICEDETAIL_H
#define GPUDEVICEDETAIL_H

#include "gpuConfig.h"

#include "gputil/gpuDeviceInfo.h"

#include <cuda_runtime.h>

#include <string>

namespace gputil
{
struct DeviceDetail
{
  int device;
  std::string name;
  DeviceInfo info;

  inline DeviceDetail()
    : device(-1)
  {}
};
}  // namespace gputil

#endif  // GPUDEVICEDETAIL_H
