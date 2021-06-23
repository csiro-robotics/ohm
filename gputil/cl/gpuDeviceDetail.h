// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUDEVICEDETAIL_H
#define GPUDEVICEDETAIL_H

#include "gpuConfig.h"

#include "gputil/gpuDeviceInfo.h"
#include "gputil/gpuQueue.h"

#include <clu/clu.h>

namespace gputil
{
struct DeviceDetail
{
  cl::Context context;
  cl::Device device;
  /// The default queue object. We need a Queue object rather than just a cl::CommandQueue because we add data members.
  Queue default_queue;
  DeviceInfo info;
  std::string description;
  std::string search_paths;
  std::string extensions;  ///< OpenCL supported extension string.
  unsigned debug = 0;
};
}  // namespace gputil

#endif  // GPUDEVICEDETAIL_H
