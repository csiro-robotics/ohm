// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef GPUDEVICEDETAIL_H
#define GPUDEVICEDETAIL_H

#include <clu/clu.h>

namespace gputil
{
  struct DeviceDetail
  {
    cl::Context context;
    cl::Device device;
    // TODO: this needs to be separated out into a queue object.
    // We may preserve this member as the default queue.
    cl::CommandQueue queue;
    std::string name;
    std::string info;
    std::string search_paths;
    unsigned debug = 0;
  };
}

#endif // GPUDEVICEDETAIL_H
