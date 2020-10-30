// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef GPUPROGRAMDETAIL_H
#define GPUPROGRAMDETAIL_H

#include "gpuConfig.h"

#include "gputil/gpuDevice.h"

#include <string>

namespace gputil
{
struct ProgramDetail
{
  Device device;
  std::string program_name;
};
}  // namespace gputil

#endif  // GPUPROGRAMDETAIL_H
