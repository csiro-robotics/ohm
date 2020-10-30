// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_LINEQUERYDETAILGPU_H
#define OHMGPU_LINEQUERYDETAILGPU_H

#include "OhmGpuConfig.h"

#include <ohm/private/LineQueryDetail.h>

#include <ohm/KeyList.h>

namespace ohm
{
class ClearanceProcess;

struct LineQueryDetailGpu : LineQueryDetail
{
  ClearanceProcess *clearance_calculator = nullptr;
};
}  // namespace ohm

#endif  // OHMGPU_LINEQUERYDETAILGPU_H
