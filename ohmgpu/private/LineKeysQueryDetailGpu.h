
// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMGPU_LINEKEYSQUERYDETAILGPU_H_
#define OHMGPU_LINEKEYSQUERYDETAILGPU_H_OHMGPU_LINEKEYSQUERYDETAILGPU_HOHMGPU_LINEKEYSQUERYDETAILGPU_H_

#include "OhmGpuConfig.h"

#include <ohm/private/LineKeysQueryDetail.h>

// Include GPU structure definition.
#include "GpuKey.h"

#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuQueue.h>

#include <atomic>

namespace ohm
{
struct LineKeysQueryDetailGpu : public LineKeysQueryDetail
{
  gputil::Kernel line_keys_kernel;
  gputil::Device gpu;

  gputil::Queue queue;

  gputil::Buffer lines_out;
  gputil::Buffer line_points;
  unsigned max_keys_per_line = 0;
  std::atomic_bool inflight{ false };

  bool gpu_ok = false;
};
}  // namespace ohm

#endif  // OHMGPU_LINEKEYSQUERYDETAILGPU_H_
