// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "OhmConfig.h"

#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuEvent.h>
#include <gputil/gpuKernel.h>

namespace ohm
{
  struct GpuTransformSamplesDetail
  {
    static const unsigned kUploadEventCount = 4;

    gputil::Buffer transform_positions_buffer;
    gputil::Buffer transform_rotations_buffer;
    gputil::Buffer transform_times_buffer;
    gputil::Event upload_events[kUploadEventCount];
    gputil::Device gpu;
    gputil::Kernel kernel;
  };
} // namespace ohm
