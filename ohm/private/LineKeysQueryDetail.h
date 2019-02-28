
// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYLINEKEYSQUERYDETAIL_H_
#define OCCUPANCYLINEKEYSQUERYDETAIL_H_

#include "OhmConfig.h"

#include "QueryDetail.h"

// Include GPU structure definition.
#include "GpuKey.h"

#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuQueue.h>

#include <atomic>

namespace ohm
{
  struct LineKeysQueryDetail : QueryDetail
  {
    std::vector<glm::dvec3> rays;
    std::vector<size_t> resultIndices;
    std::vector<size_t> resultCounts;

    gputil::Kernel line_keys_kernel;
    gputil::Device gpu;

    struct GpuData
    {
      typedef cl_float3 float3;
      gputil::Queue queue;

      gputil::Buffer linesOut;
      gputil::Buffer linePoints;
      unsigned maxKeysPerLine;
      std::atomic_bool inflight;

      inline GpuData()
        : maxKeysPerLine(0)
        , inflight(false)
      {
      }
    } gpuData;
    bool gpuOk;

    inline LineKeysQueryDetail()
    {
      gpuOk = false;
    }
  };
}

#endif // OCCUPANCYLINEKEYSQUERYDETAIL_H_
