// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYNEARESTNEIGHBOURSDETAIL_H_
#define OCCUPANCYNEARESTNEIGHBOURSDETAIL_H_

#include "ohmconfig.h"

#include "occupancyquerydetail.h"

#include <gputil/gpubuffer.h>
#include <gputil/gpudevice.h>
#include <gputil/gpuqueue.h>

// TODO: add a GPU specific header and migrate these includes and the GpuData structure there.
#include <clu/clu.h>

// #define CACHE_LOCAL_RESULTS
// #define VALIDATE_KEYS

namespace ohm
{
  struct NearestNeighboursDetail : QueryDetail
  {
    glm::dvec3 nearPoint;
    float searchRadius;
    gputil::Device gpu;

    struct GpuData
    {
      typedef cl_short3 short3;
      typedef cl_uchar3 uchar3;

      gputil::Queue queue;
      std::vector<float> localRanges;
      std::vector<short3> regionKeys;
      std::vector<uchar3> localKeys;
      gputil::Buffer gpuNodes;
      gputil::Buffer gpuNodeRegionKeys;
      gputil::Buffer gpuNodeVoxelKeys;
      gputil::Buffer gpuRanges;
      gputil::Buffer gpuResulRegionKeys;
      gputil::Buffer gpuResultNodeKeys;
      gputil::Buffer gpuResultCount;

      unsigned queuedNodes;
      unsigned resultCount;

      inline void clear()
      {
        queuedNodes = resultCount = 0u;
      }

    } gpuData;
    bool gpuOk;

    inline NearestNeighboursDetail()
      : searchRadius(0)
    {
      gpuData.clear();
      gpuOk = false;
    }
  };
}

#endif // OCCUPANCYNEARESTNEIGHBOURSDETAIL_H_
