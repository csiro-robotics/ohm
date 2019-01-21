// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_NEARESTNEIGHBOURSDETAIL_H
#define OHM_NEARESTNEIGHBOURSDETAIL_H

#include "OhmConfig.h"

#include "QueryDetail.h"

#include <gputil/gpuBuffer.h>
#include <gputil/gpuDevice.h>
#include <gputil/gpuKernel.h>
#include <gputil/gpuPlatform.h>
#include <gputil/gpuQueue.h>

// #define CACHE_LOCAL_RESULTS
// #define VALIDATE_KEYS

namespace ohm
{
  struct NearestNeighboursDetail : QueryDetail
  {
    glm::dvec3 near_point = glm::dvec3(0);
    float search_radius = 0;
    gputil::Device gpu;

    struct GpuData
    {
      gputil::Queue queue;
      std::vector<float> local_ranges;
      std::vector<gputil::short3> region_keys;
      std::vector<gputil::uchar3> local_keys;
      gputil::Buffer gpu_voxels;
      gputil::Buffer gpu_voxel_region_keys;
      gputil::Buffer gpu_voxel_voxel_keys;
      gputil::Buffer gpu_ranges;
      gputil::Buffer gpu_result_region_keys;
      gputil::Buffer gpu_result_voxel_keys;
      gputil::Buffer gpu_result_count;
      gputil::Kernel nn_kernel;
      gputil::Kernel info_kernel;

      unsigned queued_voxels = 0u;
      unsigned result_count = 0u;

      inline void clear() { queued_voxels = result_count = 0u; }

    } gpu_data;
    bool gpu_ok = false;
  };
}  // namespace ohm

#endif  // OHM_NEARESTNEIGHBOURSDETAIL_H
