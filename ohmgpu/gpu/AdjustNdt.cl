// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef ADJUSTNDT_CL
#define ADJUSTNDT_CL

#include "NdtVoxel.h"

inline __device__ float calculateOccupancyAdjustment(bool isEndVoxel, struct LineWalkData *line_data, float voxel_resolution)
{
#ifdef VOXEL_MEAN
  float3 voxel_mean = subVoxelToLocalCoord(voxels[vi].sub_voxel, voxel_resolution);
#else
#error No voxel mean information available
#endif  // VOXEL_MEAN

  float adjustment = 0;
  const int min_sample_threshold = 4; // Should be passed in.
  const float3 voxel_maximum_likelyhood =
    calculateMiss(ndt_voxel, &adjustment, sensor, sample, voxel_mean, line_data->occupied_threshold,
                  FLT_MAX, line_data->ray_adjustment, line_data->sensor_noise, min_sample_threshold);

  // NDT should do sample update in a separate process in order to update the covariance, so we should not get here.
  return (!isEndVoxel || line_data->region_update_flags & kRfEndPointAsFree) ? adjustment : 0;
}

#endif // ADJUSTNDT_CL
