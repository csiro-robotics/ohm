// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef ADJUSTNDT_CL
#define ADJUSTNDT_CL

#include "NdtVoxel.h"

inline __device__ float calculateOccupancyAdjustment(const struct GpuKey *voxelKey, bool isEndVoxel,
                                                     const struct GpuKey *startKey, const struct GpuKey *endKey,
                                                     float voxel_resolution, struct LineWalkData *line_data)
{
#ifndef VOXEL_MEAN
#error VOXEL_MEAN must be enabled for NDT update.
#endif // !VOXEL_MEAN

  const ulonglong vi_local = voxelKey->voxel[0] + voxelKey->voxel[1] * line_data->region_dimensions.x +
                             voxelKey->voxel[2] * line_data->region_dimensions.x * line_data->region_dimensions.y;
  ulonglong vi = (line_data->means_offsets[line_data->current_region_index] / sizeof(*line_data->means)) + vi_local;
  VoxelMean *mean_data = &line_data->means[vi];
  float3 voxel_mean = subVoxelToLocalCoord(mean_data->coord, voxel_resolution);
  // voxel_mean is currently relative to the voxel centre. We need to change it to be in the same reference
  // frame as the incoming rays. We need to calculate the voxel centre relative to the first ray start coordinate.
  //
  // line_data->sample is relative to the line_data->sensor, so we can get the right voxel mean as follows:
  // 1. Calculate the number of voxel between line_data->start_key and voxelKey (per axis)
  // 2. Scale this result by the voxel_resolution : this is the centre of voxelKey
  // 3. Add to voxel_mean

  // Calculate the number of voxel steps to the voxelKey
  const int3 voxel_diff = keyDiff(voxelKey, startKey, &line_data->region_dimensions);
  // Scale by voxel resolution and add to voxel_mean
  voxel_mean.x += voxel_diff.x * voxel_resolution;
  voxel_mean.y += voxel_diff.y * voxel_resolution;
  voxel_mean.z += voxel_diff.z * voxel_resolution;

  vi = (line_data->ndt_offsets[line_data->current_region_index] / sizeof(*line_data->ndt_voxels)) + vi_local;
  NdtVoxel *ndt_voxel = &line_data->ndt_voxels[vi];

  float adjustment = 0;
  const int min_sample_threshold = 4;  // Should be passed in.
  const float3 voxel_maximum_likelyhood = calculateMiss(
    ndt_voxel, &adjustment, line_data->sensor, line_data->sample, voxel_mean, mean_data->count,
    line_data->occupied_threshold, INFINITY, line_data->ray_adjustment, line_data->sensor_noise, min_sample_threshold);

  // NDT should do sample update in a separate process in order to update the covariance, so we should not get here.
  return (!isEndVoxel || line_data->region_update_flags & kRfEndPointAsFree) ? adjustment : 0;
}

#endif  // ADJUSTNDT_CL
