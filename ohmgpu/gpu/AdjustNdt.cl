// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef ADJUSTNDT_CL
#define ADJUSTNDT_CL

#include "CovarianceVoxel.h"

inline __device__ float calculateOccupancyAdjustment(const GpuKey *voxelKey, bool isEndVoxel, const GpuKey *startKey,
                                                     const GpuKey *endKey, float voxel_resolution,
                                                     LineWalkData *line_data)
{
#ifndef VOXEL_MEAN
#error VOXEL_MEAN must be enabled for NDT update.
#endif  // !VOXEL_MEAN

  const ulonglong vi_local = voxelKey->voxel[0] + voxelKey->voxel[1] * line_data->region_dimensions.x +
                             voxelKey->voxel[2] * line_data->region_dimensions.x * line_data->region_dimensions.y;
  ulonglong vi = (line_data->means_offsets[line_data->current_region_index] / sizeof(*line_data->means)) + vi_local;
  __global VoxelMean *mean_data = &line_data->means[vi];

  float3 voxel_mean = subVoxelToLocalCoord(mean_data->coord, voxel_resolution);
  // voxel_mean is currently relative to the voxel centre of the voxelKey voxel. We need to change it to be in the same reference
  // frame as the incoming rays, which is relative to the endKey voxel. For this we need to caculate the additional
  // displacement from the centre of endKey to the centre of voxelKey and add this displacement.

  // Calculate the number of voxel steps from endKey to the voxelKey
  const int3 voxel_diff = keyDiff(endKey, voxelKey, &line_data->region_dimensions);
  // Scale by voxel resolution and add to voxel_mean
  voxel_mean.x += voxel_diff.x * voxel_resolution;
  voxel_mean.y += voxel_diff.y * voxel_resolution;
  voxel_mean.z += voxel_diff.z * voxel_resolution;

  vi = (line_data->cov_offsets[line_data->current_region_index] / sizeof(*line_data->cov_voxels)) + vi_local;
  CovarianceVoxel cov_voxel;
  // Manual copy of the NDT voxel: we had some issues with OpenCL assignment on structures.
  cov_voxel.cov_sqrt_diag[0] = line_data->cov_voxels[vi].cov_sqrt_diag[0];
  cov_voxel.cov_sqrt_diag[1] = line_data->cov_voxels[vi].cov_sqrt_diag[1];
  cov_voxel.cov_sqrt_diag[2] = line_data->cov_voxels[vi].cov_sqrt_diag[2];
  cov_voxel.cov_sqrt_diag[3] = line_data->cov_voxels[vi].cov_sqrt_diag[3];
  cov_voxel.cov_sqrt_diag[4] = line_data->cov_voxels[vi].cov_sqrt_diag[4];
  cov_voxel.cov_sqrt_diag[5] = line_data->cov_voxels[vi].cov_sqrt_diag[5];

  float adjustment = 0;
  const int min_sample_threshold = 4;  // Should be passed in.
  const float3 voxel_maximum_likelyhood =
    calculateMissNdt(&cov_voxel, &adjustment, line_data->sensor, line_data->sample, voxel_mean, mean_data->count,
                     INFINITY, line_data->ray_adjustment, line_data->sensor_noise, min_sample_threshold);

  // NDT should do sample update in a separate process in order to update the covariance, so we should not get here.
  return (!isEndVoxel || line_data->region_update_flags & kRfEndPointAsFree) ? adjustment : 0;
}

#endif  // ADJUSTNDT_CL
