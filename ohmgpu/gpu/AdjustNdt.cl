// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef ADJUSTNDT_CL
#define ADJUSTNDT_CL

#include "CovarianceVoxelCompute.h"

inline __device__ float calculateOccupancyAdjustment(const GpuKey *voxel_key, const GpuKey *end_key, bool is_end_voxel,
                                                     bool is_sample_voxel, float voxel_resolution,
                                                     LineWalkData *line_data)
{
  // Note: we always ignore voxels where is_sample_voxel or is_end_voxel is true. Samples are adjusted later while
  // a non-sample is_end_voxel is a split ray.

  const ulonglong vi_local = voxel_key->voxel[0] + voxel_key->voxel[1] * line_data->region_dimensions.x +
                             voxel_key->voxel[2] * line_data->region_dimensions.x * line_data->region_dimensions.y;
  ulonglong vi = (line_data->means_offsets[line_data->current_region_index] / sizeof(*line_data->means)) + vi_local;
  __global VoxelMean *mean_data = &line_data->means[vi];

  const uint voxel_mean_coord = gputilAtomicLoadU32(&mean_data->coord);
  const uint voxel_mean_count = gputilAtomicLoadU32(&mean_data->count);
  float3 voxel_mean = subVoxelToLocalCoord(voxel_mean_coord, voxel_resolution);
  // voxel_mean is currently relative to the voxel centre of the voxel_key voxel. We need to change it to be in the same
  // reference frame as the incoming rays, which is relative to the endKey voxel. For this we need to calculate the
  // additional displacement from the centre of endKey to the centre of voxel_key and add this displacement.

  // Calculate the number of voxel steps from endKey to the voxel_key
  const int3 voxel_diff = keyDiff(voxel_key, end_key, line_data->region_dimensions);
  // Scale by voxel resolution and add to voxel_mean
  voxel_mean.x += voxel_diff.x * voxel_resolution;
  voxel_mean.y += voxel_diff.y * voxel_resolution;
  voxel_mean.z += voxel_diff.z * voxel_resolution;

  vi = (line_data->cov_offsets[line_data->current_region_index] / sizeof(*line_data->cov_voxels)) + vi_local;
  CovarianceVoxel cov_voxel;
  // Manual copy of the NDT voxel: we had some issues with OpenCL assignment on structures.
  cov_voxel.trianglar_covariance[0] = line_data->cov_voxels[vi].trianglar_covariance[0];
  cov_voxel.trianglar_covariance[1] = line_data->cov_voxels[vi].trianglar_covariance[1];
  cov_voxel.trianglar_covariance[2] = line_data->cov_voxels[vi].trianglar_covariance[2];
  cov_voxel.trianglar_covariance[3] = line_data->cov_voxels[vi].trianglar_covariance[3];
  cov_voxel.trianglar_covariance[4] = line_data->cov_voxels[vi].trianglar_covariance[4];
  cov_voxel.trianglar_covariance[5] = line_data->cov_voxels[vi].trianglar_covariance[5];

  float adjustment = 0;
  const int min_sample_threshold = 4;  // Should be passed in.

  bool is_miss;
  const float3 voxel_maximum_likelihood = calculateMissNdt(
    &cov_voxel, &adjustment, &is_miss, line_data->sensor, line_data->sample, voxel_mean, voxel_mean_count, INFINITY,
    line_data->ray_adjustment, line_data->adaptation_rate, line_data->sensor_noise, min_sample_threshold);

  // We increment the miss if needed.
  if (line_data->hit_miss && is_miss)
  {
    __global atomic_uint *miss_count_ptr =
      (__global atomic_uint *)&line_data
        ->hit_miss[(line_data->hit_miss_offsets[line_data->current_region_index] / sizeof(*line_data->hit_miss)) +
                   vi_local]
        .miss_count;
    gputilAtomicAdd(miss_count_ptr, 1u);
  }

  // NDT should do sample update in a separate process in order to update the covariance, so we should not get here.
  return (is_end_voxel || (is_sample_voxel && !(line_data->region_update_flags & kRfEndPointAsFree))) ? 0 : adjustment;
}

#endif  // ADJUSTNDT_CL
