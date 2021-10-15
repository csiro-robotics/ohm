// Copyright (c) 2021
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef OHMGPU_COVARIANCE_HIT_HDT_H_
#define OHMGPU_COVARIANCE_HIT_HDT_H_

#include "CovarianceVoxelCompute.h"
// #include "VoxelMeanCompute.h"

// #include "Regions.cl"

typedef struct WorkItem_t
{
  CovarianceVoxel cov;
  // Voxel mean relative to the voxel centre.
  float3 mean;
  uint sample_count;
  float occupancy;
} WorkItem;

// forward declarations

void __device__ collateSample(WorkItem *work_item, float3 sensor, float3 sample, int3 region_dimensions,
                              float voxel_resolution, float sample_adjustment, float occupied_threshold,
                              float sensor_noise, float reinitialise_cov_threshold,
                              unsigned reinitialise_cov_sample_count);


void __device__ collateSample(WorkItem *work_item, float3 sensor, float3 sample, int3 region_dimensions,
                              float voxel_resolution, float sample_adjustment, float occupied_threshold,
                              float sensor_noise, float reinitialise_cov_threshold,
                              unsigned reinitialise_cov_sample_count)
{
  // The sample is currently relative to the voxel centre of the end voxel. This is the same reference frame we need
  // to calculate the quantised mean, so no change is required.
  if (calculateHitWithCovariance(&work_item->cov, &work_item->occupancy, sample, work_item->mean,
                                 work_item->sample_count, sample_adjustment, INFINITY, voxel_resolution,
                                 reinitialise_cov_threshold, reinitialise_cov_sample_count))
  {
    // Covariance matrix has reset. Reset the point count to clear the mean value.
    work_item->sample_count = 0;
  }

  const float one_on_count_plus_one = 1.0f / (float)(work_item->sample_count + 1);
  work_item->mean = (work_item->sample_count * work_item->mean + sample) * one_on_count_plus_one;
  ++work_item->sample_count;
}

#endif  // OHMGPU_COVARIANCE_HIT_HDT_H_
