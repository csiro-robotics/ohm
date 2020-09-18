// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpu_ext.h"

#include "GpuKey.h"
#include "CovarianceVoxelCompute.h"
#include "MapCoord.h"
#include "VoxelMeanCompute.h"

#include "Regions.cl"

/// Value controlling how many sample each thread can buffer before forcing processing of the buffered data
#define WORKING_RAY_COUNT 128

typedef struct WorkItem_t
{
  CovarianceVoxel cov;
  // Voxel mean relative to the voxel centre.
  float3 mean;
  uint sample_count;
  float occupancy;
} WorkItem;

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

// Things to document:
// - Invocation is one thread per sample (subject to change)
// - Each thread will handle one sample voxel
//  - Contension avoided by having only the first thread targetting a particular voxel allowed to write resutls.
__kernel void covarianceHit(__global atomic_float *occupancy, __global ulonglong *occupancy_region_mem_offsets_global,
                            __global VoxelMean *means, __global ulonglong *means_region_mem_offsets_global,
                            __global CovarianceVoxel *cov_voxels, __global ulonglong *cov_region_mem_offsets_global,
                            __global int3 *occupancy_region_keys_global, uint region_count, __global GpuKey *line_keys,
                            __global float3 *local_lines, uint line_count, int3 region_dimensions,
                            float voxel_resolution, float sample_adjustment, float occupied_threshold,
                            float voxel_value_max, float sensor_noise, float reinitialise_cov_threshold,
                            unsigned reinitialise_cov_sample_count)
{
  if (get_global_id(0) >= line_count)
  {
    return;
  }

  // Get the voxel for this thread. Remember, line_keys contains sensor/sample voxel pairs. Sample is second.
  GpuKey target_voxel;
  // BUG: Intel OpenCL 2.0 compiler does not effect an assignment of GpuKey. I've had to unrolled it in copyKey().
  copyKey(&target_voxel, &line_keys[get_global_id(0) * 2 + 1]);

  // We ignore this voxel if target_voxel.voxel[3] is set to 1. This indicates a clipped sample ray and the end voxel
  // does not actually represent a real sample.
  if (target_voxel.voxel[3] != 0)
  {
    return;
  }

  const uint region_local_index = target_voxel.voxel[0] + target_voxel.voxel[1] * region_dimensions.x +
                                  target_voxel.voxel[2] * region_dimensions.x * region_dimensions.y;
  uint occupancy_index, mean_index, cov_index;

  // Resolve the read/write indices for the target voxel. We need indices into occupancy, means and cov_voxels.
  // Dummy arguments for regionsResolveRegion(). We will only perform one lookup for each data type.
  int3 dummy_region_key;
  uint region_index;

  regionsInitCurrent(&dummy_region_key, &region_index);
  if (!regionsResolveRegion(&target_voxel, &dummy_region_key, &region_index, occupancy_region_keys_global,
                            region_count))
  {
    // Data not available in GPU memory.
    return;
  }

  occupancy_index = (uint)(region_local_index + occupancy_region_mem_offsets_global[region_index] / sizeof(*occupancy));
  mean_index = (uint)(region_local_index + means_region_mem_offsets_global[region_index] / sizeof(*means));
  cov_index = (uint)(region_local_index + cov_region_mem_offsets_global[region_index] / sizeof(*cov_voxels));

  // Cache the occupancy value.
  WorkItem work_item;
  work_item.occupancy = occupancy[occupancy_index];
  work_item.mean = subVoxelToLocalCoord(means[mean_index].coord, voxel_resolution);
  work_item.sample_count = means[mean_index].count;
  // Manual copy of the NDT voxel: we had some issues with OpenCL assignment on structures.
  work_item.cov.trianglar_covariance[0] = cov_voxels[cov_index].trianglar_covariance[0];
  work_item.cov.trianglar_covariance[1] = cov_voxels[cov_index].trianglar_covariance[1];
  work_item.cov.trianglar_covariance[2] = cov_voxels[cov_index].trianglar_covariance[2];
  work_item.cov.trianglar_covariance[3] = cov_voxels[cov_index].trianglar_covariance[3];
  work_item.cov.trianglar_covariance[4] = cov_voxels[cov_index].trianglar_covariance[4];
  work_item.cov.trianglar_covariance[5] = cov_voxels[cov_index].trianglar_covariance[5];

  uint working_rays[WORKING_RAY_COUNT];
  GpuKey end_key;
  uint collected_count = 0;
  bool is_first = true;
  bool allowed_write = false;

  // Now we iterate all the samples and collate items mathcing the target_voxel. There may be multiple samples in the
  // same target voxel which different threads are referencing. That is multiple threads will calculate the same
  // results. However, only the first thread (lowest global id) is allowed to write the result. This is somewhat
  // inefficient, as multiple threads to the same work, but it avoids the contension issue and avoids sorting in CPU.
  for (uint i = 0; i < line_count; ++i)
  {
    copyKey(&end_key, &line_keys[i * 2 + 1]);
    if (equalKeys(&target_voxel, &end_key))
    {
      if (collected_count == WORKING_RAY_COUNT)
      {
        if (allowed_write)
        {
          // We have collected too many items to process and must process them now. This is inefficient as we will have
          // very few threads doing this work at the same time.
          collateSample(&work_item, local_lines[i * 2], local_lines[i * 2 + 1], region_dimensions, voxel_resolution,
                        sample_adjustment, occupied_threshold, sensor_noise, reinitialise_cov_threshold,
                        reinitialise_cov_sample_count);
        }
      }
      else
      {
        // Relevant voxel. Add to the working indices.
        working_rays[collected_count++] = i;
        // Allowed to write if this is the first item collected and the global id matches this one.
        allowed_write = allowed_write || (is_first && i == get_global_id(0));
        is_first = false;
      }
    }
  }

  // Process results.
  if (allowed_write)
  {
    for (uint i = 0; i < collected_count; ++i)
    {
      uint li = working_rays[i];
      collateSample(&work_item, local_lines[li * 2], local_lines[li * 2 + 1], region_dimensions, voxel_resolution,
                    sample_adjustment, occupied_threshold, sensor_noise, reinitialise_cov_threshold,
                    reinitialise_cov_sample_count);
    }

    // Cap occupancy to max.
    if (voxel_value_max > 0)
    {
      work_item.occupancy = min(work_item.occupancy, voxel_value_max);
    }

    // Write results. We expect no contension at this point so we write results directly. No atomic operations.
    occupancy[occupancy_index] = work_item.occupancy;
    means[mean_index].coord = subVoxelCoord(work_item.mean, voxel_resolution);
    means[mean_index].count = work_item.sample_count;
    cov_voxels[cov_index] = work_item.cov;
  }
}
