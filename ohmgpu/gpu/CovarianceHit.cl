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
  GpuKey start_voxel;
  // BUG: Intel OpenCL 2.0 compiler does not effect an assignment of GpuKey. I've had to unrolled it in copyKey().
  copyKey(&start_voxel, &line_keys[get_global_id(0) * 2 + 1]);
  start_voxel.voxel[3] = 0;  // For now we can ignore clipped sample voxels. Will check during iteration below.

  // We assume a sorted set of input points, where sample points falling in the same voxel are grouped together.
  // We then only allow the first thread in each voxel group to do the update for that group.
  // We check this now.
  GpuKey target_voxel;
  // Initialise with the target voxel index to ensure it has a value.
  copyKey(&target_voxel, &start_voxel);
  // Now fetch the previous key if we can. Only can't for global thread 0.
  if (get_global_id(0) > 0)
  {
    // Note the -1 to get the previous sample voxel.
    // copyKey(&target_voxel, &line_keys[get_global_id(0) * 2 - 1]);
    copyKey(&target_voxel, &line_keys[(get_global_id(0) - 1) * 2 + 1]);
  }
  target_voxel.voxel[3] = 0;  // Again - don't care about clipping yet.

  if (equalKeys(&target_voxel, &start_voxel) && get_global_id(0) > 0)
  {
    // This is not the first thread for this voxel grouping. Abort. The first thread in the group will do the update.
    // While this results in many idle threads, it maximise throughput while minimising iteration.
    return;
  }

  const uint region_local_index = start_voxel.voxel[0] + start_voxel.voxel[1] * region_dimensions.x +
                                  start_voxel.voxel[2] * region_dimensions.x * region_dimensions.y;
  uint occupancy_index, mean_index, cov_index;

  // Resolve the read/write indices for the target voxel. We need indices into occupancy, means and cov_voxels.
  // Dummy arguments for regionsResolveRegion(). We will only perform one lookup for each data type.
  int3 dummy_region_key;
  uint region_index;

  regionsInitCurrent(&dummy_region_key, &region_index);
  if (!regionsResolveRegion(&start_voxel, &dummy_region_key, &region_index, occupancy_region_keys_global,
                            region_count))
  {
    // Data not available in GPU memory.
    return;
  }

  occupancy_index = (uint)(region_local_index + occupancy_region_mem_offsets_global[region_index] / sizeof(*occupancy));
  mean_index = (uint)(region_local_index + means_region_mem_offsets_global[region_index] / sizeof(*means));
  cov_index = (uint)(region_local_index + cov_region_mem_offsets_global[region_index] / sizeof(*cov_voxels));

  // Cache initial values.
  WorkItem_t work_item;
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

  // Now update by iterating from the starting voxel until we change voxels or reach the end of the set.
  uint added = 0;
  for (uint i = get_global_id(0); i < line_count; ++i)
  {
    copyKey(&target_voxel, &line_keys[i * 2 + 1]);
    if (equalKeys(&target_voxel, &start_voxel))
    {
      // Now we consider rays which have been clipped. These will have Ignore voxels with voxel[3] != 0.
      // This indicates the end point is a trancated part of the ray and not a real sample.
      if (target_voxel.voxel[3] == 0)
      {
        // Still within the starting voxel.
        collateSample(&work_item, local_lines[i * 2], local_lines[i * 2 + 1], region_dimensions, voxel_resolution,
                      sample_adjustment, occupied_threshold, sensor_noise, reinitialise_cov_threshold,
                      reinitialise_cov_sample_count);
        ++added;
      }
    }
    else
    {
      // Change in voxel. Done collecting.
      break;
    }
  }

  // Cap occupancy to max.
  if (voxel_value_max > 0)
  {
    work_item.occupancy = min(work_item.occupancy, voxel_value_max);
  }

  // if (get_global_id(0) == 0)
  // {
  //   printf("added: %u\n", added);
  // }

  // Write results. We expect no contension at this point so we write results directly. No atomic operations.
  occupancy[occupancy_index] = work_item.occupancy;
  means[mean_index].coord = subVoxelCoord(work_item.mean, voxel_resolution);
  means[mean_index].count = work_item.sample_count;
  cov_voxels[cov_index] = work_item.cov;
}
