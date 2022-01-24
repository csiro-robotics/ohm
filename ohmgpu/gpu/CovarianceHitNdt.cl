// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "gpu_ext.h"  // Must be first

#include "CovarianceVoxelCompute.h"
#include "GpuKey.h"
#include "MapCoord.h"
#include "VoxelMeanCompute.h"

#include "Regions.cl"

#include "CovarianceHitNdt_h.cl"
#include "Traversal.cl"
#include "VoxelIncident.cl"

/// This kernel integrates the ray sample points only into the map and is executed one thread per sample.
///
/// This kernel works with the @c REGION_UPDATE_KERNEL compiled in NDT mode. That kernel adjusts only free space
/// skipping the sample voxels. This kernel is invoked afterwards in order to calculate the sample adjustments. This
/// is because the sample adjustment is unable to use atomic CAS semantics when updating the covariance. The six values
/// of the reduced covariance matrix - see @c CovarianceVoxel - cannot be updated atomically.
///
/// This performes the covariance update on @c CovarianceVoxel for each affected sample voxel. The kernel requires that
/// the @c line_keys and corresponding @c local_lines are grouped (or sorted) such that all samples affecting a
/// particular sample voxel appear in a contiguous range in the array - referred below as a "sample block".
///
/// We run one GPU thread per sample entry in @c line_keys and @c local_lines . Each thread starts by checking if it is
/// the first thread in a sample block. Only the first thread in the sample block continues execution with all other
/// threads returning. The remaining threads iterate over their sample block and perform the relevant updates to
/// @c CovarianceVoxel and @c VoxelMean before writing the value back to main memory.
///
/// While this results in many inactive threads, it has proven more efficient than the following other attempted
/// techniques:
/// - Performing NDT sample update on GPU resulted in too much memory synchronisation overhead between CPU/GPU
/// - Unordered GPU update with each thread iterating the entire sample range looking for relevant samples - only first
///   thread did the writing
///
/// @note Memory layout for voxel data - @p occupancy , @p means and @p cov_voxels - is the same as for the
/// @c REGION_UPDATE_KERNEL .
///
/// @param occupancy The GPU cached voxel occupancy block.
/// @param occupancy_region_mem_offsets_global Array of voxel region memory offsets into @p occupancy . Each element
///     corresponds to a key in occupancy_region_keys_global. The offsets are in bytes.
/// @param means The GPU cached @c VoxelMean data block.
/// @param means_region_mem_offsets_global Array of voxel region memory offsets into @p means . Each element
///     corresponds to a key in occupancy_region_keys_global. The offsets are in bytes.
/// @param cov_voxels The GPU cached @c CovarianceVoxel data block.
/// @param cov_region_mem_offsets_global Array of voxel region memory offsets into @p cov_voxels . Each element
///     corresponds to a key in occupancy_region_keys_global. The offsets are in bytes.
/// @param occupancy_region_keys_global Array of voxel region keys identifying regions available in GPU. There are
///     @c region_count elements in this array.
/// @param region_count Number of regions uploaded in GPU and addressable in @p occupancy_region_keys_global .
/// @param line_keys Array of origin/sample pairs converted into @c GpuKey references to integrate into the map.
///     Must be ordered such that all samples of the same value appear in a contiguous block.
/// @param local_lines Array array of origin/sample pairs which generated the @c line_keys. These are converted from the
///     original, double precision, map frame coordinates into a set of local frames. Each start/end point pair is
///     relative to the centre of the voxel containing the end point. This is to reduce floating point error in
///     double to single precision conversion and assist in voxel mean calculations which are in the same frame.
///     The original coordinates are not recoverable in this code.
/// @param line_count number of lines in @p line_keys and @p local_lines. These come in pairs, so the number of elements
///     in those arrays is double this value.
/// @param region_dimensions Specifies the size of any one region in voxels.
/// @param voxel_resolution Specifies the size of a voxel cube.
/// @param sample_adjustment Specifiest the value adjustment applied to voxels containing the sample point (line end
///     point). Should be > 0 to re-enforce as occupied.
/// @param occupied_threashold Voxel @p occupancy value at which point the voxel is considered occupied. Must be >= 0 .
///     Normally 0.
/// @param voxel_value_max Maximum clamping value for voxel adjustments.
/// @param sensor_noise Expected range sensor noise value in the same units as the line values (generally metres).
///     Used as part of the NDT model. Must be > 0
/// @param reinitialise_cov_threshold The occupancy threshold value at which the covariance matrix may be reinitialised.
///     See @c calculateHitWithCovariance()
/// @param reinitialise_cov_sample_count The point count required to allow @p reinitialise_cov_threshold to be
///     triggered. See @c calculateHitWithCovariance()
__kernel void covarianceHitNdt(
  __global atomic_float *occupancy, __global ulonglong *occupancy_region_mem_offsets_global,             //
  __global VoxelMean *means, __global ulonglong *means_region_mem_offsets_global,                        //
  __global CovarianceVoxel *cov_voxels, __global ulonglong *cov_region_mem_offsets_global,               //
  __global IntensityMeanCov *intensity_voxels, __global ulonglong *intensity_region_mem_offsets_global,  //
  __global HitMissCount *hit_miss_voxels, __global ulonglong *hit_miss_region_mem_offsets_global,        //
  __global atomic_float *traversal_voxels, __global ulonglong *traversal_region_mem_offsets_global,      //
  __global atomic_uint *touch_time_voxels, __global ulonglong *touch_times_region_mem_offsets_global,    //
  __global atomic_uint *incident_voxels, __global ulonglong *incidents_region_mem_offsets_global,        //
  __global int3 *occupancy_region_keys_global, uint region_count,                                        //
  __global GpuKey *line_keys, __global float3 *local_lines, uint line_count,                             //
  __global uint *touch_times, __global float *intensities,                                               //
  int3 region_dimensions, float voxel_resolution, float sample_adjustment, float occupied_threshold,
  float voxel_value_max, float initial_intensity_covariance, unsigned ndt_sample_threshold, float adaptation_rate,
  float sensor_noise, float reinitialise_cov_threshold, unsigned reinitialise_cov_sample_count)
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
  uint intensity_index, hit_miss_index;

  // Resolve the read/write indices for the target voxel. We need indices into occupancy, means and cov_voxels.
  // Dummy arguments for regionsResolveRegion(). We will only perform one lookup for each data type.
  int3 dummy_region_key;
  uint region_index;

  regionsInitCurrent(&dummy_region_key, &region_index);
  if (!regionsResolveRegion(&start_voxel, &dummy_region_key, &region_index, occupancy_region_keys_global, region_count))
  {
    // Data not available in GPU memory.
    return;
  }

  occupancy_index = (uint)(region_local_index + occupancy_region_mem_offsets_global[region_index] / sizeof(*occupancy));
  mean_index = (uint)(region_local_index + means_region_mem_offsets_global[region_index] / sizeof(*means));
  cov_index = (uint)(region_local_index + cov_region_mem_offsets_global[region_index] / sizeof(*cov_voxels));
  intensity_index =
    (intensity_voxels) ?
      (uint)(region_local_index + intensity_region_mem_offsets_global[region_index] / sizeof(*intensity_voxels)) :
      0;
  hit_miss_index =
    (hit_miss_voxels) ?
      (uint)(region_local_index + hit_miss_region_mem_offsets_global[region_index] / sizeof(*hit_miss_voxels)) :
      0;
  uint traversal_index =
    (traversal_voxels) ?
      (uint)(region_local_index + traversal_region_mem_offsets_global[region_index] / sizeof(*traversal_voxels)) :
      0;
  float traversal = (traversal_voxels) ? gputilAtomicLoadF32(&traversal_voxels[traversal_index]) : 0.0f;
  uint touch_time_index =
    (touch_time_voxels) ?
      (uint)(region_local_index + touch_times_region_mem_offsets_global[region_index] / sizeof(*touch_time_voxels)) :
      0;
  uint touch_time = (touch_time_voxels) ? gputilAtomicLoadU32(&touch_time_voxels[touch_time_index]) : 0u;
  uint incident_index =
    (incident_voxels) ?
      (uint)(region_local_index + incidents_region_mem_offsets_global[region_index] / sizeof(*incident_voxels)) :
      0;
  uint incident = (incident_voxels) ? gputilAtomicLoadU32(&incident_voxels[incident_index]) : 0u;

  // Cache initial values.
  WorkItem work_item;
  work_item.occupancy = gputilAtomicLoadF32(&occupancy[occupancy_index]);
  work_item.mean = subVoxelToLocalCoord(gputilAtomicLoadU32(&means[mean_index].coord), voxel_resolution);
  work_item.sample_count = gputilAtomicLoadU32(&means[mean_index].count);

  // Manual copy of the NDT voxel: we had some issues with OpenCL assignment on structures.
  work_item.cov.trianglar_covariance[0] = cov_voxels[cov_index].trianglar_covariance[0];
  work_item.cov.trianglar_covariance[1] = cov_voxels[cov_index].trianglar_covariance[1];
  work_item.cov.trianglar_covariance[2] = cov_voxels[cov_index].trianglar_covariance[2];
  work_item.cov.trianglar_covariance[3] = cov_voxels[cov_index].trianglar_covariance[3];
  work_item.cov.trianglar_covariance[4] = cov_voxels[cov_index].trianglar_covariance[4];
  work_item.cov.trianglar_covariance[5] = cov_voxels[cov_index].trianglar_covariance[5];

  IntensityMeanCov intensity_cov;  // = intensity_voxels[intensity_index];
  HitMissCount hit_miss_count;     // = hit_miss_voxels[hit_miss_index];

  if (intensity_voxels)
  {
    intensity_cov.intensity_mean = intensity_voxels[intensity_index].intensity_mean;
    intensity_cov.intensity_cov = intensity_voxels[intensity_index].intensity_cov;
  }

  if (hit_miss_voxels)
  {
    hit_miss_count.hit_count = hit_miss_voxels[hit_miss_index].hit_count;
    hit_miss_count.miss_count = hit_miss_voxels[hit_miss_index].miss_count;
  }

  // Now update by iterating from the starting voxel until we change voxels or reach the end of the set.
  uint added = 0;
  for (uint i = get_global_id(0); i < line_count; ++i)
  {
    copyKey(&target_voxel, &line_keys[i * 2 + 1]);
    if (equalKeys(&target_voxel, &start_voxel))
    {
      // Now we consider rays which have been clipped. These will have Ignore voxels with voxel[3] != 0.
      // This indicates the end point is a truncated part of the ray and not a real sample.
      if (target_voxel.voxel[3] == 0)
      {
        if (hit_miss_voxels)
        {
          const bool reinitialise_permeability_with_covariance = true;  // TODO: make a parameter of map
          calculateHitMissUpdateOnHit(&work_item.cov, work_item.occupancy, &hit_miss_count, local_lines[i * 2],
                                      local_lines[i * 2 + 1], work_item.mean, work_item.sample_count, INFINITY,
                                      reinitialise_permeability_with_covariance, adaptation_rate, sensor_noise,
                                      reinitialise_cov_threshold, reinitialise_cov_sample_count, ndt_sample_threshold);
        }
        if (intensity_voxels)
        {
          calculateIntensityUpdateOnHit(&intensity_cov, work_item.occupancy, intensities[i],
                                        initial_intensity_covariance, work_item.sample_count,
                                        reinitialise_cov_threshold, reinitialise_cov_sample_count);
        }
        if (traversal_voxels)
        {
          traversal += calculateTraversal(local_lines[i * 2], local_lines[i * 2 + 1], voxel_resolution);
        }

        if (touch_times)
        {
          touch_time = max(touch_time, touch_times[i]);
        }

        if (incident_voxels)
        {
          // The sample count won't be increment for the current point yet. That's the behaviour we want for the
          // progressive average.
          incident =
            updateIncidentNormal(incident, local_lines[i * 2] - local_lines[i * 2 + 1], work_item.sample_count);
        }

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
  gputilAtomicStoreF32(&occupancy[occupancy_index], work_item.occupancy);
  gputilAtomicStoreU32(&means[mean_index].coord, subVoxelCoord(work_item.mean, voxel_resolution));
  gputilAtomicStoreU32(&means[mean_index].count, work_item.sample_count);
  cov_voxels[cov_index] = work_item.cov;
  if (intensity_voxels)
  {
    intensity_voxels[intensity_index] = intensity_cov;
  }
  if (hit_miss_voxels)
  {
    hit_miss_voxels[hit_miss_index] = hit_miss_count;
  }
  if (traversal_voxels)
  {
    gputilAtomicStoreF32(&traversal_voxels[traversal_index], traversal);
  }
  if (touch_time_voxels)
  {
    gputilAtomicStoreU32(&touch_time_voxels[touch_time_index], touch_time);
  }
  if (incident_voxels)
  {
    gputilAtomicStoreU32(&incident_voxels[incident_index], incident);
  }
}
