// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "VoxelIncidentCompute.h"

/// Update the voxel mean pattern at @p target_address by including the bit(s) from @p pattern_to_add.
/// This is done using atomic operations.
///
/// Each bit in the pattern indicates occupancy at a particular voxel mean location.
/// @param voxel The @c VoxelMean to update.
/// @param incident_ray The incident ray from sensor to sample (un-normalised).
/// @param sample_count How many samples in the voxel *before* adding the current one.
__device__ void updateVoxelIncident(__global atomic_uint *voxel, float3 incident_ray, uint sample_count);

#ifndef VOXEL_INCIDENT_CL
#define VOXEL_INCIDENT_CL

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// Psuedo header guard to prevent function implementation duplication.
inline __device__ void updateVoxelIncident(__global atomic_uint *voxel, float3 incident_ray, uint sample_count)
{
  uint old_value;
  uint new_value;

  // Few iterations as it's less important to get this right.
  const int iteration_limit = 10;
  int iteration_count = 0;

  // First update the mean position, then the point count. There is a level of contension here, and the results may be
  // somewhat out.
  do
  {
    old_value = gputilAtomicLoadU32(voxel);
    new_value = updateIncidentNormal(old_value, incident_ray, sample_count);
    ++iteration_count;
  } while (!gputilAtomicCasU32(voxel, old_value, new_value) && iteration_limit < iteration_count);
}

#endif  // VOXEL_INCIDENT_CL
