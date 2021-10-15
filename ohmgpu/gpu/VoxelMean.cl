// Copyright (c) 2020
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

/// Update the voxel mean pattern at @p target_address by including the bit(s) from @p pattern_to_add.
/// This is done using atomic operations.
///
/// Each bit in the pattern indicates occupancy at a particular voxel mean location.
/// @param voxel The @c VoxelMean to update.
/// @param sample_pos The sample position local to the centre of the voxel in falls in.
/// @param voxel_resolution Voxel size.
__device__ uint updateVoxelMeanPosition(__global VoxelMean *voxel, float3 sample_pos, float voxel_resolution);

#ifndef VOXEL_MEAN_CL
#define VOXEL_MEAN_CL

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// Psuedo header guard to prevent function implementation duplication.
inline __device__ uint updateVoxelMeanPosition(__global VoxelMean *voxel, float3 sample_pos, float voxel_resolution)
{
  uint point_count;
  uint old_value;
  uint new_value;

  // Few iterations as it's less important to get this right.
  const int iteration_limit = 10;
  int iteration_count = 0;

  // First update the mean position, then the point count. There is a level of contension here, and the results may be
  // somewhat out.
  do
  {
    // point_count = gputilAtomicLoadU32(&voxel->count);
    point_count = gputilAtomicLoadU32(&voxel->count);
    old_value = gputilAtomicLoadU32(&voxel->coord);
    new_value = subVoxelUpdate(old_value, point_count, sample_pos, voxel_resolution);
    ++iteration_count;
  } while (!gputilAtomicCasU32(&voxel->coord, old_value, new_value) && iteration_limit < iteration_count);

  // Atomic increment for the point count.
  return gputilAtomicInc(&voxel->count);
}

#endif  // VOXEL_MEAN_CL
