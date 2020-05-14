
/// Update the voxel mean pattern at @p target_address by including the bit(s) from @p pattern_to_add.
/// This is done using atomic operations.
///
/// Each bit in the pattern indicates occupancy at a particular voxel mean location.
__device__ void updateVoxelMeanPosition(__global VoxelMean *voxel, float3 sample_pos, float voxel_resolution);

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// Psuedo header guard to prevent function implementation duplication.
__device__ void updateVoxelMeanPosition(__global VoxelMean *voxel, float3 sample_pos, float voxel_resolution)
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
    point_count = gputilAtomicLoadU32(&voxel->count);
    old_value = gputilAtomicLoadU32(&voxel->coord);
    new_value = subVoxelUpdate(old_value, point_count, sample_pos, voxel_resolution);
    ++iteration_count;
  } while (!gputilAtomicCasU32(&voxel->coord, old_value, new_value) && iteration_limit < iteration_count);

  // Atomic increment for the point count.
  gputilAtomicInc(&voxel->count);
}
