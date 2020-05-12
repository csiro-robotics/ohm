
/// Update the sub-voxel pattern at @p target_address by including the bit(s) from @p pattern_to_add.
/// This is done using atomic operations.
///
/// Each bit in the pattern indicates occupancy at a particular sub-voxel location.
__device__ void updateSubVoxelPosition(__global atomic_uint *target_address, float3 sub_voxel_pos,
                                       float voxel_resolution, float sub_voxel_weigthing);

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// Psuedo header guard to prevent function implementation duplication.
__device__ void updateSubVoxelPosition(__global atomic_uint *target_address, float3 sub_voxel_pos,
                                       float voxel_resolution, float sub_voxel_weigthing)
{
  uint old_value;
  uint new_value;

  // Few iterations as it's less important to get this right.
  const int iteration_limit = 10;
  int iteration_count = 0;
  do
  {
    old_value = gputilAtomicLoadU32(target_address);
    new_value = subVoxelUpdate(old_value, sub_voxel_pos, voxel_resolution, sub_voxel_weigthing);
    ++iteration_count;
  } while (!gputilAtomicCasU32(target_address, old_value, new_value) && iteration_limit < iteration_count);
}
