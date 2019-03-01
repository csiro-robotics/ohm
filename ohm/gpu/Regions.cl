#include "GpuKey.h"

/// Initialises the @c currentRegion and @c regionVoxelOffset parameters for @c regionsResolveRegion().
///
__device__ void regionsInitCurrent(int3 *currentRegion, uint *regionVoxelOffset);

/// A utility function for resolving the correct memory offset to the voxels for the region containing @p voxelKey.
///
/// This function expect region voxel memory to be layed out in a contiguous memory buffer, where each voxel is
/// represented by a data of size @p voxelSizeBytes. The voxels for specific regions are uploaded with memory offsets
/// into the buffer. Information about the regions is stored two buffers: an int3 buffer containing the region keys (@p
/// regionKeys), and a memory offset buffer (@p regionMemOffets) identifying the corresponding byte offsets into the
/// global voxels buffer.
///
/// The function updated @p currentRegion and @p regionVoxelOffset to the region containing @p voxelKey if possible.
/// Note that the @p regionVoxelOffset is an index offset assuming @p voxelSizeBytes striding, rather than a byte
/// offset.
///
/// The function will early out (with associated GPU thread group issues) if the @p currentRegion already references
/// the region containing @p voxelKey. The expectation is that the same region will be generally referenced repeatedly
/// due to spatial coherence, thus this will be the most common code path and generally avoid GPU threads choosing
/// different branches.
///
/// The function may fail to resolve the region in which case @p currentRegion and @p regionVoxelOffset are left as is
/// and the function returns false.
///
/// Before the first call, @p currentRegion should be initialised by calling @c regionsInitCurrent().
///
/// @param voxelKey The voxel for which we want to find the region data memory offset.
/// @param[in,out] currentRegion On enter, the current/last region reference. On successful exit, the region containing
///     @p voxelKey.
/// @param[in,out] regionVoxelOffset On enter, the current/last region voxel index offset. On successful exit, the
///     voxel index offset to the region containing @p voxelKey.
/// @param regionKeys Array of region keys associated with @p regionMemOffsets.
/// @param regionMemOffsets Byte offsets into a region memory buffer for each region identified in @p regionKeys.
/// @param regionCount The number of regions in @p regionKeys and @p regionMemOffsets.
/// @return True if a region for @p voxelKey is found, false otherwise.
__device__ bool regionsResolveRegion(const struct GpuKey *voxelKey, int3 *currentRegion, uint *regionVoxelOffset,
                                     __global int3 *regionKeys, __global ulong *regionMemOffsets, uint regionCount,
                                     unsigned voxelSizeBytes);

#ifndef REGIONS_CL
#define REGIONS_CL
__device__ void regionsInitCurrent(int3 *currentRegion, uint *regionVoxelOffset)
{
  currentRegion->x = currentRegion->y = currentRegion->z = 2147483647;
  *regionVoxelOffset = 0u;
}

__device__ bool regionsResolveRegion(const struct GpuKey *voxelKey, int3 *currentRegion, uint *regionVoxelOffset,
                                     __global int3 *regionKeys, __global ulong *regionMemOffsets, uint regionCount,
                                     unsigned voxelSizeBytes)
{
  // Check if the current region is the same as the last. This will generally be the case.
  if (voxelKey->region[0] == currentRegion->x && voxelKey->region[1] == currentRegion->y &&
      voxelKey->region[2] == currentRegion->z)
  {
    // Same region.
    return true;
  }

  // printf("%u searching: " KEY_F "\n", get_global_id(0), KEY_A(*voxelKey));
  // Need to search for the region.
  for (uint i = 0; i < regionCount; ++i)
  {
    if (voxelKey->region[0] == regionKeys[i].x && voxelKey->region[1] == regionKeys[i].y &&
        voxelKey->region[2] == regionKeys[i].z)
    {
      // Found the region for voxelKey.
      *currentRegion = regionKeys[i];
      // Voxel offset is an index offset.
      *regionVoxelOffset = regionMemOffsets[i] / voxelSizeBytes;
      // printf("%u found: [ %i %i %i ] @ %u\n", get_global_id(0),
      //        regionKeys[i].x, regionKeys[i].y, regionKeys[i].z,
      //        i);
      if (*regionVoxelOffset * voxelSizeBytes > 1073741824ul)
      {
        printf("%u Bad region address %u\n", get_global_id(0), *regionVoxelOffset);
      }
      return true;
    }
    // printf("%u not: [ %i %i %i ]\n", get_global_id(0), regionKeys[i].x, regionKeys[i].y, regionKeys[i].z);
  }

  return false;
}

#endif  // REGIONS_CL

