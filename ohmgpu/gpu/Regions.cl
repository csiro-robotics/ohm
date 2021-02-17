// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "GpuKey.h"

/// Initialises the @c currentRegion and @c regionIndex parameters for @c regionsResolveRegion().
__device__ void regionsInitCurrent(int3 *currentRegion, uint *regionIndex);

/// A utility function for resolving the correct memory address index for voxels of the region containing @p voxelKey.
///
/// This function supports the @c GpuCache and @c GpuLayerCache code on CPU. The cache structure is such that voxel
/// data for regions are uploaded into a single memory buffer with the cache tracking memory offsets for each uploaded
/// region into the voxel buffer. On GPU we require three buffers in order to resolve voxels for a specific region:
///
/// 1. The GPU cache voxel buffer, which contains the voxel data.
/// 2. @p voxelKey : an array of @c GpuKey items which is used to resolve corresponding memory offsets into the voxel
///   buffer.
/// 3. An array of memory offsets into the voxel buffer which dirrectly correspond to the @c GpuKey array.
///
/// With this information we can lookup the voxel memory for a region as follows:
/// 1. Traverse the @p voxelKey array searching for the key of interest.
/// 2. If a match is found, lookup the memory offset at the same index. This gives the memory offset into the voxel
///   memory buffer as a byte offset.
/// 3. Lookup the voxel memory buffer with the offset from 2.
///
/// This function supports resolving the index into the @p voxelKey array. This index may then be used for steps 2 and
/// 3 which are done in the calling code.
///
/// There an expectation that multiple subsequent region lookups will be required as we walk a line of voxels. Further,
/// it is expected that voxels from the same region will generally be required. For this reason, this function first
/// checks @c currentRegion to see if it matches @c voxelKey and we can early out. The @c currentRegion is maintained
/// by this function, updated as required.
///
/// Before the first call, @p currentRegion should be initialised by calling @c regionsInitCurrent().
///
/// @param voxelKey The voxel for which we want to find the region data memory offset.
/// @param[in,out] currentRegion On enter, the current/last region reference. On successful exit, the region containing
///     @p voxelKey.
/// @param[in,out] regionIndex On enter, the current/last region voxel index offset. On successful exit, the
///     voxel index offset to the region containing @p voxelKey.
/// @param regionKeys Array of region keys associated with @p regionMemOffsets.
/// @param regionCount The number of regions in @p regionKeys and @p regionMemOffsets.
/// @return True if a region for @p voxelKey is found, false otherwise.
__device__ bool regionsResolveRegion(const GpuKey *voxelKey, int3 *currentRegion, uint *regionIndex,
                                     __global int3 *regionKeys, uint regionCount);

#ifndef REGIONS_CL
#define REGIONS_CL
inline __device__ void regionsInitCurrent(int3 *currentRegion, uint *regionIndex)
{
  currentRegion->x = currentRegion->y = currentRegion->z = 2147483647;
  *regionIndex = 0u;
}

inline __device__ bool regionsResolveRegion(const GpuKey *voxelKey, int3 *currentRegion, uint *regionIndex,
                                            __global int3 *regionKeys, uint regionCount)
{
  // Check if the current region is the same as the last. This will generally be the case.
  if (voxelKey->region[0] == currentRegion->x && voxelKey->region[1] == currentRegion->y &&
      voxelKey->region[2] == currentRegion->z)
  {
    // Same region. No update required.
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
      *regionIndex = i;  // regionMemOffsets[i] / voxelSizeBytes;
      // printf("%u found: [ %i %i %i ] @ %u\n", get_global_id(0),
      //        regionKeys[i].x, regionKeys[i].y, regionKeys[i].z,
      //        i);
      // if (*regionIndex * voxelSizeBytes > 1073741824ul)
      // {
      //   printf("%u Bad region address %u\n", get_global_id(0), *regionIndex);
      // }
      return true;
    }
    // printf("%u not: [ %i %i %i ]\n", get_global_id(0), regionKeys[i].x, regionKeys[i].y, regionKeys[i].z);
  }

  return false;
}

// inline __device__ uint regionVoxelIndex(uint regionIndex, __global ulonglong *regionMemOffsets, uint voxelSizeBytes)
// {
//   return regionMemOffsets[regionIndex] / voxelSizeBytes;
// }

#endif  // REGIONS_CL
