//------------------------------------------------------------------------------
// Note on building this code.
// This code is intended to be used for updating regions both with and without
// sub voxel positioning. This is mostly the same, but the voxel type changes
// and we need an extract function call when updating a voxel with sub voxel
// positioning. To achive this, this code is intended to be includes into a
// compiled twice with the following defines changed on each compilation:
// - REGION_UPDATE_KERNEL : the kernel name for the entry point
// - REGION_UPDATE_SUFFIX : suffix applied to distinguish potentially repeaded
//  code and types.
// - SUB_VOX : defined if compiling for sub-voxels
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Debug switches for the compiling code to enable (or just uncomment here).
// Deliberately before incldues to configure those files.
//------------------------------------------------------------------------------

// Report regions we can't resolve via printf().
//#define REPORT_MISSING_REGIONS

// Limit the number of cells we can traverse in the line traversal. This is a worst case limit.
//#define LIMIT_LINE_WALK_ITERATIONS
// Limit the number of times we try update a voxel value. Probably best to always have this enabled.
#ifndef LIMIT_VOXEL_WRITE_ITERATIONS
#define LIMIT_VOXEL_WRITE_ITERATIONS
#endif  // LIMIT_VOXEL_WRITE_ITERATIONS

#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
// Store additional debug information in LineWalkData for error reporting.
//#define STORE_DEBUG_INFO
#endif  // LIMIT_VOXEL_WRITE_ITERATIONS

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "gpu_ext.h"

// Explicitly include MapCoord.h first. It's included from each of the subsequent includes, but leaving it to SubVoxel.h
// has issues with the resource generation. Essentially it causes MapCoord.h to be only included within the SUB_VOXEL
// define.
#include "ohm/MapCoord.h"
#include "ohm/SubVoxel.h"

#include "Regions.cl"

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------

#ifndef REGION_UPDATE_BASE_CL
#if defined(__CUDACC__)
using OccupancyType = atomic_float;
using SubVoxelPatternType = atomic_uint;
#else   // defined(__CUDACC__)
typedef atomic_float OccupancyType;
typedef atomic_uint SubVoxelPatternType;
#endif  // defined(__CUDACC__)

typedef struct VoxelSubVox_
{
  OccupancyType occupancy;
  SubVoxelPatternType sub_voxel;
} VoxelSubVox;

typedef struct VoxelSimple_
{
  OccupancyType occupancy;
} VoxelSimple;
#endif  // REGION_UPDATE_BASE_CL

#ifdef SUB_VOX
#define REGION_UPDATE_KERNEL regionRayUpdateSubVox
#define VISIT_LINE_VOXEL visitVoxelRegionUpdateSubVox
#define WALK_LINE_VOXELS walkRegionLineSubVox
#define VOXEL_TYPE VoxelSubVox

#else  // SUB_VOX
#define REGION_UPDATE_KERNEL regionRayUpdate
#define VISIT_LINE_VOXEL visitVoxelRegionUpdate
#define WALK_LINE_VOXELS walkRegionLine
#define VOXEL_TYPE VoxelSimple

#endif  // SUB_VOX

#ifndef REGION_UPDATE_BASE_CL
// User data for voxel visit callback.
struct LineWalkData
{
  // Voxel occupancy memory. All regions use a shared buffer.
  __global void *voxels;
  // Array of region keys for currently loaded regions.
  __global int3 *regionKeys;
  // Array of offsets for each regionKey into voxels. These are byte offsets.
  __global ulong *regionMemOffsets;
  // The region currently being traversed. Also used to reduce searching the regionKeys and regionMemOffsets.
  int3 currentRegion;
  // The region currently being traversed for sub-voxel indexing. Used to reduce searching the subVoxKeys and
  // subVoxMemOffsets.
  int3 currentSubVoxRegion;
  // Size of a region in voxels.
  int3 regionDimensions;
  // MapMode/voxel value adjustment for keys along the line segment, but not the sample voxel.
  float rayAdjustment;
  // MapMode/voxel value adjustment for the sample voxel.
  float sampleAdjustment;
  // MapMode/voxel minimum allowed value.
  float voxelValueMin;
  // MapMode/voxel maximum allowed value.
  float voxelValueMax;
  // Weighting given to new position when integrating sub-voxel positions.
  float sub_voxel_weighting;
  // Number of regions in regionKeys/regionMemOffsets.
  uint regionCount;
  // The regionMemOffsets value corresponding to the currentRegion. This is an index offset into voxels, not
  // a byte offset.
  uint regionVoxelOffset;
  // Local coordinate within the end voxel.
  float3 subVoxelCoord;
#ifdef STORE_DEBUG_INFO
  const struct GpuKey *startKey;
  const struct GpuKey *endKey;
#endif  // STORE_DEBUG_INFO
};
#endif  // REGION_UPDATE_BASE_CL

#ifdef SUB_VOX
/// Update the sub-voxel pattern at @p target_address by including the bit(s) from @p pattern_to_add.
/// This is done using atomic operations.
///
/// Each bit in the pattern indicates occupancy at a particular sub-voxel location.
__device__ void updateSubVoxelPosition(__global SubVoxelPatternType *target_address, float3 sub_voxel_pos,
                                       float voxel_resolution, float sub_voxel_weigthing);
#endif  // SUB_VOX

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// Psuedo header guard to prevent function implementation duplication.
#ifdef SUB_VOX
__device__ void updateSubVoxelPosition(__global SubVoxelPatternType *target_address, float3 sub_voxel_pos,
                                       float voxel_resolution, float sub_voxel_weigthing)
{
  uint old_value;
  uint new_value;

  // Few iterations as it's less important to get this right.
  const int iteration_limit = 10;
  int iteration_count = 0;
  do
  {
    old_value = *target_address;
    new_value = subVoxelUpdate(old_value, sub_voxel_pos, voxel_resolution, sub_voxel_weigthing);
    ++iteration_count;
  } while (!gputilAtomicCas(target_address, old_value, new_value) && iteration_limit < iteration_count);
}
#endif  // SUB_VOX


// Implement the voxel traversal function. We update the value of the voxel using atomic instructions.
__device__ bool VISIT_LINE_VOXEL(const struct GpuKey *voxelKey, bool isEndVoxel, float voxelResolution, void *userData)
{
  float old_value, new_value;
  __global OccupancyType *occupancy_ptr;

  struct LineWalkData *lineData = (struct LineWalkData *)userData;
  __global VOXEL_TYPE *voxels = (__global VOXEL_TYPE *)lineData->voxels;

  // Adjust value by rayAdjustment unless this is the sample voxel.
  const float adjustment = (!isEndVoxel) ? lineData->rayAdjustment : lineData->sampleAdjustment;

  // Resolve memory offset for the region of interest.
  if (!regionsResolveRegion(voxelKey, &lineData->currentRegion, &lineData->regionVoxelOffset, lineData->regionKeys,
                            lineData->regionMemOffsets, lineData->regionCount, sizeof(VOXEL_TYPE)))
  {
    // We can fail to resolve regions along the in the line. This can occurs for several reasons:
    // - Floating point error differences between CPU and GPU line walking means that the GPU may walk into the edge
    //   of a region not hit when walking the regions on CPU.
    // - Regions may not be uploaded due to extents limiting on CPU.
#ifdef REPORT_MISSING_REGIONS
    printf("%u region missing: " KEY_F "\n"
#ifdef STORE_DEBUG_INFO
           "  Voxels: " KEY_F "->" KEY_F "\n"
#endif
           ,
           get_global_id(0), KEY_A(*voxelKey)
#ifdef STORE_DEBUG_INFO
                               ,
           KEY_A(*lineData->startKey), KEY_A(*lineData->endKey)
#endif
    );
#endif
    return true;
  }

  // This voxel lies in the region. We will make a value adjustment.
  // Work out which voxel to modify.
  ulong vi = voxelKey->voxel[0] + voxelKey->voxel[1] * lineData->regionDimensions.x +
             voxelKey->voxel[2] * lineData->regionDimensions.x * lineData->regionDimensions.y +
             lineData->regionVoxelOffset;

  if (voxelKey->voxel[0] < lineData->regionDimensions.x && voxelKey->voxel[1] < lineData->regionDimensions.y &&
      voxelKey->voxel[2] < lineData->regionDimensions.z)
  {
    occupancy_ptr = &voxels[vi].occupancy;

#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
    // Under high contension we can end up repeatedly failing to write the voxel value.
    // The primary concern is not deadlocking the GPU, so we put a hard limit on the numebr of
    // attempts made.
    const int iterationLimit = 20;
    int iterations = 0;
#endif
    do
    {
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
      if (iterations++ > iterationLimit)
      {
        break;
      }
#endif
      // Calculate a new value for the voxel.
      old_value = new_value = gputilAtomicLoadF32(occupancy_ptr);

      // Uninitialised voxels start at INFINITY.
      new_value = (new_value != INFINITY) ? new_value + adjustment : adjustment;
      // Clamp the value.
      new_value = clamp(new_value, lineData->voxelValueMin, lineData->voxelValueMax);

      // Now try write the value, looping if we fail to write the new value.
      // mem_fence(CLK_GLOBAL_MEM_FENCE);
    } while (new_value != old_value && !gputilAtomicCasF32(occupancy_ptr, old_value, new_value));

#ifdef SUB_VOX
    if (adjustment > 0)
    {
      updateSubVoxelPosition(&voxels[vi].sub_voxel, lineData->subVoxelCoord, voxelResolution,
                             lineData->sub_voxel_weighting);
    }
#endif  // SUB_VOX
  }
  else
  {
    printf("%u Out of bounds: %u " KEY_F "\n", get_global_id(0), vi, KEY_A(*voxelKey));
    return false;
  }

  return true;
}

// Must be included after WALK_LINE_VOXELS and VISIT_LINE_VOXEL and the VISIT_LINE_VOXEL function is defined
#include "LineWalk.cl"

//------------------------------------------------------------------------------
// Kernel
//------------------------------------------------------------------------------

/// Integrate rays into voxel map regions.
///
/// Invoked one thread per ray (per @p lineKeys pair).
///
/// Like keys are provided in start/end key pairs in @p lineKeys where there are @p lineCount pairs. Each thread
/// extracts it's start/end pair and performs a line walking algorithm from start to end key. The lines start end points
/// are also provided, relative to the centre of the starting voxel. These start/end coordinate pairs are in
/// @p localLines. The coordinates for each line are local to the starting voxel centre in order to avoid precision
/// issues which may be introduced in converting from a common double precision frame on CPU into a single precision
/// frame in GPU (we do not support double precision GPU due to the limited driver support).
///
/// For each voxel key along the line, we resolve a voxel in @p voxels_mem by cross referencing in
/// @p occupancy_region_keys_global, @p occupancy_region_mem_offsets_global and @p regionCount. Voxels are split into
/// regions in contiguous chunks in @p voxels_mem. The @c GpuKey::region for a voxel is matched in lookup @p
/// occupancy_region_keys_global and the index into @p occupancy_region_keys_global recorded. This index is used to
/// lookup @p occupancy_region_mem_offsets_global, which provides a byte offset (not elements) from @p voxels_mem at
/// which the voxel memory for this voxel begins. Each voxel region has a number of voxels equal to
/// <tt>regionDimensions.x * regionDimensions.y * regionDimensions.z</tt>.
///
/// Once voxel memory is resolved, the value of that voxel is updated by either adding @p rayAdjustment for all but
/// the last voxel in the line, or @p lastVoxelAdjustment for the last voxel (exception listed below). The value is
/// clamped to the range <tt>[voxelValueMin, voxelValueMax]</tt>. This adjustment is made in global memory using
/// atomic operations. Success is not guaranteed, but is highly probably. This contension has performance impacts, but
/// was found to be the best overall approach for performance.
///
/// The value adjustment of @p lastVoxelAdjustment is normally used for the last voxel in each line. This behaviour may
/// be changed per line, by setting the value of @p GpuKey::voxel[3] (normally unused) to 1. This indicates the line has
/// been clipped.

__kernel void REGION_UPDATE_KERNEL(__global VOXEL_TYPE *voxels_mem, __global int3 *occupancy_region_keys_global,
                                   __global ulong *occupancy_region_mem_offsets_global, uint regionCount,
                                   __global struct GpuKey *lineKeys, __global float3 *localLines, uint lineCount,
                                   int3 regionDimensions, float voxelResolution, float rayAdjustment,
                                   float lastVoxelAdjustment, float voxelValueMin, float voxelValueMax,
                                   float sub_voxel_weighting)
{
  // Only process valid lines.
  if (get_global_id(0) >= lineCount)
  {
    return;
  }

  struct LineWalkData lineData;
  lineData.voxels = voxels_mem;
  lineData.regionKeys = occupancy_region_keys_global;
  lineData.regionMemOffsets = occupancy_region_mem_offsets_global;
  lineData.regionDimensions = regionDimensions;
  lineData.rayAdjustment = rayAdjustment;
  lineData.sampleAdjustment = lastVoxelAdjustment;
  lineData.voxelValueMin = voxelValueMin;
  lineData.voxelValueMax = voxelValueMax;
  lineData.sub_voxel_weighting = sub_voxel_weighting;
  lineData.regionCount = regionCount;

  regionsInitCurrent(&lineData.currentRegion, &lineData.regionVoxelOffset);

  // Now walk the clipped ray.
  struct GpuKey startKey, endKey;
  copyKey(&startKey, &lineKeys[get_global_id(0) * 2 + 0]);
  copyKey(&endKey, &lineKeys[get_global_id(0) * 2 + 1]);

  const float3 lineStart = localLines[get_global_id(0) * 2 + 0];
  const float3 lineEnd = localLines[get_global_id(0) * 2 + 1];

#ifdef SUB_VOX
  // We don't need a precise conversion to a voxel key here. We simply need to logically quantise in order to work out
  // the sub-voxel offset. Essentially we have:
  //   s = E - R floor(E/R + 0.5)
  // where:
  //   s: sub-voxel position
  //   E: ray End point
  //   R: voxel resolution
  // The addition of 0.5 applies the same half voxel offset used elsewhere. We use pointToRegionCoord() to do this
  lineData.subVoxelCoord.x = lineEnd.x - pointToRegionCoord(lineEnd.x, voxelResolution) * voxelResolution;
  lineData.subVoxelCoord.y = lineEnd.y - pointToRegionCoord(lineEnd.y, voxelResolution) * voxelResolution;
  lineData.subVoxelCoord.z = lineEnd.z - pointToRegionCoord(lineEnd.z, voxelResolution) * voxelResolution;

  // Validate sub-voxel coordinate calculation.
  // We use 0.5001 * resolution rather than 0.5 to allow for floating point error when clipping to exact voxel bounds.
  if (lineData.subVoxelCoord.x < -0.5001 * voxelResolution || lineData.subVoxelCoord.x > 0.5001 * voxelResolution ||
      lineData.subVoxelCoord.y < -0.5001 * voxelResolution || lineData.subVoxelCoord.y > 0.5001 * voxelResolution ||
      lineData.subVoxelCoord.z < -0.5001 * voxelResolution || lineData.subVoxelCoord.z > 0.5001 * voxelResolution)
  {
    printf("sub-voxel-out [%f, %f]: (%f %f %f) -> (%f %f %f)\n", -0.5 * voxelResolution, 0.5 * voxelResolution,
           lineEnd.x, lineEnd.y, lineEnd.z, lineData.subVoxelCoord.x, lineData.subVoxelCoord.y,
           lineData.subVoxelCoord.z);
  }
#else  // SUB_VOX
  lineData.subVoxelCoord = make_float3(0, 0, 0);
#endif // SUB_VOX

#ifdef STORE_DEBUG_INFO
  lineData.startKey = &startKey;
  lineData.endKey = &endKey;
#endif

  WALK_LINE_VOXELS(&startKey, &endKey, &lineStart, &lineEnd, &regionDimensions, voxelResolution, &lineData);
}

#undef REGION_UPDATE_KERNEL
#undef VISIT_LINE_VOXEL
#undef WALK_LINE_VOXELS
#undef VOXEL_TYPE

#ifndef REGION_UPDATE_BASE_CL
#define REGION_UPDATE_BASE_CL
#endif  // REGION_UPDATE_BASE_CL
