//------------------------------------------------------------------------------
// Debug switches for the compiling code to enable (or just uncomment here).
// Deliberately before incldues to configure those files.
//------------------------------------------------------------------------------

// Report regions we can't resolve via printf().
// Note: it is valid to hit a missing region for one voxel in a line due to floating point error between the CPU
// region walking and the voxel walking here. This is acceptable.
//#define REPORT_MISSING_REGIONS

// Limit the number of cells we can traverse in the line traversal. This is a worst case limit.
//#define LIMIT_LINE_WALK_ITERATIONS
// Limit the number of times we try update a voxel value. Probably best to always have this enabled.
#define LIMIT_VOXEL_WRITE_ITERATIONS
#ifdef LIMIT_VOXEL_WRITE_ITERATIONS
// Store additional debug information in LineWalkData for error reporting.
//#define STORE_DEBUG_INFO
#endif // LIMIT_VOXEL_WRITE_ITERATIONS

//------------------------------------------------------------------------------
// Includes
//------------------------------------------------------------------------------
#include "gpu_ext.h"

#include "regions.cl"
#include "linewalk.cl"

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------

// User data for walkLineVoxel() callback.
struct LineWalkData
{
  // Voxel occupancy memory. All regions use a shared buffer.
  __global float *voxels;
  // __global struct MapNode *voxels;
  // Array of region keys for currently loaded regions.
  __global int3 *regionKeys;
  // Array of offsets for each regionKey into voxels. These are byte offsets.
  __global ulong *regionMemOffsets;
  // The region currently being traversed. Also used to reduce searching the regionKeys and regionMemOffsets.
  int3 currentRegion;
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
  // Number of regions in regionKeys/regionMemOffsets.
  uint regionCount;
  // The regionMemOffsets value corresponding to the currentRegion. This is an index offset into voxels, not
  // a byte offset.
  uint regionVoxelOffset;
  #ifdef STORE_DEBUG_INFO
  const float3 *lineStart;
  const float3 *lineEnd;
  const struct GpuKey *startKey;
  const struct GpuKey *endKey;
  #endif // STORE_DEBUG_INFO
};

//------------------------------------------------------------------------------
// Functions
//------------------------------------------------------------------------------

// Implement the voxel travesal function. We update the value of the voxel using atomic instructions.
bool walkLineVoxel(const struct GpuKey *voxelKey, bool isEndVoxel, void *userData)
{
  union
  {
    float f;
    int i;
  } oldValue, newValue;

  union
  {
    __global volatile float *f;
    #if __OPENCL_C_VERSION__ >= 200
    __global atomic_int *i;
    #else  // __OPENCL_C_VERSION__ >= 200
    __global volatile int *i;
  #endif // __OPENCL_C_VERSION__ >= 200
  } valuePtr;

  struct LineWalkData *lineData = (struct LineWalkData *)userData;

  // if (get_global_id(0) == 6)
  // {
  //   printf("%u @ " KEY_F "\n", get_global_id(0), KEY_A(*voxelKey));
  // }

  // Resolve memory offset for the region of interest.
  if (!regionsResolveRegion(voxelKey, &lineData->currentRegion, &lineData->regionVoxelOffset,
                            lineData->regionKeys, lineData->regionMemOffsets, lineData->regionCount,
                            sizeof(*lineData->voxels)))
  {
    // We can fail to resolve a region an isolated voxel in the line. See REPORT_MISSING_REGIONS comments.
    #ifdef REPORT_MISSING_REGIONS
    printf("%u region missing: " KEY_F "\n"
           #ifdef STORE_DEBUG_INFO
           "  Line: (%.16f,%.16f,%.16f)->(%.16f,%.16f,%.16f)\n"
           "  Voxels: " KEY_F "->" KEY_F"\n"
           #endif // STORE_DEBUG_INFO
           , get_global_id(0), KEY_A(*voxelKey)
           #ifdef STORE_DEBUG_INFO
           , lineData->lineStart->x, lineData->lineStart->y, lineData->lineStart->z
           , lineData->lineEnd->x, lineData->lineEnd->y, lineData->lineEnd->z
           , KEY_A(*lineData->startKey), KEY_A(*lineData->endKey)
           #endif // STORE_DEBUG_INFO
           );
    #endif // REPORT_MISSING_REGIONS
    return true;
  }

  // This voxel lies in the region. We will make a value adjustment.
  // Work out which voxel to modify.
  const ulong vi = voxelKey->voxel[0] +
                   voxelKey->voxel[1] * lineData->regionDimensions.x +
                   voxelKey->voxel[2] * lineData->regionDimensions.x * lineData->regionDimensions.y +
                   lineData->regionVoxelOffset;

  if (voxelKey->voxel[0] < lineData->regionDimensions.x &&
      voxelKey->voxel[1] < lineData->regionDimensions.y &&
      voxelKey->voxel[2] < lineData->regionDimensions.z)
  {
    valuePtr.f = &lineData->voxels[vi];

    #ifdef LIMIT_VOXEL_WRITE_ITERATIONS
    // Under high contension we can end up repeatedly failing to write the voxel value.
    // The primary concern is not deadlocking the GPU, so we put a hard limit on the numebr of
    // attempts made.
    const int iterationLimit = 200;
    int iterations = 0;
    #endif // LIMIT_VOXEL_WRITE_ITERATIONS
    do
    {
      #ifdef LIMIT_VOXEL_WRITE_ITERATIONS
      if (iterations++ > iterationLimit)
      {
        printf("%u excessive voxel update iterations " KEY_F ".\n", get_global_id(0), KEY_A(*voxelKey));
        break;
      }
      #endif // LIMIT_VOXEL_WRITE_ITERATIONS

      // Calculate a new value for the voxel.
      #if __OPENCL_C_VERSION__ >= 200
      oldValue.f = newValue.f = *valuePtr.f;
      #else  // __OPENCL_C_VERSION__ >= 200
      oldValue.i = newValue.i = *valuePtr.i;
      #endif  // __OPENCL_C_VERSION__ >= 200

      // Adjust value by rayAdjustment unless this is the sample voxel.
      const float adjustment = (!isEndVoxel) ? lineData->rayAdjustment : lineData->sampleAdjustment;
      // Uninitialised voxels start at INFINITY.
      newValue.f = (newValue.f != INFINITY) ? newValue.f + adjustment : adjustment;

      // Clamp the value.
      newValue.f = clamp(newValue.f, lineData->voxelValueMin, lineData->voxelValueMax);

      // Now try write the value, looping if we fail to write the new value.
      //mem_fence(CLK_GLOBAL_MEM_FENCE);
    #if __OPENCL_C_VERSION__ >= 200
    } while(atomic_compare_exchange_strong(valuePtr.i, &oldValue.i, newValue.i) != oldValue.i);
    #else  // __OPENCL_C_VERSION__ >= 200
    } while(atomic_cmpxchg(valuePtr.i, oldValue.i, newValue.i) != oldValue.i);
    //atomic_cmpxchg(valuePtr.i, oldValue.i, newValue.i);
    #endif  // __OPENCL_C_VERSION__ >= 200
  }
  else
  {
    printf("%u Out of bounds: %u " KEY_F "\n", get_global_id(0), vi, KEY_A(*voxelKey));
    return false;
  }

  return true;
}


//------------------------------------------------------------------------------
// Kernel
//------------------------------------------------------------------------------
__kernel void regionRayUpdate(__global uchar *voxelsMem,
                              __global int3 *regionKeysGlobal,
                              __global ulong *regionMemOffsetsGlobal,
                              uint regionCount,
                              __global float3 *lines, uint lineCount,
                              int3 regionDimensions, float voxelResolution,
                              float rayAdjustment, float lastVoxelAdjustment,
                              float voxelValueMin, float voxelValueMax
                             )
{
#if 0
  __local int3 *regionKeys[MAX_REGIONS];
  __local ulong *regionMemOffset[MAX_REGIONS];

  // Cache the region keys and memory offsets into local memory.
  for (uint i = 0; i < regionCount; i += get_local_size(0))
  {
    if (i + get_local_id(0) < regionCount)
    {
      regionKeys[i + get_local_id(0)] = regionKeysGlobal[i + get_local_id(0)];
      regionMemOffsets[i + get_local_id(0)] = regionMemOffsetsGlobal[i + get_local_id(0)];
    }
  }

  barrier(CLK_LOCAL_MEM_FENCE);
#endif // #

  // Only process valid lines.
  if (get_global_id(0) >= lineCount)
  {
    return;
  }

  float3 lineStart = lines[get_global_id(0) * 2 + 0];
  // For an invalid line, set a zero length line.
  float3 lineEnd = lines[get_global_id(0) * 2 + 1];

  #if 0
  struct GpuKey startKey, endKey;
  coordToKey(&startKey, &lineStart, &regionDimensions, voxelResolution);
  coordToKey(&endKey, &lineEnd, &regionDimensions, voxelResolution);
  printf("%u " KEY_F " -> " KEY_F "  <=>  (%f %f %f) -> (%f %f %f)\n",
         get_global_id(0),
         KEY_A(startKey), KEY_A(endKey),
         lineStart.x, lineStart.y, lineStart.z,
         lineEnd.x, lineEnd.y, lineEnd.z);
  #else  // #
  struct LineWalkData lineData;
  lineData.voxels = (__global float *)voxelsMem;
  lineData.regionKeys = regionKeysGlobal;
  lineData.regionMemOffsets = regionMemOffsetsGlobal;
  lineData.regionDimensions = regionDimensions;
  lineData.rayAdjustment = rayAdjustment;
  lineData.sampleAdjustment = lastVoxelAdjustment;
  lineData.voxelValueMin = voxelValueMin;
  lineData.voxelValueMax = voxelValueMax;
  lineData.regionCount = regionCount;

  regionsInitCurrent(&lineData.currentRegion, &lineData.regionVoxelOffset);

  // Now walk the clipped ray.
  struct GpuKey startKey, endKey;
  coordToKey(&startKey, &lineStart, &regionDimensions, voxelResolution);
  coordToKey(&endKey, &lineEnd, &regionDimensions, voxelResolution);
  #ifdef STORE_DEBUG_INFO
  lineData.lineStart = &lineStart;
  lineData.lineEnd = &lineEnd;
  lineData.startKey = &startKey;
  lineData.endKey = &endKey;
  #endif // STORE_DEBUG_INFO
  walkLineVoxels(&startKey, &endKey, &lineStart, &lineEnd, &regionDimensions, voxelResolution, &lineData);
  #endif  // #
}
