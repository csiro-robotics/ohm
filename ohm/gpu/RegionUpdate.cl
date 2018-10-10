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

#include "Regions.cl"
#include "LineWalk.cl"

//------------------------------------------------------------------------------
// Declarations
//------------------------------------------------------------------------------

#if __OPENCL_C_VERSION__ >= 200
typedef atomic_float occupancy_type;
#else  // __OPENCL_C_VERSION__ >= 200
typedef float occupancy_type;
#endif // __OPENCL_C_VERSION__ >= 200

// User data for walkLineVoxel() callback.
struct LineWalkData
{
  // Voxel occupancy memory. All regions use a shared buffer.
  __global occupancy_type *voxels;
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
#if __OPENCL_C_VERSION__ >= 200
  float old_value, new_value;
  __global occupancy_type *voxel_ptr;
#else  // __OPENCL_C_VERSION__ >= 200
  union
  {
    float f;
    int i;
  } old_value, new_value;

  union
  {
    __global volatile float *f;
    __global volatile int *i;
  } voxel_ptr;
#endif // __OPENCL_C_VERSION__ >= 200

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
    #if __OPENCL_C_VERSION__ >= 200
    voxel_ptr = &lineData->voxels[vi];
    #else  // __OPENCL_C_VERSION__ >= 200
    voxel_ptr.f = &lineData->voxels[vi];
    #endif // __OPENCL_C_VERSION__ >= 200

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
      old_value = new_value = atomic_load_explicit(voxel_ptr, memory_order_relaxed);
      #else  // __OPENCL_C_VERSION__ >= 200
      old_value.i = new_value.i = *voxel_ptr.i;
      #endif  // __OPENCL_C_VERSION__ >= 200

      // Adjust value by rayAdjustment unless this is the sample voxel.
      const float adjustment = (!isEndVoxel) ? lineData->rayAdjustment : lineData->sampleAdjustment;

      #if __OPENCL_C_VERSION__ >= 200
      // Uninitialised voxels start at INFINITY.
      new_value = (new_value != INFINITY) ? new_value + adjustment : adjustment;
      // Clamp the value.
      new_value = clamp(new_value, lineData->voxelValueMin, lineData->voxelValueMax);
      #else  // __OPENCL_C_VERSION__ >= 200
      // Uninitialised voxels start at INFINITY.
      new_value.f = (new_value.f != INFINITY) ? new_value.f + adjustment : adjustment;
      // Clamp the value.
      new_value.f = clamp(new_value.f, lineData->voxelValueMin, lineData->voxelValueMax);
      #endif // __OPENCL_C_VERSION__ >= 200

      // Now try write the value, looping if we fail to write the new value.
      //mem_fence(CLK_GLOBAL_MEM_FENCE);
    #if __OPENCL_C_VERSION__ >= 200
    } while(!atomic_compare_exchange_weak_explicit(voxel_ptr, &old_value, new_value,
                                                   memory_order_release, memory_order_relaxed));
    #else  // __OPENCL_C_VERSION__ >= 200
    } while(atomic_cmpxchg(voxel_ptr.i, old_value.i, new_value.i) != old_value.i);
    //atomic_cmpxchg(voxel_ptr.i, old_value.i, new_value.i);
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
__kernel void regionRayUpdate(__global occupancy_type *voxelsMem,
                              __global int3 *regionKeysGlobal,
                              __global ulong *regionMemOffsetsGlobal,
                              uint regionCount,
                              __global float3 *lines, uint lineCount,
                              int3 regionDimensions, float voxelResolution,
                              float rayAdjustment, float lastVoxelAdjustment,
                              float voxelValueMin, float voxelValueMax
                             )
{
  // Only process valid lines.
  if (get_global_id(0) >= lineCount)
  {
    return;
  }

  float3 lineStart = lines[get_global_id(0) * 2 + 0];
  // For an invalid line, set a zero length line.
  float3 lineEnd = lines[get_global_id(0) * 2 + 1];

  struct LineWalkData lineData;
  lineData.voxels = voxelsMem;
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
}
