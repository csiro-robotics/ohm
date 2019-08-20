
#include "gpu_ext.h"
#include "MapCoord.h"
#include "GpuKey.h"

#define WALK_LINE_VOXELS lineKeysWalkLine
#define VISIT_LINE_VOXEL lineKeysVisitVoxel
__device__ bool lineKeysVisitVoxel(const struct GpuKey *voxelKey, bool isEndVoxel, float voxelResolution, void *userData);

// Must be included after above defined
#include "LineWalk.cl"

struct LineWalkData
{
  __global struct GpuKey *lineOut;
  uint maxKeys;
  uint keyCount;
};

__device__ void calculateLineKeys(__global struct GpuKey *lineOut, uint maxKeys,
                       const struct GpuKey *startKey, const struct GpuKey *endKey,
                       const float3 *startPoint, const float3 *endPoint,
                       const int3 *regionDim, float voxelResolution);


__device__ bool lineKeysVisitVoxel(const struct GpuKey *voxelKey, bool isEndVoxel, float voxelResolution, void *userData)
{
  struct LineWalkData *lineData = (struct LineWalkData *)userData;
  copyKey(&lineData->lineOut[1 + lineData->keyCount++], voxelKey);
  return lineData->keyCount + 1 < lineData->maxKeys;
}


__device__ void calculateLineKeys(__global struct GpuKey *lineOut, uint maxKeys,
                       const struct GpuKey *startKey, const struct GpuKey *endKey,
                       const float3 *startPoint, const float3 *endPoint,
                       const int3 *regionDim, float voxelResolution)
{
  struct LineWalkData lineData;
  lineData.lineOut = lineOut;
  lineData.maxKeys = maxKeys;
  lineData.keyCount = 0;

  lineKeysWalkLine(startKey, endKey, startPoint, endPoint, regionDim, voxelResolution, &lineData);

  // Write result count to the first entry.
  lineOut[0].region[0] = lineOut[0].region[1] = lineOut[0].region[2] = lineData.keyCount;
  // printf("%lu => %u keys\n", get_global_id(0), lineData.keyCount);
}


__kernel void calculateLines(__global struct GpuKey *lines_out, uint max_keys_per_line,
                             const __global float3 *queryPointPairs,
                             uint queryCount, int3 regionDim, float voxelResolution
                             )
{
  const bool validThread = (get_global_id(0) < queryCount);

  if (!validThread)
  {
    return;
  }

  float3 startPoint = (validThread) ? queryPointPairs[get_global_id(0) * 2 + 0] : make_float3(0, 0, 0);
  float3 endPoint = (validThread) ? queryPointPairs[get_global_id(0) * 2 + 1] : make_float3(0, 0, 0);
  __global struct GpuKey *lineOut = lines_out + (get_global_id(0) * max_keys_per_line);
  // We convert regionDim from an int3 to an array to allow indexed access.
  struct GpuKey startKey, endKey;

  coordToKey(&startKey, &startPoint, &regionDim, voxelResolution);
  coordToKey(&endKey, &endPoint, &regionDim, voxelResolution);

  // printf("Query Count: %u\nValid? %d\n", queryCount, validThread ? 1 : 0);
  // printf("RD: %d %d %d   Res: %f\n", regionDim.x, regionDim.y, regionDim.z, voxelResolution);
  // printf("From %f %f %f to %f %f %f\n", startPoint.x, startPoint.y, startPoint.z, endPoint.x, endPoint.y, endPoint.z);
  // printf("Keys: " KEY_F " to " KEY_F "\n", KEY_A(startKey), KEY_A(endKey));

  // Adjust start/end point to be relative to the centre of the startKey voxel.
  startPoint -= voxelCentre(&startKey, &regionDim, voxelResolution);
  endPoint -= voxelCentre(&startKey, &regionDim, voxelResolution);

  calculateLineKeys(lineOut, max_keys_per_line, &startKey, &endKey, &startPoint, &endPoint, &regionDim, voxelResolution);
}
