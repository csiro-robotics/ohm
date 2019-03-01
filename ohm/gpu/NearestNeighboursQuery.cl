// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpu_ext.h"

//-----------------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function prototypes.
//-----------------------------------------------------------------------------
__device__ float3 regionCentre(float3 regionSpatialDimensions, short3 regionKey);
__device__ float3 voxelCentre(uchar3 voxelKey, uchar3 regionVoxelDimensions, float3 regionCentre, float voxelResolution);

//-----------------------------------------------------------------------------
// Implementation
//-----------------------------------------------------------------------------
__device__ float3 regionCentre(float3 regionSpatialDimensions, short3 regionKey)
{
  float3 centre;
  centre.x = regionKey.x * regionSpatialDimensions.x;
  centre.y = regionKey.y * regionSpatialDimensions.y;
  centre.z = regionKey.z * regionSpatialDimensions.z;
  return centre;
}


__device__ float3 voxelCentre(uchar3 voxelKey, uchar3 regionVoxelDimensions, float3 regionCentre, float voxelResolution)
{
  float3 voxel;
  float3 regionMin;

  regionMin = regionCentre - make_float3(0.5f * regionVoxelDimensions.x * voxelResolution,
                                         0.5f * regionVoxelDimensions.y * voxelResolution,
                                         0.5f * regionVoxelDimensions.z * voxelResolution);

  voxel = regionMin + make_float3(voxelKey.x * voxelResolution + 0.5f * voxelResolution,
                                  voxelKey.y * voxelResolution + 0.5f * voxelResolution,
                                  voxelKey.z * voxelResolution + 0.5f * voxelResolution);

  return voxel;
}

//-----------------------------------------------------------------------------
// Kernel
//-----------------------------------------------------------------------------
// Voxels are reprented by their occupancy values,
__kernel void nearestNeighbours(uchar3 regionVoxelDimensions,
                                float3 regionSpatialDimensions,
                                __global const float *occupancy,
                                __global const short3 *nodeRegions,
                                __global const uchar3 *nodeKeys,
                                __global float *ranges,
                                __global short3 *resultRegionKeys,
                                __global uchar3 *resultVoxelKeys,
                                __global atomic_uint *resultCount,
                                float3 nearPoint,
                                float searchRadius,
                                float occupancyThresholdValue,
                                float voxelResolution,
                                int unknownAsOccupied,
                                uint queuedNodes
                               )
{
  const uint nodeIndex = get_global_id(0);
  const bool validIndex = nodeIndex < queuedNodes;
  short3 regionKey;
  uchar3 nodeKey;
  float nodeValue;
  float3 separation;
  float range;
  bool occupied;

  nodeValue = (validIndex) ? occupancy[nodeIndex] : 0.0f;
  regionKey = (validIndex) ? nodeRegions[nodeIndex] : make_short3(0, 0, 0);
  nodeKey = (validIndex) ? nodeKeys[nodeIndex] : make_uchar3(0, 0, 0);
  separation = voxelCentre(nodeKey, regionVoxelDimensions, regionCentre(regionSpatialDimensions, regionKey), voxelResolution);
  separation = nearPoint - separation;

  range = sqrt(dot(separation, separation));

  occupied = nodeValue >= occupancyThresholdValue && (unknownAsOccupied || nodeValue < INFINITY);

  if (validIndex && range < searchRadius && occupied)
  {
    uint globalIndex = gputilAtomicInc(resultCount);
    ranges[globalIndex] = range;
    resultVoxelKeys[globalIndex] = nodeKey;
    resultRegionKeys[globalIndex] = regionKey;
  }
}


__kernel void showNNInfo(__global volatile uint *resultCount)
{
  if (get_global_id(0) == 0)
  {
    printf("Count: %u\n", *resultCount);
  }
}
