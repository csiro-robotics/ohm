// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#include "gpu_ext.h"

// Validate node key generation: each thread writes a result containing the range to its
// voxel key and voxel range to query point.
//#define VALIDATE_KEYS

// Write only valid results to local memory before using local thread zero to migrate
// valid local results to global result set.
// This define should be set from the code invoking the kernel as it also removes local
// memory variables, which should no longer be allocated for the kernel.
//#define CACHE_LOCAL_RESULTS

//-----------------------------------------------------------------------------
// Types
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function prototypes.
//-----------------------------------------------------------------------------
float3 regionCentre(float3 regionSpatialDimensions, short3 regionKey);
float3 voxelCentre(uchar3 voxelKey, uchar3 regionVoxelDimensions, float3 regionCentre, float voxelResolution);

//-----------------------------------------------------------------------------
// Implementation
//-----------------------------------------------------------------------------
float3 regionCentre(float3 regionSpatialDimensions, short3 regionKey)
{
  float3 centre;
  centre.x = regionKey.x * regionSpatialDimensions.x;
  centre.y = regionKey.y * regionSpatialDimensions.y;
  centre.z = regionKey.z * regionSpatialDimensions.z;
  return centre;
}


float3 voxelCentre(uchar3 voxelKey, uchar3 regionVoxelDimensions, float3 regionCentre, float voxelResolution)
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
                                volatile __global uint *resultCount,
                                float3 nearPoint,
                                float searchRadius,
                                float occupancyThresholdValue,
                                float voxelResolution,
                                int unknownAsOccupied,
                                uint queuedNodes
#ifdef CACHE_LOCAL_RESULTS
                                , __local float *localRanges
                                , __local short3 *localRegionKeys
                                , __local uchar3 *localVoxelKeys
#endif // CACHE_LOCAL_RESULTS
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
#ifdef CACHE_LOCAL_RESULTS
  volatile __local uint nextLocalIndex;

  if (get_local_id(0) == 0)
  {
    nextLocalIndex = 0;
  }
#endif // CACHE_LOCAL_RESULTSs

  nodeValue = (validIndex) ? occupancy[nodeIndex] : 0.0f;
  regionKey = (validIndex) ? nodeRegions[nodeIndex] : make_short3(0, 0, 0);
  nodeKey = (validIndex) ? nodeKeys[nodeIndex] : make_uchar3(0, 0, 0);
  separation = voxelCentre(nodeKey, regionVoxelDimensions, regionCentre(regionSpatialDimensions, regionKey), voxelResolution);
  separation = nearPoint - separation;

  range = sqrt(dot(separation, separation));

  occupied = nodeValue >= occupancyThresholdValue && (unknownAsOccupied || nodeValue < INFINITY);

#ifndef VALIDATE_KEYS

#ifndef CACHE_LOCAL_RESULTS
  if (validIndex && range < searchRadius && occupied)
  {
    uint globalIndex = atomic_inc(resultCount);
    ranges[globalIndex] = range;
    resultVoxelKeys[globalIndex] = nodeKey;
    resultRegionKeys[globalIndex] = regionKey;
  }
#else  // !CACHE_LOCAL_RESULTS
  barrier(CLK_LOCAL_MEM_FENCE);

  // Add valid results to the localRanges/localVoxelKeys arrays.
  // These will be migrated to the global results by thread 0 in the local group.
  if (validIndex && range <= searchRadius && occupied)
  {
    uint localIndex = atomic_inc(&nextLocalIndex);
    localRanges[localIndex] = range;
    localVoxelKeys[localIndex] = nodeKey;
    localRegionKeys[localIndex] = regionKey;
  }

  barrier(CLK_LOCAL_MEM_FENCE);

  // Migrate local results to global.
  if (get_local_id(0) == 0)
  {
    uint globalOffset = atomic_add(resultCount, nextLocalIndex);
    for (uint i = 0; i < nextLocalIndex; ++i)
    {
      ranges[globalOffset + i] = localRanges[i];
      resultVoxelKeys[globalOffset + i] = localVoxelKeys[i];
      resultRegionKeys[globalOffset + i] = localRegionKeys[i];
    }
  }
#endif // !CACHE_LOCAL_RESULTS

#else  // !VALIDATE_KEYS
  if (validIndex)
  {
    ranges[nodeIndex] = range;
    resultVoxelKeys[nodeIndex] = nodeKey;
    resultRegionKeys[nodeIndex] = regionKey;
  }

  if (get_global_id(0) == 0)
  {
    *resultCount = queuedNodes;
  }
#endif // !VALIDATE_KEYS
}


__kernel void showNNInfo(__global volatile uint *resultCount)
{
  if (get_global_id(0) == 0)
  {
    printf("Count: %u\n", *resultCount);
  }
}
