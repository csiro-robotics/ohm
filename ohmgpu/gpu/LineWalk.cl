// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

//------------------------------------------------------------------------------
// Note on building this code.
// This code may be used by multiple OpenCL kernels, but has to support the same
// for CUDA compilation. This is slightly tricky because OpenCL has entiredly
// separate compilation units, while CUDA does not. To work around this, we
// allow the code to be included/compiled multiple times with a different set
// of controlling defines. These are documented below:
// - WALK_LINE_VOXELS : the main function name
// - VISIT_LINE_VOXEL : the function to call when visiting a voxel. Must have
// the signature given below.
//
/// This function is called for each voxel traversed by the line walking function.
///
/// This function is not implemented in this source file, and must be implemented by the source including this file.
///
/// The @p walkFlags may be used to modify the algorithm behaviour and semantics as follows:
///
/// - @c kLineWalkFlagReverse reverse walk. The algorithm iterates from @p endVoxel to @p startVoxel instead.
/// - @c kLineWalkFlagReverse force reporting the @p endVoxel last in a reverse walk.
///
/// @param voxelKey The key for the voxel currently being traversed. This voxel is on the line.
/// @param voxelMarker Set to @p kLineWalkMarkerStart (1) if @p voxelKey is the @c startKey or
///   @c kLineWalkMarkerEnd (2) if it is the @c endKey . Zero otherwise.
/// @param voxelResolution The edge length of each voxel cube.
/// @param startKey The first voxel in the line segment.
/// @param endKey The last voxel in the line segment.
/// @param voxelResolution The size of the voxels.
/// @param entryRange How far from the origin has been traversed before entering @p voxelKey . Is adapted for
///   @c kLineWalkFlagReverse .
/// @param exitRange How far from the origin has been traversed when exiting @p voxelKey . Is adapted for
///   @c kLineWalkFlagReverse .
/// @param walkFlags Flags affecting how the algorithm operates.
/// @param userData User data pointer.
/// @return True to continue traversing the line, false to abort traversal.
// __device__ bool VISIT_LINE_VOXEL(const GpuKey *voxelKey, int voxelMarker,
//                                  const GpuKey *startKey, const GpuKey *endKey,
//                                  float voxelResolution, float entryRange, float exitRange, int walkFlags,
///                                 void *userData);
//------------------------------------------------------------------------------
#include "GpuKey.h"
#include "LineWalkMarkers.cl"

#ifndef WALK_LINE_VOXELS
#error WALK_LINE_VOXELS must be used to define the voxel walking function.
#endif  // WALK_LINE_VOXELS

#ifndef VISIT_LINE_VOXEL
#error VISIT_LINE_VOXEL must be used to define the voxel visiting function
#endif  // VISIT_LINE_VOXEL

/// Calculate the @p GpuKey for @p point local to the region's minimum extents corner.
/// @param[out] key The output key.
/// @param point The coordinate to calculate the @p key for in region local coordinates.
/// @paramregionDim Defines the size of a region in voxels. Used to update the @p GpuKey.
/// @param voxelResolution Size of a voxel from one face to another.
/// @return True if @c point lies in the region, false otherwise.
__device__ bool coordToKey(GpuKey *key, const float3 *point, const int3 *regionDim, float voxelResolution);

/// Calculates the centre of the voxel defined by @p key (global space).
/// @param key The key marking the voxel of interest.
/// @paramregionDim Defines the size of a region in voxels. Used to update the @p GpuKey.
/// @param voxelResolution Size of a voxel from one face to another.
/// @return The centre of the voxel defined by @p key.
inline __device__ float3 voxelCentre(const GpuKey *key, const int3 *regionDim, float voxelResolution);

/// Get the @p index component of @c float3 @p v
inline __device__ float getf3(const float3 *v, int index);
/// Get the @p index component of @c int3 @p v
inline __device__ int geti3(const int3 *v, int index);

/// Choose the next axis to walk. In a forward walk, we bias axis 2, 1, 0 when equal. In a reverse walk, we bias
/// 0, 1, 2 to yield similar results.
inline __device__ int nextAxis(float timeMax[3], bool forwardWalk);

// Psuedo header guard to prevent symbol duplication.
#ifndef LINE_WALK_CL
#define LINE_WALK_CL

/// Line walking function for use by kernels.
/// The algorithm walks the voxels from @p startKey to @p endKey. The line segment is defined relative to the centre of
/// the @p startkey voxel with line points @p startPoint and @p endPoint respectively.
///
/// @c WALK_LINE_VOXELS() is invoked for each voxel traversed.
///
/// Based on J. Amanatides and A. Woo, "A fast voxel traversal algorithm for raytracing," 1987.
///
/// @param startKey The key for the voxel containing @p startPoint.
/// @param endKey The key for the voxel containing @p endPoint.
/// @param firstVoxelCentre Coordinate of the centre of the first voxel to walk, in the same frame as @c startPoint
///   and @c endPoint. Normally this is the coordinate of the start voxle, but when @p kLineWalkFlagReverse is set,
///   this must be the coordinate of the end voxel.
/// @param startPoint The start point of the line segment to traverse, relative to the centre of the
///   start voxel (identified by startKey). That is the origin is the centre of the startKey voxel.
/// @param endPoint The end point of the line segment to traverse, relative to the centre of the
///   start voxel (identified by startKey). That is the origin is the centre of the startKey voxel.
/// @paramregionDim Defines the size of a region in voxels. Used to update the @p GpuKey.
/// @param voxelResolution Size of a voxel from one face to another.
/// @param walkFlags Flags affecting the algorithm behaviour. See @c LineWalkFlag .
/// @param userData User pointer passed to @c walkLineVoxel().
__device__ void WALK_LINE_VOXELS(const GpuKey *startKey, const GpuKey *endKey, const float3 *firstVoxelCentre,
                                 const float3 *startPoint, const float3 *endPoint, const int3 *regionDim,
                                 float voxelResolution, int walkFlags, void *userData);


inline __device__ bool coordToKey(GpuKey *key, const float3 *point, const int3 *regionDim, float voxelResolution)
{
  // Quantise.
  key->region[0] = pointToRegionCoord(point->x, regionDim->x * voxelResolution);
  key->region[1] = pointToRegionCoord(point->y, regionDim->y * voxelResolution);
  key->region[2] = pointToRegionCoord(point->z, regionDim->z * voxelResolution);

  // Localise.
  // Trying to minimise local variables for GPU.
  // The value we pass to pointToRegionVoxel is logically:
  //    point - regionCentre - regionHalfExtents
  // or
  //    point - regionMin
  // which equates to the point in region local coordinates.
  // printf("p.x(%f) = %f - (%f - %f)   : regionMin: %f\n",
  //        point->x - (regionCentreCoord(key->region[0], regionDim->x * voxelResolution) - 0.5f * regionDim->x *
  //        voxelResolution), point->x, regionCentreCoord(key->region[0], regionDim->x * voxelResolution), 0.5f *
  //        regionDim->x * voxelResolution, regionCentreCoord(key->region[0], regionDim->x * voxelResolution) - 0.5f *
  //        regionDim->x * voxelResolution);
  key->voxel[0] = pointToRegionVoxel(point->x - (regionCentreCoord(key->region[0], regionDim->x * voxelResolution) -
                                                 0.5f * regionDim->x * voxelResolution),
                                     voxelResolution, regionDim->x * voxelResolution);
  key->voxel[1] = pointToRegionVoxel(point->y - (regionCentreCoord(key->region[1], regionDim->y * voxelResolution) -
                                                 0.5f * regionDim->y * voxelResolution),
                                     voxelResolution, regionDim->y * voxelResolution);
  key->voxel[2] = pointToRegionVoxel(point->z - (regionCentreCoord(key->region[2], regionDim->z * voxelResolution) -
                                                 0.5f * regionDim->z * voxelResolution),
                                     voxelResolution, regionDim->z * voxelResolution);

  if (key->voxel[0] < regionDim->x && key->voxel[1] < regionDim->y && key->voxel[2] < regionDim->z)
  {
    return true;
  }

// Out of range.
#if 0
  printf("%u Bad key: " KEY_F "\nfrom (%.16f,%.16f,%.16f)\n"
         "  quantisation: (%.16f,%.16f,%.16f)\n"
         "  region: (%.16f,%.16f,%.16f)\n",
         (uint)get_global_id(0), KEY_A(*key), point->x, point->y, point->z,
         point->x -
           (regionCentreCoord(key->region[0], regionDim->x * voxelResolution) - 0.5f * regionDim->x * voxelResolution),
         point->y -
           (regionCentreCoord(key->region[1], regionDim->y * voxelResolution) - 0.5f * regionDim->y * voxelResolution),
         point->z -
           (regionCentreCoord(key->region[2], regionDim->z * voxelResolution) - 0.5f * regionDim->z * voxelResolution),
         regionDim->x * voxelResolution, regionDim->y * voxelResolution, regionDim->z * voxelResolution);
  printf("pointToRegionCoord(%.16f, %d * %.16f = %.16f)\n", point->y, regionDim->y, voxelResolution,
         regionDim->y * voxelResolution);
  printf("pointToRegionVoxel(%.16f - %.16f, ...)\n", point->y,
         regionCentreCoord(key->region[1], regionDim->y * voxelResolution) - 0.5f * (regionDim->y * voxelResolution));
#endif  // #
  return false;
}


inline __device__ float3 voxelCentre(const GpuKey *key, const int3 *regionDim, float voxelResolution)
{
  float3 voxel;

  // printf("voxelCentre(" KEY_F ", [%d %d %d], %f)\n", KEY_A(*key), regionDim->x, regionDim->y, regionDim->z,
  // voxelResolution);

  // Calculation is:
  //  - region centre - region half extents => region min extents.
  //  - add voxel region local coordiate.
  // Using terse code to reduce local variable load.
  voxel.x = regionCentreCoord(key->region[0], regionDim->x * voxelResolution) - 0.5f * regionDim->x * voxelResolution +
            key->voxel[0] * voxelResolution + 0.5f * voxelResolution;
  voxel.y = regionCentreCoord(key->region[1], regionDim->y * voxelResolution) - 0.5f * regionDim->y * voxelResolution +
            key->voxel[1] * voxelResolution + 0.5f * voxelResolution;
  voxel.z = regionCentreCoord(key->region[2], regionDim->z * voxelResolution) - 0.5f * regionDim->z * voxelResolution +
            key->voxel[2] * voxelResolution + 0.5f * voxelResolution;

  return voxel;
}


inline __device__ float getf3(const float3 *v, int index)
{
  return (index == 0) ? v->x : ((index == 1) ? v->y : v->z);
}


inline __device__ int geti3(const int3 *v, int index)
{
  return (index == 0) ? v->x : ((index == 1) ? v->y : v->z);
}

inline __device__ int nextAxis(float timeMax[3], bool forwardWalk)
{
  if (forwardWalk)
  {
    return (timeMax[0] < timeMax[2]) ? ((timeMax[0] < timeMax[1]) ? 0 : 1) : ((timeMax[1] < timeMax[2]) ? 1 : 2);
  }

  return (timeMax[2] < timeMax[0]) ? ((timeMax[2] < timeMax[1]) ? 2 : 1) : ((timeMax[1] < timeMax[0]) ? 1 : 0);
}
#endif  // LINE_WALK_CL

__device__ void WALK_LINE_VOXELS(const GpuKey *startKey, const GpuKey *endKey, const float3 *firstVoxelCentre,
                                 const float3 *startPoint, const float3 *endPoint, const int3 *regionDim,
                                 float voxelResolution, int walkFlags, void *userData)
{
  // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
  float timeMax[3];
  float timeDelta[3];
  float timeLimit[3];
  int step[3] = { 0 };
  float timeCurrent = 0.0f;
  float timeNext = 0.0f;
  bool continueTraversal = true;
  float length = 0;

  const bool forwardWalk = (walkFlags & kLineWalkFlagReverse) == 0;
  const bool reportWalkEndLast = (walkFlags & (kLineWalkFlagForReportEndLast | kLineWalkFlagReverse)) ==
                                 (kLineWalkFlagForReportEndLast | kLineWalkFlagReverse);

  const GpuKey *walkStartKey = (forwardWalk) ? startKey : endKey;
  const GpuKey *walkEndKey = (forwardWalk) ? endKey : startKey;
  const float3 *walkStartPoint = (forwardWalk) ? startPoint : endPoint;
  const float3 *walkEndPoint = (forwardWalk) ? endPoint : startPoint;

  // BUG: Intel OpenCL 2.0 compiler does not effect the commented assignment below. I've had to unrolled it in copyKey()
  // GpuKey currentKey = *walkStartKey;
  GpuKey currentKey;
  copyKey(&currentKey, walkStartKey);

  // printf("Start point : %f %f %f, " KEY_F "\n", walkStartPoint->x, walkStartPoint->y, walkStartPoint->z,
  //        KEY_A(*walkStartKey));
  // printf("End point : %f %f %f\n", walkEndPoint->x, walkEndPoint->y, walkEndPoint->z);
  // printf("currentKey: " KEY_F "\n", KEY_A(currentKey));

  // Compute step direction, increments and maximums along each axis.
  // Things to remember:
  // - start and end keys come precalcualted in the map space.
  // - float3 start/end points are single precision. To deal with this, they are likely in to be in some non-global
  //   frame. This frame is irrelevant here so long as walkStartPoint, walkEndPoint and firstVoxelCentre are in the same
  //   frame.
  {
    // Scoped to try reduce local variable load on local memory.
    float3 direction = *walkEndPoint - *walkStartPoint;
    // Check for degenerate rays: start/end in the same voxel.
    if (fabs(dot(direction, direction)) > 1e-3f)
    {
      length = sqrt(dot(direction, direction));
      direction *= 1.0f / length;
    }
    else
    {
      // Denegerate ray. Set the direction to be walkEndKey - walkStartKey.
      direction = keyDirection(walkStartKey, walkEndKey);
      if (direction.x || direction.y || direction.z)
      {
        direction = normalize(direction);
        length = voxelResolution;  // Ensure a non-zero length.
      }
    }

    // const float3 voxel = voxelCentre(&currentKey, regionDim, voxelResolution);
    // printf("V: %f %f %f\n", voxel.x, voxel.y, voxel.z);
    // printf("C: " KEY_F "\n", KEY_A(currentKey));
    for (unsigned i = 0; i < 3; ++i)
    {
      if (getf3(&direction, i) != 0)
      {
        const float directionAxisInv = 1.0f / getf3(&direction, i);
        step[i] = (getf3(&direction, i) > 0) ? 1 : -1;
        // Time delta is the ray time between voxel boundaries calculated for each axis.
        timeDelta[i] = voxelResolution * fabs(directionAxisInv);
        // Calculate the distance from the origin to the nearest voxel edge for this axis.
        // const float nextVoxelBorder = getf3(&voxel, i) + step[i] * 0.5f * voxelResolution;
        const float nextVoxelBorder = getf3(firstVoxelCentre, i) + step[i] * 0.5f * voxelResolution;
        timeMax[i] = (nextVoxelBorder - getf3(walkStartPoint, i)) * directionAxisInv;
        // Set the distance limit
        // original...
        // timeLimit[i] = fabs((getf3(walkEndPoint, i) - getf3(walkStartPoint, i)) * directionAxisInv);
        // which is equivalent to...
        timeLimit[i] = length;
      }
      else
      {
        timeMax[i] = timeDelta[i] = FLT_MAX;
        timeLimit[i] = 0;
      }
    }
  }

  // printf("\n");
  // for (int i = 0; i < 3; ++i)
  // {
  //   printf("timeMax[%d]: %f timeLimit[%d]: %f timeDelta[%d]: %f\n",
  //          i, timeMax[i], i, timeLimit[i], i, timeDelta[i]);
  // }

  // printf("S: " KEY_F " C: " KEY_F " E: " KEY_F "\n", KEY_A(*walkStartKey), KEY_A(currentKey), KEY_A(*walkEndKey));

  int axis = 0;
  float timeFirst = 0.0f;
  bool limitReached = false;
#ifdef LIMIT_LINE_WALK_ITERATIONS
  int iterations = 0;
  const int iterLimit = 2 * 32768;
#endif  // LIMIT_LINE_WALK_ITERATIONS
  // Flag next voxel as the sample voxel if required (can happen in reverse traversal).
  int voxelMarker = (forwardWalk) ? kLineWalkMarkerStart : kLineWalkMarkerEnd;

  if (!forwardWalk)
  {
    axis = nextAxis(timeMax, forwardWalk);
    timeCurrent = timeNext = timeLimit[axis];
  }

  while (!limitReached && !equalKeys(&currentKey, walkEndKey) && continueTraversal)
  {
#ifdef LIMIT_LINE_WALK_ITERATIONS
    if (iterations++ > iterLimit)
    {
      printf("%u excessive line walk iterations.\n"
             "S: " KEY_F " E: " KEY_F "\n",
             "C: " KEY_F "\n" get_global_id(0), KEY_A(*walkStartKey), KEY_A(*walkEndKey), KEY_A(currentKey));
      break;
    }
#endif  // LIMIT_LINE_WALK_ITERATIONS
    // Select the minimum timeMax as the next axis.
    axis = nextAxis(timeMax, forwardWalk);
    if (forwardWalk)
    {
      timeNext = (limitReached) ? timeLimit[axis] : timeMax[axis];
    }
    else
    {
      timeCurrent = timeLimit[axis] - timeMax[axis];
    }
    // Skip reporting the end voxel if forcing its reporting last.
    if (voxelMarker != kLineWalkMarkerEnd || !reportWalkEndLast)
    {
      continueTraversal =
        VISIT_LINE_VOXEL(&currentKey, voxelMarker, startKey, endKey, voxelResolution, timeCurrent, timeNext, userData);
    }
    else
    {
      timeFirst = timeCurrent;
    }
    voxelMarker = kLineWalkMarkerSegment;
    limitReached = fabs(timeMax[axis]) > timeLimit[axis];
    stepKeyAlongAxis(&currentKey, axis, step[axis], regionDim);
    currentKey.voxel[3] = 0;  // Always clear the clip marker.
    timeMax[axis] += timeDelta[axis];
    if (forwardWalk)
    {
      timeCurrent = timeNext;
    }
    else
    {
      timeNext = timeCurrent;
    }
  }

  // if (limitReached)
  // {
  //   printf("%u limitReached\n", get_global_id(0));
  //   printf("timeMax[%d]: %f timeLimit[%d]: %f timeDelta[%d]: %f\n",
  //          axis, timeMax[axis], axis, timeLimit[axis], axis, timeDelta[axis]);
  // }
  // if (equalKeys(&currentKey, walkEndKey))
  // {
  //   printf("%u currentKey == walkEndKey\n", get_global_id(0));
  // }
  // if (!continueTraversal)
  // {
  //   printf("%u continueTraversal = false\n", get_global_id(0));
  // }

  // Walk end point. Make sure we don't report it when start/end voxles are the same and reportWalkEndLast is set.
  if (continueTraversal && (!equalKeys(walkEndKey, walkStartKey) || !reportWalkEndLast))
  {
    if (forwardWalk)
    {
      timeNext = length;
    }
    VISIT_LINE_VOXEL(walkEndKey, (forwardWalk) ? kLineWalkMarkerEnd : kLineWalkMarkerStart, startKey, endKey,
                     voxelResolution, timeCurrent, timeNext, userData);
  }

  // For reverse iteration, report the start voxel last.
  if (reportWalkEndLast)
  {
    VISIT_LINE_VOXEL(walkStartKey, kLineWalkMarkerEnd, startKey, endKey, voxelResolution, timeFirst, length, userData);
  }
}
