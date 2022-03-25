// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#ifndef WALK_NAME
#error WALK_NAME must be used to define the voxel walking function.
#endif  // WALK_NAME

#ifndef WALK_VISIT_VOXEL
#error WALK_VISIT_VOXEL must be used to define the voxel visiting function
#endif  // WALK_VISIT_VOXEL

//------------------------------------------------------------------------------
/// Note on building this code.
/// This code may be used by multiple OpenCL kernels, but has to support the same for CUDA compilation. This is slightly
/// tricky because OpenCL has entirely separate compilation units, while CUDA does not. The preprocessor is used to
/// workaround this by aliasing the key function names. Before including, the following must be defined:
///
/// @code{.c}
/// #define WALK_NAME        MyWalkName
/// #define WALK_VISIT_VOXEL visitVoxelMyWalkName
/// @endcode
///
/// The first is the main function name and the second is the function to call for each voxel visited.
///
/// This function is called for each voxel traversed by the line walking function.
///
/// Note the @c WALK_VISIT_VOXEL() function is not implemented in this source file, and must be implemented by the
/// source including this file.
///
/// The @p walk_flags may be used to modify the algorithm behaviour and semantics as follows:
///
/// - @c kLineWalkFlagReverse reverse walk. The algorithm iterates from @p endVoxel to @p startVoxel instead.
/// - @c kLineWalkFlagForReportEndLast force reporting the @p endVoxel last in a reverse walk.
///
/// @param voxel_key The key for the voxel currently being traversed. This voxel is on the line.
/// @param walk_start_key The first voxel visited by the walk.
/// @param walk_end_key The last voxel to be visited by the walk.
/// @param voxel_marker Set to @p kLineWalkMarkerStart (1) if @p voxelKey is the @c start_key or
///   @c kLineWalkMarkerEnd (2) if it is the @c end_key . Zero otherwise.
/// @param enter_range How far from the origin has been traversed before entering @p voxelKey . Is adapted for
///   @c kLineWalkFlagReverse .
/// @param exit_range How far from the origin has been traversed when exiting @p voxelKey . Is adapted for
///   @c kLineWalkFlagReverse .
/// @param stepped The number of voxel steps which have been made along each axis.
/// @param user_data A pointer to the user data given to @c walkVoxels()
/// @return True to continue traversing the line, false to abort traversal.
/// __device__ bool WALK_VISIT_VOXEL(const GpuKey *voxel_key, const GpuKey *walk_start_key, const GpuKey *walk_end_key,
///                                  int voxel_marker, float enter_range, float exit_range, const int stepped[3],
///                                  void *user_data);
//------------------------------------------------------------------------------
#include "GpuKey.h"
#include "LineWalkMarkers.cl"

// Define the context based function names

#define WALK_BUILD_FUNC_NAME(A, B)  WALK_BUILD_FUNC_NAME_(A, B)
#define WALK_BUILD_FUNC_NAME_(A, B) A##B

#define walkVoxels     WALK_BUILD_FUNC_NAME(walkVoxels, WALK_NAME)
#define walkVisitVoxel WALK_BUILD_FUNC_NAME(walkVisit, WALK_NAME)

typedef struct WalkContext
{
  int region_dimensions[3];
  bool suppress_next_visit_call;
  void *user_data;
  // Values used for deferring calling the sample voxel in a reverse trace.
  float suppressed_enter_range;
  float suppressed_exit_range;
} WalkContext;

/// Calculate the @p GpuKey for @p point local to the region's minimum extents corner.
/// @param[out] key The output key.
/// @param point The coordinate to calculate the @p key for in region local coordinates.
/// @paramregion_dimensions Defines the size of a region in voxels. Used to update the @p GpuKey.
/// @param voxel_resolution Size of a voxel from one face to another.
/// @return True if @c point lies in the region, false otherwise.
__device__ bool coordToKey(GpuKey *key, const float3 *point, const int3 *region_dimensions, float voxel_resolution);

/// Calculates the centre of the voxel defined by @p key (global space).
/// @param key The key marking the voxel of interest.
/// @param region_dimensions Defines the size of a region in voxels. Used to update the @p GpuKey.
/// @param voxel_resolution Size of a voxel from one face to another.
/// @return The centre of the voxel defined by @p key.
inline __device__ float3 voxelCentre(const GpuKey *key, const int3 *region_dimensions, float voxel_resolution);

/// Get the @p index component of @c float3 @p v
inline __device__ float getf3(const float3 *v, int index);
/// Get the @p index component of @c int3 @p v
inline __device__ int geti3(const int3 *v, int index);

/// Line walking function for use by kernels.
/// The algorithm walks the voxels from @p start_key to @p end_key. The line segment is defined relative to the centre
/// of the @p startkey voxel with line points @p start_point and @p end_point respectively.
///
/// @c WALK_NAME() is invoked for each voxel traversed.
///
/// Based on J. Amanatides and A. Woo, "A fast voxel traversal algorithm for raytracing," 1987.
///
/// @param context User context data.
/// @param start_key The key for the voxel containing @p start_point.
/// @param end_key The key for the voxel containing @p end_point.
/// @param start_point The start point of the line segment to traverse, relative to the centre of the
///   start voxel (identified by start_key). That is the origin is the centre of the start_key voxel.
/// @param end_point The end point of the line segment to traverse, relative to the centre of the
///   start voxel (identified by start_key). That is the origin is the centre of the start_key voxel.
/// @param start_voxel_centre Coordinate of the centre of the first voxel to walk, in the same frame as @c start_point
///   and @c end_point. Normally this is the coordinate of the start voxel, but when @p kLineWalkFlagReverse is set,
///   this must be the coordinate of the end voxel.
/// @param region_dimensions Defines the size of a region in voxels. Used to update the @p GpuKey.
/// @param voxel_resolution Size of a voxel from one face to another.
/// @param walk_flags Flags affecting the algorithm behaviour. See @c LineWalkFlag .
/// @param userData User pointer passed to @c walkLineVoxel().
__device__ void walkVoxels(WalkContext *context, const GpuKey *start_key, const GpuKey *end_key,
                           const float3 start_point, const float3 end_point, const float3 *start_voxel_centre,
                           const int3 region_dimensions, float voxel_resolution, int walk_flags);

// Psuedo header guard to prevent symbol duplication.
#ifndef LINE_WALK_CL
#define LINE_WALK_CL


inline __device__ bool coordToKey(GpuKey *key, const float3 *point, const int3 *region_dimensions,
                                  float voxel_resolution)
{
  // Quantise.
  key->region[0] = pointToRegionCoord(point->x, region_dimensions->x * voxel_resolution);
  key->region[1] = pointToRegionCoord(point->y, region_dimensions->y * voxel_resolution);
  key->region[2] = pointToRegionCoord(point->z, region_dimensions->z * voxel_resolution);

  // Localise.
  // Trying to minimise local variables for GPU.
  // The value we pass to pointToRegionVoxel is logically:
  //    point - regionCentre - regionHalfExtents
  // or
  //    point - regionMin
  // which equates to the point in region local coordinates.
  // printf("p.x(%f) = %f - (%f - %f)   : regionMin: %f\n",
  //        point->x - (regionCentreCoord(key->region[0], region_dimensions->x * voxel_resolution) - 0.5f *
  //        region_dimensions->x * voxel_resolution), point->x, regionCentreCoord(key->region[0], region_dimensions->x *
  //        voxel_resolution), 0.5f * region_dimensions->x * voxel_resolution, regionCentreCoord(key->region[0],
  //        region_dimensions->x * voxel_resolution) - 0.5f * region_dimensions->x * voxel_resolution);
  key->voxel[0] =
    pointToRegionVoxel(point->x - (regionCentreCoord(key->region[0], region_dimensions->x * voxel_resolution) -
                                   0.5f * region_dimensions->x * voxel_resolution),
                       voxel_resolution, region_dimensions->x * voxel_resolution);
  key->voxel[1] =
    pointToRegionVoxel(point->y - (regionCentreCoord(key->region[1], region_dimensions->y * voxel_resolution) -
                                   0.5f * region_dimensions->y * voxel_resolution),
                       voxel_resolution, region_dimensions->y * voxel_resolution);
  key->voxel[2] =
    pointToRegionVoxel(point->z - (regionCentreCoord(key->region[2], region_dimensions->z * voxel_resolution) -
                                   0.5f * region_dimensions->z * voxel_resolution),
                       voxel_resolution, region_dimensions->z * voxel_resolution);

  if (key->voxel[0] < region_dimensions->x && key->voxel[1] < region_dimensions->y &&
      key->voxel[2] < region_dimensions->z)
  {
    return true;
  }

// Out of range.
#if 0
  printf("%u Bad key: " KEY_F "\nfrom (%.16f,%.16f,%.16f)\n"
         "  quantisation: (%.16f,%.16f,%.16f)\n"
         "  region: (%.16f,%.16f,%.16f)\n",
         (uint)get_global_id(0), KEY_A(*key), point->x, point->y, point->z,
         point->x - (regionCentreCoord(key->region[0], region_dimensions->x * voxel_resolution) -
                     0.5f * region_dimensions->x * voxel_resolution),
         point->y - (regionCentreCoord(key->region[1], region_dimensions->y * voxel_resolution) -
                     0.5f * region_dimensions->y * voxel_resolution),
         point->z - (regionCentreCoord(key->region[2], region_dimensions->z * voxel_resolution) -
                     0.5f * region_dimensions->z * voxel_resolution),
         region_dimensions->x * voxel_resolution, region_dimensions->y * voxel_resolution,
         region_dimensions->z * voxel_resolution);
  printf("pointToRegionCoord(%.16f, %d * %.16f = %.16f)\n", point->y, region_dimensions->y, voxel_resolution,
         region_dimensions->y * voxel_resolution);
  printf("pointToRegionVoxel(%.16f - %.16f, ...)\n", point->y,
         regionCentreCoord(key->region[1], region_dimensions->y * voxel_resolution) -
           0.5f * (region_dimensions->y * voxel_resolution));
#endif  // #
  return false;
}


inline __device__ float3 voxelCentre(const GpuKey *key, const int3 *region_dimensions, float voxel_resolution)
{
  float3 voxel;

  // printf("voxelCentre(" KEY_F ", [%d %d %d], %f)\n", KEY_A(*key), region_dimensions->x, region_dimensions->y,
  // region_dimensions->z, voxel_resolution);

  // Calculation is:
  //  - region centre - region half extents => region min extents.
  //  - add voxel region local coordiate.
  // Using terse code to reduce local variable load.
  voxel.x = regionCentreCoord(key->region[0], region_dimensions->x * voxel_resolution) -
            0.5f * region_dimensions->x * voxel_resolution + key->voxel[0] * voxel_resolution + 0.5f * voxel_resolution;
  voxel.y = regionCentreCoord(key->region[1], region_dimensions->y * voxel_resolution) -
            0.5f * region_dimensions->y * voxel_resolution + key->voxel[1] * voxel_resolution + 0.5f * voxel_resolution;
  voxel.z = regionCentreCoord(key->region[2], region_dimensions->z * voxel_resolution) -
            0.5f * region_dimensions->z * voxel_resolution + key->voxel[2] * voxel_resolution + 0.5f * voxel_resolution;

  return voxel;
}

#endif  // LINE_WALK_CL

inline __device__ void walkKeyDiff(WalkContext *context, int *diff, const GpuKey *key_a, const GpuKey *key_b)
{
  const int3 diff_iv3 =
    keyDiff(key_a, key_b,
            make_int3(context->region_dimensions[0], context->region_dimensions[1], context->region_dimensions[2]));
  diff[0] = diff_iv3.x;
  diff[1] = diff_iv3.y;
  diff[2] = diff_iv3.z;
}

inline __device__ void walkStepKey(WalkContext *context, GpuKey *key, int axis, int step_dir)
{
  stepKeyAlongAxis(
    key, axis, step_dir,
    make_int3(context->region_dimensions[0], context->region_dimensions[1], context->region_dimensions[2]));
  // Do not propagate the clipping bit.
  key->voxel[3] = 0;
}

inline __device__ bool walkVisitVoxel(WalkContext *context, const GpuKey *voxel_key, const GpuKey *start_key,
                                      const GpuKey *end_key, int voxel_marker, float enter_range, float exit_range,
                                      const int *stepped)
{
  // return WALK_VISIT_VOXEL(voxel_key, start_key, end_key, voxel_marker, enter_range, exit_range, stepped,
  //                         context->user_data);
  if (context->suppress_next_visit_call)
  {
    context->suppress_next_visit_call = false;
    context->suppressed_enter_range = enter_range;
    context->suppressed_exit_range = exit_range;
    return true;
  }
  return WALK_VISIT_VOXEL(voxel_key, start_key, end_key, voxel_marker, enter_range, exit_range, stepped,
                          context->user_data);
}

// Must be included after ther visit function is defined.
#include "LineWalkCompute.h"

__device__ void walkVoxels(const GpuKey *start_key, const GpuKey *end_key, const float3 start_point,
                           const float3 end_point, const int3 region_dimensions, float voxel_resolution, int walk_flags,
                           void *user_data)
{
  WalkContext context;
  context.region_dimensions[0] = region_dimensions.x;
  context.region_dimensions[1] = region_dimensions.y;
  context.region_dimensions[2] = region_dimensions.z;
  context.suppress_next_visit_call = false;
  context.user_data = user_data;
  context.suppressed_enter_range = 0;
  context.suppressed_exit_range = 0;

  const float3 voxel_resolution_3 = make_float3(voxel_resolution, voxel_resolution, voxel_resolution);
  if ((walk_flags & kLineWalkFlagReverse) == 0)
  {
    // We need to calculate the start voxel centre in the right coordinate space. All coordinates are relative to the
    // end voxel centre.
    // 1. Calculate the voxel step from endKey to startKey.
    // 2. Scale results by voxelResolution.
    const int3 voxel_diff = keyDiff(start_key, end_key, region_dimensions);
    const float3 start_voxel_centre =
      make_float3(voxel_diff.x * voxel_resolution, voxel_diff.y * voxel_resolution, voxel_diff.z * voxel_resolution);
    walkLineVoxels(&context, start_point, end_point, start_key, end_key, start_voxel_centre, voxel_resolution_3, true);
  }
  else
  {
    const bool defer_sample = (walk_flags & kLineWalkFlagForReportEndLast);  // && end_key->voxel[3] == 0;
    context.suppress_next_visit_call = defer_sample;
    const float3 end_voxel_centre = make_float3(0, 0, 0);
    walkLineVoxels(&context, end_point, start_point, end_key, start_key, end_voxel_centre, voxel_resolution_3, true);
    if (defer_sample)
    {
      const int stepped[3] = { 0, 0, 0 };
      WALK_VISIT_VOXEL(end_key, start_key, end_key, kLineWalkMarkerStart, context.suppressed_enter_range,
                       context.suppressed_exit_range, stepped, context.user_data);
    }
  }
}

#undef walkVisitVoxel
#undef walkVoxels
#undef WALK_NAME
#undef WALK_VISIT_VOXEL
