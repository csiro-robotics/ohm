// Copyright (c) 2022
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_LINEWALKCOMPUTE_H
#define OHM_LINEWALKCOMPUTE_H

// Note: this header is included in GPU code.
// When included from CPU code, note that all functions are static and can be either wrapped in a class or private
// translation unit.

// Additional code to be defined before including this file:
// - WalkContext for use with the functions below. E.g., ohm::OccupancyMap
// - WalkKey type: either ohm::Key or ohm::GpuKey
// - WalkKey functions:
//  - void walkKeyDiff(WalkContext *context, int[3], WalkKey *, WalkKey *)
//  - void walkStepKey(WalkContext *context, WalkKey *, int axis, int step_dir)
//  - bool walkVisitVoxel(WalkContext *context, const WalkKey *voxel_key, WalkReal enter_time, WalkReal exit_time, const
//                        int steps_remaining[3])

#if GPUTIL_DEVICE
#define WALK_FUNC __device__

// Define GPU type aliases
typedef GpuKey WalkKey;
typedef float3 WalkVec3;
typedef float WalkReal;

__device__ inline float walkInfinity()
{
  return INFINITY;
}

__device__ inline float walkHalf()
{
  return 0.5f;
}

__device__ inline bool walkEqualKeys(const WalkKey *key_a, const WalkKey *key_b)
{
  // From GpuKey.h
  return equalKeys(key_a, key_b);
}

#if GPUTIL_DEVICE == GPUTIL_OPENCL
#define walkCopyKey copyKey
#else   // GPUTIL_DEVICE == GPUTIL_OPENCL
__device__ inline void walkCopyKey(WalkKey *dst, const WalkKey *src)
{
  // From GpuKey.h
  copyKey(dst, src);
}
#endif  // GPUTIL_DEVICE == GPUTIL_OPENCL

#else  // GPUTIL_DEVICE
#define WALK_FUNC

// Define CPU types
using WalkKey = ohm::Key;
using WalkVec3 = glm::dvec3;
using WalkReal = double;

WALK_FUNC inline WalkReal walkInfinity()
{
  return std::numeric_limits<WalkReal>::infinity();
}


// Constants to address OpenCL floating point literal strictness.
WALK_FUNC inline WalkReal walkHalf()
{
  return 0.5;
}

WALK_FUNC inline bool walkEqualKeys(const WalkKey *key_a, const WalkKey *key_b)
{
  return *key_a == *key_b;
}

WALK_FUNC inline void walkCopyKey(WalkKey *dst, const WalkKey *src)
{
  *dst = *src;
}

#endif  // GPUTIL_DEVICE

struct LineWalkRay
{
  WalkVec3 origin;
  WalkVec3 direction;
  WalkVec3 direction_inverse;
  int sign[3];
  WalkReal initial_exit_time[3];
  WalkReal step_delta[3];
  WalkReal length;
};

WALK_FUNC inline int walkSignToStep(int sign)
{
  // Derive the step direction from the ray->sign. The ray->sign values are 0 for positive step direction, 1 for
  // a negative step direction. Thus we can resolve the step direction as:
  // -2 * sign + 1
  //
  // When sign is 0:
  //  -2 * 0 + 1 = 1
  // When sign is 1:
  //  -2 * 1 + 1 = -1
  return -2 * sign + 1;
}

/// Calculate the times at which the ray will exit a voxel wall along each axis.
/// Note: @p exit_time must have space for three elements to be written back to it.
WALK_FUNC inline void walkCalculateVoxelWallExit(const LineWalkRay *ray, const WalkVec3 voxel_min,
                                                 const WalkVec3 voxel_max, WalkReal *exit_time, WalkReal length_epsilon)
{
  // Based on:
  // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
  const WalkVec3 *bounds[2] = { &voxel_min, &voxel_max };
  exit_time[0] = (bounds[1 - ray->sign[0]]->x - ray->origin.x) * ray->direction_inverse.x;
  exit_time[1] = (bounds[1 - ray->sign[1]]->y - ray->origin.y) * ray->direction_inverse.y;
  exit_time[2] = (bounds[1 - ray->sign[2]]->z - ray->origin.z) * ray->direction_inverse.z;
}

WALK_FUNC inline void walkInitRay(LineWalkRay *ray, const WalkVec3 start, const WalkVec3 end,
                                  const WalkVec3 &start_voxel_centre, const WalkVec3 voxel_resolution,
                                  WalkReal length_epsilon)
{
  ray->origin = start;
  ray->direction = end - start;
  ray->length =
    ray->direction.x * ray->direction.x + ray->direction.y * ray->direction.y + ray->direction.z * ray->direction.z;
  // OpenCL 3.0 does not like WalkReal precision literals without WalkReal support enabled, so we have to use floats
  // and cast to our WalkReal if that's what we are using.
  ray->length = (ray->length > length_epsilon) ? sqrt(ray->length) : 0;

  // Resolve the direction before we potentially divide by zero as we can for very small rays.
  // This at leasts sets the ray direction correctly.
  ray->sign[0] = ray->direction.x < 0;
  ray->sign[1] = ray->direction.y < 0;
  ray->sign[2] = ray->direction.z < 0;

  ray->direction.x /= ray->length;
  ray->direction.y /= ray->length;
  ray->direction.z /= ray->length;

  ray->direction_inverse.x = (ray->length > 0) ? 1 / ray->direction.x : 0;
  ray->direction_inverse.y = (ray->length > 0) ? 1 / ray->direction.y : 0;
  ray->direction_inverse.z = (ray->length > 0) ? 1 / ray->direction.z : 0;

  // Calculate how much the exit time changes along an axis each time we leave an axis.
  // We start with the initial voxel to also calculate start_voxel_centre, but it could be arbitrary for the delta
  // calculation. Then we calculate the exit time for the next voxel along each axis.
  WalkVec3 voxel_min = start_voxel_centre;
  WalkVec3 voxel_max = start_voxel_centre;

  voxel_min -= walkHalf() * voxel_resolution;
  voxel_max += walkHalf() * voxel_resolution;

  walkCalculateVoxelWallExit(ray, voxel_min, voxel_max, ray->initial_exit_time, length_epsilon);

  // Move the voxel along on each axis.
  WalkVec3 voxel_shift;
  voxel_shift.x = walkSignToStep(ray->sign[0]) * voxel_resolution.x;
  voxel_shift.y = walkSignToStep(ray->sign[1]) * voxel_resolution.y;
  voxel_shift.z = walkSignToStep(ray->sign[2]) * voxel_resolution.z;

  voxel_min += voxel_shift;
  voxel_max += voxel_shift;

  // Calculate the time to exit the next voxel wall along each axis.
  walkCalculateVoxelWallExit(ray, voxel_min, voxel_max, ray->step_delta, length_epsilon);

  // The difference between them is the step delta. This is invariant for this ray.
  if (ray->step_delta[0] != walkInfinity())
  {
    ray->step_delta[0] -= ray->initial_exit_time[0];
  }
  if (ray->step_delta[1] != walkInfinity())
  {
    ray->step_delta[1] -= ray->initial_exit_time[1];
  }
  if (ray->step_delta[2] != walkInfinity())
  {
    ray->step_delta[2] -= ray->initial_exit_time[2];
  }
}

struct WalkSteps
{
  WalkReal time_next[3];
  WalkReal initial_delta[3];
  WalkReal step_delta[3];
  int sign[3];
  WalkReal length;
};

WALK_FUNC inline void walkCalculateSteps(WalkSteps *walk_steps, const WalkVec3 start_point, const WalkVec3 end_point,
                                         const WalkVec3 start_voxel_centre, const WalkVec3 voxel_resolution,
                                         WalkReal length_epsilon)
{
  LineWalkRay ray;
  walkInitRay(&ray, start_point, end_point, start_voxel_centre, voxel_resolution, length_epsilon);

  walk_steps->initial_delta[0] = walk_steps->time_next[0] = ray.initial_exit_time[0];
  walk_steps->initial_delta[1] = walk_steps->time_next[1] = ray.initial_exit_time[1];
  walk_steps->initial_delta[2] = walk_steps->time_next[2] = ray.initial_exit_time[2];

  walk_steps->step_delta[0] = ray.step_delta[0];
  walk_steps->step_delta[1] = ray.step_delta[1];
  walk_steps->step_delta[2] = ray.step_delta[2];

  walk_steps->sign[0] = ray.sign[0];
  walk_steps->sign[1] = ray.sign[1];
  walk_steps->sign[2] = ray.sign[2];

  walk_steps->length = ray.length;
}

WALK_FUNC inline int walkSelectNextAxis(const WalkReal *time_next)
{
  // Select next axis based on the earliest next time. An expired axis will have infinity for next time.
  int axis = 0;
  axis = (time_next[axis] < time_next[1]) ? axis : 1;
  axis = (time_next[axis] < time_next[2]) ? axis : 2;
  return axis;
}

WALK_FUNC inline int walkStepDir(const int sign)
{
  // Derive the step direction from the ray->sign. The ray->sign values are 0 for positive step direction, 1 for
  // a negative step direction. Thus we can resolve the step direction as:
  // -2 * sign + 1
  //
  // When sign is 0:
  //  -2 * 0 + 1 = 1
  // When sign is 1:
  //  -2 * 1 + 1 = -1
  return -2 * sign + 1;
}

/// A templatised, voxel based line walking algorithm. Voxels are accurately traversed from @p startPoint to
/// @p endPoint, invoking @p walkFunc for each traversed voxel.
///
/// The @p walkFunc is simply a callable object which accepts a @p KEY argument. Keys are provided in order of
/// traversal.
///
/// The templatisation requires @p funcs to provide a set of key manipulation utility functions. Specifically,
/// the @p KEYFUNCS type must have the following signature:
/// @code
/// struct KeyFuncs
/// {
///   // Query the voxel resolution along a particular axis. Axis may be { 0, 1, 2 } corresponding to XYZ.
///   WalkReal voxelResolution(int axis) const;
///   // Convert from pt to it's voxel key. The result may be null/invalid
///   KEY voxelKey(const glm::dvec3 &pt) const;
///   // Check if key is a null or invalid key.
///   bool isNull(const KEY &key) const;
///   // Convert from key to the centre of the corresponding voxel.
///   glm::dvec3 voxelCentre(const KEY &key) const;
///   // Move the key by one voxel. The axis may be {0, 1, 2} correlating the XYZ axes respectively.
///   // The step will be 1 or -1, indicating the direction of the step.
///   void stepKey(KEY &key, int axis, int step) const;
///   // Calculate the voxel difference between two voxel keys : `key_a - key_b`.
///   glm::ivec3 keyDiff(const KEY &key_a, const KEY &key_b);
/// };
/// @endcode
///
/// Based on J. Amanatides and A. Woo, "A fast voxel traversal algorithm for raytracing," 1987.
///
/// @param walk_func The callable object to invoke for each traversed voxel key.
/// @param start_point The start of the line in 3D space.
/// @param end_point The end of the line in 3D space.
/// @param include_end_point Should be @c true if @p walkFunc should be called for the voxel containing
///     @c endPoint, when it does not lie in the same voxel as @p startPoint.
/// @param funcs Key helper functions object.
/// @return The number of voxels traversed. This includes @p endPoint when @p includeEndPoint is true.
WALK_FUNC inline unsigned walkSegmentKeys(WalkContext *context, const WalkVec3 start_point, const WalkVec3 end_point,
                                          const WalkKey *start_point_key, const WalkKey *end_point_key,
                                          const WalkVec3 start_voxel_centre, const WalkVec3 voxel_resolution,
                                          bool include_end_point,
                                          WalkReal length_epsilon = 1e-6)  // NOLINT(readability-magic-numbers)
{
  WalkSteps steps;
  walkCalculateSteps(&steps, start_point, end_point, start_voxel_centre, voxel_resolution, length_epsilon);

  int steps_remaining[3] = { 0, 0, 0 };
  int stepped[3] = { 0, 0, 0 };

  walkKeyDiff(context, steps_remaining, end_point_key, start_point_key);

  WalkKey current_key;
  walkCopyKey(&current_key, start_point_key);

  WalkReal last_time = 0;
  unsigned axis = 0;
  unsigned limit_flags = 0;
  unsigned voxel_count = 0;
  bool user_exit = false;

  // Initialise limit flags to mark which axes won't be stepped. Bits 0, 1, 2 map to axis X, Y, Z respectively.
  limit_flags |= !!(steps_remaining[0] == 0) * (1u << 0u);
  limit_flags |= !!(steps_remaining[1] == 0) * (1u << 1u);
  limit_flags |= !!(steps_remaining[2] == 0) * (1u << 2u);

  for (int i = 0; i < 3; ++i)
  {
    steps.time_next[i] = (steps_remaining[i]) ? steps.initial_delta[i] : walkInfinity();
  }

  // Select next axis based on the earliest next time.
  axis = walkSelectNextAxis(steps.time_next);

  while (!user_exit && limit_flags < 7u && !walkEqualKeys(&current_key, end_point_key))
  {
    // Visit the current voxel.
    user_exit = !walkVisitVoxel(context, &current_key, last_time, steps.time_next[axis], steps_remaining);
    last_time = steps.time_next[axis];
    ++voxel_count;

    // Step on from the current voxel.
    const int step_dir = walkStepDir(steps.sign[axis]);
    walkStepKey(context, &current_key, axis, step_dir);
    steps_remaining[axis] -= step_dir;
    stepped[axis] += step_dir;
    steps.time_next[axis] = (steps_remaining[axis]) ?
                              steps.initial_delta[axis] + steps.step_delta[axis] * abs(stepped[axis]) :
                              // steps.time_next[axis] + steps.step_delta[axis] :
                              walkInfinity();
    limit_flags |= !!(steps_remaining[axis] == 0) * (1u << axis);

    // Choose the next step axis.
    axis = walkSelectNextAxis(steps.time_next);
  }

  // Touch the last voxel.
  if (!user_exit && include_end_point)
  {
    walkVisitVoxel(context, end_point_key, last_time, steps.length, steps_remaining);
    ++voxel_count;
  }

  return voxel_count;
}

#endif  // OHM_LINEWALKCOMPUTE_H
