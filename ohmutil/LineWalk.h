// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_LINEWALK_H
#define OHMUTIL_LINEWALK_H

#include <glm/glm.hpp>

#include <array>
#include <cassert>
#include <functional>

namespace ohm
{
class Key;

namespace detail
{
struct LineWalkRay
{
  glm::dvec3 origin;
  glm::dvec3 direction;
  glm::dvec3 direction_inverse;
  int sign[3];
  double initial_exit_time[3];
  double step_delta[3];
  double length;
};

inline int lineWalkSignToStep(int sign)
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
inline void calculateVoxelWallExit(const LineWalkRay *ray, const glm::dvec3 &voxel_min, const glm::dvec3 &voxel_max,
                                   double *exit_time, double length_epsilon)
{
  // Based on:
  // https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection
  const glm::dvec3 *bounds[2] = { &voxel_min, &voxel_max };
  exit_time[0] = (bounds[1 - ray->sign[0]]->x - ray->origin.x) * ray->direction_inverse.x;
  exit_time[1] = (bounds[1 - ray->sign[1]]->y - ray->origin.y) * ray->direction_inverse.y;
  exit_time[2] = (bounds[1 - ray->sign[2]]->z - ray->origin.z) * ray->direction_inverse.z;
}

inline void initLineWalkRay(LineWalkRay *ray, const glm::dvec3 &start, const glm::dvec3 &end,
                            const glm::dvec3 &start_voxel_centre, const glm::dvec3 &voxel_resolution,
                            double length_epsilon)
{
  ray->origin = start;
  ray->direction = end - start;
  ray->length =
    ray->direction.x * ray->direction.x + ray->direction.y * ray->direction.y + ray->direction.z * ray->direction.z;
  // OpenCL 3.0 does not like double precision literals without double support enabled, so we have to use floats
  // and cast to our double if that's what we are using.
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
  glm::dvec3 voxel_min = start_voxel_centre;
  glm::dvec3 voxel_max = start_voxel_centre;

  voxel_min -= 0.5 * voxel_resolution;
  voxel_max += 0.5 * voxel_resolution;

  calculateVoxelWallExit(ray, voxel_min, voxel_max, ray->initial_exit_time, length_epsilon);

  // Move the voxel along on each axis.
  const double voxel_shift[3] = { lineWalkSignToStep(ray->sign[0]) * voxel_resolution[0],
                                  lineWalkSignToStep(ray->sign[1]) * voxel_resolution[1],
                                  lineWalkSignToStep(ray->sign[2]) * voxel_resolution[2] };

  voxel_min.x += voxel_shift[0];
  voxel_min.y += voxel_shift[1];
  voxel_min.z += voxel_shift[2];
  voxel_max.x += voxel_shift[0];
  voxel_max.y += voxel_shift[1];
  voxel_max.z += voxel_shift[2];

  // Calculate the time to exit the next voxel wall along each axis.
  calculateVoxelWallExit(ray, voxel_min, voxel_max, ray->step_delta, length_epsilon);

  // The difference between them is the step delta. This is invariant for this ray.
  if (ray->step_delta[0] != std::numeric_limits<double>::infinity())
  {
    ray->step_delta[0] -= ray->initial_exit_time[0];
  }
  if (ray->step_delta[1] != std::numeric_limits<double>::infinity())
  {
    ray->step_delta[1] -= ray->initial_exit_time[1];
  }
  if (ray->step_delta[2] != std::numeric_limits<double>::infinity())
  {
    ray->step_delta[2] -= ray->initial_exit_time[2];
  }
}

struct WalkSteps
{
  double time_next[3];
  double initial_delta[3];
  double step_delta[3];
  int sign[3];
  double length;
};

inline void calculateWalkSteps(WalkSteps *walk_steps, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                               const glm::dvec3 &start_voxel_centre, const glm::dvec3 &voxel_resolution,
                               double length_epsilon)
{
  LineWalkRay ray;
  initLineWalkRay(&ray, start_point, end_point, start_voxel_centre, voxel_resolution, length_epsilon);

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

inline int nextAxis(const double *time_next)
{
  // Select next axis based on the earliest next time. An expired axis will have infinity for next time.
  int axis = 0;
  axis = (time_next[axis] < time_next[1]) ? axis : 1;
  axis = (time_next[axis] < time_next[2]) ? axis : 2;
  return axis;
}

inline int stepDir(const int sign)
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
}  // namespace detail


/// Function signature for the visit function called from @c walkSegmentKeys().
/// @param key The key of the current voxel being visited.
/// @param enter_range Range at which the voxel is entered. detail required
/// @param exit_range Range at which the voxel is entered. detail required
/// @param steps_remaining The number of voxels steps remaining along each axis.
/// @return True to keep walking the voxels along the ray, false to abort walking the ray.
using WalkSegmentFunc = std::function<bool(const Key &, double, double, const glm::ivec3 &)>;

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
///   double voxelResolution(int axis) const;
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
template <typename KEY, typename KEYFUNCS>
size_t walkSegmentKeys(WalkSegmentFunc walk_func, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                       bool include_end_point, const KEYFUNCS &funcs,
                       double length_epsilon = 1e-6)  // NOLINT(readability-magic-numbers)
{
  const KEY start_point_key = funcs.voxelKey(start_point);
  const KEY end_point_key = funcs.voxelKey(end_point);

  if (funcs.isNull(start_point_key) || funcs.isNull(end_point_key))
  {
    return 0;
  }

  const glm::dvec3 voxel_resolution(funcs.voxelResolution(0), funcs.voxelResolution(1), funcs.voxelResolution(2));
  const glm::dvec3 start_voxel_centre = funcs.voxelCentre(start_point_key);

  detail::WalkSteps steps;
  detail::calculateWalkSteps(&steps, start_point, end_point, start_voxel_centre, voxel_resolution, length_epsilon);
  glm::ivec3 steps_remaining = funcs.keyDiff(end_point_key, start_point_key);
  glm::ivec3 stepped(0);

  KEY current_key = start_point_key;
  double last_time = 0;
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
    steps.time_next[i] = (steps_remaining[i]) ? steps.initial_delta[i] : std::numeric_limits<double>::infinity();
  }

  // Select next axis based on the earliest next time.
  axis = detail::nextAxis(steps.time_next);

  while (!user_exit && limit_flags < 7u && current_key != end_point_key)
  {
    // Visit the current voxel.
    user_exit = !walk_func(current_key, last_time, steps.time_next[axis], steps_remaining);
    last_time = steps.time_next[axis];
    ++voxel_count;

    // Step on from the current voxel.
    const int step_dir = detail::stepDir(steps.sign[axis]);
    const KEY previous_key = current_key;
    funcs.stepKey(current_key, axis, step_dir);
    steps_remaining[axis] -= step_dir;
    stepped[axis] += step_dir;
    steps.time_next[axis] = (steps_remaining[axis]) ?
                              steps.initial_delta[axis] + steps.step_delta[axis] * abs(stepped[axis]) :
                              // steps.time_next[axis] + steps.step_delta[axis] :
                              std::numeric_limits<double>::infinity();
    limit_flags |= !!(steps_remaining[axis] == 0) * (1u << axis);

    // Choose the next step axis.
    axis = detail::nextAxis(steps.time_next);
  }

  // Touch the last voxel.
  if (!user_exit && include_end_point)
  {
    walk_func(end_point_key, last_time, steps.length, steps_remaining);
    ++voxel_count;
  }

  assert(voxel_count || !include_end_point);
  return voxel_count;
}


/// A @c walkSegmentKeys() overload which passes @p includeEndPoint as @c true.
/// @param walk_func The callable object to invoke for each traversed voxel key.
/// @param start_point The start of the line in 3D space.
/// @param end_point The end of the line in 3D space.
/// @param funcs Key helper functions object.
/// @return The number of voxels traversed. This includes @p endPoint.
template <typename KEY, typename KEYFUNCS>
size_t walkSegmentKeys(WalkSegmentFunc walk_func, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                       const KEYFUNCS &funcs)
{
  return walkSegmentKeys(walk_func, start_point, end_point, true, funcs);
}

/// A @c walkSegmentKeys() overload which uses a default passes constructed @p KEYFUNCS object and sets
/// @p includeEndPoint @c true.
/// @param walk_func The callable object to invoke for each traversed voxel key.
/// @param start_point The start of the line in 3D space.
/// @param end_point The end of the line in 3D space.
/// @return The number of voxels traversed. This includes @p endPoint.
template <typename KEY, typename KEYFUNCS>
size_t walkSegmentKeys(WalkSegmentFunc walk_func, const glm::dvec3 &start_point, const glm::dvec3 &end_point)
{
  return walkSegmentKeys(walk_func, start_point, end_point, true, KEYFUNCS());
}
}  // namespace ohm

#endif  // OHMUTIL_LINEWALK_H
