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

/// Function signature for the visit function called from @c walkSegmentKeys().
/// @param key The key of the current voxel being visited.
/// @param enter_range Range at which the voxel is entered. detail required
/// @param exit_range Range at which the voxel is entered. detail required
/// @return True to keep walking the voxels along the ray, false to abort walking the ray.
using WalkSegmentFunc = std::function<bool(const Key &, double, double)>;

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
  // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
  KEY start_point_key = funcs.voxelKey(start_point);
  KEY end_point_key = funcs.voxelKey(end_point);

  if (funcs.isNull(start_point_key) || funcs.isNull(end_point_key))
  {
    return 0;
  }

  glm::dvec3 direction = glm::dvec3(end_point - start_point);
  const double length = std::sqrt(glm::dot(direction, direction));
  const glm::dvec3 voxel = funcs.voxelCentre(start_point_key);

  // Very small segments which straddle a voxel boundary can be problematic. We want to avoid
  // inverting a very small number, but be robust enough to handle the situation.
  // To that end, we skip normalising the direction for directions below a tenth of the voxel.
  // Then we will exit either with start/end voxels being the same, or we will step from start
  // to end in one go.
  const bool valid_length = length >= length_epsilon;
  if (valid_length)
  {
    direction *= 1.0 / length;
  }

  if (start_point_key == end_point_key)
  {
    if (include_end_point)
    {
      walk_func(end_point_key, 0.0, length);
    }
    return 1;
  }

  if (!valid_length)
  {
    // Start/end points are in different, but adjacent voxels. Prevent issues with the loop by
    // early out.
    const int axis =
      start_point_key.axisMatches(0, end_point_key) ? (start_point_key.axisMatches(1, end_point_key) ? 2 : 1) : 0;
    const double sign_direction = (direction[axis] > 0) ? 1 : -1;
    const glm::dvec3 voxel = funcs.voxelCentre(start_point_key);
    const double next_voxel_border =
      voxel[axis] + sign_direction * 0.5 * funcs.voxelResolution(axis);  // NOLINT(readability-magic-numbers)
    const double first_voxel_length =
      std::abs((next_voxel_border - start_point[axis]) / (end_point[axis] - start_point[axis])) * length;
    if (walk_func(start_point_key, 0.0, first_voxel_length))
    {
      if (include_end_point)
      {
        walk_func(end_point_key, first_voxel_length, length);
        return 2;
      }
    }
    return 1;
  }

  std::array<int, 3> step = { 0, 0, 0 };
  std::array<double, 3> time_max;
  std::array<double, 3> time_delta;
  std::array<double, 3> time_limit;
  double next_voxel_border;
  double direction_axis_inv;
  size_t added = 0;
  KEY current_key = start_point_key;
  double time_current = 0.0;

  // Compute step direction, increments and maximums along each axis.
  for (unsigned i = 0; i < 3; ++i)
  {
    if (direction[i] != 0)
    {
      direction_axis_inv = 1.0 / direction[i];
      step[i] = (direction[i] > 0) ? 1 : -1;
      // Time delta is the ray time between voxel boundaries calculated for each axis.
      time_delta[i] = funcs.voxelResolution(i) * std::abs(direction_axis_inv);
      // Calculate the distance from the origin to the nearest voxel edge for this axis.
      next_voxel_border = voxel[i] + step[i] * 0.5 * funcs.voxelResolution(i);  // NOLINT(readability-magic-numbers)
      time_max[i] = (next_voxel_border - start_point[i]) * direction_axis_inv;
      // Set the distance limit
      // original...
      // time_limit[i] = std::abs((end_point[i] - start_point[i]) * direction_axis_inv);
      // which is equivalent to...
      time_limit[i] = length;
    }
    else
    {
      time_max[i] = time_delta[i] = std::numeric_limits<double>::max();
      time_limit[i] = 0;
    }
  }

  int axis = 0;
  bool limit_reached = false;
  bool user_exit = false;
  while (!limit_reached && !user_exit && current_key != end_point_key)
  {
    axis = (time_max[0] < time_max[2]) ? ((time_max[0] < time_max[1]) ? 0 : 1) : ((time_max[1] < time_max[2]) ? 1 : 2);
    // Strictly speaking std::abs() is unnecessary here. However, from memory there were instances where it could be
    // negative in practice (floating point error). Possibly in the zero case (positive and negative zeros).
    limit_reached = std::abs(time_max[axis]) > time_limit[axis];
    const double new_time_current = limit_reached ? time_limit[axis] : time_max[axis];
    user_exit = !walk_func(current_key, time_current, new_time_current);
    time_max[axis] += time_delta[axis];
    time_current = new_time_current;
    ++added;
    funcs.stepKey(current_key, axis, step[axis]);
  }

  if (!user_exit && include_end_point)
  {
    walk_func(end_point_key, time_current, length);
    ++added;
  }

  assert(added);
  return added;
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
