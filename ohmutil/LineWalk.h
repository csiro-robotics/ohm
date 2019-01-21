// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMUTIL_LINEWALK_H
#define OHMUTIL_LINEWALK_H

#include <glm/glm.hpp>

#include <cassert>

namespace ohm
{
  /// A templatised, voxel based line walking algorithm. Voxels are accurately traversed from @p startPoint to
  /// @p endPoint, invoking @p walkFunc for each traversed voxel.
  ///
  /// The @p walkFunc is simply a callable object which accepts a @p KEY argument. Keys are provided in order of
  /// traversal.
  ///
  /// The templatisation requires @p funcs to provide a set of key manipulation utily functions. Specifically,
  /// the @p KEYFUNCS type must have the following signature:
  /// @code
  /// struct KeyFuncs
  /// {
  ///   // Query the voxel resolution along a particular axis. Axis may be { 0, 1, 2 } corresponding to XYZ.
  ///   double voxelResolutin(int axis) const;
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
  /// @param walk_func The callable object to invoke for each traversed voxel key.
  /// @param start_point The start of the line in 3D space.
  /// @param end_point The end of the line in 3D space.
  /// @param include_end_point Should be @c true if @p walkFunc should be called for the voxel containing
  ///     @c endPoint, when it does not lie in the same voxel as @p startPoint.
  /// @param funcs Key helper functions object.
  /// @return The number of voxels traversed. This includes @p endPoint when @p includeEndPoint is true.
  template <typename KEY, typename KEYFUNCS, typename WALKFUNC>
  size_t walkSegmentKeys(const WALKFUNC &walk_func, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                         bool include_end_point, const KEYFUNCS &funcs)
  {
    // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    KEY start_point_key = funcs.voxelKey(start_point);
    KEY end_point_key = funcs.voxelKey(end_point);

    glm::dvec3 direction = glm::dvec3(end_point - start_point);
    double length = glm::dot(direction, direction);
    length = (length >= 1e-6) ? std::sqrt(length) : 0;
    direction *= 1.0 / length;

    if (funcs.isNull(start_point_key) || funcs.isNull(end_point_key))
    {
      return 0;
    }

    if (start_point_key == end_point_key)
    {
      if (include_end_point)
      {
        walk_func(start_point_key);
      }
      return 1;
    }

    int step[3] = { 0 };
    glm::dvec3 voxel;
    double time_max[3];
    double time_delta[3];
    double time_limit[3];
    double next_voxel_border;
    double direction_axis_inv;
    size_t added = 0;
    KEY current_key = start_point_key;

    //  debugOut = true;
    voxel = funcs.voxelCentre(current_key);
    //  debugOut = false;

    // printf("Start point : %f %f %f\n", startPoint.x, startPoint.y, startPoint.z);
    // printf("End point : %f %f %f\n", endPoint.x, endPoint.y, endPoint.z);
    // printf("V: %f %f %f\n", voxel.x, voxel.y, voxel.z);
    // std::cout << currentKey << std::endl;

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
        next_voxel_border = voxel[i] + step[i] * 0.5 * funcs.voxelResolution(i);
        time_max[i] = (next_voxel_border - start_point[i]) * direction_axis_inv;
        time_limit[i] =
          std::abs((end_point[i] - start_point[i]) * direction_axis_inv);  // +0.5f * funcs.voxelResolution(i);
      }
      else
      {
        time_max[i] = time_delta[i] = std::numeric_limits<double>::max();
        time_limit[i] = 0;
      }
    }

    int axis = 0;
    bool limit_reached = false;
    while (!limit_reached && current_key != end_point_key)
    {
      walk_func(current_key);
      ++added;
      axis =
        (time_max[0] < time_max[2]) ? ((time_max[0] < time_max[1]) ? 0 : 1) : ((time_max[1] < time_max[2]) ? 1 : 2);
      limit_reached = std::abs(time_max[axis]) > time_limit[axis];
      funcs.stepKey(current_key, axis, step[axis]);
      time_max[axis] += time_delta[axis];
    }

    if (include_end_point)
    {
      walk_func(end_point_key);
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
  template <typename KEY, typename KEYFUNCS, typename WALKFUNC>
  size_t walkSegmentKeys(const WALKFUNC &walk_func, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
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
  template <typename KEY, typename KEYFUNCS, typename WALKFUNC>
  size_t walkSegmentKeys(const WALKFUNC &walk_func, const glm::dvec3 &start_point, const glm::dvec3 &end_point)
  {
    return walkSegmentKeys(walk_func, start_point, end_point, true, KEYFUNCS());
  }
}  // namespace ohm

#endif  // OHMUTIL_LINEWALK_H
