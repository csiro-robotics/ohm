// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_LINEWALK_H
#define OHM_LINEWALK_H

#include "Key.h"
#include "OccupancyMap.h"

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
/// @param steps_remaining The number of voxels steps remaining along each axis.
/// @return True to keep walking the voxels along the ray, false to abort walking the ray.
using WalkVisitFunction = std::function<bool(const Key &, double, double, const glm::ivec3 &)>;

/// A utility key adaptor around an @c OccupancyMap for use with @c walkSegmentKeys() .
struct ohm_API LineWalkContext
{
  /// Map reference.
  const OccupancyMap &map;
  const glm::ivec3 region_voxel_dimensions;
  const WalkVisitFunction visit;

  /// Create an adaptor for @p map .
  /// @param map The map to adapt
  inline LineWalkContext(const OccupancyMap &map, WalkVisitFunction visit)
    : map(map)
    , region_voxel_dimensions(map.regionVoxelDimensions())
    , visit(visit)
  {}
};


namespace detail
{
using WalkContext = const LineWalkContext;
inline void walkKeyDiff(const LineWalkContext *context, int diff[3], const Key *key_a, const Key *key_b)
{
  (void)context;  // Unused
  const glm::ivec3 diff_iv3 = OccupancyMap::rangeBetween(*key_b, *key_a, context->region_voxel_dimensions);
  diff[0] = diff_iv3.x;
  diff[1] = diff_iv3.y;
  diff[2] = diff_iv3.z;
}


/// Adjust the value of @p key by stepping it along @p axis
/// @param key The key to modify.
/// @param axis The axis to modifier where 0, 1, 2 map to X, Y, Z respectively.
/// @param dir The direction to step: must be 1 or -1
inline void walkStepKey(const LineWalkContext *context, Key *key, int axis, int step_dir)
{
  OccupancyMap::stepKey(*key, axis, step_dir, context->region_voxel_dimensions);
}


inline bool walkVisitVoxel(const LineWalkContext *context, const Key *voxel_key, double enter_time, double exit_time,
                           const int steps_remaining[3])
{
  return context->visit(*voxel_key, enter_time, exit_time,
                        glm::ivec3(steps_remaining[0], steps_remaining[1], steps_remaining[2]));
}

#include "LineWalkCompute.h"
}  // namespace detail


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
inline unsigned walkSegmentKeys(const LineWalkContext &context, const glm::dvec3 &start_point,
                                const glm::dvec3 &end_point, bool include_end_point = true,
                                double length_epsilon = 1e-6)  // NOLINT(readability-magic-numbers)
{
  const Key start_point_key = context.map.voxelKey(start_point);
  const Key end_point_key = context.map.voxelKey(end_point);

  if (start_point_key.isNull() || end_point_key.isNull())
  {
    return 0;
  }

  const glm::dvec3 start_voxel_centre = context.map.voxelCentreGlobal(start_point_key);
  const glm::dvec3 voxel_resolution(context.map.resolution());

  return detail::walkSegmentKeys(&context, start_point, end_point, &start_point_key, &end_point_key, start_voxel_centre,
                                 voxel_resolution, include_end_point, length_epsilon);
}
}  // namespace ohm

#endif  // OHM_LINEWALK_H
