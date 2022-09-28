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
/// @return True to keep walking the voxels along the ray, false to abort walking the ray.
using WalkVisitFunction = std::function<bool(const Key &, double, double)>;

/// A utility key adaptor around an @c OccupancyMap for use with @c walkSegmentKeys() .
struct ohm_API LineWalkContext
{
  /// Map reference.
  const OccupancyMap &map;
  /// Cached value of @c OccupancyMap::regionVoxelDimensions().
  const glm::ivec3 region_voxel_dimensions;
  /// Function to invoke to visit each voxel.
  const WalkVisitFunction visit;

  /// Create an adaptor for @p map .
  /// @param map The map to adapt.
  /// @param visit The function to call when visiting each voxel.
  inline LineWalkContext(const OccupancyMap &map, WalkVisitFunction visit)
    : map(map)
    , region_voxel_dimensions(map.regionVoxelDimensions())
    , visit(visit)
  {}
};

/// Flags for use with @c walkSegmentKeys() .
enum WalkKeyFlag : unsigned
{
  /// Skip reporting the voxel containing the start point/origin if different from the end point?
  kExcludeStartVoxel = (1u << 0u),
  /// Skip reporting the voxel containing the end point/sample if different from the start point.
  kExcludeEndVoxel = (1u << 1u),
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


inline bool walkVisitVoxel(const LineWalkContext *context, const Key *voxel_key, const Key *start_key,
                           const Key *end_key, unsigned voxel_marker, double enter_time, double exit_time)
{
  (void)start_key;     // Unused.
  (void)end_key;       // Unused.
  (void)voxel_marker;  // Unused.
  return context->visit(*voxel_key, enter_time, exit_time);
}

#include "LineWalkCompute.h"
}  // namespace detail


/// Implements the voxel tracing algorithm with the given @p context .
///
/// This traces the voxels intersected by the line segment from @p start_point to @p end_point invoking
/// @c context.visit for each voxel intersected by the line segment. Each such visit passes the @c Key of the visited
/// voxel and the distanaces from the @p start_point at which the voxel is entered and exited.
///
/// See @c walkLineVoxels() for further implementation details.
///
/// @param context Context within which we are tracing. Provides the @c OccupancyMap and a @c WalkVisitFunction to
///   invoke.
/// @param start_point The start of the line in 3D space.
/// @param end_point The end of the line in 3D space.
/// @param flags Flags from @c WalkKeyFlag. Should be @c true if @p walkFunc should be called for the voxel containing
///   @c endPoint, when it does not lie in the same voxel as @p startPoint.
/// @param length_epsilon The segment length below which a ray is considered degenerate and will only report the start
///   voxel.
/// @return The number of voxels traversed. This includes @p end_point when @p include_end_point is true.
inline unsigned walkSegmentKeys(const LineWalkContext &context, const glm::dvec3 &start_point,
                                const glm::dvec3 &end_point, unsigned flags = 0u,
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

  return detail::walkLineVoxels(&context, start_point, end_point, &start_point_key, &end_point_key, start_voxel_centre,
                                voxel_resolution, flags, length_epsilon);
}
}  // namespace ohm

#endif  // OHM_LINEWALK_H
