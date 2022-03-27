// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas

#include "WalkSegmentKeysLegacy.h"

#include <ohm/Key.h>
#include <ohm/OccupancyMap.h>

namespace ohm
{
size_t walkSegmentKeysLegacy(WalkSegmentFunc walk_func, const glm::dvec3 &start_point, const glm::dvec3 &end_point,
                             bool include_end_point, const ohm::OccupancyMap &map,
                             double length_epsilon)  // NOLINT(readability-magic-numbers)
{
  // see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
  Key start_point_key = map.voxelKey(start_point);
  Key end_point_key = map.voxelKey(end_point);

  if (start_point_key.isNull() || end_point_key.isNull())
  {
    return 0;
  }

  glm::dvec3 direction = glm::dvec3(end_point - start_point);
  const double length = std::sqrt(glm::dot(direction, direction));
  const glm::dvec3 voxel = map.voxelCentreGlobal(start_point_key);

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
    const glm::dvec3 voxel = map.voxelCentreGlobal(start_point_key);
    const double next_voxel_border =
      voxel[axis] + sign_direction * 0.5 * map.resolution();  // NOLINT(readability-magic-numbers)
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
  // Correction offset for time_limit for when we are tracing from the exact bottom corner of a voxel to another exact
  // corner in the positive direction.
  glm::dvec3 time_limit_offset = glm::dvec3(0.0);
  double next_voxel_border;
  double direction_axis_inv;
  size_t added = 0;
  Key current_key = start_point_key;
  double time_current = 0.0;

  // Compute step direction, increments and maximums along each axis.
  for (unsigned i = 0; i < 3; ++i)
  {
    if (direction[i] != 0)
    {
      direction_axis_inv = 1.0 / direction[i];
      step[i] = (direction[i] > 0) ? 1 : -1;
      // Time delta is the ray time between voxel boundaries calculated for each axis.
      time_delta[i] = map.resolution() * std::abs(direction_axis_inv);
      // Calculate the distance from the origin to the nearest voxel edge for this axis.
      next_voxel_border = voxel[i] + step[i] * 0.5 * map.resolution();  // NOLINT(readability-magic-numbers)
      time_max[i] = (next_voxel_border - start_point[i]) * direction_axis_inv;
      // Set the distance limit
      // original...
      // time_limit[i] = std::abs((end_point[i] - start_point[i]) * direction_axis_inv);
      // which is equivalent to...
      time_limit[i] = length;
      // Add an effective epsilon to handle traversing exact voxel boundaries.
      // time_limit[i] += 0.5 * time_delta[i];
      if (std::abs(time_max[i] - glm::length(glm::dvec3(map.resolution()))) < 1e-6)
      {
        time_limit_offset[i] = step[i] * 0.5 * map.resolution();
      }
    }
    else
    {
      time_max[i] = time_delta[i] = std::numeric_limits<double>::max();
      time_limit[i] = 0;
    }
  }

  for (int i = 0; i < 3; ++i)
  {
    time_limit[i] += time_limit_offset[i];
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
    map.stepKey(current_key, axis, step[axis]);
  }

  if (!user_exit && include_end_point)
  {
    walk_func(end_point_key, time_current, length);
    ++added;
  }

  assert(added);
  return added;
}
}  // namespace ohm
