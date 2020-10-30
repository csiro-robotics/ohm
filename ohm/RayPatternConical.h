// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef RAYPATTERNCONICAL_H
#define RAYPATTERNCONICAL_H

#include "OhmConfig.h"

#include "RayPattern.h"

namespace ohm
{
  /// A conical @c RayPattern construction
  class RayPatternConical : public RayPattern
  {
  public:
    /// Constructor.
    /// @param cone_axis The cone's primary axis/directory - expected to be a unit vector.
    /// @param cone_angle The angle between the cone's axis and the cone wall (radians).
    /// @param range Length of the rays in the cone. This makes the cone bottom spherical.
    /// @param angular_resolution Appoximate angular separation between rays (radians).
    /// @param min_range Constrols the near point of each line segment. Zero starts all rays at the apex, while
    /// increasing this value moves the starting points out. Must be less than @p range .
    RayPatternConical(const glm::dvec3 &cone_axis, double cone_angle, double range, double angular_resolution,
                      double min_range = 0);
  };
}  // namespace ohm

#endif  // RAYPATTERNCONICAL_H
