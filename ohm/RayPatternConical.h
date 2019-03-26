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
  class RayPatternConical : public RayPattern
  {
  public:
    RayPatternConical(const glm::dvec3 &cone_axis, double cone_angle, double range, double angular_resolution);
  };
}

#endif // RAYPATTERNCONICAL_H
