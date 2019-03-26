// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef RAYPATTERNDETAIL_H
#define RAYPATTERNDETAIL_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

#include <vector>

namespace ohm
{
  struct RayPatternDetail
  {
    std::vector<glm::dvec3> points;
  };
}

#endif // RAYPATTERNDETAIL_H
