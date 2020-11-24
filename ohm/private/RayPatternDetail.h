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
  /// Ray start/end point pairs in sensor space.
  std::vector<glm::dvec3> sample_pairs;
};
}  // namespace ohm

#endif  // RAYPATTERNDETAIL_H
