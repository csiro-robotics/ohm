// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef CLEARINGPATTERNDETAIL_H
#define CLEARINGPATTERNDETAIL_H

#include "OhmConfig.h"

#include <glm/glm.hpp>

#include <vector>

namespace ohm
{
class RayPattern;

struct ClearingPatternDetail
{
  std::vector<glm::dvec3> ray_set;
  const RayPattern *pattern = nullptr;
  unsigned ray_flags = 0u;  // Default value should be ClearingPattern::kDefaultFlags.
  bool has_pattern_ownership = false;
};
}  // namespace ohm

#endif  // CLEARINGPATTERNDETAIL_H
