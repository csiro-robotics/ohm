// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_NEARESTNEIGHBOURSDETAIL_H
#define OHM_NEARESTNEIGHBOURSDETAIL_H

#include "OhmConfig.h"

#include "QueryDetail.h"

namespace ohm
{
  struct ohm_API NearestNeighboursDetail : QueryDetail
  {
    glm::dvec3 near_point = glm::dvec3(0);
    float search_radius = 0;
  };
}  // namespace ohm

#endif  // OHM_NEARESTNEIGHBOURSDETAIL_H
