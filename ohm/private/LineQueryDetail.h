// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_LINEQUERYDETAIL_H
#define OHM_LINEQUERYDETAIL_H

#include "OhmConfig.h"

#include "QueryDetail.h"

#include "KeyList.h"

namespace ohm
{
  class ClearanceProcess;

  struct LineQueryDetail : QueryDetail
  {
    glm::dvec3 start_point = glm::dvec3(0);
    glm::dvec3 end_point = glm::dvec3(0);
    // Internal: calculated on execute.
    glm::dvec3 segment_dir = glm::dvec3(0);
    glm::dvec3 axis_scaling = glm::dvec3(1, 1, 1);
    ClearanceProcess *clearance_calculator = nullptr;
    KeyList segment_keys;
    // Internal: calculated on execute.
    double segment_length = 0;
    /// Range reported for unobstructed voxels.
    float default_range = -1;
    float search_radius = 0;
  };
}  // namespace ohm

#endif  // OHM_LINEQUERYDETAIL_H
