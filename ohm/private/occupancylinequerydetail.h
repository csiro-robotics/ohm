// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYLINEQUERYDETAIL_H_
#define OCCUPANCYLINEQUERYDETAIL_H_

#include "ohmconfig.h"

#include "occupancyquerydetail.h"

#include "occupancykeylist.h"

namespace ohm
{
  class ClearanceProcess;

  struct LineQueryDetail : QueryDetail
  {
    glm::dvec3 startPoint;
    glm::dvec3 endPoint;
    // Internal: calculated on execute.
    glm::dvec3 segmentDir;
    glm::dvec3 axisScaling = glm::dvec3(1, 1, 1);
    ClearanceProcess *clearanceCalculator = nullptr;
    OccupancyKeyList segmentKeys;
    // Internal: calculated on execute.
    double segmentLength = 0;
    /// Range reported for unobstructed voxels.
    float defaultRange = -1;
    float searchRadius = 0;
  };
}

#endif // OCCUPANCYLINEQUERYDETAIL_H_
