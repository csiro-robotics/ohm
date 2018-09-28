// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OCCUPANCYQUERYDETAIL_H_
#define OCCUPANCYQUERYDETAIL_H_

#include "ohmconfig.h"

#include "occupancykey.h"

#include <limits>
#include <vector>

namespace ohm
{
  class OccupancyMap;

  struct QueryDetail
  {
    OccupancyMap *map = nullptr;
    std::vector<OccupancyKey> intersectedVoxels;
    std::vector<float> ranges;
    size_t numberOfResults = 0;
    unsigned queryFlags = 0;
  };

  struct ClosestResult
  {
    size_t index;
    float range;

    inline ClosestResult()
      : index(0), range(std::numeric_limits<float>::max())
    {}
  };
}

#endif // OCCUPANCYQUERYDETAIL_H_
