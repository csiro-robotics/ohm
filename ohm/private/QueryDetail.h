// Copyright (c) 2017
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_QUERYDETAIL_H
#define OHM_QUERYDETAIL_H

#include "OhmConfig.h"

#include "Key.h"

#include <limits>
#include <vector>

namespace ohm
{
  class OccupancyMap;

  struct QueryDetail
  {
    OccupancyMap *map = nullptr;
    std::vector<Key> intersected_voxels;
    std::vector<float> ranges;
    size_t number_of_results = 0;
    unsigned query_flags = 0;
  };

  struct ClosestResult
  {
    size_t index = 0;
    float range = std::numeric_limits<float>::max();
  };
}  // namespace ohm

#endif  // OHM_QUERYDETAIL_H
