// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPLAYOUTDETAIL_H
#define OHM_MAPLAYOUTDETAIL_H

#include "OhmConfig.h"

#include "ohm/MapLayer.h"

#include <vector>

namespace ohm
{
  struct ohm_API MapLayoutDetail
  {
    std::vector<MapLayer *> layers;
    int occupancy_layer = -1;
    int mean_layer = -1;
    int covariance_layer = -1;
    int clearance_layer = -1;
    int intensity_layer = -1;
    int hit_miss_count_layer = -1;

    inline ~MapLayoutDetail() { clear(); }

    inline void clear()
    {
      for (MapLayer *layer : layers)
      {
        delete layer;
      }
      layers.clear();
      occupancy_layer = mean_layer = covariance_layer = clearance_layer = intensity_layer = hit_miss_count_layer = -1;
    }
  };
}  // namespace ohm

#endif  // OHM_MAPLAYOUTDETAIL_H
