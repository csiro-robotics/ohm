// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHM_MAPLAYOUTDETAIL_H
#define OHM_MAPLAYOUTDETAIL_H

#include "OhmConfig.h"

#include "MapLayer.h"

#include <vector>

namespace ohm
{
  struct MapLayoutDetail
  {
    std::vector<MapLayer *> layers;
    int occupancy_layer = -1;
    int sub_voxel_layer = -1;
    int clearance_layer = -1;

    inline ~MapLayoutDetail() { clear(); }

    inline void clear()
    {
      for (MapLayer *layer : layers)
      {
        delete layer;
      }
      layers.clear();
      occupancy_layer = sub_voxel_layer = clearance_layer = -1;
    }
  };
}

#endif // OHM_MAPLAYOUTDETAIL_H
