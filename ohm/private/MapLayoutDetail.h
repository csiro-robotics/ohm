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
    /// Possible values for tracking whether sub-voxel patterns are in use.
    enum SubVoxelState : uint8_t
    {
      /// Unknown: Layer needs to be inspected.
      kSubUnknown,
      /// No sub-voxel patterns.
      kSubOff,
      /// Sub-voxel patterns.
      kSubOn
    };

    std::vector<MapLayer *> layers;
    int occupancy_layer = -1;
    int sub_voxel_layer = -1;
    int clearance_layer = -1;
    SubVoxelState using_sub_voxel_patterns = kSubUnknown;

    inline ~MapLayoutDetail() { clear(); }

    inline void clear()
    {
      for (MapLayer *layer : layers)
      {
        delete layer;
      }
      layers.clear();
      occupancy_layer = sub_voxel_layer = clearance_layer = -1;
      using_sub_voxel_patterns = kSubUnknown;
    }
  };
}

#endif // OHM_MAPLAYOUTDETAIL_H
