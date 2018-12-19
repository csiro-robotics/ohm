// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMDEFAULTLAYER_H
#define OHMDEFAULTLAYER_H

#include "OhmConfig.h"

namespace ohm
{
  namespace default_layer
  {
    /// Name of the occupancy layer.
    /// @return "occupancy"
    const char * ohm_API occupancyLayerName();
    /// Name of the voxel clearance layer.
    /// @return "clearance"
    const char * ohm_API clearanceLayerName();
  }
}

#endif // OHMDEFAULTLAYER_H
