// Copyright (c) 2018
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Kazys Stepanas
#ifndef OHMDEFAULTLAYERS_H_
#define OHMDEFAULTLAYERS_H_

#include "ohmconfig.h"

namespace ohm
{
  /// Default @c MapLayer identification.
  enum DefaultLayers
  {
    /// Voxel occupancy values.
    DL_Occupancy,
    /// Voxel clearance values: distance to nearest obstruction.
    DL_Clearance,
    /// Downsampled @c DL_Clearance
    DL_CoarseClearance,

    /// Number of default layers.
    DL_Count
  };
}

#endif // OHMDEFAULTLAYERS_H_
